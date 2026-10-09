"""The real transports fleet_link runs on: a UDP socket for WiFi, a serial port for
the RFD900ux, and one loopback socket per output. All threads live here; all
routing lives in hub.py, which these call into.

OWNERSHIP, the rule that cost an afternoon on 20 Sep (see radio_link.py): a serial
port has exactly one owner. SerialLinkIO's single thread does every read AND every
write; everything else only puts bytes on its queue. A write that fails is counted
and logged and the link carries on -- it never takes the uplink down with it.

WINDOWS TRAPS HANDLED HERE
  * A UDP socket that sent to a port nobody is listening on gets an ICMP
    "unreachable" back, and Windows then makes the NEXT recvfrom on that socket
    raise ConnectionResetError. Every receive loop here swallows it and carries on;
    without that, the first consumer to exit would kill the loop that feeds the
    others.
  * SO_REUSEADDR on Windows lets a second socket bind a port that is already in use
    and STEAL its datagrams, with no error on either side. We never set it. We set
    SO_EXCLUSIVEADDRUSE instead, so another program that does set it (Qt's UDP
    sockets, i.e. QGroundControl, default to sharing) fails to bind rather than
    silently splitting the traffic with us.
"""
from __future__ import annotations

import errno
import logging
import queue
import socket
import threading
from typing import Callable, Optional

from .hub import LinkIO, OutputIO

log = logging.getLogger("fleet")

LOOPBACK = "127.0.0.1"
_RECV_TIMEOUT_S = 0.5            # how often a blocked receive checks the stop flag
_WSAEADDRINUSE = 10048
_WSAEACCES = 10013


class EndpointError(RuntimeError):
    """A transport could not be claimed; the message is written for the operator."""


class _Guard:
    """Run a callback so an exception in routing code never kills a receive thread,
    and never floods the console either: log the first few, then every 1000th."""

    def __init__(self, what: str):
        self.what = what
        self.count = 0

    def __call__(self, fn, *args) -> None:
        try:
            fn(*args)
        except Exception:                                            # noqa: BLE001
            self.count += 1
            if self.count <= 3 or self.count % 1000 == 0:
                log.exception("%s raised (%d so far)", self.what, self.count)


def _describe(exc: BaseException) -> str:
    text = str(exc) or type(exc).__name__
    if "PermissionError" in text or "Access is denied" in text:
        text += "  [port busy: another program has it open?]"
    return text


def claim_exclusively(sock: socket.socket) -> None:
    """Windows: refuse to share this socket's port with anyone, even a program that
    asks to share. Deliberately NOT SO_REUSEADDR -- see the module docstring."""
    if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)


def _bind_udp(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    claim_exclusively(sock)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
    try:
        sock.bind((host, port))
    except OSError:
        sock.close()
        raise
    sock.settimeout(_RECV_TIMEOUT_S)
    return sock


def _recv_loop(sock: socket.socket, stop: threading.Event, handle: Callable, what: str,
               on_socket_error: Callable[[OSError], None]) -> None:
    """Receive datagrams until `stop`, passing each to handle(data, addr)."""
    guarded = _Guard(what)
    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except ConnectionResetError:
            continue                     # Windows: ICMP unreachable from an earlier send
        except OSError as e:
            if not stop.is_set():
                on_socket_error(e)
            return
        guarded(handle, data, addr)


class _Threaded:
    """A transport with one worker thread and a stop flag. Mixed in ahead of
    LinkIO / OutputIO; `_run` is the thread body, `_release` frees the resource
    (and must unblock `_run`)."""

    def __init__(self, name: str, *args, **kwargs):
        super().__init__(name, *args, **kwargs)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def _run(self) -> None:
        raise NotImplementedError

    def _release(self) -> None:
        pass

    def start(self) -> None:
        prefix = "link-" if isinstance(self, LinkIO) else "out-"
        self._thread = threading.Thread(target=self._run, name=prefix + self.name, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._release()
        if self._thread is not None:
            self._thread.join(3.0)


def _attempt_write(hub, name: str, nframes: int, nbytes: int, write: Callable[[], object]) -> None:
    """Run `write()` and tell the hub whether it worked. A failure is reported, never raised."""
    try:
        write()
    except Exception as e:                                           # noqa: BLE001
        hub.link_tx(name, nframes, nbytes, _describe(e))
    else:
        hub.link_tx(name, nframes, nbytes)


class UdpLinkIO(_Threaded, LinkIO):
    """The WiFi link: binds host:port and receives; replies go out of the SAME
    socket (so they leave from the port the boat sent to)."""
    kind = "udp"
    needs_addr = True
    stream = False

    def __init__(self, name: str, host: str, port: int, taken_hint: str = ""):
        super().__init__(name)
        self.host = host
        self.port = port
        self.taken_hint = taken_hint
        self._sock: Optional[socket.socket] = None

    def open(self) -> None:
        try:
            self._sock = _bind_udp(self.host, self.port)
        except OSError as e:
            code = getattr(e, "winerror", None) or e.errno
            if code in (_WSAEADDRINUSE, _WSAEACCES, errno.EADDRINUSE, errno.EACCES):
                raise EndpointError(
                    "UDP %d is taken - %s (%s)" % (
                        self.port,
                        self.taken_hint or "is QGroundControl running with UDP autoconnect? "
                        "Close it, or move its UDP link to another port",
                        e)) from e
            raise EndpointError("cannot bind UDP %s:%d: %s" % (self.host, self.port, e)) from e
        self.hub.link_state(self.name, True, None)

    def _release(self) -> None:
        if self._sock is not None:
            self._sock.close()

    def _run(self) -> None:
        _recv_loop(self._sock, self._stop,
                   lambda data, addr: self.hub.link_rx(self.name, data, addr),
                   "link %s rx" % self.name,
                   lambda e: self.hub.link_state(self.name, False, "socket error: %s" % e))

    def send(self, data: bytes, nframes: int, addr) -> bool:
        if self._sock is None or addr is None:
            return False
        _attempt_write(self.hub, self.name, nframes, len(data),
                       lambda: self._sock.sendto(data, addr))
        return True


def open_serial(device: str, baud: int):
    """Open a serial port (or a pyserial URL such as loop://, used by the tests)."""
    import serial                                    # pyserial, imported late: only needed here
    return serial.serial_for_url(device, baudrate=baud, timeout=0.02, write_timeout=1.0)


class SerialLinkIO(_Threaded, LinkIO):
    """The RFD900ux. One thread owns the port: it reads, it writes what has been
    queued, and when the port goes away it reopens it every `reopen_s`."""
    kind = "serial"
    needs_addr = False
    stream = True

    def __init__(self, name: str, device: str, baud: int, reopen_s: float = 5.0, *,
                 opener: Callable = open_serial, tx_queue_max: int = 256):
        super().__init__(name)
        self.device = device
        self.baud = baud
        self.reopen_s = reopen_s
        self._opener = opener
        self._tx: "queue.Queue" = queue.Queue(maxsize=tx_queue_max)
        self._is_open = False

    def send(self, data: bytes, nframes: int, addr) -> bool:
        """Queue for the owning thread. Refused while the port is down: a command
        queued now would be written when the port comes back, possibly minutes
        later, and a stale ARM or mode change is worse than a lost one."""
        if not self._is_open:
            return False
        try:
            self._tx.put_nowait((data, nframes))
        except queue.Full:
            return False
        return True

    # ---- everything below runs on the one thread that owns the port

    def _flush_tx(self) -> None:
        while True:
            try:
                self._tx.get_nowait()
            except queue.Empty:
                return

    def _run(self) -> None:
        hub = self.hub
        while not self._stop.is_set():
            try:
                ser = self._opener(self.device, self.baud)
            except Exception as e:                                   # noqa: BLE001
                hub.link_state(self.name, False, _describe(e))       # logs only on change
                if self._stop.wait(self.reopen_s):
                    return
                continue
            self._flush_tx()
            self._is_open = True
            hub.link_state(self.name, True, None)
            error = self._pump(ser)
            self._is_open = False
            try:
                ser.close()
            except Exception:                                        # noqa: BLE001
                pass
            if error is None:
                return
            hub.link_state(self.name, False, error)
            if self._stop.wait(self.reopen_s):
                return

    def _pump(self, ser) -> Optional[str]:
        """Read and write until told to stop (None) or the port fails (its error)."""
        deliver = _Guard("link %s rx" % self.name)
        while not self._stop.is_set():
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting if waiting else 1)
            except Exception as e:                                   # noqa: BLE001
                return _describe(e)
            if data:
                deliver(self.hub.link_rx, self.name, data, None)
            self._write_pending(ser)
        return None

    def _write_pending(self, ser) -> None:
        while True:
            try:
                data, nframes = self._tx.get_nowait()
            except queue.Empty:
                return
            _attempt_write(self.hub, self.name, nframes, len(data), lambda: ser.write(data))


class DisabledLinkIO(LinkIO):
    """A link switched off (--serial NONE, --wifi-port NONE). It still appears in
    /fleet, as disabled, so a consumer never has to guess whether a key is missing."""
    enabled = False

    def __init__(self, name: str, kind: str):
        super().__init__(name)
        self.kind = kind

    def send(self, data: bytes, nframes: int, addr) -> bool:
        return False


class UdpOutputIO(_Threaded, OutputIO):
    """One consumer port. Frames go to host:port from this socket's own ephemeral
    source port; whatever the consumer sends back arrives on this same socket, so
    each consumer's replies are attributable to its output."""

    def __init__(self, name: str, port: int, host: str = LOOPBACK):
        super().__init__(name)
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None

    def open(self) -> None:
        try:
            self._sock = _bind_udp(self.host, 0)
        except OSError as e:
            raise EndpointError("cannot open a socket for output %s: %s" % (self.name, e)) from e
        self.local_port = self._sock.getsockname()[1]

    def _release(self) -> None:
        if self._sock is not None:
            self._sock.close()

    def _run(self) -> None:
        _recv_loop(self._sock, self._stop,
                   lambda data, addr: self.hub.output_rx(self.name, data),
                   "output %s rx" % self.name, lambda e: None)

    def send(self, data: bytes) -> bool:
        try:
            self._sock.sendto(data, (self.host, self.port))
        except OSError:
            return False
        return True
