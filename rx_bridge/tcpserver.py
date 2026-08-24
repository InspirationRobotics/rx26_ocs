"""tcpserver — the vehicle link: one TCP connection per vehicle, both directions.

Replaces a broker with a socket. The vehicles dial IN to this server, so the OCS
never needs their addresses -- and since each vehicle holds its connection open,
the same socket carries commands back out without a second channel.

IDENTITY COMES FROM THE MESSAGE, NOT THE CONNECTION. A freshly accepted socket is
anonymous: we do not know, and must not guess, which craft is on the far end. The
first frame carries `vehicle_id`, and only then is the socket filed under that
name so `send()` can find it. This is why `on_frame` returns the id rather than
the server parsing payloads it has no business understanding -- the protobuf
lives in bridge.py, the sockets live here, and neither imports the other's job.

THREE FAILURES THIS HANDLES THAT THE OBVIOUS VERSION DOES NOT:

  * `recv()` returning b'' means the peer closed cleanly. Read as "no data
    right now" -- which is what `if data:` does -- the reader spins forever on a
    dead socket while the OCS reports the vehicle as connected. A link that is
    down while the screen says up is the exact failure an OCS exists to prevent.
  * A desynced stream cannot be resynchronised. Once a length prefix is wrong
    there is no way to find where the next frame starts, and guessing produces
    frames that parse as valid protobuf while meaning something else. Drop the
    connection and let the vehicle reconnect into a clean stream.
  * `send()` is not `sendall()`. The short-write case is rare and silent, and it
    truncates a command rather than failing it.
"""
from __future__ import annotations

import socket
import socketserver
import threading
from typing import Callable

from .framing import FrameReader, encode

#: How long a reader blocks before checking whether the server is shutting down.
#: Long enough not to spin, short enough that stop() returns promptly.
_POLL_S = 1.0


class _Server(socketserver.ThreadingTCPServer):
    # Both must be class attributes: server_bind() reads allow_reuse_address
    # before __init__ returns, so setting it on the instance is too late and the
    # OCS fails to restart for a minute after a crash with "address in use".
    allow_reuse_address = True
    daemon_threads = True


class TcpServer:
    """Accepts vehicle connections; reads frames; sends frames back.

    on_frame(payload) -> vehicle_id | None
        Called for every complete frame. Returning an id files this connection
        under that name; returning None leaves it anonymous, which is what an
        undecodable frame should do.
    """

    def __init__(self, host: str, port: int, *,
                 on_frame: Callable[[bytes], str | None],
                 log=print) -> None:
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.log = log

        self._srv: _Server | None = None
        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()
        self._lock = threading.Lock()
        self._peers: dict[str, tuple[socket.socket, str]] = {}

    # ---- lifecycle --------------------------------------------------------

    def start(self) -> None:
        # Cleared, not merely unset: stop() sets this, and a server that
        # is stopped and started again would otherwise accept every
        # connection and drop it on the first pass round the read loop --
        # which looks exactly like a vehicle that cannot stay connected.
        self._stopping.clear()
        owner = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self) -> None:                      # one per connection
                owner._serve(self.request, self.client_address)

        self._srv = _Server((self.host, self.port), Handler)
        self._thread = threading.Thread(
            target=self._srv.serve_forever, name="tcp-accept", daemon=True
        )
        self._thread.start()
        self.log("vehicle link: listening on %s:%d" % (self.host, self.port))

    def stop(self) -> None:
        self._stopping.set()
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()
        with self._lock:
            peers = list(self._peers.values())
            self._peers.clear()
        for sock, _ in peers:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    # ---- one connection ---------------------------------------------------

    def _serve(self, sock: socket.socket, addr) -> None:
        who = "%s:%d" % addr
        self.log("vehicle link: %s connected" % who)
        reader = FrameReader()
        vid: str | None = None
        sock.settimeout(_POLL_S)
        try:
            while not self._stopping.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break          # clean close -- NOT "no data yet"
                try:
                    frames = reader.feed(chunk)
                except ValueError as exc:
                    self.log("vehicle link %s: %s -- dropping" % (who, exc))
                    break
                for payload in frames:
                    got = self.on_frame(payload)
                    if got and got != vid:
                        vid = got
                        self._register(vid, sock, who)
        finally:
            self._unregister(vid, sock)
            try:
                sock.close()
            except OSError:
                pass
            self.log("vehicle link: %s (%s) disconnected"
                     % (who, vid or "unidentified"))

    # ---- the registry -----------------------------------------------------

    def _register(self, vid: str, sock: socket.socket, who: str) -> None:
        with self._lock:
            old = self._peers.get(vid)
            self._peers[vid] = (sock, who)
        if old is not None and old[0] is not sock:
            # Same vehicle on a new socket: it reconnected before we noticed the
            # old one had gone. Close the stale half rather than leaking it.
            self.log("vehicle link: %s reconnected, closing stale %s"
                     % (vid, old[1]))
            try:
                old[0].close()
            except OSError:
                pass
        self.log("vehicle link: %s identified as %s" % (who, vid))

    def _unregister(self, vid: str | None, sock: socket.socket) -> None:
        if vid is None:
            return
        with self._lock:
            cur = self._peers.get(vid)
            # Only if it is still OUR socket. A reconnect may already have filed
            # a new one under this id, and tearing that down because an old
            # reader thread finally noticed its own death would drop a live link.
            if cur is not None and cur[0] is sock:
                del self._peers[vid]

    # ---- outbound ---------------------------------------------------------

    def send(self, vehicle_id: str, payload: bytes) -> bool:
        """Frame and send to one vehicle. False if it is not connected."""
        with self._lock:
            entry = self._peers.get(vehicle_id)
        if entry is None:
            return False
        sock, who = entry
        try:
            sock.sendall(encode(payload))    # sendall: a short write truncates
            return True
        except OSError as exc:
            self.log("vehicle link: send to %s (%s) failed: %s"
                     % (vehicle_id, who, exc))
            return False

    def peers(self) -> dict[str, str]:
        """vehicle_id -> "ip:port", for the operator page."""
        with self._lock:
            return {vid: who for vid, (_, who) in self._peers.items()}
