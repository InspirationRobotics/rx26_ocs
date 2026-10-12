"""tcpserver — the vehicle link: TCP connections from the vehicles, both directions.

Replaces a broker with a socket. The vehicles dial IN to this server, so the OCS
never needs their addresses -- and since each vehicle holds its connection open,
the same socket carries commands back out without a second channel.

IDENTITY COMES FROM THE MESSAGE, NOT THE CONNECTION. A freshly accepted socket is
anonymous: we do not know, and must not guess, which craft is on the far end. The
first frame carries `vehicle_id`, and only then is the socket filed under that
name so `send()` can find it. This is why `on_frame` returns the id rather than
the server parsing payloads it has no business understanding -- the protobuf
lives in bridge.py, the sockets live here, and neither imports the other's job.

ONE SOCKET PER VEHICLE PER LINK. A vehicle may be connected over more than one
transport at once: its own WiFi client, and the laptop-side radio adapter. Each
socket carries a `link` name (set by the bridge from the client's `ocs_hello`;
"wifi" until told otherwise). A new socket claiming the same vehicle on the SAME
link replaces the old one, as it always did -- that is a reconnect. A socket on a
DIFFERENT link coexists with it. Which of them the OCS listens to and talks on is
policy, and lives in linkpolicy.py and bridge.py, not here.

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
    truncates a command rather than failing it. And a send that fails part way
    leaves the stream torn for the peer, so a failed send closes the socket
    instead of carrying on writing frames into the wreckage.
"""
from __future__ import annotations

import socket
import socketserver
import threading
import time
from typing import Callable

from .framing import FrameReader, encode

#: How long a reader blocks before checking whether the server is shutting down.
#: Long enough not to spin, short enough that stop() returns promptly.
_POLL_S = 1.0

#: How often the accept loop looks for a stop() request. serve_forever's default
#: of 0.5 s is also how long stop() takes to return, which a restart-heavy test
#: suite pays on every case.
_ACCEPT_POLL_S = 0.1

#: The link a socket is on until its client says otherwise. Today's boat and
#: Ekko clients send no hello and must keep working unchanged.
DEFAULT_LINK = "wifi"


class Conn:
    """One accepted socket, as much of it as the bridge may look at or set.

    `link`, `agent`, `hello` and `synthesized` are written by the bridge from
    the client's first frame; everything else belongs to this module. A bridge
    that wants the socket dropped sets `close_reason` from inside on_frame.
    """

    def __init__(self, sock: socket.socket, who: str) -> None:
        self.sock = sock
        self.who = who                       #: "ip:port" of the far end
        self.link = DEFAULT_LINK
        self.agent = ""                      #: free text from the hello
        self.hello = False                   #: True once a hello was accepted
        #: Report kinds this link rebuilds from a side channel rather than
        #: relaying from the vehicle (the hello's `synthesized`). Empty = none.
        self.synthesized: frozenset[str] = frozenset()
        self.vids: list[str] = []            #: ids this socket is filed under
        self.connected_at = time.time()
        self.last_rx = time.monotonic()      #: last whole frame of ANY kind
        self.send_lock = threading.Lock()    #: one frame at a time per socket
        self.close_reason: str | None = None

    @property
    def adapter(self) -> bool:
        """True for a peer that said, in a hello, that it carries a vehicle over
        a link that is not the vehicle's own WiFi socket -- the laptop-side radio
        adapter. Such a socket being up says nothing about the vehicle."""
        return self.hello and self.link != DEFAULT_LINK

    @property
    def label(self) -> str:
        """How the operator should read this peer: "radio via fleet_link" for an
        adapter (its ip:port is the laptop's own loopback, which names nothing),
        the address for a vehicle that dialled in itself."""
        if self.adapter:
            return "%s via %s" % (self.link, self.agent or "an adapter")
        return self.who


class _Server(socketserver.ThreadingTCPServer):
    # Both must be class attributes: server_bind() reads allow_reuse_address
    # before __init__ returns, so setting it on the instance is too late and the
    # OCS fails to restart for a minute after a crash with "address in use".
    allow_reuse_address = True
    daemon_threads = True


def _kill(sock: socket.socket) -> None:
    """Shut a socket down so a reader blocked on it wakes now, not in a second."""
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        sock.close()
    except OSError:
        pass


class TcpServer:
    """Accepts vehicle connections; reads frames; sends frames back.

    on_frame(payload, conn) -> vehicle_id | None
        Called for every complete frame. Returning an id files this connection
        under that name (on conn.link); returning None leaves it anonymous, which
        is what an undecodable frame should do. The callback may set conn.link /
        conn.agent / conn.hello BEFORE returning the id -- that is how a hello
        chooses the link -- and may set conn.close_reason to have the socket
        dropped.

    on_link(vehicle_id, link, up, conn)
        Called after a (vehicle, link) pair is filed (up=True) or goes away
        (up=False). Never called with a server lock held.
    """

    def __init__(self, host: str, port: int, *,
                 on_frame: Callable[[bytes, Conn], str | None],
                 on_link: Callable[[str, str, bool, Conn], None] | None = None,
                 log=print) -> None:
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.on_link = on_link
        self.log = log

        self._srv: _Server | None = None
        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()
        self._lock = threading.Lock()
        #: vehicle_id -> link -> the one live Conn
        self._peers: dict[str, dict[str, Conn]] = {}

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
            target=self._srv.serve_forever, kwargs={"poll_interval": _ACCEPT_POLL_S},
            name="tcp-accept", daemon=True
        )
        self._thread.start()
        self.log("vehicle link: listening on %s:%d" % (self.host, self.bound_port))

    def stop(self) -> None:
        self._stopping.set()
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()
        with self._lock:
            conns = {c for by_link in self._peers.values()
                     for c in by_link.values()}
            self._peers.clear()
        for conn in conns:
            _kill(conn.sock)

    @property
    def bound_port(self) -> int:
        """The port actually listening -- differs from `port` when that is 0."""
        if self._srv is not None:
            return self._srv.server_address[1]
        return self.port

    # ---- one connection ---------------------------------------------------

    def _serve(self, sock: socket.socket, addr) -> None:
        who = "%s:%d" % addr[:2]
        conn = Conn(sock, who)
        self.log("vehicle link: %s connected" % who)
        reader = FrameReader()
        sock.settimeout(_POLL_S)
        try:
            while not self._stopping.is_set() and conn.close_reason is None:
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
                    conn.last_rx = time.monotonic()
                    try:
                        got = self.on_frame(payload, conn)
                    except Exception as exc:  # noqa: BLE001
                        # One poison frame must not take a vehicle off the
                        # link. The frame is already in the wire log.
                        self.log("vehicle link %s: frame handler raised %s: %s "
                                 "-- frame skipped"
                                 % (who, type(exc).__name__, exc))
                        continue
                    if conn.close_reason is not None:
                        break
                    if got:
                        self._register(conn, got)
        finally:
            self._unregister(conn)
            _kill(sock)
            self.log("vehicle link: %s (%s) disconnected%s"
                     % (who, ",".join(conn.vids) or "unidentified",
                        " [%s]" % conn.close_reason if conn.close_reason else ""))

    # ---- the registry -----------------------------------------------------

    def _register(self, conn: Conn, vid: str) -> None:
        """File `conn` under (vid, conn.link). Replaces any older socket there."""
        with self._lock:
            by_link = self._peers.setdefault(vid, {})
            old = by_link.get(conn.link)
            if old is conn:
                return                      # already filed; nothing to do
            by_link[conn.link] = conn
            if vid not in conn.vids:
                conn.vids.append(vid)
        if old is not None:
            # Same vehicle, same link, new socket: it reconnected before we
            # noticed the old one had gone. Close the stale half rather than
            # leaking it. A DIFFERENT link is left strictly alone.
            self.log("vehicle link: %s reconnected on %s, closing stale %s"
                     % (vid, conn.link, old.who))
            _kill(old.sock)
        self.log("vehicle link: %s identified as %s on %s%s"
                 % (conn.who, vid, conn.link,
                    " (%s)" % conn.agent if conn.agent else ""))
        self._notify(vid, conn.link, True, conn)

    def _unregister(self, conn: Conn) -> None:
        gone: list[str] = []
        with self._lock:
            for vid in conn.vids:
                by_link = self._peers.get(vid)
                # Only if it is still OUR socket. A reconnect may already have
                # filed a new one under this id, and tearing that down because
                # an old reader thread finally noticed its own death would drop
                # a live link.
                if by_link is not None and by_link.get(conn.link) is conn:
                    del by_link[conn.link]
                    gone.append(vid)
                    if not by_link:
                        del self._peers[vid]
        for vid in gone:
            self._notify(vid, conn.link, False, conn)

    def _notify(self, vid: str, link: str, up: bool, conn: Conn) -> None:
        if self.on_link is None:
            return
        try:
            self.on_link(vid, link, up, conn)
        except Exception as exc:  # noqa: BLE001
            self.log("vehicle link: on_link(%s, %s, %s) raised %s: %s"
                     % (vid, link, up, type(exc).__name__, exc))

    # ---- outbound ---------------------------------------------------------

    def send(self, vehicle_id: str, payload: bytes, link: str | None = None) -> bool:
        """Frame and send to ONE vehicle socket. False if there is none to send to.

        `link` names the socket. None means "any": the wifi one if there is
        one, else whatever is connected -- the bridge never relies on that, it
        chooses the link itself (linkpolicy.send_order).
        """
        with self._lock:
            by_link = self._peers.get(vehicle_id) or {}
            if link is not None:
                conn = by_link.get(link)
            else:
                conn = by_link.get(DEFAULT_LINK) or next(iter(by_link.values()), None)
        if conn is None:
            return False
        try:
            frame = encode(payload)
        except ValueError as exc:
            self.log("vehicle link: not sending to %s: %s" % (vehicle_id, exc))
            return False
        try:
            with conn.send_lock:
                conn.sock.sendall(frame)    # sendall: a short write truncates
            return True
        except OSError as exc:
            self.log("vehicle link: send to %s (%s, %s) failed: %s -- closing it"
                     % (vehicle_id, conn.link, conn.who, exc))
            # A write that died part way has torn the stream for the peer.
            # Closing makes it reconnect into a clean one.
            _kill(conn.sock)
            return False

    def peers(self) -> dict[str, str]:
        """vehicle_id -> Conn.label ("ip:port", or "radio via fleet_link").

        With two links up it names the wifi socket (else the other): a vehicle
        has a socket if it is on either. The per-link detail is all_links().
        """
        with self._lock:
            out = {}
            for vid, by_link in self._peers.items():
                conn = by_link.get(DEFAULT_LINK) or next(iter(by_link.values()))
                out[vid] = conn.label
            return out

    def all_links(self) -> dict[str, dict[str, dict]]:
        """vehicle_id -> link -> {who, label, adapter, agent, hello, synthesized,
        last_rx, connected_at}.

        `last_rx` is time.monotonic() at the last whole frame of any kind; the
        caller subtracts it from its own monotonic now.
        """
        with self._lock:
            return {vid: self._describe(by_link)
                    for vid, by_link in self._peers.items()}

    def links_of(self, vehicle_id: str) -> dict[str, dict]:
        """all_links() for one vehicle; empty if it is not connected."""
        with self._lock:
            return self._describe(self._peers.get(vehicle_id) or {})

    @staticmethod
    def _describe(by_link: dict[str, Conn]) -> dict[str, dict]:
        return {link: {"who": c.who, "label": c.label, "adapter": c.adapter,
                       "agent": c.agent, "hello": c.hello,
                       "synthesized": sorted(c.synthesized),
                       "last_rx": c.last_rx, "connected_at": c.connected_at}
                for link, c in by_link.items()}
