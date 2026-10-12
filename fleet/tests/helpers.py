"""Shared test fixtures: frame builders, fake transports, and a pre-wired Hub.

Frames are built two independent ways on purpose. `build_frame` assembles bytes by
hand and takes its CRC from pymavlink's x25crc, so a bug in fleet's own CRC cannot
agree with itself; `encode` asks pymavlink to pack a real message.
"""
from __future__ import annotations

import collections
import importlib.util
import logging
import socket
from typing import Optional

from pymavlink.dialects.v20 import ardupilotmega as ap
from pymavlink.generator.mavcrc import x25crc

from fleet import mavframe
from fleet.fleet_link import DEFAULT_CONFIG, load_config
from fleet.hub import Hub, LinkIO, OutputIO

# Keep the library's warnings (link DOWN, write failed) off the test console; the
# tests that care capture them with assertLogs.
logging.getLogger("fleet").addHandler(logging.NullHandler())

DIALECT = mavframe.default_dialect()
HAVE_ROBOTX = DIALECT.robotx_path is not None


def load_robotx_module():
    spec = importlib.util.spec_from_file_location("test_robotx", DIALECT.robotx_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_frame(msgid: int, payload: bytes = b"", *, sysid: int = 1, compid: int = 1,
                seq: int = 0, version: int = 2, signed: bool = False,
                crc_extra: Optional[int] = None) -> bytes:
    """A frame assembled by hand, CRC from pymavlink's x25crc."""
    if crc_extra is None:
        crc_extra = DIALECT.get(msgid).crc_extra
    if version == 1:
        head = bytes([0xFE, len(payload), seq, sysid, compid, msgid])
    else:
        head = (bytes([0xFD, len(payload), 1 if signed else 0, 0, seq, sysid, compid])
                + msgid.to_bytes(3, "little"))
    crc = x25crc(head[1:] + payload)
    crc.accumulate(bytes([crc_extra]))
    out = head + payload + crc.crc.to_bytes(2, "little")
    if signed:
        out += bytes(range(100, 113))          # 13 signature bytes; never checked
    return out


def encode(name: str, *args, sysid: int = 1, compid: int = 1, seq: int = 0,
           version: int = 2) -> bytes:
    """A real frame from pymavlink: encode('heartbeat', 2, 3, 129, 4, 4, 3, ...)."""
    mav = ap.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    mav.seq = seq
    msg = getattr(mav, name + "_encode")(*args)
    return bytes(msg.pack(mav, force_mavlink1=(version == 1)))


def heartbeat(sysid=1, *, type=2, autopilot=3, base_mode=129, custom_mode=4, compid=1,
              seq=0, version=2) -> bytes:
    return encode("heartbeat", type, autopilot, base_mode, custom_mode, 4, 3,
                  sysid=sysid, compid=compid, seq=seq, version=version)


def position(sysid=1, *, lat=-353632621, lon=1491652374, alt=584090, rel=10500,
             vx=300, vy=400, hdg=9000, seq=0) -> bytes:
    return encode("global_position_int", 0, lat, lon, alt, rel, vx, vy, 0, hdg,
                  sysid=sysid, seq=seq)


def gps_raw(sysid=1, *, fix=3, lat=-353632621, lon=1491652374, alt=584090, vel=250,
            yaw=0, seq=0) -> bytes:
    """GPS_RAW_INT: time, fix_type, lat, lon, alt, eph, epv, vel, cog, sats, then the
    extensions (alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw)."""
    return encode("gps_raw_int", 0, fix, lat, lon, alt, 100, 100, vel, 9000, 10,
                  0, 0, 0, 0, 0, yaw, sysid=sysid, seq=seq)


def command_long(target: int, *, source=255, seq=0, command=511) -> bytes:
    return encode("command_long", target, 1, command, 0, 0, 0, 0, 0, 0, 0, 0,
                  sysid=source, compid=190, seq=seq)


def mission_request_list(target: int, *, source=254, seq=0) -> bytes:
    return encode("mission_request_list", target, 1, sysid=source, compid=190, seq=seq)


def gcs_heartbeat(*, source=255, seq=0) -> bytes:
    return encode("heartbeat", 6, 8, 0, 0, 4, 3, sysid=source, compid=190, seq=seq)


class Clock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeLink(LinkIO):
    """A link whose sends are recorded instead of written."""

    def __init__(self, name, kind="udp", *, needs_addr=True, stream=False, accept=True):
        super().__init__(name)
        self.kind = kind
        self.needs_addr = needs_addr
        self.stream = stream
        self.accept = accept
        self.sent = []                    # [(data, addr)]

    def send(self, data, nframes, addr):
        if not self.accept:
            return False
        self.sent.append((data, addr))
        self.hub.link_tx(self.name, nframes, len(data))
        return True


class FakeOutput(OutputIO):
    def __init__(self, name, accept=True):
        super().__init__(name)
        self.accept = accept
        self.sent = []                    # [bytes]
        self.local_port = 50000

    def send(self, data):
        if not self.accept:
            return False
        self.sent.append(data)
        return True


def frames_of(datagrams):
    """Every frame inside a list of datagrams, flattened, asserting alignment."""
    out = []
    for d in datagrams:
        stats = mavframe.ParseStats()
        got = mavframe.split_datagram(DIALECT, d, stats)
        assert stats.garbage_bytes == 0 and stats.bad_crc == 0, "datagram is not frame-aligned"
        out.extend(got)
    return out


class Rig:
    """The shipped fleet.toml's outputs on a Hub with fake links and a fake clock."""

    def __init__(self):
        self.cfg = load_config(DEFAULT_CONFIG)
        self.clock = Clock()
        self.wifi = FakeLink("wifi", "udp", needs_addr=True, stream=False)
        self.rfd = FakeLink("rfd", "serial", needs_addr=False, stream=True)
        self.out = {o.name: FakeOutput(o.name) for o in self.cfg.outputs}
        self.hub = Hub(DIALECT, [self.wifi, self.rfd],
                       [(o, self.out[o.name]) for o in self.cfg.outputs],
                       self.cfg.vehicle_names(), clock=self.clock, wall=lambda: 1.7e9)

    def heard(self, link: str, raw: bytes, addr=None):
        self.hub.link_rx(link, raw, addr)

    def from_consumer(self, output: str, raw: bytes):
        self.hub.output_rx(output, raw)

    def got(self, output: str):
        return frames_of(self.out[output].sent)

    def got_sysids(self, output: str):
        return sorted({f.sysid for f in self.got(output)})


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def free_tcp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def tunnel_frame(payload_type: int, data: bytes, *, sysid: int = 200, compid: int = 191,
                 target: int = 0, target_comp: int = 0, seq: int = 0,
                 declared_len: int = None) -> bytes:
    """A TUNNEL frame built by pymavlink (independent of fleet's own builder)."""
    padded = data + bytes(128 - len(data))
    return encode("tunnel", target, target_comp, payload_type,
                  len(data) if declared_len is None else declared_len, padded,
                  sysid=sysid, compid=compid, seq=seq)


def make_hub(*, rfd_accept: bool = True, with_wifi: bool = False, clock: Clock = None):
    """A Hub with fake links ("rfd" serial, optionally "wifi" udp) and NO outputs, on a fake
    clock. Returns (hub, clock, rfd, wifi)."""
    clock = clock or Clock()
    rfd = FakeLink("rfd", "serial", needs_addr=False, stream=True, accept=rfd_accept)
    wifi = FakeLink("wifi", "udp", needs_addr=True, stream=False) if with_wifi else None
    cfg = load_config(DEFAULT_CONFIG)
    hub = Hub(DIALECT, [rfd] + ([wifi] if wifi else []), [], cfg.vehicle_names(), clock=clock,
              wall=lambda: 1.7e9)
    return hub, clock, rfd, wifi


class FakeAutopilot:
    """ArduPilot's side of the fence dialog, as MissionItemProtocol_Fence plays it: PARAM_REQUEST_READ
    answered with PARAM_VALUE, a fence download answered item by item, an upload driven by its own
    MISSION_REQUEST_INTs (re-requested about once a second) and committed only when complete.

    pymavlink decodes everything sent to it and encodes every reply, so each exchange also checks
    fleet's hand-packed layouts against an independent implementation. Knobs: `drop_ours` loses
    our next frame(s) of a type before they arrive, `drop_replies` loses its own next reply of a
    type, `nack` rejects every upload with that MAV_MISSION_RESULT, `nack_without_polygon_bit`
    rejects one while FENCE_TYPE lacks bit 4 (as Ekko's bring-up notes), `shift_m` commits every
    vertex that far north (a readback mismatch), `silent_missions` answers no mission traffic."""

    def __init__(self, *, params=None, fence=None, sysid=1, compid=1):
        self.params = dict({"FENCE_TYPE": 7.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 0.0}
                           if params is None else params)
        self.fence = [dict(i) for i in (fence or [])]   # {command, param1, lat, lon}
        self.mav = ap.MAVLink(None, srcSystem=sysid, srcComponent=compid)
        self.dec = ap.MAVLink(None)
        self.out = []                                    # raw reply frames, oldest first
        self.heard = []                                  # every decoded message of ours (dropped ones too)
        self.drop_ours = collections.Counter()
        self.drop_replies = collections.Counter()
        self.nack = None
        self.nack_without_polygon_bit = True
        self.shift_m = 0.0
        self.silent_missions = False
        self.on_count = None                             # called when an upload's MISSION_COUNT arrives
        self.uploads = 0                                 # completed (committed) uploads
        self._recv = None                                # an upload in progress: [item or None]
        self._recv_from = None
        self._last_req = 0.0

    def names(self):
        return [m.get_type() for m in self.heard]

    # ---- what we send it
    def hear(self, raw, now):
        msg = self.dec.decode(bytearray(raw))
        self.heard.append(msg)
        name = msg.get_type()
        if self.drop_ours[name]:
            self.drop_ours[name] -= 1
            return
        src = (msg.get_srcSystem(), msg.get_srcComponent())
        if name == "PARAM_REQUEST_READ":
            pid = msg.param_id
            if pid in self.params:
                self._reply("PARAM_VALUE", "param_value", pid.encode(), self.params[pid], 9, 900, 1)
            return
        if getattr(msg, "mission_type", None) != 1 or self.silent_missions:
            return
        if name == "MISSION_REQUEST_LIST":
            self._reply("MISSION_COUNT", "mission_count", *src, len(self.fence), 1)
        elif name == "MISSION_REQUEST_INT":
            if msg.seq < len(self.fence):
                i = self.fence[msg.seq]
                self._reply("MISSION_ITEM_INT", "mission_item_int", *src, msg.seq, 0, i["command"], 0, 0,
                            i["param1"], 0, 0, 0, int(round(i["lat"] * 1e7)), int(round(i["lon"] * 1e7)), 0, 1)
            else:
                self._reply("MISSION_ACK", "mission_ack", *src, 13, 1)
        elif name == "MISSION_COUNT":
            if self.on_count:
                self.on_count()
            nack = self.nack
            if nack is None and self.nack_without_polygon_bit and not int(self.params.get("FENCE_TYPE", 0)) & 4:
                nack = 1
            if nack is not None:
                self._reply("MISSION_ACK", "mission_ack", *src, nack, 1)
                return
            if msg.count == 0:                       # MissionItemProtocol: nothing to request, clear and ACK
                self.fence, self._recv = [], None
                self.uploads += 1
                self._reply("MISSION_ACK", "mission_ack", *src, 0, 1)
                return
            self._recv, self._recv_from = [None] * msg.count, src
            self._request(now)
        elif name == "MISSION_ITEM_INT" and self._recv is not None and src == self._recv_from:
            nxt = self._recv.index(None)
            if msg.seq != nxt:
                return
            self._recv[nxt] = {"command": msg.command, "param1": msg.param1,
                               "lat": msg.x * 1e-7 + self.shift_m / 111195.0, "lon": msg.y * 1e-7}
            if None in self._recv:
                self._request(now)
                return
            self.fence, self._recv = self._recv, None
            self.uploads += 1
            self._reply("MISSION_ACK", "mission_ack", *src, 0, 1)

    def tick(self, now):
        """ArduPilot re-requests a missing upload item about once a second."""
        if self._recv is not None and now - self._last_req >= 1.0:
            self._request(now)

    def set_param(self, name, value):
        """A parameter set from QGC: ArduPilot broadcasts the new PARAM_VALUE."""
        self.params[name] = value
        self._reply("PARAM_VALUE", "param_value", name.encode(), value, 9, 900, 1)

    def _request(self, now):
        self._last_req = now
        self._reply("MISSION_REQUEST_INT", "mission_request_int", *self._recv_from, self._recv.index(None), 1)

    def _reply(self, name, encoder, *args):
        if self.drop_replies[name]:
            self.drop_replies[name] -= 1
            return
        self.mav.seq = (self.mav.seq + 1) & 0xFF
        self.out.append(bytes(getattr(self.mav, encoder + "_encode")(*args).pack(self.mav)))

    def take(self):
        out, self.out = self.out, []
        return out


def fence_items(polygon):
    """An autopilot fence holding exactly this (open) polygon."""
    return [{"command": 5001, "param1": float(len(polygon)), "lat": lat, "lon": lon} for lat, lon in polygon]
