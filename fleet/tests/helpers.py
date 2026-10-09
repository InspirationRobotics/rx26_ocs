"""Shared test fixtures: frame builders, fake transports, and a pre-wired Hub.

Frames are built two independent ways on purpose. `build_frame` assembles bytes by
hand and takes its CRC from pymavlink's x25crc, so a bug in fleet's own CRC cannot
agree with itself; `encode` asks pymavlink to pack a real message.
"""
from __future__ import annotations

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
