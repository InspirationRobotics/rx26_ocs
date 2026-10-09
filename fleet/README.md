# fleet_link — the process that owns the OCS laptop's MAVLink inputs

The operator laptop has exactly two physical MAVLink inputs, and each can have
exactly one owner:

| Link | What it is | Why it needs one owner |
|---|---|---|
| `wifi` | UDP `0.0.0.0:14550`. The boat's MAVProxy unicasts its autopilot stream (sysid 2) here **and** broadcasts the same frames to `192.168.100.255:14550`, so every boat frame can arrive twice from two source ports. | A second process binding 14550 does not share the stream, it steals it, and the displaced one sees silence, not an error. |
| `rfd` | Serial `COM8` @ 115200. The RFD900ux base station, SiK multipoint master (NODEID 1 — **the mesh stops without it**). Hears Ekko's autopilot (sysid 1), Ekko's companion (200), the boat's companion `rxl_link_node` (42), and the radio's own RADIO_STATUS (51). | A serial port has one owner. |

`fleet_link` holds both, fans every frame out to loopback ports for the local
consumers (QGroundControl, the boat GUI, `gcs_radio.py`, diagnostics), carries
their replies back to the right vehicle, and reports what is alive on
`http://127.0.0.1:8098/fleet`.

It **replaces `Drone/rx26_uav/tools/scripts/radio_link.py` on this laptop** and adds
WiFi. Do not run both: they would fight over COM8.

## Run it

**System Python, not `.venv`.** `fleet_link` needs `pymavlink` and `pyserial`, which
are installed in the system Python 3.13 (stdlib `tomllib` too) and are *not* in the
OCS `.venv`. The `.venv` is deliberately untouched.

From the `OCS` directory, in PowerShell or cmd:

    python -m fleet.fleet_link --config fleet\fleet.toml

or double-click / run `OCS\FLEET_LINK.cmd`, which opens it in its own window (and
says "already running" instead of starting a second one when `:8098` answers). Any
option below can be added after the `.cmd` name. The window stays open after a
configuration error or a refused port so you can read why, and `FLEET_LINK.cmd` itself
stops with a message if the `python` on PATH has no `pymavlink`/`pyserial`.

Options (each overrides `fleet.toml`):

| Flag | |
|---|---|
| `--serial COM8` | the radio's port. `--serial NONE` switches the radio link off (it then shows as `enabled: false` in `/fleet`) |
| `--baud 115200` | the mesh's SERIAL_SPEED |
| `--wifi-port 14550` | UDP port to listen on. `--wifi-port NONE` switches WiFi off |
| `--status-port 8098` | port for `GET /fleet` |

Stop it with ctrl-C (or ctrl-Break) in its window.

Tests, from `OCS\`, on the system Python (no hardware needed):

    python -m unittest discover -s fleet/tests -t .

## Ports

Aircraft tooling owns `1454x`, the boat owns `1455x`, so an operator reading a port
number never has to work out which vehicle it is. QGroundControl is neither, so it
gets `14570`.

| Output | Loopback port | Gets | May transmit | Consumer |
|---|---|---|---|---|
| `qgc` | 14570 | `wifi` + `rfd`, all sysids | yes | QGroundControl, a UDP link listening on 14570 |
| `boat_gui` | 14554 | `wifi`, sysid 2 only | **no** (listen-only) | `Boat/rx26_asv/tools/scripts/gcs_laptop.py` (its default `--mav` is `udpin:127.0.0.1:14554`) |
| `drone_gui` | 14543 | `rfd`, all sysids | yes | `Drone/.../gcs_radio.py` (its default `--from` is `udpin:127.0.0.1:14543`) |
| `drone_diag` | 14544 | `rfd`, all sysids | **no** (listen-only) | ad-hoc probes. Kept last on purpose |

Status: `127.0.0.1:8098`.

`fleet_link` only ever **sends** to these ports, from one ephemeral source socket
per output, and replies from that consumer arrive on that same socket — so every
reply is attributable to one output. The consumer binds the port (`udpin:`); a
consumer that is not running is simply not heard, and nothing breaks. The actual
source port of each output is shown in `/fleet` as `reply_port`.

## QGroundControl setup

QGC currently holds UDP 14550 on this laptop through its **UDP autoconnect**. That
is the port `fleet_link` has to own, so one of the two has to move:

1. In QGC: *Application Settings → General → Autoconnect to the following devices*
   and **untick UDP**. (UDP autoconnect listens on 14550, which is the port `fleet_link` must own.)
2. *Application Settings → Comm Links → Add*: type **UDP**, **Listening Port 14570**,
   no server address. Connect it.

If you leave autoconnect on, `fleet_link` refuses to start and says so:
`UDP 14550 is taken - QGroundControl with UDP autoconnect? Close it or set its UDP
link to 14570`. (It binds with `SO_EXCLUSIVEADDRUSE` and never `SO_REUSEADDR`, so on
Windows a program that does set `SO_REUSEADDR` — Qt's UDP sockets share by default —
fails to start rather than silently splitting the traffic.)

QGC must not also open COM8 itself; that is what this program is for.

## How the consumers use it

* **`gcs_radio.py`** needs no flag: it already defaults to `udpin:127.0.0.1:14543`.
  It does transmit — SET_MESSAGE_INTERVAL and the mission-protocol fence reads —
  and those reach Ekko through `fleet_link`, on the radio, only if Ekko has been
  heard in the last 30 s (see *Reverse path*).
* **The boat laptop GUI** (`gcs_laptop.py`) already defaults to
  `udpin:127.0.0.1:14554`. It is listen-only: anything it sends is dropped and
  counted in `/fleet` under `dropped_listen_only`.
* **Anything new**: add an `[outputs.<name>]` table to `fleet.toml` with its own port
  (`1454x` aircraft, `1455x` boat). Never point an output at a port a link binds —
  the config loader refuses, because that would feed `fleet_link` its own output.

## What it does with frames

**Framing** (`mavframe.py`, pure). A MAVLink v1/v2 splitter for the serial byte
stream; each UDP datagram goes through the same splitter, a fresh one per datagram.
Frames are forwarded whole, bit-identical, never rewritten, and a frame is never
split across datagrams (several whole frames may share one, up to 1400 bytes).
CRC is X.25 plus `crc_extra`, from pymavlink's ardupilotmega message table **merged
with the robotx dialect** (`Mesh/dialect/gen/robotx.py`, found automatically, or set
`[dialect] robotx` / `$FLEET_ROBOTX_DIALECT`). Known id + bad CRC → counted, one
byte dropped, resync. **Unknown id → passed through unvalidated and counted**
(`unknown_id`), as `mavlink-router` does. On a healthy link `unknown_id` stays at 0;
a rising count means line noise or a message nobody loaded. Unvalidated frames are
forwarded but teach the hub nothing (no vehicle entry, no route, no reply address).

**Dedup.** Per link, a frame byte-identical to one accepted from the same link in
the last 1.0 s is dropped and counted (`dups`). This is what makes the boat's
unicast + broadcast pair arrive once. Memory is bounded (2048 frames per link).

**Forward path.** `link → outputs`, filtered by each output's `links` and `sysids`.
**Never `link → link`**: bridging WiFi into the radio would flood 7 kbit/s of air
with the boat's telemetry and make loops possible.

**Reverse path.** `output → links`, only for outputs with `transmit = true`
(everything a listen-only output sends is dropped and counted).

* The frame is parsed. If its message has a `target_system` field and it is not 0 →
  sent **only** to the link, among that output's own links, on which that sysid was
  most recently heard, provided that was within 30 s. Otherwise it is dropped and
  counted as `no_route`.
* `target_system` of 0, no such field (HEARTBEAT, ...), or an unknown msgid → sent to
  **every** link of that output.
* To `rfd`: queued; only the one thread that owns the port writes (a stale command is
  never held over a reopen: while the port is down the send is refused and counted
  as `tx_dropped`). To `wifi`: sent from the 14550 socket to the address the target
  sysid was last heard from (for target-less frames, the address of the last accepted
  frame on that link). The boat's unicast and broadcast copies come from two source
  ports; whichever copy won the dedup race is the reply address.

## `GET /fleet`

JSON, schema version 1. Blanks over guesses: a measured value older than
`stale_after_s` (3 s) is `null` and its `age_s` is kept; a position of exactly
0,0 (no fix) is `null`. `?pretty=1` indents. Sent with `Access-Control-Allow-Origin: *`
(read-only, loopback-only), so a map page served from another localhost port can
`fetch()` it directly. Numbers are never NaN.

**For a map page:** poll `/fleet` about once a second and draw every entry of `sysids`
whose `position.lat` is not `null`. Per entry, `name` is the label (`Ekko`, `Crusader`),
`link` is where it was last heard (`rfd` = the radio, `wifi` = UDP 14550),
`age_s` is how long since ANY frame from it, `position.lat/lon/heading_deg/alt_m/
groundspeed_mps` are the point and its motion, `position.age_s` is the age of that
position, and `heartbeat.mode` / `heartbeat.armed` are the flight mode and arm state.
Companion computers and the radio (sysid 200, 42, 51) are entries too, but never have
a position. A vehicle that goes quiet stays in the list with `lat`/`lon` `null` and a
growing `age_s`: grey the marker, do not keep drawing its last point. Fetch
`http://127.0.0.1:8098/fleet`, not `localhost` (the server listens on IPv4 loopback only).

```jsonc
{
  "schema": 1,
  "time": 1791500000.123,          // wall clock, unix seconds
  "uptime_s": 842.0,
  "stale_after_s": 3.0,
  "dialect": { "messages": 300, "ardupilotmega": 295, "robotx": 5,
               "robotx_path": "...\\Mesh\\dialect\\gen\\robotx.py",
               "notes": [] },      // non-empty = something degraded, e.g. robotx not loaded
  "decode_errors": 0,              // HEARTBEAT/position frames pymavlink refused (still forwarded)
  "links": {
    "wifi": {
      "type": "udp", "enabled": true,
      "open": true, "error": null, // error: why it is down (or "disabled"), else null
      "bytes_in": 0, "frames_in": 0,   // frames_in = accepted: not duplicates
      "bad_crc": 0, "unknown_id": 0, "dups": 0, "garbage_bytes": 0,
      "bytes_out": 0, "frames_out": 0, // frames actually written to the link
      "tx_dropped": 0,             // offered to the link and refused (port down, queue full)
      "write_errors": 0,
      "rate_in_Bps": 0.0,          // bytes/s in, 5 s window
      "last_frame_age_s": null     // null = never heard
    },
    "rfd": { /* same keys */ }
  },
  "outputs": {
    "qgc": {
      "port": 14570, "reply_port": 50231,  // where it sends / where its replies arrive
      "links": ["wifi", "rfd"], "sysids": null,  // null = all
      "transmit": true,
      "frames_out": 0, "bytes_out": 0, "send_errors": 0,
      "frames_in_from_consumer": 0,    // valid frames the consumer sent us
      "dropped_listen_only": 0,        // ...of which dropped because the output is listen-only
      "no_route": 0,                   // dropped: no link has heard the target (or no address)
      "bad_crc_in": 0, "garbage_bytes_in": 0
    }
  },
  "sysids": {                      // only sysids heard with a CRC-valid frame
    "1": {
      "sysid": 1,
      "vehicle": "drone",          // boat | drone | radio | unknown, from [vehicles]
      "name": "Ekko",              // display name, from [names]; "sysid 77" when unlisted
      "link": "rfd",               // where it was last heard: a link name, "wifi" (UDP) or "rfd" (serial)
      "age_s": 0.2,                // since ANY frame from it
      "msgs_per_s": 33.4,          // 5 s window; decays to 0 with silence
      "heartbeat": {
        "age_s": 0.4,              // null = no heartbeat ever
        "type": "QUADROTOR",       // MAV_TYPE name, "#n" if not in the enum
        "autopilot": "ARDUPILOTMEGA",
        "mode": "GUIDED",          // pymavlink mode_string_v10
        "armed": true              // mode and armed are null for a companion (autopilot INVALID)
      },
      "position": {
        "age_s": 0.3,              // null = no position message ever
        "source": "GLOBAL_POSITION_INT",   // or "GPS_RAW_INT"; null when blank
        "lat": -35.3632621, "lon": 149.1652374,   // null at exactly 0,0 (no fix), or stale
        "alt_m": 584.09,           // MSL, also null with no fix
        "rel_alt_m": 10.5,         // above home; null from GPS_RAW_INT (it has none)
        "heading_deg": 90.0,       // null when unknown; never a GPS course-over-ground
        "groundspeed_mps": 5.0     // null with no fix
      }
    }
  }
}
```

**Position source.** `GLOBAL_POSITION_INT` (the EKF's) is used while it holds a fresh
fix. When it does not (never sent, older than `stale_after_s`, or 0,0 because the EKF
has no origin yet) `GPS_RAW_INT` stands in **if it reports at least a 2D fix**, and
`source` says so. A GPS_RAW_INT heading comes only from its `yaw` field (a dual-antenna
GPS), never from course-over-ground. With neither, the view is blank and `age_s` is the
age of the newest position message.

When several components share one sysid (an autopilot and a companion), the
heartbeat shown is the autopilot's, so the mode does not flip between them.

## Console

One summary line every 10 s, e.g.

    19:09:05 fleet | wifi 1.4 kB/s | rfd 1.3 kB/s | 1 drone 0.2s, 200 drone 0.9s, 2 boat 0.1s | rfd bad_crc 3

Link state changes (`link rfd: DOWN - ...`, `link rfd: open`) are logged **once per
change**. The radio is reopened every 5 s while its port is missing or busy.

## Not yet verified on real hardware

Everything is tested on loopback, with a scripted fake serial port and pyserial's
`loop://`. Not yet seen live: a real RFD900ux on COM8; the boat's MAVProxy actually
unicasting and broadcasting to a laptop running this; QGC accepting a UDP link on
14570 and sending through `fleet_link`; `gcs_radio.py` and `gcs_laptop.py` against
the outputs.

## Files

| | |
|---|---|
| `mavframe.py` | framing, CRC, dialect table, target_system lookup — pure |
| `vehicles.py` | per-sysid heartbeat/position (GLOBAL_POSITION_INT, GPS_RAW_INT fallback)/rate, with the staleness rule — pure |
| `hub.py` | routing core: dedup, fan-out, learned routes, listen-only — no sockets or threads |
| `endpoints.py` | the transports: UDP link, serial link (one owner thread), output sockets |
| `fleet_link.py` | config, CLI, `/fleet` server, console summary |
| `fleet.toml` | the default configuration |
| `tests/` | `unittest`, loopback only |
| `..\FLEET_LINK.cmd` | launcher |
