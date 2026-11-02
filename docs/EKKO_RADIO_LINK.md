# Ekko's radio link to RoboCommand — the fleet_link OCS adapter

    Ekko's companion --TUNNEL--> RFD900ux / COM8 --> fleet_link --TCP 37564--> rx_bridge --> RoboCommand
    RoboCommand --> rx_bridge --TCP--> fleet_link --TUNNEL--> COM8 --> Ekko's companion

Decided 2026-10-08: Crusader keeps TCP over WiFi; **Ekko (UAV1) reaches the OCS
over the RFD radio**, as RXL catalogue messages carried in MAVLink TUNNEL, and a
`fleet_link` adapter on the laptop translates. The bridge is not told it is a
radio: the adapter connects to it as an ordinary vehicle client that says
`ocs_hello link=radio` ([VEHICLE_LINK.md](VEHICLE_LINK.md)).

This file is the spec for the drone side, and the record of **every place the
implementation departs from the catalogue**
(`Boat/RXL_Inter-Vehicle_Message_Catalogue`, source HTML beside the PDF) — see
*Deviations* at the end. Code: `fleet/ocs_adapter.py` (connection, policies),
`fleet/ocs_translate.py` (JSON ⇄ RXL), `fleet/ocs_heartbeat.py` (the stand-in
heartbeat), `fleet/rxl_codec.py` (the wire codec), `fleet/tunnel_log.py`.
Tests: `fleet/tests/test_ocs_*.py`, `test_rxl_codec.py`, `test_tunnel_log.py`,
`test_mavframe_tunnel.py` — loopback only.

**Nothing on Ekko speaks this yet.** Until it does, the adapter's only uplink is
the stand-in heartbeat it builds from the autopilot stream. What Ekko would have
to change: [VEHICLE_SIDE_PROPOSAL.md](VEHICLE_SIDE_PROPOSAL.md).

## Identities

| Who | MAVLink sysid / compid | RXL node | Set in |
|---|---|---|---|
| the OCS adapter | **250** / 190 | 0x1 | `fleet.toml:97-98` `[ocs]`. QGC on this laptop is 255, `gcs_radio.py` 254 |
| Ekko's companion | 200 | 0x3 | `fleet.toml:108`; = `mav_source_system`, `uav_params.yaml:103` |
| Ekko's autopilot | 1 / 1 | — | `fleet.toml:109` |
| the link | `rfd` (COM8) | — | `fleet.toml:110`: the only link the adapter listens or transmits on for this vehicle |

An RXL node id is **not** a sysid. The config loader refuses an OCS sysid equal
to a vehicle's, a companion equal to its autopilot, one sysid on two vehicles,
and a node of 0x1 or 0xF (`ocs_adapter.py:152-212`).

## On the wire

    MAVLink 2 TUNNEL (msg 385)
      payload_type     = 0x8000 | catalogue id          (0x8010 HEARTBEAT ... 0x8038 ACK)
      target_system    = 200 downlink;  0 or 250 accepted uplink
      target_component = 0
      payload          = the catalogue PAYLOAD only, little-endian, packed (<= 128 B)

The catalogue's own 12-byte frame does not travel: source, destination,
sequence and CRC are the MAVLink header's job (`rxl_codec.py:4-16`). Frames are
built by `mavframe.build_tunnel_frame` (trailing zero bytes trimmed, as MAVLink 2
requires), so a frame is **17 B + payload** at most: 12 of MAVLink 2, 5 of TUNNEL
header.

What the adapter listens to (`RadioVehicle.interested`, `ocs_adapter.py:549-554`):
on `rfd` only, TUNNELs from sysid 200 and the autopilot's HEARTBEAT,
GLOBAL_POSITION_INT, ATTITUDE and EXTENDED_SYS_STATE. A TUNNEL whose payload type
is not one of the ids below (Ekko's own boat_link traffic, inter-vehicle ids) is
counted as `foreign_tunnel`; one addressed to anyone but 0 or 250 as
`not_addressed_to_ocs` (a multipoint radio hears everything).

### Messages

Every decoder checks the exact length, every count, every enum against the
pinned RoboNation table and every coordinate against ±90/±180, and raises a plain
`RxlError` — never `struct.error`, never a half-read dict (`rxl_codec.py:24-30`).
Every encoder **refuses** a value that does not fit its field rather than
truncating it. Enums are RoboNation's numbers (`rx_bridge/gen`, revision
`8fc09f2`). lat/lon are i32 × 1e-7 deg throughout. MovingObjectAlert's
`keep_out_radius_m` (new in 8fc09f2) has **no byte** in 0x24: the adapter logs
`keep_out_radius_m=… NOT relayed over the radio` once per radius. Add it when Task 4
Disruptive is attempted.

| id | name | dir | bytes | layout (offset: field) |
|---|---|---|---|---|
| 0x10 | HEARTBEAT | up | 24 | 0 vehicle u8 (node) · 1 flags u8 (b0-1 RobotState, b2-3 FlightPhase, b4 pos_valid, b5 att_valid, b6 alt_valid, b7 dead_reckoned) · 2 lat · 6 lon · 10 spd cm/s u16 · 12 hdg cdeg u16 (0xFFFF = unresolved) · 14 roll cdeg i16 · 16 pitch cdeg i16 · 18 alt cm i32 (HAE) · 22 task u8 (RxTask) · 23 phase u8 |
| 0x11 | SAFE_PASSAGE | up | 21+9n, n ≤ 10 (111) | 0 time_boot_ms u32 · 4 entry lat · 8 entry lon · 12 exit lat · 16 exit lon · 20 n u8 · 21+9i: lat, lon, BeaconState u8. Entry/exit 0,0 = not known |
| 0x12 | PIPELINE_SURVEY | up | 11 | 0 lat · 4 lon · 8 count u8 (≤ 8) · 9 segments u16, 2 bits each, segment i at bits 2i |
| 0x13 | RESOURCE_DELIVERY | up | 3 | task u8 · resource_color u8 · delivery_circle_color u8 |
| 0x14 | DOCKING | up | 1 | bay_id u8 |
| 0x15 | FIREFIGHTING | up | 1 | window_id u8 |
| 0x16 | INCIDENT_ACK | up | 2 | cmd_seq u16 |
| 0x17 | READINESS | up | 2 | cmd_seq u16 |
| 0x20 | RUN_START | down | 3 | 0 run_id u16 · 2 flags u8 (0) |
| 0x21 | ASSISTANCE_REQUEST | down | 11 | 0 cmd_seq u16 · 2 lat · 6 lon · 10 vehicle_type u8 |
| 0x22 | KEEP_OUT_ZONE | down | 14 | 0 cmd_seq u16 · 2 lat · 6 lon · 10 radius dm u16 · 12 affected mask u8 · 13 flags u8 (b0 active) |
| 0x23 | ALL_CLEAR | down | 3 | 0 cmd_seq u16 · 2 vehicle_type u8 |
| 0x24 | MOVING_OBJECT_ALERT | down | 13 | 0 lat · 4 lon · 8 heading cdeg u16 (< 36000) · 10 speed cm/s u16 · 12 affected mask u8 |
| 0x25 | READINESS_CONFIRM | down | 3 | 0 report_seq u16 · 2 vehicle u8 (node) |
| 0x26 | COURSE | down | 4+8n, 3 ≤ n ≤ 15 (≤ 124) | 0 course_epoch u8 · 1 pinger_freq_hz u16 · 3 n u8 · 4+8i: lat, lon |
| 0x27 | GEOFENCE_INFO | down | 5+8n, 1 ≤ n ≤ 15 per part | 0 fence_epoch u8 · 1 flags u8 (b0 closed) · 2 part u8 · 3 n_parts u8 (≤ 16) · 4 count u8 · 5+8i: lat, lon |
| 0x38 | ACK | both | 3 | msg_id u8 · seq u8 (the MAVLink header seq of the frame acknowledged) · src u8 (RXL node of its sender) |

- **affected mask**: bit *n* = VehicleType *n* (USV bit 1, UUV bit 2, UAV
  bit 3). A mask with a bit for a type that does not exist is refused.
- **course_epoch**: low byte of CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
  over the UTF-8 `course_id`. The id itself does not travel; a changed epoch
  says the course changed.
- **fence_epoch**: low byte of the same CRC over every vertex as `<ii`, in
  order. All parts of one fence share it.
- **HEARTBEAT.phase** (ours, not RoboNation's): SEARCH 0, APPROACH 1, EXECUTE 2,
  COMPLETE 3, HOLD 4, none 0xFF. The OCS does not use it.

### Sizes against the 128-byte TUNNEL

Measured with the codec and `build_tunnel_frame`:

| Message | payload | frame |
|---|---|---|
| HEARTBEAT | 24 | 41 |
| SAFE_PASSAGE, 10 buoys | 111 | 128 |
| RUN_START | 3 | ≤ 20 (18 with flags 0) |
| ASSISTANCE_REQUEST | 11 | 28 |
| KEEP_OUT_ZONE | 14 | 31 |
| ALL_CLEAR, ACK, READINESS_CONFIRM | 3 | 20 |
| MOVING_OBJECT_ALERT | 13 | 30 |
| COURSE, 5 corners / 15 corners | 44 / 124 | 61 / 141 |
| GEOFENCE_INFO, the 5-vertex bench fence | 45 | 62 |
| COMMAND_LONG (the EXTENDED_SYS_STATE request) | — | 44 |

**COURSE** above 15 corners is refused, not split (the handbook's boundary is a
handful of corners). **GEOFENCE_INFO** splits: 15 vertices a part, up to 16
parts (240 vertices); a fence that fits one part is one message with
`part = 0, n_parts = 1` (`pack_geofence_parts`, `rxl_codec.py:671-687`). A
31-vertex fence is three parts of 125, 125 and 13 bytes.

**The receiver's rule** (`GeofenceAssembler`, `rxl_codec.py:708-735` — the
executable form of what Ekko must implement): collect parts by
`(fence_epoch, n_parts)`; a part with a different epoch or part count
**discards everything held** (a newer fence supersedes an older one; never mix
two generations); when all `n_parts` are in, concatenate them in part order,
re-derive the epoch from the assembled polygon and compare it with the one on
the air — a mismatch discards the parts and is an error. The result is
**information**: compare it with the aircraft's own fence and alarm. Never
upload it.

## Uplink: RXL → RxReport JSON

`_on_tunnel` (`ocs_adapter.py:564-606`) and `report_from_rxl`
(`ocs_translate.py:231-239`). Every report is attributed to the configured
`vehicle_id`, `sent_at` is the **laptop's receive time** (the bridge re-stamps
it and keeps ours only to measure staleness), and it carries a `seq` of the
adapter's own (below). Refusal, not repair (`ocs_translate.py:148-228`):

| RXL | RxReport body | refused when |
|---|---|---|
| HEARTBEAT | `heartbeat` (vehicle_type from config, `depth_m` 0, `altitude_hae_m` = alt) | vehicle node ≠ the configured node; robot_state, task or (UAV) flight_phase UNKNOWN; task not an RxTask; any valid flag clear; heading 0xFFFF |
| SAFE_PASSAGE | `safe_passage` (0,0 entry/exit omitted; time_boot_ms not forwarded) | a buoy state UNKNOWN |
| PIPELINE_SURVEY | `pipeline_survey` | a segment UNKNOWN |
| RESOURCE_DELIVERY | `resource_delivery` | any field UNKNOWN |
| DOCKING / FIREFIGHTING | `docking` / `firefighting` | — |
| INCIDENT_ACK | `incident_ack.command_seq` | — |
| READINESS | `readiness.command_seq` | — |

**ACKs the adapter sends.** INCIDENT_ACK and READINESS are answered with an RXL
ACK `{msg_id, seq = the MAVLink seq of the frame received, src = the vehicle's
node}`, addressed to sysid 200 — **only once the bridge has taken the report**.
An ACK is a promise; with the bridge down the report is not ACKed and Ekko's
retry is the net (`not_forwarded_no_bridge`).

**Duplicates.** Retries arrive more than once, and every attempt has a new
MAVLink seq, so the seen-set is `(msg_id, payload bytes)` with a 30 s horizon
(`DEDUP_HORIZON_S`), for the ACKed kinds only. A duplicate is re-ACKed (the
first ACK may be what was lost) and not forwarded again. A report enters the
seen-set only after the bridge accepted it.

## The heartbeat

**Ekko's own RXL HEARTBEAT always wins.** While one has been heard within
`companion_fresh_s` (2.0 s), only the companion's heartbeat is relayed — the
latest one per period, at `rate_hz` (2.0, the handbook's rate) — and **if it
cannot be forwarded** (a clear valid flag, an UNKNOWN), nothing is sent and the
stand-in stays quiet: the vehicle saying "I do not know my position" must not be
contradicted by us guessing one (`heartbeat_tick`, `ocs_adapter.py:648-673`).

**Otherwise the stand-in.** `ocs_heartbeat.py` builds UAV1's heartbeat from the
autopilot's own MAVLink on the radio — a port of Ekko's
`uav_groundstation/heartbeat_core.py`, fed from the same messages
`telemetry_bridge.py` reads, and different only toward sending **less**:

| Field | From |
|---|---|
| `state` | STATE_MANUAL if disarmed; STATE_AUTO if armed in AUTO, GUIDED, GUIDED_NOGPS, RTL, SMART_RTL, LAND, LOITER, AUTO_RTL, CIRCLE, FOLLOW, ZIGZAG; else STATE_MANUAL (HEARTBEAT from compid 1). Never UNKNOWN |
| `position`, `heading_deg`, `spd_mps` | GLOBAL_POSITION_INT lat/lon, `hdg`, `hypot(vx, vy)` |
| `altitude_hae_m` | GLOBAL_POSITION_INT.alt (AMSL) + `geoid_separation_m` |
| `roll_deg`, `pitch_deg` | ATTITUDE |
| `flight_phase` | EXTENDED_SYS_STATE.landed_state: ON_GROUND → GROUNDED; IN_AIR, TAKEOFF, LANDING → AIRBORNE |
| `current_task` | the literal **TASK_NONE** — exactly what `heartbeat_core.py:138` sends ("no task-state source exists on the aircraft yet"). A mirror of the aircraft's reporter, not our guess |
| `vehicle_type`, `depth_m` | config, 0.0 |
| `sent_at` | the time of the **newest input used**, not now — old data must not look fresh |

The adapter's hello declares `synthesized: ["heartbeat"]`, so the bridge never
lets this outrank a real heartbeat from Ekko's WiFi client.

**It sends nothing** — and says why in `/fleet` `heartbeat.reason` — when:
no GLOBAL_POSITION_INT within 1.0 s; no autopilot HEARTBEAT within 2.5 s;
position exactly 0,0 (EKF has no origin; `heartbeat_core` passes this); heading
65535; no ATTITUDE within 1.0 s (an omitted float would go out as 0.0, a claim of
"level"); `geoid_separation_m` not configured; flight phase unknown (no
EXTENDED_SYS_STATE within 2.5 s and the fallback off, landed_state UNDEFINED, or
a landed_state not in the table). The bridge refuses FLIGHT_PHASE_UNKNOWN from a
UAV, and the field is relayed to Garuda for Network Remote ID, so the
armed + height-above-home fallback is **off** unless `phase_fallback = true`.

**`geoid_separation_m = -24.6`** (`fleet.toml:120`) is copied from
`uav_params.yaml:95/:188` and is Sarasota's — wrong for San Diego and
Singapore. It is left equal to the aircraft's on purpose (`TODO(Chase)`,
`fleet.toml:115`). `GPS_RAW_INT.alt_ellipsoid` would remove the constant.

### Asking the autopilot for EXTENDED_SYS_STATE

ArduCopter 4.7 puts EXTENDED_SYS_STATE (245) in no stream group; it is on the
radio only if asked for. The adapter sends the autopilot exactly **one** kind of
frame: COMMAND_LONG `MAV_CMD_SET_MESSAGE_INTERVAL` (511), sysid 250 → (1, 1),
param1 245, param2 1 000 000 µs (`request_flight_state`,
`ocs_adapter.py:677-718`). The rule and interval are `gcs_radio.py`'s `WANTED`
list (`(245, 1.0 Hz)`, `WANTED_STALE_S = 10`), so the two requesters can never
fight: each one's request is the other's no-op. It asks only

- while the stand-in is the one speaking (not while a companion heartbeat is fresh),
- when 245 has not been heard for 10 s (or ever) — never re-asking for one that
  is arriving (that stalled the aircraft's telemetry for 5-10 s once),
- when the autopilot is alive on the link (HEARTBEAT within 2.5 s, and heard
  within the hub's 30 s route age), and
- at most once per 10 s.

State and counts: `/fleet` `ocs.vehicles.UAV1.heartbeat.flight_state_request`
(`state` in words, `sent`, `refused`, `last_sent_age_s`). Not a TUNNEL, so not in
the tunnel log.

## Downlink: bridge JSON → RXL

`on_bridge_frame` → `_dispatch` (`ocs_adapter.py:722-754`):
`ocs_directive` is counted and not sent (no RXL message; the vehicle holds until
RUN_START); `ocs_info` becomes COURSE and GEOFENCE_INFO, each with its own
refusal so a course that does not fit cannot cost the vehicle its fence; an
RxCommand becomes one of 0x20-0x25 (`command_to_outgoing`,
`ocs_translate.py:295-355`). Both proto names and lowerCamelCase are accepted,
enums by name or number; a field protobuf omitted is its zero value.

**Domain.** ASSISTANCE_REQUEST, KEEP_OUT_ZONE, ALL_CLEAR and
MOVING_OBJECT_ALERT go only to a vehicle whose type they name; an empty list or
`TYPE_UNKNOWN` means every type, on both sides of the comparison
(`types_overlap`, `ocs_adapter.py:430-436`). Otherwise `not_for_this_vehicle`.

**Pending messages** are keyed: `run_start`; `assist`; `(zone, cmd_seq)`;
`(clear, cmd_seq)`; `moving`; `(confirm, report_seq, node)`; `course`;
`(fence, part)`. The same key with the same payload is a duplicate (counted,
not restarted); the same key with a new payload supersedes the old. One
assistance request and one moving-object alert at a time.

### Repeat, ACK and give-up policies

`POLICIES`, `ocs_adapter.py:403-423`:

| Message | Burst | Then | Ended by | Safety stop |
|---|---|---|---|---|
| RUN_START | 5 × 0.5 s | every 10 s | RXL ACK | give up after 600 s (`retry_give_up_s`) |
| ASSISTANCE_REQUEST | 3 × 0.5 s | every 5 s | RXL ACK, or INCIDENT_ACK quoting its cmd_seq | give up after 600 s |
| KEEP_OUT_ZONE | 3 × 0.5 s | every 5 s, for the life of the zone | ALL_CLEAR covering its type; a new run_id. An ACK or INCIDENT_ACK only cuts the burst short | expires after `zone_ttl_s` 3600 s; at most 8 zones (the 9th is refused) |
| ALL_CLEAR | 3 × 0.5 s | every 5 s | RXL ACK, or INCIDENT_ACK | give up after 600 s |
| MOVING_OBJECT_ALERT | — | every 1 s | not refreshed by the bridge for `moving_ttl_s` 30 s; ALL_CLEAR covering its type; a new run_id | — |
| READINESS_CONFIRM | 3 × 0.5 s | — | RXL ACK, early | — |
| COURSE | once | every 10 s | a new course, or `course: null` | none: repeats while known |
| GEOFENCE_INFO | once per part, parts 0.25 s apart | every 10 s per part | a different fence | none |

- An RXL ACK matches when its `msg_id` is the message's, its `seq` is the
  MAVLink seq of **any of the last 16 attempts**, and `src` is 0x1 (the OCS).
- ALL_CLEAR first cancels the pending zones and moving alerts it covers, then is
  sent itself.
- A RUN_START with a run_id different from the last drops the previous run's
  zones, alerts, assistance requests, all-clears and confirms (`cancelled`).
- Neither the companion nor the autopilot heard on `rfd` within 30 s → the
  send is not attempted (`tx_no_route`);
  the link refuses it → `tx_refused`. Either way it is retried in 0.5 s and
  does not count as an attempt. Nothing is keyed into an empty radio.
- Give-ups and expiries are logged; give-ups as a WARNING.

### u16 refusals

The radio carries `run_id`, every `cmd_seq` and `report_seq` as u16;
RoboCommand's are uint32. A value that does not fit — a `run_id` above 65535, a
radius above 6553.5 m, a speed above 655.35 m/s — is **refused, never
truncated** (a wrapped number is a wrong number that looks right): a WARNING
`... NOT sent on the radio - refused, not truncated or repaired`, plus
`downlink.refusals_by_reason` keyed without the value (`"RUN_START.run_id does
not fit u16"`; at most 32 distinct reasons, then `"(other reasons)"`),
`encode_errors` and `last_encode_error` (`_refuse`, `ocs_adapter.py:777-788`).
The bridge has already logged the command as routed to `UAV1/radio`; this is the
only place the operator learns it never went.

### Readiness round trip

1. Ekko sends RXL READINESS `{cmd_seq: c}`.
2. The adapter forwards `{seq: n, readiness: {command_seq: c}}` — `n` is its
   own counter, 1 … 2³²−1, stepped for **every** report it forwards — and,
   once the bridge has taken it, remembers `n → c` (last 256) and ACKs.
3. The bridge keeps `n` as the report's own seq, publishes it as its own
   seq `m`, and remembers `(UAV1, m) → n`.
4. RoboCommand's ReadinessConfirm names `m`; the bridge rewrites it to `n`
   ([VEHICLE_LINK.md](VEHICLE_LINK.md)).
5. The adapter maps `n → c` and sends READINESS_CONFIRM
   `{report_seq: c, vehicle: 0x3}` — so on the radio, **report_seq is the
   cmd_seq of the READINESS being confirmed**, the one number Ekko has for it.
   An `n` it does not hold is refused, never guessed
   (`ocs_translate.py:341-353`).

Both maps are in memory: a restart of fleet_link (or ocs_web) between a
READINESS and its confirm loses the confirm, loudly.

## The tunnel log

`run/tunnel/<UTC>.jsonl` (`YYYYmmddTHHMMSSZ`, `-2`… on a clash), one file per
fleet_link start, relative to the OCS folder (`[tunnel_log] dir`,
`fleet.toml:82-84`; `--tunnel-dir DIR|NONE`). **Only** TUNNEL (385) and the
robotx dialect's own 42xxx messages — or the whole 42000-42999 block if the
dialect failed to load — both directions: `rx` (accepted from a link), `tx`
(written toward a link, `origin` `output:<name>` or `adapter:UAV1`), `refused`
(a consumer's frame that went nowhere). Everything else is in the drone's
flight log. Per line: `t`, `mono`, `dir`, `link`/`output`, `origin`, `seq`,
`msgid`, `msg`, `src_sysid`, `src_compid`, `validated`; for TUNNEL also
`target_sysid`, `target_compid`, `payload_type` (+ `_hex`, `_name`), `len`, `b64`,
and `decoded` (or `decode_error` — a corrupt frame is evidence). Id 0x13 carries
a `note` about the Mesh-bench clash.

**File I/O is not on the routing path** (`tunnel_log.py:35-43`): the hub
observer only queues a dict (bounded, 4096 lines); one writer thread writes,
flushes when caught up and at least every second, and reopens a failed file
every 5 s. A full queue drops the line (`queue_dropped`) rather than wait — a
stalled OneDrive costs log lines, never routing. A clean stop writes out the
queue; a hard crash loses about a second. `/fleet` `tunnel_log`: `path`,
`lines`, `errors`, `dropped`, `last_error`, `open`, `queued`, `queue_max`,
`queue_dropped`.

## `/fleet` → `ocs`

`{"enabled": false, "reason": ...}` when off (`[ocs] enabled = false`,
`--ocs-bridge NONE`, or no `[[ocs.radio_vehicle]]`). Else `bridge`, `sysid`,
`compid`, `tx` (`frames`, `bytes`, `bytes_per_s` — everything the adapter put
on the air), `inbox_overflow`, and per vehicle (`status`,
`ocs_adapter.py:975-1019`):

- `bridge`: `connected`, `connects`, `disconnects`, `retry_in_s`, `last_error`,
  frame counts. ocs_web not running is normal; it redials 1 → 10 s.
- `uplink`: `last_report_type`/`_age_s`, `by_type`, `forwarded`,
  `not_forwarded_no_bridge`, `decode_errors`, `refused` + `last_refusal`,
  `duplicates`, `report_seq`, `readiness_seqs_held`, `ignored`.
- `heartbeat`: `source` (`companion` | `synthesized` | `none`), `reason`,
  `sent`, `suppressed`, `phase_source`, `current_task`, `companion_age_s`,
  `inputs` (the four input ages), `flight_state_request`.
- `downlink`: `queue`, `sent` by type, `retries`, `acks_matched`/`_unmatched`/
  `_sent`, `give_ups`, `superseded`, `expired`, `cancelled`,
  `duplicates_from_bridge`, `tx_refused`, `tx_no_route`, `not_for_this_vehicle`,
  `refusals_by_reason`, `directives_ignored`, `run_id`, `course`, `geofence`.

The console summary adds `ocs UAV1 bridge up|DOWN hb <source> (<reason>)`.

## Radio budget

The catalogue budgets about 2 kB/s of application payload for the whole mesh
(§6) and sizes frames at payload + 12. On TUNNEL it is payload + 17, so a
heartbeat is 41 B, not 36. What the adapter itself costs:

| | Air |
|---|---|
| idle: COURSE + a 1-part fence, every 10 s | ≈ 12 B/s |
| each active keep-out zone (31 B / 5 s) | ≈ 6 B/s (8 zones ≈ 50 B/s) |
| a moving-object alert | 30 B/s |
| RUN_START until ACKed | 5 × 18 B in 2.5 s, then 18 B / 10 s |
| the EXTENDED_SYS_STATE request | ≤ 44 B / 10 s |
| EXTENDED_SYS_STATE itself, from the autopilot, once asked | ≈ 14 B/s, on the autopilot's radio port |
| the stand-in heartbeat | **0** — built on the laptop from traffic already on the air |

Ekko's own RXL heartbeat, when it exists, costs 41 B per frame at whatever rate
it sends (the catalogue's 4 Hz is 164 B/s); the adapter relays at most 2 Hz of it.

## Deviations from the catalogue

Each is deliberate; each is a place the drone side must follow this file, not
the catalogue.

1. **Carrier.** The RXL frame (SYNC, VER|CLS, MSG_ID, SRC|DST, SEQ, T_MS, LEN,
   CRC-16) is replaced by MAVLink TUNNEL, `payload_type = 0x8000 | id`. No
   protocol-version nibble, no priority-class nibble; 17 B of overhead, not 12.
2. **No T_MS.** Relayed reports are stamped with the laptop's receive time;
   SAFE_PASSAGE's own `time_boot_ms` is carried but not used. Time spent on the
   aircraft is invisible to the bridge's staleness check.
3. **SAFE_PASSAGE is 21 + 9n (111 B at 10 buoys)**, not the catalogue's
   18 + 9n (108): the Mesh bench layout (`Mesh/rxl.py:211`), `time_boot_ms` u32
   first, standing in for T_MS.
4. **Addressing.** Downlink goes to sysid 200 (`target_system`), per vehicle —
   not broadcast to node 0xF, including RUN_START ("broadcast 5×"). Uplink is
   accepted addressed to 0 or 250.
5. **Duplicate suppression** is `(msg_id, payload)` over 30 s, not
   `(src, msg_id, seq)`: every MAVLink retry has a new seq. Only for the ACKed
   uplink kinds.
6. **ACK.seq** is the MAVLink header seq of the attempt acknowledged, and the
   OCS accepts any of a message's last 16 attempts. **ACK.src** is the RXL node
   of the acknowledged frame's sender (0x1 for ours). The adapter addresses its
   ACKs to sysid 200.
7. **RUN_START** repeats per its own row (5 × 2 Hz, then 0.1 Hz), not §4.5's
   general 3 × then 0.2 Hz — and gives up after 600 s.
8. **ASSISTANCE_REQUEST** has no ACK mark; it is retried like a state transfer
   until Ekko's INCIDENT_ACK (or an RXL ACK) arrives, then given up after 600 s.
9. **KEEP_OUT_ZONE is 14 bytes: `cmd_seq u16` replaces `zone_id u8`**
   (catalogue 13 B). KeepOutZone has no zone id; the command seq is its only
   identity, and the IncidentAck Ekko owes must quote it. A 3 × 2 Hz burst
   precedes the catalogue's 0.2 Hz. The active flag is always 1 on the air — a
   cleared zone stops being repeated; no `active = 0` frame is sent. Safety
   expiry 3600 s; at most 8 zones.
10. **ALL_CLEAR with TYPE_UNKNOWN** (absent in the bridge's JSON) means every
    type: it cancels every zone and alert.
11. **MOVING_OBJECT_ALERT "while active"** is defined as "refreshed by the bridge
    within 30 s"; one alert at a time, the latest wins at a steady 1 Hz. An
    empty affected list means every type. (Handbook 3.4.12: it has no
    acknowledgement or all-clear; ALL_CLEAR covering its type still ends it here.)
12. **READINESS_CONFIRM.report_seq** carries the `cmd_seq` from Ekko's own
    READINESS (see *Readiness round trip*); `vehicle` is the RXL node. Sent 3 ×
    at 2 Hz; an ACK ends it early.
13. **INCIDENT_ACK and READINESS** are `cmd_seq u16` — the catalogue sizes them
    at 2 B without naming the field.
14. **COURSE**: `course_epoch` is defined (low byte of CRC-16/CCITT-FALSE over
    the UTF-8 `course_id`; the catalogue says only "hash"); an explicit corner
    count byte; at most 15 corners, refused beyond; repeated every 10 s
    indefinitely.
15. **GEOFENCE_INFO, id 0x27, is new.** The catalogue has no message for the
    UAV geofence. 0x27 is free in the catalogue, the Mesh bench, the dialect
    (native 42027 unused) and `boat_link.py` (`rxl_codec.py:105-111`).
    Information only for the companion. The autopilot gets the same polygon
    separately, as an ArduPilot inclusion fence written by the adapter itself over
    MAVLink when `fence_upload = true` (`fleet/fence_upload.py`; fleet/README.md).
16. **The affected-type mask** is defined: bit n = VehicleType n.
17. **HEARTBEAT.phase** is numbered 0-4 with 0xFF = none (the catalogue lists
    the names only). The adapter relays at most 2 Hz of Ekko's heartbeat (the
    handbook's rate), not the catalogue's 4 Hz.
18. **0x13 clashes** with the Mesh bench / native `RXL_RESOURCE_DELIVERY`
    (platform_id, tin_color, state) — same 3 bytes, different meaning. The codec
    decodes the catalogue form; the tunnel log marks it.
19. **Not implemented:** §5's priority classes, shaping and shedding (the
    adapter's own traffic is a handful of small frames); decoding of the
    inter-vehicle ids 0x18, 0x19, 0x30-0x37, 0x39 (named in the log only).
20. **Link choice (§9.1)** is per report kind, real over synthesized, not
    "radio wins, ignore TCP for that vehicle" — see VEHICLE_LINK.md. A drop is
    wire-logged per frame, not as one "switch" line.

## Not yet verified on hardware

All of the above runs on loopback. Not yet seen live: Ekko's companion sending or
receiving one RXL TUNNEL; the Cube forwarding a TUNNEL from 250 to 200 (ArduPilot
forwards to a sysid only once it has heard it on a port); SET_MESSAGE_INTERVAL
from the radio actually putting EXTENDED_SYS_STATE on the radio port; the
stand-in heartbeat reaching RoboCommand's test server.
