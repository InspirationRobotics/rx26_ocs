# Vehicle-side changes — a proposal for Chase and Dominick

**Status: proposal. No vehicle code has been changed.** The 2026-10-08 OCS round
was OCS-side only; the team rule is that a new or edited ROS node is proposed
first, described by what it makes the vehicle *do*. Each item below says the
behaviour change, the files, the risk and the test. Nothing here may actuate
anything from the network: the RC e-stop stays the only safety path, and every
inbound frame stays advisory input for a mission executive that decides.

## What the OCS now sends and expects

Full contracts: [VEHICLE_LINK.md](VEHICLE_LINK.md) (TCP 37564, both vehicles'
WiFi clients and the radio adapter) and [EKKO_RADIO_LINK.md](EKKO_RADIO_LINK.md)
(Ekko over the RFD radio).

| | Crusader (USV1) — TCP over WiFi | Ekko (UAV1) — RXL TUNNEL over the radio |
|---|---|---|
| OCS expects up | RxReport JSON, any kind; an optional `ocs_hello` first (none needed) | TUNNEL from sysid 200, `payload_type = 0x8000 \| id`: HEARTBEAT 0x10 and reports 0x11-0x17 |
| OCS sends down | RxCommand JSON (Task 4, RunStart, ReadinessConfirm), `ocs_info {course}`, `ocs_directive {standby_auto}` | TUNNEL to sysid 200: 0x20-0x25 (RunStart, Task 4, ReadinessConfirm), 0x26 COURSE, 0x27 GEOFENCE_INFO, 0x38 ACK |
| Today, on the vehicle | `ocs_client` sends heartbeats only; every inbound frame is republished raw to `/crsd/ocs_command` | `ocs_client` sends heartbeats over WiFi only; nothing speaks RXL; the OCS builds a stand-in heartbeat from the autopilot stream |

Ekko may also keep its WiFi client. With both up the OCS takes each report
*kind* from the radio, except that Ekko's real WiFi heartbeat always beats the
laptop's synthesized one, and a kind only WiFi carries is never dropped.

## Crusader

Today: `ocs_link._deliver` hands every decoded JSON object to the node
(`Boat/rx26_asv/crusader_groundstation/crusader_groundstation/ocs_link.py:297-309`),
which republishes it unchanged as a String on `/crsd/ocs_command`
(`ocs_client_node.py:119`, `:229-233`). The one consumer is
`crusader_bt/src/bt_runner_node.cpp:813-818`, which looks for the substring
`"readiness_confirm"` and stops re-sending the docking report
(`task3_leaves.cpp:486`). **Nothing consumes RunStart, any Task 4 command,
`ocs_info` or `ocs_directive`.** The BT's task reports go to
`/crsd/docking_report`, `/crsd/firefighting_report` and
`/crsd/resource_delivery_request` (`bt_runner_node.cpp:249-252`), which
`ocs_client` does not subscribe to — so **no boat task report reaches
RoboCommand**.

### B1 — Typed topics out of `/crsd/ocs_command`

- **Behaviour change:** none by itself. A router splits the raw stream by
  top-level key into typed topics: `/crsd/ocs/run_start`, `/crsd/ocs/task4`
  (assistance_request, keep_out_zone, all_clear, moving_object_alert, with the
  command `seq`), `/crsd/course` (the catalogue §8.5 name) and
  `/crsd/ocs/directive`. `/crsd/ocs_command` stays as it is. Parsing is not
  acting.
- **Files:** `ocs_client_node.py` (or a new `ocs_router` node beside it); message
  types in `crusader_msgs` if typed, else JSON Strings.
- **Risk:** low. The JSON omits default fields (no `seq` = 0, no `vehicle_type` =
  TYPE_UNKNOWN); the router must treat a missing field as the zero value, not as
  an error.
- **Test:** `bench_stage2.py` B9/B10 still pass; `ros2 topic echo` each topic
  while `fake_course.py` and the inject buttons drive the OCS.

### B2 — Who acts on what (the mission executive, not the comms layer)

| Frame | Proposed consumer | Behaviour |
|---|---|---|
| `run_start` | BT runner | Logged and shown. Whether it gates leaving position-hold is **Chase's call**; if it does, a lost RunStart holds the boat, so it needs an operator override |
| `assistance_request` (surface or UNKNOWN) | mission executive | Decide to assist; send IncidentAck at once (`command_seq` = the command's `seq`), Readiness when loitering on station, then wait for ReadinessConfirm |
| `keep_out_zone` | Nav2 costmap (keep-out layer) + executive | Add the circle for its domain; send IncidentAck. Lasts the run or until AllClear |
| `all_clear` | same | Remove that domain's zones; send IncidentAck |
| `moving_object_alert` | Nav2 / tracker | A virtual moving obstacle with a 10 m standoff on its projected track (handbook 3.3.5). No acknowledgement exists |
| `ocs_info.course` | planners | Compare with the configured course; alarm on a mismatch |
| `ocs_directive` | BT runner | Display only |

- **Files:** `crusader_bt` (new leaves/conditions), the Nav2 costmap params YAML.
- **Risk:** this is what puts the network into the mission's decisions — every
  path must be gated on RC-armed AUTO and must fail to "keep doing what you were
  doing", never to a manoeuvre.
- **Test:** the Gazebo sim with `fake_course.py`; then the four Task 4 chains of
  handbook 3.4.12 against RoboNation's test server.

### B3 — Send task reports and Task 4 responses up

- **Behaviour change:** `ocs_client` subscribes to the three report topics above
  and forwards them as RxReport `docking`, `firefighting`, `resource_delivery`,
  plus new `incident_ack` and `readiness` from B2. The boat's Task 3 reports
  reach RoboCommand for the first time.
- **Files:** `ocs_client_node.py`, `crusader_params.yaml` (`ocs_client`).
- **Risk:** the governor allows 5 msg/s per vehicle with 2 reserved for
  heartbeats; reports are rare, but a retry loop must be bounded. The OCS refuses
  any UNKNOWN enum, so a report built from a half-known state is dropped, loudly.
- **Test:** `bench_stage2.py`-style check that a docking report appears as a `tx`
  line in `run/wire`; `bench_stage3.py` unchanged.

### B4 — Match ReadinessConfirm by report, not by substring

- **Behaviour change:** the substring test at `bt_runner_node.cpp:815` matches
  *any* frame containing `readiness_confirm` — a Task 4 ReadinessConfirm would
  stop the Task 3 docking re-send. Proposed: `ocs_link` puts its own increasing
  `seq` on every non-heartbeat report and remembers seq → kind; the bridge
  hands that seq back in `readiness_confirm.report_seq`
  (`OCS/rx_bridge/bridge.py:545-602`); the BT matches on it.
- **Files:** `ocs_link.py` (this file's framing is mirrored in the OCS — the
  `seq` field is not framing, so no OCS change), `ocs_client_node.py`,
  `bt_runner_node.cpp`.
- **Risk:** low; a vehicle that sends no seq gets the OCS's number back, as now.
- **Test:** inject a ReadinessConfirm naming the docking report's OCS seq; the
  boat receives its own seq. `tests/test_readiness_and_wirelog.py` covers the
  OCS half.

## Ekko

Today Ekko's companion (sysid 200, `uav_params.yaml:103`) sends boat_link TUNNELs
to target 0 and acts on exactly one inbound payload type, `PAYLOAD_BOAT`
(`Drone/rx26_uav/uav_fcu/uav_fcu/telemetry_bridge.py:553-571`); the OCS's RXL
types fall through that branch and are ignored. `ocs_client` republishes every
WiFi frame raw to `/uav/ocs_command` (`ocs_client_node.py:107`, `:170-173`), and
nothing reads that topic.

### E1 — Ekko's own RXL HEARTBEAT on the radio

- **Behaviour change:** the companion sends HEARTBEAT 0x10 (24 bytes, layout in
  EKKO_RADIO_LINK.md) at 2-4 Hz, built by the same rules as its WiFi heartbeat
  (`heartbeat_core.py`). Within 2 s of the first one the laptop's stand-in
  falls silent and its EXTENDED_SYS_STATE requests stop; 2 s after Ekko's stop,
  the stand-in resumes. A heartbeat with a clear valid flag or an UNKNOWN is
  **not forwarded and not replaced**: Ekko saying "I do not know" is relayed as
  silence. Then the reports 0x11-0x17 as each source exists (SAFE_PASSAGE from
  the buoy mapper first). INCIDENT_ACK and READINESS are ACK-retried: 2 Hz × 3,
  then 0.2 Hz until the OCS's ACK 0x38.
- **Addressing:** send uplink to `target_system 0`. Broadcast TUNNEL from the
  companion is measured to cross ArduPilot routing; a frame to 250 crosses only
  once the Cube has heard 250 on the radio port.
- **Files:** `telemetry_bridge.py` (TX at `:817`), a new
  `uav_common/rxl.py` — a **copy** of `OCS/fleet/rxl_codec.py`, as the framing is
  copied (a ROS package must not import from the OCS) — and `heartbeat_core.py`
  for the shared rules.
- **Risk:** 41 B per heartbeat frame: 164 B/s at 4 Hz on a mesh budgeted at
  ~2 kB/s. Payload types 0x8010-0x8038 do not collide with boat_link's
  0x8000-0x8006 / 0x80FE.
- **Test:** fleet_link + ocs_web on the laptop, Ekko on the bench: `/fleet`
  `ocs.vehicles.UAV1.heartbeat.source` reads `companion`; the page's radio line
  reads `fresh`; `run/tunnel/*.jsonl` shows decoded HEARTBEATs.

### E2 — Decode the downlink and ACK it

- **Behaviour change:** decode TUNNELs from sysid 250 addressed to 200 (or 0),
  ids 0x20-0x27, with the codec's length and enum checks; republish each as the
  **same JSON** the WiFi client receives, on `/uav/ocs_command`, so one consumer
  serves both links (catalogue §8.5: "kept as-is"). ACK RUN_START and ALL_CLEAR
  with `{msg_id, seq = the received frame's MAVLink seq, src = 0x1}` addressed to
  250; an ACK to ASSISTANCE_REQUEST, KEEP_OUT_ZONE or READINESS_CONFIRM is
  optional and honoured. Answer ASSISTANCE_REQUEST, KEEP_OUT_ZONE and ALL_CLEAR
  with INCIDENT_ACK quoting `cmd_seq`. Discard repeats by `(msg_id, payload)`
  over 30 s — every OCS retry has a new MAVLink seq. A READINESS_CONFIRM's
  `report_seq` is the `cmd_seq` of the READINESS it confirms. **Never act:**
  republish only, exactly as `ocs_client` does.
- **Files:** `telemetry_bridge.py` (the TUNNEL branch), `uav_common/rxl.py`,
  `ocs_client_node.py` or a small router.
- **Risk:** the RX loop has no per-frame guard today (a short boat frame can kill
  it — see the mismatch note below). Every decode must be wrapped; a bad frame is
  counted and dropped, never raised.
- **Test:** feed the frames `OCS/fleet/tests/test_ocs_adapter.py` builds into the
  decoder; on the bench, press the OCS inject buttons and watch
  `/uav/ocs_command` and `/fleet` `downlink.acks_matched`.

### E3 — Geofence and course: compare and ALARM, never upload

> **Changed 2026-10-10 (Chase):** the OCS now writes the declared geofence to Ekko's
> autopilot itself — `OCS/fleet/fence_upload.py`, `fence_upload = true` in
> `fleet.toml`, over the radio as sysid 250, with `uav_fcu`'s rules (disarmed only,
> no parameter written, read back within 1 m). So the *companion* still never
> uploads, but the FC fence is no longer only what `/uav/fence_upload` put there:
> `/uav/fence` will show the OCS-written polygon within 15 s, and the buoy search
> plans inside it. The comparison and alarm below still stand.

- **Behaviour change:** assemble GEOFENCE_INFO with the `GeofenceAssembler` rule
  (`OCS/fleet/rxl_codec.py:708-735`: parts keyed by `(fence_epoch, n_parts)`, a
  different epoch discards what is held, re-derive the epoch from the assembled
  polygon and refuse a mismatch). Compare the result, vertex by vertex at 1e-7°,
  with the declared fence in `uav_params.yaml` and the fence read back from the FC
  (`/uav/fence`). A mismatch raises an alarm on the GCS and in the log. It
  **never** uploads, edits or clears the FC fence. COURSE (0x26) likewise: check
  the fence lies inside the course boundary (handbook 3.4.10) and alarm if not.
- **Files:** `telemetry_bridge.py` or `preflight_core.py`, `gcs_node.py` for the
  alarm.
- **Risk:** false alarms from closure or vertex order — compare the closed,
  ordered vertex lists exactly as declared
  (`Drone/rx26_uav/tools/scripts/check_config.py` already keeps
  `uav_params.yaml` and `bridge.toml` equal, so a mismatch on the air is news).
- **Test:** feed `pack_geofence_parts(bridge.toml's fence)` → "match"; move one
  vertex 1 m → alarm; a 31-vertex fence in three parts out of order → assembled.

### E4 — EXTENDED_SYS_STATE on the radio

- **Behaviour change:** none if accepted as is. ArduCopter 4.7 sends 245 only
  when asked, per port; `telemetry_bridge` already asks on its own port, and the
  OCS asks on the radio port (1 Hz, ≈ 14 B/s) only while its stand-in speaks.
  Recommended: accept the OCS's request until E1 lands, after which it stops by
  itself.
- **Risk:** none new; the alternative (adding it to a stream group) is not
  available on 4.7.
- **Test:** `/fleet` `heartbeat.flight_state_request.state` reads "arriving".

### E5 — HAE from the receiver, not a constant

- **Behaviour change:** `altitude_hae_m` = GLOBAL_POSITION_INT.alt (EKF, AMSL) +
  the receiver's own separation, `GPS_RAW_INT.alt_ellipsoid − GPS_RAW_INT.alt`,
  instead of `geoid_separation_m = -24.6` (Sarasota: `uav_params.yaml:95`,
  `:188`; copied into `OCS/fleet/fleet.toml:120`). When the receiver does not
  report `alt_ellipsoid` (0), **no heartbeat** — not a fallback to the constant.
- **Files:** `heartbeat_core.py`, `telemetry_bridge.py` (subscribe GPS_RAW_INT's
  extension fields); the OCS stand-in would have to request GPS_RAW_INT on the
  radio the same way it requests EXTENDED_SYS_STATE.
- **Risk:** unverified whether ArduPilot fills `alt_ellipsoid` for Ekko's GPS —
  read it from a tlog before deciding.
- **Test:** a tlog: `alt_ellipsoid − alt` near the published geoid height at the
  test site.

## Both vehicles: `ocs_host` is hard-coded

> **Boat: implemented 2026-10-10, not yet deployed.** `ocs_host: "auto"` finds the OCS
> from its UDP beacon and falls back to `ocs_fallback_hosts`; see
> [VEHICLE_LINK.md](VEHICLE_LINK.md) "Discovery". **Ekko is unchanged** and everything
> below still describes it (the boat's YAML line has moved).

`ocs_host: "192.168.8.107"` in `Boat/rx26_asv/crusader_bringup/config/crusader_params.yaml:838`
and `Drone/rx26_uav/uav_bringup/config/uav_params.yaml:165`, both `[RO]`. There is
no discovery; a laptop that comes up on another address is silent on both
vehicles, and the symptom is "never heard".

- **Proposal:** (1) **no code** — a DHCP reservation for the OCS laptop's team
  NIC at 192.168.8.107 on the team router, written into the bring-up checklist;
  `bridge.toml:86-91` already asks for this. (2) **small code change** — let
  `ocs_host` be a comma-separated list of addresses or hostnames, tried in turn
  with the existing backoff, so a spare laptop can be listed second.
- **Files:** (2) both `ocs_link.py` (`_connect`) and both params YAMLs.
- **Risk:** (2) a dead first entry costs one connect timeout per cycle. `[RO]`
  stays: the address must not be changeable mid-run.
- **Test:** `bench_stage2.py` with `"10.255.255.1,127.0.0.1"`: connects to the
  second.

## Separate: the boat ⇄ Ekko TUNNEL formats do not match

Not part of the OCS link, and not redone here — see the memory note
`boat-drone-tunnel-format-mismatch.md` (checked 2026-10-08). In short: the only
frame both understand is the TEST text frame. Ekko sends boat_link v3
(POSITIONS 32772, LIGHTS 32773, GNSS 32774) and expects BOAT 32768 v3 from sysid
42; the boat acts only on native robotx 42011/42019, never sends BOAT, and its
`tunnel_link.py` is a frozen copy of Ekko's old v1. Ekko's RX loop can be killed
by a BOAT frame shorter than 14 B. A fix (boat adopts v3, B1-B6, plus a drone
length guard D1) awaits Chase and Dominick. Whatever is chosen, keep it out of
0x8010-0x8038, which the OCS link now uses — unless it deliberately adopts the
catalogue's own inter-vehicle ids (0x8030 PASSAGE_PLAN, ...), which the OCS
codec already names.

## Open decisions for Chase

1. **Geoid.** Keep a per-venue constant (and set it for San Diego and Singapore),
   or derive HAE from the receiver (E5)? The OCS stand-in copies whatever the
   aircraft uses.
2. **`current_task`.** The stand-in sends the literal TASK_NONE, mirroring
   `heartbeat_core.py:138`. Handbook 3.4.11: only a transition into a task value
   begins an attempt, so a heartbeat that stays TASK_NONE never starts one. Only
   Ekko can know its task; the fix is a
   task-state source on the aircraft, carried in its own heartbeat (E1).
3. **u16 on the radio.** `run_id`, `cmd_seq` and `report_seq` are u16 on the air
   and uint32 in RoboCommand; anything above 65535 is refused, loudly. Check what
   RoboNation's test server and demo actually issue. If they can exceed it,
   widen to u32 at both ends (+2 B a field, a catalogue change) or accept the
   refusal.
4. **Per-frame `synthesized`.** The hello marks a whole link. Once Ekko sends its
   own heartbeat on the radio, the bridge still treats it as synthesized. Fix in
   the bridge (a per-frame key stripped before `ParseDict`), or accept it.
5. **Where a ReadinessConfirm goes.** Today the best link at the moment; it could
   be pinned to the link that carried the READINESS, so the seq it carries is one
   that link's client knows.
6. **`phase_fallback`** (armed + height → flight phase) is off by default because
   the field feeds Network Remote ID. Keep it off?
