# The OCS — RobotX 2026 Operator Control Station

The OCS is the team's **single connection to RoboNation's RoboCommand**, and the
hub every message passes through. Vehicles never talk to RoboCommand: the OCS
publishes every report on their behalf and routes every command back to the
vehicle it names. Graey reaches it only through Crusader.

Python and a stdlib web page, no framework — the laptop at the dock needs nothing
installed that the bridge does not already need. Runs on Windows.

Message contract: **`docs/Fleet-Message-Contents.docx`** — field by field,
same tables as `Fleet-Message-Contents.xlsx`. Link budget:
**`docs/Fleet-Message-Bandwidth.docx`**.

## The one rule the whole design rests on

**Two networks, never bridged.** Team WiFi on one interface, course DHCP on the
other, `ip_forward=0` on every interface. Vehicles must not be able to reach
RoboCommand — verify *from a vehicle* that reaching the course network **fails**.
If this is wrong everything appears to work right up until the course.

Never set a static address on the RoboCommand-facing interface. The handbook
forbids it, and DHCP is what makes one config work on all four courses (Alpha
`192.168.65.2`, Bravo `.66.2`, Charlie `.67.2`, Delta `.68.2`). Nor is the
broker's address configured: `[robocommand] host = "auto"` finds whichever host
holds the course retained on `robocommand/robotx/course` (`rx_bridge/rcfind.py`;
the screening's was `192.168.69.3`, on none of those subnets).

## Running it

Double-click **`OCS.cmd`** (here, or the one-line twin at the top of
`RobotX_2026`): it starts `FLEET_LINK.cmd` and ocs_web in their own windows
unless they already answer, then opens `http://127.0.0.1:8099/map`. Arguments
pass through to ocs_web. By hand:

```
.venv\Scripts\python.exe -m ocs_web --config bridge.toml --inject --local-broker
```

`--inject` enables the fake-data buttons and puts an orange banner across the
page; without it they are gone and the endpoint returns 403. `--local-broker`
defaults to **11883, not 1883** — Windows reserves 1883 in the Hyper-V range and
the bind fails with `WinError 10013`.

`--bench-auto USV1,UAV1` is the bench dry run: those vehicles' `STATE_MANUAL`
heartbeats are published as `STATE_AUTO`, so RoboCommand starts a run with the
vehicles disarmed. Command line only, orange banner on both pages, never at
competition — `docs/TESTING.md` "Bench dry run".

`start_ekko_ocs.py` (`.venv`) is the Ekko-only start from the 7-8 Nov
assessment: it stops both, then starts fleet_link on the radio alone (COM28, no
WiFi) and ocs_web on `bridge.toml`. With ocs_web stopped, `--find` runs one
search for RoboCommand and prints every host that answered and what it was.

Ports: **37564** vehicle link (TCP) · **37565** discovery beacon (UDP, OCS to the boat, broadcast — never on the course network; `docs/VEHICLE_LINK.md` "Discovery") · **8099** the page · **11883** dev broker · **1883** RoboCommand's broker (TCP, on the host the search finds).

## Before you commit — five gates, in order

| | Command | Expect |
|---|---|---|
| 1 | `.venv\Scripts\python.exe -m unittest discover -s tests -t .` | `Ran 610 tests`, `OK` |
| 2 | `python -m unittest discover -s fleet/tests -t .` — **system** python | `Ran 533 tests`, `OK` — fleet_link, the radio adapter, the RXL codec, the fence read and upload, `--sim-boat` |
| 3 | `.venv\Scripts\python.exe bench_stage1.py` | `21/21` — the RoboCommand sequence, over MQTT |
| 4 | `.venv\Scripts\python.exe bench_stage2.py` | `22/22` — the TCP vehicle link, with receipts, tiers from a vehicle and run_end |
| 5 | `.venv\Scripts\python.exe bench_stage3.py` | `25/25` — the two-vehicle Proof of Readiness rehearsal, with the UAV fence read off the aircraft |

Gates 1 and 3-5 run on `.venv`; gate 2 needs `pymavlink` and `pyserial`, which
live only in the system Python 3.13.

Stages 2 and 3 **import the boat's own `ocs_link.py`** out of `../Boat/rx26_asv`
rather than a stand-in. That is the point: the frame format is duplicated across
the two repos by necessity — a ROS package must not import from here — so the one
failure worth catching is those two copies drifting apart. A bench speaking our
own framing back to itself would agree with itself perfectly while the boat sent
garbage. Set `RX26_ASV` if the checkout is somewhere else.

Procedure, level by level: `docs/TESTING.md`.

## The hot path, and why it is in that order

    vehicle report → [TCP] link preference → staleness → governor → validate
                   → sequence number → re-stamp sent_at from OUR clock → publish

Two orderings are load-bearing. **The governor runs before the sequence number**,
so a dropped frame does not burn a seq and leave a hole that looks like a lost
message. **`sent_at` is re-stamped last**, so a vehicle with a skewed clock
cannot corrupt the one field RoboCommand judges us on — the vehicle's own
timestamp survives only as the staleness measurement.

Everything after the transport is deliberately transport-agnostic. That is why
adding the TCP link did not touch `governor.py`, `seqstore.py` or `validate.py`.
The one transport-aware rule, **the link preference**, runs first for the
governor's reason: Ekko can be on the radio and WiFi at once, and a duplicate
must neither reach the governor (a doubled rate) nor take a seq. It is per
report kind — real beats synthesized, then `prefer`, and a kind only one link
carries is never dropped. `docs/VEHICLE_LINK.md`.

## Never invent a value

The umbrella file's "blanks over guesses" is not advisory here. A gap reads on
the page as rising silence; a fabricated value is indistinguishable from a real
one and scores as though the vehicle were somewhere, or in a state, it is not.
When something cannot be known truthfully the answer is to publish **nothing** —
and `validate.py` refuses NaN and any `*_UNKNOWN` enum on the way out.

## Traps that have already cost time

**The `robocommand` clone at `GitHub/robocommand` is stale** — dated 2025-12, with
no `RobotX_2026/` directory at all. The schema lives upstream at
`RobotX_2026/proto`, revision `8fc09f2` (adopted 2026-10-31; was `f6457fa`), which
is exactly what `rx_bridge/gen/REVISION` pins. Read upstream; never that clone.

**RoboCommand refuses a declaration with an aircraft SILENTLY.** No reply on MQTT:
the run just never starts, which looks exactly like the "declared a vehicle that is
not on the water" hang below. Since 8fc09f2 it refuses a UAV fence that leaves the
course and its `uav_transit_area` by more than 1 m, fewer than 4 points, and a
ceiling (`uav_geofence_max_altitude_hae_m`) that is unset or outside 0–50 m HAE.
Both are read off Ekko, never typed: the polygon is the pilot's, drawn in QGC
inside RoboCommand's, and the ceiling is home HAE + FENCE_ALT_MAX.
The bridge applies the same rule first (`geofence.rc_declaration_problems`, a
transcription of their test server's) and refuses in words on the page; their
test server's log is the only other place a refusal shows. On the sim box,
`rc_watch.py` turns that log line into a red card (`docs/TESTING.md` 4e).

**`run/` is durable state and is gitignored.** `run/seq.json` holds the sequence
counters *and* the run itself, so a crash mid-run resumes rather than rewinding.
Committing it would let two machines think they own the same counters. Deleting
it is the only thing that rewinds sequence numbers — re-declaring does not, on
purpose: two declarations carrying `seq=1` would let a stale RunStart answering
the first run match the second.

**A flapping RoboCommand link is a second OCS, not the network.** Our MQTT client
id is stable by design — it is what lets the broker keep our session across a drop
— so a second client using the same id takes the session and the two kick each
other off forever. After three drops in 30 s the log says `DUPLICATE OCS?`. The
handbook permits the team one connection, so the fight is the rule being enforced.
The broker search is not a second OCS: it probes only while the bridge is not
connected, under a client id of its own (`ocs-<team>-find-…`, clean session).

**Declaring a vehicle that is not on the water hangs the run.** RoboCommand
publishes RunStart only once *every* declared vehicle reports `STATE_AUTO`, with
no timeout and no error, so the symptom is indistinguishable from a broken link to
their server. Only vehicles that are actually reporting go into the declaration,
and handbook §3.1 requires at least two.

**`protobuf` 7.x and `paho` 2.x are hard requirements**, not preferences. The
schemas declare `edition = "2024"`, which a 5.x runtime cannot load at all — the
import fails outright rather than degrading — and the bridge uses paho's VERSION2
callback signatures, which do not exist in 1.x.

**Never edit `rx_bridge/gen/`.** Generated, committed, and pinned by `REVISION`.
Regenerate with `make_protos.sh` (Git Bash) only when deliberately adopting a
newer robocommand revision, and read the diff when you do.

## Where things are

| | |
|---|---|
| `rx_bridge/bridge.py` | the two connections and the rules between them |
| `rx_bridge/runstate.py` | the run state machine — no I/O, all of the ordering rules |
| `rx_bridge/active.py` | which vehicles go into the declaration |
| `rx_bridge/geofence.py` | RoboCommand's UAV polygon for the course in hand (put on the aircraft for the pilot to draw inside), and what a declaration with an aircraft carries: the pilot's polygon and ceiling, read off its autopilot, or why not — pure |
| `rx_bridge/rcfind.py` | finds RoboCommand's broker by the course it keeps retained (`host = "auto"`): last host, gateways, .2/.3, then a /24 sweep; again after 30 s down; its probes never overlap the bridge's connection |
| `rx_bridge/operatorstore.py` | the two values no vehicle reports, the pilot call sign and each aircraft's registration: typed on the page > this laptop's saved default (`run/operator.json`) > `bridge.toml` |
| `rx_bridge/framing.py` | the vehicle link's frame boundaries — **mirrored in both vehicles' `ocs_link.py`** |
| `rx_bridge/beacon.py` | the discovery beacon: which laptop addresses it broadcasts from (never the RoboCommand broker's /24 or the address that reaches it, and none on a /24 not yet searched for it) and what the boat does with it |
| `rx_bridge/linkpolicy.py` | which of a vehicle's two links the OCS listens to (per kind) and talks on (one, never both) — pure |
| `rx_bridge/validate.py` | the NaN and UNKNOWN nets |
| `rx_bridge/benchauto.py` | the `--bench-auto` rewrite (MANUAL to AUTO, listed vehicles only, never KILLED) — the one deliberate exception to "never invent a value" |
| `ocs_web/` | the operator page and `/map`, the fleet map (course, RoboCommand's UAV polygon, the fence read off each aircraft and the polygon a declaration carries; `/api/fleet` proxies fleet_link) |
| `fleet/` | `fleet_link`: owns the laptop's MAVLink inputs (WiFi UDP 14550, RFD900ux COM8) and fans them out (`127.0.0.1:8098/fleet`); the **OCS adapter** (Ekko's RXL TUNNELs ⇄ bridge as **sysid 250**), which also **reads Ekko's fence** for the declaration and **writes RoboCommand's polygon onto it**, disarmed only and never over the pilot's; `--sim-boat` is simulation only. **System python, not `.venv`**; start with `FLEET_LINK.cmd`; `fleet/README.md` |
| `bridge.toml` | the fleet, the brokers (`[robocommand] host = "auto"`), the tiers, the geofence margin, the pilot call sign and registration defaults, `prefer` and `link_prefer_s` |
| `rc_watch.py`, `rc_box_tests.py` | **runs on the sim box, not here**: a live page (`:8088`) of everything that crossed RoboNation's broker, refusals from their log, and narrated tests (`docs/TESTING.md` 4e) |
| `rc_e2e.py` | runs **here**: our OCS (isolated copy) + the boat's `ocs_link.py` against the sim box's test server, narrated on the box's page over ssh (`docs/TESTING.md` 4e) |
| `docs/VEHICLE_LINK.md` | the TCP 37564 contract: framing, `ocs_hello`, the link preference, `ocs_info`, the ReadinessConfirm rewrite |
| `docs/EKKO_RADIO_LINK.md` | the radio side: every RXL message and byte, repeat/ACK policies, the stand-in heartbeat, every deviation from the catalogue |
| `docs/VEHICLE_SIDE_PROPOSAL.md` | what the boat and Ekko would have to change — a proposal, nothing applied |
| `docs/` | also: testing procedure, the message contract, open questions for RoboNation |

## Still open

`docs/ROBONATION_QUESTIONS.md` holds the unanswered ones. **Q1 blocks the UUV
reporter** — a 9600-baud acoustic link cannot carry a 2 Hz heartbeat, and the
handbook never says what a submerged vehicle owes. Do not design that half until
RoboNation answers.
