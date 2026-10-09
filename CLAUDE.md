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
`192.168.65.2`, Bravo `.66.2`, Charlie `.67.2`, Delta `.68.2`).

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

Ports: **37564** vehicle link · **8099** the page · **11883** dev broker.

## Before you commit — four gates, in order

| | Command | Expect |
|---|---|---|
| 1 | `python -m unittest discover -s tests -t .` | `Ran 121 tests`, `OK` |
| 2 | `python bench_stage1.py` | `21/21` — the RoboCommand sequence, over MQTT |
| 3 | `python bench_stage2.py` | `16/16` — the TCP vehicle link |
| 4 | `python bench_stage3.py` | `21/21` — the two-vehicle Proof of Readiness rehearsal |

Stages 2 and 3 **import the boat's own `ocs_link.py`** out of `../Boat/rx26_asv`
rather than a stand-in. That is the point: the frame format is duplicated across
the two repos by necessity — a ROS package must not import from here — so the one
failure worth catching is those two copies drifting apart. A bench speaking our
own framing back to itself would agree with itself perfectly while the boat sent
garbage. Set `RX26_ASV` if the checkout is somewhere else.

Procedure, level by level: `docs/TESTING.md`.

## The hot path, and why it is in that order

    vehicle report → staleness → governor → validate → sequence number
                   → re-stamp sent_at from OUR clock → publish

Two orderings are load-bearing. **The governor runs before the sequence number**,
so a dropped frame does not burn a seq and leave a hole that looks like a lost
message. **`sent_at` is re-stamped last**, so a vehicle with a skewed clock
cannot corrupt the one field RoboCommand judges us on — the vehicle's own
timestamp survives only as the staleness measurement.

Everything after the transport is deliberately transport-agnostic. That is why
adding the TCP link did not touch `governor.py`, `seqstore.py` or `validate.py`.

## Never invent a value

The umbrella file's "blanks over guesses" is not advisory here. A gap reads on
the page as rising silence; a fabricated value is indistinguishable from a real
one and scores as though the vehicle were somewhere, or in a state, it is not.
When something cannot be known truthfully the answer is to publish **nothing** —
and `validate.py` refuses NaN and any `*_UNKNOWN` enum on the way out.

## Traps that have already cost time

**The `robocommand` clone at `GitHub/robocommand` is stale** — dated 2025-12, with
no `RobotX_2026/` directory at all. The schema lives upstream at
`RobotX_2026/proto`, revision `f6457fa`, which is exactly what
`rx_bridge/gen/REVISION` pins. Read upstream; never that clone.

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
| `rx_bridge/framing.py` | the vehicle link's frame boundaries — **mirrored in `rx26_asv`** |
| `rx_bridge/validate.py` | the NaN and UNKNOWN nets |
| `ocs_web/` | the operator page, and `/map` — the fleet map (Leaflet vendored in `ocs_web/vendor/`, routes in `maproutes.py`; `--fleet-url` points its `/api/fleet` proxy at fleet_link) |
| `fleet/` | `fleet_link`: owns the laptop's two MAVLink inputs (WiFi UDP 14550, RFD900ux COM8) and fans them out to loopback consumers; `GET 127.0.0.1:8098/fleet`. **System python, not `.venv`**; start with `FLEET_LINK.cmd`; see `fleet/README.md` |
| `bridge.toml` | the fleet, the brokers, the tiers, the geofence |
| `docs/` | testing procedure, the message contract, open questions for RoboNation |

## Still open

`docs/ROBONATION_QUESTIONS.md` holds the unanswered ones. **Q1 blocks the UUV
reporter** — a 9600-baud acoustic link cannot carry a 2 Hz heartbeat, and the
handbook never says what a submerged vehicle owes. Do not design that half until
RoboNation answers.
