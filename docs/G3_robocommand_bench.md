# Gate G3 — RoboCommand link bench validation

**Purpose:** prove the OCS can complete the handbook's start-of-run sequence and hold a
2 Hz USV heartbeat against a RoboCommand broker — before any of it is trusted on the
water, and before the Jetson is involved at all.

**USV only.** One vehicle, one sequence counter. Adding the UAV and UUV is config, not
code, and is out of scope here.

**Nothing in this gate can move the boat.** Part A and B use `fake_vehicle.py`; Part C
adds the real Jetson but only as a *publisher*. The RC e-stop remains the only safety
path throughout, exactly as on every other gate.

## The sequence you are proving

```
subscribe -> retained RxCourse -> RunDeclaration -> heartbeats -> RunStart(declaration_seq)
```

Heartbeats begin **after** the declaration and **before** `RunStart`. Read past that and
you sit silent through the window RoboCommand is watching to confirm the team is alive.

## Which machine runs what

Every script has exactly one home. Only two of your three machines are involved
until `robocommand_reporter` exists.

| Script | Runs on | Stands in for |
|---|---|---|
| `rx_bridge` | **host computer**, always, never anywhere else | this *is* the OCS |
| `bench_stage1.py` | host computer | everything at once, for Part A |
| `fake_vehicle.py` | host computer, for now | the Jetson |
| `fake_course.py` | host computer | RoboCommand, when the stub sends no course |
| `docker compose` (RoboNation stub) | sim box — or the host, in Part A | RoboCommand |
| `mosquitto` (team broker) | host computer | the team radio network |

| Stage | Jetson | Host computer | RoboNation server |
|---|---|---|---|
| **A** | *off* | broker, `rx_bridge`, fake USV | *off* |
| **B** | *off* (used only for the B4 ping test) | mosquitto, `rx_bridge`, fake USV | docker stub + DHCP |
| **C** | `robocommand_reporter` | mosquitto, `rx_bridge` | docker stub + DHCP |

`rx_bridge` never moves. It is the only process permitted to talk to RoboNation.
Everything else is a stand-in that gets deleted later, or a broker.

## Prerequisites

- [ ] `ocs/.venv` built: `python -m venv .venv && .venv/Scripts/python -m pip install -r requirements.txt`
      (Linux/macOS: `.venv/bin/python`)
- [ ] `bash ocs/make_protos.sh` run; `ocs/rx_bridge/gen/` populated
- [ ] `.venv/Scripts/python -m unittest discover -s tests -t .` → **41 tests OK**
- [ ] Docker available for RoboNation's stub
- [ ] `mosquitto-clients` installed (`mosquitto_sub`) — the single most useful
      diagnostic in this document
- [ ] Team ID agreed and identical in `bridge.toml` and every `--team` flag

---

## Part A — one laptop, no network (start here)

### A0 — the whole matrix in one command

```bash
cd ocs && .venv/Scripts/python bench_stage1.py
```

Runs an MQTT broker in-process, plays both RoboCommand and the USV, and checks
every criterion below. No docker, no mosquitto, no network. Expect **15/15**.

Do this first. It cannot tell you whether we agree with *RoboNation* about the
bytes — only their stub can — but it tells you in thirty seconds whether the
bridge's own logic is sound, and it is the thing to re-run after every change.

The manual walkthrough below is still worth doing once, because at a competition
you will be driving the bridge by hand and it should not be the first time.


Everything on loopback against one broker. Two topic namespaces, two MQTT clients,
zero networking questions. If Part A does not pass, no amount of cabling will help.

**Terminal 1 — RoboNation's stub.** We use theirs, not a fake of ours; `test_client.py`
alongside it is a reference OCS to diff against.

```bash
git clone https://github.com/robonation/robocommand
cd robocommand/RobotX_2026 && docker compose up --build
```

**Terminal 2 — watch the wire.** Leave this running for the whole gate.

```bash
mosquitto_sub -h 127.0.0.1 -t 'robocommand/robotx/#' -v
```

**Terminal 3 — the bridge.** Set both `team.host` and `robocommand.host` to `127.0.0.1`
in `bridge.toml` for Part A only.

```bash
cd ocs && .venv/Scripts/python -m rx_bridge --config bridge.toml
```

**Terminal 4 — a USV that does not exist.**

```bash
cd ocs && .venv/Scripts/python fake_vehicle.py --host 127.0.0.1
```

> **Their stub is more active than a log sink.** `test_server.py` publishes the retained
> `RxCourse` on startup and then *auto-issues `RunStart`* the moment every vehicle in your
> `RunDeclaration` reports `STATE_AUTO`. You do not send `RunStart` by hand. This is a
> gift: it means the bench also proves your heartbeat carries the right `state` and the
> right `vehicle_id`, because a mismatch in either silently prevents the run from starting.
> `test_client.py` is still there for injecting Task 4 traffic and malformed frames.

### Test matrix A (all must pass; record pass/fail + timestamp)

| # | Test | Procedure | Pass criterion |
|---|------|-----------|----------------|
| A1 | Connect | Start the bridge | Logs `RoboCommand: connected`; `status` shows `CONNECTED` |
| A2 | Course | Watch for the retained course | State becomes `COURSE_RX`; course id and pinger frequency logged. If it never arrives, see T6 |
| A3 | Declare refused early | Type `declare` **before** the course arrives | Refused with `need the retained RxCourse first`; state unchanged |
| A4 | Silence before declaring | Run `fake_vehicle.py` while in `COURSE_RX`; `status` | `forwarded 0`. Pre-declaration silence is correct, not a fault |
| A5 | Declare | Type `declare` | State `DECLARED`; `declaration_seq=1`; terminal 2 shows a frame on `.../request` |
| A6 | Heartbeats flow | `status` after 10 s | `forwarded` climbing ~2/s; terminal 2 shows `.../USV1/report` |
| A7 | Rate is right | `status` twice, 30 s apart | `forwarded` delta ≈ 60 (±3). Not 30, not 120 |
| A8 | NaN is caught | Restart the fake with `--nan` | `DROP ... heading_deg: NaN`; `dropped ... invalid` climbing; **`forwarded` stops** |
| A9 | UNKNOWN is caught | Restart with `--unknown` | `DROP ... current_task: UNKNOWN (TASK_UNKNOWN...)` |
| A10 | Run start | **Nothing — it is automatic.** Their `test_server.py` publishes `RunStart` as soon as every vehicle named in your `RunDeclaration` is reporting `STATE_AUTO` | `RUN START: run N started`; state `RUNNING`. If it never fires, your heartbeat's `state` is not `STATE_AUTO`, or a `vehicle_id` disagrees with the declaration |
| A11 | Wrong seq refused | Send `RunStart` with a different `declaration_seq` | `RUN START REFUSED: ... does not match ours`; state stays put |
| A12 | Run survives a restart | Note `seq` and the state, Ctrl-C the bridge, restart it, `status` | Resumes `DECLARED`/`RUNNING` on connect; report seq continues, never restarts at 1; `declare` is **not** needed and is refused |
| A13 | Wire log | `tail` the file named in `status` | One JSON object per frame, `b64` payloads present |

> **A12 note.** `run/seq.json` holds the run itself — state, `declaration_seq`, `run_id` —
> not only the counters. Durable counters alone are worthless: a bridge that came back
> with correct sequence numbers and no memory of having declared would sit in `COURSE_RX`
> unable to publish, and the only way out would be to declare again, which mints a new
> `RxRequest.seq` and orphans the `RunStart` it is waiting for. To start a genuinely new
> run, delete `ocs/run/seq.json` and declare again.
>
> This was found by `bench_stage1.py` check A12b, which failed the first time it ran.

---

## Part B — two machines, the real segment

Only after A passes. This is where the network question gets settled.

```
   sim box  192.168.65.2  ── straight ethernet ──  OCS laptop  ── wifi ──  team router
   (docker stub + dnsmasq)                          (rx_bridge)             192.168.8.0/24
```

**The sim box goes on neither the LAN nor the WAN port of the team router.** LAN puts it
beside the Jetson, so the boat reaches the broker directly — the prohibited topology, and
it will appear to work. WAN is the RoboBoat arrangement and is worse: the router NATs the
boat out to it, so vehicles still reach RoboCommand, now through a layer that hides it.

Serve DHCP from the sim box (`dnsmasq`, handing out `192.168.65.100–200`), because the
handbook forbids a static address on the RoboCommand-facing interface and you need that
exercised. On Windows, use a second router for the course segment instead — a separate
broadcast domain is what matters, not which box serves DHCP.

Then set `robocommand.host = "192.168.65.2"` and `team.host` to the OCS's own team-network
address in `bridge.toml`.

### Test matrix B

| # | Test | Procedure | Pass criterion |
|---|------|-----------|----------------|
| B1 | DHCP | `ip addr` / `ipconfig` on the OCS ethernet | Address in `192.168.65.0/24`, **leased not static** |
| B2 | Both networks live | Ping `192.168.65.2` and the team router | Both reply |
| B3 | Forwarding off | `sysctl net.ipv4.ip_forward` (Linux) / routing config | `0` |
| B4 | **The compliance test** | From the **Jetson**: `ping 192.168.65.2` | **Must FAIL.** If it succeeds you are non-compliant — see T13 |
| B5 | Jetson cannot see the broker | From the Jetson: `mosquitto_sub -h 192.168.65.2 -t '#' -W 5` | Times out |
| B6 | Full sequence over the wire | Repeat A5–A10 with the split hosts | Same criteria |
| B7 | Reconnect | Unplug the ethernet 30 s, replug | Resubscribes, resumes reporting, **does not re-declare**, seq continues |
| B8 | Clock | `chronyc tracking` on the OCS | Synced; no `STALE` lines in the bridge log |

---

## Part C — the real Jetson

Blocked until `crusader_comms/robocommand_reporter` exists. When it does, the only change
is swapping `fake_vehicle.py` for the Jetson; every criterion above still applies, plus:

| # | Test | Pass criterion |
|---|------|----------------|
| C1 | Heading substitution | With GPS yaw unresolved, `heading_deg` carries EKF yaw and the substitution is logged — **never** NaN, never a frozen last value |
| C2 | `altitude_hae_m` | Real value, not `0.0` (needs the `LatLonHead` change) |
| C3 | `state` mapping | `STATE_MANUAL` / `STATE_AUTO` track the FCU mode; kill state from RC ch5 |
| C4 | Radio dropout | 60 s of WiFi loss changes **no** vehicle behaviour |

---

## Troubleshooting

### Setup and toolchain

| # | Symptom | Cause and fix |
|---|---------|---------------|
| T1 | `Edition 2024 is later than the maximum supported edition 2023` | Your `protoc` is too old. The schemas are Edition 2024, not proto3. Needs libprotoc ≥ 30; Ubuntu 22.04 ships 3.12 and `grpcio-tools` 1.71 ships 29.0. Build the venv from `requirements.txt` — `make_protos.sh` prefers its `protoc` over any system one |
| T2 | `ImportError: no generated protobuf at .../gen` | `bash ocs/make_protos.sh` was never run |
| T3 | `ModuleNotFoundError: No module named 'paho'` | You are on system Python. Use `.venv/Scripts/python` (or `.venv/bin/python`) |
| T4 | `TypeError` / descriptor errors on import | `protobuf` 5.x cannot load Edition 2024 descriptors at all. `pip install -r requirements.txt` (pins 7.36.0) |
| T5 | `AttributeError: 'FieldDescriptor' has no attribute 'label'` | Old code against protobuf 7. Should not happen — `validate.py` uses `is_repeated`. If you see it, something outside this package is stale |

### The bridge will not advance

| # | Symptom | Cause and fix |
|---|---------|---------------|
| T6 | Stuck in `CONNECTED`, never `COURSE_RX` | No retained `RxCourse`. Their `test_server.py` *does* publish one on startup, so suspect the `rc-test` container rather than the protocol: `docker compose logs rc-test` should show `Published RxCourse (retained)`. Confirm with `mosquitto_sub -h HOST -t 'robocommand/robotx/course' -v` — a fresh subscriber getting nothing means no retained message exists. If you are running without their stub at all, `python fake_course.py --host HOST` supplies one |
| T7 | Stuck in `DISCONNECTED` | Broker unreachable. `docker compose ps` — is it up and is 1883 published to the host? Try `mosquitto_sub -h HOST -p 1883 -t '#'`. A broker bound to `127.0.0.1` inside the container is invisible from another machine |
| T8 | `cannot declare in CONNECTED` | You have no course yet. See T6 |
| T9 | `cannot declare in DECLARED` | Already declared. To start a new run, delete `ocs/run/seq.json` and restart the bridge |
| T9b | `DECLARED`, every vehicle AUTO, RunStart never comes, UAV1 declared | Their test server (8fc09f2) refused the declaration and **sent nothing back**. Its log has `[ERROR] .../request: RunDeclaration for ASTA refused: <why>`: a fence outside Course Alpha + Transit A by more than 1 m, fewer than 4 points, or a ceiling unset or outside 0–50 m HAE (`MAX_UAV_ALTITUDE_HAE_M`). The OCS applies the same rule before declaring, so this means the OCS is older than 8fc09f2 or the rule moved upstream. `rc_watch` on the box shows it as a red card (TESTING.md 4e). Since 8fc09f2 RoboCommand may also hold RunStart for Garuda's clearance and the course manager: their test server does not |

### Heartbeats are not arriving

| # | Symptom | Cause and fix |
|---|---------|---------------|
| T10 | `forwarded 0`, fake vehicle running, no errors | **Check the state first.** Reports are dropped silently before `DECLARED` — that is correct behaviour, not a bug. Type `declare` |
| T11 | `DROP: USV1 is not in vehicle_ids` | `fake_vehicle.py --vehicle` disagrees with `bridge.toml`'s `vehicle_ids` |
| T12 | `DROP: topic says X, message says Y` | `--team` or `--vehicle` mismatch. The handbook requires the topic's vehicle id to match the message's, so the bridge refuses rather than guessing |
| T13 | `forwarded` climbing at ~5/s not 2/s, or `dropped ... rate` climbing | `--rate` on the fake is wrong, or something else is publishing to the same topic. Check with `mosquitto_sub -t 'team/robotx/#' -v` |
| T14 | `STALE USV1: NNN ms old` | Clock skew between the publisher and the OCS, not a slow link — `sent_at` is measured against the *vehicle's* clock. Run chrony/NTP on both. The frame is still forwarded; only the warning is new |

### Network and compliance

| # | Symptom | Cause and fix |
|---|---------|---------------|
| T15 | **Jetson CAN ping `192.168.65.2`** | You are non-compliant. Either the sim box is on the team router (move it to its own segment), or the OCS is forwarding (`sysctl -w net.ipv4.ip_forward=0`, and check for an enabled ICS / bridge on Windows) |
| T16 | No DHCP lease on the OCS ethernet | `dnsmasq` not bound to the right interface, or a host firewall is dropping DHCP. Confirm with `journalctl -u dnsmasq`. Do **not** work around this with a static address — DHCP is what the handbook requires and what you are testing |
| T17 | Team link dies when the ethernet is plugged in | The course DHCP pushed a default route that beat your wifi. Both subnets are directly connected so traffic still works; if it does not, set a higher metric on the ethernet interface. Never solve this by bridging the two |
| T18 | Connection refused from another machine, fine on loopback | Broker listening on loopback only, or a host firewall. On Windows, allow inbound 1883 for the team broker |
| T19 | Bridge reconnects but receives nothing | Subscriptions not restored. `bridge.py` re-subscribes on every CONNACK, so suspect the broker: check it accepted a persistent session with the same client id (`ocs-<team_id>`) |

### Sequence numbers and run start

| # | Symptom | Cause and fix |
|---|---------|---------------|
| T20 | `RUN START REFUSED: ... does not match ours` | Working as intended. The stub answered a declaration that is not the one you hold — usually a leftover from an earlier `declare`. Restart the stub, delete `ocs/run/seq.json`, restart the bridge, declare once |
| T21 | Report seq restarted at 1 mid-run | `ocs/run/seq.json` was deleted or the epoch changed. On RoboCommand's side this is indistinguishable from a replayed stream |
| T22 | Seq has gaps | Expected if frames were rate-dropped or rejected — the governor runs *before* the counter, so a dropped frame does not burn a number. Persistent gaps mean the validator is rejecting; read the `DROP` lines |

### Best diagnostic when nothing above fits

```bash
mosquitto_sub -h HOST -t '#' -v          # everything, both namespaces
python -c "import json,sys; [print(json.loads(l)) for l in open(sys.argv[1])]" ocs/run/wire/<file>.jsonl
```

The wire log holds raw payloads, so a frame captured once can be replayed into the stub.

## Sign-off

| Part | Result | Date | Signed |
|---|---|---|---|
| A — one laptop | | | |
| B — two machines | | | |
| C — real Jetson | blocked on `robocommand_reporter` | | |

## Open questions for RoboNation

Answer these before the gate is considered closed:

1. Is the 5 msg/s cap **per vehicle or per team**? Read as per-team, three vehicles at
   the mandated 2 Hz breach it on heartbeats alone (6 > 5). We meter per vehicle.
2. Does their broker speak **MQTT 3.1.1 or 5**? Decides whether heartbeat QoS can use a
   message expiry interval.
3. Is `FLIGHT_PHASE_UNKNOWN` **accepted for a surface vehicle**? There is no
   "not applicable" value in the enum.
4. What should `state` reflect when the **physical e-stop** is pressed? It cuts the
   relay coil directly and is invisible to software, so the USV would keep reporting
   `STATE_AUTO` while dead in the water.
