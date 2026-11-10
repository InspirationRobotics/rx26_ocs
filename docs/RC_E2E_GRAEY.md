# Shared Crusader / Ekko / GRAEY communications test

`rc_e2e.py` now includes GRAEY as **UUV1**, alongside USV1 and UAV1 in the
existing Crusader harness. It runs the shared OCS unchanged. No second OCS
implementation or vehicle protocol is introduced.

The UUV fixture imports GRAEY's actual `CompetitionTelemetry`,
`CompetitionSender` and `WifiLink`. Synthetic Cube GLOBAL_POSITION_INT and
ATTITUDE, control state and depth/surface samples enter the telemetry builder.
Reports then use the production encoder, SQLite outbox, TCP framing, reconnect
and receipt handling. No ROS, Cube serial port or physical vehicle is accessed.
USV1 and UAV1 continue using Crusader's actual `OcsLink`; UAV1 retains the
existing simulated Ekko fence read-back.

## Run it

Use the OCS Python environment with its normal dependencies (including amqtt).
Check out [RoboNation robocommand](https://github.com/robonation/robocommand),
revision `8fc09f295fe266ad8106cc0344a3e4d57f844359`, matching the OCS schema.
From this repository, in PowerShell:

```powershell
$env:RX26_ASV = 'C:\path\to\crusader'
$env:RX26_GRAEY = 'C:\path\to\robotx_graey_2026'
.venv\Scripts\python.exe rc_e2e.py --local-rc 'C:\path\to\robocommand\RobotX_2026' --pause 0
```

`--local-rc` runs RoboNation's unchanged server and client with an isolated
local amqtt broker (the broker used by Crusader's local benches). It runs the
same `rc_watch.py`, test actions and verdicts as the remote sim-box workflow.
Without that option, the existing SSH relay and official Docker test client
on `inspiration@192.168.65.2` are used.

Default isolated ports: OCS HTTP **18099**, vehicle TCP **38564**, team MQTT
**12883**, official-server MQTT **12884**, watcher HTTP **18088**. Each has a
CLI override. The vehicle listener binds loopback; discovery is disabled;
the OCS fleet proxy is disabled by pointing it at loopback port 1. Existing
OCS8099, fleet8098, TCP37564 and live vehicle services are not used.
Run and sequence state, outbox, logs and wire captures live in a new temporary
directory printed at exit. `results.json` contains all verdicts, OCS and
watcher snapshots, and GRAEY's final outbox/link state.

## Checks

| Steps | Evidence |
|---|---|
| E1 | Course, UAV fence read-back, all three vehicle identities active |
| E2 | Empty UAV pilot call sign refused; no declaration reaches broker |
| E3–E4 | Official server accepts declaration/fence; RunStart reaches OCS and GRAEY with matching run ID; all three declared |
| E5 | All three heartbeat streams observed at RC broker, using Crusader's existing 1.5 Hz minimum for the nominal 2 Hz stream |
| E6–E7 | Crusader docking report reaches broker and gets PUBACK |
| E8–E9 | Official client alert reaches all three clients with 7 m keep-out radius intact |
| E10 | GRAEY survey persists pending while submerged; no survey appears at broker for six seconds |
| E11–E12 | GRAEY socket reconnects, receives run context, surfaces; ordered intact/damaged/intact survey arrives and PUBACK clears its pending outbox |
| E13 | Shared GUI End API reaches GRAEY; ENDED clears run confirmation |

GRAEY adaptations are limited to its production telemetry source, UUV identity,
depth/surface state, pipeline-survey payload and durable surfaced-report policy.
Task 2 is declared CORE in the temporary test configuration. The committed
operator configuration is not changed. Synthetic AUTO state is fixture data;
no arming or mission execution occurs.

This validates the application communications chain against the official test
server, not real Wi-Fi range, RF, underwater transmission, actual mission
execution, physical sensor accuracy or competition scoring. PUBACK proves
broker delivery, not acceptance by a judge. The simulator is not a substitute
for the event's required hardware/network demonstration.

## Recorded validation — November 10, 2026

All **13/13** shared end-to-end steps passed with RoboNation revision
`8fc09f2`, Crusader `abf7d82`, shared OCS base `1a3cb40`, and GRAEY base
`249d641` plus the local harness changes. The final run forwarded 165 reports
with zero invalid, rate, type, registration or preference drops, zero stale
reports and zero undeliverable commands. GRAEY reconnected twice in total,
received one survey acknowledgement, retained zero pending reports, and
finished with remote run ENDED and run confirmation false.

The [machine-readable result](validation/rc-e2e-graey-2026-11-10.json) includes
test verdicts, source hashes, official-server observations and evidence paths.
These are local test changes; no deployment or publication was performed.

Shared OCS regressions: 611 tests run, OK with one skipped. An initial run
concurrent with E2E hit the existing declaration-snapshot timing race in
`test_a_refused_run_start_sends_nothing`; its 27-test module and the full
sequential rerun passed without changing that test or production code.
