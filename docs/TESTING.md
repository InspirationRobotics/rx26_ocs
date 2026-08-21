# How to test the OCS

**Written for Windows / PowerShell**, which is what the OCS laptop runs. Linux
equivalents are in the last section.

Four levels, cheapest first. Level 4 is the one that counts for the
communications Proof of Readiness; the first three are how you get there
without burning sim-box time.

Run everything from the repo root. All three of the first levels have been run
on this laptop under PowerShell — they are not translated-and-hoped-for.

---

## 0. Setup, once

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

**No Docker needed on the OCS laptop**, and none is installed. Docker is only
needed on the *sim box* (level 4).

**No protoc needed.** The generated bindings are committed. `make_protos.sh` is
a bash script and needs Git Bash — but you only run it when deliberately
adopting a newer robocommand revision, and you read the diff when you do.

---

## 1. Unit tests — 1 second

```powershell
.venv\Scripts\python.exe -m unittest discover -s tests -t .
```

**Expect:** `Ran 57 tests` then `OK`.

Config validation, sequence-store durability, the rate governor, the run state
machine, the wire format. If this fails, nothing below is worth running.

---

## 2. The bench — about 60 seconds

```powershell
.venv\Scripts\python.exe bench_stage1.py
```

**Expect:** every line `PASS`, ending `17/17 checks passed`.

Runs the entire start-of-run sequence against a broker inside the same process:
connect, receive the course, refuse a premature declaration, declare, heartbeat
at 2 Hz, refuse a NaN heading, refuse an unset task, accept a matching
RunStart, refuse a mismatched one, relay the good one, and survive a restart
without rewinding sequence numbers.

The two that matter most:

| Check | What it proves |
|---|---|
| `A10b` | RunStart from RoboNation actually reaches the vehicles' command topic |
| `A11b` | A RunStart answering someone else's declaration is **not** relayed |

No docker, no broker install, no network, no boat. Run this before every commit.

To see just the summary:

```powershell
.venv\Scripts\python.exe bench_stage1.py | Select-String "PASS|FAIL|passed"
```

---

## 3. The GUI, standalone — no sim box needed

```powershell
.venv\Scripts\python.exe -m ocs_web --config bridge.toml --inject --local-broker
```

Open <http://127.0.0.1:8099>. Stop it with `Ctrl+C`.

`--local-broker` runs an MQTT broker inside the process on port 11883 for the
team side, so nothing needs installing. `--inject` turns on the fake-data
buttons and puts an orange banner across the top; without it the buttons are
gone and the endpoint returns 403.

**Expect:** RoboCommand `DOWN 192.168.65.2:1883` (correct — no sim box), team
broker `connected 127.0.0.1:11883`, three vehicle cards reading "never heard".

Walk the fault buttons on the USV card and watch the Log pane:

| Button | Expected log line | Counter |
|---|---|---|
| Inject 1 | *(no complaint)* | `forwarded`, but only after you declare |
| NaN hdg | `DROP USV1: heartbeat.heading_deg: NaN` | `invalid` |
| no task | `DROP USV1: heartbeat.current_task: UNKNOWN` | `invalid` |
| wrong type | `DROP: USV1 is configured TYPE_USV but reported TYPE_UAV` | `mistyped` |
| no phase *(UAV card)* | `DROP UAV1: heartbeat.flight_phase: UNKNOWN` | `invalid` |

A fault button that produces no DROP line means a safety net is not wired up —
a real bug, not a display glitch.

**`forwarded` staying 0 until you declare is correct.** The bridge publishes
nothing to RoboCommand before a run is declared. Telemetry still shows on the
page, which is the point: pre-declaration is when you decide whether the fleet
is fit to declare at all.

---

## 4. Against RoboNation's real stack — the one that counts

### 4a. Start their server *on the sim box*

Docker runs on the sim box, not on the OCS laptop.

```bash
git clone https://github.com/robonation/robocommand
cd robocommand/RobotX_2026
docker compose up --build
```

That is Eclipse Mosquitto on 1883 plus their `test_server.py`, which logs every
decoded message to `./logs/` and the console. **Those logs are your Proof of
Readiness evidence — keep them.**

If the sim box is also Windows, you need Docker Desktop with WSL2. A cheap
Linux box or a spare laptop is less trouble.

### 4b. Check the OCS can actually reach it

Before touching the config:

```powershell
Test-NetConnection 192.168.65.2 -Port 1883
```

Want `TcpTestSucceeded : True`. If it is False, fix the network before blaming
the software. Check what address you actually got:

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Select-Object InterfaceAlias, IPAddress
```

You want an address on the sim box's subnet, **handed out by DHCP**. Never set
a static address on the RoboCommand-facing interface — the handbook forbids it,
and DHCP is what makes one OCS config work on all four courses (Alpha
`192.168.65.2`, Bravo `.66.2`, Charlie `.67.2`, Delta `.68.2`).

### 4c. Point the OCS at it

In `bridge.toml`, set `[robocommand] host` to the sim box's address.

Trim the `[[vehicle]]` blocks to **only the vehicles that will actually report**
— see the first gotcha below, it is the one that will get you.

### 4d. Run and drive it

```powershell
.venv\Scripts\python.exe -m ocs_web --config bridge.toml --inject --local-broker
```

1. RoboCommand pill goes green and Course fills in within a second or two —
   their server publishes a *retained* `RxCourse`, so it arrives on connect.
   **If Course stays empty nothing else can proceed.**
2. Press **Declare run**. Their log shows `[RxRequest]` then
   `[auto-start] Tracking ASTA vehicles [...] — waiting for all to report AUTO`.
3. Press **Stream 2 Hz** on every vehicle you declared.
4. Their server publishes RunStart by itself once all declared vehicles report
   `STATE_AUTO`. Your page flips to `RUNNING run_id=N` and the Commands pane
   shows `run_start → ...`.

### Pass criteria

- [ ] Their log has `[RxRequest]` with your declaration, decoded correctly
- [ ] Their log has `[RxReport]` heartbeats for every declared vehicle
- [ ] Their log has `[auto-start] ... publishing RunStart`
- [ ] Their log has **no** `[ERROR]` lines
- [ ] Your page reaches `RUNNING` with a `run_id`
- [ ] Your Commands pane shows the RunStart routed to each vehicle

---

## Gotchas that will cost you an afternoon

**Their server waits forever for a vehicle that never reports.**
`rc_test/test_server.py:118-131` publishes RunStart only when *every*
`vehicle_id` in your declaration has reported `STATE_AUTO`. No timeout, no
error — it just sits there. **Declare only the vehicles that are actually
running.**

**The geofence check keys on the vehicle ID string, not the type field.**
`test_server.py:104` rejects the declaration if any `vehicle_id` starts with
`"UAV"` and `uav_geofence` has fewer than 4 points.

**The geofence in `bridge.toml` is a placeholder** — a box around their test
course in Singapore. Their server only counts points, so it will accept a fence
anywhere on Earth without complaint. Replace it with the real one.

**Windows reserves port 1883.** Binding a broker there fails with
`WinError 10013`, which is why `--local-broker` defaults to 11883. To see the
reserved ranges:

```powershell
netsh interface ipv4 show excludedportrange protocol=tcp
```

**Do not plug the sim box into the team router** — not the LAN port, not the
WAN port. Vehicles must not be able to reach RoboCommand. Give it its own
segment and verify *from a vehicle* that pinging the sim box **fails**. If it
succeeds you are non-compliant, and everything will otherwise appear to work
right up until the course.

**The OCS must not route between its two networks:**

```powershell
Get-NetIPInterface -AddressFamily IPv4 | Select-Object InterfaceAlias, Forwarding
```

Every line must say `Disabled`.

---

## Linux / macOS equivalents

Replace `.venv\Scripts\python.exe` with `.venv/bin/python` throughout. Then:

| Windows | Linux |
|---|---|
| `Test-NetConnection host -Port 1883` | `nc -vz host 1883` |
| `Get-NetIPAddress` | `ip addr` |
| `Get-NetIPInterface ... Forwarding` | `sysctl net.ipv4.ip_forward` (must be `0`) |
| `netsh interface ipv4 show excludedportrange` | n/a |

---

## What is not tested yet

- **Nothing has run against Eclipse Mosquitto.** The verification so far used
  RoboNation's `test_server.py` against a Python broker, because Docker is not
  installed on the dev laptop. Their server code and protobuf were exercised;
  the real broker was not. Do 4a before believing it.
- **No real vehicle has ever published to this OCS.** Every heartbeat so far
  came from the inject buttons.
- The UUV's submerged heartbeat is unresolved — see
  [ROBONATION_QUESTIONS.md](ROBONATION_QUESTIONS.md) Q1.
