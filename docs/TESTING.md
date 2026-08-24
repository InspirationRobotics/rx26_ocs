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

**Expect:** `Ran 69 tests` then `OK`.

Config validation, sequence-store durability, the rate governor, the run state
machine, the wire format, and the vehicle link's frame boundaries. If this
fails, nothing below is worth running.

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

## 2b. The vehicle link bench — about 30 seconds

```powershell
.venv\Scripts\python.exe bench_stage2.py
```

**Expect:** every line `PASS`, ending `16/16 checks passed`.

Stage 1 exercises the bridge over MQTT. This one exercises the **TCP link the
vehicles actually use** — and it does it by importing the boat's own client,
`crusader_groundstation/ocs_link.py` out of the rx26_asv checkout, rather than
writing a stand-in here.

**That import is the whole point.** The frame format is duplicated across the
two repos by necessity — a ROS package cannot import from the OCS — so the one
failure this bench exists to catch is those two copies drifting apart. A test
that spoke the OCS's own framing back to itself would agree with itself
perfectly while the boat sent garbage.

It expects `rx26_asv` to be a sibling of this repo. If it is not:

```powershell
$env:RX26_ASV = "C:\path\to\rx26_asv"
```

| Check | What it proves |
|---|---|
| `B2` | The OCS learns which vehicle is on a socket from the message, not the connection |
| `B5`/`B6` | The NaN and TASK_UNKNOWN nets still fire on the TCP path |
| `B7` | Two frames in **one** write arrive as two, not one |
| `B8` | One frame split across two writes arrives once — not twice, not never |
| `B9b` | A command for USV1 does **not** go down UUV1's socket |
| `B12b` | 20 heartbeats queued during an outage produce **one** frame on reconnect |

`B7` and `B8` are the ones that matter most. TCP does not preserve message
boundaries, and a client that reads whatever `recv()` happens to return works
on a bench at 2 Hz and starts tearing messages the moment traffic is bursty —
which is exactly what a reconnect produces.

`B12b` is the other one. Without a latest-wins slot on the boat, a dropped link
accumulates heartbeats and dumps them all on reconnect; the OCS rate governor
then discards most as a rate problem when it was a link problem, and the
survivors get re-stamped with the current time — so a position from ten seconds
ago reaches RoboNation labelled as current.

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
broker `connected 127.0.0.1:11883`, `vehicle link: listening on 0.0.0.0:37564`,
and three vehicle cards reading "never heard".

### Driving it with the real boat client

The inject buttons fake data *inside* the OCS. To exercise the actual vehicle
link, run the boat's client against it — no ROS and no Pixhawk needed:

```powershell
cd ..\rx26_asv\crusader_groundstation
..\..\OCS\.venv\Scripts\python.exe -m crusader_groundstation.ocs_link --host 127.0.0.1
```

**`127.0.0.1` is correct only because both ends are on this laptop.** On the
Jetson that address is the Jetson itself and the connect fails; there, `ocs_host`
in `crusader_params.yaml` must be the OCS's address on the team subnet.

**Expect** in the OCS log:

```
vehicle link: 127.0.0.1:13474 connected
vehicle link: 127.0.0.1:13474 identified as USV1
```

and the USV1 card showing `link  tcp  127.0.0.1:13474`. Stop the client with
Ctrl+C and the OCS must log `(USV1) disconnected` within a second — if it does
not, clean-close detection is broken and a dead link will read as healthy.

`--fault nan` and `--fault unknown_task` provoke the two validator faults over
the real link.

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

## Run against the real sim box — 2026-08-20

Level 4 done, over Ethernet, against Eclipse Mosquitto on the sim box. All of
it passed:

| | Result |
|---|---|
| DHCP lease on the course subnet | `192.168.65.191/24`, `PrefixOrigin: Dhcp` — not static |
| IP forwarding | `Disabled` on every interface |
| TCP to `192.168.65.2:1883` | succeeds (ICMP is filtered — ping fails, which is fine) |
| Retained `RxCourse` | ALPHA, 25000 Hz, 5 corners, corner[0] `1.280329, 103.8556869` |
| RunDeclaration | accepted, `seq=1` |
| Their auto-start | RunStart within 1 s of all three vehicles reaching `STATE_AUTO` |
| Run state | `RUNNING run_id=1` |
| RunStart relay | routed to USV1, UUV1, UAV1 |
| Heartbeats | 24 forwarded, 0 dropped, 0 stale |
| Fault nets | 4 fired, 4 refused, 0 reached RoboCommand |
| Restart durability | run restored as RUNNING, seq 8 → 14, no rewind, no re-declaration |

Evidence on the OCS side is in `run/wire/*.jsonl` (every frame both directions)
and `run/seq.json`. Evidence on RoboNation's side is the `./logs/` directory of
their docker stack — **that is the half that counts for the Proof of Readiness,
and it lives on the sim box, not here.**

## What is still not tested

- **The cable has never been pulled.** The handbook requires the OCS to
  reconnect and restore its subscriptions. Restarting the process is proven;
  losing the link underneath a running process is not. Ten-second test: with
  the OCS running and streaming, unplug the Ethernet, watch the RoboCommand
  pill go red, plug it back in, and confirm it goes green and reports
  `reconnected into RUNNING; NOT re-declaring` without minting a new
  declaration.
- **No real vehicle has ever published to this OCS.** The boat's own client
  code now drives the link (level 2b and level 3), but every heartbeat so far
  is still synthesized — nothing has yet come from a Pixhawk, over WiFi, on a
  Jetson. `ros2 run crusader_groundstation ocs_client` with real topics flowing
  is the step that has not been taken.
- **The UUV and UAV have no client at all.** Only `crusader_groundstation` in
  rx26_asv has one; Graey and BabyDragon still report nothing.
- The UUV's submerged heartbeat is unresolved — see
  [ROBONATION_QUESTIONS.md](ROBONATION_QUESTIONS.md) Q1.
