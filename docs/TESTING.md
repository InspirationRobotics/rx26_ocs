# How to test the OCS

Four levels, cheapest first. Level 4 is the one that counts for the
communications Proof of Readiness; the first three are how you get there
without burning sim-box time.

Every command runs from the repo root.

---

## 0. Setup, once

```bash
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements.txt
```

Linux/macOS: `.venv/bin/python` everywhere below instead of `.venv/Scripts/python`.

You do **not** need protoc. The generated bindings are committed. Only re-run
`bash make_protos.sh` when you deliberately want to adopt a newer robocommand
revision, and read the diff when you do.

---

## 1. Unit tests — 1 second

```bash
.venv/Scripts/python -m unittest discover -s tests -t .
```

**Expect:** `Ran 57 tests` / `OK`.

Covers config validation, the sequence store's durability, the rate governor,
the run state machine, and the wire format. If this fails, nothing below is
worth running.

---

## 2. The bench — about 60 seconds

```bash
.venv/Scripts/python bench_stage1.py
```

**Expect:** `17/17 checks passed`, every line `PASS`.

This runs the entire start-of-run sequence against a broker inside the same
process: connect, receive the course, refuse a premature declaration, declare,
heartbeat at 2 Hz, refuse a NaN heading, refuse an unset task, accept a
matching RunStart, refuse a mismatched one, relay the good one to the vehicle,
and survive a restart without rewinding sequence numbers.

The two checks to care about most:

| Check | What it proves |
|---|---|
| `A10b` | RunStart from RoboNation actually reaches the vehicles' command topic |
| `A11b` | A RunStart answering someone else's declaration is **not** relayed |

No docker, no broker install, no network, no boat. This is the one to run
before every commit.

---

## 3. The GUI, standalone — no sim box needed

```bash
.venv/Scripts/python -m ocs_web --config bridge.toml --inject --local-broker
```

Open <http://127.0.0.1:8099>.

`--local-broker` runs an MQTT broker inside the process on port 11883 for the
team side, so you need nothing installed. `--inject` turns on the fake-data
buttons and puts an orange banner across the top; without it the buttons are
gone and the endpoint returns 403.

**Expect:** RoboCommand shows `DOWN 192.168.65.2:1883` (correct — no sim box),
team broker shows `connected 127.0.0.1:11883`, three vehicle cards reading
"never heard".

Now walk the fault buttons on the USV card and watch the Log pane:

| Button | Expected log line | Counter that moves |
|---|---|---|
| Inject 1 | *(no complaint)* | `forwarded` — only after you declare |
| NaN hdg | `DROP USV1: heartbeat.heading_deg: NaN` | `invalid` |
| no task | `DROP USV1: heartbeat.current_task: UNKNOWN` | `invalid` |
| wrong type | `DROP: USV1 is configured TYPE_USV but reported TYPE_UAV` | `mistyped` |
| no phase *(UAV card)* | `DROP UAV1: heartbeat.flight_phase: UNKNOWN` | `invalid` |

If a fault button produces no DROP line, a safety net is not wired up — that is
a real bug, not a display glitch.

**Note:** `forwarded` stays 0 until you declare. That is correct: the bridge
publishes nothing to RoboCommand before a run is declared. Telemetry still
appears on the page, which is the point — pre-declaration is when you decide
whether the fleet is fit to declare at all.

---

## 4. Against RoboNation's real stack — the one that counts

### 4a. Start their server

On the sim box:

```bash
git clone https://github.com/robonation/robocommand
cd robocommand/RobotX_2026
docker compose up --build
```

That brings up Eclipse Mosquitto on 1883 plus their `test_server.py`, which
logs every decoded message to `./logs/` and to the console. **Those logs are
your evidence for the Proof of Readiness — keep them.**

### 4b. Point the OCS at it

In `bridge.toml`, set `[robocommand] host` to the sim box's address. On the real
course that is the `.2` address for your course (Alpha `192.168.65.2`, Bravo
`.66.2`, Charlie `.67.2`, Delta `.68.2`).

**Never set a static address on the OCS's own RoboCommand-facing interface** —
the handbook forbids it, and DHCP is what makes one OCS config work on all four
courses.

### 4c. Run and drive it

```bash
.venv/Scripts/python -m ocs_web --config bridge.toml --inject --local-broker
```

1. RoboCommand pill goes green, and Course fills in within a second or two —
   their server publishes a *retained* `RxCourse`, so it arrives on connect.
   **If Course stays empty, nothing else can proceed** and their server is not
   publishing it.
2. Press **Declare run**. Their log should show `[RxRequest]` and
   `[auto-start] Tracking ASTA vehicles [...] — waiting for all to report AUTO`.
3. Press **Stream 2 Hz** on every vehicle you declared.
4. Their server publishes RunStart on its own once all declared vehicles report
   `STATE_AUTO`. Your page flips to `RUNNING run_id=N`, and the Commands pane
   shows `run_start → USV1, UUV1, UAV1`.

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
`vehicle_id` in your declaration has reported `STATE_AUTO`. There is no timeout
and no error — it just sits there. **Declare only the vehicles that are
actually running.** Edit the `[[vehicle]]` blocks in `bridge.toml` to match
what is really on the water.

**The geofence check keys on the vehicle ID string, not the type.**
`test_server.py:104` rejects the declaration if any `vehicle_id` starts with
`"UAV"` and `uav_geofence` has fewer than 4 points. Renaming `UAV1` to
something else would silently skip the check.

**The geofence in `bridge.toml` is a placeholder.** It is a box around their
test course in Singapore. Their server only counts points — it will accept a
fence anywhere on Earth without complaint. Replace it with the real one.

**Windows reserves port 1883.** Binding a local broker there fails with
`WinError 10013` (Hyper-V reserved range). That is why `--local-broker`
defaults to 11883. Check with:

```bash
netsh interface ipv4 show excludedportrange protocol=tcp
```

**Do not plug the sim box into the team router.** Not the LAN port and not the
WAN port. Vehicles must not be able to reach RoboCommand. Give it its own
segment, and verify from a vehicle that `ping <sim-box>` *fails*. If it
succeeds you are non-compliant, and everything will otherwise appear to work
right up until the course.

**Check the OCS is not routing between its two networks:**

```bash
netsh interface ipv4 show interface          # Windows: Forwarding must be Disabled
sysctl net.ipv4.ip_forward                   # Linux: must be 0
```

---

## What is not tested yet

- Nothing has run against **Eclipse Mosquitto**. The verification in
  `docs/` used their `test_server.py` against a Python broker, because Docker
  is not installed on the dev laptop. Their server code and protobuf were
  exercised; the real broker was not. Do 4a on the sim box before believing it.
- No real vehicle has ever published to this OCS. Every heartbeat so far came
  from the inject buttons.
- The UUV's submerged heartbeat is unresolved — see
  [ROBONATION_QUESTIONS.md](ROBONATION_QUESTIONS.md) Q1.
