# The vehicle link — TCP 37564

The contract between the OCS bridge and every vehicle client, **as the code
implements it**. Two kinds of client speak it today:

| Client | Where it runs | Says hello? | Link |
|---|---|---|---|
| `crusader_groundstation/ocs_link.py` (Crusader) | the boat's Jetson, over team WiFi; finds the OCS from its [beacon](#discovery--udp-37565) | no | `wifi` |
| `uav_groundstation/ocs_link.py` (Ekko) | Ekko's Jetson, over team WiFi | no | `wifi` |
| `fleet/ocs_adapter.py` (Ekko's radio) | this laptop, inside `fleet_link`, on loopback | yes | `radio` |

The radio side of the third one — how it turns RXL TUNNELs into these frames
and back — is [EKKO_RADIO_LINK.md](EKKO_RADIO_LINK.md). What the vehicles
would have to change to use any of this is
[VEHICLE_SIDE_PROPOSAL.md](VEHICLE_SIDE_PROPOSAL.md). Tests:
`tests/test_vehicle_link.py`, `tests/test_ocs_info.py`,
`tests/test_readiness_and_wirelog.py`, `tests/test_fence_gate.py` and
`tests/test_geofence.py` (the geofence and its gate), and `bench_stage2.py` against
the boat's real client. `bench_stage3.py` plays an aircraft that answers the fence
handshake.

## Framing

Every frame, both directions (`rx_bridge/framing.py`):

    4 bytes   big-endian unsigned length N         (struct ">I")
    N bytes   UTF-8 JSON, one object

`N` above `MAX_FRAME` (1 MiB, `framing.py:32`) is a desynced stream: the
connection is dropped and the client reconnects into a clean one. (The
`framing.py` docstring still says "the protobuf payload"; it is JSON.) The
framing is **duplicated** in the boat's and Ekko's `ocs_link.py` — change one,
change all three in the same round.

The vehicle dials in; the OCS never needs its address. One socket carries both
directions. `[tcp] host = "0.0.0.0"`, `port = 37564` (`bridge.toml:95-97`). How the
vehicle learns *our* address is [Discovery](#discovery--udp-37565), next.

## Discovery — UDP 37565

The vehicle dials in, so it needs the OCS's address — and that address has been
`192.168.100.107` (lab WiFi), `192.168.8.107` (the Rocket network) and
`192.168.1.75`. Rather than type one into a vehicle, the OCS **broadcasts where it
is** and the boat listens. Code: `rx_bridge/beacon.py` (OCS side),
`crusader_groundstation/ocs_link.py` (boat side). Tests: `tests/test_beacon.py`
(including a check that the boat's copy of the format has not drifted from ours)
and the boat's `crusader_groundstation/test/test_ocs_link.py`.

**What the OCS broadcasts.** Every `[discovery] interval_s` (1.0 s) it sends one
UDP datagram to `255.255.255.255:37565`:

```json
{"ocs_beacon": {"team": "ASTA", "port": 37564, "v": 1}}
```

It means: *an OCS for team ASTA accepts vehicle links on TCP 37564, at the address
this datagram came from.* There is **no address in the payload** — the boat uses
the datagram's source IP, which is by construction an address on a network the boat
can reach. `team` is `team_id` from `bridge.toml`; `port` is the **bound** `[tcp]`
port (the real one even if `[tcp] port = 0`); `v` is 1 and a receiver ignores any
other value. One datagram goes out **per eligible laptop address**, from a socket
bound to that address, so each copy leaves by that address's own interface and
carries its own source IP. The laptop's addresses are listed again every 10 s,
because WiFi comes and goes. The OCS does not listen on 37565 and never binds it.

**Where it never broadcasts, and why.** The two networks are never bridged
(`CLAUDE.md`): vehicles must not be able to reach RoboCommand, and a beacon on the
course network would put our team code and vehicle port on RoboNation's side. An
address is skipped — and listed with its reason on the operator page — if it is:

| Skipped when | Why |
|---|---|
| loopback (`127/8`) or unspecified | nothing off the laptop can hear it |
| link-local (`169.254/16`) | an interface with no real address (Tailscale's, an unplugged NIC's) |
| inside a `[discovery] never_on` subnet — default **Alpha `192.168.65.0/24`, Bravo `.66`, Charlie `.67`, Delta `.68`** | the RoboCommand-facing DHCP networks |
| in the same /24 as `[robocommand] host` | catches a bench stub on a subnet that is not one of the four, or a renumbered course. A hostname there adds no rule (it is not resolved) — put its subnet in `never_on` |

`never_on`, when set in `bridge.toml`, **replaces** the default rather than adding
to it; the bridge logs a WARNING at start if a course subnet is missing from it.
Nothing here sets an address or touches a route: no static address on the
RoboCommand-facing interface, as ever.

**Windows sends a limited broadcast out of the bound address's interface —
measured.** On this laptop Wi-Fi, a USB Ethernet adapter on the course subnet and
a VMware adapter all hold an on-link `255.255.255.255` route at the same metric.
100 × 1200-byte datagrams sent from a socket bound to the Wi-Fi address raised that
adapter's `SentBroadcastPackets` by exactly 100 and the Ethernet adapter's by 0
(`Get-NetAdapterStatistics`, before and after). So no per-subnet directed broadcast
is computed and no mask is guessed. Verified for Windows only. If an adapter is
missing from the sending list, the address enumeration (`socket.getaddrinfo` on the
machine's own name — it returned Wi-Fi, USB Ethernet, VMware and Tailscale here, and
not the tentative or disconnected ones) did not list it; the boat's fallback hosts
cover that.

**What the boat does with it** (`ocs_host: "auto"`, the shipped value). It listens on
UDP `ocs_discovery_port` (37565, `0` = do not listen) and keeps the newest beacon
whose `team` equals its own `team_id` and whose source is not on a course network.

- **Not connected:** it dials the **source IP of the newest beacon heard in the
  last 5 s, at the port that beacon names**. With no such beacon it takes the next
  address in `ocs_fallback_hosts` (default `192.168.100.107`, `192.168.8.107`) at
  `ocs_port`, round-robin; a fallback host that answered is tried first next time.
  With neither, it waits and says so once.
- **Connected:** it stays. A later beacon from anywhere else changes nothing; the
  choice is made again only after the connection drops. Each change of target is
  logged once, with why: `OCS link: target -> 192.168.100.107:37564 (beacon from
  192.168.100.107)` or `... (fallback 192.168.8.107)`.
- `ocs_host: "<ip>"` is exactly today's behaviour: dial that address, no listener.
- Never pointed at a course network: a fallback entry there is dropped at startup
  and `tools/scripts/check_config.py` fails the YAML.
- **A taken beacon port is tried again every 5 s** (`LISTEN_RETRY_S`) and logged
  once, not on every try. Until it binds, the boat runs on its fallback hosts and the
  GUI's OCS tooltip reads `beacon listener on UDP 37565 NOT RUNNING (<why>; retrying)`.
  2026-10-31: a hand-run `ocs_client` held the port while the launch respawned its
  own, and the respawned one never listened again, even after the other was gone.
  Two `ocs_client`s are a fault in themselves: each takes the other's USV1 socket
  several times a second (`USV1 reconnected on wifi, closing stale ...` in the log).

**Two OCS laptops on one network.** Both beacon. A boat with no connection takes
whichever it heard last when it retries and keeps it until the link drops; nothing
arbitrates between them. The handbook allows the team one connection to
RoboCommand, so the second laptop should not be running `ocs_web` at all. Ekko's
client still uses a fixed `ocs_host` and is unaffected either way.

**When no beacon arrives** — an access point that isolates clients and drops
broadcasts, `[discovery] enabled = false`, an older OCS — the boat falls back to
`ocs_fallback_hosts`, so the worst case is the old behaviour, not silence. A laptop
at an address outside that list is found only by its beacon.

**Off unless asked.** A `bridge.toml` without `[discovery] enabled = true` sends
nothing (so a bench or test that writes its own config cannot advertise a test port
to a real boat). The shipped `bridge.toml` turns it on. The bridge snapshot has a
`beacon` object — `enabled`, `why` (when off), `port`, `tcp_port`, `interval_s`,
`team`, `sending_on[]`, `skipped[{addr, why}]`, `sent` (datagrams), `errors`,
`last_error` — shown on the operator page under *Beacon* and in the console
`status()`.

## The first frame: `ocs_hello` (optional)

```json
{"ocs_hello": {"vehicle_id": "UAV1", "link": "radio", "agent": "fleet_link",
               "synthesized": ["heartbeat"]}}
```

| Field | Rule (`bridge.py:698-753`) |
|---|---|
| `vehicle_id` | must be a `[[vehicle]] id` in `bridge.toml`, else **refused** |
| `link` | one of `LINKS = ("radio", "wifi")` (`config.py:39`); absent = `"wifi"`; anything else **refused** |
| `agent` | free text, cut to 64 characters; shown on the page as "radio via *agent*" |
| `synthesized` | report kinds this link **builds itself** rather than relays from the vehicle. Absent, `null` or `[]` = none. Otherwise a list of `REPORT_KINDS` names; anything else (not a list, a misspelt kind) is **refused** (`_parse_synthesized`, `bridge.py:1415-1431`) |

- **No hello = `wifi`.** A socket that never sends one is filed as `wifi` under
  the `vehicle_id` of its first report (`tcpserver.py:58`). Both vehicles'
  own clients work this way and need no change.
- **Refused** means: `close_reason` is set, the socket is dropped, the wire log
  gets `ocs_hello REFUSED: <why>` and a `hello_refused` event, and nothing is
  filed — so nothing is sent to it, not even `ocs_info`. Refusing rather than
  guessing is deliberate: a radio adapter filed as `wifi` would have its
  telemetry dropped as the non-preferred copy, silently.
- **A hello is only honoured before the socket is identified.** One arriving
  after a report has filed the socket is logged as `ocs_hello IGNORED` and
  changes nothing (`bridge.py:720-728`).
- **A hello files the socket at once**, before any report — so the link-up
  `ocs_info` (below) reaches an adapter immediately. A hello is not a report: it
  does not make the vehicle active, and does not count as a report on the page.
- **One socket per vehicle per link.** A second socket for the same
  `(vehicle, link)` replaces the first (a reconnect); a socket on the *other*
  link coexists with it (`tcpserver.py:242-262`).

## Uplink: RxReport JSON

Every frame after the hello is an `RxReport` as JSON, parsed with protobuf's
`ParseDict` (`bridge.py:663-677`): proto field names or lowerCamelCase, enums
by name, non-finite floats as the quoted string `"NaN"`, timestamps RFC 3339.
An unknown key, a bad enum name or a bare `NaN` makes the frame undecodable: it
is wire-logged with the reason and skipped, and the socket stays up. The one other
uplink frame is [`ocs_fence`](#ocs_fence--what-the-autopilot-holds-vehicle--ocs),
recognised by its top-level key before the report parse and never treated as a report.

`seq` is optional. If the vehicle puts its own non-zero `seq` on a report the
bridge keeps it (as the report's "own seq") before overwriting `seq` with the
OCS's — that is what makes the ReadinessConfirm rewrite below possible, and a
numbered task report is answered with an [`ocs_receipt`](#ocs_receipt--what-became-of-a-numbered-task-report).

### Report kinds

A report's **kind** is the name of the `RxReport.body` oneof field it fills.
`REPORT_KINDS` (`bridge.py:69-70`) is read from the generated schema, so it
cannot fall behind it:

    heartbeat  safe_passage  pipeline_survey  resource_delivery
    docking    firefighting  incident_ack     readiness

These are the only strings a hello's `synthesized` may hold, the keys the link
preference files arrivals under, and the keys of the per-kind drop counters. A
report with no body is filed as kind `"none"`.

### The link preference, per kind

`rx_bridge/linkpolicy.py`, called from `_prefer_gate` (`bridge.py:761-785`).
For each vehicle and each kind the bridge remembers, per link, when that kind
last arrived and whether that link declared it synthesized. A report of kind K
arriving on link L is **dropped** only if another link M delivered K within
`link_prefer_s` **and** M's copy outranks L's (`duplicate_reason`,
`linkpolicy.py:64-92`):

1. **Real beats synthesized**, whatever the links (`_outranks`,
   `linkpolicy.py:56-61`). A heartbeat the radio adapter rebuilt from the
   autopilot stream never displaces Ekko's own heartbeat from WiFi.
2. **Among equals, `prefer` wins** (`[[vehicle]] prefer`, `config.py:98`;
   `prefer = "radio"` on UAV1, `bridge.toml:65`). With no `prefer`, equal
   copies both stand.
3. **A kind seen on only one link is never dropped.** WiFi's task reports,
   IncidentAck and Readiness are kept while the radio is up and fresh, because
   the radio has not delivered *that kind*.

`link_prefer_s` (`[limits]`, default 3.0, must be > 0 — `config.py:164`,
`:409-413`; `bridge.toml:136`) is the freshness window. Six missed frames at
2 Hz; short enough that a dead radio hands over in seconds.

An arrival counts even if the governor or validator later refuses it: link
choice is about which pipe is alive, not whether its content was good. A drop
is wire-logged with its reason (`DROPPED not preferred: ...`) and counted per
vehicle and per kind; it does **not** update the page's report, `last_seen` or
the active set.

**Where it sits.** First, before everything else in the hot path, and on TCP
only (MQTT reports bypass it):

    report → [TCP] link preference → staleness → governor → validate
           → sequence number → re-stamp sent_at → publish

Ahead of the governor so a duplicate is never seen as a doubled rate (and a
legitimate frame dropped for it); ahead of the sequence number so a dropped
duplicate never burns a seq.

## Downlink

Everything the OCS sends a vehicle goes down **one** link, never two — a
command delivered twice is a command a vehicle may act on twice.

### Which link: `send_order`

`linkpolicy.send_order` (`linkpolicy.py:95-108`) ranks the vehicle's connected
sockets: **fresh before stale**, then the `prefer` link, then the wider pipe
(`wifi` before `radio`, `_TIEBREAK`, `:48`). Fresh here means the socket
received *any* frame within `link_prefer_s` (`Conn.last_rx`: a hello counts) —
not the same thing as the page's per-link `fresh`, which is about reports. The
frame goes to the first socket that accepts it (`_send_to_vehicle`,
`bridge.py:938-955`). A socket that is connected but stale is still tried
before giving up.

### RxCommand frames

Every RoboCommand `RxCommand` is routed by `_route_command`
(`bridge.py:464-543`) as JSON from protobuf's `MessageToJson` with proto field
names (`_to_json`, `bridge.py:1501-1514`). **Default-valued fields are
omitted**: no `seq` means 0, no `vehicle_type` means `TYPE_UNKNOWN`, a
coordinate of exactly 0.0 is absent.

```json
{"team_id": "ASTA", "seq": 12, "sent_at": "2026-11-09T02:03:04.5Z",
 "vehicle_id": "UAV1", "keep_out_zone": {"center": {...}, "radius_m": 20.0,
 "vehicle_type": "TYPE_UAV"}}
```

- **Addressed** (`vehicle_id` set): that vehicle only. A vehicle not in the
  fleet → `COMMAND UNDELIVERABLE`, counted, never invented into a topic.
- **Unaddressed**: every configured vehicle, each on its own best link.
- **RunStart** is relayed only if it answers *our* declaration (`v.ok`,
  `bridge.py:453-454`).
- The command's own `seq` passes through untouched — the vehicle's
  `IncidentAck.command_seq` must quote it.
- **MQTT fallback.** When no socket accepts the frame, the command is
  published as **protobuf binary** to `team/robotx/<team>/<vehicle>/command`,
  QoS 1 (`bridge.py:525-532`). Only commands fall back; `ocs_info` and
  `ocs_directive` do not.

### ReadinessConfirm: the one rewritten command

The handbook (3.4.12 Task 4, 3.4.15) requires the OCS to preserve sequence references
and to identify the readiness report by report seq *and* vehicle. The bridge
re-numbers every report, so the `report_seq` RoboCommand echoes back is the
OCS's number, which the vehicle never saw. `_resolve_confirm`
(`bridge.py:545-602`):

1. Every published non-heartbeat report is remembered as
   `(vehicle, OCS seq) → (vehicle's own seq, kind)`, last 256
   (`_remember_seq`, `bridge.py:917-922`; captured at `:886-893`).
2. The confirm's vehicle is `readiness_confirm.vehicle_id`, else the command's
   `vehicle_id` (a disagreement is logged, the confirm wins). Named nowhere: a
   seq held by exactly one vehicle is unambiguous, otherwise none.
3. Found, and the vehicle had sent its own seq → a copy goes out with
   `report_seq` = the vehicle's own seq, to that vehicle only. The wire log
   records `report_seq ours=M vehicle's=N` and a `readiness_seq` event.
4. Found but the vehicle sent no seq, or not found → relayed **unchanged**, to
   the vehicle if one is known (otherwise, like any unaddressed command, to
   every vehicle), and the log says so. Never a guessed number.

The radio adapter stamps its own seq on every report precisely so this works
over the radio; see EKKO_RADIO_LINK.md, *Readiness round trip*.

### `ocs_info` — information, never a command

```json
{"ocs_info": {"course": {RxCourse as MessageToDict, proto names} | null,
              "uav_geofence": [{"latitude": 1.2801, "longitude": 103.8552}, ...] | null}}
```

`uav_geofence` is the UAV geofence **for the course in hand** (next section), closed
(first point == last), in `RunDeclaration.uav_geofence`'s own JSON shape — built from
a real `RunDeclaration` so it cannot drift from what RoboCommand was declared
(`_fence_json`). RoboCommand never sends a geofence — the team declares it — so the
fence is ours, forwarded so the aircraft can hold it. It goes to `TYPE_UAV` vehicles
only, and **to them the key is always there**: both keys are explicit, and the pair
says which of three things is true:

| `course` | `uav_geofence` | Means | What the radio adapter does |
|---|---|---|---|
| `null` | `null` | this OCS has received **no course** since it started | clears the autopilot's polygon |
| `{...}` | `null` | there IS a course and **no fence could be made for it** — a bad `[[geofence.course]]` entry, a boundary the inset cannot trust, a margin that collapses it | leaves the old fence standing; the OCS makes this a loud **course finding** on the page |
| `{...}` | `[...]` | the fence for that course | writes it, reads it back, answers with an [`ocs_fence`](#ocs_fence--what-the-autopilot-holds-vehicle--ocs) |

The first row is the only one that clears anything. **The course is never reset to
`null` by a RoboCommand disconnect or an MQTT reconnect** — it is what RoboCommand
last told us, replaced only by a newer accepted `RxCourse`, so a dropped link never
tells an aircraft to forget its fence. (A restart of the OCS does lose it, until the
retained course is redelivered.) Nothing is sent to clear a fence when a run ends:
the next course replaces it. Each frame is a **complete snapshot**, never a delta
(`_info_obj`).

Sent (`_send_info`) when:

| Trigger | To | Link |
|---|---|---|
| a course that **differs** from the last one, `sent_at` ignored (RoboCommand re-delivers the retained course on every reconnect) — the fence is recomputed first, so the aircraft is told the NEW one | every vehicle | its best link |
| a declaration | the declared UAVs | its best link |
| a `(vehicle, link)` socket comes up — `_on_link_change` | that vehicle | **pinned to the socket that came up**, no fallback |

Only to connected vehicles with `send_info = true` (`[[vehicle]]`, default
true). Not delivered = not sent; the next link-up repairs it. Both vehicle clients
tolerate it today: their `ocs_link._deliver` hands any JSON object to the node, which
republishes it raw to `/crsd/ocs_command` or `/uav/ocs_command`. Set
`send_info = false` on any client found to choke on it.

#### Where the fence comes from — `rx_bridge/geofence.py`

There are four courses (ALPHA .. DELTA), each a different place, so the fence is
worked out per course when an `RxCourse` is accepted — **"table, else the boundary"**:

1. **A `[[geofence.course]]` entry** in `bridge.toml` whose `course_id` matches
   (`uav_geofence` = a closed `[[lat, lon], ...]`). Used **only if every vertex lies
   inside the course boundary or within 0.5 m of it**. Otherwise: no fence for this
   course, and a course finding naming the vertex that is outside. A bad entry never
   falls through to (2) — somebody wrote it on purpose.
2. **Otherwise the boundary, shrunk inward by `[geofence] boundary_margin_m`**
   (default 2.0 m, chosen so the declared vertices are strictly inside the boundary,
   never on it, where a float-rounding validation could reject them). Needs a simple
   **convex** boundary of at least three distinct corners, and a margin that does not
   collapse it; otherwise no fence and a finding that says which ("margin of 2.0 m
   collapses a course 3.0 m wide", "boundary not convex: add a `[[geofence.course]]`
   entry for ALPHA"). Computed in a local flat metre frame (WGS84 radii at the
   boundary's latitude); vertex *i* of the answer is the inset of corner *i*, same
   winding.
3. **No course yet:** no fence (`null`).

The top-level `uav_geofence` of `bridge.toml` is **retired**: the bridge no longer
declares or sends it. It stays in the file only because the drone repo's
`tools/scripts/check_config.py` still compares it with `uav_params.yaml`.

### `ocs_fence` — what the autopilot holds (vehicle → OCS)

```json
{"ocs_fence": {"vehicle_id": "UAV1",
               "state": "done",
               "polygon": [[1.2801, 103.8552], [1.2801, 103.85625], [1.2811, 103.85625]],
               "verified": true,
               "verified_at": "2026-10-11T00:31:05.123Z",
               "reason": "written and read back: 3 vertices, each within 1 m",
               "lockout": null}}
```

Sent by the radio adapter (`fleet/ocs_adapter.py`) after it has written a fence to
Ekko's autopilot and read it back: right after its `ocs_hello` on every (re)connect,
whenever any field changes, and at least every 60 s while verified.

| Field | Meaning |
|---|---|
| `vehicle_id` | must be a vehicle **this connection** has identified itself as, and an aircraft |
| `state` | the upload's own state: `off`, `waiting`, `busy`, `held`, `done`, `gave up` |
| `polygon` | the **OPEN** polygon it was given (no closing vertex), `[[lat, lon], ...]`; `null` = none known; `[]` = the autopilot's fence was **cleared and read back empty** (what the adapter does for `course: null, uav_geofence: null`) |
| `verified` | a real boolean — the autopilot was READ BACK holding exactly `polygon` |
| `verified_at` | RFC 3339 UTC of that read-back, or `null` |
| `reason` | free text, cut to 300 characters |
| `lockout` | `null`, or a string: the aircraft will not arm where it is. Shown, never gated on |

The bridge (`_on_fence`):

- keeps the **latest per vehicle** (newest wins). It never goes to RoboCommand, the
  governor or a sequence number — it is not a report;
- believes a body only about a vehicle **its connection has identified itself as**
  (hello, or a first report). From an unidentified socket, naming another vehicle,
  naming a vehicle that is not an aircraft, or malformed (`verified` not a boolean,
  a polygon that is not finite `[lat, lon]` pairs): the frame is **ignored,
  counted** (`counters.fence_frames_ignored`), logged, wire-logged
  (`ocs_fence IGNORED: <why>`, event `fence_ignored`) and the socket stays up;
- wire-logs every accepted one like any other vehicle frame (`veh tcp://in/<link>`,
  note `ocs_fence vehicle=UAV1 state=done verified=True vertices=4`);
- needs no clean-up when the link closes: freshness is judged from `verified_at`, so
  a confirmation from a dead adapter simply ages out.

#### The pre-run gate: refuse to declare

**The bridge does not declare a run that includes a `TYPE_UAV` unless that
vehicle's autopilot is confirmed to hold the fence being declared.** For every
aircraft going into the declaration (`Bridge._uav_gate`, rule in
`geofence.confirmation_problem`), the latest `ocs_fence` must have:

1. `verified: true`,
2. a `polygon` equal — each coordinate within 1e-6 deg — to the OPEN form of the
   current course's fence, and
3. a `verified_at` no older than `[geofence] confirm_max_age_s` (default 180 s; the
   adapter re-verifies every 60 s) **by the OCS clock**. A timestamp that is missing,
   unreadable or more than 5 s in the future is not believed.

With **no fence for the course, an aircraft cannot be declared at all.** A cleared
(`[]`) confirmation is a polygon that does not match, like any other. The USV and UUV
are not affected.

The check lives with the other declaration preconditions: `Bridge.declare_blocker()`
(state, then the two-vehicle floor, then this gate) is what `declare()` asks first —
so the page's button (`run.may_declare`, `run.declare_blocker` in the snapshot) and
`POST /api/declare` cannot disagree, and the endpoint refuses on its own whatever
the page showed. The refusal names the vehicle and what is missing:

    cannot declare -- UAV1: autopilot fence not confirmed (state held: armed: not writing)
    cannot declare -- UAV1: autopilot fence not confirmed (no ocs_fence received from it yet)
    cannot declare -- UAV1: autopilot holds a different polygon than the ALPHA fence (4 vertices, the fence has 4)
    cannot declare -- UAV1: fence confirmation 412 s old (limit 180 s)
    cannot declare -- UAV1: no UAV geofence for this course (no course received yet) -- an aircraft cannot be declared without one

The declaration itself carries the course's fence, closed
(`RunDeclaration.uav_geofence`), whenever there is one.

### `ocs_directive` — stand by in auto

```json
{"ocs_directive": {"action": "standby_auto", "declaration_seq": 3,
                   "vehicles": ["USV1", "UAV1"], "sent_at": "..."}}
```

Sent to each declared vehicle right after the declaration is published
(`_send_standby_auto`, `bridge.py:1078-1113`). Advisory: it changes no flight
mode; the RC e-stop stays the only safety path. No MQTT fallback; a vehicle
with no socket is logged as `NOT DELIVERED`. The radio adapter does not put it
on the air (there is no RXL message for it) and counts it under
`directives_ignored`.

### `ocs_receipt` — what became of a numbered task report

```json
{"ocs_receipt": {"vehicle_seq": 1792345601, "kind": "docking", "verdict": "acknowledged",
                 "seq": 34, "detail": "seq=34, RoboCommand's broker acknowledged it (PUBACK)"}}
```

For every task report that carries the vehicle's own `seq` (heartbeats never get
one), on the link it came in on, never the radio (no RXL message). `seq` is the
OCS's RoboCommand seq, `null` until it is published. Verdicts:

| `verdict` | Final? | Meaning |
|---|---|---|
| `held` | yes | no run declared: RoboCommand gets nothing before a declaration |
| `dropped` | yes | over the rate cap, or invalid; `detail` says which |
| `published` | no | sent to RoboCommand, its PUBACK not back yet (or no RoboCommand link) |
| `acknowledged` | yes | RoboCommand's broker returned the PUBACK |

Usually `published` then `acknowledged`; a PUBACK that beats the bridge's own
bookkeeping gives `acknowledged` alone. `_track`, `_await_ack`, `_send_receipt`.

**A resend is answered, not republished.** The boat's `ocs_link` keeps each
report until a final receipt and sends it again on its next connection — only
once this OCS has sent it a receipt, so an OCS that cannot answer one never sees a
resend. The bridge files every numbered report under (vehicle, own seq, `sent_at`)
and answers a repeat from what became of the first, wire-logging a `resend`
event. A restarted node numbers from the clock (`_first_seq`), so a new report
never repeats an old key; the same seq with a new `sent_at` is a new report.

## What the page and the snapshot show

`Bridge.snapshot()` (`bridge.py:1216-1306`), per vehicle:

| Field | Meaning |
|---|---|
| `prefer` | the `bridge.toml` preference, or null |
| `links.radio`, `links.wifi` | `null` = no socket. Else `connected`, `last_rx_age_s` (any frame, a hello counts), `last_report_age_s` (null = never relayed a report), `fresh` (a report within `link_prefer_s`), `peer`, `label`, `agent`, `hello`, `adapter` (a hello'd non-wifi socket), `synthesized` — `_links_view`, `bridge.py:1434-1460` |
| `active_link` | where a command would go right now (`send_order`'s first) |
| `dropped_not_preferred`, `dropped_not_preferred_by_kind` | the link-preference drops, total and per kind |
| `link`, `peer` | `"tcp"` while any socket is up, else what it last spoke on; the wifi socket's label |

Geofence fields (`geofence.py`, `_on_fence`): `uav_geofence` (the CURRENT course's fence,
closed `[{latitude, longitude}]`, or `null`), `uav_geofence_source` (`"table: ALPHA"`,
`"course boundary inset 2.0 m"`, or `null`) and `uav_geofence_reason` (why there is none, else
`null`); `fence_confirm` — `{"UAV1": {<the ocs_fence as received>, "age_s", "matches_course_fence"}}`
for each aircraft that has sent one, `age_s` being null when `verified_at` is unreadable —
`fence_confirm_max_age_s`; `run.may_declare` (the whole verdict) and `run.declare_blocker`
(its reason, or `null`); `counters.fence_frames_ignored`. A fence that could not be made is
also in `course_findings`.

Top level: `link_prefer_s`, `tcp_peers`, `counters.dropped_not_preferred`
(not part of `dropped_rate`: those frames never reached the governor), and
`commands[]` with `routes` (`"UAV1/radio"`, `"UUV1/mqtt"`) and `rewrite` for a
ReadinessConfirm. On the page an adapter with no report reads **"adapter up,
vehicle silent"** — its socket says nothing about the vehicle. The console
`status()` shows `radio+wifi (+N dup dropped)` when both are up.

Wire log (`run/wire/*.jsonl`): uplink frames as `veh tcp://in/<link>`, downlink
as `vtx tcp://<vehicle>/<link>` (or the MQTT command topic), each with
`decoded`; events `link_up`, `link_down`, `hello_refused`, `ocs_info`, `resend`,
`standby_auto`, `readiness_seq`, `readiness_mismatch`, `undecodable`, `geofence` /
`geofence_none` (a course was accepted and what its fence is), `fence_ignored`. An
`ocs_fence` is logged as `veh tcp://in/<link>` with its decoded body.

## Known limitations

- **`synthesized` is per link, not per frame.** The adapter's hello declares
  `["heartbeat"]` for the life of the socket. Once Ekko's companion sends its
  own RXL HEARTBEAT, the adapter relays that instead — but the bridge still
  files it as synthesized, so a fresh WiFi heartbeat outranks it. Harmless while
  WiFi's is real; wrong in principle. Fix: a per-frame flag the bridge reads
  before `ParseDict` (an unknown key would fail the parse today).
- **The readiness maps are in memory.** fleet_link's report-seq → cmd_seq map
  and the bridge's `_seq_map` both die with their process. A ReadinessConfirm
  for a report relayed before a restart is relayed unchanged by the bridge
  and refused (loudly) by the adapter.
- **So is the resend memory.** The last 500 numbered reports (`_receipted`) die
  with the bridge: a resend that crosses an OCS restart is published again.
- **A confirm is rewritten for the link that carried the report but routed by
  `send_order`.** If the READINESS came over the radio and the radio has since
  gone stale, the confirm goes down WiFi carrying the adapter's seq, which Ekko's
  WiFi client never saw (and the reverse).
- **Unaddressed commands go to every configured vehicle**, including ones with
  no socket — those fall back to MQTT, where nothing may be listening.
- **Delivered means "a socket took it".** The adapter can still refuse a
  command the radio cannot carry (a `run_id` above 65535); the bridge logs it
  as routed to `UAV1/radio`. The adapter's WARNING and `/fleet` counter are the
  only record — see EKKO_RADIO_LINK.md.
- **`sent_at` from the radio is the laptop's receive time** for relayed
  reports, so staleness cannot see time spent on the aircraft or in the air.
