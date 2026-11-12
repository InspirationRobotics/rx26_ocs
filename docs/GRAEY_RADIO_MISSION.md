# GRAEY radio mission integration

Status: locally implemented and software-tested on 2026-11-11; not field-tested.

The shared `fleet_link` now treats GRAEY as RoboCommand vehicle `UUV1` and as a
radio companion application endpoint:

- companion system: 43 (`radio_mission_node` on the Jetson)
- planned unique Cube system: 3 (must be verified/applied on hardware)
- OCS identity: 250/190
- laptop GUI identity: 254/191 on UDP 14572
- preferred field link: `rfd`; WiFi remains the pre-deployment link

The GUI sends application missions as bounded, checksummed MAVLink `TUNNEL`
messages. `fleet_link` routes those messages without becoming a mission
executor. GRAEY returns acknowledgements/status on the same route and sends the
standard RXL UUV heartbeat. The normal OCS adapter translates RoboCommand
RunStart into the existing retry/ACK protocol.

For a prepared competition mission, GRAEY waits in actual Cube POSHOLD but
reports `STATE_AUTO`, because its autonomous mission is claimed. A matching,
verified RunStart is acknowledged and releases the Jetson mission into GUIDED.
An early RunStart is not acknowledged, so `fleet_link` retries it. Link loss
after admission does not clear the onboard run latch.

Final local validation uses RoboNation server revision `8fc09f2`: all 13 shared
E2E stages pass, UUV1 is observed at the nominal 2 Hz rate, and the full shared
OCS suite passes 1,145 tests with one skip. The shipped RFD adapter has a direct
regression for companion system43 reporting `UUV1`, `TYPE_UUV`, `STATE_AUTO`
and `TASK_INFRA_SURVEY_REPAIR` at 2 Hz.

This follows the shared Crusader fleet/adapter and RunStart behavior. The GRAEY
mission document is a vehicle adaptation because the UUV's waypoint, pipeline,
survey, and 5-2-5 implementations execute on its Jetson rather than as an
ArduPilot AUTO mission.

See the GRAEY repository document
`docs/rfd-mission-relay-2026-11-11.md` for the complete ownership, validation,
and hardware-acceptance record.

## Laptop startup

Create/update the repository environment with
`.venv\Scripts\python.exe -m pip install -r requirements.txt`. The requirements
pin the OCS/MQTT stack plus `pymavlink` and `pyserial`; `OCS.cmd` starts
`FLEET_LINK.cmd` with that same environment. No separately configured global
Python is required.

`fleet/fleet.toml` currently names `COM8`. This is a configuration value, not a
claim about the connected radio. If Windows assigns another port, start
`FLEET_LINK.cmd --serial COMx` or change the reviewed configuration before the
test. The historical GRAEY radio was once observed on COM3; it must be detected
again rather than assumed.
