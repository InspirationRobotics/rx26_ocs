# Open questions for RoboNation

One design decision in the OCS is blocked on an answer to Q1. The other two are
things we can proceed past by assuming, but where being wrong costs a run.

**Send Q1 first and separately if that gets it answered faster.** Everything
about how the UUV appears to RoboCommand depends on it, and the vehicle-side
reporter cannot be finished until we know.

---

## Q1 — Can a submerged UUV's heartbeat be interrupted? (BLOCKING)

Handbook §3.4 requires the OCS to publish an `RxReport` heartbeat on behalf of
each **active vehicle at 2 Hz**, and lists depth among the heartbeat contents —
so submerged vehicles are clearly contemplated. But the section does not define
"active vehicle", and does not say what happens when a vehicle is physically
unable to sustain 2 Hz.

Our UUV reaches the USV over an acoustic modem at 9600 baud with multi-second
latency. It cannot carry a 2 Hz stream of position, attitude and depth. The USV
relays what it receives to the OCS, which holds the only RoboCommand connection.

Three readings, and we cannot tell which you intend:

1. **The OCS synthesizes.** The USV propagates the last acoustic fix and the OCS
   publishes at 2 Hz continuously, so every frame between acoustic updates is
   dead-reckoned rather than measured. This meets the letter of the 2 Hz rule
   but means most published UUV positions are computed, not observed. If this is
   what you expect, **is there a field or convention for marking a heartbeat as
   propagated rather than measured?** We would rather not report a computed
   position as though it were a fix.
2. **The heartbeat may lapse.** We publish only what the modem actually
   delivers, and the UUV's heartbeat rate drops well below 2 Hz while it is
   submerged. Is that scored as a communications failure?
3. **A submerged UUV is not "active".** We stop publishing for UUV1 while it is
   down and resume when it surfaces. This is the cleanest compliance story but
   leaves Task 2 unobserved by RoboCommand for most of its duration.

**Our question:** which of these do you intend, and is option 1 acceptable —
specifically, is it acceptable to publish dead-reckoned positions at 2 Hz for a
submerged vehicle, and if so how should they be marked?

---

## Q2 — Is a vehicle that is powered but not deployed "active"?

Same undefined term, different case. Our UAV sits on the USV deck, powered and
reporting, for parts of a run before it launches.

Do we publish heartbeats for it from power-on (`FLIGHT_PHASE_GROUNDED`), or only
once it is airborne? We currently assume from power-on, on the grounds that
`FLIGHT_PHASE_GROUNDED` would otherwise be a value that never appears on the
wire, which would make it a strange thing for the enum to contain.

Please confirm, since this also determines when the Network Remote ID relay to
Garuda Robotics starts receiving our telemetry.

---

## Q3 — Response expectations for Task 4 commands

`RxCommand` carries `assistance_request`, `keep_out_zone`, `all_clear`,
`moving_object_alert` and `readiness_confirm`. `RxReport` carries a matching
`incident_ack` and `readiness_report`.

1. Is there a **required response time** between receiving one of these commands
   and publishing the corresponding report?
2. Are `keep_out_zone` and `all_clear` intended as **broadcasts to the whole
   fleet**, or do they always name a single vehicle in `RxCommand.vehicle_id`?
   We currently treat an empty `vehicle_id` as fleet-wide, since a keep-out zone
   delivered to only one of three vehicles seems worse than useless.
3. Should the OCS **acknowledge receipt** of a command it cannot act on — for
   example a command addressed to a vehicle we did not declare for this run?

---

## Already answered by the handbook — do not re-ask

- **Is the 5 msg/s cap per team or per vehicle?** Per vehicle. §3.4 says "no more
  than five(5) messages per second **per system**". The USV repo's README flagged
  this as ambiguous and noted the team had asked; the published text settles it,
  and our rate governor already meters per vehicle. If a previous answer from
  RoboNation contradicts this, that answer wins — check the thread before acting
  on this note.
