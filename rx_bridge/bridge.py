"""bridge — the OCS's one connection to RoboCommand, and the rules around it.

The handbook permits the team exactly one OCS connection to RoboCommand and
forbids vehicles from reaching it at all. This process is that connection. It
holds a second, entirely separate connection to our own broker, where the
vehicles publish -- and it never routes between them. Those are two MQTT
clients on two networks, joined only by the code below.

THE HOT PATH, in the order it must happen:

    vehicle publishes RxReport (its own clock in sent_at)
      -> staleness check against that clock
      -> governor: may this class of message spend a token?
      -> validate: no UNKNOWN enums, no NaN
      -> seq: durable, per vehicle
      -> re-stamp sent_at from OUR clock
      -> publish, and log the frame

Two orderings in there are deliberate. The governor runs BEFORE the sequence
number so a dropped frame does not burn a seq and leave a hole that looks like
a lost message. And sent_at is re-stamped LAST, from the OCS clock, so a vehicle
with a skewed clock cannot corrupt the field RoboCommand judges us on -- the
vehicle's own timestamp survives only as the staleness measurement.
"""
from __future__ import annotations

import json
import threading
import time
from datetime import datetime, timezone
from collections import deque
from dataclasses import dataclass, field

from google.protobuf import json_format

import paho.mqtt.client as mqtt

from . import validate
from .active import ACTIVE_WINDOW_S, choose_active
from .config import Config
from .governor import Governor
from .proto import common_pb2, rx_commands_pb2, rx_common_pb2, rx_course_pb2
from .proto import rx_reports_pb2, rx_requests_pb2
from .runstate import RunMachine
from .seqstore import SeqStore
from .tcpserver import TcpServer
from .wirelog import WireLog


#: Handbook 3.1's floor, for the warning below. The value actually enforced
#: is cfg.min_declared, so a bench can lower it -- see config.py.
_POR_MIN_VEHICLES = 2

#: Window over which the published heartbeat rate is measured. Long enough
#: that one late frame does not read as a rate collapse, short enough that a
#: vehicle going quiet shows up while the operator can still act on it.
_HB_WINDOW_S = 10.0


class HeartbeatRate:
    """What we are ACTUALLY publishing per vehicle, in Hz.

    The handbook obliges the OCS to publish a heartbeat per active vehicle at
    2 Hz. Until now the bridge forwarded whatever arrived and nobody counted,
    so "are we meeting our obligation" was a question you answered by reading
    a wire log after the run. This answers it during the run.

    Measured on what we PUBLISHED, not on what the vehicle sent -- a frame
    the governor dropped or the validator refused did not reach RoboCommand,
    and counting it would report compliance we do not have.
    """

    def __init__(self) -> None:
        self._t: dict[str, deque] = {}

    def record(self, vehicle_id: str, now: float) -> None:
        d = self._t.setdefault(vehicle_id, deque())
        d.append(now)
        cut = now - _HB_WINDOW_S
        while d and d[0] < cut:
            d.popleft()

    def hz(self, vehicle_id: str, now: float) -> float:
        d = self._t.get(vehicle_id)
        if not d:
            return 0.0
        cut = now - _HB_WINDOW_S
        recent = [t for t in d if t >= cut]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0


@dataclass
class Counters:
    forwarded: int = 0
    dropped_rate: int = 0
    dropped_invalid: int = 0
    dropped_mistyped: int = 0
    stale: int = 0
    commands_routed: int = 0
    commands_undeliverable: int = 0
    last_command: str = ""
    last_seen: dict[str, float] = field(default_factory=dict)
    #: vehicle_id -> "tcp" | "mqtt". Which link a vehicle last spoke on, so
    #: the page can say so while both transports are wired up.
    last_via: dict[str, str] = field(default_factory=dict)


class Bridge:
    def __init__(self, cfg: Config, *, log=print) -> None:
        self.cfg = cfg
        self.log = log
        self.machine = RunMachine()
        self.seqs = SeqStore(cfg.seq_store)
        self.counters = Counters()
        self.hb_rate = HeartbeatRate()
        #: Findings from the last RxCourse. Advisory -- see validate.check_course.
        self.course_findings: list[str] = []
        #: True once every declared vehicle has been seen in STATE_AUTO, so the
        #: 'all AUTO but no RunStart' case is distinguishable from 'still
        #: waiting on a vehicle'. Logged on the edge, not every frame.
        self._all_auto = False
        #: Recent RoboCommand disconnects, for the duplicate-client check.
        self._rc_drops: deque = deque(maxlen=8)
        self._warned_duplicate = False
        #: Operator overrides for the active set: vehicle_id -> True (force
        #: in) / False (force out). Absent means 'decide from the telemetry'.
        #: Deliberately NOT persisted -- which vehicles are on the water is a
        #: fact about right now, and a stale answer restored from disk is
        #: worse than asking the fleet again.
        self._active_override: dict[str, bool] = {}
        self.wire = WireLog(cfg.wire_log, run_tag=cfg.team_id)

        # What the web page reads. Decoded with protobuf's own MessageToDict
        # rather than a hand-written field list, so every field the handbook
        # defines shows up and a new one from RoboNation appears on its own.
        self.latest: dict[str, dict] = {}       # vehicle_id -> last report
        self.recent: deque[dict] = deque(maxlen=50)   # commands in, newest first
        self.course: dict = {}
        self.links = {"robocommand": False, "team": False}

        # Pick the run back up if we are a restart rather than a cold start.
        # Durable counters without a durable run leave us with correct sequence
        # numbers and no right to publish anything.
        saved = self.seqs.run
        if saved.get("state"):
            v = self.machine.restore(
                str(saved["state"]),
                saved.get("declaration_seq"),  # type: ignore[arg-type]
                saved.get("run_id"),           # type: ignore[arg-type]
            )
            self.log("run store: " + v.reason)
            self.wire.event("restore", v.reason)

        # Per vehicle, because the 5/s cap is metered per vehicle. See
        # governor.py for the ambiguity in the handbook, and what to change if
        # RoboNation answers that it is per-team.
        self._gov = {
            v: Governor(rate=cfg.rate, reserve=cfg.heartbeat_reserve)
            for v in cfg.vehicle_ids
        }

        self._lock = threading.RLock()

        # clean_session=False + a STABLE client id is what makes the handbook's
        # "reconnect and restore subscriptions" cheap: the broker keeps our
        # session across a drop. We re-subscribe on every CONNACK regardless,
        # because trusting session-present to be true is how you end up
        # connected and deaf. (MQTT5 would use clean_start and a session expiry
        # here instead -- confirm which version their broker speaks.)
        self.rc = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="ocs-" + cfg.team_id,
            clean_session=False,
        )
        self.rc.on_connect = self._on_rc_connect
        self.rc.on_disconnect = self._on_rc_disconnect
        self.rc.on_message = self._on_rc_message

        self.team = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="ocs-team-" + cfg.team_id,
        )
        self.team.on_connect = self._on_team_connect
        self.team.on_message = self._on_team_message

        # The vehicle link. MQTT above stays wired up alongside it: both feed
        # the same _ingest_report, so the bench and the inject buttons keep
        # working as a regression gate while TCP is still unproven on hardware.
        self.tcp = (
            TcpServer(cfg.tcp_host, cfg.tcp_port,
                      on_frame=self._on_tcp_frame, log=self.log)
            if cfg.tcp_enabled else None
        )

    # ---- lifecycle --------------------------------------------------------

    def start(self) -> None:
        if self.tcp is not None:
            self.tcp.start()
        self.team.connect_async(
            self.cfg.team.host, self.cfg.team.port, self.cfg.team.keepalive
        )
        self.team.loop_start()
        self.rc.connect_async(
            self.cfg.robocommand.host,
            self.cfg.robocommand.port,
            self.cfg.robocommand.keepalive,
        )
        self.rc.loop_start()
        self.log(
            "bridge up: team=%s robocommand=%s"
            % (self.cfg.team.host, self.cfg.robocommand.host)
        )
        if self.cfg.min_declared < _POR_MIN_VEHICLES:
            # Every start, not once: a floor lowered for a bench and left
            # there is only caught by saying so until somebody fixes it.
            self.log(
                "WARNING: limits.min_declared = %d. Handbook 3.1 requires %d "
                "vehicles in the run_declaration -- rc-test logs from a run "
                "declared below that will not satisfy the Communications "
                "Proof of Readiness."
                % (self.cfg.min_declared, _POR_MIN_VEHICLES)
            )

    def stop(self) -> None:
        if self.tcp is not None:
            self.tcp.stop()
        for c in (self.rc, self.team):
            c.loop_stop()
            c.disconnect()
        self.wire.close()

    # ---- RoboCommand side -------------------------------------------------

    def _on_rc_connect(self, client, _ud, _flags, reason, _props=None) -> None:
        if reason != 0:
            self.log("RoboCommand refused the connection: %s" % reason)
            return
        # Unconditionally, every time. See the note in __init__.
        client.subscribe(
            [(self.cfg.rc_course_sub(), 1), (self.cfg.rc_command_sub(), 1)]
        )
        with self._lock:
            v = self.machine.on_connect()
        self.links["robocommand"] = True
        self._on_rc_connect_stable()
        self.log("RoboCommand: " + v.reason)
        self.wire.event("connect", v.reason)

    def _on_rc_disconnect(self, _client, _ud, _flags, reason, _props=None) -> None:
        with self._lock:
            v = self.machine.on_disconnect()
        self.links["robocommand"] = False
        self.log("RoboCommand lost (%s): %s" % (reason, v.reason))
        self.wire.event("disconnect", "%s: %s" % (reason, v.reason))
        self._check_duplicate_ocs()

    def _check_duplicate_ocs(self) -> None:
        """Explain a flapping RoboCommand link, once.

        Our MQTT client id is stable by design -- it is what lets the broker
        keep our session across a drop, which the handbook asks for. The cost
        is that a SECOND client using the same id takes the session away, and
        the two then kick each other off forever. On the wire that looks like
        an unstable network, and people go and re-crimp a perfectly good
        cable. It is not the network; it is two OCS instances.

        Not repaired automatically, deliberately. Giving each instance a
        unique id would let both stay connected, and the handbook permits the
        team exactly one OCS connection -- so the fight is the rule being
        enforced. The fix is to stop the other instance, which needs a human.
        """
        now = time.monotonic()
        self._rc_drops.append(now)
        recent = [t for t in self._rc_drops if now - t <= 30.0]
        if len(recent) >= 3 and not self._warned_duplicate:
            self._warned_duplicate = True
            detail = (
                "RoboCommand link has dropped %d times in 30 s. This is "
                "almost never the network: another client is connecting as "
                "\"ocs-%s\" and taking the session. Find the other OCS "
                "instance and stop it -- the handbook permits the team one "
                "connection." % (len(recent), self.cfg.team_id)
            )
            self.log("DUPLICATE OCS? " + detail)
            self.wire.event("duplicate_ocs", detail)

    def _on_rc_connect_stable(self) -> None:
        """Clear the warning latch after a spell of stability."""
        now = time.monotonic()
        if self._warned_duplicate and not [
            t for t in self._rc_drops if now - t <= 60.0
        ]:
            self._warned_duplicate = False

    def _on_rc_message(self, _client, _ud, msg) -> None:
        self.wire.frame("rx", msg.topic, msg.payload)
        if msg.topic == self.cfg.rc_course_sub():
            self._handle_course(msg.payload)
        elif msg.topic == self.cfg.rc_command_sub():
            self._handle_command(msg.payload)

    def _handle_course(self, payload: bytes) -> None:
        course = rx_course_pb2.RxCourse()
        try:
            course.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001
            self.log("undecodable RxCourse: %s" % exc)
            return
        # Validate before advancing. Advisory only: a bridge that refuses to
        # leave CONNECTED because it dislikes their course cannot declare at
        # all, and their frame is authoritative even when it is odd. Say so
        # loudly instead, in the window where a person can still act on it.
        findings = validate.check_course(course)
        with self._lock:
            self.course_findings = [str(f) for f in findings]
            v = self.machine.on_course(course)
        self.log(
            "course %s: pinger %d Hz, %d corners -- %s"
            % (course.course_id, course.pinger_freq_hz, len(course.corners), v.reason)
        )
        if findings:
            detail = "; ".join(str(f) for f in findings)
            self.log("COURSE SUSPECT: %s -- accepted anyway; check before "
                     "declaring" % detail)
            self.wire.event("course_suspect", detail)
        self.course = _decode(course)

    def _handle_command(self, payload: bytes) -> None:
        cmd = rx_commands_pb2.RxCommand()
        try:
            cmd.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001
            self.log("undecodable RxCommand: %s" % exc)
            return

        which = cmd.WhichOneof("body")
        if which == "run_start":
            with self._lock:
                v = self.machine.on_run_start(
                    cmd.run_start.declaration_seq, cmd.run_start.run_id
                )
                if v.ok:
                    self.seqs.save_run(self.machine.state.name,
                                       self.machine.declaration_seq,
                                       self.machine.run_id)
            # A refusal here is the loudest thing this process can say: it means
            # RoboCommand answered a declaration that is not the one we hold.
            self.log(("RUN START: " if v.ok else "RUN START REFUSED: ") + v.reason)
            self.wire.event("run_start", v.reason)

            # The vehicles are position-holding in autonomous mode and cannot
            # reach RoboCommand themselves, so RunStart is the only way they
            # learn the run began -- relay it, or three vehicles hold station
            # through the whole window.
            #
            # Only when v.ok. A refused RunStart answers somebody else's
            # declaration; relaying it would start our fleet on a run we were
            # never granted.
            if v.ok:
                self._route_command(cmd, which)
        else:
            # Task 4 lands here, and the handbook requires the OCS to route it
            # to the vehicle it names. It is advisory input for the mission
            # executive and is NEVER allowed to actuate anything -- see the
            # README. Routing it is not actuating it: the vehicle side decides
            # what, if anything, to do, and the RC e-stop remains the only
            # safety path.
            self._route_command(cmd, which)

    def _route_command(self, cmd, which: str) -> None:
        """Hand an inbound RxCommand to the vehicle named in it.

        RxCommand carries its own `vehicle_id`. An unaddressed command is a
        broadcast to the whole fleet -- KeepOutZone and AllClear are the
        obvious cases, and sending those to only one vehicle would be worse
        than dropping them.
        """
        targets: tuple[str, ...]
        if cmd.vehicle_id:
            if self.cfg.vehicle(cmd.vehicle_id) is None:
                # Do not invent a topic for a vehicle we do not fly. A typo on
                # their side becoming a subscription nobody reads is exactly
                # the kind of quiet that costs a task.
                self.counters.commands_undeliverable += 1
                self.log(
                    "COMMAND UNDELIVERABLE: %s addressed to %s, which is not "
                    "in the fleet" % (which, cmd.vehicle_id)
                )
                self.wire.event(
                    "command_undeliverable", "%s -> %s" % (which, cmd.vehicle_id)
                )
                self.recent.appendleft({"at": time.time(), "which": which,
                                        "seq": cmd.seq, "targets": [],
                                        "delivered": False,
                                        "detail": "%s is not in the fleet"
                                                  % cmd.vehicle_id,
                                        "body": _decode(cmd)})
                return
            targets = (cmd.vehicle_id,)
        else:
            targets = self.cfg.vehicle_ids

        payload = cmd.SerializeToString()
        as_json = _to_json(cmd)
        routes: list[str] = []
        for vid in targets:
            # The vehicle link first: a vehicle holding a TCP connection is
            # listening on that socket, and it is the one that will still
            # exist once the broker is gone. MQTT below is the fallback while
            # both transports are wired up.
            if self.tcp is not None and self.tcp.send(vid, as_json):
                routes.append("%s/tcp" % vid)
                self.wire.frame("tx", "tcp://%s" % vid, as_json,
                                note="%s seq=%d" % (which, cmd.seq))
                continue
            topic = self.cfg.team_command_topic(vid)
            # QoS 1 without exception. Unlike a heartbeat, a command does not
            # come round again in 500 ms -- if it is dropped it is simply gone,
            # and the operator is left waiting on a response to something the
            # vehicle never heard.
            self.team.publish(topic, payload, qos=1)
            routes.append("%s/mqtt" % vid)
            self.wire.frame("tx", topic, payload, note="%s seq=%d" % (which, cmd.seq))

        with self._lock:
            self.counters.commands_routed += 1
            self.counters.last_command = "%s -> %s" % (which, ", ".join(routes))
            self.recent.appendleft({"at": time.time(), "which": which,
                                    "seq": cmd.seq, "targets": list(targets),
                                    "routes": routes,
                                    "delivered": True, "body": _decode(cmd)})
        self.log("COMMAND %s (seq=%d) -> %s" % (which, cmd.seq, ", ".join(routes)))

    # ---- vehicle side, the hot path --------------------------------------

    def _on_team_connect(self, client, _ud, _flags, reason, _props=None) -> None:
        if reason != 0:
            self.log("team broker refused the connection: %s" % reason)
            return
        self.links["team"] = True
        client.subscribe(self.cfg.team_report_sub(), 0)
        self.log("team broker: subscribed " + self.cfg.team_report_sub())

    def _on_team_message(self, _client, _ud, msg) -> None:
        report = rx_reports_pb2.RxReport()
        try:
            report.ParseFromString(msg.payload)
        except Exception as exc:  # noqa: BLE001
            self.log("undecodable RxReport on %s: %s" % (msg.topic, exc))
            return

        topic_vehicle = _vehicle_from_topic(msg.topic)
        if topic_vehicle and topic_vehicle != report.vehicle_id:
            # The handbook requires the topic's vehicle_id to match the message's.
            self.log("DROP: topic says %s, message says %s"
                     % (topic_vehicle, report.vehicle_id))
            self.wire.event("mismatch",
                            "%s vs %s" % (msg.topic, report.vehicle_id))
            return
        self._ingest_report(report, via="mqtt")

    def _on_tcp_frame(self, payload: bytes) -> str | None:
        """One framed JSON RxReport off the vehicle link.

        Returns the vehicle_id, which is what tcpserver files the connection
        under. Returning None for a frame we could not read leaves that socket
        anonymous -- correct, because we have not learned who is on it, and
        guessing would route somebody else's commands down it.

        JSON in, protobuf out. The bridge re-stamps seq and sent_at and
        re-serializes every report regardless, so the format arriving from a
        vehicle was never coupled to the one RoboCommand receives. That is what
        lets the boat carry no protobuf at all.
        """
        self.wire.frame("veh", "tcp://in", payload)
        report = rx_reports_pb2.RxReport()
        try:
            json_format.ParseDict(json.loads(payload.decode("utf-8")), report)
        except (UnicodeDecodeError, json.JSONDecodeError,
                json_format.ParseError) as exc:
            # ParseError names the offending field, which is most of what goes
            # wrong while a reporter is being written: a misspelled key, or an
            # enum spelled as something that is not one of its values.
            #
            # NOTE a bare NaN lands here too -- protobuf's JSON mapping requires
            # non-finite floats as the quoted string "NaN". The vehicle sends
            # them that way; see ocs_link.py in rx26_asv.
            self.log("undecodable RxReport on the vehicle link: %s" % exc)
            self.wire.event("undecodable", str(exc))
            return None
        self._ingest_report(report, via="tcp")
        return report.vehicle_id or None

    def _ingest_report(self, report, *, via: str) -> None:
        """The hot path, from either transport.

        Everything from here down is deliberately transport-agnostic: staleness,
        the governor, validation, sequencing and the re-stamp are rules about the
        message, not about the socket it arrived on. That separation is the
        reason adding TCP did not require touching governor.py, seqstore.py or
        validate.py at all.
        """
        vehicle = report.vehicle_id
        spec = self.cfg.vehicle(vehicle)
        if spec is None:
            self.log("DROP: %s is not a configured vehicle" % vehicle)
            return

        now = time.time()
        self.counters.last_seen[vehicle] = now
        self.counters.last_via[vehicle] = via
        body = report.WhichOneof("body")
        # Before the gate below: the bridge is correctly silent toward
        # RoboCommand until we declare, but the operator still needs to see the
        # fleet then -- that is the window in which a sick vehicle can still be
        # fixed.
        self.latest[vehicle] = _decode(report)

        with self._lock:
            if not self.machine.may_report():
                return  # pre-declaration silence is correct, not an error

            is_hb = body == "heartbeat"

            if report.HasField("sent_at"):
                age_ms = (now - report.sent_at.ToNanoseconds() / 1e9) * 1000.0
                # Per vehicle: an acoustic hop cannot meet a WiFi deadline, and
                # a UUV warning twice a second would bury the USV warning that
                # actually means something.
                if age_ms > self.cfg.stale_ms_for(vehicle):
                    # Forward it anyway -- a late position beats no position --
                    # but never silently. Silence here would let a wedged
                    # vehicle look healthy all the way through a run.
                    self.counters.stale += 1
                    self.log("STALE %s: %.0f ms old" % (vehicle, age_ms))

            if not self._gov[vehicle].allow(time.monotonic(), heartbeat=is_hb):
                self.counters.dropped_rate += 1
                self.wire.event("rate_drop", "%s %s" % (vehicle, body))
                return

            # A heartbeat that disagrees with the config about what kind of
            # craft this is means a reporter was copy-pasted between vehicles.
            # It serializes fine and scores as the wrong vehicle, so it has to
            # die here -- and it must be checked BEFORE validate(), because the
            # type is what selects the validation policy below. Trusting the
            # message's own type to choose its own leniency would let a
            # mislabelled UAV buy itself the surface-vehicle exemption.
            if is_hb:
                declared = rx_common_pb2.VehicleType.Value(spec.type)
                if report.heartbeat.vehicle_type != declared:
                    actual = rx_common_pb2.VehicleType.Name(
                        report.heartbeat.vehicle_type
                    )
                    self.counters.dropped_mistyped += 1
                    detail = "%s is configured %s but reported %s" % (
                        vehicle, spec.type, actual,
                    )
                    self.log("DROP: " + detail)
                    self.wire.event("mistyped", detail)
                    return

            findings = validate.check(report, allow_unknown=spec.allow_unknown)
            if findings:
                self.counters.dropped_invalid += 1
                detail = "; ".join(str(f) for f in findings)
                self.log("DROP %s: %s" % (vehicle, detail))
                self.wire.event("invalid", detail)
                return

            report.seq = self.seqs.next_report(vehicle)
            report.team_id = self.cfg.team_id
            report.sent_at.FromNanoseconds(time.time_ns())

            payload = report.SerializeToString()
            seq = report.seq
            self.counters.forwarded += 1
            if is_hb:
                self.hb_rate.record(vehicle, now)
            # Record it again now that seq and sent_at are ours, so the page
            # shows the frame RoboCommand actually received rather than the
            # vehicle's draft of it. The earlier write above is what keeps a
            # dropped or pre-declaration frame visible at all.
            self.latest[vehicle] = _decode(report)

        topic = self.cfg.rc_report_topic(vehicle)
        self.rc.publish(topic, payload, qos=0 if is_hb else 1)
        self.wire.frame("tx", topic, payload, note="seq=%d %s" % (seq, body))
        if is_hb:
            self._note_auto_edge()

    def awaiting_auto(self) -> list[str]:
        """Declared vehicles not yet seen in STATE_AUTO.

        RoboCommand publishes RunStart only once every vehicle in the
        declaration reports STATE_AUTO -- with no timeout and no error, so a
        fleet stuck one vehicle short looks exactly like a broken link to
        their server. This names the vehicle, which turns a silent wait into
        something the operator can act on.
        """
        out = []
        for spec in self.cfg.vehicles:
            hb = (self.latest.get(spec.id) or {}).get("heartbeat") or {}
            if hb.get("state") != "STATE_AUTO":
                out.append(spec.id)
        return out

    def _note_auto_edge(self) -> None:
        """Log the moment the fleet becomes fully AUTO, once.

        That instant is when RunStart becomes their side's responsibility. If
        it does not arrive after this line, the problem is not our fleet.
        """
        waiting = self.awaiting_auto()
        if not waiting and not self._all_auto:
            self._all_auto = True
            self.log("all declared vehicles report STATE_AUTO -- RunStart is "
                     "now RoboCommand's move")
            self.wire.event("all_auto", ", ".join(self.cfg.vehicle_ids))
        elif waiting and self._all_auto:
            self._all_auto = False
            self.log("no longer all AUTO -- waiting on %s" % ", ".join(waiting))

    def active_ids(self, now: float | None = None) -> tuple[str, ...]:
        """Which vehicles go into the declaration, in config order.

        A vehicle is active if it has reported within _ACTIVE_WINDOW_S, unless
        the operator has forced it in or out.

        THIS IS ON THE CRITICAL PATH FOR THE RUN, not a display nicety.
        RoboCommand publishes RunStart only once EVERY declared vehicle has
        reported STATE_AUTO -- with no timeout and no error. Declare a vehicle
        that is not on the water and the run never starts, and the failure
        looks like a broken link to their server rather than like a fleet one
        vehicle short.
        """
        return choose_active(
            self.cfg.vehicle_ids,
            self._active_override,
            self.counters.last_seen,
            time.time() if now is None else now,
        )

    def set_active(self, vehicle_id: str, on: bool | None) -> str:
        """Force a vehicle in (True) or out (False) of the declaration.

        None returns it to being decided by whether it is reporting. Forcing a
        vehicle IN is legitimate -- a reporter that is about to come up, or a
        vehicle relayed by another -- so this is not merely an exclude list.
        """
        if self.cfg.vehicle(vehicle_id) is None:
            return "%s is not a configured vehicle" % vehicle_id
        with self._lock:
            if on is None:
                self._active_override.pop(vehicle_id, None)
            else:
                self._active_override[vehicle_id] = bool(on)
            active = self.active_ids()
        word = "auto" if on is None else ("forced IN" if on else "forced OUT")
        self.log("active set: %s %s -- declaring %s"
                 % (vehicle_id, word, ", ".join(active) or "nothing"))
        return "ok"

    def _send_standby_auto(self, vehicles: tuple[str, ...], seq: int) -> None:
        """Tell the declared vehicles to stand by in auto and await RunStart.

        ADVISORY, and it has to stay that way. This does not change anybody's
        flight mode: the standing rule in both vehicle repos is that the RC
        e-stop is the only safety path and WiFi is never a safety mechanism.
        The vehicle's mission planner reads this and decides; a vehicle that
        RC has not armed must ignore it.

        Its own message type, not a field bolted onto RxCommand -- that schema
        is RoboNation's and must not grow team-local inventions.
        """
        if self.tcp is None:
            return
        body = json.dumps({"ocs_directive": {
            "action": "standby_auto",
            "declaration_seq": seq,
            "vehicles": list(vehicles),
            "sent_at": _rfc3339(),
        }}).encode("utf-8")
        reached, missed = [], []
        for vid in vehicles:
            if self.tcp.send(vid, body):
                reached.append(vid)
                self.wire.frame("tx", "tcp://%s" % vid, body,
                                note="standby_auto seq=%d" % seq)
            else:
                missed.append(vid)
        self.log("standby_auto -> %s%s"
                 % (", ".join(reached) or "nobody",
                    "" if not missed
                    else "  (NOT DELIVERED: %s -- no vehicle link)"
                         % ", ".join(missed)))
        self.wire.event("standby_auto", "sent=%s missed=%s"
                        % (",".join(reached), ",".join(missed)))

    # ---- operator actions -------------------------------------------------

    def declare(self) -> str:
        """Publish the RunDeclaration. This is what creates the obligations."""
        with self._lock:
            if not self.machine.may_declare():
                if self.machine.state.name == "RUNNING":
                    return ("cannot declare during a run -- end it first, "
                            "then declare the next one")
                return (
                    "cannot declare in %s -- need the retained RxCourse first"
                    % self.machine.state.name
                )

            # Only what is actually on the water. See active_ids().
            active = self.active_ids()
            if len(active) < self.cfg.min_declared:
                return (
                    "cannot declare %d vehicle(s) -- need at least %d in the "
                    "run_declaration (handbook 3.1, for the Communications "
                    "Proof of Readiness). Active: %s. Start a reporter, or "
                    "force one in from the page."
                    % (len(active), self.cfg.min_declared,
                       ", ".join(active) or "nothing")
                )

            # ensure_, not new_: a second run in the same timeslot must NOT
            # rewind the request counter, or its declaration reuses seq=1 and
            # a stale RunStart from the first run matches it. See seqstore.
            self.seqs.ensure_epoch("%s-%d" % (self.cfg.team_id, int(time.time())))

            req = rx_requests_pb2.RxRequest()
            req.team_id = self.cfg.team_id
            req.seq = self.seqs.next_request()
            req.sent_at.FromNanoseconds(time.time_ns())

            decl = req.run_declaration
            decl.vehicle_ids.extend(active)
            tier = common_pb2.TaskTier
            decl.task1_tier = tier.Value(self.cfg.tiers[0])
            decl.task2_tier = tier.Value(self.cfg.tiers[1])
            decl.task3_tier = tier.Value(self.cfg.tiers[2])
            decl.task4_tier = tier.Value(self.cfg.tiers[3])
            for lat, lon in self.cfg.uav_geofence:
                decl.uav_geofence.add(latitude=lat, longitude=lon)

            # Nothing in a declaration has an honest UNKNOWN: there are no
            # per-vehicle exemptions to inherit here, only four tiers that must
            # each be a real value. So the allowance is empty, deliberately,
            # rather than borrowed from some vehicle's policy.
            findings = validate.check(req, allow_unknown=())
            if findings:
                return "declaration REFUSED: " + "; ".join(str(f) for f in findings)

            payload = req.SerializeToString()
            seq = req.seq
            v = self.machine.on_declared(seq)
            # A new run needs its own readiness edge, or the "all AUTO"
            # line from the previous run suppresses this run's.
            self._all_auto = False
            self.seqs.save_run(self.machine.state.name,
                               self.machine.declaration_seq, self.machine.run_id)

        self.rc.publish(self.cfg.rc_request_topic(), payload, qos=1)
        self.wire.frame(
            "tx", self.cfg.rc_request_topic(), payload,
            note="RunDeclaration seq=%d [%s]" % (seq, ",".join(active)),
        )
        # After the declaration is on the wire, not before: the vehicles are
        # being told to stand by for a run we have actually declared.
        self._send_standby_auto(active, seq)
        return "%s; declared %s" % (v.reason, ", ".join(active))

    def end(self) -> str:
        with self._lock:
            v = self.machine.on_end()
            if v.ok:
                self.seqs.save_run(self.machine.state.name,
                                   self.machine.declaration_seq, self.machine.run_id)
            return v.reason

    def snapshot(self) -> dict:
        """Everything the web page shows, in one read."""
        now = time.time()
        # Read outside the bridge lock. tcpserver holds its own, and always
        # taking them bridge-then-tcp is cheaper to keep true than an
        # argument about why no inversion can happen.
        peers = self.tcp.peers() if self.tcp is not None else {}
        with self._lock:
            c, m = self.counters, self.machine
            active_now = self.active_ids(now)
            return {
                "now": now,
                "team_id": self.cfg.team_id,
                "links": dict(self.links),
                # The addresses, not just up/down. "DOWN" alone sends people
                # hunting through config files; "DOWN 192.168.65.2:1883" tells
                # them whether the OCS is even aiming at the right box.
                "endpoints": {
                    "robocommand": "%s:%d" % (self.cfg.robocommand.host,
                                              self.cfg.robocommand.port),
                    "team": "%s:%d" % (self.cfg.team.host, self.cfg.team.port),
                    "tcp": ("%s:%d" % (self.cfg.tcp_host, self.cfg.tcp_port)
                            if self.tcp is not None else "off"),
                },
                "tcp_peers": peers,
                "run": {"state": m.state.name,
                        "declaration_seq": m.declaration_seq,
                        "run_id": m.run_id,
                        "runs_declared": m.runs_declared,
                        "may_declare": m.may_declare()},
                "course": self.course,
                "course_findings": list(self.course_findings),
                "active": list(self.active_ids(now)),
                "por_min_vehicles": self.cfg.min_declared,
                "awaiting_auto": [
                    v.id for v in self.cfg.vehicles
                    if ((self.latest.get(v.id) or {}).get("heartbeat") or {})
                    .get("state") != "STATE_AUTO"
                ],
                "vehicles": [
                    {"id": v.id, "type": v.type, "relay_via": v.relay_via,
                     "stale_ms": self.cfg.stale_ms_for(v.id),
                     "silent_s": (None if v.id not in c.last_seen
                                  else now - c.last_seen[v.id]),
                     # "tcp" only while the socket is actually up; last_via
                     # is what it last spoke on, which is the more useful
                     # thing to show once it has gone quiet.
                     "link": ("tcp" if v.id in peers
                              else c.last_via.get(v.id, "")),
                     "peer": peers.get(v.id, ""),
                     "active": v.id in active_now,
                     "active_forced": self._active_override.get(v.id),
                     # What we are actually PUBLISHING for this vehicle. The
                     # handbook obliges 2 Hz; this is the measurement, not
                     # the intention.
                     "published_hz": round(self.hb_rate.hz(v.id, now), 2),
                     "report": self.latest.get(v.id, {})}
                    for v in self.cfg.vehicles
                ],
                "commands": list(self.recent),
                "counters": {"forwarded": c.forwarded,
                             "dropped_rate": c.dropped_rate,
                             "dropped_invalid": c.dropped_invalid,
                             "dropped_mistyped": c.dropped_mistyped,
                             "stale": c.stale,
                             "commands_routed": c.commands_routed,
                             "commands_undeliverable": c.commands_undeliverable},
            }

    def status(self) -> str:
        peers = self.tcp.peers() if self.tcp is not None else {}
        with self._lock:
            c, m = self.counters, self.machine
            now = time.time()
            lines = ["state      " + m.state.name]
            if m.declaration_seq is not None:
                lines[0] += "  (declaration_seq=%d)" % m.declaration_seq
            if m.run_id is not None:
                lines[0] += "  run_id=%d" % m.run_id

            # Every configured vehicle gets a line whether or not it has ever
            # been heard from. A fleet view that lists only what is talking
            # cannot show the one thing the operator most needs to notice --
            # the vehicle that has said nothing at all.
            for spec in self.cfg.vehicles:
                seen = c.last_seen.get(spec.id)
                age = "never heard" if seen is None else "%.1fs ago" % (now - seen)
                relay = " via %s" % spec.relay_via if spec.relay_via else ""
                link = peers.get(spec.id) or c.last_via.get(spec.id, "-")
                hz = self.hb_rate.hz(spec.id, now)
                lines.append(
                    "  %-6s %-9s %-14s %5.2f Hz  %-22s%s"
                    % (spec.id, spec.type.removeprefix("TYPE_"), age, hz,
                       link, relay)
                )

            lines += [
                "forwarded  %d" % c.forwarded,
                "dropped    %d rate, %d invalid, %d mistyped"
                % (c.dropped_rate, c.dropped_invalid, c.dropped_mistyped),
                "stale      %d" % c.stale,
                "commands   %d routed, %d undeliverable%s"
                % (
                    c.commands_routed,
                    c.commands_undeliverable,
                    "  last: " + c.last_command if c.last_command else "",
                ),
                "course     %s" % (
                    "not received" if not self.course
                    else "%s, %d corners%s" % (
                        self.course.get("course_id", "?"),
                        len(self.course.get("corners", [])),
                        "" if not self.course_findings
                        else "  SUSPECT: " + "; ".join(self.course_findings),
                    )
                ),
                "active     %s%s" % (
                    ", ".join(self.active_ids()) or "nothing",
                    "" if len(self.active_ids()) >= self.cfg.min_declared
                    else "   << below the %d required to declare"
                         % self.cfg.min_declared,
                ),
                "awaiting   %s" % (
                    ", ".join(self.awaiting_auto()) or "nothing -- all AUTO"
                ),
                "vehicle link %s" % (
                    "off" if self.tcp is None
                    else "%s:%d, %d connected" % (self.cfg.tcp_host,
                                                 self.cfg.tcp_port, len(peers))
                ),
                "seq        %s" % (self.seqs.snapshot(),),
                "wire log   %s" % self.wire.path,
            ]
            return "\n".join(lines)


def _decode(msg) -> dict:
    """Protobuf -> dict, keeping the handbook's own field names so the page and
    the spec can be read side by side."""
    return json_format.MessageToDict(
        msg, always_print_fields_with_no_presence=True,
        preserving_proto_field_name=True,
    )


def _rfc3339() -> str:
    """Now, in the form protobuf and the vehicles both parse."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def _to_json(msg) -> bytes:
    """Protobuf -> the JSON bytes the vehicle link carries.

    protobuf's own encoder rather than json.dumps, because it is the half
    that already knows the convention for non-finite floats: they go out as
    the quoted string "NaN", which is exactly what ParseDict demands coming
    back the other way. Hand-rolling this is how two directions drift apart.

    preserving_proto_field_name so the boat's keys read the same as the
    handbook's field names.
    """
    return json_format.MessageToJson(
        msg, preserving_proto_field_name=True, indent=None
    ).encode("utf-8")


def _vehicle_from_topic(topic: str) -> str:
    """team/robotx/<team>/<vehicle>/report -> <vehicle>."""
    parts = topic.split("/")
    return parts[-2] if len(parts) >= 2 and parts[-1] == "report" else ""
