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

import threading
import time
from collections import deque
from dataclasses import dataclass, field

from google.protobuf import json_format

import paho.mqtt.client as mqtt

from . import validate
from .config import Config
from .governor import Governor
from .proto import common_pb2, rx_commands_pb2, rx_common_pb2, rx_course_pb2
from .proto import rx_reports_pb2, rx_requests_pb2
from .runstate import RunMachine
from .seqstore import SeqStore
from .wirelog import WireLog


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


class Bridge:
    def __init__(self, cfg: Config, *, log=print) -> None:
        self.cfg = cfg
        self.log = log
        self.machine = RunMachine()
        self.seqs = SeqStore(cfg.seq_store)
        self.counters = Counters()
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

    # ---- lifecycle --------------------------------------------------------

    def start(self) -> None:
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

    def stop(self) -> None:
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
        self.log("RoboCommand: " + v.reason)
        self.wire.event("connect", v.reason)

    def _on_rc_disconnect(self, _client, _ud, _flags, reason, _props=None) -> None:
        with self._lock:
            v = self.machine.on_disconnect()
        self.links["robocommand"] = False
        self.log("RoboCommand lost (%s): %s" % (reason, v.reason))
        self.wire.event("disconnect", "%s: %s" % (reason, v.reason))

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
        with self._lock:
            v = self.machine.on_course(course)
        self.log(
            "course %s: pinger %d Hz, %d corners -- %s"
            % (course.course_id, course.pinger_freq_hz, len(course.corners), v.reason)
        )
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
        for vid in targets:
            topic = self.cfg.team_command_topic(vid)
            # QoS 1 without exception. Unlike a heartbeat, a command does not
            # come round again in 500 ms -- if it is dropped it is simply gone,
            # and the operator is left waiting on a response to something the
            # vehicle never heard.
            self.team.publish(topic, payload, qos=1)
            self.wire.frame("tx", topic, payload, note="%s seq=%d" % (which, cmd.seq))

        with self._lock:
            self.counters.commands_routed += 1
            self.counters.last_command = "%s -> %s" % (which, ", ".join(targets))
            self.recent.appendleft({"at": time.time(), "which": which,
                                    "seq": cmd.seq, "targets": list(targets),
                                    "delivered": True, "body": _decode(cmd)})
        self.log("COMMAND %s (seq=%d) -> %s" % (which, cmd.seq, ", ".join(targets)))

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

        vehicle = report.vehicle_id
        topic_vehicle = _vehicle_from_topic(msg.topic)
        if topic_vehicle and topic_vehicle != vehicle:
            # The handbook requires the topic's vehicle_id to match the message's.
            self.log("DROP: topic says %s, message says %s" % (topic_vehicle, vehicle))
            self.wire.event("mismatch", "%s vs %s" % (msg.topic, vehicle))
            return
        spec = self.cfg.vehicle(vehicle)
        if spec is None:
            self.log("DROP: %s is not a configured vehicle" % vehicle)
            return

        now = time.time()
        self.counters.last_seen[vehicle] = now
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
            # Record it again now that seq and sent_at are ours, so the page
            # shows the frame RoboCommand actually received rather than the
            # vehicle's draft of it. The earlier write above is what keeps a
            # dropped or pre-declaration frame visible at all.
            self.latest[vehicle] = _decode(report)

        topic = self.cfg.rc_report_topic(vehicle)
        self.rc.publish(topic, payload, qos=0 if is_hb else 1)
        self.wire.frame("tx", topic, payload, note="seq=%d %s" % (seq, body))

    # ---- operator actions -------------------------------------------------

    def declare(self) -> str:
        """Publish the RunDeclaration. This is what creates the obligations."""
        with self._lock:
            if not self.machine.may_declare():
                return (
                    "cannot declare in %s -- need the retained RxCourse first"
                    % self.machine.state.name
                )

            self.seqs.new_epoch("%s-%d" % (self.cfg.team_id, int(time.time())))

            req = rx_requests_pb2.RxRequest()
            req.team_id = self.cfg.team_id
            req.seq = self.seqs.next_request()
            req.sent_at.FromNanoseconds(time.time_ns())

            decl = req.run_declaration
            decl.vehicle_ids.extend(self.cfg.vehicle_ids)
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
            self.seqs.save_run(self.machine.state.name,
                               self.machine.declaration_seq, self.machine.run_id)

        self.rc.publish(self.cfg.rc_request_topic(), payload, qos=1)
        self.wire.frame(
            "tx", self.cfg.rc_request_topic(), payload,
            note="RunDeclaration seq=%d" % seq,
        )
        return v.reason

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
        with self._lock:
            c, m = self.counters, self.machine
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
                },
                "run": {"state": m.state.name,
                        "declaration_seq": m.declaration_seq,
                        "run_id": m.run_id},
                "course": self.course,
                "vehicles": [
                    {"id": v.id, "type": v.type, "relay_via": v.relay_via,
                     "stale_ms": self.cfg.stale_ms_for(v.id),
                     "silent_s": (None if v.id not in c.last_seen
                                  else now - c.last_seen[v.id]),
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
                lines.append(
                    "  %-6s %-9s %s%s"
                    % (spec.id, spec.type.removeprefix("TYPE_"), age, relay)
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


def _vehicle_from_topic(topic: str) -> str:
    """team/robotx/<team>/<vehicle>/report -> <vehicle>."""
    parts = topic.split("/")
    return parts[-2] if len(parts) >= 2 and parts[-1] == "report" else ""
