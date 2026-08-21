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
from dataclasses import dataclass, field

import paho.mqtt.client as mqtt

from . import validate
from .config import Config
from .governor import Governor
from .proto import common_pb2, rx_commands_pb2, rx_course_pb2
from .proto import rx_reports_pb2, rx_requests_pb2
from .runstate import RunMachine
from .seqstore import SeqStore
from .wirelog import WireLog


@dataclass
class Counters:
    forwarded: int = 0
    dropped_rate: int = 0
    dropped_invalid: int = 0
    stale: int = 0
    last_seen: dict[str, float] = field(default_factory=dict)


class Bridge:
    def __init__(self, cfg: Config, *, log=print) -> None:
        self.cfg = cfg
        self.log = log
        self.machine = RunMachine()
        self.seqs = SeqStore(cfg.seq_store)
        self.counters = Counters()
        self.wire = WireLog(cfg.wire_log, run_tag=cfg.team_id)

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
        self.log("RoboCommand: " + v.reason)
        self.wire.event("connect", v.reason)

    def _on_rc_disconnect(self, _client, _ud, _flags, reason, _props=None) -> None:
        with self._lock:
            v = self.machine.on_disconnect()
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
        else:
            # Task 4 lands here. It is advisory input for the mission executive
            # and is NEVER allowed to actuate anything -- see the README.
            self.log("command: %s (seq=%d) -- not yet handled" % (which, cmd.seq))
            self.wire.event("command", "%s seq=%d" % (which, cmd.seq))

    # ---- vehicle side, the hot path --------------------------------------

    def _on_team_connect(self, client, _ud, _flags, reason, _props=None) -> None:
        if reason != 0:
            self.log("team broker refused the connection: %s" % reason)
            return
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
        if vehicle not in self._gov:
            self.log("DROP: %s is not in vehicle_ids" % vehicle)
            return

        now = time.time()
        self.counters.last_seen[vehicle] = now
        body = report.WhichOneof("body")

        with self._lock:
            if not self.machine.may_report():
                return  # pre-declaration silence is correct, not an error

            is_hb = body == "heartbeat"

            if report.HasField("sent_at"):
                age_ms = (now - report.sent_at.ToNanoseconds() / 1e9) * 1000.0
                if age_ms > self.cfg.stale_ms:
                    # Forward it anyway -- a late position beats no position --
                    # but never silently. Silence here would let a wedged
                    # vehicle look healthy all the way through a run.
                    self.counters.stale += 1
                    self.log("STALE %s: %.0f ms old" % (vehicle, age_ms))

            if not self._gov[vehicle].allow(time.monotonic(), heartbeat=is_hb):
                self.counters.dropped_rate += 1
                self.wire.event("rate_drop", "%s %s" % (vehicle, body))
                return

            findings = validate.check(report, allow_unknown=self.cfg.allow_unknown)
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

            findings = validate.check(req, allow_unknown=self.cfg.allow_unknown)
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

    def status(self) -> str:
        with self._lock:
            c, m = self.counters, self.machine
            now = time.time()
            seen = ", ".join(
                "%s:%.1fs" % (v, now - t) for v, t in sorted(c.last_seen.items())
            ) or "nothing heard"
            lines = ["state      " + m.state.name]
            if m.declaration_seq is not None:
                lines[0] += "  (declaration_seq=%d)" % m.declaration_seq
            if m.run_id is not None:
                lines[0] += "  run_id=%d" % m.run_id
            lines += [
                "vehicles   " + seen,
                "forwarded  %d" % c.forwarded,
                "dropped    %d rate, %d invalid" % (c.dropped_rate, c.dropped_invalid),
                "stale      %d" % c.stale,
                "seq        %s" % (self.seqs.snapshot(),),
                "wire log   %s" % self.wire.path,
            ]
            return "\n".join(lines)


def _vehicle_from_topic(topic: str) -> str:
    """team/robotx/<team>/<vehicle>/report -> <vehicle>."""
    parts = topic.split("/")
    return parts[-2] if len(parts) >= 2 and parts[-1] == "report" else ""
