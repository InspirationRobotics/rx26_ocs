"""bridge — the OCS's one connection to RoboCommand, and the rules around it.

The handbook permits the team exactly one OCS connection to RoboCommand and
forbids vehicles from reaching it at all. This process is that connection. It
holds a second, entirely separate connection to our own broker, where the
vehicles publish -- and it never routes between them. Those are two MQTT
clients on two networks, joined only by the code below.

THE HOT PATH, in the order it must happen:

    vehicle publishes RxReport (its own clock in sent_at)
      -> [TCP only] link preference, per report kind: a copy that another link
         has just delivered and that outranks it (real over synthesized, then
         the vehicle's `prefer`) is dropped here
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
vehicle's own timestamp survives only as the staleness measurement. The link
preference gate sits ahead of both for the same reason as the first: a duplicate
must not reach the governor (it would see a doubled rate) or take a seq.
"""
from __future__ import annotations

import json
import threading
import time
from datetime import datetime, timezone
from collections import OrderedDict, deque
from dataclasses import dataclass, field

from google.protobuf import json_format

import paho.mqtt.client as mqtt

from . import benchauto, geofence, linkpolicy, validate
from .active import ACTIVE_WINDOW_S, choose_active
from .beacon import BeaconSender, off_snapshot
from .config import LINKS, Config
from .governor import Governor
from .proto import common_pb2, rx_commands_pb2, rx_common_pb2, rx_course_pb2
from .proto import rx_reports_pb2, rx_requests_pb2
from .runstate import RunMachine
from .seqstore import SeqStore
from .tcpserver import DEFAULT_LINK, Conn, TcpServer
from .wirelog import WireLog


#: Handbook 3.1's floor, for the warning below. The value actually enforced
#: is cfg.min_declared, so a bench can lower it -- see config.py.
_POR_MIN_VEHICLES = 2

#: Window over which the published heartbeat rate is measured. Long enough
#: that one late frame does not read as a rate collapse, short enough that a
#: vehicle going quiet shows up while the operator can still act on it.
_HB_WINDOW_S = 10.0

#: Every RxReport body kind: the oneof's own field names, so a regenerated schema
#: cannot leave this list behind. These are the strings a hello's `synthesized`
#: may hold ("heartbeat" is the one that exists today), the keys the link
#: preference files arrivals under, and the keys of the per-kind drop counters.
REPORT_KINDS = tuple(
    f.name for f in rx_reports_pb2.RxReport.DESCRIPTOR.oneofs_by_name["body"].fields)

#: How many non-heartbeat report seqs are remembered for ReadinessConfirm. A
#: run produces a handful; the cap is only so a long-lived process cannot grow
#: this without bound.
_SEQ_MAP_MAX = 256
#: How many numbered vehicle reports are remembered for answering a resend.
_RECEIPTED_MAX = 500
#: How many receipts wait for a socket to take them (_flush_receipts).
_RECEIPTS_OWED_MAX = 200


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
    #: Aircraft heartbeats carrying a uav_registration other than bridge.toml's.
    dropped_misregistered: int = 0
    stale: int = 0
    commands_routed: int = 0
    commands_undeliverable: int = 0
    last_command: str = ""
    last_seen: dict[str, float] = field(default_factory=dict)
    #: vehicle_id -> "tcp" | "mqtt". Which link a vehicle last spoke on, so
    #: the page can say so while both transports are wired up.
    last_via: dict[str, str] = field(default_factory=dict)
    #: vehicle_id -> reports dropped because another link had just delivered the
    #: same kind and outranked this one (linkpolicy.duplicate_reason). Not part of
    #: `dropped_rate`: those never reached the governor, which is the point.
    dropped_not_preferred: dict[str, int] = field(default_factory=dict)
    #: The same drops, split by report kind: vehicle_id -> kind -> count.
    dropped_not_preferred_kinds: dict[str, dict[str, int]] = field(default_factory=dict)
    #: vehicle_id -> heartbeats PUBLISHED as STATE_AUTO that the vehicle sent as
    #: STATE_MANUAL (benchauto.py). Counted at publish, so a frame the validator
    #: refused after the rewrite is not in it.
    bench_auto_substituted: dict[str, int] = field(default_factory=dict)
    #: `ocs_fence` frames set aside rather than stored: from a socket that has not
    #: said which vehicle it carries, naming a different vehicle than the socket's,
    #: naming a vehicle that is not an aircraft, or malformed. Each is also logged.
    fence_frames_ignored: int = 0

    def bench_auto_counts(self, vehicles) -> dict[str, int]:
        """`vehicles` -> how many frames each had substituted (0 until the first)."""
        return {v: self.bench_auto_substituted.get(v, 0) for v in vehicles}

    def count_not_preferred(self, vehicle_id: str, kind: str) -> None:
        self.dropped_not_preferred[vehicle_id] = (
            self.dropped_not_preferred.get(vehicle_id, 0) + 1)
        by_kind = self.dropped_not_preferred_kinds.setdefault(vehicle_id, {})
        by_kind[kind] = by_kind.get(kind, 0) + 1


class Bridge:
    def __init__(self, cfg: Config, *, log=print, bench_auto=()) -> None:
        self.cfg = cfg
        self.log = log
        #: Vehicles whose STATE_MANUAL heartbeat is published as STATE_AUTO -- the
        #: bench dry-run switch (benchauto.py). Command line only, never config.
        #: ValueError here for an id not in the fleet.
        self.bench_auto: tuple[str, ...] = (
            benchauto.resolve(bench_auto, cfg.vehicle_ids) if bench_auto else ())
        # First, not beside the MQTT clients: the restore event just below takes
        # it, and a lock created after its first use is no lock.
        self._lock = threading.RLock()
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
        #: vehicle_id -> report kind -> link -> (time.monotonic(), synthesized) of
        #: the last REPORT of that kind to arrive on the link (counted even if
        #: the gate or the hot path then refused it). What the link preference
        #: gate reads. See linkpolicy.duplicate_reason.
        self._link_rx: dict[str, dict[str, dict[str, tuple[float, bool]]]] = {}
        #: (vehicle_id, OUR report seq) -> (the vehicle's own seq, 0 if it sent
        #: none, body name). Bounded. ReadinessConfirm names a report by OUR
        #: seq, which the vehicle never saw -- see _resolve_confirm.
        self._seq_map: OrderedDict[tuple[str, int], tuple[int, str]] = OrderedDict()
        #: The UAV geofence for the course in hand (geofence.CourseFence): the table
        #: entry or the boundary inset, or None-with-a-reason. Recomputed whenever a
        #: RxCourse is accepted and NEVER reset by a disconnect: the course is what
        #: RoboCommand last told us, and a dropped link does not un-tell us.
        self._fence = geofence.NO_COURSE
        #: The same fence in RunDeclaration.uav_geofence's own JSON shape (None when
        #: there is none), built from a real RunDeclaration so what a vehicle is told
        #: cannot drift from what RoboCommand is sent. Set with `_fence`.
        self._fence_json: list | None = None
        #: `[[geofence.course]]` as a dict, built once: the config is frozen.
        self._fence_table = cfg.geofence_table
        #: vehicle_id -> {"body": the latest `ocs_fence` as geofence.parse_confirmation
        #: keeps it, "received_at": time.time(), "link": which link carried it}. Kept
        #: per vehicle, newest wins. The gate reads it (_uav_gate); never forwarded.
        self._fence_confirms: dict[str, dict] = {}
        #: The declaration_seq a course change arrived under, while DECLARED or
        #: RUNNING; None otherwise. Cleared by the next declaration or by ending.
        self._course_changed_since: int | None = None
        #: The vehicles the last declaration named, for "all declared AUTO". Empty
        #: after a restart restores a run: the store keeps no vehicle list.
        self._declared: tuple[str, ...] = ()
        #: When every declared vehicle was last seen to become STATE_AUTO, or None.
        self._all_auto_since: float | None = None
        #: Task reports (every kind but heartbeat) with what became of each, newest
        #: last: held before a declaration, dropped, published, acknowledged by
        #: RoboCommand's broker. The page's answer to "did my report get there".
        self.task_reports: deque[dict] = deque(maxlen=50)
        #: MQTT mid -> its task_reports entry, until the broker's PUBACK; plus the
        #: mids acknowledged before we could file them (publish() returns first).
        self._awaiting_ack: dict[int, dict] = {}
        self._early_acks: deque[int] = deque(maxlen=64)
        #: (vehicle, its own seq, its sent_at in ns) -> the task_reports entry, for a
        #: report that came in numbered on the vehicle link. ocs_link sends such a
        #: report again after a reconnect until it has a final `ocs_receipt`; the
        #: resend is answered from here and never published twice. In memory only:
        #: a resend that crosses an OCS restart is published again.
        self._receipted: OrderedDict[tuple[str, int, int], dict] = OrderedDict()
        #: (entry, link) receipts owed, sent by _flush_receipts outside the lock.
        self._receipts_owed: list[tuple[dict, str | None]] = []

        # What the web page reads. Decoded with protobuf's own MessageToDict
        # rather than a hand-written field list, so every field the handbook
        # defines shows up and a new one from RoboNation appears on its own.
        self.latest: dict[str, dict] = {}       # vehicle_id -> last report
        #: vehicle_id -> the RobotState its last HEARTBEAT carried. latest[] holds
        #: task reports too, and a vehicle must not read as "not AUTO" just because
        #: its last frame was a DockingReport.
        self.last_state: dict[str, str] = {}
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
            self._run_event("restore", v.reason)

        # Per vehicle, because the 5/s cap is metered per vehicle. See
        # governor.py for the ambiguity in the handbook, and what to change if
        # RoboNation answers that it is per-team.
        self._gov = {
            v: Governor(rate=cfg.rate, reserve=cfg.heartbeat_reserve)
            for v in cfg.vehicle_ids
        }

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
        self.rc.on_publish = self._on_rc_publish

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
                      on_frame=self._on_tcp_frame,
                      on_link=self._on_link_change, log=self.log)
            if cfg.tcp_enabled else None
        )

        # The beacon that tells vehicles where that server is. It lives and dies
        # with the TCP server -- an address advertised for a socket that is not
        # listening is worse than none -- so it exists only when both are on.
        d = cfg.discovery
        self.beacon = (
            BeaconSender(cfg.team_id, port=d.port, interval_s=d.interval_s,
                         never_on=d.never_on, broker_host=cfg.robocommand.host,
                         log=self.log)
            if d.enabled and self.tcp is not None else None
        )

    # ---- lifecycle --------------------------------------------------------

    def start_vehicle_link(self) -> None:
        """The TCP server and, with it, the beacon that advertises it."""
        if self.tcp is None:
            return
        self.tcp.start()
        if self.beacon is not None:
            # bound_port, not the configured one: a server on port 0 (tests)
            # must advertise where it really is.
            self.beacon.start(self.tcp.bound_port)

    def stop_vehicle_link(self) -> None:
        # The beacon first: stop advertising the door before closing it.
        if self.beacon is not None:
            self.beacon.stop()
        if self.tcp is not None:
            self.tcp.stop()

    def start(self) -> None:
        self.start_vehicle_link()
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
        self._announce_bench_auto()

    def _announce_bench_auto(self) -> None:
        """Say, in the console and at the head of the evidence log, that the OCS is
        publishing a state the vehicles are not in. Every start, like the
        min_declared warning above."""
        if not self.bench_auto:
            return
        ids = ", ".join(self.bench_auto)
        self.log("WARNING: BENCH AUTO: publishing STATE_AUTO for %s while they "
                 "report MANUAL -- never at competition" % ids)
        self.wire.event("bench_auto", "ENABLED for %s: STATE_MANUAL heartbeats "
                        "are published as STATE_AUTO; this log's AUTO states "
                        "are not the vehicles'" % ids)

    def stop(self) -> None:
        self.stop_vehicle_link()
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
        if msg.topic == self.cfg.rc_course_sub():
            self._handle_course(msg.payload)
        elif msg.topic == self.cfg.rc_command_sub():
            self._handle_command(msg.payload)
        else:
            self.wire.frame("rx", msg.topic, msg.payload,
                            note="unexpected topic -- ignored")

    def _on_rc_publish(self, _client, _ud, mid, _reason=None, _props=None) -> None:
        """RoboCommand's broker acknowledged a QoS 1 publish (paho also calls this
        when a QoS 0 heartbeat leaves, which nothing is waiting for)."""
        with self._lock:
            entry = self._awaiting_ack.pop(mid, None)
            if entry is None:
                self._early_acks.append(mid)
            else:
                _acknowledged(entry)
                self._owe_receipt(entry)
        self._flush_receipts()

    def _handle_course(self, payload: bytes) -> None:
        topic = self.cfg.rc_course_sub()
        course = rx_course_pb2.RxCourse()
        try:
            course.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001
            self.log("undecodable RxCourse: %s" % exc)
            self.wire.frame("rx", topic, payload,
                            note="undecodable RxCourse: %s" % exc)
            return
        self.wire.frame("rx", topic, payload, note="RxCourse",
                        decoded=lambda: _wire_dict(course))
        # Validate before advancing. Advisory only: a bridge that refuses to
        # leave CONNECTED because it dislikes their course cannot declare at
        # all, and their frame is authoritative even when it is odd. Say so
        # loudly instead, in the window where a person can still act on it.
        findings = [str(f) for f in validate.check_course(course)]
        # The UAV geofence for THIS course, from the course alone (and the static
        # table and margin in bridge.toml), so a redelivered course recomputes the
        # same answer. A fence we had to refuse is one more finding: the page shows
        # it beside the others, where the operator is looking before declaring.
        fence = geofence.for_course(
            course.course_id, [(c.latitude, c.longitude) for c in course.corners],
            self._fence_table, self.cfg.geofence_margin_m,
            [(p.latitude, p.longitude) for p in course.uav_transit_area])
        if fence.polygon is None and self.cfg.has_uav():
            findings.append("UAV geofence: " + (fence.reason or "none"))
        decoded = _decode(course)
        with self._lock:
            # RoboCommand re-delivers the retained course on every reconnect of OUR
            # link to it. Only a course that actually differs is news to a vehicle;
            # sent_at is ignored because a re-stamp is not a different course. The
            # fence is a function of the course, so it changed exactly when the
            # course did.
            changed = _without_sent_at(decoded) != _without_sent_at(self.course)
            # RxCourse is republished whenever it changes (8fc09f2), including after
            # we declared: the declaration then names a fence for the old course. Only
            # a course that REPLACES one we held counts -- the first after a restart
            # is not a change, we just never saw the one before.
            if (changed and self.course and self.machine.state.name in ("DECLARED", "RUNNING")
                    and self._course_changed_since is None):
                self._course_changed_since = self.machine.declaration_seq
            self.course_findings = findings
            # The course and its fence move together, so a vehicle that connects
            # in this instant is never told one course with the other's fence.
            self.course = decoded
            self._set_fence(fence)
            v = self.machine.on_course(course)
        self.log(
            "course %s: pinger %d Hz, %d corners -- %s"
            % (course.course_id, course.pinger_freq_hz, len(course.corners), v.reason)
        )
        if fence.polygon is not None:
            self.log("UAV geofence for %s: %s, %d vertices"
                     % (course.course_id, fence.source, fence.vertices))
            self.wire.event("geofence", "%s: %s, %d vertices"
                            % (course.course_id, fence.source, fence.vertices))
        else:
            self.log("UAV GEOFENCE NONE for %s: %s" % (course.course_id, fence.reason))
            self.wire.event("geofence_none", "%s: %s" % (course.course_id, fence.reason))
        if findings:
            detail = "; ".join(findings)
            self.log("COURSE SUSPECT: %s -- accepted anyway; check before "
                     "declaring" % detail)
            self.wire.event("course_suspect", detail)
        if changed and self._course_changed_since is not None:
            self.log("COURSE CHANGED since declaration seq=%s -- end and re-declare: the "
                     "declared UAV geofence is the old course's" % self._course_changed_since)
            self._run_event("course_changed", "since declaration seq=%s"
                            % self._course_changed_since)
        if changed:
            # Information only -- the vehicle decides what, if anything, to do.
            # The new fence is already in place (above), so every aircraft is told
            # THIS course's, or an explicit null when there is none.
            self._send_info(self.cfg.vehicle_ids, "course")

    def _set_fence(self, fence: geofence.CourseFence) -> None:
        """Make `fence` the current one. Caller holds the lock."""
        self._fence = fence
        self._fence_json = _fence_json(fence.polygon)

    def _handle_command(self, payload: bytes) -> None:
        topic = self.cfg.rc_command_sub()
        cmd = rx_commands_pb2.RxCommand()
        try:
            cmd.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001
            self.log("undecodable RxCommand: %s" % exc)
            self.wire.frame("rx", topic, payload,
                            note="undecodable RxCommand: %s" % exc)
            return

        which = cmd.WhichOneof("body")
        self.wire.frame("rx", topic, payload, note="%s seq=%d" % (which, cmd.seq),
                        decoded=lambda: _wire_dict(cmd))
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
            self._run_event("run_start" if v.ok else "run_start_refused", v.reason)

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

        Each target gets the frame on ONE link -- see _send_to_vehicle. The
        command's own `seq` is passed through untouched: IncidentAck.command_seq
        from the vehicle refers to it, and nothing here may translate it.
        ReadinessConfirm is the one exception, and for the opposite reason: it
        names a REPORT, by a seq only we assigned -- see _resolve_confirm.
        """
        fwd, vehicle_id, rewrite = cmd, cmd.vehicle_id, None
        if which == "readiness_confirm":
            fwd, vehicle_id, rewrite = self._resolve_confirm(cmd)

        targets: tuple[str, ...]
        if vehicle_id:
            if self.cfg.vehicle(vehicle_id) is None:
                # Do not invent a topic for a vehicle we do not fly. A typo on
                # their side becoming a subscription nobody reads is exactly
                # the kind of quiet that costs a task.
                self.counters.commands_undeliverable += 1
                self.log(
                    "COMMAND UNDELIVERABLE: %s addressed to %s, which is not "
                    "in the fleet" % (which, vehicle_id)
                )
                self.wire.event(
                    "command_undeliverable", "%s -> %s" % (which, vehicle_id)
                )
                self.recent.appendleft({"at": time.time(), "which": which,
                                        "seq": cmd.seq, "targets": [],
                                        "routed": [], "delivered": False,
                                        "detail": "%s is not in the fleet"
                                                  % vehicle_id,
                                        "body": _decode(cmd)})
                return
            targets = (vehicle_id,)
        else:
            targets = self.cfg.vehicle_ids

        payload = fwd.SerializeToString()
        as_json = _to_json(fwd)
        decoded = lambda: _wire_dict(fwd)  # noqa: E731 - decoded lazily, inside the log's guard
        note = "%s seq=%d" % (which, cmd.seq)
        if rewrite:
            note += " report_seq ours=%d vehicle's=%d" % (
                rewrite["ocs_seq"], rewrite["vehicle_seq"])
        routed: list[dict] = []
        for vid in targets:
            # The vehicle link first: a vehicle holding a TCP connection is
            # listening on that socket, and it is the one that will still
            # exist once the broker is gone. MQTT below is the fallback while
            # both transports are wired up.
            link = self._send_to_vehicle(vid, as_json)
            if link is not None:
                routed.append({"vehicle": vid, "link": link})
                self._wire_to_vehicle(vid, link, as_json, decoded, note)
                continue
            topic = self.cfg.team_command_topic(vid)
            # QoS 1 without exception. Unlike a heartbeat, a command does not
            # come round again in 500 ms -- if it is dropped it is simply gone,
            # and the operator is left waiting on a response to something the
            # vehicle never heard.
            self.team.publish(topic, payload, qos=1)
            routed.append({"vehicle": vid, "link": "mqtt"})
            self._wire_to_vehicle(vid, "mqtt", payload, decoded, note)

        routes = ["%s/%s" % (r["vehicle"], r["link"]) for r in routed]
        with self._lock:
            self.counters.commands_routed += 1
            self.counters.last_command = "%s -> %s" % (which, ", ".join(routes))
            self.recent.appendleft({"at": time.time(), "which": which,
                                    "seq": cmd.seq, "targets": list(targets),
                                    "routes": routes, "routed": routed,
                                    "rewrite": rewrite,
                                    "delivered": True, "body": _decode(cmd)})
        self.log("COMMAND %s (seq=%d) -> %s" % (which, cmd.seq, ", ".join(routes)))

    def _resolve_confirm(self, cmd):
        """(frame to send, vehicle to send it to, rewrite or None) for a ReadinessConfirm.

        The handbook: "the OCS must preserve the command and report sequence
        references", and "for ReadinessConfirm, use both the report sequence and
        vehicle ID to identify the readiness report". We re-assign every report's
        seq (the hot path), so the report_seq RoboCommand echoes back is OUR
        number -- one the vehicle never saw. Left alone, the vehicle is told that
        report 41 was confirmed when it only ever sent report 7.

        So: look up (vehicle, our seq) in the bounded map the hot path fills, send
        the confirm to that vehicle only, and -- if the vehicle had put its own
        nonzero seq on the report -- put that seq back in report_seq. A vehicle
        that sent no seq has nothing to be told in its own terms, so the frame is
        relayed unchanged (still to the right vehicle). Never invent a value: an
        unknown seq is relayed as received, and said so in the log.
        """
        rc = cmd.readiness_confirm
        ocs_seq = rc.report_seq
        vid = rc.vehicle_id or cmd.vehicle_id
        if rc.vehicle_id and cmd.vehicle_id and rc.vehicle_id != cmd.vehicle_id:
            self.log("READINESS CONFIRM: command addressed to %s but confirm "
                     "names %s -- trusting the confirm"
                     % (cmd.vehicle_id, rc.vehicle_id))
            self.wire.event("readiness_mismatch", "%s vs %s"
                            % (cmd.vehicle_id, rc.vehicle_id))
        with self._lock:
            if vid:
                hit = self._seq_map.get((vid, ocs_seq))
            else:
                # No vehicle named anywhere: seqs are per vehicle, so only a seq
                # held by exactly one vehicle is unambiguous.
                found = [(k, v) for k, v in self._seq_map.items() if k[1] == ocs_seq]
                hit = found[0][1] if len(found) == 1 else None
                if hit is not None:
                    vid = found[0][0][0]
        if hit is None:
            self.log("READINESS CONFIRM: no report seq=%d%s in our record -- "
                     "relayed unchanged" % (ocs_seq, " from " + vid if vid else ""))
            self.wire.event("readiness_seq", "no report seq=%d%s on record; "
                            "relayed unchanged"
                            % (ocs_seq, " for " + vid if vid else ""))
            return cmd, vid, None
        own_seq, body = hit
        if not own_seq:
            self.wire.event("readiness_seq",
                            "%s: our seq %d, the vehicle sent no seq of its own; "
                            "relayed unchanged to %s" % (body, ocs_seq, vid))
            return cmd, vid, None
        fwd = rx_commands_pb2.RxCommand()
        fwd.CopyFrom(cmd)
        fwd.readiness_confirm.report_seq = own_seq
        rewrite = {"vehicle": vid, "ocs_seq": ocs_seq, "vehicle_seq": own_seq}
        self.log("READINESS CONFIRM %s: report_seq %d (ours) -> %d (the vehicle's own)"
                 % (vid, ocs_seq, own_seq))
        self.wire.event("readiness_seq", "%s %s: report_seq %d (ours) -> %d "
                        "(the vehicle's own)" % (vid, body, ocs_seq, own_seq))
        return fwd, vid, rewrite

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
            self.wire.frame("veh", msg.topic, msg.payload,
                            note="team mqtt; undecodable: %s" % exc)
            return

        topic_vehicle = _vehicle_from_topic(msg.topic)
        if topic_vehicle and topic_vehicle != report.vehicle_id:
            # The handbook requires the topic's vehicle_id to match the message's.
            self.log("DROP: topic says %s, message says %s"
                     % (topic_vehicle, report.vehicle_id))
            self.wire.event("mismatch",
                            "%s vs %s" % (msg.topic, report.vehicle_id))
            self.wire.frame("veh", msg.topic, msg.payload,
                            note="team mqtt; DROPPED: topic says %s"
                                 % topic_vehicle,
                            decoded=lambda: _wire_dict(report))
            return
        self.wire.frame("veh", msg.topic, msg.payload, note="team mqtt",
                        decoded=lambda: _wire_dict(report))
        self._ingest_report(report, via="mqtt")

    def _on_tcp_frame(self, payload: bytes, conn: Conn) -> str | None:
        """One framed JSON frame off the vehicle link: a hello, or an RxReport.

        Returns the vehicle_id, which is what tcpserver files the connection
        under (on conn.link). Returning None for a frame we could not read
        leaves that socket anonymous -- correct, because we have not learned who
        is on it, and guessing would route somebody else's commands down it.

        JSON in, protobuf out. The bridge re-stamps seq and sent_at and
        re-serializes every report regardless, so the format arriving from a
        vehicle was never coupled to the one RoboCommand receives. That is what
        lets the boat carry no protobuf at all.

        Every frame is wire-logged exactly once, with the link it arrived on --
        a duplicate dropped by the link preference is logged too, with why.
        """
        try:
            obj = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return self._bad_vehicle_frame(payload, conn, exc)
        if isinstance(obj, dict) and "ocs_hello" in obj:
            return self._on_hello(payload, obj, conn)
        if isinstance(obj, dict) and "ocs_fence" in obj:
            # Not a report and never one: it is read here and goes nowhere else --
            # not the governor, not a seq, not RoboCommand.
            return self._on_fence(payload, obj, conn)

        report = rx_reports_pb2.RxReport()
        try:
            if not isinstance(obj, dict):
                raise ValueError("a frame must be a JSON object, got %s"
                                 % type(obj).__name__)
            json_format.ParseDict(obj, report)
        except Exception as exc:  # noqa: BLE001
            # ParseError names the offending field, which is most of what goes
            # wrong while a reporter is being written: a misspelled key, or an
            # enum spelled as something that is not one of its values.
            #
            # NOTE a bare NaN lands here too -- protobuf's JSON mapping requires
            # non-finite floats as the quoted string "NaN". The vehicle sends
            # them that way; see ocs_link.py in rx26_asv.
            return self._bad_vehicle_frame(payload, conn, exc)

        vehicle = report.vehicle_id
        # THE LINK PREFERENCE GATE: first thing after the frame is understood,
        # before staleness and the governor. See linkpolicy.py for why.
        why = self._prefer_gate(report, conn)
        self._wire_from_vehicle(conn.link, payload,
                                "vehicle=%s link=%s%s" % (
                                    vehicle, conn.link,
                                    " DROPPED %s" % why if why else ""), obj)
        if why is None:
            self._ingest_report(report, via="tcp", link=conn.link)
            self._flush_receipts()
        return vehicle or None

    def _bad_vehicle_frame(self, payload: bytes, conn: Conn, exc) -> None:
        self.log("undecodable RxReport on the vehicle link: %s" % exc)
        self._wire_from_vehicle(conn.link, payload,
                                "link=%s undecodable: %s" % (conn.link, exc))
        self.wire.event("undecodable", str(exc))
        return None

    def _on_hello(self, payload: bytes, obj: dict, conn: Conn) -> str | None:
        """`{"ocs_hello": {"vehicle_id", "link", "agent", "synthesized"}}` -- a client
        saying which vehicle it carries, over which transport, and (optionally)
        which report kinds it builds itself rather than relays: a list of
        REPORT_KINDS names, e.g. ["heartbeat"]. Absent means none.

        Optional, and only meaningful as the FIRST frame: a socket that never
        sends one is link "wifi", which is every vehicle client written so far.
        A bad hello drops the connection rather than guessing -- filing a radio
        adapter as wifi would make the OCS drop its telemetry as the
        non-preferred copy, silently, which is the failure this exists to avoid.
        """
        h = obj.get("ocs_hello")

        def refuse(why: str) -> None:
            conn.close_reason = "bad ocs_hello: " + why
            self.log("vehicle link %s: bad ocs_hello (%s) -- dropping" % (conn.who, why))
            self._wire_from_vehicle(conn.link, payload,
                                    "ocs_hello REFUSED: %s" % why, obj)
            self.wire.event("hello_refused", "%s: %s" % (conn.who, why))
            return None

        if conn.vids:
            # Changing a socket's link after it has been filed would move it
            # under a live registration. Say so and carry on.
            self.log("vehicle link %s: ocs_hello after identification as %s -- ignored"
                     % (conn.who, ",".join(conn.vids)))
            self._wire_from_vehicle(conn.link, payload,
                                    "ocs_hello IGNORED: already identified as %s"
                                    % ",".join(conn.vids), obj)
            return None
        if not isinstance(h, dict):
            return refuse("ocs_hello is not an object")
        vid = h.get("vehicle_id")
        if not isinstance(vid, str) or self.cfg.vehicle(vid) is None:
            return refuse("vehicle_id %r is not in the fleet" % (vid,))
        link = h.get("link", DEFAULT_LINK)
        if link not in LINKS:
            return refuse("link %r is not one of %s" % (link, ", ".join(LINKS)))
        agent = str(h.get("agent", ""))[:64]
        synthesized, why = _parse_synthesized(h.get("synthesized"))
        if why:
            # Ignoring it would file a rebuilt heartbeat as the vehicle's own and
            # let it outrank the real one -- the silent failure this field is for.
            return refuse(why)

        conn.link, conn.agent, conn.hello = link, agent, True
        conn.synthesized = synthesized
        synth = ",".join(sorted(synthesized))
        self.log("vehicle link: %s hello: %s on %s%s%s"
                 % (conn.who, vid, link, " (%s)" % agent if agent else "",
                    " synthesizing %s" % synth if synth else ""))
        self._wire_from_vehicle(link, payload,
                                "ocs_hello vehicle=%s link=%s agent=%s synthesized=%s"
                                % (vid, link, agent, synth or "-"), obj)
        return vid

    def _on_fence(self, payload: bytes, obj: dict, conn: Conn) -> None:
        """`{"ocs_fence": {...}}` -- the radio adapter saying what Ekko's autopilot
        holds, read back. docs/VEHICLE_LINK.md has the frame; geofence.py has the rule
        that decides whether the OCS may rely on it (the pre-run gate, _uav_gate).

        The latest one per vehicle is kept, newest wins; freshness is judged by the
        gate from the frame's own `verified_at`, so a link that closes needs no
        clean-up here: its confirmation simply ages out.

        WHICH VEHICLE IS THE SOCKET'S, NOT THE BODY'S. A body is believed only about
        a vehicle this connection has identified itself as (its hello, or a report):
        otherwise any socket could vouch for an aircraft it has nothing to do with.
        A frame from an unidentified socket, one naming another vehicle, one naming
        something that is not an aircraft, or a malformed one is set aside, counted
        and logged, and the socket stays up. Returns None either way -- identifying
        a socket is a hello's and a report's job, not this frame's.
        """
        body, why = geofence.parse_confirmation(obj.get("ocs_fence"))
        vid = body["vehicle_id"] if body else None
        spec = self.cfg.vehicle(vid) if vid else None
        if body is not None:
            if not conn.vids:
                body, why = None, ("this socket has not said which vehicle it carries "
                                   "(no hello, no report yet)")
            elif vid not in conn.vids:
                body, why = None, ("it names %s but this connection carries %s"
                                   % (vid, ",".join(conn.vids)))
            elif spec is None or not spec.is_uav:
                body, why = None, "%s is not an aircraft; only a UAV has a fence" % vid
        if body is None:
            with self._lock:
                self.counters.fence_frames_ignored += 1
            self.log("vehicle link %s: ocs_fence IGNORED (%s)" % (conn.who, why))
            self._wire_from_vehicle(conn.link, payload,
                                    "ocs_fence IGNORED: %s" % why, obj)
            self.wire.event("fence_ignored", "%s: %s" % (conn.who, why))
            return None

        with self._lock:
            before = (self._fence_confirms.get(vid) or {}).get("body") or {}
            self._fence_confirms[vid] = {"body": body, "received_at": time.time(),
                                         "link": conn.link}
        poly = body["polygon"]
        self._wire_from_vehicle(
            conn.link, payload,
            "ocs_fence vehicle=%s state=%s verified=%s vertices=%s"
            % (vid, body["state"] or "-", body["verified"],
               "-" if poly is None else len(poly)), obj)
        # The console only on a CHANGE: the adapter repeats the frame every minute
        # for as long as it is verified, and the wire log already has each one.
        keys = ("state", "polygon", "verified", "reason", "lockout")
        if any(before.get(k) != body[k] for k in keys):
            self.log("fence confirmation %s: %s, %s%s%s"
                     % (vid, body["state"] or "no state",
                        "verified" if body["verified"] else "NOT verified",
                        "" if poly is None else ", %d vertices" % len(poly),
                        " -- LOCKOUT: %s" % body["lockout"] if body["lockout"] else ""))
        return None

    def _wire_from_vehicle(self, link: str, payload: bytes, note: str,
                           decoded=None) -> None:
        """Wire-log one frame that arrived on the vehicle link, with its link."""
        self.wire.frame("veh", "tcp://in/%s" % link, payload, note=note,
                        decoded=decoded)

    def _prefer_gate(self, report, conn: Conn) -> str | None:
        """Note that `report` arrived on conn.link; return why to drop it, or None.

        Per vehicle and per report KIND: what it is weighed against is the same
        kind's last arrival on the vehicle's other link, never the other link's
        traffic in general (see linkpolicy.py). Only for vehicles we know -- an
        unknown id must not be able to grow this table, and _ingest_report logs
        and drops it anyway.
        """
        vehicle = report.vehicle_id
        spec = self.cfg.vehicle(vehicle)
        if spec is None:
            return None
        kind = report.WhichOneof("body") or "none"
        synthesized = kind in conn.synthesized
        now = time.monotonic()
        with self._lock:
            arrivals = self._link_rx.setdefault(vehicle, {}).setdefault(kind, {})
            why = linkpolicy.duplicate_reason(
                spec.prefer, conn.link, kind, synthesized, arrivals, now,
                self.cfg.link_prefer_s)
            arrivals[conn.link] = (now, synthesized)
            if why:
                self.counters.count_not_preferred(vehicle, kind)
            return why

    def _last_report_rx(self, vehicle: str) -> dict[str, float]:
        """link -> time.monotonic() of the newest report of ANY kind to arrive on
        it (dropped or not). Absent: nothing but hellos and housekeeping so far.
        Caller holds the lock."""
        out: dict[str, float] = {}
        for by_link in self._link_rx.get(vehicle, {}).values():
            for link, (t, _synth) in by_link.items():
                out[link] = max(t, out.get(link, t))
        return out

    def _on_link_change(self, vid: str, link: str, up: bool, conn: Conn) -> None:
        """A (vehicle, link) socket was filed or went away -- tcpserver's callback."""
        self.wire.event("link_up" if up else "link_down",
                        "%s %s %s%s" % (vid, link, conn.who,
                                        " (%s)" % conn.agent if conn.agent else ""))
        if up:
            # A client that has just come up knows nothing the OCS knows. Told on
            # the socket that came up, not on the vehicle's best link: a fresh
            # WiFi client is not served by a snapshot that went down the radio.
            self._send_info([vid], "%s link up" % link, link=link)
            # A report that was this socket's first frame was receipted before the
            # socket was filed under its vehicle, so that receipt is still owed.
            self._flush_receipts()

    def _ingest_report(self, report, *, via: str, link: str | None = None) -> None:
        """The hot path, from either transport.

        Everything from here down is deliberately transport-agnostic: staleness,
        the governor, validation, sequencing and the re-stamp are rules about the
        message, not about the socket it arrived on. That separation is the
        reason adding TCP did not require touching governor.py, seqstore.py or
        validate.py at all. (The one transport-aware rule, the link preference,
        runs in _on_tcp_frame before a duplicate gets this far.)
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
        # A task report the vehicle numbered itself. ocs_link sends one again after a
        # reconnect until this end has receipted it: the same report twice. The
        # resend is answered with what became of the first and never published
        # again -- before the governor, so it spends no rate token and no seq.
        origin = (None, link)
        if body != "heartbeat" and report.seq:
            origin = ((vehicle, report.seq, report.sent_at.ToNanoseconds()), link)
            with self._lock:
                first = self._receipted.get(origin[0])
            if first is not None:
                self.log("%s sent its report seq=%d again: answered %s, not republished"
                         % (vehicle, report.seq, first["verdict"]))
                self.wire.event("resend", "%s seq=%d %s" % (vehicle, report.seq,
                                                             first["verdict"]))
                with self._lock:
                    self._receipts_owed.append((first, link))
                self._flush_receipts()
                return
        # Before the gate below: the bridge is correctly silent toward
        # RoboCommand until we declare, but the operator still needs to see the
        # fleet then -- that is the window in which a sick vehicle can still be
        # fixed.
        #
        # Inside the lock, because with bench auto on this draft says MANUAL and
        # the published frame below says AUTO: a reader taking the lock (the
        # snapshot, _note_auto_edge) must see one or the other, never the draft
        # in the middle of this vehicle's own publish.
        with self._lock:
            self._remember(vehicle, report)
            is_hb = body == "heartbeat"
            if not self.machine.may_report():
                if not is_hb:
                    self._track(vehicle, body, report.seq, "held",
                                "not declared: RoboCommand gets nothing before a declaration",
                                origin)
                return  # pre-declaration silence is correct, not an error

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
                if not is_hb:
                    self._track(vehicle, body, report.seq, "dropped",
                                "over the 5/s rate cap (governor)", origin)
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
                # uav_registration (8fc09f2) is stamped here, once, so every source
                # of an aircraft's heartbeats -- its own client, the radio, the
                # adapter's stand-in -- carries the one in bridge.toml. A reporter
                # sending a DIFFERENT one is misconfigured, and Garuda relays these
                # for Network Remote ID: dropped like a mistyped one, not published.
                if spec.is_uav:
                    sent = report.heartbeat.uav_registration
                    if sent and sent != spec.registration:
                        self.counters.dropped_misregistered += 1
                        detail = "%s sent uav_registration %r but bridge.toml has %s" % (
                            vehicle, sent, repr(spec.registration) if spec.registration
                            else "none")
                        self.log("DROP: " + detail)
                        self.wire.event("misregistered", detail)
                        return
                    if spec.registration:
                        report.heartbeat.uav_registration = spec.registration

            # BENCH AUTO (benchauto.py): the one rewrite of a vehicle's own value,
            # chosen by the operator on the command line. After the gates above
            # and before validate, so what is checked is what is published.
            bench_from = (benchauto.promote(report, self.bench_auto)
                          if self.bench_auto else None)

            findings = validate.check(report, allow_unknown=spec.allow_unknown)
            if findings:
                self.counters.dropped_invalid += 1
                detail = "; ".join(str(f) for f in findings)
                self.log("DROP %s: %s" % (vehicle, detail))
                self.wire.event("invalid", detail)
                if not is_hb:
                    self._track(vehicle, body, report.seq, "dropped", "invalid: " + detail,
                                origin)
                return

            # The vehicle's own seq, if it put one on the report (0 if not).
            # Read BEFORE the line below overwrites it: ReadinessConfirm will
            # name this report by OUR seq, and the vehicle only knows its own.
            own_seq = report.seq
            report.seq = self.seqs.next_report(vehicle)
            report.team_id = self.cfg.team_id
            report.sent_at.FromNanoseconds(time.time_ns())

            payload = report.SerializeToString()
            seq = report.seq
            self.counters.forwarded += 1
            if bench_from:
                self._count_bench_auto(vehicle, bench_from)
            if is_hb:
                self.hb_rate.record(vehicle, now)
                entry = None
            else:
                self._remember_seq(vehicle, seq, own_seq, body)
                entry = self._track(vehicle, body, own_seq, "published",
                                    "seq=%d, awaiting the broker's PUBACK" % seq, origin)
                entry["seq"] = seq
            # Record it again now that seq and sent_at are ours, so the page
            # shows the frame RoboCommand actually received rather than the
            # vehicle's draft of it. The earlier write above is what keeps a
            # dropped or pre-declaration frame visible at all.
            self._remember(vehicle, report)

        topic = self.cfg.rc_report_topic(vehicle)
        info = self.rc.publish(topic, payload, qos=0 if is_hb else 1)
        if entry is not None:
            self._await_ack(entry, info)
        note = "seq=%d %s%s" % (seq, body,
                                " (vehicle seq=%d)" % own_seq if own_seq else "")
        if bench_from:
            note += benchauto.wire_note(bench_from)
        self.wire.frame("tx", topic, payload, note=note,
                        decoded=lambda: _wire_dict(report))
        if is_hb:
            self._note_auto_edge()

    def _track(self, vehicle: str, kind: str, own_seq: int, verdict: str,
               detail: str, origin: tuple = (None, None)) -> dict:
        """File what became of one task report, for the page. Caller holds the lock.

        `origin` is (resend key, vehicle link) from _ingest_report: a report the
        vehicle numbered is remembered for answering its resend, and a verdict that
        is already final (held, dropped) owes the vehicle its receipt now. A
        published one owes it from _await_ack."""
        entry = {"at": time.time(), "vehicle": vehicle, "kind": kind,
                 "vehicle_seq": own_seq or None, "seq": None, "verdict": verdict,
                 "detail": detail}
        self.task_reports.append(entry)
        key, link = origin
        if key is not None:
            entry["link"] = link
            self._receipted[key] = entry
            while len(self._receipted) > _RECEIPTED_MAX:
                self._receipted.popitem(last=False)
            if verdict != "published":
                self._receipts_owed.append((entry, link))
        return entry

    def _await_ack(self, entry: dict, info) -> None:
        """Hold a published task report until RoboCommand's broker acknowledges it.
        paho keeps a QoS 1 message published with the link down and sends it on
        reconnect, so "queued" is not "lost" -- but it is not "there" either."""
        with self._lock:
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                entry["detail"] = "seq=%d, queued: no RoboCommand link (%s)" % (
                    entry["seq"], mqtt.error_string(info.rc))
            if info.mid in self._early_acks:
                self._early_acks.remove(info.mid)
                _acknowledged(entry)
            else:
                self._awaiting_ack[info.mid] = entry
                while len(self._awaiting_ack) > 200:       # a link down for a long time
                    self._awaiting_ack.pop(next(iter(self._awaiting_ack)))
            self._owe_receipt(entry)
        self._flush_receipts()

    def _owe_receipt(self, entry: dict) -> None:
        """Caller holds the lock."""
        if "link" in entry:
            self._receipts_owed.append((entry, entry["link"]))

    def _flush_receipts(self) -> None:
        """Send the receipts owed, outside the lock: a vehicle socket slow to drain
        must not stall the hot path for every vehicle. One that no socket took stays
        owed -- a socket's first frame is handled before tcpserver files the socket
        under its vehicle -- and goes when that link comes up (_on_link_change)."""
        with self._lock:
            owed, self._receipts_owed = self._receipts_owed, []
        kept = [(entry, link) for entry, link in owed if not self._send_receipt(entry, link)]
        if kept:
            with self._lock:
                self._receipts_owed[:0] = kept
                del self._receipts_owed[:-_RECEIPTS_OWED_MAX]

    def _send_receipt(self, entry: dict, link: str | None) -> bool:
        """`ocs_receipt`: what became of one numbered task report, as it stands now, on
        `link`, the one it (or its resend) came in on. Never over the radio -- RXL has
        no message for it. False if no socket took it. A receipt that never arrives
        costs nothing: the vehicle sends the report again after a reconnect and is
        answered from _receipted. Final verdicts are acknowledged, held and dropped."""
        if link is None or link == "radio":
            return True
        obj = {"ocs_receipt": {"vehicle_seq": entry["vehicle_seq"], "kind": entry["kind"],
                               "verdict": entry["verdict"], "seq": entry["seq"],
                               "detail": entry["detail"]}}
        body = json.dumps(obj).encode("utf-8")
        if self._send_to_vehicle(entry["vehicle"], body, link) is None:
            return False
        self._wire_to_vehicle(entry["vehicle"], link, body, obj, "ocs_receipt %s seq=%d"
                              % (entry["verdict"], entry["vehicle_seq"]))
        return True

    def _count_bench_auto(self, vehicle: str, original: str) -> None:
        """One frame of `vehicle` was published as AUTO. Caller holds the lock.

        The first one per vehicle also gets a console line and a wire-log event,
        so the moment the dry run began to differ from the vehicle is findable."""
        n = self.counters.bench_auto_substituted.get(vehicle, 0) + 1
        self.counters.bench_auto_substituted[vehicle] = n
        if n == 1:
            self.log("BENCH AUTO: %s now published as STATE_AUTO (reports %s)"
                     % (vehicle, original))
            self.wire.event("bench_auto", "%s: first frame published STATE_AUTO "
                            "for a vehicle reporting %s" % (vehicle, original))

    def _remember_seq(self, vehicle: str, ocs_seq: int, own_seq: int,
                      body: str) -> None:
        """Record which of OUR seqs was which vehicle report. Caller holds the lock."""
        self._seq_map[(vehicle, ocs_seq)] = (own_seq, body)
        while len(self._seq_map) > _SEQ_MAP_MAX:
            self._seq_map.popitem(last=False)

    # ---- sending to a vehicle --------------------------------------------

    def _link_order(self, vid: str, now: float | None = None,
                    links: dict | None = None) -> list[str]:
        """The vehicle's connected links, best first. See linkpolicy.send_order."""
        spec = self.cfg.vehicle(vid)
        if links is None:
            links = self.tcp.links_of(vid) if self.tcp is not None else {}
        return linkpolicy.send_order(
            {link: info["last_rx"] for link, info in links.items()},
            spec.prefer if spec else None,
            time.monotonic() if now is None else now,
            self.cfg.link_prefer_s)

    def _send_to_vehicle(self, vid: str, body: bytes, link: str | None = None) -> str | None:
        """Send one frame to a vehicle on ONE link. The link used, or None.

        With `link`, that socket only -- no fallback to the other one. Without,
        the best link first (linkpolicy.send_order), then the next.

        None means no socket accepted it; the caller decides whether that falls
        back to MQTT (a command) or is simply not delivered (a directive, info).
        The loop stops at the first success: a frame never goes down two links,
        because a command delivered twice is a command a vehicle may act on
        twice.
        """
        if self.tcp is None:
            return None
        for candidate in ([link] if link else self._link_order(vid)):
            if self.tcp.send(vid, body, link=candidate):
                return candidate
        return None

    def _wire_to_vehicle(self, vid: str, link: str, payload: bytes, decoded,
                         note: str) -> None:
        """Wire-log one frame sent to a vehicle. link "mqtt" means the team broker."""
        topic = (self.cfg.team_command_topic(vid) if link == "mqtt"
                 else "tcp://%s/%s" % (vid, link))
        self.wire.frame("vtx", topic, payload, note=note, decoded=decoded)

    def _info_obj(self, spec) -> dict:
        """The `ocs_info` frame for one vehicle, as a dict.

        A complete snapshot of what the OCS knows, never a delta: a vehicle that
        reconnects, or missed one, is fully informed by the next. The geofence
        is for aircraft only -- the other vehicles are not bound by it.

        An aircraft ALWAYS gets both keys, and the pair means something:
        `course: null, uav_geofence: null` is "this OCS has received no course", and
        `course: {...}, uav_geofence: null` is "there is a course but no fence could
        be made for it" (a bad table entry, a boundary the inset cannot trust). The
        radio adapter clears the autopilot's polygon in the first case only; in the
        second it leaves the old one standing, which is why that case is also a
        loud course finding. docs/VEHICLE_LINK.md.
        """
        info: dict = {"course": self.course or None}
        if spec.is_uav:
            info["uav_geofence"] = self._fence_json
        return {"ocs_info": info}

    def _send_info(self, vehicles, why: str, link: str | None = None) -> None:
        """Send `ocs_info` to each named vehicle that is connected and wants it.

        INFORMATION ONLY. Nothing in the frame is a command, and the OCS has no
        say over what a vehicle does with it. One link per vehicle, no MQTT
        fallback: a vehicle that is not connected gets the snapshot when it next
        connects (_on_link_change). `link` pins it to the socket that just came
        up; otherwise it goes down the vehicle's best link.
        """
        if self.tcp is None:
            return
        reached, missed = [], []
        for vid in vehicles:
            spec = self.cfg.vehicle(vid)
            if spec is None or not spec.send_info:
                continue
            obj = self._info_obj(spec)
            body = json.dumps(obj).encode("utf-8")
            sent_on = self._send_to_vehicle(vid, body, link)
            if sent_on is None:
                missed.append(vid)
                continue
            reached.append("%s/%s" % (vid, sent_on))
            self._wire_to_vehicle(vid, sent_on, body, obj, "ocs_info (%s)" % why)
        if reached:
            self.log("ocs_info (%s) -> %s" % (why, ", ".join(reached)))
        if reached or missed:
            self.wire.event("ocs_info", "%s: sent=%s not_connected=%s"
                            % (why, ",".join(reached), ",".join(missed)))

    def awaiting_auto(self) -> list[str]:
        """Declared vehicles not yet seen in STATE_AUTO.

        RoboCommand publishes RunStart only once every vehicle in the
        declaration reports STATE_AUTO -- with no timeout and no error, so a
        fleet stuck one vehicle short looks exactly like a broken link to
        their server. This names the vehicle, which turns a silent wait into
        something the operator can act on. Judged on the vehicles the last
        declaration named; every configured one when there is none to go by (before
        declaring, or after a restart restored the run without its vehicle list).
        """
        return [vid for vid in self._declared or self.cfg.vehicle_ids
                if self.last_state.get(vid) != "STATE_AUTO"]

    def _any_uav(self, ids) -> bool:
        return any(self.cfg.vehicle(v).is_uav for v in ids)

    def _remember(self, vehicle: str, report) -> None:
        """latest[] for the page, and for a heartbeat the state awaiting_auto judges by.
        Caller holds the lock."""
        decoded = _decode(report)
        self.latest[vehicle] = decoded
        if "heartbeat" in decoded:
            self.last_state[vehicle] = decoded["heartbeat"].get("state", "")

    def _note_auto_edge(self) -> None:
        """Log the moment the fleet becomes fully AUTO, once.

        That instant is when RunStart becomes their side's responsibility. If
        it does not arrive after this line, the problem is not our fleet --
        though since 8fc09f2 RoboCommand may also hold RunStart for Garuda's
        clearance and the course manager's approval, so a wait is not a fault.
        """
        with self._lock:      # latest[] is rewritten under it -- see _ingest_report
            waiting = self.awaiting_auto()
            if not waiting and not self._all_auto:
                self._all_auto = True
                self._all_auto_since = time.time()
                self.log("all declared vehicles report STATE_AUTO -- RunStart is "
                         "now RoboCommand's move")
                self.wire.event("all_auto", ", ".join(self._declared or self.cfg.vehicle_ids))
            elif waiting and self._all_auto:
                self._all_auto = False
                self._all_auto_since = None
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
        obj = {"ocs_directive": {
            "action": "standby_auto",
            "declaration_seq": seq,
            "vehicles": list(vehicles),
            "sent_at": _rfc3339(),
        }}
        body = json.dumps(obj).encode("utf-8")
        reached, missed = [], []
        for vid in vehicles:
            link = self._send_to_vehicle(vid, body)
            if link is None:
                missed.append(vid)
                continue
            reached.append("%s/%s" % (vid, link))
            self._wire_to_vehicle(vid, link, body, obj, "standby_auto seq=%d" % seq)
        self.log("standby_auto -> %s%s"
                 % (", ".join(reached) or "nobody",
                    "" if not missed
                    else "  (NOT DELIVERED: %s -- no vehicle link)"
                         % ", ".join(missed)))
        self.wire.event("standby_auto", "sent=%s missed=%s"
                        % (",".join(reached), ",".join(missed)))

    # ---- operator actions -------------------------------------------------

    def declare_blocker(self, now: float | None = None) -> str | None:
        """Why a declaration cannot be made right now, in words, or None if it can.

        THE ONE PLACE the preconditions live, in the order the operator can act on
        them: the run state, the handbook's two-vehicle floor, and then the UAV
        fence gate. declare() asks it before it builds anything and the page's
        button asks it every poll (snapshot `run.may_declare` and
        `run.declare_blocker`), so the button and the endpoint cannot disagree --
        the endpoint refuses on its own, whatever the page showed. The one thing
        the page holds is the pilot's call sign, judged only when given -- so the
        button's verdict is everything but that, and the page checks its box.
        """
        return self._check_declare(now)[0]

    def _check_declare(self, now: float | None = None, callsign: str | None = None
                       ) -> tuple[str | None, tuple[str, ...]]:
        """(declare_blocker, the active set it was judged on). One call, so the
        set the gate looked at is exactly the set a declaration then names."""
        with self._lock:
            if not self.machine.may_declare():
                if self.machine.state.name == "RUNNING":
                    return ("cannot declare during a run -- end it first, "
                            "then declare the next one"), ()
                return (
                    "cannot declare in %s -- need the retained RxCourse first"
                    % self.machine.state.name
                ), ()

            # Only what is actually on the water. See active_ids().
            now = time.time() if now is None else now
            active = self.active_ids(now)
            if len(active) < self.cfg.min_declared:
                return (
                    "cannot declare %d vehicle(s) -- need at least %d in the "
                    "run_declaration (handbook 3.1, for the Communications "
                    "Proof of Readiness). Active: %s. Start a reporter, or "
                    "force one in from the page."
                    % (len(active), self.cfg.min_declared,
                       ", ".join(active) or "nothing")
                ), active

            problems = self._rc_rules(active, callsign) + self._uav_gate(active, now)
            if problems:
                return "cannot declare -- " + "; ".join(problems), active
            return None, active

    def _rc_rules(self, active, callsign: str | None) -> list[str]:
        """What RoboCommand itself refuses a declaration with an aircraft for, and
        refuses SILENTLY -- no reply, the run just never starts: its rules on the
        fence and the ceiling (geofence.rc_declaration_problems), then a blank
        pilot call sign when one is given. Empty for a declaration without an
        aircraft. Caller holds the lock."""
        if not self._any_uav(active):
            return []
        out = geofence.rc_declaration_problems(
            self._fence.polygon,            # None: _uav_gate says why there is none
            _latlons(self.course.get("corners")),
            _latlons(self.course.get("uav_transit_area")),
            self.cfg.geofence_ceiling_hae_m, self.cfg.geofence_ceiling_limit_hae_m)
        if callsign is not None and not callsign.strip():
            out.append("the UAV pilot's call sign is blank: type it next to Declare "
                       "(MyDroneFleets: first name + UAPL number)")
        return out

    def _uav_gate(self, active, now: float) -> list[str]:
        """One problem per declared aircraft whose autopilot is not KNOWN to hold
        this course's fence (geofence.confirmation_problem has the rule). Empty
        when every aircraft going in is confirmed, and for a declaration with none:
        the gate is about aircraft, and the surface and underwater vehicles are not
        bound by the fence. Caller holds the lock."""
        out = []
        for vid in active:
            if not self.cfg.vehicle(vid).is_uav:
                continue
            rec = self._fence_confirms.get(vid)
            why = geofence.confirmation_problem(
                vid, self._fence, rec["body"] if rec else None, now,
                self.cfg.geofence_confirm_max_age_s)
            if why:
                out.append(why)
        return out

    def declare(self, callsign: str | None = None) -> str:
        """Publish the RunDeclaration. This is what creates the obligations.

        `callsign` is the page's box; None falls back to `[uav] pilot_callsign`.
        Only a declaration with an aircraft carries it and the ceiling."""
        callsign = (self.cfg.uav_pilot_callsign if callsign is None else callsign).strip()
        with self._lock:
            blocked, active = self._check_declare(callsign=callsign)
            if blocked:
                return blocked

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
            _fill_fence(decl, self._fence.polygon)
            if self._any_uav(active):
                # Both checked by _rc_rules above: set, in range, not blank.
                decl.uav_geofence_max_altitude_hae_m = self.cfg.geofence_ceiling_hae_m
                decl.uav_pilot_callsign = callsign

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
            self._all_auto_since = None
            self._declared = tuple(active)
            self._course_changed_since = None
            self.seqs.save_run(self.machine.state.name,
                               self.machine.declaration_seq, self.machine.run_id)

        self.rc.publish(self.cfg.rc_request_topic(), payload, qos=1)
        self.wire.frame(
            "tx", self.cfg.rc_request_topic(), payload,
            note="RunDeclaration seq=%d [%s]" % (seq, ",".join(active)),
            decoded=lambda: _wire_dict(req),
        )
        self._run_event("declare", "RunDeclaration seq=%d [%s]"
                        % (seq, ",".join(active)))
        # After the declaration is on the wire, not before: the vehicles are
        # being told to stand by for a run we have actually declared.
        self._send_standby_auto(active, seq)
        # The declared geofence goes to the declared aircraft, as information.
        self._send_info([vid for vid in active if self.cfg.vehicle(vid).is_uav],
                        "declaration")
        return "%s; declared %s" % (v.reason, ", ".join(active))

    def end(self) -> str:
        with self._lock:
            v = self.machine.on_end()
            if v.ok:
                self._course_changed_since = None
                self.seqs.save_run(self.machine.state.name,
                                   self.machine.declaration_seq, self.machine.run_id)
        if v.ok:
            self._run_event("end", v.reason)
        return v.reason

    def _run_event(self, kind: str, detail: str) -> None:
        """A wire-log event stamped with the run it belongs to.

        Declaration, RunStart and end each write one, so a run can be cut out
        of the file by run_id (or by declaration_seq, which is known from the
        moment of declaring -- run_id only arrives with RunStart).
        """
        with self._lock:
            m = self.machine
            extra = {"run_id": m.run_id, "declaration_seq": m.declaration_seq,
                     "state": m.state.name}
        self.wire.event(kind, detail, **extra)

    def snapshot(self) -> dict:
        """Everything the web page shows, in one read."""
        now = time.time()
        # Read outside the bridge lock. tcpserver holds its own, and always
        # taking them bridge-then-tcp is cheaper to keep true than an
        # argument about why no inversion can happen.
        peers = self.tcp.peers() if self.tcp is not None else {}
        sockets = self.tcp.all_links() if self.tcp is not None else {}
        mono = time.monotonic()
        with self._lock:
            c, m = self.counters, self.machine
            active_now = self.active_ids(now)
            blocked = self.declare_blocker(now)
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
                "beacon": self._beacon_view(),
                "link_prefer_s": self.cfg.link_prefer_s,
                # Whether the evidence log is complete. errors > 0 means a line
                # (or a decode) failed and the file has a gap -- the page says so.
                "wire": {"path": str(self.wire.path),
                         "errors": self.wire.errors,
                         "last_error": self.wire.last_error},
                "run": {"state": m.state.name,
                        "declaration_seq": m.declaration_seq,
                        "run_id": m.run_id,
                        "runs_declared": m.runs_declared,
                        # The WHOLE verdict -- state, the two-vehicle floor and the UAV
                        # fence gate -- and, when it is false, the reason in words.
                        # null reason = declarable. The page's button reads these.
                        "may_declare": blocked is None,
                        "declare_blocker": blocked,
                        # A course that replaced the one we declared on: the
                        # declaration_seq it arrived under, or null. End, re-declare.
                        "course_changed_since": self._course_changed_since,
                        # When every declared vehicle became STATE_AUTO, or null.
                        # RoboCommand may hold RunStart past it (Garuda, the course
                        # manager), so the page counts, it does not time out.
                        "all_auto_since": self._all_auto_since,
                        "declared": list(self._declared)},
                # What a declaration with an aircraft carries besides the fence. The
                # call sign here is only the default for the page's box.
                "uav": {"in_declaration": self._any_uav(active_now),
                        "ceiling_hae_m": self.cfg.geofence_ceiling_hae_m,
                        "ceiling_limit_hae_m": self.cfg.geofence_ceiling_limit_hae_m,
                        "pilot_callsign": self.cfg.uav_pilot_callsign,
                        "registration": {v.id: v.registration for v in self.cfg.vehicles
                                         if v.is_uav}},
                # Every task report (not heartbeats) and what became of it, newest
                # first: held / dropped / published / acknowledged by the broker.
                "task_reports": [dict(e) for e in reversed(self.task_reports)],
                "course": self.course,
                "course_findings": list(self.course_findings),
                # The UAV geofence for the CURRENT course (closed, RunDeclaration's own JSON
                # shape), or null -- the /map draws it next to the course and to what Ekko's
                # autopilot actually holds (fleet_link's fence_upload). `uav_geofence_source` says
                # where it came from ("table: ALPHA", "course boundary inset 2.0 m"); when
                # there is no fence it is null and `uav_geofence_reason` says why.
                "uav_geofence": self._fence_json,
                "uav_geofence_source": self._fence.source,
                "uav_geofence_reason": self._fence.reason,
                # Each aircraft's latest `ocs_fence`, as received, with its age by our clock
                # and whether its polygon is the one above: {"UAV1": {..., "age_s",
                # "matches_course_fence"}}. Only vehicles that have sent one appear.
                "fence_confirm": self._fence_confirm_view(now),
                "fence_confirm_max_age_s": self.cfg.geofence_confirm_max_age_s,
                "active": list(self.active_ids(now)),
                "por_min_vehicles": self.cfg.min_declared,
                # The bench dry-run switch. Always present so a page need not
                # guess: empty `vehicles` means off.
                "bench_auto": {
                    "vehicles": list(self.bench_auto),
                    "substituted": c.bench_auto_counts(self.bench_auto),
                },
                "awaiting_auto": self.awaiting_auto(),
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
                     # Per transport, null where there is no socket. `active_link`
                     # is the one a command would go down right now.
                     "prefer": v.prefer,
                     "links": _links_view(sockets.get(v.id, {}), mono,
                                          self._last_report_rx(v.id),
                                          self.cfg.link_prefer_s),
                     "active_link": next(iter(self._link_order(
                         v.id, mono, sockets.get(v.id, {}))), None),
                     "dropped_not_preferred":
                         c.dropped_not_preferred.get(v.id, 0),
                     "dropped_not_preferred_by_kind":
                         dict(c.dropped_not_preferred_kinds.get(v.id, {})),
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
                             "dropped_misregistered": c.dropped_misregistered,
                             "dropped_not_preferred":
                                 sum(c.dropped_not_preferred.values()),
                             "stale": c.stale,
                             "commands_routed": c.commands_routed,
                             "commands_undeliverable": c.commands_undeliverable,
                             "fence_frames_ignored": c.fence_frames_ignored},
            }

    def _fence_confirm_view(self, now: float) -> dict:
        """vehicle_id -> its latest `ocs_fence` as received, plus `age_s` (seconds since
        its read-back by OUR clock, None when it carries no readable `verified_at`)
        and `matches_course_fence`. Config order. Caller holds the lock."""
        out = {}
        for vid in self.cfg.vehicle_ids:
            rec = self._fence_confirms.get(vid)
            if rec is None:
                continue
            age = geofence.age_s(rec["body"], now)
            out[vid] = dict(rec["body"],
                            age_s=None if age is None else round(age, 1),
                            matches_course_fence=geofence.matches(self._fence, rec["body"]))
        return out


    def _beacon_view(self) -> dict:
        """The beacon's state for the page, with the reason when there is none."""
        if self.beacon is not None:
            return self.beacon.snapshot()
        d = self.cfg.discovery
        return off_snapshot(
            d.port,
            "[discovery] enabled is not true in bridge.toml" if not d.enabled
            else "the TCP vehicle link is off")

    def status(self) -> str:
        peers = self.tcp.peers() if self.tcp is not None else {}
        sockets = self.tcp.all_links() if self.tcp is not None else {}
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
                if len(sockets.get(spec.id, {})) > 1:
                    # Both links up: name them, and what each has cost.
                    link = "%s (+%d dup dropped)" % (
                        "+".join(sorted(sockets[spec.id])),
                        c.dropped_not_preferred.get(spec.id, 0))
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
                "uav fence  %s" % self._fence_line(now),
                "active     %s%s" % (
                    ", ".join(self.active_ids()) or "nothing",
                    "" if len(self.active_ids()) >= self.cfg.min_declared
                    else "   << below the %d required to declare"
                         % self.cfg.min_declared,
                ),
                "awaiting   %s" % (
                    ", ".join(self.awaiting_auto()) or "nothing -- all AUTO"
                ),
            ]
            if self.bench_auto:
                lines.append(
                    "BENCH AUTO %s published as AUTO while MANUAL (%s) -- "
                    "never at competition"
                    % (", ".join(self.bench_auto),
                       ", ".join("%s=%d" % kv for kv in
                                 c.bench_auto_counts(self.bench_auto).items())))
            lines += [
                "vehicle link %s" % (
                    "off" if self.tcp is None
                    else "%s:%d, %d connected" % (self.cfg.tcp_host,
                                                 self.cfg.tcp_port, len(peers))
                ),
                "beacon     %s" % _beacon_line(self._beacon_view()),
                "seq        %s" % (self.seqs.snapshot(),),
                "wire log   %s%s" % (
                    self.wire.path,
                    "" if not self.wire.errors
                    else "   << %d WRITE ERROR(S), log has gaps: %s"
                         % (self.wire.errors, self.wire.last_error)),
            ]
            return "\n".join(lines)


    def _fence_line(self, now: float) -> str:
        """One console line: the course's fence and each aircraft's confirmation.
        Caller holds the lock."""
        f = self._fence
        head = ("none: %s" % f.reason if f.polygon is None
                else "%s, %d vertices" % (f.source, f.vertices))
        views = self._fence_confirm_view(now)
        for vid, c in views.items():
            age = "age unknown" if c["age_s"] is None else "%.0f s old" % c["age_s"]
            what = ("cleared" if c["polygon"] == [] else
                    "holds the fence" if c["matches_course_fence"] else "holds another polygon")
            head += "; %s %s, %s, %s" % (
                vid, "verified" if c["verified"] else "NOT verified (%s)" % (c["state"] or "?"),
                what, age)
        return head


def _beacon_line(b: dict) -> str:
    """One console line for the beacon snapshot."""
    if not b["enabled"]:
        return "off (%s)" % b["why"]
    on = ", ".join(b["sending_on"]) or "NOWHERE (no eligible address)"
    return "UDP %d from %s; %d sent, %d skipped, %d errors" % (
        b["port"], on, b["sent"], len(b["skipped"]), b["errors"])


def _decode(msg) -> dict:
    """Protobuf -> dict, keeping the handbook's own field names so the page and
    the spec can be read side by side."""
    return json_format.MessageToDict(
        msg, always_print_fields_with_no_presence=True,
        preserving_proto_field_name=True,
    )


def _parse_synthesized(raw) -> tuple[frozenset[str], str | None]:
    """A hello's `synthesized` as a set of report kinds, or (empty, why it is bad).

    Absent (None) means the link synthesizes nothing. Anything else must be a
    list of REPORT_KINDS names -- a misspelt kind would otherwise be accepted and
    do nothing, leaving a rebuilt report to outrank the vehicle's own.
    """
    if raw is None:
        return frozenset(), None
    if not isinstance(raw, list):
        return frozenset(), ("synthesized must be a list of report kinds, got %s"
                             % type(raw).__name__)
    bad = [k for k in raw if not isinstance(k, str) or k not in REPORT_KINDS]
    if bad:
        return frozenset(), ("synthesized names %s, not a report kind (%s)"
                             % (", ".join(repr(k) for k in bad), ", ".join(REPORT_KINDS)))
    return frozenset(raw), None


def _links_view(sockets: dict, mono: float, report_rx: dict, fresh_s: float) -> dict:
    """{link: {connected, last_rx_age_s, last_report_age_s, fresh, peer, label,
    agent, hello, adapter, synthesized} | None} for every link.

    None for a link with no socket -- "not connected" and "connected, quiet"
    must read differently, and an age of 0 would say the wrong one.

    A socket being up and a vehicle being heard are different facts. `last_rx_age_s`
    is the last frame of ANY kind (a hello counts); `last_report_age_s` is the last
    REPORT, null when the link has relayed none, and `fresh` is that report being
    within `fresh_s`. A radio adapter that is connected and has relayed nothing
    has `adapter` true and `last_report_age_s` null -- "adapter up, vehicle silent".
    `peer` is the address; `label` is how to read it ("radio via fleet_link").
    """
    def view(link: str) -> dict | None:
        info = sockets.get(link)
        if info is None:
            return None
        got = report_rx.get(link)
        return {"connected": True,
                "last_rx_age_s": round(mono - info["last_rx"], 2),
                "last_report_age_s": None if got is None else round(mono - got, 2),
                "fresh": linkpolicy.is_fresh(got, mono, fresh_s),
                "peer": info["who"], "label": info["label"],
                "agent": info["agent"], "hello": info["hello"],
                "adapter": info["adapter"], "synthesized": info["synthesized"]}
    return {link: view(link) for link in LINKS}


def _wire_dict(msg) -> dict:
    """Protobuf -> the readable form the wire log keeps beside the raw bytes.

    The handbook's own field names, like _decode and _to_json (so a wire line,
    the page and the vehicle link all read alike), but only the fields that were
    set: this is a record of what was ON the wire, not of the schema's defaults.
    """
    return json_format.MessageToDict(msg, preserving_proto_field_name=True)


def _latlons(points) -> list[tuple[float, float]]:
    """[(lat, lon)] from a decoded repeated LatLng; [] when the field is absent."""
    return [(p.get("latitude", 0.0), p.get("longitude", 0.0)) for p in points or ()]


def _acknowledged(entry: dict) -> None:
    """A task report's PUBACK arrived: RoboCommand's broker has it."""
    entry["verdict"] = "acknowledged"
    entry["acked_at"] = time.time()
    entry["detail"] = "seq=%d, RoboCommand's broker acknowledged it (PUBACK)" % entry["seq"]


def _without_sent_at(course: dict) -> dict:
    return {k: v for k, v in course.items() if k != "sent_at"}


def _fill_fence(decl, polygon) -> None:
    """Put a closed geofence ((lat, lon), ...) into a RunDeclaration, point by point.
    None adds nothing: an empty uav_geofence is correct when there is no fence, and
    a declaration with an aircraft in it is never built without one (the gate)."""
    for lat, lon in polygon or ():
        decl.uav_geofence.add(latitude=lat, longitude=lon)


def _fence_json(polygon) -> list | None:
    """The fence in RunDeclaration.uav_geofence's own JSON shape, or None.

    Built from a real RunDeclaration and decoded the way every other message is,
    so what a vehicle is told cannot drift from what RoboCommand is sent.
    """
    if not polygon:
        return None
    decl = rx_requests_pb2.RunDeclaration()
    _fill_fence(decl, polygon)
    return _decode(decl)["uav_geofence"]


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
