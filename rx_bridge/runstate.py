"""runstate — the run's state machine, and nothing else.

No MQTT, no protobuf, no I/O: the handbook's start-of-run sequence is a set of
rules about ordering, and rules you can exercise on a laptop are rules you can
trust at the dock. bridge.py owns the sockets; this file owns what is allowed.

THE SEQUENCE (handbook 3.4), which is the reason this file exists:

    subscribe -> receive RxCourse -> publish RunDeclaration
              -> publish heartbeats -> receive RunStart matching declaration_seq

Note where heartbeats begin: AFTER the declaration and BEFORE RunStart, not
after the run starts. A bridge that waits for RunStart before reporting will sit
silent through the exact window RoboCommand is watching to confirm the team is
alive. So DECLARED permits reports; everything earlier does not.

THE RECONNECT RULE. On a dropped connection we return to whatever state we were
in, and we DO NOT re-declare. Re-declaring mints a new RxRequest.seq, and
RoboCommand's RunStart carries the declaration_seq it intends to answer -- so a
reflexive re-declare on reconnect orphans the RunStart we are waiting for and
the run never starts. Restoring subscriptions is bridge.py's job; remembering
that we were already declared is this file's.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class RunState(Enum):
    DISCONNECTED = auto()   # no MQTT session to RoboCommand
    CONNECTED = auto()      # session up, subscriptions restored, no course yet
    COURSE_RX = auto()      # retained RxCourse in hand; may declare
    DECLARED = auto()       # RunDeclaration published; reports flow; awaiting RunStart
    RUNNING = auto()        # RunStart received and matched
    ENDED = auto()          # run over; reports stop


#: States in which RxReport traffic is legal. See the module docstring.
_REPORTING = frozenset({RunState.DECLARED, RunState.RUNNING})

#: States we restore to after a reconnect rather than rewinding to CONNECTED.
_STICKY = frozenset({RunState.DECLARED, RunState.RUNNING})

#: States from which a RunDeclaration may be published.
#:
#: ENDED is here because a timeslot holds more than one run. Finishing a
#: run must not strand the OCS: the course we were given is still the
#: course, so the next declaration needs no reconnect and no restart --
#: end, declare, go again.
#:
#: RUNNING is deliberately NOT here. Declaring over a live run would leave
#: the fleet flying one run while the OCS reports another, and the operator
#: pressing 'end' first is the whole cost of ruling that out.
_DECLARABLE = frozenset({RunState.COURSE_RX, RunState.ENDED})


@dataclass
class Verdict:
    """Every transition answers 'did it happen, and if not why not'.

    Callers log `reason` verbatim. A rejected transition is not an exception:
    RoboCommand can and will send us things out of order, and the bridge's job
    is to refuse them and stay up, not to die.
    """
    ok: bool
    reason: str = ""

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class RunMachine:
    state: RunState = RunState.DISCONNECTED

    #: RxRequest.seq of the RunDeclaration we published. RunStart must match it.
    declaration_seq: int | None = None
    #: run_id from RunStart, carried into the wire log so a run's frames group.
    run_id: int | None = None
    #: Latest RxCourse, kept as an opaque object; only bridge.py reads inside it.
    course: object | None = None
    #: How many runs we have declared this session. For the operator, who
    #: needs to know whether the RunStart on screen belongs to this run or
    #: the last one.
    runs_declared: int = 0

    _prior: RunState = field(default=RunState.DISCONNECTED, repr=False)

    # ---- reload after a process restart -----------------------------------

    def restore(self, state_name: str, declaration_seq: int | None,
                run_id: int | None) -> Verdict:
        """Reload a run from disk. Sets where we RETURN to, not where we are.

        The socket does not exist yet at construction time, so restoring
        straight into DECLARED would claim a connection we do not have. Instead
        this primes the same `_prior` that a dropped TCP connection sets, and
        the next on_connect() lands us back in DECLARED or RUNNING through
        exactly one code path rather than two.
        """
        try:
            prior = RunState[state_name]
        except KeyError:
            return Verdict(False, "unknown persisted state %r; starting clean"
                                  % state_name)
        if prior not in _STICKY:
            return Verdict(False, "persisted state %s is not resumable; starting clean"
                                  % prior.name)
        self._prior = prior
        self.declaration_seq = declaration_seq
        self.run_id = run_id
        self.state = RunState.DISCONNECTED
        return Verdict(True, "restored %s (declaration_seq=%s) -- will resume on connect"
                             % (prior.name, declaration_seq))

    # ---- connection edges -------------------------------------------------

    def on_connect(self) -> Verdict:
        """MQTT session established and subscriptions (re)issued."""
        if self.state is not RunState.DISCONNECTED:
            return Verdict(False, f"on_connect in {self.state.name}; ignored")
        if self._prior in _STICKY:
            self.state = self._prior
            return Verdict(True, f"reconnected into {self.state.name}; NOT re-declaring")
        self.state = RunState.CONNECTED
        return Verdict(True, "connected")

    def on_disconnect(self) -> Verdict:
        if self.state is RunState.DISCONNECTED:
            return Verdict(False, "already disconnected")
        self._prior = self.state
        self.state = RunState.DISCONNECTED
        return Verdict(True, f"disconnected from {self._prior.name}")

    # ---- inbound from RoboCommand ----------------------------------------

    def on_course(self, course: object) -> Verdict:
        """Retained RxCourse. Arrives on every reconnect; only the first advances."""
        if self.state is RunState.DISCONNECTED:
            return Verdict(False, "course while disconnected; ignored")
        self.course = course
        if self.state is RunState.CONNECTED:
            self.state = RunState.COURSE_RX
            return Verdict(True, "course received; may declare")
        return Verdict(True, f"course refreshed in {self.state.name}")

    def on_run_start(self, declaration_seq: int, run_id: int) -> Verdict:
        """RunStart is only ours if its declaration_seq matches what we sent.

        A mismatch is not a hiccup to paper over. It means RoboCommand answered
        a declaration that is not the one we are holding -- a stale frame, or
        another team's run leaking through a topic mistake. Starting on it would
        move the boat against the wrong course configuration.
        """
        if self.state is not RunState.DECLARED:
            return Verdict(False, f"RunStart in {self.state.name}; expected DECLARED")
        if declaration_seq != self.declaration_seq:
            return Verdict(
                False,
                f"RunStart declaration_seq={declaration_seq} does not match ours "
                f"({self.declaration_seq}); REFUSED -- do not start",
            )
        self.run_id = run_id
        self.state = RunState.RUNNING
        return Verdict(True, f"run {run_id} started")

    # ---- outbound, driven by the operator --------------------------------

    def on_declared(self, seq: int) -> Verdict:
        """Called *after* the RunDeclaration is on the wire, with its seq."""
        if self.state not in _DECLARABLE:
            want = " or ".join(sorted(s.name for s in _DECLARABLE))
            hint = ("  -- end the current run first"
                    if self.state is RunState.RUNNING else "")
            return Verdict(
                False,
                f"declare in {self.state.name}; expected {want}{hint}",
            )
        again = self.state is RunState.ENDED
        # The previous run's id must not survive into this one. Left set, the
        # page shows a run_id while we are only DECLARED, and the operator
        # reads a run that has not started yet as already running.
        self.run_id = None
        self.declaration_seq = seq
        self.state = RunState.DECLARED
        self.runs_declared += 1
        word = "re-declared" if again else "declared"
        # "by this process", not "this timeslot": a restart restores the run
        # but not the count, so claiming a timeslot-wide run number would be
        # wrong exactly when someone is relying on it after a crash.
        return Verdict(True, f"{word} with seq={seq} (declaration "
                             f"{self.runs_declared} by this process); "
                             f"awaiting RunStart")

    def on_end(self) -> Verdict:
        if self.state not in _REPORTING:
            return Verdict(False, f"end in {self.state.name}; nothing running")
        self.state = RunState.ENDED
        self._prior = RunState.ENDED
        return Verdict(True, "run ended")

    # ---- predicates the bridge asks before it acts -----------------------

    def may_declare(self) -> bool:
        return self.state in _DECLARABLE

    def may_report(self) -> bool:
        return self.state in _REPORTING
