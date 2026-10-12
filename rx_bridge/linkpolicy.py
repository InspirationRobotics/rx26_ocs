"""linkpolicy — which of a vehicle's links the OCS listens to and talks on.

Pure: no clock, no sockets, no protobuf -- the caller passes the time in. Same
split as governor.py, runstate.py and active.py, and for the same reason: a rule
you can exercise on a laptop with nothing running is a rule you can trust at the
dock.

A vehicle may be connected on two links at once -- Ekko over the RFD radio (the
laptop-side fleet_link adapter) and over team WiFi (its own ocs_client). The two
do NOT carry the same things: the radio adapter relays what the radio carries
(and may SYNTHESIZE a heartbeat from the autopilot stream until the vehicle sends
its own), while WiFi carries the vehicle's own heartbeats, task reports,
IncidentAcks and Readiness. Two questions follow, and this file answers both:

  RECEIVING. Which report do we keep? `duplicate_reason` decides per report KIND
  (the RxReport body oneof name: heartbeat, safe_passage, pipeline_survey,
  resource_delivery, docking, firefighting, incident_ack, readiness). A report
  of kind K that arrives on link L is dropped only when another link M has just
  delivered the same kind K and M's copy outranks L's:

      1. REAL beats SYNTHESIZED -- a link says in its hello which kinds it
         synthesizes; a copy rebuilt from a side channel never displaces the
         vehicle's own report, whichever link either arrived on.
      2. Among copies of the same standing, the vehicle's `prefer` link wins.
         With no preference, equals both stand.

  A kind that has only ever come down one link is never dropped: the other
  link's silence on that kind is not a reason to throw this one away, which is
  how a WiFi-only task report used to vanish while the radio was up. The rule
  runs BEFORE staleness and the governor, so the governor never sees a doubled
  rate (and so drops a legitimate frame as a "rate problem"), and no sequence
  number is burned on a frame we then throw away.

  SENDING. Which link carries a command, a directive or an info frame?
  `send_order` ranks the connected links. A frame goes to the first one that
  accepts it and to NO other -- a command delivered twice, once per link, is a
  command a vehicle may act on twice.

"Fresh" means the link delivered something within `fresh_s` (cfg.link_prefer_s).
A link that is connected but silent is not fresh, and is tried only after any
fresh one -- but it is still tried before giving up, because a socket that is up
is a better bet than a broker nobody on the vehicle subscribes to.
"""
from __future__ import annotations

#: Tie-break when two links are equally fresh and neither is preferred: the
#: wider pipe first.
_TIEBREAK = ("wifi", "radio")


def is_fresh(last_rx: float | None, now: float, fresh_s: float) -> bool:
    """True if `last_rx` (the same clock as `now`) is within `fresh_s`."""
    return last_rx is not None and (now - last_rx) <= fresh_s


def _outranks(prefer: str | None, other: str, other_synthesized: bool,
              link: str, synthesized: bool) -> bool:
    """True if `other`'s copy of a kind beats `link`'s. Strict: never both ways."""
    if other_synthesized != synthesized:
        return synthesized              # real beats synthesized, whatever the link
    return other == prefer


def duplicate_reason(prefer: str | None, link: str, kind: str, synthesized: bool,
                     arrivals: dict[str, tuple[float, bool]], now: float,
                     fresh_s: float) -> str | None:
    """Why a report that just arrived on `link` should be dropped, or None.

    kind:        the RxReport body name (see the module docstring); here it only
                 words the reason -- the caller has already filed `arrivals` by it.
    synthesized: True if `link` declared this kind as synthesized in its hello.
    arrivals:    link -> (clock time, synthesized) of the last report OF THIS KIND
                 that arrived on it, NOT counting the one being decided. A frame
                 the governor or validator later refuses still counts as an
                 arrival: link choice is about which pipe is alive, not about
                 whether its content was good.
    """
    for other in sorted(arrivals):
        if other == link:
            continue
        seen, other_synth = arrivals[other]
        if not is_fresh(seen, now, fresh_s):
            continue
        if not _outranks(prefer, other, other_synth, link, synthesized):
            continue
        if other_synth == synthesized:
            return ("not preferred: %s delivered %s %.2f s ago (limit %.1f s)"
                    % (other, kind, now - seen, fresh_s))
        return ("not preferred: this %s is synthesized on %s; %s delivered a real "
                "one %.2f s ago (limit %.1f s)"
                % (kind, link, other, now - seen, fresh_s))
    return None


def send_order(connected: dict[str, float], prefer: str | None, now: float,
               fresh_s: float) -> list[str]:
    """Connected links, best first. `connected` maps link -> last_rx time.

    Order: fresh before stale; within that, the preferred link first; then the
    wider pipe. Empty if nothing is connected, which is the caller's cue to fall
    back to MQTT.
    """
    def rank(link: str):
        fresh = is_fresh(connected[link], now, fresh_s)
        tie = _TIEBREAK.index(link) if link in _TIEBREAK else len(_TIEBREAK)
        return (0 if fresh else 1, 0 if link == prefer else 1, tie, link)

    return sorted(connected, key=rank)
