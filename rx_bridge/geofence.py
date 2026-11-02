"""geofence — the UAV geofence for the course in hand, and the gate on declaring an aircraft.

Pure: no sockets, no clock of its own, no protobuf. bridge.py holds the state and
the connections; this file holds the geometry and the rules, so they can be
exercised on a laptop with nothing running -- the split active.py and runstate.py
already use.

WHERE THE FENCE COMES FROM, "table, else the boundary". There are four courses
(ALPHA .. DELTA), each a different place, so one static polygon in bridge.toml
was only ever right for one of them. When RoboCommand's RxCourse is accepted the
OCS works out THIS course's fence:

  1. If bridge.toml has a `[[geofence.course]]` entry for that course_id, use it --
     but only if every vertex lies inside the course boundary (or within
     TABLE_TOLERANCE_M of it). An entry that is outside is a typed-in fence for the
     wrong place, and it is refused with the vertex named. It never falls through
     to (2): a table entry somebody wrote on purpose and got wrong must be seen,
     not quietly replaced by something they did not choose.
  2. Otherwise the boundary itself, shrunk INWARD by `[geofence] boundary_margin_m`.
     Handbook 3.4.10: RoboCommand validates the declared fence against the course
     boundary. A fence that IS the boundary sits exactly on the line where a
     float-rounding comparison can reject it, so the vertices are pulled inside by
     a margin (default 2 m) that is the operator's to change.

No course yet means no fence (NO_COURSE): there is nothing to be inside of. And
NO FENCE IS NEVER A GUESS -- when the table entry is bad, or the boundary is not
a shape the inset is trustworthy for (it must be a simple CONVEX polygon, and the
inset must not collapse it), the answer is `polygon=None` with the reason, and the
operator is told loudly. A convex polygon is required because the inset offsets
every edge inward and joins neighbours; at a reflex corner that join is a
different construction altogether, and a wrong fence that looks right is exactly
what this OCS exists to avoid.

THE GATE. An aircraft whose autopilot is not KNOWN to hold the declared fence is
not declared. The knowledge is a frame the radio adapter sends after reading the
fence back from the autopilot (`ocs_fence`, docs/VEHICLE_LINK.md), and the rule
for believing it is `confirmation_problem` below: verified, the same polygon, and
recent. The adapter writes the fence; this file decides whether the OCS may rely
on that having happened.

GEOMETRY. Lat/lon are turned into a local flat metre frame around the boundary's
mean vertex with the WGS84 radii of curvature at that latitude. At course scale
(hundreds of metres) the flat-earth error is far below a millimetre, and the only
place a metre figure matters -- the margin -- is not near a hazard.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timezone

LatLon = tuple[float, float]

#: `[geofence] boundary_margin_m` when bridge.toml says nothing. Strictly inside
#: the boundary, never on it -- see the module docstring. Chase's to change.
DEFAULT_MARGIN_M = 2.0

#: `[geofence] confirm_max_age_s` when bridge.toml says nothing. The adapter
#: re-verifies every 60 s, so three of its cycles can be lost before the OCS stops
#: believing the aircraft still holds the fence.
DEFAULT_CONFIRM_MAX_AGE_S = 180.0

#: How far outside the boundary a `[[geofence.course]]` vertex may be and still
#: count as inside it: the width of a hand-typed coordinate, not of a mistake.
TABLE_TOLERANCE_M = 0.5

#: How far a vertex where the course meets its transit area may sit off the straight
#: line through its neighbours and still be no corner. The two rings share the seam's
#: endpoints but their other edges were surveyed separately, so the "corner" a union
#: leaves there is rounding, not shape -- 4 mm on the test server's Course Alpha. Left
#: in, it reads as a reflex corner and the convex inset refuses the whole union.
_SEAM_TOL_M = 0.05

#: RoboCommand's limits for a declared UAV fence, as its test server applies them
#: (robocommand@8fc09f2, RobotX_2026/rc_test/test_server.py `geofence_problems`).
RC_TOLERANCE_M = 1.0
RC_EDGE_STEP_M = 0.5
RC_MAX_CEILING_HAE_M = 50.0

#: A confirmation's polygon must match the fence to this, per coordinate. About
#: 0.1 m; ArduPilot stores 1e-7 deg, so a faithful read-back is well inside it.
MATCH_TOL_DEG = 1e-6

#: Two boundary vertices closer than this are one vertex (a ring that repeats its
#: first corner at the end, or a corner entered twice).
_SAME_VERTEX_M = 0.01

#: sin of the turn at a vertex below which the two edges are one straight edge.
_COLLINEAR = 1e-6

#: A confirmation timestamped this far ahead of the OCS clock is not believed: an
#: age of "-3 minutes" would pass any freshness limit.
_FUTURE_S = 5.0

#: Decimal places kept in a computed vertex: 1e-8 deg is about a millimetre.
_ROUND_DP = 8

# WGS84.
_A = 6378137.0
_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class CourseFence:
    """The UAV geofence for the course in hand, or the reason there is none."""

    course_id: str | None
    #: CLOSED (first == last), as RunDeclaration.uav_geofence needs it. None = none.
    polygon: tuple[LatLon, ...] | None = None
    #: Where it came from: "table: ALPHA", "course boundary inset 2.0 m". None when
    #: there is no fence.
    source: str | None = None
    #: Why there is no fence. None when there is one.
    reason: str | None = None

    @property
    def open_polygon(self) -> tuple[LatLon, ...] | None:
        """The same polygon with the closing vertex removed -- the form the
        adapter writes to the autopilot and reports back."""
        return None if self.polygon is None else self.polygon[:-1]

    @property
    def vertices(self) -> int:
        return 0 if self.polygon is None else len(self.polygon) - 1


#: Before any course has been received. Not a refusal: nothing to refuse yet.
NO_COURSE = CourseFence(None, reason="no course received yet")


def fmt_m(x: float) -> str:
    """A metre figure as the operator would type it: 2.0, 1.75, 12.5."""
    s = ("%.3f" % x).rstrip("0")
    return s + "0" if s.endswith(".") else s


# ---- the local flat frame ---------------------------------------------------

class _Frame:
    """East/north metres around (lat0, lon0)."""

    def __init__(self, lat0: float, lon0: float) -> None:
        phi = math.radians(lat0)
        w = math.sqrt(1.0 - _E2 * math.sin(phi) ** 2)
        self.lat0, self.lon0 = lat0, lon0
        self.m_per_deg_lat = math.radians(1.0) * _A * (1.0 - _E2) / w ** 3
        self.m_per_deg_lon = math.radians(1.0) * _A / w * math.cos(phi)

    def xy(self, lat: float, lon: float) -> tuple[float, float]:
        return ((lon - self.lon0) * self.m_per_deg_lon,
                (lat - self.lat0) * self.m_per_deg_lat)

    def ll(self, x: float, y: float) -> LatLon:
        return (round(self.lat0 + y / self.m_per_deg_lat, _ROUND_DP),
                round(self.lon0 + x / self.m_per_deg_lon, _ROUND_DP))


def _cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def _signed_area(pts) -> float:
    n = len(pts)
    return 0.5 * sum(_cross(*pts[i], *pts[(i + 1) % n]) for i in range(n))


def _inside(p, ring) -> bool:
    """Even-odd ray cast: is `p` inside the ring (any simple polygon)?"""
    x, y = p
    inside = False
    n = len(ring)
    for i in range(n):
        (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _dist_to_ring(p, ring) -> float:
    """Metres from `p` to the nearest point on the ring's edges."""
    best = math.inf
    n = len(ring)
    for i in range(n):
        (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / L2))
        best = min(best, math.hypot(p[0] - (x1 + t * dx), p[1] - (y1 + t * dy)))
    return best


def _distinct(pts):
    """Consecutive near-duplicates removed, and a repeated closing vertex too."""
    out: list = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > _SAME_VERTEX_M:
            out.append(p)
    while len(out) > 1 and math.hypot(out[0][0] - out[-1][0],
                                      out[0][1] - out[-1][1]) <= _SAME_VERTEX_M:
        out.pop()
    return out


def _drop_straight_through(pts):
    """Vertices where the boundary does not turn are not corners. Without this two
    collinear edges would give parallel offset lines, which never meet."""
    pts = list(pts)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            d1, d2 = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
            n1, n2 = math.hypot(*d1), math.hypot(*d2)
            # dot > 0: straight THROUGH. A reversal (a spike) is left in place,
            # and fails the convexity test below as the defect it is.
            if (abs(_cross(*d1, *d2)) <= _COLLINEAR * n1 * n2
                    and d1[0] * d2[0] + d1[1] * d2[1] > 0):
                del pts[i]
                changed = True
                break
    return pts


# ---- the inset --------------------------------------------------------------

def _inset(pts, margin: float):
    """(inset ring in the same order, None, None) or (None, why not, kind).

    `kind` is "shape" when the boundary is not something an inset is trustworthy for
    (only a table entry can help) and "margin" when the margin is the problem (a
    smaller one would also do).

    `pts` is the boundary in local metres, open (no repeated closing vertex), in
    either winding. Each edge is moved `margin` along its inward normal and each
    vertex becomes the meeting point of its two moved edges, so vertex i of the
    answer is the inset of vertex i of the input, in the same order and winding.
    """
    pts = _drop_straight_through(_distinct(pts))
    n = len(pts)
    no_area = ("boundary corners do not enclose an area (%d distinct corner(s))" % n,
               "shape")
    if n < 3:
        return (None,) + no_area

    turn, straight = [], 0
    for i in range(n):
        d1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        d2 = (pts[(i + 1) % n][0] - pts[i][0], pts[(i + 1) % n][1] - pts[i][1])
        c = _cross(*d1, *d2)
        if abs(c) <= _COLLINEAR * math.hypot(*d1) * math.hypot(*d2):
            straight += 1
        turn.append((c, math.atan2(c, d1[0] * d2[0] + d1[1] * d2[1])))
    if straight == n:
        return (None,) + no_area
    if straight:
        return None, "boundary not convex (a corner doubles back on itself)", "shape"
    if len({c > 0 for c, _ in turn}) > 1:
        return None, "boundary not convex (it turns both ways)", "shape"
    if abs(abs(sum(a for _, a in turn)) - 2 * math.pi) > 1e-6:
        return None, "boundary is not a simple polygon (it winds round more than once)", "shape"
    if abs(_signed_area(pts)) < 1.0:
        return (None,) + no_area

    s = 1.0 if _signed_area(pts) > 0 else -1.0
    norm, line = [], []
    for i in range(n):
        dx, dy = pts[(i + 1) % n][0] - pts[i][0], pts[(i + 1) % n][1] - pts[i][1]
        L = math.hypot(dx, dy)
        nx, ny = -dy * s / L, dx * s / L                   # inward, whichever the winding
        norm.append((nx, ny))
        line.append(nx * pts[i][0] + ny * pts[i][1] + margin)

    # The narrowest the course is, measured across each edge: the most a
    # margin can ever be is half of it, from both sides.
    width = min(max(norm[i][0] * (p[0] - pts[i][0]) + norm[i][1] * (p[1] - pts[i][1])
                    for p in pts) for i in range(n))
    collapse = ("a margin of %s m collapses a course %s m wide at its narrowest"
                % (fmt_m(margin), "%.1f" % width))
    if width <= 2 * margin:
        return None, collapse, "margin"

    out = []
    for i in range(n):                       # vertex i joins edge i-1 and edge i
        (ax, ay), (bx, by) = norm[i - 1], norm[i]
        ca, cb = line[i - 1], line[i]
        det = _cross(ax, ay, bx, by)
        out.append(((ca * by - ay * cb) / det, (ax * cb - ca * bx) / det))

    # Belt and braces against a shape the arithmetic above survived but the
    # geometry did not: an edge that vanished inverts (a short edge between two
    # long ones), and a vertex pushed past another edge is no longer inside.
    for i in range(n):
        old = (pts[(i + 1) % n][0] - pts[i][0], pts[(i + 1) % n][1] - pts[i][1])
        new = (out[(i + 1) % n][0] - out[i][0], out[(i + 1) % n][1] - out[i][1])
        if old[0] * new[0] + old[1] * new[1] <= 0:
            return None, collapse, "margin"
    area = _signed_area(out)
    if area * s <= 0 or abs(area) < 1.0:
        return None, collapse, "margin"
    for p in out:
        for i in range(n):
            if (norm[i][0] * (p[0] - pts[i][0])
                    + norm[i][1] * (p[1] - pts[i][1])) < margin - 1e-4:
                return None, collapse, "margin"
    return out, None, None


# ---- the fence for one course -----------------------------------------------

def _valid_ll(points) -> bool:
    # Not merely NaN: a position off the globe would turn into metres the arithmetic
    # below happily overflows, and a fence made of NaN is worse than none.
    return all(math.isfinite(a) and math.isfinite(b) and abs(a) <= 90.0 and abs(b) <= 180.0
               for a, b in points)


def for_course(course_id: str, corners, table, margin_m: float, transit=()) -> CourseFence:
    """The UAV geofence for a course: the table entry, else the boundary inset.

    corners   [(lat, lon), ...] as RoboCommand sent them -- open or closed, either
              winding.
    table     {course_id: closed ((lat, lon), ...)} from `[[geofence.course]]`.
    margin_m  `[geofence] boundary_margin_m`.
    transit   RxCourse.uav_transit_area, the same way, or empty. RoboCommand accepts a
              fence anywhere in the course OR this area, which is where Ekko takes off
              and lands; a course-only fence leaves the launch point outside it.

    With a transit area that shares a whole edge with the course, the inset is of the
    two together; when they cannot be joined (no shared edge, or not convex together)
    the course alone is inset as before and `source` says the transit area was left
    out. A table entry is checked against course and transit area together.

    Returns a CourseFence; `polygon` is None, with `reason`, whenever the answer
    is "no fence". Never raises on a bad course: a frame we cannot make a fence
    from is a finding, and the OCS stays up to show it.
    """
    cid = course_id or "?"
    corners = [(float(a), float(b)) for a, b in corners]
    transit = [(float(a), float(b)) for a, b in transit]
    if not _valid_ll(corners):
        return CourseFence(cid, reason="the course boundary has a corner that is not a "
                                       "valid latitude/longitude (not a number, or out of range)")
    if not corners:
        return CourseFence(cid, reason="the course has no boundary corners")
    frame = _Frame(sum(a for a, _ in corners) / len(corners),
                   sum(b for _, b in corners) / len(corners))
    ring = _distinct([frame.xy(a, b) for a, b in corners])
    left_out = None
    tring = []
    if transit and not _valid_ll(transit):
        left_out = "it has a point that is not a valid latitude/longitude"
    elif transit:
        tring = _distinct([frame.xy(a, b) for a, b in transit])
        if len(tring) < 3:
            left_out = "it has only %d distinct point(s)" % len(tring)
            tring = []

    entry = table.get(course_id)
    if entry is not None:
        return _from_table(cid, entry, [ring] + ([tring] if tring else []), frame)

    if len(ring) < 3:
        return CourseFence(cid, reason=(
            "the course boundary has only %d distinct corner(s), not enough to "
            "inset: add a [[geofence.course]] entry for %s" % (len(ring), cid)))
    if tring:
        union, left_out = _union(ring, tring)
        if union is not None:
            inset, why, _kind = _inset(union, margin_m)
            if inset is not None:
                open_ring = [frame.ll(x, y) for x, y in inset]
                return CourseFence(cid, tuple(open_ring) + (open_ring[0],),
                                   source="course + transit area inset %s m" % fmt_m(margin_m))
            left_out = "course and transit area together: %s" % why
    inset, why, kind = _inset(ring, margin_m)
    if inset is None:
        advice = ("lower [geofence] boundary_margin_m or add a" if kind == "margin"
                  else "add a")
        return CourseFence(cid, reason="%s: %s [[geofence.course]] entry for %s"
                                       % (why, advice, cid))
    open_ring = [frame.ll(x, y) for x, y in inset]
    source = "course boundary inset %s m" % fmt_m(margin_m)
    if left_out:
        source += ("; transit area NOT included (%s) -- add a [[geofence.course]] entry "
                   "to use it" % left_out)
    return CourseFence(cid, tuple(open_ring) + (open_ring[0],), source=source)


def _union(ring, tring):
    """(the course and its transit area as one ring, None), or (None, why not).

    Only the shape RoboCommand's courses have: two simple rings sharing one whole edge.
    Both are wound counter-clockwise, so the transit area runs the shared edge a -> b
    the other way, b -> a. Walk the course from b all the way round to a, then the
    transit area from just after a to just before b: the seam is gone, and its two
    endpoints go too when the boundary runs straight through them (_SEAM_TOL_M).
    """
    course = ring if _signed_area(ring) > 0 else ring[::-1]
    area = tring if _signed_area(tring) > 0 else tring[::-1]
    n, m = len(course), len(area)
    same = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1]) <= _SEAM_TOL_M  # noqa: E731
    for i in range(n):
        a, b = course[i], course[(i + 1) % n]
        for j in range(m):
            if same(area[j], b) and same(area[(j + 1) % m], a):
                out = [course[(i + 1 + k) % n] for k in range(n)]
                out += [area[(j + 2 + k) % m] for k in range(m - 2)]
                for k in (n - 1, 0):                          # a, then b
                    p, q, r = out[k - 1], out[k], out[(k + 1) % len(out)]
                    span = (r[0] - p[0], r[1] - p[1])
                    run = (q[0] - p[0], q[1] - p[1])
                    L = math.hypot(*span)
                    if (L > 0 and abs(_cross(*span, *run)) / L <= _SEAM_TOL_M
                            and run[0] * span[0] + run[1] * span[1] > 0):
                        del out[k]
                return out, None
    return None, "it shares no whole edge with the course boundary"


def _from_table(cid: str, entry, areas, frame: _Frame) -> CourseFence:
    """A `[[geofence.course]]` entry, if and only if all of it -- every point along every
    edge, as RoboCommand checks it -- lies inside the course or the transit area."""
    ring = areas[0]
    if len(ring) < 3:
        return CourseFence(cid, reason=(
            "the table fence for %s cannot be checked: the course boundary has only "
            "%d distinct corner(s)" % (cid, len(ring))))
    where = "the course boundary" if len(areas) == 1 else "the course and transit area"

    def outside_by(p) -> float:
        return min(0.0 if _inside(p, a) else _dist_to_ring(p, a) for a in areas)

    outside = []
    for i, (lat, lon) in enumerate(entry[:-1], start=1):
        d = outside_by(frame.xy(lat, lon))
        if d > TABLE_TOLERANCE_M:
            outside.append((i, lat, lon, d))
    if outside:
        i, lat, lon, d = outside[0]
        more = "" if len(outside) == 1 else " (and %d more vertices outside)" % (
            len(outside) - 1)
        return CourseFence(cid, reason=(
            "the [[geofence.course]] entry for %s is outside %s: "
            "vertex %d [%s, %s] is %.1f m outside it%s -- no fence for this course "
            "until the entry is fixed or removed"
            % (cid, where, i, repr(lat), repr(lon), d, more)))
    # Every vertex inside is not every edge inside: one across a notch leaves.
    pts = [frame.xy(lat, lon) for lat, lon in entry]
    for i, (p, q) in enumerate(zip(pts, pts[1:]), start=1):
        steps = max(1, math.ceil(math.dist(p, q) / RC_EDGE_STEP_M))
        worst = max(outside_by((p[0] + (q[0] - p[0]) * k / steps, p[1] + (q[1] - p[1]) * k / steps))
                    for k in range(steps + 1))
        if worst > TABLE_TOLERANCE_M:
            return CourseFence(cid, reason=(
                "the [[geofence.course]] entry for %s leaves %s along edge %d-%d, by "
                "%.1f m -- no fence for this course until the entry is fixed or removed"
                % (cid, where, i, i + 1 if i < len(pts) - 1 else 1, worst)))
    return CourseFence(cid, tuple((float(a), float(b)) for a, b in entry),
                       source="table: %s" % cid)


# ---- RoboCommand's own check of a declaration -------------------------------

def rc_declaration_problems(fence, corners, transit, ceiling_m,
                            max_ceiling_m: float = RC_MAX_CEILING_HAE_M) -> list[str]:
    """Why RoboCommand would refuse a declaration with a UAV; empty when it would not.

    A TRANSCRIPTION of the test server's `geofence_problems` (see RC_* above), not a
    re-derivation: the same sphere, origin, sampling and wording, so the page and their
    log say the same thing. It has to exist because RoboCommand answers a refused
    declaration with nothing at all -- the run just never starts.

    fence     the closed uav_geofence about to be declared; None judges the ceiling
              alone (there is no fence to declare, and the caller says why)
    corners   RxCourse.corners; transit RxCourse.uav_transit_area (may be empty)
    ceiling_m uav_geofence_max_altitude_hae_m, or None when there is none to send
    """
    def closed(pts):
        pts = [(float(a), float(b)) for a, b in pts]
        return pts + pts[:1] if pts and pts[0] != pts[-1] else pts

    rings = [r for r in (closed(corners), closed(transit)) if len(r) >= 4]
    problems = []
    fence = None if fence is None else closed(fence)
    if fence is None:
        pass
    elif len(fence) < 4:
        problems.append("uav_geofence has %d point(s), needs at least 4" % len(fence))
    elif rings:
        origin = rings[0][0]
        r_earth = 6371000.0

        def xy(p):
            return (math.radians(p[1] - origin[1]) * r_earth * math.cos(math.radians(origin[0])),
                    math.radians(p[0] - origin[0]) * r_earth)

        def outside_by(p, ring) -> float:
            x, y = p
            inside = False
            for (x1, y1), (x2, y2) in zip(ring, ring[1:]):
                if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                    inside = not inside
            if inside:
                return 0.0
            best = math.inf
            for (ax, ay), (bx, by) in zip(ring, ring[1:]):
                dx, dy = bx - ax, by - ay
                t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy)
                                                         / (dx * dx + dy * dy)))
                best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
            return best

        areas = [[xy(p) for p in ring] for ring in rings]
        pts = [xy(p) for p in fence]
        worst, where = 0.0, None
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            steps = max(1, math.ceil(math.dist(a, b) / RC_EDGE_STEP_M))
            for k in range(steps + 1):
                q = (a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                out = min(outside_by(q, area) for area in areas)
                if out > worst:
                    worst = out
                    where = ("point %d" % (i + 1) if k == 0 else "point %d" % (i + 2)
                             if k == steps else "edge %d-%d" % (i + 1, i + 2))
        if worst > RC_TOLERANCE_M:
            problems.append("uav_geofence leaves the course and transit area by %.1f m at %s"
                            % (worst, where))
    if ceiling_m is None:
        problems.append("uav_geofence_max_altitude_hae_m is not set: [geofence] "
                        "max_altitude_hae_m in bridge.toml")
    elif not 0 < ceiling_m <= max_ceiling_m:
        problems.append("uav_geofence_max_altitude_hae_m is %g: it must be above 0 and at most %g"
                        % (ceiling_m, max_ceiling_m))
    return problems


# ---- the gate on declaring an aircraft --------------------------------------

def parse_confirmation(raw):
    """An `ocs_fence` body -> (the fields we keep, None), or (None, why it is bad).

    Strict about types and loose about omissions: `verified` is the field the gate
    trusts, so it must be a real boolean; the free-text fields are cut to a length
    the page can show. An absent polygon is None, never an empty list -- "the
    autopilot holds nothing" is `[]`, which the adapter says on purpose.
    """
    if not isinstance(raw, dict):
        return None, "ocs_fence is not an object"
    vid = raw.get("vehicle_id")
    if not isinstance(vid, str) or not vid:
        return None, "vehicle_id must be a non-empty string"
    verified = raw.get("verified")
    if not isinstance(verified, bool):
        return None, "verified must be true or false, got %r" % (verified,)
    poly = raw.get("polygon")
    if poly is not None:
        if not isinstance(poly, list):
            return None, "polygon must be a list of [lat, lon] or null"
        clean = []
        for p in poly:
            ok = (isinstance(p, (list, tuple)) and len(p) == 2
                  and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                          and math.isfinite(v) for v in p))
            if not ok:
                return None, "polygon holds %r, not a finite [lat, lon]" % (p,)
            clean.append([float(p[0]), float(p[1])])
        poly = clean
    at = raw.get("verified_at")
    if at is not None and not isinstance(at, str):
        return None, "verified_at must be an RFC 3339 string or null"
    for key in ("state", "reason", "lockout"):
        v = raw.get(key)
        if v is not None and not isinstance(v, str):
            return None, "%s must be a string or null" % key
    return {"vehicle_id": vid,
            "state": str(raw.get("state") or "")[:32],
            "polygon": poly,
            "verified": verified,
            "verified_at": at[:40] if at else None,
            "reason": str(raw.get("reason") or "")[:300],
            "lockout": (str(raw["lockout"])[:300] if raw.get("lockout") else None)}, None


def parse_utc(text) -> float | None:
    """An RFC 3339 UTC timestamp as epoch seconds, or None if it is not one. A
    timestamp with no zone is read as UTC, which is what the contract says it is."""
    if not isinstance(text, str):
        return None
    try:
        when = datetime.fromisoformat(text.strip().replace("Z", "+00:00"))
    except ValueError:
        return None
    if when.tzinfo is None:
        when = when.replace(tzinfo=timezone.utc)
    return when.timestamp()


def age_s(confirm: dict | None, now: float) -> float | None:
    """Seconds since the confirmation's read-back, by the OCS clock. None when it
    carries no usable `verified_at` -- a blank, not a guess."""
    at = parse_utc(confirm.get("verified_at")) if confirm else None
    return None if at is None else now - at


def same_polygon(a, b, tol: float = MATCH_TOL_DEG) -> bool:
    """Two OPEN polygons, vertex for vertex, each coordinate within `tol` degrees."""
    if a is None or b is None or len(a) != len(b):
        return False
    return all(abs(p[0] - q[0]) <= tol and abs(p[1] - q[1]) <= tol
               for p, q in zip(a, b))


def matches(fence: CourseFence, confirm: dict | None) -> bool:
    """Does this confirmation's polygon equal the open form of the fence?"""
    return (fence.polygon is not None and confirm is not None
            and same_polygon(confirm.get("polygon"), fence.open_polygon))


def confirmation_problem(vid: str, fence: CourseFence, confirm: dict | None,
                         now: float, max_age_s: float) -> str | None:
    """Why `vid` may not be declared, or None when its autopilot is known to hold
    the fence. The rule, in the order the operator can act on it:

      * there must be a fence for this course at all;
      * the aircraft's latest `ocs_fence` must say `verified` (read back);
      * its polygon must equal the fence, each coordinate within 1e-6 deg;
      * and that read-back must be no older than `max_age_s` by OUR clock.

    The message names the vehicle and the one thing that is missing.
    """
    if fence.polygon is None:
        return ("%s: no UAV geofence for this course (%s) -- an aircraft cannot be "
                "declared without one" % (vid, fence.reason or "none"))
    cid = fence.course_id or "this course"
    if confirm is None:
        return ("%s: autopilot fence not confirmed (no ocs_fence received from it yet)"
                % vid)
    if not confirm.get("verified"):
        detail = confirm.get("reason") or "no reason given"
        return ("%s: autopilot fence not confirmed (state %s: %s)"
                % (vid, confirm.get("state") or "unknown", detail))
    if not matches(fence, confirm):
        poly = confirm.get("polygon")
        what = ("it was cleared: no polygon" if poly == []
                else "it holds no polygon at all" if poly is None
                else "%d vertices, the fence has %d" % (len(poly), fence.vertices))
        return ("%s: autopilot holds a different polygon than the %s fence (%s)"
                % (vid, cid, what))
    age = age_s(confirm, now)
    if age is None:
        return ("%s: fence confirmation has no readable verified_at, so its age "
                "is unknown" % vid)
    if age < -_FUTURE_S:
        return ("%s: fence confirmation is dated %.0f s in the future -- the clocks "
                "disagree" % (vid, -age))
    if age > max_age_s:
        return "%s: fence confirmation %.0f s old (limit %.0f s)" % (vid, age, max_age_s)
    return None
