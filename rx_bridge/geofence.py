"""geofence — RoboCommand's polygon for the course in hand, and what a declaration with an aircraft carries.

Pure: no sockets, no clock of its own, no protobuf. bridge.py holds the state and
the connections; this file holds the geometry and the rules, so they can be
exercised on a laptop with nothing running -- the split active.py and runstate.py
already use.

THE WORKFLOW (Chase, 7 Nov 2026). The OCS is read-only on the aircraft but for one
polygon, and it declares only what the aircraft itself holds:

  1. RoboCommand's RxCourse arrives. `for_course` turns it into ROBOCOMMAND'S
     POLYGON: the course joined with its UAV transit area, shrunk INWARD by
     `[geofence] boundary_margin_m`. The bridge sends it to the aircraft in
     `ocs_info`, and fleet_link's fence_upload puts it on the autopilot as an
     inclusion polygon, so QGC shows it.
  2. The pilot draws Ekko's OWN fence inside it in QGC, uploads it, and sets
     FENCE_ALT_MAX.
  3. fleet_link reads the autopilot every 30 s while it is disarmed and sends what
     it holds (`ocs_fence`: every shape, the FENCE_* params, home HAE).
     `aircraft_fence` picks what a declaration carries from that read: the ONE
     inclusion polygon that is not RoboCommand's, and the ceiling home HAE +
     FENCE_ALT_MAX. Nothing is typed into the OCS and nothing is made up.

THE INSET. Handbook 3.4.10: RoboCommand validates a declared fence against the
course. A polygon that IS the boundary sits exactly on the line where a
float-rounding comparison can reject it, so the vertices are pulled inside by a
margin (default 2 m) that is the operator's to change. No course yet means no
polygon (NO_COURSE): there is nothing to be inside of. And NO POLYGON IS NEVER A
GUESS -- when the boundary is not a shape the inset is trustworthy for (it must be
a simple CONVEX polygon, and the inset must not collapse it), the answer is
`polygon=None` with the reason. A convex polygon is required because the inset
offsets every edge inward and joins neighbours; at a reflex corner that join is a
different construction altogether. Without it nothing is put on the aircraft, and
the pilot draws against the course on /map instead.

THE GATE. An aircraft is declared only from a RECENT read of its autopilot holding
exactly one polygon of the pilot's and a known ceiling (`aircraft_fence`), and only
when that passes RoboCommand's own rule (`rc_declaration_problems`), which it
applies SILENTLY.

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

#: `[geofence] confirm_max_age_s` when bridge.toml says nothing. The adapter reads
#: the autopilot every 30 s while it is disarmed, so several reads can be lost
#: before the OCS stops believing what the last one said.
DEFAULT_CONFIRM_MAX_AGE_S = 180.0

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
    """RoboCommand's polygon for the course in hand (what is put on the aircraft for the
    pilot to draw inside), or the reason there is none."""

    course_id: str | None
    #: CLOSED (first == last), as `ocs_info.uav_geofence` carries it. None = none.
    polygon: tuple[LatLon, ...] | None = None
    #: How it was made: "course + transit area inset 2.0 m", "course boundary inset
    #: 2.0 m". None when there is none.
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
    (only drawing it by hand can help) and "margin" when the margin is the problem (a
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


#: Said whenever RoboCommand's polygon cannot be made: nothing is put on the aircraft.
_DRAW_ANYWAY = ("so nothing is put on the aircraft for this course -- draw Ekko's fence in QGC "
                "against the course on /map")


def for_course(course_id: str, corners, margin_m: float, transit=()) -> CourseFence:
    """RoboCommand's polygon for a course: the course (joined with its transit area)
    inset by `margin_m`.

    corners   [(lat, lon), ...] as RoboCommand sent them -- open or closed, either
              winding.
    margin_m  `[geofence] boundary_margin_m`.
    transit   RxCourse.uav_transit_area, the same way, or empty. RoboCommand accepts a
              fence anywhere in the course OR this area, which is where Ekko takes off
              and lands; a course-only polygon leaves the launch point outside it.

    With a transit area that shares a whole edge with the course, the inset is of the
    two together; when they cannot be joined (no shared edge, or not convex together)
    the course alone is inset and `source` says the transit area was left out.

    Returns a CourseFence; `polygon` is None, with `reason`, whenever the answer
    is "none". Never raises on a bad course: a frame we cannot make a polygon
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

    if len(ring) < 3:
        return CourseFence(cid, reason=(
            "the course boundary has only %d distinct corner(s), not enough to "
            "inset, %s" % (len(ring), _DRAW_ANYWAY)))
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
        return CourseFence(cid, reason="%s for %s, %s%s" % (
            why, cid, _DRAW_ANYWAY,
            " (or lower [geofence] boundary_margin_m)" if kind == "margin" else ""))
    open_ring = [frame.ll(x, y) for x, y in inset]
    source = "course boundary inset %s m" % fmt_m(margin_m)
    if left_out:
        source += ("; transit area NOT included (%s) -- a fence drawn in QGC may still "
                   "use it" % left_out)
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
        problems.append("uav_geofence_max_altitude_hae_m is not set")
    elif not 0 < ceiling_m <= max_ceiling_m:
        problems.append("uav_geofence_max_altitude_hae_m is %g: it must be above 0 and at most %g"
                        % (ceiling_m, max_ceiling_m))
    return problems


# ---- what the radio adapter read off the autopilot --------------------------

#: FENCE_TYPE bit 0: the altitude ceiling, FENCE_ALT_MAX (metres above home).
FENCE_TYPE_ALT_MAX = 1
#: FENCE_TYPE bit 2: inclusion/exclusion circles and polygons.
FENCE_TYPE_POLYGON = 4
#: FENCE_OPTIONS bit 1: inclusion areas are a UNION (ArduPilot's default is their intersection).
FENCE_OPTIONS_UNION = 2


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _finite_pair(p) -> bool:
    return isinstance(p, (list, tuple)) and len(p) == 2 and all(_finite(v) for v in p)


def _latlons_or_why(raw, what: str):
    """[[lat, lon], ...] of finite numbers -> (list, None), or (None, why not)."""
    if not isinstance(raw, list):
        return None, "%s must be a list of [lat, lon] or null" % what
    for p in raw:
        if not _finite_pair(p):
            return None, "%s holds %r, not a finite [lat, lon]" % (what, p)
    return [[float(p[0]), float(p[1])] for p in raw], None


def _shapes_or_why(raw):
    """The adapter's `shapes` (fleet/fence_upload.shapes) -> (list, None), or (None, why not).
    Absent stays None: never read, which is not the same as read and empty."""
    if raw is None:
        return None, None
    if not isinstance(raw, list):
        return None, "shapes must be a list or null"
    out = []
    for n, s in enumerate(raw):
        if not isinstance(s, dict) or not isinstance(s.get("inclusion"), bool):
            return None, "shape %d is not {type, inclusion, ...}" % n
        if s.get("type") == "polygon":
            pts, why = _latlons_or_why(s.get("points"), "shape %d's points" % n)
            if why:
                return None, why
            if len(pts) < 3:
                return None, "shape %d is a polygon of %d point(s)" % (n, len(pts))
            out.append({"type": "polygon", "inclusion": s["inclusion"], "points": pts})
        elif s.get("type") == "circle":
            if not (_finite_pair(s.get("center")) and _finite(s.get("radius_m"))):
                return None, "shape %d is a circle without a finite center and radius_m" % n
            out.append({"type": "circle", "inclusion": s["inclusion"],
                        "center": [float(v) for v in s["center"]], "radius_m": float(s["radius_m"])})
        else:
            return None, "shape %d has type %r, not polygon or circle" % (n, s.get("type"))
    return out, None


def _params_or_why(raw):
    """The adapter's FENCE_* `params` -> (dict, None), or (None, why not). A name that is
    not a FENCE_* parameter is not ours to judge and is left out."""
    if raw is None:
        return {}, None
    if not isinstance(raw, dict):
        return None, "params must be an object or null"
    out = {}
    for name, v in raw.items():
        if not (isinstance(name, str) and name.startswith("FENCE_") and len(name) <= 16):
            continue
        if v is not None and not _finite(v):
            return None, "params.%s must be a number or null, got %r" % (name, v)
        out[name] = None if v is None else float(v)
    return out, None


def parse_confirmation(raw):
    """An `ocs_fence` body -> (the fields we keep, None), or (None, why it is bad).

    Strict about types and loose about omissions: a field an older adapter does not
    send is None, and the gate then has nothing to go on rather than something wrong.
    The free-text fields are cut to a length the page can show.

      polygon     RoboCommand's (open) polygon the adapter was told to put on the
                  autopilot, or None; `verified` whether it was among the autopilot's
                  inclusion polygons at the last read, `verified_at` when.
      read_at     when the autopilot's fence was last read (RFC 3339); None: never.
      shapes      everything it held then (fleet/fence_upload.shapes); None: never read.
      params      the FENCE_* parameters as last heard; one never heard is None.
      home_hae_m  home altitude above the WGS84 ellipsoid at that read, or None.
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
        poly, why = _latlons_or_why(poly, "polygon")
        if why:
            return None, why
    for key in ("verified_at", "read_at"):
        v = raw.get(key)
        if v is not None and not isinstance(v, str):
            return None, "%s must be an RFC 3339 string or null" % key
    for key in ("state", "reason", "lockout"):
        v = raw.get(key)
        if v is not None and not isinstance(v, str):
            return None, "%s must be a string or null" % key
    shapes, why = _shapes_or_why(raw.get("shapes"))
    if why:
        return None, why
    params, why = _params_or_why(raw.get("params"))
    if why:
        return None, why
    home = raw.get("home_hae_m")
    if home is not None and not _finite(home):
        return None, "home_hae_m must be a number or null, got %r" % (home,)
    at, read_at = raw.get("verified_at"), raw.get("read_at")
    return {"vehicle_id": vid,
            "state": str(raw.get("state") or "")[:32],
            "polygon": poly,
            "verified": verified,
            "verified_at": at[:40] if at else None,
            "reason": str(raw.get("reason") or "")[:300],
            "lockout": (str(raw["lockout"])[:300] if raw.get("lockout") else None),
            "read_at": read_at[:40] if read_at else None,
            "shapes": shapes,
            "params": params,
            "home_hae_m": None if home is None else float(home)}, None


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


def age_s(confirm: dict | None, now: float, key: str = "read_at") -> float | None:
    """Seconds since the confirmation's `key` time (the read; "verified_at" for when
    RoboCommand's polygon was last seen on it), by the OCS clock. None when it carries
    no usable time -- a blank, not a guess."""
    at = parse_utc(confirm.get(key)) if confirm else None
    return None if at is None else now - at


def same_polygon(a, b, tol: float = MATCH_TOL_DEG) -> bool:
    """Two OPEN polygons, vertex for vertex, each coordinate within `tol` degrees."""
    if a is None or b is None or len(a) != len(b):
        return False
    return all(abs(p[0] - q[0]) <= tol and abs(p[1] - q[1]) <= tol
               for p, q in zip(a, b))


def same_ring(a, b, tol: float = MATCH_TOL_DEG) -> bool:
    """Two OPEN polygons that are one ring: from any starting vertex, in either
    direction, each coordinate within `tol` degrees. QGC keeps the order it downloaded;
    a polygon put back by hand need not."""
    if not a or not b or len(a) != len(b):
        return False
    n = len(a)
    for ring in (list(b), list(b)[::-1]):
        for k in range(n):
            if same_polygon(a, ring[k:] + ring[:k], tol):
                return True
    return False


def matches(fence: CourseFence, confirm: dict | None) -> bool:
    """Was the adapter told the polygon the bridge holds now (is it up to date)?"""
    return (fence.polygon is not None and confirm is not None
            and same_polygon(confirm.get("polygon"), fence.open_polygon))


# ---- what a declaration with an aircraft carries ----------------------------

@dataclass(frozen=True)
class AircraftFence:
    """What a declaration with this aircraft would carry, read off its own autopilot,
    or why it cannot be declared. `problems` empty: nothing on the aircraft's side stops
    it -- RoboCommand's own rule is judged on top (declaration_problems)."""

    vehicle_id: str
    #: The pilot's polygon, CLOSED, as RunDeclaration.uav_geofence carries it. None: none.
    polygon: tuple[LatLon, ...] | None = None
    #: Home HAE + FENCE_ALT_MAX: RunDeclaration.uav_geofence_max_altitude_hae_m. None: unknown.
    ceiling_hae_m: float | None = None
    #: How the ceiling was made, for the page and a refusal.
    ceiling_from: str | None = None
    #: Why it cannot be declared, in the order the operator can act on them.
    problems: tuple[str, ...] = ()
    #: Worth knowing before flight, never a reason to refuse.
    notes: tuple[str, ...] = ()
    #: Seconds since the read, by the OCS clock; None when there has been none.
    read_age_s: float | None = None


def aircraft_fence(vid: str, confirm: dict | None, ours, now: float,
                   max_age_s: float) -> AircraftFence:
    """What a declaration with `vid` carries, from its latest `ocs_fence` (as
    parse_confirmation keeps it).

    ours  OPEN polygons the OCS put there itself -- RoboCommand's for the course in
          hand, and the one the adapter says it was told -- which are never the pilot's.

    The rule, in the order the operator can act on it:
      * a read of the autopilot no older than `max_age_s` by OUR clock (the adapter
        reads only while the aircraft is disarmed and heard on the radio);
      * among the inclusion polygons it held, exactly ONE that is not one of `ours`:
        the pilot's, drawn in QGC. RunDeclaration carries a single polygon, and the
        OCS never chooses between two of the pilot's;
      * FENCE_ALT_MAX and home HAE both known: the ceiling is their sum.
    """
    def refused(why: str, age=None) -> AircraftFence:
        return AircraftFence(vid, problems=(why,), read_age_s=age)

    if confirm is None:
        return refused("%s: nothing read from its autopilot yet (no ocs_fence: is fleet_link "
                       "running, and the radio up?)" % vid)
    if confirm.get("read_at") is None or confirm.get("shapes") is None:
        return refused("%s: its autopilot's fence has not been read yet (%s)"
                       % (vid, confirm.get("reason") or confirm.get("state") or "no reason given"))
    age = age_s(confirm, now)
    if age is None:
        return refused("%s: the fence read has no readable read_at, so its age is unknown" % vid)
    if age < -_FUTURE_S:
        return refused("%s: the fence read is dated %.0f s in the future -- the clocks disagree"
                       % (vid, -age), age)
    if age > max_age_s:
        return refused("%s: its autopilot's fence was last read %.0f s ago (limit %.0f s) -- it is "
                       "read only while disarmed and heard on the radio" % (vid, age, max_age_s), age)

    shapes = confirm["shapes"]
    polygons = [[(p[0], p[1]) for p in s["points"]] for s in shapes
                if s["type"] == "polygon" and s["inclusion"]]
    ours = [o for o in ours if o]
    pilots = [p for p in polygons if not any(same_ring(p, o) for o in ours)]
    problems, notes = [], []
    polygon = None
    if len(pilots) == 1:
        polygon = tuple(pilots[0]) + (pilots[0][0],)
    elif not pilots:
        problems.append("%s: its autopilot holds %s -- draw Ekko's own fence %s in QGC and upload it"
                        % (vid, "only RoboCommand's polygon" if polygons else "no inclusion polygon",
                           "inside RoboCommand's" if polygons else "inside the course"))
    else:
        problems.append("%s: its autopilot holds %d inclusion polygons besides RoboCommand's, and a "
                        "declaration carries one -- leave only Ekko's own in QGC" % (vid, len(pilots)))

    params = confirm.get("params") or {}
    alt_max, home = params.get("FENCE_ALT_MAX"), confirm.get("home_hae_m")
    ceiling = ceiling_from = None
    if alt_max is None:
        problems.append("%s: FENCE_ALT_MAX has not been read from its autopilot, so there is no "
                        "ceiling to declare" % vid)
    elif home is None:
        problems.append("%s: its home altitude is unknown (no fresh position on the radio at the "
                        "read, or fleet.toml geoid_separation_m unset), so there is no ceiling to "
                        "declare" % vid)
    else:
        ceiling = round(home + alt_max, 2)
        ceiling_from = "home %s m HAE + FENCE_ALT_MAX %s m" % (fmt_m(home), fmt_m(alt_max))

    ftype, enable, auto, options = (params.get(n) for n in (
        "FENCE_TYPE", "FENCE_ENABLE", "FENCE_AUTOENABLE", "FENCE_OPTIONS"))
    if ftype is not None and not int(ftype) & FENCE_TYPE_POLYGON:
        notes.append("FENCE_TYPE %g has no polygon bit (4): the autopilot does not enforce the "
                     "polygon" % ftype)
    if ftype is not None and not int(ftype) & FENCE_TYPE_ALT_MAX:
        notes.append("FENCE_TYPE %g has no altitude bit (1): FENCE_ALT_MAX, the declared ceiling, "
                     "is not enforced" % ftype)
    if enable == 0 and auto == 0:
        notes.append("the fence is off (FENCE_ENABLE 0, FENCE_AUTOENABLE 0): enable it in QGC "
                     "before flight")
    if options is not None and int(options) & FENCE_OPTIONS_UNION and len(polygons) > 1:
        notes.append("FENCE_OPTIONS %g makes the inclusion areas a union: the aircraft is held "
                     "to the largest of them, not to the polygon declared" % options)
    circles = sum(1 for s in shapes if s["type"] == "circle")
    exclusions = sum(1 for s in shapes if s["type"] == "polygon" and not s["inclusion"])
    if circles or exclusions:
        notes.append("its autopilot also holds %s: enforced by it, never declared" % " and ".join(
            "%d %s%s" % (n, what, "" if n == 1 else "s")
            for n, what in ((circles, "circle"), (exclusions, "exclusion polygon")) if n))
    return AircraftFence(vid, polygon, ceiling, ceiling_from, tuple(problems), tuple(notes), age)


def declaration_problems(af: AircraftFence, corners, transit,
                         max_ceiling_m: float = RC_MAX_CEILING_HAE_M) -> list[str]:
    """Everything that keeps `af` out of a declaration: the aircraft's own problems, then
    RoboCommand's rule (rc_declaration_problems) on the polygon and ceiling read off it,
    each naming the aircraft and where the value came from."""
    out = list(af.problems)
    for p in rc_declaration_problems(af.polygon, corners, transit, af.ceiling_hae_m, max_ceiling_m):
        if p.startswith("uav_geofence_max_altitude_hae_m"):
            if af.ceiling_hae_m is None:
                continue                       # af.problems says why there is none
            p += " (%s%s)" % (af.ceiling_from, "; lower FENCE_ALT_MAX in QGC"
                              if af.ceiling_hae_m > max_ceiling_m else "")
        else:
            p += " (the polygon drawn in QGC)"
        out.append("%s: %s" % (af.vehicle_id, p))
    return out
