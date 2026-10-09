"""maproutes -- the fleet map page's routes, with no bridge in them.

Pulled out of server.py so they can be tested without a Bridge, a broker or a
protobuf runtime, and so the page's whole server side reads in one file.

    GET /map                         map.html
    GET /vendor/leaflet/leaflet.js   Leaflet 1.9.4, served from disk, no CDN
    GET /vendor/leaflet/leaflet.css
    GET /api/fleet                   fleet_link's GET /fleet, proxied
    GET /api/map-config              map centre + the links to each vehicle GUI

THE VENDOR ROUTES ARE A WHITELIST, NOT A DIRECTORY. Two exact paths, each mapped
to a fixed file name here. Nothing from the request is ever joined onto a path,
so there is no `..` to defend against: the URL is looked up, never interpreted.

THE PROXY EXISTS SO THE PAGE NEEDS ONE ORIGIN. fleet_link does send
Access-Control-Allow-Origin: *, but going through here means the browser never
has to be told where fleet_link is, and a refusal comes back as an error body
the page can explain ("run FLEET_LINK.cmd") instead of an opaque failed fetch.
The URL is fixed at startup by --fleet-url; no part of the request chooses it, so
this is not an open proxy.

BLANKS OVER GUESSES, HERE TOO. When fleet_link does not answer, the reply is an
error and nothing else. It is never the last good body: a position that looks
current but is a minute old is the one thing the map must not be able to show.
"""
from __future__ import annotations

import json
import math
import tomllib
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

HERE = Path(__file__).resolve().parent

DEFAULT_FLEET_URL = "http://127.0.0.1:8098/fleet"
FLEET_TIMEOUT_S = 0.7
FLEET_MAX_BYTES = 2_000_000
FLEET_SCHEMA = 1

#: The team's local lake / pool test site. The venue is Singapore (~1.2806,
#: 103.8557); `[map] center = [lat, lon]` in the --config TOML overrides this.
DEFAULT_CENTER = (32.92357, -117.03857)

#: THE one list of where each vehicle's own GUI lives. url None = "whatever
#: --fleet-url says", so the fleet_link link cannot disagree with the proxy.
#: Replaced wholesale (not merged) by `[[map.gui]]` entries in the --config TOML.
DEFAULT_GUIS = (
    {"name": "Crusader — laptop GUI", "url": "http://localhost:8150",
     "note": "BOAT_GUI.cmd"},
    {"name": "Crusader — Jetson", "url": "http://192.168.100.109:8090",
     "note": ""},
    {"name": "Ekko — laptop GUI", "url": "http://localhost:8090",
     "note": "DRONE_GUI.cmd"},
    {"name": "Ekko — Jetson", "url": "http://ekko.local:8090", "note": ""},
    {"name": "Graey — Jetson", "url": "http://graey.local:8090", "note": ""},
    {"name": "fleet_link status", "url": None, "note": "FLEET_LINK.cmd"},
    {"name": "OCS main page", "url": "/", "note": ""},
)

#: exact URL path -> (file under ocs_web/, Content-Type)
_VENDOR = {
    "/vendor/leaflet/leaflet.js": ("vendor/leaflet/leaflet.js",
                                   "application/javascript; charset=utf-8"),
    "/vendor/leaflet/leaflet.css": ("vendor/leaflet/leaflet.css",
                                    "text/css; charset=utf-8"),
}

_HTML = "text/html; charset=utf-8"
_JSON = "application/json"

# Never let an environment HTTP_PROXY sit between this process and loopback.
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


# ---------------------------------------------------------------- /api/fleet

def _reason(exc: BaseException, timeout: float) -> str:
    """A short cause the operator can act on, never a traceback."""
    if isinstance(exc, urllib.error.HTTPError):
        return "HTTP %d" % exc.code
    inner = exc.reason if isinstance(exc, urllib.error.URLError) else exc
    if isinstance(inner, TimeoutError):          # socket.timeout is an alias
        # urllib wraps a CONNECT timeout in URLError and lets a read timeout
        # through bare. Windows takes ~2 s to report a refused loopback
        # connect (it retries the SYN), so with a 0.7 s budget "fleet_link is
        # not running" arrives here as a connect timeout, not as a refusal.
        if isinstance(exc, urllib.error.URLError):
            return "no answer within %.1f s (not running?)" % timeout
        return "connected but no reply within %.1f s" % timeout
    if isinstance(inner, ConnectionRefusedError):
        return "connection refused"
    return "%s: %s" % (type(inner).__name__, inner)


def fetch_fleet(url: str, timeout: float = FLEET_TIMEOUT_S) -> dict:
    """fleet_link's /fleet body, parsed -- or {"error", "fleet_url"}.

    Never raises. Anything that is not a schema-1 JSON object is an error: a
    different schema may mean different field meanings, and guessing at them is
    how a stale number gets drawn as a position.
    """
    def err(why: str) -> dict:
        return {"error": why, "fleet_url": url}

    try:
        with _OPENER.open(url, timeout=timeout) as resp:
            body = resp.read(FLEET_MAX_BYTES + 1)
    except Exception as exc:                     # noqa: BLE001 - reported, not hidden
        return err(_reason(exc, timeout))
    if len(body) > FLEET_MAX_BYTES:
        return err("reply larger than %d bytes" % FLEET_MAX_BYTES)
    try:
        data = json.loads(body)
    except ValueError:
        return err("reply was not JSON")
    if not isinstance(data, dict):
        return err("reply was not a JSON object")
    if data.get("schema") != FLEET_SCHEMA:
        return err("unexpected fleet schema %r (this page reads %d)"
                   % (data.get("schema"), FLEET_SCHEMA))
    return data


# ------------------------------------------------------------ /api/map-config

def load_map_table(path) -> dict:
    """The `[map]` table of the --config TOML, or {}.

    server.py does not keep the parsed TOML (rx_bridge.config.load reads the file
    and returns a dataclass), so this reads it a second time rather than touching
    that loader. By the time this runs `load` has already accepted the file, so
    an error here is a race with an editor and {} is the right answer.
    """
    try:
        raw = tomllib.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    table = raw.get("map")
    return table if isinstance(table, dict) else {}


def _center(raw) -> list[float] | None:
    if (isinstance(raw, (list, tuple)) and len(raw) == 2
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in raw)
            and abs(raw[0]) <= 90 and abs(raw[1]) <= 180):
        return [float(raw[0]), float(raw[1])]
    return None


def _gui(raw) -> dict | None:
    """One link entry, or None if it is not usable.

    The url goes into an href, so it must be http(s) or a same-origin path: a
    config typo should not be able to put a javascript: link on the operator's
    screen.
    """
    if not isinstance(raw, dict):
        return None
    name, url = raw.get("name"), raw.get("url")
    note = raw.get("note", "")
    if not (isinstance(name, str) and name.strip()
            and isinstance(url, str) and isinstance(note, str)):
        return None
    parts = urlsplit(url)
    is_http = parts.scheme in ("http", "https") and bool(parts.netloc)
    is_local_path = url.startswith("/") and not url.startswith("//")
    if not (is_http or is_local_path):
        return None
    return {"name": name.strip(), "url": url, "note": note}


def build_map_config(table: dict | None, fleet_url: str) -> dict:
    """What GET /api/map-config returns. Computed once, at startup."""
    table = table or {}
    warnings: list[str] = []

    center, center_source = list(DEFAULT_CENTER), "default"
    if "center" in table:
        c = _center(table["center"])
        if c:
            center, center_source = c, "config"
        else:
            warnings.append("[map] center ignored: want [lat, lon] in range")

    # url None in DEFAULT_GUIS means "the --fleet-url in use"; a config entry
    # with no url is simply invalid, so the substitution is for defaults only.
    guis = [g for g in (_gui({**d, "url": d["url"] or fleet_url})
                        for d in DEFAULT_GUIS) if g]
    guis_source = "default"
    if "gui" in table:
        raw = table["gui"] if isinstance(table["gui"], list) else []
        parsed = [g for g in (_gui(d) for d in raw) if g]
        if parsed:
            guis, guis_source = parsed, "config"
            if len(parsed) != len(raw):
                warnings.append("[[map.gui]]: %d of %d entries ignored "
                                "(need name and an http(s) or / url)"
                                % (len(raw) - len(parsed), len(raw)))
        else:
            warnings.append("[[map.gui]] ignored: no usable entry; "
                            "showing the defaults")

    return {"center": center, "center_source": center_source,
            "guis": guis, "guis_source": guis_source,
            "fleet_url": fleet_url, "warnings": warnings}


# --------------------------------------------------------------------- routing

def _file(rel: str, ctype: str) -> tuple[int, bytes, str]:
    try:
        return 200, (HERE / rel).read_bytes(), ctype
    except OSError:
        return 404, ("missing on disk: ocs_web/%s" % rel).encode(), "text/plain"


def route(path: str, *, fleet_url: str, map_cfg: dict,
          timeout: float = FLEET_TIMEOUT_S):
    """(status, body, content_type) for a GET this module owns, else None.

    None means "not mine": the caller carries on with its own routes and its own
    404, so adding this changes nothing about any path it did not claim.
    """
    p = urlsplit(path).path
    if p in ("/map", "/map/"):
        return _file("map.html", _HTML)
    if p in _VENDOR:
        return _file(*_VENDOR[p])
    if p == "/api/fleet":
        return 200, json.dumps(fetch_fleet(fleet_url, timeout)).encode(), _JSON
    if p == "/api/map-config":
        return 200, json.dumps(map_cfg).encode(), _JSON
    return None
