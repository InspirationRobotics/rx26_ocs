"""Tests for the fleet map page's server side: ocs_web/maproutes.py.

    .venv\\Scripts\\python.exe -m unittest discover -s tests -t .

The route logic is a pure stdlib module on purpose, so most of this needs no
bridge and no broker. The last class drives the REAL server.make_handler over
a socket, to prove server.py actually wires those routes in and that the paths
it did not hand over (`/`, `/nope`) behave as before; it needs paho and
protobuf, which the .venv has, and skips where they are missing.

Every socket here is on 127.0.0.1 with an OS-chosen port. Nothing binds 14550,
14554, 14570, 8098, 8099 or any COM port.
"""
from __future__ import annotations

import base64
import http.client
import importlib.util
import json
import re
import socket
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ocs_web import maproutes  # noqa: E402

VENDOR = Path(maproutes.HERE) / "vendor" / "leaflet"
STATIC = Path(maproutes.HERE) / "static"

#: (the route, the file under ocs_web/static/, its Content-Type, the first bytes of a real one)
LOGOS = (("/static/ti_logo.jpg", "ti_logo.jpg", "image/jpeg", b"\xff\xd8\xff"),
         ("/static/asta_logo.png", "asta_logo.png", "image/png", b"\x89PNG\r\n\x1a\n"))
#: The drone GUI embeds the same two images as data: URIs; present when the monorepo is checked out.
DRONE_LOGOS = (Path(__file__).resolve().parents[2] / "Drone" / "rx26_uav" / "uav_groundstation"
               / "uav_groundstation" / "team_logos.py")

# Nothing listening: Linux refuses at once; Windows retries the SYN for ~2 s,
# so inside the 0.7 s budget it surfaces as a connect timeout instead.
def is_refused(err: str) -> bool:
    return err == "connection refused" or err.startswith("no answer within")

FLEET_BODY = {"schema": 1, "time": 1791500000.0, "stale_after_s": 3.0,
              "links": {}, "sysids": {"2": {"sysid": 2, "name": "Crusader"}}}


class _Stub(ThreadingHTTPServer):
    """A fleet_link stand-in: serves whatever status/body/delay is set on it."""
    daemon_threads = True

    def __init__(self, status=200, body=None, delay=0.0):
        self.status, self.delay = status, delay
        self.body = json.dumps(FLEET_BODY).encode() if body is None else body

        stub = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                time.sleep(stub.delay)
                self.send_response(stub.status)
                self.send_header("Content-Length", str(len(stub.body)))
                self.end_headers()
                self.wfile.write(stub.body)

        super().__init__(("127.0.0.1", 0), H)
        # poll_interval: shutdown() waits out one poll, and the default 0.5 s
        # times twenty-odd servers is most of this file's runtime.
        threading.Thread(target=self.serve_forever, args=(0.01,), daemon=True).start()

    def handle_error(self, request, client_address):
        pass        # the slow-reply test hangs up on purpose; keep its noise off stderr

    @property
    def url(self) -> str:
        return "http://127.0.0.1:%d/fleet" % self.server_address[1]


def _closed_port_url() -> str:
    """A loopback URL that nothing is listening on."""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return "http://127.0.0.1:%d/fleet" % port


class StubTestCase(unittest.TestCase):
    def stub(self, **kw) -> _Stub:
        s = _Stub(**kw)
        self.addCleanup(s.server_close)
        self.addCleanup(s.shutdown)
        return s


class TestRoute(StubTestCase):
    """maproutes.route(), called directly."""

    def route(self, path, fleet_url=maproutes.DEFAULT_FLEET_URL, **kw):
        cfg = maproutes.build_map_config(None, fleet_url)
        return maproutes.route(path, fleet_url=fleet_url, map_cfg=cfg, **kw)

    def test_map_page(self):
        for path in ("/map", "/map/", "/map?x=1"):
            code, body, ctype = self.route(path)
            self.assertEqual(code, 200, path)
            self.assertEqual(ctype, "text/html; charset=utf-8")
            self.assertIn(b"/vendor/leaflet/leaflet.js", body)

    def test_vendor_files_exact_bytes_and_types(self):
        code, body, ctype = self.route("/vendor/leaflet/leaflet.js")
        self.assertEqual((code, ctype), (200, "application/javascript; charset=utf-8"))
        self.assertEqual(body, (VENDOR / "leaflet.js").read_bytes())
        self.assertIn(b"Leaflet 1.9.4", body)

        code, body, ctype = self.route("/vendor/leaflet/leaflet.css")
        self.assertEqual((code, ctype), (200, "text/css; charset=utf-8"))
        self.assertEqual(body, (VENDOR / "leaflet.css").read_bytes())

    def test_vendor_licence_ships_but_is_not_served(self):
        self.assertTrue((VENDOR / "LICENSE").is_file())
        self.assertIsNone(self.route("/vendor/leaflet/LICENSE"))

    def test_vendor_is_a_whitelist_not_a_directory(self):
        for path in ("/vendor/leaflet/", "/vendor/leaflet", "/vendor/",
                     "/vendor/leaflet/../../server.py",
                     "/vendor/leaflet/%2e%2e/%2e%2e/server.py",
                     "/vendor/leaflet/%6Ceaflet.js",
                     "/vendor/leaflet/leaflet.js/", "/vendor/leaflet/LEAFLET.JS",
                     "/vendor/leaflet/images/layers.png"):
            self.assertIsNone(self.route(path), path)

    def test_team_logos_exact_bytes_types_and_magic(self):
        for path, name, ctype, magic in LOGOS:
            with self.subTest(path):
                code, body, got_type = self.route(path)
                self.assertEqual((code, got_type), (200, ctype))
                self.assertEqual(body, (STATIC / name).read_bytes())
                self.assertTrue(body.startswith(magic), body[:8])
                self.assertGreater(len(body), 1000)

    def test_team_logos_are_a_whitelist_not_a_directory(self):
        self.assertEqual({p for p in maproutes._VENDOR if p.startswith("/static")},
                         {path for path, *_ in LOGOS})
        for path in ("/static/", "/static", "/static/ti_logo.jpg/", "/static/TI_LOGO.JPG",
                     "/static/ti_logo.jpeg", "/static/asta_logo.png.bak", "/static/nope.png",
                     "/static/../server.py", "/static/%2e%2e/server.py", "/static/%74i_logo.jpg",
                     "/static//ti_logo.jpg", "/static/asta_logo.png/../ti_logo.jpg"):
            self.assertIsNone(self.route(path), path)

    @unittest.skipUnless(DRONE_LOGOS.is_file(), "the drone GUI's team_logos.py is not checked out beside the OCS")
    def test_team_logos_are_the_drones_images_byte_for_byte(self):
        spec = importlib.util.spec_from_file_location("drone_team_logos", DRONE_LOGOS)
        drone = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(drone)
        self.assertEqual(len(drone.LOGOS), len(LOGOS))
        for (name, uri), (path, fname, ctype, _magic) in zip(drone.LOGOS, LOGOS):
            with self.subTest(name):
                head, _, payload = uri.partition(",")
                self.assertEqual(head, "data:%s;base64" % ctype)
                self.assertEqual((STATIC / fname).read_bytes(), base64.b64decode(payload))

    def test_unrelated_paths_are_not_claimed(self):
        for path in ("/", "/index.html", "/api/state", "/nope", "/mapx"):
            self.assertIsNone(self.route(path), path)

    def test_missing_file_on_disk_is_a_404_not_a_crash(self):
        saved = dict(maproutes._VENDOR)
        self.addCleanup(maproutes._VENDOR.update, saved)
        maproutes._VENDOR["/vendor/leaflet/leaflet.js"] = (
            "vendor/leaflet/not-there.js", "application/javascript")
        code, body, _ = self.route("/vendor/leaflet/leaflet.js")
        self.assertEqual(code, 404)
        self.assertIn(b"not-there.js", body)


class TestFleetProxy(StubTestCase):
    def fetch(self, url, **kw):
        code, body, ctype = maproutes.route(
            "/api/fleet", fleet_url=url,
            map_cfg=maproutes.build_map_config(None, url), **kw)
        self.assertEqual((code, ctype), (200, "application/json"))
        return json.loads(body)

    def test_passes_the_body_through(self):
        self.assertEqual(self.fetch(self.stub().url), FLEET_BODY)

    def test_nothing_listening_is_an_error_body_not_a_failure(self):
        url = _closed_port_url()
        got = self.fetch(url, timeout=0.2)
        self.assertEqual(got["fleet_url"], url)
        self.assertTrue(is_refused(got["error"]), got["error"])
        self.assertNotIn("sysids", got)

    def test_slow_fleet_link_times_out(self):
        got = self.fetch(self.stub(delay=1.5).url, timeout=0.2)
        self.assertIn("connected but no reply", got["error"])
        self.assertNotIn("sysids", got)

    def test_other_failures_are_errors_too(self):
        cases = [
            (dict(status=500, body=b"{}"), "HTTP 500"),
            (dict(body=b"<html>"), "not JSON"),
            (dict(body=b"[1, 2]"), "not a JSON object"),
            (dict(body=b'{"schema": 2}'), "unexpected fleet schema"),
            (dict(body=b'{"sysids": {}}'), "unexpected fleet schema"),
        ]
        for kw, want in cases:
            got = self.fetch(self.stub(**kw).url)
            self.assertIn(want, got["error"], kw)
            self.assertIn("fleet_url", got)
            self.assertNotIn("sysids", got)

    def test_oversize_reply_is_refused(self):
        big = json.dumps({"schema": 1, "pad": "x" * (maproutes.FLEET_MAX_BYTES + 10)})
        got = self.fetch(self.stub(body=big.encode()).url)
        self.assertIn("larger than", got["error"])


class TestMapConfig(unittest.TestCase):
    URL = "http://127.0.0.1:9/fleet"

    def test_defaults(self):
        cfg = maproutes.build_map_config(None, self.URL)
        self.assertEqual(cfg["center"], [32.92357, -117.03857])
        self.assertEqual(cfg["center_source"], "default")
        self.assertEqual(cfg["guis_source"], "default")
        self.assertEqual(cfg["warnings"], [])
        by_name = {g["name"]: g for g in cfg["guis"]}
        self.assertEqual(len(cfg["guis"]), 7)
        self.assertEqual(by_name["Crusader — laptop GUI"],
                         {"name": "Crusader — laptop GUI",
                          "url": "http://localhost:8150", "note": "BOAT_GUI.cmd"})
        self.assertEqual(by_name["Crusader — Jetson"]["url"],
                         "http://192.168.100.109:8090")
        self.assertEqual(by_name["Ekko — laptop GUI"]["url"], "http://localhost:8090")
        self.assertEqual(by_name["Ekko — laptop GUI"]["note"], "DRONE_GUI.cmd")
        self.assertEqual(by_name["Ekko — Jetson"]["url"], "http://ekko.local:8090")
        self.assertEqual(by_name["Graey — Jetson"]["url"], "http://graey.local:8090")
        self.assertEqual(by_name["OCS main page"]["url"], "/")
        # the fleet_link link follows --fleet-url, so it cannot disagree with the proxy
        self.assertEqual(by_name["fleet_link status"]["url"], self.URL)
        self.assertEqual(cfg["fleet_url"], self.URL)

    def test_the_default_url_is_the_default_when_not_overridden(self):
        cfg = maproutes.build_map_config(None, maproutes.DEFAULT_FLEET_URL)
        self.assertEqual({g["name"]: g["url"] for g in cfg["guis"]}["fleet_link status"],
                         "http://127.0.0.1:8098/fleet")

    def test_toml_overrides(self):
        cfg = maproutes.build_map_config({
            "center": [1.2806, 103.8557],
            "gui": [{"name": " Mine ", "url": "http://h:1/x", "note": "n"},
                    {"name": "Local", "url": "/map"},
                    {"name": "evil", "url": "javascript:alert(1)"},
                    {"name": "no url"},
                    {"url": "http://nameless:1"},
                    "not a table"],
        }, self.URL)
        self.assertEqual((cfg["center"], cfg["center_source"]),
                         ([1.2806, 103.8557], "config"))
        self.assertEqual(cfg["guis_source"], "config")
        self.assertEqual(cfg["guis"], [
            {"name": "Mine", "url": "http://h:1/x", "note": "n"},
            {"name": "Local", "url": "/map", "note": ""}])      # replaces, not merges
        self.assertEqual(len(cfg["warnings"]), 1)

    def test_bad_values_fall_back_to_defaults_and_say_so(self):
        for center in ([1], "x", [95, 0], [0, 181], [float("nan"), 0], [True, 1], None):
            cfg = maproutes.build_map_config({"center": center}, self.URL)
            self.assertEqual(cfg["center"], [32.92357, -117.03857], center)
            self.assertTrue(cfg["warnings"], center)
        cfg = maproutes.build_map_config({"gui": [{"name": "x", "url": "ftp://h"}]}, self.URL)
        self.assertEqual(cfg["guis_source"], "default")
        self.assertEqual(len(cfg["guis"]), 7)
        self.assertTrue(cfg["warnings"])

    def test_load_map_table_reads_the_config_toml(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "c.toml"
            p.write_text('team_id = "X"\n[map]\ncenter = [1.5, 103.5]\n'
                         '[[map.gui]]\nname = "A"\nurl = "http://a:1"\nnote = "n"\n',
                         encoding="utf-8")
            cfg = maproutes.build_map_config(maproutes.load_map_table(p), self.URL)
            self.assertEqual(cfg["center"], [1.5, 103.5])
            self.assertEqual(cfg["guis"], [{"name": "A", "url": "http://a:1", "note": "n"}])

            p.write_text('team_id = "X"\n', encoding="utf-8")
            self.assertEqual(maproutes.load_map_table(p), {})
            p.write_text("not = [valid", encoding="utf-8")
            self.assertEqual(maproutes.load_map_table(p), {})
            self.assertEqual(maproutes.load_map_table(Path(d) / "absent.toml"), {})


def _can_import_server() -> bool:
    try:
        from ocs_web import server  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(_can_import_server(), "needs paho and protobuf (the .venv has them)")
class TestRealHandler(StubTestCase):
    """server.make_handler over a real socket: the wiring, not just the logic."""

    def get(self, path, fleet_url=maproutes.DEFAULT_FLEET_URL, map_cfg=None):
        from ocs_web import server
        srv = ThreadingHTTPServer(("127.0.0.1", 0), server.make_handler(
            None, None, [], fleet_url=fleet_url, map_cfg=map_cfg))
        threading.Thread(target=srv.serve_forever, args=(0.01,), daemon=True).start()
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)
        conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
        conn.request("GET", path)         # sent as written: no ../ normalising
        r = conn.getresponse()
        out = (r.status, r.getheader("Content-Type"), r.read())
        conn.close()
        return out

    def test_map_and_vendor(self):
        code, ctype, body = self.get("/map")
        self.assertEqual((code, ctype), (200, "text/html; charset=utf-8"))
        self.assertIn(b"Leaflet", body)
        code, ctype, body = self.get("/vendor/leaflet/leaflet.js")
        self.assertEqual((code, ctype), (200, "application/javascript; charset=utf-8"))
        self.assertEqual(len(body), (VENDOR / "leaflet.js").stat().st_size)
        code, ctype, _ = self.get("/vendor/leaflet/leaflet.css")
        self.assertEqual((code, ctype), (200, "text/css; charset=utf-8"))

    def test_team_logos_are_served_by_the_real_server(self):
        for path, name, ctype, magic in LOGOS:
            with self.subTest(path):
                code, got_type, body = self.get(path)
                self.assertEqual((code, got_type), (200, ctype))
                self.assertEqual(body, (STATIC / name).read_bytes())
                self.assertTrue(body.startswith(magic))
        for path in ("/static/", "/static/nope.png", "/static/../server.py"):
            self.assertEqual(self.get(path)[0], 404, path)

    def test_unknown_vendor_path_is_404(self):
        for path in ("/vendor/leaflet/LICENSE", "/vendor/leaflet/",
                     "/vendor/leaflet/../../server.py", "/vendor/nope.js"):
            self.assertEqual(self.get(path)[0], 404, path)

    def test_fleet_proxy_up_and_down(self):
        code, ctype, body = self.get("/api/fleet", fleet_url=self.stub().url)
        self.assertEqual((code, ctype), (200, "application/json"))
        self.assertEqual(json.loads(body), FLEET_BODY)

        url = _closed_port_url()
        code, _, body = self.get("/api/fleet", fleet_url=url)
        self.assertEqual(code, 200)
        got = json.loads(body)
        self.assertEqual(set(got), {"error", "fleet_url"})
        self.assertEqual(got["fleet_url"], url)
        self.assertTrue(is_refused(got["error"]), got["error"])

    def test_map_config_defaults(self):
        code, ctype, body = self.get("/api/map-config")
        self.assertEqual((code, ctype), (200, "application/json"))
        cfg = json.loads(body)
        self.assertEqual(cfg["center"], [32.92357, -117.03857])
        self.assertEqual(cfg["fleet_url"], "http://127.0.0.1:8098/fleet")
        self.assertEqual(len(cfg["guis"]), 7)

    def test_existing_routes_unchanged(self):
        code, ctype, body = self.get("/")
        self.assertEqual((code, ctype), (200, "text/html"))
        self.assertIn(b'id="fleet"', body)
        self.assertEqual(self.get("/index.html")[0], 200)
        self.assertEqual(self.get("/nope")[0], 404)


class TestPagesCarryTheLogos(unittest.TestCase):
    """Both operator pages put the two logos first in their header, on a white badge, from the whitelist."""

    #: (file, the header's opening tag, the rule that lays the header out)
    PAGES = (("index.html", '<div class="top">', r"\.top \{[^}]*\}"),
             ("map.html", '<div id="top">', r"#top \{[^}]*\}"))

    def page(self, name):
        return (Path(maproutes.HERE) / name).read_text(encoding="utf-8")

    def test_the_logos_are_the_first_thing_in_the_header(self):
        for name, header, _rule in self.PAGES:
            with self.subTest(name):
                text = self.page(name)
                self.assertEqual(text.count(header), 1)
                self.assertEqual(text.count('<div class="logos">'), 1)
                start = text.index(header) + len(header)
                self.assertRegex(text[start:], r'\A\s*<div class="logos">')
                self.assertLess(text.index('<div class="logos">'), text.index("<h1>"))
                logos = text[start:text.index("</div>", start)]
                self.assertEqual(re.findall(r'<img src="([^"]+)"', logos), [path for path, *_ in LOGOS])
                self.assertIn('alt="Team Inspiration" title="Team Inspiration"', logos)
                self.assertIn('alt="Advancing Science, Technology &amp; Art" '
                              'title="Advancing Science, Technology &amp; Art"', logos)

    def test_every_image_the_pages_ask_for_is_a_whitelisted_route(self):
        cfg = maproutes.build_map_config(None, maproutes.DEFAULT_FLEET_URL)
        for name, *_ in self.PAGES:
            for src in re.findall(r'src="(/static/[^"]+)"', self.page(name)):
                with self.subTest(name + " " + src):
                    hit = maproutes.route(src, fleet_url=maproutes.DEFAULT_FLEET_URL, map_cfg=cfg)
                    self.assertIsNotNone(hit)
                    self.assertEqual(hit[0], 200)

    def test_the_badge_is_white_in_both_themes_and_the_header_still_wraps(self):
        for name, _header, rule in self.PAGES:
            with self.subTest(name):
                text = self.page(name)
                self.assertEqual(text.count("--logo-bg:#ffffff"), 2)         # night and day, both white
                self.assertRegex(text, r":root \{[^}]*--logo-bg:#ffffff")
                self.assertRegex(text, r'html\[data-theme="day"\] \{[^}]*--logo-bg:#ffffff')
                badge = re.search(r"\.logos \{[^}]*\}", text).group(0)
                self.assertIn("background:var(--logo-bg)", badge)
                self.assertIn("flex:none", badge)
                self.assertRegex(text, r"\.logos img \{[^}]*height:26px")
                # What keeps 320 px from overflowing: the header wraps its items, the logos do not shrink.
                self.assertIn("flex-wrap:wrap", re.search(rule, text).group(0))


if __name__ == "__main__":
    unittest.main()
