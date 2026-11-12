"""Simulated sensor inputs, real GRAEY reporting/outbox/TCP implementation.

No ROS, serial port, vehicle service or actuator is accessed. RX26_GRAEY points
to the repository root, just as RX26_ASV selects Crusader for bench_stage2.
"""
import os
import math
import queue
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace


class GraeyVehicle:
    vid = 'UUV1'

    def __init__(self, where, port, team, outbox):
        root = Path(os.environ.get('RX26_GRAEY',
                    str(Path(__file__).resolve().parent.parent / 'robotx_graey_2026')))
        if not (root / 'robotx_graey_2026/api/comms/competition_wifi.py').is_file():
            raise RuntimeError('Set RX26_GRAEY to the GRAEY repository root')
        sys.path.insert(0, str(root))
        self.where, self.port, self.team, self.outbox = where, port, team, outbox
        self.received, self.snapshot = [], {}
        self.error = None
        self.commands = queue.Queue()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def command(self, name):
        done = threading.Event()
        self.commands.put((name, done))
        if not done.wait(5) or self.error:
            raise RuntimeError(self.error or 'GRAEY fixture did not respond')

    def _run(self):
        from robotx_graey_2026.api.comms.competition_wifi import CompetitionSender, WifiLink
        from robotx_graey_2026.api.comms.competition_telemetry import CompetitionTelemetry
        from robotx_graey_2026.api.pixhawk.identity import FCU_COMPONENT, FCU_SYSTEM
        sender = link = None
        try:
            sender = CompetitionSender(self.outbox, team_id=self.team)
            link = WifiLink('127.0.0.1', self.port, team_id=self.team)
            telemetry = CompetitionTelemetry(time.monotonic)
            receive = sender.receive

            def capture(message, now):
                self.received.append(message)
                receive(message, now)

            sender.receive = capture
            surface = False
            while not self.stop.is_set():
                while not self.commands.empty():
                    name, done = self.commands.get_nowait()
                    if name == 'survey':
                        sender.survey('e2e-submerged-survey', *self.where, [1, 2, 1])
                    elif name == 'surface':
                        surface = True
                    elif name == 'reconnect':
                        link.drop('intentional isolated e2e disconnect')
                        sender.disconnected()
                    else:
                        raise ValueError(name)
                    done.set()
                now = time.monotonic()
                # Synthetic Cube/bridge/control samples enter the production builder.
                # No mission frame, GPS HAE or local-velocity stream is fabricated.
                source = dict(get_srcSystem=lambda: FCU_SYSTEM,
                              get_srcComponent=lambda: FCU_COMPONENT)
                telemetry.consume('GLOBAL_POSITION_INT', SimpleNamespace(**source,
                    time_boot_ms=int(now * 1000), lat=round(self.where[0] * 1e7),
                    lon=round(self.where[1] * 1e7), alt=0, vx=0, vy=0, vz=0, hdg=9000))
                telemetry.consume('ATTITUDE', SimpleNamespace(**source,
                    roll=0., pitch=0., yaw=math.pi / 2))
                telemetry.put('control', dict(vehicle_fresh=True, sa_active=False,
                    esc_commanded_on=True, armed=True, autonomous=True))
                telemetry.put('bridge', dict(published_monotonic=now,
                    surfaced=surface, depth_m=0.0 if surface else 2.0))
                telemetry.update(sender, now)
                link.tick(sender)
                self.snapshot = dict(sender.status(now), link=link.status(), telemetry_reason=telemetry.reason)
                self.stop.wait(.02)
        except Exception as exc:
            self.error = repr(exc)
        finally:
            if link:
                link.close()
            if sender:
                sender.close()

    def close(self):
        self.stop.set()
        self.thread.join(5)
        if self.thread.is_alive():
            raise RuntimeError('GRAEY fixture did not stop')
