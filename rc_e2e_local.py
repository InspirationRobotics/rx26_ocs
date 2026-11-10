"""Run the unchanged RoboNation test server/client locally for rc_e2e.

Separate loopback brokers, ports and temporary logs; never connects to a vehicle.
"""
import json
import os
import subprocess
import sys
import urllib.request
from pathlib import Path


class LocalBox:
    def __init__(self, root, tmp, broker_port, watch_port):
        self.processes = []
        self.files = []
        self.root, self.tmp = Path(root).resolve(), tmp
        self.watch = 'http://127.0.0.1:%d' % watch_port
        self.env = dict(os.environ, PYTHONUTF8='1', PYTHONIOENCODING='utf-8',
                        BROKER_HOST='127.0.0.1', BROKER_PORT=str(broker_port),
                        LOG_DIR=str(tmp / 'server-logs'))
        self.env['PYTHONPATH'] = str(self.root / 'gen/python') + os.pathsep + os.environ.get('PYTHONPATH', '')
        self.broker_port, self.watch_port = broker_port, watch_port

    def launch(self, name, command, stdin=None):
        log = (self.tmp / (name + '.log')).open('w', encoding='utf-8')
        self.files.append(log)
        proc = subprocess.Popen(command, cwd=Path(__file__).resolve().parent,
                                env=self.env, stdin=stdin, stdout=log,
                                stderr=subprocess.STDOUT, text=True, encoding='utf-8')
        self.processes.append(proc)
        return proc

    def start(self):
        # The same amqtt broker implementation used by Crusader's local benches.
        self.launch('broker', [sys.executable, '-u', '-c',
                    'import bench_stage1 as b; import time; b.PORT=%d; b.start_broker(); time.sleep(86400)' % self.broker_port])
        import socket
        import time
        for _ in range(100):
            try:
                with socket.create_connection(('127.0.0.1', self.broker_port), .2):
                    break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError('isolated broker did not start')
        self.launch('watcher', [sys.executable, '-u', 'rc_watch.py', '--broker', '127.0.0.1',
                    '--port', str(self.broker_port), '--http', str(self.watch_port),
                    '--logs', self.env['LOG_DIR'], '--record', str(self.tmp / 'watch')])
        self.launch('official-server', [sys.executable, '-u', str(self.root / 'rc_test/test_server.py')])
        self.client = self.launch('official-client', [sys.executable, '-u',
                    str(self.root / 'rc_test/test_client.py')], stdin=subprocess.PIPE)

    def send(self, line):
        self.client.stdin.write(line + '\n')
        self.client.stdin.flush()

    def api(self, path, body):
        req = urllib.request.Request(self.watch + path, data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.load(resp)

    def close(self):
        for proc in reversed(self.processes):
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(5)
        for log in self.files:
            log.close()
