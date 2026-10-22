#!/usr/bin/env python3
"""
kinetix_bridge.py — GCS buttons for Kinetix (whole-room photogrammetry).

Runs inside the kinetix:jazzy image with the Kinetix repo at /kinetix (compose
service `kinetix-bridge`), and drives Kinetix only through its CLI, one job at
a time:

  run    python3 -m kinetix run --scene config/scenes/<scene>.yaml --method <m> --rig mavros --seed <n>
  sfm    python3 -m kinetix sfm runs/<run>
  abort  SIGINT to the running job — Kinetix's Ctrl+C path: the rig flies home
         at a safe height, lands (AUTO.LAND) and disarms

Kinetix streams the setpoints itself and only one setpoint producer may run per
drone, so `run` is refused while anything else publishes
<DRONE_NS>/mavros/setpoint_position/local (another Kinetix run, the brain node).

Subscribes:  /gcs/kinetix/cmd     std_msgs/String  {"action":"run","scene":"warehouse_room","method":"tour","seed":0}
                                                   {"action":"sfm","run":"<run id>"}   (no run = newest)
                                                   {"action":"abort"}
Publishes:   /gcs/kinetix/status  std_msgs/String  JSON @ 1 Hz: state, job, log tail, scenes, methods, recent runs,
                                                   setpoint producers, last message

Environment: KINETIX_DIR (/kinetix), DRONE_NS (/drone_1)
"""

import collections
import json
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

KX       = Path(os.environ.get('KINETIX_DIR', '/kinetix'))
DRONE_NS = os.environ.get('DRONE_NS', '/drone_1')
SP_TOPIC = f'{DRONE_NS}/mavros/setpoint_position/local'
N_RUNS   = 8

try:
    from kinetix.planner.loop import METHODS            # PYTHONPATH=/kinetix/src in the kinetix image
    METHODS = list(METHODS)
except Exception:
    METHODS = ['tour', 'roi_coverage']


class KinetixBridge(Node):

    def __init__(self):
        super().__init__('kinetix_bridge')
        self._pub   = self.create_publisher(String, '/gcs/kinetix/status', 10)
        self.create_subscription(String, '/gcs/kinetix/cmd', self._on_cmd, 10)

        self._lock  = threading.Lock()
        self._proc  = None            # running job (subprocess.Popen)
        self._job   = None            # {kind, scene, method, seed, run, started}
        self._log   = collections.deque(maxlen=14)
        self._msg   = ''              # last refusal / result, shown in the panel
        self._abort = False
        self._runs  = []
        self._runs_t = 0.0

        self.create_timer(1.0, self._publish)
        self.get_logger().info(f'kinetix_bridge ready — {KX}, setpoint topic {SP_TOPIC}, methods {METHODS}')

    # ── commands ──────────────────────────────────────────────────────────────
    def _on_cmd(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        action = d.get('action')
        if action == 'abort':
            self._do_abort()
        elif action == 'run':
            self._do_run(d)
        elif action == 'sfm':
            self._do_sfm(d)

    def _busy(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _do_run(self, d):
        scene, method = str(d.get('scene', '')), str(d.get('method', ''))
        try:
            seed = int(d.get('seed', 0))
        except (TypeError, ValueError):
            seed = 0
        if self._busy():
            return self._say(f'refused: a {self._job["kind"]} job is running')
        if scene not in self._scenes():
            return self._say(f'refused: unknown scene "{scene}"')
        if method not in METHODS:
            return self._say(f'refused: unknown method "{method}"')
        producers = self.count_publishers(SP_TOPIC)
        if producers:
            return self._say(f'refused: {producers} other setpoint producer(s) on {SP_TOPIC} '
                             '(a stuck Kinetix run? the brain node?) — stop it first')
        self._start('run', ['python3', '-m', 'kinetix', 'run', '--scene', f'config/scenes/{scene}.yaml',
                            '--method', method, '--rig', 'mavros', '--seed', str(seed)],
                    scene=scene, method=method, seed=seed)

    def _do_sfm(self, d):
        if self._busy():
            return self._say(f'refused: a {self._job["kind"]} job is running')
        runs = self._run_dirs()
        run = str(d.get('run') or (runs[0].name if runs else ''))
        if run not in {p.name for p in runs}:
            return self._say(f'refused: no run "{run}"')
        self._start('sfm', ['python3', '-m', 'kinetix', 'sfm', f'runs/{run}'], run=run)

    def _do_abort(self):
        if not self._busy():
            return self._say('nothing to abort')
        if self._abort:
            return self._say('already aborting — Kinetix is flying home and landing')
        self._abort = True
        # SIGINT = Ctrl+C: Kinetix raises KeyboardInterrupt, its finally lands and disarms (SIGTERM would skip that)
        self._proc.send_signal(signal.SIGINT)
        self._say(f'abort sent to {self._job["kind"]} (SIGINT): returning home and landing')

    def _start(self, kind, argv, **job):
        self._log.clear()
        self._abort = False
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        self._proc = subprocess.Popen(argv, cwd=KX, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True, bufsize=1)
        self._job = dict(kind=kind, started=time.time(), **job)
        threading.Thread(target=self._read, args=(self._proc,), daemon=True).start()
        self._say(f'started {kind}: {" ".join(argv[3:])}')

    def _read(self, proc):
        for line in proc.stdout:
            line = line.rstrip()
            with self._lock:
                self._log.append(line)
                m = re.search(r'\brun (\S+): scene ', line)       # RunLog's first line names the run
                if m and self._job and self._job['kind'] == 'run':
                    self._job['run'] = m.group(1)
        code = proc.wait()
        self._runs_t = 0.0                                        # refresh the run list now
        kind = self._job['kind'] if self._job else 'job'
        self._say(f'{kind} finished — exit {code}' + (' (aborted)' if self._abort else ''))

    def _say(self, text):
        self._msg = text
        self.get_logger().info(text)

    # ── runs / scenes ─────────────────────────────────────────────────────────
    def _scenes(self):
        return sorted(p.stem for p in (KX / 'config' / 'scenes').glob('*.yaml'))

    def _run_dirs(self):
        d = KX / 'runs'
        return sorted((p for p in d.iterdir() if p.is_dir()), reverse=True) if d.is_dir() else []

    def _recent_runs(self):
        if time.time() - self._runs_t < 5.0:
            return self._runs
        out = []
        for p in self._run_dirs()[:N_RUNS]:
            r = {'id': p.name}
            try:
                y = yaml.safe_load((p / 'run.yaml').read_text()) or {}
                s = y.get('summary') or {}
                r.update(scene=y.get('scene'), method=y.get('method'), frames=s.get('frames'),
                         stop=s.get('stop_reason'), coverage=s.get('coverage'), wall_s=s.get('wall_s'))
            except Exception:
                pass
            try:
                m = json.loads((p / 'sfm' / 'offline' / 'metrics.json').read_text())
                ate = m.get('ate_vs_gt') or m.get('ate_vs_ekf2') or {}
                r['sfm'] = {'registered': m.get('registered'), 'n_frames': m.get('n_frames'),
                            'points3D': m.get('points3D'), 'reproj_px': m.get('mean_reproj_error_px'),
                            'ate_m': ate.get('rmse_m'),
                            'completeness': (m.get('completeness_proxy') or {}).get('all')}
            except Exception:
                pass
            out.append(r)
        self._runs, self._runs_t = out, time.time()
        return out

    # ── status (1 Hz) ─────────────────────────────────────────────────────────
    def _publish(self):
        busy = self._busy()
        if not busy:
            state = 'idle'
        elif self._job['kind'] == 'sfm':
            state = 'reconstructing'
        else:
            state = 'aborting' if self._abort else 'flying'
        with self._lock:
            log = list(self._log)
            job = dict(self._job) if self._job else None
        producers = self.count_publishers(SP_TOPIC)
        self._pub.publish(String(data=json.dumps({
            'state': state,
            'job': job,
            'elapsed_s': round(time.time() - job['started']) if job and busy else None,
            'log': log,
            'msg': self._msg,
            'scenes': self._scenes(),
            'methods': METHODS,
            'runs': self._recent_runs(),
            # our own run is one producer; anything beyond that is someone else
            'other_setpoint_producers': max(0, producers - (1 if busy and job['kind'] == 'run' else 0)),
        })))

    def destroy_node(self):
        if self._busy():
            self._proc.send_signal(signal.SIGINT)              # land before the bridge goes away
            try:
                self._proc.wait(timeout=180)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        super().destroy_node()


def main():
    rclpy.init()
    node = KinetixBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()   # SIGTERM may have shut the context down already


if __name__ == '__main__':
    main()
