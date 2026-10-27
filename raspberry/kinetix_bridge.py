#!/usr/bin/env python3
"""
kinetix_bridge.py — GCS buttons for Kinetix (whole-room photogrammetry).

Runs inside the kinetix:jazzy image with the Kinetix repo at /kinetix (compose
service `kinetix-bridge`), and drives Kinetix only through its CLI, one job at
a time:

  run    python3 -m kinetix run --scene config/scenes/<scene>.yaml --method <m> --rig mavros --seed <n>
  abort  SIGINT to the running job — Kinetix's Ctrl+C path: the rig flies home
         at a safe height, lands (AUTO.LAND) and disarms

Kinetix streams the setpoints itself and only one setpoint producer may run per
drone, so `run` is refused while anything else publishes
<DRONE_NS>/mavros/setpoint_position/local (another Kinetix run, the brain node).

Subscribes:  /gcs/kinetix/cmd     std_msgs/String  {"action":"run","scene":"warehouse_room","method":"tour","seed":0,"explore":false}
                                                   explore: true = tour only; Kinetix first explores the room with the ZED depth
                                                   (`--explore`) and plans the tour on what it found, not on the scene file's room
                                                   {"action":"abort"}
Publishes:   /gcs/kinetix/status  std_msgs/String  JSON @ 1 Hz: state, job, log tail, scenes, methods, recent runs
                                                   (run.yaml summary), tol (Kinetix's capture tolerances), plan (viewpoints + flown path of the active or
                                                   newest run, in the scene's map frame), setpoint producers (count and node names), last message

Planning and results only: the GCS does no post-processing (no SfM / reconstruction).

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
N_PATH   = 150          # flown-path points sent to the GCS (a tour has ~400 frames)
LABEL    = re.compile(r'x([+-]?[\d.]+)_y([+-]?[\d.]+)_z([+-]?[\d.]+)_h(\d+)')   # Kinetix viewpoint id

try:
    from kinetix.planner.loop import METHODS            # PYTHONPATH=/kinetix/src in the kinetix image
    METHODS = list(METHODS)
except Exception:
    METHODS = ['tour', 'roi_coverage']


def _tolerances():
    """Capture tolerances from Kinetix's own config/default.yaml (one source of truth)."""
    keys = {'pos_m': 'tol_pos_m', 'yaw_deg': 'tol_yaw_deg', 'speed_mps': 'tol_speed_mps', 'settle_s': 'settle_s'}
    def find(node, key):
        if isinstance(node, dict):
            if key in node:
                return node[key]
            for v in node.values():
                r = find(v, key)
                if r is not None:
                    return r
        return None
    try:
        cfg = yaml.safe_load((KX / 'config' / 'default.yaml').read_text()) or {}
    except Exception:
        cfg = {}
    return {k: find(cfg, v) for k, v in keys.items()}


def _snap(pose):
    """Camera snap as [x, y, z, fx, fy, fz]: position + viewing direction (optical z in map; pose q is T_map_cam, xyzw)."""
    (x, y, z), (qx, qy, qz, qw) = pose['p'], pose['q']
    return [x, y, z, 2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy)]


def _viewpoint(label):
    m = LABEL.fullmatch(str(label))
    return [float(m[1]), float(m[2]), float(m[3]), int(m[4])] if m else None     # x, y, z (map), heading deg


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
        self._plan_key, self._plan_cache = None, None
        self._tol = _tolerances()

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

    def _producers(self):
        """Nodes publishing setpoints to the drone right now, e.g. ['/drone_1/brain_node']."""
        return sorted(f"{i.node_namespace.rstrip('/')}/{i.node_name}"
                      for i in self.get_publishers_info_by_topic(SP_TOPIC))

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
        explore = bool(d.get('explore'))
        if explore and method != 'tour':
            return self._say('refused: exploring the room first works with the tour method only')
        producers = self._producers()
        if producers:
            return self._say(f'refused: other setpoint producer(s) on {SP_TOPIC}: {", ".join(producers)} '
                             '— stop them first (a stuck Kinetix run? the brain node?)')
        self._start('run', ['python3', '-m', 'kinetix', 'run', '--scene', f'config/scenes/{scene}.yaml',
                            '--method', method, '--rig', 'mavros', '--seed', str(seed)] + (['--explore'] if explore else []),
                    scene=scene, method=method, seed=seed, explore=explore)

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
                         stop=s.get('stop_reason'), coverage=s.get('coverage'), wall_s=s.get('wall_s'),
                         travel_m=s.get('travel_m'))
            except Exception:
                pass
            out.append(r)
        self._runs, self._runs_t = out, time.time()
        return out

    # ── plan + flown path of one run ──────────────────────────────────────────
    def _scene_box(self, scene, run_dir=None):
        """Room box, workspace and obstacle boxes: the room Kinetix explored in this run if it did, else the scene file's."""
        try:
            fitted = run_dir / 'explore' / 'scene.yaml' if run_dir else None
            y = yaml.safe_load((fitted if fitted and fitted.exists() else KX / 'config' / 'scenes' / f'{scene}.yaml').read_text()) or {}
        except Exception:
            return {}
        out = {k: y[k] for k in ('roi', 'workspace') if isinstance(y.get(k), dict)}
        out['no_fly'] = [b for b in (y.get('no_fly') or []) if isinstance(b, dict)]
        out['explored'] = bool(fitted and fitted.exists())
        return out

    def _plan(self, run_id, scene):
        """Viewpoints Kinetix planned (steps.jsonl) and the path it flew (frames.jsonl), both in the scene's map frame.
        tour plans every viewpoint up front; roi_coverage picks the next best view per step, so its list grows in flight."""
        d = KX / 'runs' / run_id
        mt = lambda f: f.stat().st_mtime if f.exists() else 0
        key = (run_id, mt(d / 'steps.jsonl'), mt(d / 'frames.jsonl'), mt(d / 'explore' / 'steps.jsonl'), mt(d / 'explore' / 'scene.yaml'))
        if key == self._plan_key:
            return self._plan_cache
        steps = self._jsonl(d / 'steps.jsonl')
        if steps and 'waypoints' in steps[0]:
            labels = [w[0] if isinstance(w, list) and w else w for w in steps[0]['waypoints']]
            info = {k: steps[0].get(k) for k in ('plan', 'viewpoints', 'plan_coverage', 'tour_m', 'est_flight_s')}
        else:
            labels = [s.get('chosen') for s in steps if s.get('chosen')]
            info = {'plan': 'greedy', 'viewpoints': len(labels), 'plan_coverage': steps[-1].get('coverage') if steps else None}
        views = [v for v in map(_viewpoint, labels) if v]
        poses = [_snap(f['pose']) for f in self._jsonl(d / 'frames.jsonl') if f.get('pose')]
        stride = max(1, len(poses) // N_PATH)
        path = [[round(c, 2) for c in p] for p in poses[::stride]]
        if poses and (len(poses) - 1) % stride:
            path.append([round(c, 2) for c in poses[-1]])
        ex = self._jsonl(d / 'explore' / 'steps.jsonl')                   # exploration progress (explore runs only)
        if ex:
            faces = (ex[-1].get('faces') or {})
            info['explore'] = {'views': ex[-1].get('frames'), 'faces_min': round(min(faces.values()), 2) if faces else None,
                               'known': ex[-1].get('known'), 'obstacles': ex[-1].get('obstacles'),
                               'done': (d / 'explore' / 'report.json').exists()}
        self._plan_key = key
        self._plan_cache = dict(run=run_id, scene=scene, info=info, views=views, path=path, frames=len(poses),
                                **self._scene_box(scene, d))
        return self._plan_cache

    @staticmethod
    def _jsonl(path):
        out = []
        try:
            for line in path.read_text().splitlines():
                try:
                    out.append(json.loads(line))
                except ValueError:                               # last line may be mid-write
                    pass
        except OSError:
            pass
        return out

    # ── status (1 Hz) ─────────────────────────────────────────────────────────
    def _publish(self):
        busy = self._busy()
        if not busy:
            state = 'idle'
        else:
            state = 'aborting' if self._abort else 'flying'
        with self._lock:
            log = list(self._log)
            job = dict(self._job) if self._job else None
        runs = self._recent_runs()
        shown = job['run'] if busy and job.get('run') else (runs[0]['id'] if runs else None)
        scene = job['scene'] if busy else (runs[0].get('scene') if runs else None)
        producers = self._producers()
        self._pub.publish(String(data=json.dumps({
            'state': state,
            'job': job,
            'elapsed_s': round(time.time() - job['started']) if job and busy else None,
            'log': log,
            'msg': self._msg,
            'scenes': self._scenes(),
            'methods': METHODS,
            'runs': runs,
            'plan': self._plan(shown, scene) if shown else None,
            # our own run is one producer; anything beyond that is someone else
            'other_setpoint_producers': max(0, len(producers) - (1 if busy and job['kind'] == 'run' else 0)),
            'setpoint_producers': producers,
            'tol': self._tol,
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
