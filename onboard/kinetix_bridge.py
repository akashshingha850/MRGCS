#!/usr/bin/env python3
"""
kinetix_bridge.py — GCS buttons for Kinetix (whole-room photogrammetry).

Runs inside the kinetix:jazzy image with the Kinetix repo at /kinetix (compose
service `kinetix-bridge`), and drives Kinetix only through its CLI, one job at
a time:

  run    python3 -m kinetix run --scene kinetix/scenes/<scene>.yaml --method <m> --rig mavros --seed <n>
  sfm    python3 -m kinetix sfm data/<run>   (reconstruct a flown run: data/<run>/sfm/)
  abort  SIGINT to the running job — for a flight Kinetix's Ctrl+C path: the rig
         flies home at a safe height, lands (AUTO.LAND) and disarms

Kinetix streams the setpoints itself and only one setpoint producer may run per
drone, so `run` is refused while anything else publishes
<DRONE_NS>/mavros/setpoint_position/local (another Kinetix run, the brain node).

Subscribes:  /gcs/kinetix/cmd     std_msgs/String  {"action":"run","scene":"warehouse_room","method":"tour","seed":0,"explore":false}
                                                   explore: true = tour only; Kinetix first explores the room with the ZED depth
                                                   (`--explore`) and plans the tour on what it found, not on the scene file's room
                                                   reuse (explore only, default true): optimized = the exploration views count, the tour
                                                   flies the gaps; false = traditional, the tour scans from scratch (explore.reuse_views=false).
                                                   The run dir is named ..._tour-explore-opt / ..._tour-explore-trad (before 2026-10-30: -optimized_..., -traditional_...)
                                                   {"action":"sfm","run":"<run id>"}   reconstruct (refused while any job runs)
                                                   {"action":"cloud","run":"<run id>"} send that run's reconstruction once on /gcs/kinetix/cloud
                                                   {"action":"delete","runs":["<run id>", ...]} remove those run folders (frames, depth, rosbag,
                                                   reconstruction); never the run of a job that is running
                                                   {"action":"abort"}
Publishes:   /gcs/kinetix/status  std_msgs/String  JSON @ 1 Hz: state, job, log tail, scenes, methods, recent runs
                                                   (run.yaml summary + data/<run>/sfm/metrics.json), tol (Kinetix's capture tolerances), plan (viewpoints +
                                                   flown path of the active or newest run, in the scene's map frame), setpoint producers (count and node
                                                   names), last message
             /gcs/kinetix/cloud   std_msgs/String  JSON, on request: the sparse cloud (points.ply, map frame, cm) and every camera:
                                                   SfM centre (map frame, like the cloud), ground truth and EKF2 centre, its error against
                                                   ground truth after the Sim3 fit metrics.json uses, and its 3D point count

Environment: KINETIX_DIR (/kinetix), DRONE_NS (/drone_1)
"""

import collections
import json
import os
import re
import shutil
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
N_RUNS   = 40           # runs listed (newest first): the Runs tab deletes from this list
N_PATH   = 150          # flown-path points sent to the GCS (a tour has ~400 frames)
N_CLOUD  = 60000        # sparse points sent to the GCS at most (~1.3 MB of JSON)
LABEL    = re.compile(r'x([+-]?[\d.]+)_y([+-]?[\d.]+)_z([+-]?[\d.]+)_h(\d+)')   # Kinetix viewpoint id

try:
    from acquisition.planner.loop import METHODS        # PYTHONPATH=/kinetix in the kinetix image
    METHODS = list(METHODS)
except Exception:
    METHODS = ['tour', 'roi_coverage']


def _tolerances():
    """Capture tolerances from Kinetix's own acquisition/config/common.yaml (one source of truth)."""
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
        cfg = yaml.safe_load((KX / 'acquisition' / 'config' / 'common.yaml').read_text()) or {}
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
        self._cloud_pub = self.create_publisher(String, '/gcs/kinetix/cloud', 1)
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
        self._sizes = {}              # run id -> (dir mtimes, bytes)
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
        elif action == 'sfm':
            self._do_sfm(d)
        elif action == 'delete':
            self._do_delete(d)
        elif action == 'cloud':
            threading.Thread(target=self._send_cloud, args=(str(d.get('run', '')),), daemon=True).start()

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
        reuse = explore and d.get('reuse', True) is not False       # benchmark arm: optimized (reuse the views) / traditional
        if explore and method != 'tour':
            return self._say('refused: exploring the room first works with the tour method only')
        producers = self._producers()
        if producers:
            return self._say(f'refused: other setpoint producer(s) on {SP_TOPIC}: {", ".join(producers)} '
                             '— stop them first (a stuck Kinetix run? the brain node?)')
        self._start('run', ['python3', '-m', 'kinetix', 'run', '--scene', f'kinetix/scenes/{scene}.yaml',
                            '--method', method, '--rig', 'mavros', '--seed', str(seed)]
                    + (['--explore'] + ([] if reuse else ['explore.reuse_views=false']) if explore else []),
                    scene=scene, method=method, seed=seed, explore=explore, reuse=reuse)

    def _do_sfm(self, d):
        run = str(d.get('run', ''))
        if self._busy():
            return self._say(f'refused: a {self._job["kind"]} job is running')
        if run not in {p.name for p in self._run_dirs()}:             # a run id, never a path
            return self._say(f'refused: unknown run "{run}"')
        if not (KX / 'data' / run / 'frames.jsonl').exists():
            return self._say(f'refused: {run} has no frames to reconstruct')
        try:
            scene = (yaml.safe_load((KX / 'data' / run / 'run.yaml').read_text()) or {}).get('scene')
        except Exception:
            scene = None
        self._start('sfm', ['python3', '-m', 'kinetix', 'sfm', f'data/{run}'], run=run, scene=scene)

    def _do_delete(self, d):
        ids = d.get('runs')
        if not isinstance(ids, list) or not ids:
            return self._say('refused: no runs to delete')
        known = {p.name for p in self._run_dirs()}
        active = self._job.get('run') if self._busy() and self._job else None
        bad = [r for r in ids if not isinstance(r, str) or r not in known]               # run ids only, never a path
        if bad:
            return self._say(f'refused: unknown run(s) {", ".join(map(str, bad))}')
        if active in ids:
            return self._say(f'refused: {active} belongs to the running {self._job["kind"]} job')
        threading.Thread(target=self._delete, args=(ids,), daemon=True).start()          # a rosbag is GBs: off the executor

    def _delete(self, ids):
        freed, failed = 0, []
        for r in ids:
            freed += self._size(KX / 'data' / r)
            try:
                shutil.rmtree(KX / 'data' / r)                         # its reconstructions too (data/<run>/sfm/)
            except OSError as e:
                failed.append(f'{r} ({e.strerror})')
        self._runs_t, self._plan_key = 0.0, None                  # refresh the list and the plan view now
        self._say(f'deleted {len(ids) - len(failed)} run(s), {freed / 1e9:.1f} GB freed'
                  + (f'; failed: {", ".join(failed)}' if failed else ''))

    @staticmethod
    def _size(path):
        total = 0
        for root, _, files in os.walk(path):
            for f in files:
                try:
                    total += os.lstat(os.path.join(root, f)).st_size
                except OSError:
                    pass
        return total

    def _do_abort(self):
        if not self._busy():
            return self._say('nothing to abort')
        if self._abort:
            return self._say('already aborting')
        self._abort = True
        # SIGINT = Ctrl+C: Kinetix raises KeyboardInterrupt, its finally lands and disarms (SIGTERM would skip that)
        self._proc.send_signal(signal.SIGINT)
        self._say(f'abort sent to {self._job["kind"]} (SIGINT)' + (': returning home and landing' if self._job['kind'] == 'run' else ''))

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
        return sorted(p.stem for p in (KX / 'kinetix' / 'scenes').glob('*.yaml'))

    def _run_dirs(self):
        d = KX / 'data'
        return sorted((p for p in d.iterdir() if p.is_dir()), reverse=True) if d.is_dir() else []

    def _recent_runs(self):
        if time.time() - self._runs_t < 5.0:
            return self._runs
        out = []
        for p in self._run_dirs()[:N_RUNS]:
            r = {'id': p.name}
            try:
                key = (p.name, p.stat().st_mtime, (p / 'sfm').stat().st_mtime if (p / 'sfm').exists() else 0)
                live = self._busy() and self._job and self._job.get('run') == p.name   # still growing below the top level
                if live or self._sizes.get(p.name, (None,))[0] != key:    # a finished run does not change: walk it once
                    self._sizes[p.name] = (key, self._size(p))
                r['size_gb'] = round(self._sizes[p.name][1] / 1e9, 2)
            except OSError:                                               # deleted meanwhile
                pass
            try:
                y = yaml.safe_load((p / 'run.yaml').read_text()) or {}
                s = y.get('summary') or {}
                ex = (y.get('config') or {}).get('explore') or {}   # no reuse_views key = flown before it existed = traditional
                arm = (' · explore ' + ('optimized' if ex.get('reuse_views', False) else 'traditional')) if ex.get('enabled') else ''
                r.update(scene=y.get('scene'), method=(y.get('method') or '') + arm, frames=s.get('frames'),
                         stop=s.get('stop_reason'), coverage=s.get('coverage'), wall_s=s.get('wall_s'),
                         travel_m=s.get('travel_m'))
            except Exception:
                pass
            try:
                m = json.loads((p / 'sfm' / 'metrics.json').read_text())
                gt, cp = m.get('ate_vs_gt') or {}, m.get('completeness_proxy') or {}
                r['sfm'] = {'registered': m.get('registered'), 'frames': m.get('n_frames'), 'points': m.get('points3D'),
                            'reproj_px': m.get('mean_reproj_error_px'), 'ate_median_m': gt.get('median_m'),
                            'ate_rmse_m': gt.get('rmse_m'), 'ate_max_m': gt.get('max_m'), 'ekf2_vs_gt_m': m.get('ekf2_vs_gt_m'),
                            'completeness': cp.get('all'), 'walls': cp.get('walls')}
            except (OSError, ValueError):
                pass
            out.append(r)
        self._runs, self._runs_t = out, time.time()
        return out

    # ── plan + flown path of one run ──────────────────────────────────────────
    def _scene_box(self, scene, run_dir=None):
        """Room box, workspace and obstacle boxes: the room Kinetix explored in this run if it did, else the scene file's."""
        try:
            fitted = run_dir / 'explore' / 'scene.yaml' if run_dir else None
            y = yaml.safe_load((fitted if fitted and fitted.exists() else KX / 'kinetix' / 'scenes' / f'{scene}.yaml').read_text()) or {}
        except Exception:
            return {}
        out = {k: y[k] for k in ('roi', 'workspace') if isinstance(y.get(k), dict)}
        out['no_fly'] = [b for b in (y.get('no_fly') or []) if isinstance(b, dict)]
        out['explored'] = bool(fitted and fitted.exists())
        if out['explored']:                                       # the scene file's room too: the GCS draws how far off the fit is
            try:
                out['truth_roi'] = (yaml.safe_load((KX / 'kinetix' / 'scenes' / f'{scene}.yaml').read_text()) or {}).get('roi')
            except Exception:
                pass
        return out

    def _plan(self, run_id, scene):
        """Viewpoints Kinetix planned (steps.jsonl) and the path it flew (frames.jsonl), both in the scene's map frame.
        tour plans every viewpoint up front; roi_coverage picks the next best view per step, so its list grows in flight."""
        d = KX / 'data' / run_id
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

    # ── reconstruction for the 3D view ─────────────────────────────────────────
    def _send_cloud(self, run):
        try:
            self._cloud_pub.publish(String(data=json.dumps(self._cloud(run))))
        except Exception as e:                                    # a missing / half-written reconstruction
            self._cloud_pub.publish(String(data=json.dumps({'run': run, 'error': str(e)})))

    def _cloud(self, run):
        import numpy as np
        import pycolmap                                           # the kinetix image has it (kinetix sfm)
        from kinetix.align import apply, umeyama
        if run not in {p.name for p in self._run_dirs()}:
            raise ValueError(f'unknown run "{run}"')
        d = KX / 'data' / run
        off = d / 'sfm'
        if not (off / 'points.ply').exists():
            raise ValueError('not reconstructed yet')
        with open(off / 'points.ply') as f:                      # ascii: x y z r g b, map frame (kinetix sfm aligns it to EKF2)
            n_head = next(i for i, line in enumerate(f, 1) if line.strip() == 'end_header')
        P = np.loadtxt(off / 'points.ply', skiprows=n_head, ndmin=2)
        if len(P) > N_CLOUD:
            P = P[np.random.default_rng(0).choice(len(P), N_CLOUD, replace=False)]
        frames = {f['image']: f for f in self._jsonl(d / 'frames.jsonl')}
        rec = max((pycolmap.Reconstruction(str(m)) for m in (off / 'sparse').iterdir()), key=lambda r: r.num_reg_images())
        rows = [(frames[i.name], np.asarray(i.projection_center()), i.num_points3D) for i in rec.images.values()
                if i.has_pose and i.name in frames]
        S = np.array([r[1] for r in rows])
        E = np.array([r[0]['pose']['p'] for r in rows])
        cams = {'idx': [r[0]['idx'] for r in rows], 'points': [int(r[2]) for r in rows]}
        if len(rows) >= 3:
            s, R, t = umeyama(S, E)
            cams['sfm'] = np.round(apply(s, R, t, S), 2).tolist()          # in the cloud's frame
            cams['ekf'] = np.round(E, 2).tolist()
            if all(r[0].get('gt_pose') for r in rows):
                G = np.array([r[0]['gt_pose']['p'] for r in rows])
                s, R, t = umeyama(S, G)                                    # as metrics.json ate_vs_gt
                cams['err_m'] = np.round(np.linalg.norm(apply(s, R, t, S) - G, axis=1), 3).tolist()
                cams['ekf_err_m'] = np.round(np.linalg.norm(E - G, axis=1), 3).tolist()
                cams['gt'] = np.round(G, 2).tolist()
        rgb = (P[:, 3].astype(int) << 16) | (P[:, 4].astype(int) << 8) | P[:, 5].astype(int)
        return {'run': run, 'n_total': int(rec.num_points3D()), 'xyz_cm': np.round(P[:, :3] * 100).astype(int).ravel().tolist(),
                'rgb': rgb.tolist(), 'cams': cams}

    # ── status (1 Hz) ─────────────────────────────────────────────────────────
    def _publish(self):
        busy = self._busy()
        if not busy:
            state = 'idle'
        else:
            state = 'aborting' if self._abort else ('reconstructing' if self._job['kind'] == 'sfm' else 'flying')
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
