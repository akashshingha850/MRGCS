#!/usr/bin/env python3
"""
sensor_check.py — the GCS Sensors tab: which source EKF2 should trust for heading (magnetometer or ZED), height (baro, LW20 range,
ZED) and horizontal velocity (optical flow, ZED) on this drone in this room, plus the IMU (noise, vibration, clipping), the ZED's
stamp latency, the EV lever arm (sim) and how well the drone holds a Kinetix capture. Calibrations are QGC's job, not this one's.

Runs next to gcs_control in the drone's namespace (relative MAVROS names; rosbridge_boot.sh starts it).

Live:     every source's rate and latest value, in the status (2 Hz).
analyse   the automated flight, OFFBOARD all the way (no GPS: AUTO modes refuse or fly off): GROUND_S on the ground (motors off),
          arm, climb ALT_M, hover HOVER_S, 8 yaw steps of 45 deg one way and 8 back at TURN_DPS, a RISE_M height step up and down,
          a SQUARE_M square at MOVE_MPS (forward first, in the start heading), HOLD_S after every step, land on the start point,
          disarm, GROUND_S on the ground again. Needs ~SQUARE_M + 1 m of clear space ahead and to the left. Abort lands from where
          it is; a pilot mode change ends it and leaves the drone to the pilot.
record    by hand, motors off: carry / turn / lift the drone, then Stop to analyse (no hover, vibration or capture numbers).
The analysis is sensor_analysis.py; each run is saved to SENSORS_DIR (<run>.npz + <run>.json) and re-analysed offline with
`python3 sensor_analysis.py <run>.npz`.

At the start of a run it asks PX4 for VIBRATION and ESTIMATOR_STATUS at 10 Hz (MAV_CMD_SET_MESSAGE_INTERVAL; they stay at that rate
until the FCU reboots). mavros/vibration needs MAVROS' vibration plugin (bisg_isaac: MAVROS_PLUGINS=lean+debug). EKF2's test
ratios are decoded from the raw ESTIMATOR_STATUS on MAVROS' router (/uas1/mavlink_source): mavros/estimator_status has only flags.

Subscribes:  /gcs/sensors/cmd      std_msgs/String  {"action":"analyse"} {"action":"record"} {"action":"stop"} (= abort / end record)
             /gcs/kinetix/status   std_msgs/String  Kinetix's capture tolerances (kinetix_bridge.py `tol`)
Publishes:   /gcs/sensors/status   std_msgs/String  JSON @ 2 Hz: state, msg, step, live, params, result, tables
"""
import json
import math
import os
import struct
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, Mavlink, OpticalFlowRad, PositionTarget, State, StatusText
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, FluidPressure, Imu, MagneticField, Range
from std_msgs.msg import String

import sensor_analysis as sa

try:
    from mavros_msgs.msg import Vibration
except ImportError:                                    # older mavros_msgs: the IMU section goes without PX4's vibration metrics
    Vibration = None

ALT_M, HOVER_S, HOLD_S, GROUND_S = 1.5, 8.0, 4.0, 5.0
TURN_DPS, STEP_DEG, RISE_M = 20.0, 45, 0.6
SQUARE_M, MOVE_MPS, CLIMB_MPS = 1.0, 0.3, 0.5
MAX_REC_S = 900.0                                      # a record stops itself after this long
PARAMS = ['EKF2_MAG_TYPE', 'EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'EKF2_OF_CTRL', 'EKF2_BARO_CTRL', 'EKF2_GPS_CTRL',
          'EKF2_EV_DELAY', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z', 'SENS_FLOW_ROT']
MSG_RATES = {241: 10.0, 230: 10.0}                     # VIBRATION, ESTIMATOR_STATUS (Hz)
MAVLINK_SOURCE = '/uas1/mavlink_source'                # MAVROS' router output (absolute: MAVROS' default uas id)
RUN_DIR = os.environ.get('SENSORS_DIR', '/data/sensor_runs')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SensorCheck(Node):
    def __init__(self):
        super().__init__('sensor_check')
        self.state, self.msg, self.result, self.tables = 'idle', '', None, None
        self.pose = self.mavstate = self.landed = None
        self.pose_t = 0.0
        self.rec, self.phases, self.recording, self.rec_t0 = None, [], False, 0.0
        self.last, self.counts, self.clock = {}, {k: deque(maxlen=400) for k in sa.STREAMS}, {}
        self.params, self.tol, self.cam, self.cam_n = {}, None, {}, deque(maxlen=60)
        self.zstat, self.zstart, self.zseries, self.px4_text = None, None, [], ''

        self.fl = None                                     # the analyse flight: step list, index, setpoint
        sub = lambda t, n, cb, q=qos_profile_sensor_data: self.create_subscription(t, n, cb, q)
        sub(PoseStamped, 'mavros/local_position/pose', self._on_pose)
        sub(TwistStamped, 'mavros/local_position/velocity_body',
            lambda m: self._put('ekf_vb', m.header, m.twist.linear.x, m.twist.linear.y, m.twist.linear.z))
        sub(Imu, 'mavros/imu/data', lambda m: self._put('att', m.header, m.orientation.x, m.orientation.y, m.orientation.z,
                                                        m.orientation.w, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        sub(Imu, 'mavros/imu/data_raw', lambda m: self._put('imu_raw', m.header, m.linear_acceleration.x, m.linear_acceleration.y,
                                                            m.linear_acceleration.z, m.angular_velocity.x, m.angular_velocity.y,
                                                            m.angular_velocity.z))
        sub(MagneticField, 'mavros/imu/mag', lambda m: self._put('mag', m.header, m.magnetic_field.x, m.magnetic_field.y,
                                                                 m.magnetic_field.z))
        sub(FluidPressure, 'mavros/imu/static_pressure', lambda m: self._put('baro', m.header, m.fluid_pressure))
        # the LW20 (DISTANCE_SENSOR); px4flow/ground_distance carries PX4Flow's 0.3..5 m limits, so on the ground it is always invalid
        sub(Range, 'mavros/hrlv_ez4_pub',
            lambda m: self._put('range', m.header, m.range if m.min_range <= m.range <= m.max_range else float('nan')))
        sub(OpticalFlowRad, 'mavros/px4flow/raw/optical_flow_rad',
            lambda m: self._put('flow', m.header, m.integrated_x, m.integrated_y, m.integrated_xgyro, m.integrated_ygyro,
                                m.integration_time_us * 1e-6, m.quality))
        sub(Odometry, 'zed/zed_node/odom', lambda m: self._put_pose('zed', m.header, m.pose.pose))
        sub(Odometry, 'mavros/odometry/out', lambda m: self._put_pose('vision', m.header, m.pose.pose))
        sub(PoseStamped, 'state/pose', lambda m: self._put_pose('truth', m.header, m.pose))
        sub(Imu, 'zed/zed_node/imu/data', lambda m: self._put('zimu', m.header, m.angular_velocity.x, m.angular_velocity.y,
                                                              m.angular_velocity.z, m.linear_acceleration.x,
                                                              m.linear_acceleration.y, m.linear_acceleration.z))
        if Vibration is not None:
            sub(Vibration, 'mavros/vibration/raw/vibration',          # metrics are magnitudes: MAVROS rotates them FRD -> FLU anyway
                lambda m: self._put('vib', m.header, abs(m.vibration.x), abs(m.vibration.y), abs(m.vibration.z), *m.clipping))
        sub(Mavlink, MAVLINK_SOURCE, self._on_mavlink)
        sub(CameraInfo, 'zed/zed_node/left/color/rect/camera_info', self._on_caminfo)
        sub(String, 'zed_stack/status', self._on_zstat)
        sub(State, 'mavros/state', lambda m: setattr(self, 'mavstate', m), 10)
        sub(StatusText, 'mavros/statustext/recv', lambda m: setattr(self, 'px4_text', m.text) if m.severity <= StatusText.WARNING else None)
        sub(ExtendedState, 'mavros/extended_state', lambda m: setattr(self, 'landed', m.landed_state), 10)
        self.create_subscription(String, '/gcs/sensors/cmd', self._on_cmd, 10)
        self.create_subscription(String, '/gcs/kinetix/status', self._on_kinetix, 1)
        self.pub = self.create_publisher(String, '/gcs/sensors/status', 1)
        self.sp_pub = self.raw_pub = None                  # only during the flight: Kinetix refuses to start while any node publishes setpoints
        self.cli_cmd = self.create_client(CommandLong, 'mavros/cmd/command')
        self.cli_arm = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, 'mavros/set_mode')
        self.param_cli = AsyncParameterClient(self, 'mavros/param')
        self.create_timer(0.05, self._fly)
        self.create_timer(0.5, self._publish)
        self.create_timer(10.0, self._read_params)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── inputs ────────────────────────────────────────────────────────────────
    def _put(self, key, header, *v):
        """Store a sample on its header stamp, or on its arrival time when the stream stamps on another clock (sim: the ZED odom and
        Pegasus' state/pose stamp wall time, MAVROS and the bridge sim time). Decided once per stream, on its first message."""
        now = self.now()
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        if key not in self.clock:
            self.clock[key] = 'stamp' if abs(t - now) < 2.0 else 'arrival'
        if self.clock[key] == 'arrival':
            t = now
        self.last[key] = (t, v)
        self.counts[key].append(now)
        if self.recording:
            self.rec[key].append((t,) + tuple(float(x) for x in v))

    def _put_pose(self, key, header, p):
        self._put(key, header, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))

    def _on_pose(self, m):
        p = m.pose.position
        self.pose, self.pose_t = (p.x, p.y, p.z, yaw_of(m.pose.orientation)), self.now()
        self._put_pose('ekf', m.header, m.pose)

    def _on_mavlink(self, m):
        if m.msgid != 230 or m.compid != 1:                # ESTIMATOR_STATUS from the autopilot
            return
        b = b''.join(struct.pack('<Q', w) for w in m.payload64)[:m.len].ljust(42, b'\0')   # MAVLink 2 trims trailing zeros
        vel, pos_h, pos_v, mag, hagl = struct.unpack_from('<5f', b, 8)
        t = self.now()
        self.last['est'] = (t, (vel, pos_h, pos_v, mag, hagl))
        self.counts['est'].append(t)
        if self.recording:
            self.rec['est'].append((t, vel, pos_h, pos_v, mag, hagl))

    def _on_caminfo(self, m):
        self.cam_n.append(self.now())
        self.cam['fx'] = float(m.k[0])
        if len(self.cam_n) > 10 and self.cam_n[-1] > self.cam_n[0]:
            self.cam['fps'] = round((len(self.cam_n) - 1) / (self.cam_n[-1] - self.cam_n[0]), 1)

    def _on_zstat(self, m):
        try:
            self.zstat = json.loads(m.data)
        except ValueError:
            return
        if self.recording:
            odo = (self.zstat.get('components') or {}).get('odometry') or {}
            last = (odo.get('compass') or {}).get('last')
            dv = ((self.zstat.get('components') or {}).get('depth') or {}).get('valid')
            self.zseries.append((self.now(), dv, last[2] if last else None))

    def _on_kinetix(self, m):
        try:
            self.tol = json.loads(m.data).get('tol') or self.tol
        except (ValueError, AttributeError):
            pass

    def _read_params(self):
        if self.recording or not self.param_cli.services_are_ready():
            return
        def done(fut):
            try:
                vals = fut.result().values
            except Exception:                              # noqa: BLE001  (MAVROS has not pulled the params yet)
                return
            for n, v in zip(PARAMS, vals):
                x = v.integer_value if v.type == 2 else v.double_value if v.type == 3 else None
                if x is not None:
                    self.params[n] = x
        self.param_cli.get_parameters(PARAMS).add_done_callback(done)

    def _on_cmd(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        a = d.get('action')
        if a == 'stop':
            if self.fl:
                self._abort('aborted from the GCS')
            elif self.state == 'recording':
                self._finish()
            return
        if self.state in ('flying', 'recording'):
            return self._say(f'refused: {self.state}')
        if a == 'record':
            self._start_rec()
            self.phases = [[self.now(), None, 'hand', None]]
            self.state = 'recording'
            return self._say('recording: turn the drone slowly through 360 deg, pausing every 45 deg, lift it, then Stop')
        if a == 'analyse':
            return self._start_flight()

    def _say(self, text):
        self.msg = text
        self.get_logger().info(text)

    def _start_rec(self):
        self.rec, self.result, self.tables, self.zseries = {k: [] for k in sa.STREAMS}, None, None, []
        self.zstart, self.recording, self.rec_t0 = self.zstat, True, self.now()
        if self.cli_cmd.service_is_ready():
            for msgid, hz in MSG_RATES.items():            # MAV_CMD_SET_MESSAGE_INTERVAL
                self.cli_cmd.call_async(CommandLong.Request(command=511, param1=float(msgid), param2=1e6 / hz))

    # ── the analyse flight (20 Hz state machine) ─────────────────────────────
    def _start_flight(self):
        if self.mavstate is None or not self.mavstate.connected:
            return self._say('refused: no FCU link')
        if self.mavstate.armed:
            return self._say('refused: armed')
        others = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                  for i in self.get_publishers_info_by_topic(self.resolve_topic_name('mavros/setpoint_position/local'))
                  if i.node_name != self.get_name()]
        if others:
            return self._say(f'refused: another node streams setpoints ({", ".join(others)})')
        if self.pose is None or self.now() - self.pose_t > 1.0:
            return self._say('refused: no local position (VIO / EKF2)')
        if self.landed != ExtendedState.LANDED_STATE_ON_GROUND:
            return self._say('refused: not on the ground')
        x, y, z, yaw = self.pose
        top = z + ALT_M
        steps = [('ground', GROUND_S), ('prestream', 2.0), ('arm', 10.0), ('climb', top), ('hover', HOVER_S)]
        for k in list(range(1, 9)) + list(range(7, -1, -1)):
            steps += [('turn', wrap(yaw + math.radians(STEP_DEG * k))), ('hold', HOLD_S)]
        steps += [('rise', top + RISE_M), ('hold', HOLD_S), ('rise', top), ('hold', HOLD_S)]
        c, s = math.cos(yaw), math.sin(yaw)
        for fx, fy in ((1, 0), (1, 1), (0, 1), (0, 0)):   # forward, left, back, home: in the start heading
            steps += [('move', (x + SQUARE_M * (c * fx - s * fy), y + SQUARE_M * (s * fx + c * fy))), ('hold', HOLD_S)]
        steps += [('land', 90.0), ('after', GROUND_S)]
        self._start_rec()
        self.phases, self.state = [], 'flying'
        self.sp_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.raw_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)
        self.fl = {'steps': steps, 'i': -1, 'sp': [x, y, z, yaw], 'disarm_t': 0.0, 'prev': None}
        self._next()

    def _next(self):
        f = self.fl
        if f['i'] >= 0:
            f['prev'] = f['steps'][f['i']][0]
        f['i'] += 1
        f['t'] = self.now()
        name, arg = f['steps'][f['i']]
        if self.phases and self.phases[-1][1] is None:
            self.phases[-1][1] = f['t']
        info = {'kind': {'turn': 'yaw', 'rise': 'height', 'move': 'move'}.get(f['prev'], 'hover'), 'sp': list(f['sp'])} \
            if name == 'hold' else None
        self.phases.append([f['t'], None, name, info])
        self._say(f'{name}' + (f' to {math.degrees(arg):.0f} deg' if name == 'turn' else '') + f' ({f["i"] + 1}/{len(f["steps"])})')

    def _abort(self, why):
        f = self.fl
        if f['steps'][f['i']][0] in ('ground', 'prestream', 'after') or not (self.mavstate and self.mavstate.armed):
            return self._end('failed', why)
        self._say(why + ': landing')
        f['steps'] = f['steps'][:f['i'] + 1] + [('land', 90.0), ('after', GROUND_S)]
        self._next()

    def _end(self, state, why):
        self.fl = None
        for pub in (self.sp_pub, self.raw_pub):
            if pub:
                self.destroy_publisher(pub)
        self.sp_pub = self.raw_pub = None
        self._say(why)
        self._finish(state)

    def _finish(self, state='done'):
        """Close the recording, analyse it, save it."""
        self.recording = False
        t = self.now()
        if self.phases and self.phases[-1][1] is None:
            self.phases[-1][1] = t
        phases = [tuple(p) for p in self.phases]
        meta = {'params': dict(self.params), 'tol': self.tol, 'cam': dict(self.cam), 'clock': dict(self.clock),
                'zstat': {'start': self.zstart, 'end': self.zstat, 'series': self.zseries}}
        try:
            self.result = sa.analyse({k: v for k, v in self.rec.items() if v}, phases, meta)
            self.tables = sa.tables(self.result)
        except Exception as e:                             # noqa: BLE001  (a broken analysis must not kill the node)
            self.result, self.tables = {'error': f'analysis failed: {e!r}'}, []
        self.state = state
        self._say(self.msg + ' · ' + (self.result.get('error') or f'analysed {self.result["duration_s"]} s'))
        self._save(phases, meta)

    def _save(self, phases, meta):
        try:
            os.makedirs(RUN_DIR, exist_ok=True)
            name = os.path.join(RUN_DIR, time.strftime('%Y%m%d-%H%M%S-') + (self.result or {}).get('kind', 'run'))
            arrays = {k: np.asarray(v, float) for k, v in self.rec.items() if v}
            np.savez_compressed(name + '.npz', meta=json.dumps({**meta, 'phases': phases}), **arrays)
            with open(name + '.json', 'w') as fh:
                json.dump({'result': self.result, 'tables': self.tables}, fh, indent=1)
            self.result['saved'] = name + '.npz'
        except (OSError, TypeError, ValueError) as e:
            self.get_logger().warning(f'run not saved: {e}')

    def _fly(self):
        if self.recording and self.fl is None and self.now() - self.rec_t0 > MAX_REC_S:
            self._say(f'record stopped after {MAX_REC_S:.0f} s')
            return self._finish()
        f = self.fl
        if f is None:
            return
        name, arg = f['steps'][f['i']]
        t, st = self.now() - f['t'], self.mavstate
        if self.now() - self.pose_t > 1.0 and name != 'after':
            return self._end('failed', 'local position lost: stopped streaming (PX4 failsafe takes over)')
        if name in ('climb', 'hover', 'turn', 'hold', 'rise', 'move', 'land') and st and st.armed and st.mode != 'OFFBOARD':
            return self._end('failed', f'mode changed to {st.mode}: the pilot has the drone, check ended')
        if name not in ('ground', 'after'):
            self._stream(name == 'land')
        sp, (x, y, z, yaw) = f['sp'], self.pose
        step = 0.05
        if name in ('ground', 'prestream', 'hover', 'hold', 'after'):
            if t >= arg:
                self._next() if name != 'after' else self._end('done', 'landed and disarmed')
        elif name == 'arm':                               # asked every 2 s: PX4 refuses while EKF2 is still settling
            if st and st.armed and st.mode == 'OFFBOARD':
                self._next()
            elif round(t / step) % 40 == 0:
                self.cli_mode.call_async(SetMode.Request(custom_mode='OFFBOARD'))
                self.cli_arm.call_async(CommandBool.Request(value=True))
            elif t >= arg:
                self._end('failed', f'arming / OFFBOARD refused (mode {st.mode if st else "?"}, armed {st.armed if st else "?"}); '
                                    f'PX4: {self.px4_text or "no warning"}')
        elif name in ('climb', 'rise'):                    # the setpoint ramps: a step would climb at PX4's max rate
            d = arg - sp[2]
            sp[2] = arg if abs(d) <= CLIMB_MPS * step else sp[2] + math.copysign(CLIMB_MPS * step, d)
            if sp[2] == arg and abs(z - arg) < 0.15:
                self._next()
            elif t >= 30:
                self._abort(f'{name} timed out')
        elif name == 'turn':
            d = wrap(arg - sp[3])
            sp[3] = arg if abs(d) <= math.radians(TURN_DPS) * step else wrap(sp[3] + math.copysign(math.radians(TURN_DPS) * step, d))
            if sp[3] == arg and abs(wrap(yaw - arg)) < math.radians(5) or t > 30:
                self._next()
        elif name == 'move':
            dx, dy = arg[0] - sp[0], arg[1] - sp[1]
            n = math.hypot(dx, dy)
            if n <= MOVE_MPS * step:
                sp[0], sp[1] = arg
            else:
                sp[0], sp[1] = sp[0] + MOVE_MPS * step * dx / n, sp[1] + MOVE_MPS * step * dy / n
            if (sp[0], sp[1]) == tuple(arg) and math.hypot(x - arg[0], y - arg[1]) < 0.15 or t > 30:
                self._next()
        elif name == 'land':
            if self.landed == ExtendedState.LANDED_STATE_ON_GROUND and t > 2.0:
                if not (st and st.armed):
                    return self._next()                    # 'after': motors off on the take-off floor
                if self.now() - f['disarm_t'] > 1.0:
                    f['disarm_t'] = self.now()
                    self.cli_arm.call_async(CommandBool.Request(value=False))
            elif t >= arg:
                self._end('failed', 'landing timed out: take over')

    def _stream(self, land):
        sp, stamp = self.fl['sp'], self.get_clock().now().to_msg()
        if land:                                          # a descent velocity: PX4's land detector needs one (vio_flight.py)
            m = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED, yaw=float(sp[3]),
                               type_mask=(PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                                          | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                                          | PositionTarget.IGNORE_YAW_RATE))
            m.header.stamp, m.velocity.z = stamp, -0.5
            return self.raw_pub.publish(m)
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = stamp, 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, sp[:3])
        m.pose.orientation.z, m.pose.orientation.w = math.sin(sp[3] / 2), math.cos(sp[3] / 2)
        self.sp_pub.publish(m)

    # ── status ────────────────────────────────────────────────────────────────
    def _live(self):
        """Per stream: rate (Hz over the last 2 s) and the latest values, rounded."""
        t, out = self.now(), {}
        for k, (ts, v) in self.last.items():
            c = self.counts[k]
            hz = sum(1 for x in c if x > t - 2.0) / 2.0
            out[k] = {'hz': hz, 'age': round(t - c[-1], 1) if c else None,
                      'v': [None if not math.isfinite(x) else float(f'{x:.4g}') for x in map(float, v)]}
        return out

    def _publish(self):
        out = {'t': time.time(), 'state': self.state, 'msg': self.msg, 'live': self._live(), 'params': self.params,
               'step': None if self.fl is None else [self.fl['i'] + 1, len(self.fl['steps'])],
               'samples': sum(len(v) for v in self.rec.values()) if self.recording else None,
               'result': None if self.result is None else {'recommend': self.result.get('recommend'), 'error': self.result.get('error'),
                                                           'kind': self.result.get('kind'), 'saved': self.result.get('saved'),
                                                           'duration_s': self.result.get('duration_s'), 'truth': self.result.get('truth')},
               'tables': self.tables, 'vibration_topic': Vibration is not None,
               'flight': {'alt_m': ALT_M, 'square_m': SQUARE_M, 'rise_m': RISE_M}}
        self.pub.publish(String(data=json.dumps(out)))


def main():
    rclpy.init()
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
