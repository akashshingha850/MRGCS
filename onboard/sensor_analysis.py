#!/usr/bin/env python3
"""
sensor_analysis.py — the analysis behind the GCS Sensors tab: which source EKF2 should trust for heading, height and horizontal
velocity on this drone in this room, the IMU, the ZED latency, the EV lever arm and how well the drone holds a Kinetix capture.
numpy only (no ROS): sensor_check.py records, this judges; tests/test_sensor_analysis.py feeds it synthetic flights.

Recording: {stream: array of rows (t, ...)}, t in seconds on one clock (ROS time: header stamps), columns in STREAMS. Phases:
[(t0, t1, name, info)] from the flight (ground, climb, hover, turn, hold, rise, move, land, after) or one 'hand' phase.
Meta: params (EKF2_* now on the FCU), tol (Kinetix capture tolerances), cam (fx, fps), zstat (bridge status at start / end).

How a source is judged without ground truth: heading, by its error per 45 deg yaw step against the FCU gyro integrated over the
step (a few seconds: well under a degree of gyro drift), which is where VIO slips and magnetometer deviation show; height, by the
three-cornered hat of baro, range and ZED (three independent sources of one quantity give each one's noise from the variances of their
pairwise differences: var(a-b) = sa^2 + sb^2); velocity, by the same hat over flow, ZED and EKF2 (approximate: EKF2 fuses the other
two). A constant offset between sources (frame, declination, sensor height) is removed first; it is not an error.
With ground truth (sim, state/pose) every source is compared with it directly and that decides.
Truth-free absolute checks: the drone lands on the floor it took off from (height closure = 0) and on the start point when EKF2 is
right (horizontal closure: measure the real landing offset with a tape; the source that agrees is right).

Re-analyse a saved run:  python3 sensor_analysis.py /data/sensor_runs/<run>.npz
"""
import json
import math
import sys

import numpy as np

STREAMS = {
    'att': ('qx', 'qy', 'qz', 'qw', 'gx', 'gy', 'gz'),        # mavros/imu/data: attitude (ENU, FLU body) + gyro (rad/s)
    'imu_raw': ('ax', 'ay', 'az', 'gx', 'gy', 'gz'),           # mavros/imu/data_raw (m/s^2, rad/s)
    'mag': ('mx', 'my', 'mz'),                                 # mavros/imu/mag (T, body FLU)
    'baro': ('p',),                                            # mavros/imu/static_pressure (Pa)
    'range': ('r',),                                           # mavros/hrlv_ez4_pub, the LW20 (m along body -z; NaN = invalid)
    'flow': ('ix', 'iy', 'igx', 'igy', 'dt', 'q'),             # mavros/px4flow/raw/optical_flow_rad (rad, s)
    'ekf': ('x', 'y', 'z', 'yaw'),                             # mavros/local_position/pose
    'ekf_vb': ('vx', 'vy', 'vz'),                              # mavros/local_position/velocity_body (FLU)
    'zed': ('x', 'y', 'z', 'yaw'),                             # zed/zed_node/odom: the ZED's own VIO
    'vision': ('x', 'y', 'z', 'yaw'),                          # mavros/odometry/out: what EKF2 gets (VIO + the bridge's corrections)
    'truth': ('x', 'y', 'z', 'yaw'),                           # state/pose: sim ground truth only
    'zimu': ('gx', 'gy', 'gz', 'ax', 'ay', 'az'),              # zed/zed_node/imu/data
    'vib': ('coning', 'gyro', 'accel', 'clip0', 'clip1', 'clip2'),   # mavros/vibration/raw/vibration (PX4 VIBRATION.hpp)
    'est': ('vel', 'pos_h', 'pos_v', 'mag', 'hagl'),           # ESTIMATOR_STATUS test ratios (> 1 = measurement rejected)
}
AIR = ('hover', 'turn', 'hold', 'rise', 'move')                # phases in the air, steady enough to compare sources
GROUND = ('ground', 'hand')                                    # the reference stillness at the start (hand: the whole record)
DT = 0.05                                                      # comparison grid (s)
MAX_RATE_DPS = 8.0                                             # heading compared only while turning slower (mag vs VIO bandwidth)
FLOW_QMIN = 1                                                  # flow samples below this quality are dropped (EKF2_OF_QMIN)
BLUR_DEPTH_M = 3.0                                             # scene distance for the motion-blur estimate (Kinetix room views)
MAG_LIMITS = {'deviation_deg': 5.0, 'field_pct': 5.0, 'incl_deg': 3.0, 'motor_deg': 3.0}
EV_BITS = {'hpos': 1, 'vpos': 2, 'vel': 4, 'yaw': 8}           # EKF2_EV_CTRL


# ── geometry ──────────────────────────────────────────────────────────────────
def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def circ_median(a):
    """Median of angles (rad) around their circular mean, so a wrap at +-180 does not split it."""
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    if not len(a):
        return float('nan')
    c = math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a)))
    return float(wrap(c + np.median(wrap(a - c))))


def r33(q):
    """cos(tilt) of attitudes q (n, 4: x y z w): the vertical component of the body z axis."""
    return 1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2)


def mag_heading(q, m):
    """(heading rad, inclination deg) per sample of the field m (n, 3, body FLU) seen by attitudes q (n, 4, ENU): north = 0 up to a
    constant (declination + frame), which the offset absorbs. Only q's roll / pitch matter."""
    x, y, z, w = q.T
    R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                  np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                  np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)
    f = np.einsum('nij,nj->ni', R, m)                          # field in the world frame
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = np.cos(yaw), np.sin(yaw)
    lx, ly = c * f[:, 0] + s * f[:, 1], -s * f[:, 0] + c * f[:, 1]   # level, yaw-free body frame
    return -np.arctan2(ly, lx), np.degrees(np.arctan2(-f[:, 2], np.hypot(lx, ly)))


# ── resampling ────────────────────────────────────────────────────────────────
def on_grid(t, v, grid, angle=False, gap=0.5):
    """v(t) linearly interpolated at grid; NaN where the nearest sample is more than `gap` s away (dropouts stay dropouts)."""
    ok = np.isfinite(v) & np.isfinite(t)
    t, v = t[ok], v[ok]
    if len(t) < 2:
        return np.full(len(grid), np.nan)
    o = np.argsort(t)
    t, v = t[o], (np.unwrap(v[o]) if angle else v[o])
    out = np.interp(grid, t, v)
    i = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    near = np.minimum(np.abs(grid - t[i - 1]), np.abs(t[i] - grid))
    out[near > gap] = np.nan
    return wrap(out) if angle else out


def rate(v, dt, k=5, angle=False):
    """Derivative of a grid series over +-k samples (smooths stereo / quantisation noise)."""
    v = np.unwrap(np.nan_to_num(v)) if angle else v
    d = np.full(len(v), np.nan)
    if len(v) > 2 * k:
        d[k:-k] = (v[2 * k:] - v[:-2 * k]) / (2 * k * dt)
    return d


def to_body(vx, vy, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


# ── statistics ────────────────────────────────────────────────────────────────
def stats(e):
    """(median, p90, max) of |e|, NaNs ignored."""
    e = np.abs(np.asarray(e, float))
    e = e[np.isfinite(e)]
    return [round(float(v), 3) for v in (np.median(e), np.percentile(e, 90), e.max())] if len(e) else None


def hat(a, b, c, angle=False):
    """Three-cornered hat: the noise std of each of three series with independent errors, from their pairwise differences (offsets
    removed). A negative variance estimate (the errors are correlated, or too few samples) comes back as 0."""
    ok = np.isfinite(a) & np.isfinite(b) & np.isfinite(c)
    if ok.sum() < 30:
        return None
    a, b, c = a[ok], b[ok], c[ok]
    def var(x, y):
        d = wrap(x - y) if angle else x - y
        d = wrap(d - circ_median(d)) if angle else d - np.median(d)
        return float(np.mean(d ** 2))
    vab, vac, vbc = var(a, b), var(a, c), var(b, c)
    return [math.sqrt(max(0.0, v)) for v in ((vab + vac - vbc) / 2, (vab + vbc - vac) / 2, (vac + vbc - vab) / 2)]


def residual(src, ref, angle=False):
    """src - ref with the constant offset removed: (offset, residual series)."""
    d = wrap(src - ref) if angle else src - ref
    off = circ_median(d) if angle else float(np.nanmedian(d)) if np.isfinite(d).any() else float('nan')
    return off, (wrap(d - off) if angle else d - off)


def lag(ref, src, dt, max_lag=0.3):
    """Delay of src behind ref (s; + = src lags the motion), from the peak of their normalised cross-correlation (parabolic peak)."""
    ok = np.isfinite(ref) & np.isfinite(src)
    if ok.sum() < 100 or np.nanstd(ref[ok]) < math.radians(5):
        return None
    n, best = int(max_lag / dt), []
    for k in range(-n, n + 1):
        a, b = (ref[:len(ref) - k], src[k:]) if k >= 0 else (ref[-k:], src[:len(src) + k])
        m = np.isfinite(a) & np.isfinite(b)
        best.append(np.corrcoef(a[m], b[m])[0, 1] if m.sum() > 50 else -1.0)
    best = np.array(best)
    i = int(np.argmax(best))
    if best[i] < 0.7:
        return None
    off = 0.0
    if 0 < i < len(best) - 1:
        den = best[i - 1] - 2 * best[i] + best[i + 1]
        off = 0.5 * (best[i - 1] - best[i + 1]) / den if den else 0.0
    return {'lag_ms': round(float((i - n + off) * dt * 1000), 1), 'corr': round(float(best[i]), 3)}


def mask_of(grid, phases, names):
    m = np.zeros(len(grid), bool)
    for t0, t1, name, _ in phases:
        if name in names:
            m |= (grid >= t0) & (grid <= t1)
    return m


def r(v, n=2):
    return None if v is None or not np.isfinite(v) else round(float(v), n)


# ── the analysis ──────────────────────────────────────────────────────────────
def analyse(rec, phases, meta):
    if not phases:
        return {'error': 'nothing recorded'}
    t0, t1 = phases[0][0], phases[-1][1]                       # the grid spans the run; a stream on another clock falls outside it
    rec = {k: np.asarray(v, float).reshape(-1, len(STREAMS[k]) + 1) for k, v in rec.items() if k in STREAMS and len(v)}
    off_clock = [k for k, v in rec.items() if ((v[:, 0] >= t0) & (v[:, 0] <= t1)).mean() < 0.5]
    rec = {k: v for k, v in rec.items() if k not in off_clock}
    if not rec or t1 - t0 < 1.0:
        return {'error': 'nothing recorded', 'off_clock': off_clock}
    grid = np.arange(t0, t1, DT)
    g = lambda k, c, **kw: on_grid(rec[k][:, 0], rec[k][:, 1 + STREAMS[k].index(c)], grid, **kw) if k in rec \
        else np.full(len(grid), np.nan)
    ground, air = mask_of(grid, phases, GROUND), mask_of(grid, phases, AIR)
    after = mask_of(grid, phases, ('after',))
    hand = any(p[2] == 'hand' for p in phases)
    if hand:                                                   # by hand: the whole record is "steady", no flight phases
        air = ground.copy()
    truth = 'truth' in rec
    meta = dict(meta or {})
    res = {'kind': 'hand' if hand else 'flight', 'duration_s': round(float(grid[-1] - grid[0]), 1), 'truth': truth,
           'samples': {k: len(v) for k, v in rec.items()}, 'off_clock': off_clock,
           'arrival_time': [k for k, c in (meta or {}).get('clock', {}).items() if c == 'arrival']}

    q = np.stack([g('att', c) for c in ('qx', 'qy', 'qz', 'qw')], 1)
    yaw = {k: g(k, 'yaw', angle=True) for k in ('ekf', 'zed', 'vision', 'truth')}
    res['heading'] = heading(rec, grid, g, q, yaw, ground, air, after, phases, truth, gyro_yaw(rec, grid))
    res['height'] = height(rec, grid, g, q, ground, air, after, truth)
    res['velocity'] = velocity(rec, grid, g, yaw, ground, air, after, truth)
    res['imu'] = imu(rec, grid, phases)
    res['latency'] = latency(grid, g, yaw, phases)
    res['lever_arm'] = lever_arm(grid, g, yaw, phases, meta.get('params', {})) if truth else None
    res['kinetix'] = kinetix(grid, g, yaw, phases, meta, truth)
    res['innovation'] = innovation(rec, grid, air)
    res['recommend'] = recommend(res, meta.get('params', {}))
    return res


def gyro_yaw(rec, grid):
    """Yaw from the FCU gyro alone (rad on the grid, arbitrary start): the world-z rate (body rates through the attitude's tilt)
    integrated at the gyro's own rate. Short-term exact, drifts with the gyro bias (~0.01 deg/s)."""
    if 'att' not in rec or len(rec['att']) < 10:
        return np.full(len(grid), np.nan)
    a = rec['att'][np.argsort(rec['att'][:, 0])]
    x, y, z, w, gx, gy, gz = a[:, 1:].T
    wz = 2 * (x * z - y * w) * gx + 2 * (y * z + x * w) * gy + (1 - 2 * (x * x + y * y)) * gz
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(a[:, 0]))])
    return on_grid(a[:, 0], psi, grid, gap=1.0)


def yaw_steps(grid, src, gyro, phases):
    """Per yaw step (hover/hold, turn, hold): each source's heading change minus the gyro's, over the steady last 1.5 s of each
    hold. -> {source: [error rad per step]}."""
    wins, turned = [], False
    for t0, t1, name, info in phases:
        if name == 'turn':
            turned = True
        elif name in ('hover', 'hold') and (name == 'hover' or (info or {}).get('kind') == 'yaw'):
            wins.append(((grid >= max(t0, t1 - 1.5)) & (grid <= t1), turned))
            turned = False
    out = {k: [] for k in src}
    for (a, _), (b, turned_between) in zip(wins, wins[1:]):
        if not turned_between:
            continue
        dg = np.nanmedian(gyro[b]) - np.nanmedian(gyro[a])
        for k, v in src.items():
            e = wrap(circ_median(v[b]) - circ_median(v[a]) - dg)
            if np.isfinite(e):
                out[k].append(float(e))
    return out


def heading(rec, grid, g, q, yaw, ground, air, after, phases, truth, gyro):
    src = {k: yaw[k] for k in ('zed', 'vision', 'ekf', 'truth') if k in rec}
    field = incl = None
    if 'mag' in rec and 'att' in rec:
        m = rec['mag']
        qa = np.stack([on_grid(rec['att'][:, 0], rec['att'][:, i], m[:, 0]) for i in range(1, 5)], 1)
        ok = np.isfinite(qa).all(1) & (np.linalg.norm(m[:, 1:], axis=1) > 0)
        h, inc = mag_heading(qa[ok], m[ok, 1:])
        src['mag'] = on_grid(m[ok, 0], h, grid, angle=True)
        field = on_grid(m[ok, 0], 1e6 * np.linalg.norm(m[ok, 1:], axis=1), grid)
        incl = on_grid(m[ok, 0], inc, grid)
    ref_yaw = yaw['truth'] if truth else yaw['ekf']
    turn = np.abs(rate(ref_yaw, DT, angle=True))
    steady = (ground | air) & (np.degrees(turn) < MAX_RATE_DPS)
    out = {'ref': 'truth' if truth else 'ekf', 'sources': {}}
    steps = yaw_steps(grid, src, gyro, phases)
    out['steps'] = len(next(iter(steps.values()), []))
    for k, v in src.items():
        if k == ('truth' if truth else 'ekf'):
            continue
        off, e = residual(np.where(steady, v, np.nan), np.where(steady, ref_yaw, np.nan), angle=True)
        d = {'offset_deg': r(math.degrees(off), 1), 'residual_deg': stats(np.degrees(e)),
             'valid_pct': r(100 * np.isfinite(v[ground | air]).mean(), 0) if (ground | air).any() else None}
        if steps.get(k):
            d['step_err_deg'] = stats(np.degrees(steps[k]))
            d['step_sum_deg'] = r(math.degrees(sum(steps[k])), 1)      # accumulated over all steps: the drift a scan collects
        if after.any() and ground.any():                       # heading change between the two ground phases (motors off both)
            d['ground_change_deg'] = r(math.degrees(wrap(circ_median(v[after]) - circ_median(v[ground]))), 1)
        out['sources'][k] = d
    if steps.get('truth'):                                     # the gyro reference itself, checked against truth
        out['gyro_step_err_deg'] = stats(np.degrees(steps['truth']))
    if after.any() and ground.any() and np.isfinite(gyro[after]).any():
        out['gyro_ground_change_deg'] = r(math.degrees(wrap(np.nanmedian(gyro[after]) - np.nanmedian(gyro[ground]))), 1)
    if truth and after.any():
        tc = math.degrees(wrap(circ_median(yaw['truth'][after]) - circ_median(yaw['truth'][ground])))
        for d in out['sources'].values():
            if d.get('ground_change_deg') is not None:
                d['closure_err_deg'] = r(abs(math.degrees(wrap(math.radians(d['ground_change_deg'] - tc)))), 1)
    if 'mag' in src:
        out['mag'] = mag_health(src['mag'], ref_yaw, field, incl, steady, mask_of(grid, phases, ('ground',)),
                                mask_of(grid, phases, ('hover',)))
    return out


def mag_health(mag, ref, field, incl, steady, ground, hover):
    """The magnetometer's own health: deviation per 45 deg of heading after the constant offset, a steady field strength and
    inclination as the drone turns (a steel structure or a hard-iron error changes both), and no heading change with the motors on."""
    off, e = residual(np.where(steady, mag, np.nan), np.where(steady, ref, np.nan), angle=True)
    hdg = np.degrees(ref) % 360
    sectors = []
    for k in range(8):
        sel = steady & np.isfinite(e) & (np.abs(wrap(np.radians(hdg - 45 * k))) < math.radians(22.5))
        if sel.sum() >= 10:
            sectors.append({'hdg': 45 * k, 'dev': r(math.degrees(np.median(e[sel])))})
    f, inc = field[steady & np.isfinite(field)], incl[steady & np.isfinite(incl)]
    if len(f) < 30:
        return None
    checks = {'deviation_deg': max([abs(s['dev']) for s in sectors] or [0.0]),
              'field_pct': 100 * float(f.std() / f.mean()), 'incl_deg': float(inc.std())}
    d = wrap(mag - ref)
    gd, hv = d[ground & np.isfinite(d)], d[hover & np.isfinite(d)]
    if len(gd) > 20 and len(hv) > 20:                          # motor current: same heading, motors off vs hovering
        checks['motor_deg'] = abs(math.degrees(wrap(circ_median(hv) - circ_median(gd))))
    swept = float(np.ptp(np.degrees(np.unwrap(ref[steady & np.isfinite(ref)])))) if steady.any() else 0.0
    out = {'field_uT': r(f.mean(), 1), 'incl_deg': r(inc.mean(), 1), 'swept_deg': r(swept, 0), 'sectors': sectors,
           'checks': {k: {'value': r(v), 'limit': MAG_LIMITS[k], 'ok': v <= MAG_LIMITS[k]} for k, v in checks.items()}}
    out['healthy'] = all(c['ok'] for c in out['checks'].values()) and swept >= 270
    if swept < 270:
        out['note'] = 'turn through at least 270 deg: a deviation that changes with heading hides in the offset otherwise'
    return out


def height(rec, grid, g, q, ground, air, after, truth):
    src = {}
    if 'baro' in rec:
        p = g('baro', 'p')
        p0 = np.nanmedian(p[ground]) if np.isfinite(p[ground]).any() else np.nanmedian(p)
        src['baro'] = 44330.77 * (1 - (p / p0) ** 0.190263)
    if 'range' in rec:
        src['range'] = g('range', 'r') * r33(q)                # LW20 along body -z -> vertical
    for k in ('zed', 'vision', 'ekf', 'truth'):
        if k in rec:
            z = g(k, 'z')
            src[k] = z - (np.nanmedian(z[ground]) if np.isfinite(z[ground]).any() else 0.0)
    ref_k = 'truth' if truth else 'range' if 'range' in src else 'ekf'
    out = {'ref': ref_k, 'sources': {}}
    if ref_k not in src:
        return out
    ref = src[ref_k]
    sig = hat(*(np.where(air, src.get(k, np.nan), np.nan) for k in ('baro', 'range', 'zed')))
    for k, v in src.items():
        d = {'valid_pct': r(100 * np.isfinite(v[air]).mean(), 0) if air.any() else None}
        if k != ref_k:
            off, e = residual(np.where(air, v, np.nan), np.where(air, ref, np.nan))
            d['residual_m'] = stats(e)
            ok = air & np.isfinite(v) & np.isfinite(ref)
            if ok.sum() > 50:
                d['drift_m_min'] = r(60 * np.polyfit(grid[ok], (v - ref)[ok], 1)[0], 3)
                if np.ptp(ref[ok]) > 0.3:                      # scale needs a height change (the rise step)
                    d['scale'] = r(np.polyfit(ref[ok], v[ok], 1)[0], 3)
        if sig and k in ('baro', 'range', 'zed'):
            d['sigma_m'] = r(sig[('baro', 'range', 'zed').index(k)], 3)
        if k != 'range' and after.any() and np.isfinite(v[after]).any():   # landed on the same floor: should be 0
            d['closure_m'] = r(np.nanmedian(v[after]), 3)
        out['sources'][k] = d
    return out


def velocity(rec, grid, g, yaw, ground, air, after, truth):
    src = {}
    for k in ('zed', 'vision', 'truth'):
        if k in rec:
            src[k] = to_body(rate(g(k, 'x'), DT), rate(g(k, 'y'), DT), yaw[k])
    if 'ekf_vb' in rec:
        src['ekf'] = (g('ekf_vb', 'vx'), g('ekf_vb', 'vy'))
    out = {'ref': 'truth' if truth else None, 'sources': {}}
    if 'flow' in rec and 'range' in rec:
        f = rec['flow']
        ok = (f[:, 5] > 0) & (f[:, 6] >= FLOW_QMIN)
        d = on_grid(rec['range'][:, 0], rec['range'][:, 1], f[:, 0])
        wx, wy = (f[:, 1] - f[:, 3]) / np.where(ok, f[:, 5], 1), (f[:, 2] - f[:, 4]) / np.where(ok, f[:, 5], 1)
        ok &= np.isfinite(d)
        ux, uy = on_grid(f[ok, 0], wx[ok] * d[ok], grid), on_grid(f[ok, 0], wy[ok] * d[ok], grid)
        src['flow'] = (-uy, ux)                                # MAVROS FLU: body velocity = (-wy, wx) * distance
        fit_ref = src.get('truth') or src.get('zed') or src.get('ekf')
        if fit_ref is not None:
            out['flow_axes'] = flow_axes(ux, uy, fit_ref, air)
        out['flow_quality_ok_pct'] = r(100 * ok.mean(), 0)
    if not src:
        return out
    ref = src.get('truth')
    trio = ('flow', 'zed', 'ekf')
    sig = [hat(*(np.where(air, src[k][i], np.nan) if k in src else np.full(len(grid), np.nan) for k in trio)) for i in (0, 1)]
    for k, (vx, vy) in src.items():
        if k == 'truth':
            continue
        d = {'valid_pct': r(100 * np.isfinite(vx[air]).mean(), 0) if air.any() else None}
        if ref is not None:
            d['error_mps'] = stats(np.where(air, np.hypot(vx - ref[0], vy - ref[1]), np.nan))
        if 'ekf' in src and k != 'ekf':
            d['vs_ekf_rms_mps'] = r(np.sqrt(np.nanmean(np.where(air, (vx - src['ekf'][0]) ** 2 + (vy - src['ekf'][1]) ** 2, np.nan))), 3)
        if sig[0] and sig[1] and k in trio:
            i = trio.index(k)
            d['sigma_mps'] = r(math.hypot(sig[0][i], sig[1][i]), 3)
        out['sources'][k] = d
    for k in ('zed', 'vision', 'ekf', 'truth'):                # start -> landing displacement, each in its own frame
        if k in rec and ground.any() and after.any():
            x, y = g(k, 'x'), g(k, 'y')
            dx, dy = np.nanmedian(x[after]) - np.nanmedian(x[ground]), np.nanmedian(y[after]) - np.nanmedian(y[ground])
            out.setdefault('closure_m', {})[k] = r(math.hypot(dx, dy), 3)
    return out


def flow_axes(ux, uy, ref, air):
    """Least-squares 2x2 map from the flow's raw (wx, wy) * distance to the body velocity: rotation vs the expected one (0 = the
    PX4 / MAVROS conventions hold, else SENS_FLOW_ROT is off by it), scale (1 = the sensor's focal length is right), mirrored."""
    ok = air & np.isfinite(ux) & np.isfinite(uy) & np.isfinite(ref[0]) & np.isfinite(ref[1])
    U, V = np.stack([ux[ok], uy[ok]], 1), np.stack([ref[0][ok], ref[1][ok]], 1)
    if ok.sum() < 50 or np.std(V) < 0.05:
        return None                                            # no motion to fit
    M = np.linalg.lstsq(U, V, rcond=None)[0].T
    det = float(np.linalg.det(M))
    ang = math.degrees(math.atan2(M[1, 0] - M[0, 1], M[0, 0] + M[1, 1])) if det > 0 else \
        math.degrees(math.atan2(M[1, 0] + M[0, 1], M[0, 0] - M[1, 1]))
    rot = (ang - 90 + 180) % 360 - 180                         # expected map: [[0, -1], [1, 0]] = +90 deg
    return {'rotation_deg': r(rot, 0), 'scale': r(math.sqrt(abs(det)), 3), 'mirrored': det < 0,
            'ok': det > 0 and abs(rot) < 20 and 0.8 < math.sqrt(abs(det)) < 1.25}


def imu(rec, grid, phases):
    out = {}
    def window(k, names):
        if k not in rec:
            return None
        t, sel = rec[k][:, 0], np.zeros(len(rec[k]), bool)
        for t0, t1, name, _ in phases:
            if name in names:
                sel |= (t >= t0) & (t <= t1)
        return rec[k][sel, 1:] if sel.sum() > 20 else None
    for k, gi, ai in (('imu_raw', 3, 0), ('zimu', 0, 3)):      # FCU (raw) and ZED IMU: noise at rest (motors off) and hovering
        for ph in ('ground', 'hover'):
            w = window(k, (ph,) if ph == 'hover' else GROUND)
            if w is not None:
                out.setdefault(k, {})[ph] = {'gyro_std_dps': r(math.degrees(np.linalg.norm(w[:, gi:gi + 3].std(0))), 3),
                                             'accel_std_mps2': r(np.linalg.norm(w[:, ai:ai + 3].std(0)), 3)}
    fg, zg = window('imu_raw', GROUND), window('zimu', GROUND)
    if fg is not None and zg is not None:                     # yaw-rate bias between the two IMUs at rest (deg/s)
        out['gyro_z_bias_diff_dps'] = r(math.degrees(fg[:, 5].mean() - zg[:, 2].mean()), 3)
    if 'vib' in rec:
        v = rec['vib']
        for ph in ('ground', 'hover'):
            w = window('vib', (ph,) if ph == 'hover' else GROUND)
            if w is not None:
                out.setdefault('vibration', {})[ph] = {c: r(np.median(np.abs(w[:, i])), 4) for i, c in enumerate(('coning', 'gyro', 'accel'))}
        clip = int((v[-1, 4:7] - v[0, 4:7]).sum())             # cumulative counters: new clipping during the run
        out.setdefault('vibration', {})['clipping'] = clip
        out['vibration']['ok'] = clip == 0
    return out


def latency(grid, g, yaw, phases):
    """Stamp delay of each yaw source behind the FCU gyro, from the turns: the EV delay EKF2 should use (EKF2_EV_DELAY on the drone,
    the bridge's restamp in the sim). The gyro's own stamps are the FCU's (MAVROS timesync)."""
    turns = mask_of(grid, phases, ('turn', 'hold', 'hand'))
    gyro = np.where(turns, g('att', 'gz'), np.nan)
    out = {}
    for k in ('zed', 'vision', 'truth'):
        out[k] = lag(gyro, np.where(turns, rate(yaw[k], DT, k=2, angle=True), np.nan), DT) if np.isfinite(yaw[k]).any() else None
    out['zimu'] = lag(gyro, np.where(turns, g('zimu', 'gz'), np.nan), DT)
    return out


def lever_arm(grid, g, yaw, phases, params):
    """Sim only: the EV position relative to the body (truth) in in-place turns traces the camera's lever arm: fit
    p_vision - p_truth = c + Rz(yaw) r (vision rotated into the truth frame first). r should equal EKF2_EV_POS (FRD -> FLU)."""
    sel = mask_of(grid, phases, ('turn', 'hold')) & np.isfinite(g('vision', 'x')) & np.isfinite(g('truth', 'x'))
    if sel.sum() < 100:
        return None
    gnd = mask_of(grid, phases, GROUND)                       # frames aligned before any turn: a VIO yaw slip is not a frame rotation
    th0 = circ_median((yaw['vision'] - yaw['truth'])[gnd if gnd.any() else sel])
    c, s = math.cos(-th0), math.sin(-th0)
    vx, vy = g('vision', 'x')[sel], g('vision', 'y')[sel]
    dx, dy = c * vx - s * vy - g('truth', 'x')[sel], s * vx + c * vy - g('truth', 'y')[sel]
    psi = yaw['truth'][sel]
    if np.ptp(np.unwrap(psi)) < math.radians(90):
        return None
    cp, sp = np.cos(psi), np.sin(psi)
    one, zero = np.ones_like(cp), np.zeros_like(cp)
    A = np.concatenate([np.stack([one, zero, cp, -sp], 1), np.stack([zero, one, sp, cp], 1)])
    sol, *_ = np.linalg.lstsq(A, np.concatenate([dx, dy]), rcond=None)
    rx, ry = float(sol[2]), float(sol[3])
    cfg = (params.get('EKF2_EV_POS_X'), None if params.get('EKF2_EV_POS_Y') is None else -params['EKF2_EV_POS_Y'])
    return {'fit_flu_m': [r(rx, 3), r(ry, 3)], 'config_flu_m': [r(v, 3) if v is not None else None for v in cfg],
            'err_m': r(math.hypot(rx - (cfg[0] or 0.0), ry - (cfg[1] or 0.0)), 3)}


def kinetix(grid, g, yaw, phases, meta, truth):
    """How the drone holds a Kinetix capture: per hold, the time from the setpoint stopping to the first moment EKF2 is inside the
    capture tolerances for settle_s (what Kinetix waits for), the motion blur at those moments and, in the sim, the real pose error."""
    tol = {'pos_m': 0.10, 'yaw_deg': 3.0, 'speed_mps': 0.05, 'settle_s': 1.0, **(meta.get('tol') or {})}
    px, py, pz, pyaw = g('ekf', 'x'), g('ekf', 'y'), g('ekf', 'z'), yaw['ekf']
    speed = np.linalg.norm(np.stack([g('ekf_vb', c) for c in ('vx', 'vy', 'vz')], 1), axis=1)
    w = np.linalg.norm(np.stack([g('att', c) for c in ('gx', 'gy', 'gz')], 1), axis=1)
    holds, ready = {}, np.zeros(len(grid), bool)
    n_settle = int(round(tol['settle_s'] / DT))
    for t0, t1, name, info in phases:
        if name != 'hold' or not info or 'sp' not in info:
            continue
        x, y, z, sy = info['sp']
        sel = np.where((grid >= t0) & (grid <= t1))[0]
        inside = (np.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2) < tol['pos_m']) & \
                 (np.abs(np.degrees(wrap(pyaw - sy))) < tol['yaw_deg']) & (speed < tol['speed_mps'])
        st = None
        for j in range(len(sel) - n_settle):
            if inside[sel[j:j + n_settle]].all():
                st = float(grid[sel[j + n_settle]] - t0)
                ready[sel[j + n_settle]] = True
                break
        ready[sel] |= inside[sel]
        holds.setdefault(info.get('kind', 'hold'), []).append(st)
    out = {'tol': tol, 'settle_s': {}}
    for k, v in holds.items():
        ok = [x for x in v if x is not None]
        out['settle_s'][k] = {'n': len(v), 'settled': len(ok), 'median': r(np.median(ok)) if ok else None,
                              'max': r(max(ok)) if ok else None}
    cam = meta.get('cam') or {}
    fx, fps = cam.get('fx'), cam.get('fps') or 30.0
    if fx and ready.any():                                    # exposure <= 1 / fps: an upper bound, auto exposure is shorter
        blur = fx / fps * (w + speed / BLUR_DEPTH_M)
        out['blur_px'] = stats(blur[ready])
        out['blur_assumed'] = {'fx': r(fx, 1), 'exposure_ms': r(1000 / fps, 1), 'depth_m': BLUR_DEPTH_M}
    if truth and ready.any():                                 # EKF2 pose vs truth at capture-ready moments, frames aligned at start
        gnd = mask_of(grid, phases, ('ground',))
        tx, ty, tyaw = g('truth', 'x'), g('truth', 'y'), yaw['truth']
        if gnd.any():
            th = circ_median(pyaw[gnd] - tyaw[gnd])
            c, s = math.cos(-th), math.sin(-th)
            ex, ey = px - np.nanmedian(px[gnd]), py - np.nanmedian(py[gnd])
            ax, ay = c * ex - s * ey + np.nanmedian(tx[gnd]), s * ex + c * ey + np.nanmedian(ty[gnd])
            out['pose_err_m'] = stats(np.hypot(ax - tx, ay - ty)[ready])
            out['yaw_err_deg'] = stats(np.degrees(wrap(pyaw - th - tyaw))[ready])
    z = meta.get('zstat') or {}
    out['room'] = room(z.get('start'), z.get('end'), z.get('series') or [])
    return out


def room(start, end, series):
    """Is this room good for the ZED and the wall compass (from the bridge's zed_stack/status at the start and end of the run)."""
    def odo(s):
        return ((s or {}).get('components') or {}).get('odometry') or {}
    if not end:
        return None
    out = {'state': end.get('state')}
    c0, c1 = odo(start).get('compass') or {}, odo(end).get('compass') or {}
    if c1:
        acc, rej = c1.get('accepted', 0) - c0.get('accepted', 0), c1.get('rejected', 0) - c0.get('rejected', 0)
        out['compass_accept_pct'] = r(100 * acc / (acc + rej), 0) if acc + rej else None
        out['compass_drift_deg'] = c1.get('drift_deg')
    sh = [s[2] for s in series if s[2] is not None]
    dv = [s[1] for s in series if s[1] is not None]
    if sh:
        out['compass_sharpness_med'] = r(np.median(sh), 2)
    if dv:
        out['depth_valid_med'] = r(np.median(dv), 2)
    if odo(end):
        out['odom_lost_events'] = odo(end).get('lost_events', 0) - odo(start).get('lost_events', 0)
        out['odom_hz'] = odo(end).get('rate_hz')
    cam = ((end.get('components') or {}).get('camera') or {})
    out['camera'] = cam.get('detail') or cam.get('state')
    return out


def innovation(rec, grid, air):
    if 'est' not in rec:
        return None
    e = rec['est']
    sel = np.interp(e[:, 0], grid, air.astype(float)) > 0.5 if air.any() else np.ones(len(e), bool)
    if sel.sum() < 10:
        return None
    out = {}
    for i, c in enumerate(STREAMS['est']):
        v = e[sel, 1 + i]
        v = v[np.isfinite(v)]
        if len(v) and v.max() > 0:                             # an all-zero ratio = that aid is not fused
            out[c] = {'p50': r(np.median(v)), 'p95': r(np.percentile(v, 95)), 'max': r(v.max()),
                      'over_half_pct': r(100 * (v > 0.5).mean(), 1), 'rejected_pct': r(100 * (v > 1).mean(), 1)}
    return out


# ── what to set ───────────────────────────────────────────────────────────────
def recommend(res, params):
    """EKF2 params that suit the measured sources, against the FCU's current ones. Each with the reason; only the source choice
    (which aid is fused), never the noise tuning."""
    truth = res['truth']
    cur = {k: params.get(k) for k in ('EKF2_MAG_TYPE', 'EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'EKF2_OF_CTRL', 'EKF2_EV_DELAY')}
    ev = int(cur['EKF2_EV_CTRL']) if cur['EKF2_EV_CTRL'] is not None else EV_BITS['vel'] | EV_BITS['yaw']
    new, why = dict(cur), {}

    def pick(sources, names, key_truth, key_hat):
        out = {}
        for n in names:
            d = sources.get(n)
            if d:
                v = d.get(key_truth if truth else key_hat)
                v = v[1] if isinstance(v, list) else v         # stats lists: the p90
                if v is not None:
                    out[n] = v
        return out

    h = res['heading']
    hs = pick(h['sources'], ('mag', 'vision'), 'residual_deg', 'step_err_deg')
    mag_ok = (h.get('mag') or {}).get('healthy', False)
    if 'vision' in hs and ('mag' not in hs or not mag_ok or hs['vision'] <= hs['mag']):
        new['EKF2_MAG_TYPE'], ev = 5, ev | EV_BITS['yaw']
        why['EKF2_MAG_TYPE'] = f'heading from the ZED (EV yaw, {hs["vision"]:.1f} deg)' + (
            f' beats the magnetometer ({hs["mag"]:.1f} deg)' if 'mag' in hs else '') + ('' if mag_ok else ', magnetometer not healthy here')
    elif 'mag' in hs and mag_ok:
        new['EKF2_MAG_TYPE'], ev = 1, ev & ~EV_BITS['yaw']
        why['EKF2_MAG_TYPE'] = f'magnetometer ({hs["mag"]:.1f} deg) beats the ZED heading ({hs.get("vision", float("nan")):.1f} deg): ' \
                               'magnetic heading, EV yaw off (PX4 fuses only one)'

    z = res['height']
    zs = pick(z['sources'], ('baro', 'range', 'zed'), 'residual_m', 'sigma_m')
    if zs:
        best = min(zs, key=zs.get)
        rng_valid = (z['sources'].get('range') or {}).get('valid_pct') or 0
        if best == 'range' and rng_valid < 95:                 # a reference with gaps: EKF2 falls back to another one mid-flight
            best = min((k for k in zs if k != 'range'), key=zs.get, default='baro')
        new['EKF2_HGT_REF'] = {'baro': 0, 'range': 2, 'zed': 3}[best]
        ev = ev | EV_BITS['vpos'] if best == 'zed' else ev & ~EV_BITS['vpos']
        why['EKF2_HGT_REF'] = f'height reference {best} ({zs[best]:.3f} m' + ', '.join([''] + [f'{k} {v:.3f}' for k, v in zs.items() if k != best]) + ')'
        if 'range' in zs:
            new['EKF2_RNG_CTRL'] = 2 if rng_valid >= 80 else 1
            why['EKF2_RNG_CTRL'] = f'range valid {rng_valid:.0f} % of the flight' + ('' if rng_valid >= 80 else ': only low and slow')

    v = res['velocity']
    vs = pick(v['sources'], ('flow', 'zed'), 'error_mps', 'sigma_mps')
    fa = v.get('flow_axes')
    if 'zed' in vs:
        bad_zed = 'flow' in vs and vs['zed'] > 2 * vs['flow'] and vs['zed'] > 0.15
        ev = ev & ~EV_BITS['vel'] if bad_zed else ev | EV_BITS['vel']
        why['EKF2_EV_CTRL'] = f'ZED velocity {vs["zed"]:.3f} m/s' + (f' vs flow {vs["flow"]:.3f}' if 'flow' in vs else '') + \
            (': worse than twice the flow' if bad_zed else '')
    if 'flow' in vs:
        bad_flow = ('zed' in vs and vs['flow'] > 2 * vs['zed'] and vs['flow'] > 0.15) or (fa is not None and not fa['ok'])
        new['EKF2_OF_CTRL'] = 0 if bad_flow else 1
        why['EKF2_OF_CTRL'] = (f'flow axes off by {fa["rotation_deg"]} deg / scale {fa["scale"]} (fix SENS_FLOW_ROT / the sensor first)'
                               if fa is not None and not fa['ok'] else f'flow {vs["flow"]:.3f} m/s' + (': too noisy, off' if bad_flow else ': on'))
    if truth and (v.get('closure_m') or {}).get('vision') is not None:
        vc, tc = v['closure_m']['vision'], v['closure_m'].get('truth', 0.0)
        if abs(vc - tc) > 0.3:                                 # the ZED position drifted more than EKF2's would: keep it out
            ev &= ~EV_BITS['hpos']
    new['EKF2_EV_CTRL'] = ev
    why['EKF2_EV_CTRL'] = ', '.join(f'{b} {"on" if ev & v else "off"}' for b, v in EV_BITS.items()) + \
        (f' ({why["EKF2_EV_CTRL"]})' if 'EKF2_EV_CTRL' in why else '')

    lv = (res.get('latency') or {}).get('vision')
    if lv and abs(lv['lag_ms']) >= 10:
        base = cur['EKF2_EV_DELAY'] or 0.0
        new['EKF2_EV_DELAY'] = max(0, int(round(base + lv['lag_ms'])))
        why['EKF2_EV_DELAY'] = f'EV stamps {lv["lag_ms"]:+.0f} ms behind the gyro (sim: change the bridge restamp instead)'
    return [{'param': k, 'current': cur[k], 'suggest': new[k], 'change': new[k] is not None and cur[k] != new[k],
             'why': why.get(k, '')} for k in cur]


# ── display tables for the GCS (the UI renders them as they are) ────────────
def tables(res):
    """[{title, note, cols, rows}] with cells as values or {'v', 'c': ok|warn|bad}."""
    if 'error' in res:
        return []
    T = []
    fmt = lambda s: None if s is None else ' / '.join(f'{x:g}' for x in s)
    T.append({'title': 'Recording', 'note': 'samples per stream; arrival time = the stream stamps on another clock, so it is timed when '
              'it arrived (its latency then includes transport); off clock = dropped, stamps outside the run',
              'cols': ['stream', 'samples', 'timing'],
              'rows': [[k, n, 'arrival time' if k in res.get('arrival_time', []) else 'stamp'] for k, n in res['samples'].items()] +
                      [[k, 0, {'v': 'off clock', 'c': 'bad'}] for k in res.get('off_clock', [])]})
    bad = lambda v, ok: {'v': v, 'c': 'ok' if ok else 'bad'}
    h = res['heading']
    gyro_note = (f'; gyro ground change {h["gyro_ground_change_deg"]}°' if h.get('gyro_ground_change_deg') is not None else '') + \
        (f'; gyro step error vs truth {fmt(h["gyro_step_err_deg"])}°' if h.get('gyro_step_err_deg') else '')
    T.append({'title': 'Heading', 'note': f'residual against {h["ref"]} (offset = frame + declination, not an error); step error = '
              f'heading change per 45° step minus the FCU gyro\'s, over {h["steps"]} steps (sum = the drift a scan collects); '
              'ground change = heading between the two motor-off ground phases' + gyro_note,
              'cols': ['source', 'valid %', 'offset °', 'residual p50 / p90 / max °', 'step err p50 / p90 / max °', 'step sum °',
                       'ground change °', 'closure err °'],
              'rows': [[k, d.get('valid_pct'), d.get('offset_deg'), fmt(d.get('residual_deg')), fmt(d.get('step_err_deg')),
                        d.get('step_sum_deg'), d.get('ground_change_deg'), d.get('closure_err_deg')] for k, d in h['sources'].items()]})
    m = h.get('mag')
    if m:
        names = {'deviation_deg': 'max deviation (per 45° sector)', 'field_pct': 'field strength spread %',
                 'incl_deg': 'inclination spread °', 'motor_deg': 'motors on vs off (heading) °'}
        T.append({'title': 'Magnetometer health: ' + ('HEALTHY' if m['healthy'] else 'NOT HEALTHY'),
                  'note': f'field {m["field_uT"]} µT, inclination {m["incl_deg"]}°, swept {m["swept_deg"]}°. ' + m.get('note', ''),
                  'cols': ['check', 'value', 'limit'],
                  'rows': [[names.get(k, k), bad(c['value'], c['ok']), c['limit']] for k, c in m['checks'].items()] +
                          [[f'deviation at {s["hdg"]}°', s['dev'], ''] for s in m['sectors']]})
    z = res['height']
    T.append({'title': 'Height', 'note': f'reference {z["ref"]}; closure = height after landing on the take-off floor (should be 0); '
              'scale = slope against the reference over the rise step',
              'cols': ['source', 'valid %', 'residual p50 / p90 / max m', 'σ m', 'drift m/min', 'scale', 'closure m'],
              'rows': [[k, d.get('valid_pct'), fmt(d.get('residual_m')), d.get('sigma_m'), d.get('drift_m_min'), d.get('scale'),
                        d.get('closure_m')] for k, d in z['sources'].items()]})
    v = res['velocity']
    T.append({'title': 'Horizontal velocity', 'note': 'body frame; σ = three-cornered hat (flow, ZED, EKF2: EKF2 fuses the other two, '
              'so their σ are approximate)' + (f'; flow quality ok {v["flow_quality_ok_pct"]} %' if 'flow_quality_ok_pct' in v else ''),
              'cols': ['source', 'valid %', 'error vs truth p50 / p90 / max m/s', 'σ m/s', 'rms vs EKF2 m/s'],
              'rows': [[k, d.get('valid_pct'), fmt(d.get('error_mps')), d.get('sigma_mps'), d.get('vs_ekf_rms_mps')]
                       for k, d in v['sources'].items()]})
    if v.get('closure_m'):
        T.append({'title': 'Start → landing displacement', 'note': 'what each source says the drone moved between take-off and '
                  'landing; measure the real offset from the take-off mark with a tape: the source that agrees is right',
                  'cols': ['source', 'm'], 'rows': [[k, x] for k, x in v['closure_m'].items()]})
    fa = v.get('flow_axes')
    if fa:
        T.append({'title': 'Optical flow axes', 'note': 'fit of the raw flow to the body velocity: rotation ≠ 0 = SENS_FLOW_ROT is off by it',
                  'cols': ['rotation °', 'scale', 'mirrored'], 'rows': [[bad(fa['rotation_deg'], fa['ok']), fa['scale'], fa['mirrored']]]})
    im = res['imu']
    rows = []
    for k, name in (('imu_raw', 'FCU'), ('zimu', 'ZED')):
        for ph, d in (im.get(k) or {}).items():
            rows.append([name, ph, d['gyro_std_dps'], d['accel_std_mps2']])
    vib = im.get('vibration') or {}
    T.append({'title': 'IMU', 'note': 'noise std at rest (motors off) and hovering' +
              (f'; FCU − ZED yaw-rate bias at rest {im["gyro_z_bias_diff_dps"]} °/s' if im.get('gyro_z_bias_diff_dps') is not None else ''),
              'cols': ['IMU', 'phase', 'gyro σ °/s', 'accel σ m/s²'], 'rows': rows})
    if vib:
        T.append({'title': 'Vibration (PX4)', 'note': 'VIBRATION metrics (gyro coning, gyro and accel high-frequency); any accel '
                  'clipping in flight is a fault: isolate the FCU / balance the props',
                  'cols': ['phase', 'coning', 'gyro HF', 'accel HF'],
                  'rows': [[ph, d['coning'], d['gyro'], d['accel']] for ph, d in vib.items() if isinstance(d, dict)] +
                          [['clipping (new)', bad(vib['clipping'], vib['ok']), '', '']]})
    else:
        T.append({'title': 'Vibration (PX4)', 'note': 'no mavros/vibration: start MAVROS with MAVROS_PLUGINS=lean+debug', 'cols': [], 'rows': []})
    la = res['latency']
    T.append({'title': 'Latency vs FCU gyro', 'note': 'yaw rate cross-correlation in the turns; + = the source is stamped late. '
              'vision is what EKF2 gets: its lag is the EV delay error',
              'cols': ['source', 'lag ms', 'corr'],
              'rows': [[k, d['lag_ms'], d['corr']] if d else [k, '—', ''] for k, d in la.items()]})
    if res.get('lever_arm'):
        a = res['lever_arm']
        T.append({'title': 'EV lever arm (sim)', 'note': 'vision position vs truth in the turns; should equal EKF2_EV_POS',
                  'cols': ['fit FLU m', 'EKF2_EV_POS (FLU) m', 'error m'],
                  'rows': [[fmt(a['fit_flu_m']), fmt([x or 0 for x in a['config_flu_m']]), bad(a['err_m'], a['err_m'] < 0.03)]]})
    kx = res['kinetix']
    t = kx['tol']
    rows = [[k, f'{d["settled"]}/{d["n"]}', d['median'], d['max']] for k, d in kx['settle_s'].items()]
    T.append({'title': 'Kinetix capture hold', 'note': f'time from the setpoint stopping until EKF2 is inside {t["pos_m"]} m / '
              f'{t["yaw_deg"]}° / {t["speed_mps"]} m/s for {t["settle_s"]} s (what Kinetix waits for at every view)',
              'cols': ['after', 'settled', 'median s', 'max s'], 'rows': rows})
    rows = []
    if kx.get('blur_px'):
        b = kx['blur_assumed']
        rows.append([f'motion blur px (fx {b["fx"]}, exposure ≤ {b["exposure_ms"]} ms, {b["depth_m"]} m)', fmt(kx['blur_px'])])
    if kx.get('pose_err_m'):
        rows += [['EKF2 position error at capture m (truth)', fmt(kx['pose_err_m'])],
                 ['EKF2 yaw error at capture ° (truth)', fmt(kx['yaw_err_deg'])]]
    for k, x in (kx.get('room') or {}).items():
        rows.append([f'room: {k.replace("_", " ")}', x])
    T.append({'title': 'Kinetix views and room', 'note': 'p50 / p90 / max; room = the ZED bridge status during the run',
              'cols': ['', 'value'], 'rows': rows})
    inn = res.get('innovation')
    T.append({'title': 'EKF2 test ratios', 'note': 'ESTIMATOR_STATUS in flight: > 0.5 the aid fights the others, > 1 EKF2 rejects it'
              if inn else 'no ESTIMATOR_STATUS received', 'cols': ['ratio', 'p50', 'p95', 'max', '> 0.5 %', 'rejected %'],
              'rows': [[k, d['p50'], d['p95'], d['max'], d['over_half_pct'], bad(d['rejected_pct'], d['rejected_pct'] < 1)]
                       for k, d in (inn or {}).items()]})
    return T


def load(path):
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z['meta']))
    return {k: z[k] for k in z.files if k in STREAMS}, [tuple(p) for p in meta.pop('phases')], meta


if __name__ == '__main__':
    print(json.dumps(analyse(*load(sys.argv[1])), indent=1))
