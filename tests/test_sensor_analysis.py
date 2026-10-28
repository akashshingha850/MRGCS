#!/usr/bin/env python3
"""sensor_analysis.py on a synthetic analyse flight with known errors: the analysis must find them.
    python3 tests/test_sensor_analysis.py      (or pytest)"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'onboard'))
import sensor_analysis as sa  # noqa: E402

SLIP_DEG, LAG_S, ARM_M = 2.0, 0.05, 0.12        # ZED heading slip per turn, vision stamp delay, camera lever arm (FLU x)


def flight(seed=0):
    rng = np.random.default_rng(seed)
    dt = 0.01
    sp, phases, t = [0.0, 0.0, 0.0, 0.0], [], 0.0
    track = []                                    # (t, sp x y z yaw)

    def step(name, dur, target=None, info=None):
        nonlocal t
        t0, n = t, int(round(dur / dt))
        start = list(sp)
        for i in range(n):
            if target is not None:
                f = (i + 1) / n
                for j in range(4):
                    sp[j] = start[j] + f * (target[j] - start[j])
            track.append([t] + list(sp))
            t += dt
        phases.append((t0, t, name, info))

    step('ground', 5)
    step('climb', 3, [0, 0, 1.5, 0])
    step('hover', 5)
    for k in list(range(1, 9)) + list(range(7, -1, -1)):
        step('turn', 2.25, [sp[0], sp[1], sp[2], math.radians(45 * k)])
        step('hold', 4, info={'kind': 'yaw', 'sp': list(sp)})
    for dz in (0.6, 0.0):
        step('rise', 1.2, [sp[0], sp[1], 1.5 + dz, sp[3]])
        step('hold', 4, info={'kind': 'height', 'sp': list(sp)})
    for x, y in ((1, 0), (1, 1), (0, 1), (0, 0)):
        step('move', 3.3, [x, y, sp[2], sp[3]])
        step('hold', 4, info={'kind': 'move', 'sp': list(sp)})
    step('land', 3, [0, 0, 0, sp[3]])
    step('after', 5)

    tr = np.array(track)
    tt = tr[:, 0]
    a = dt / 0.4                                  # the drone follows the setpoint with a 0.4 s lag
    p = np.zeros((len(tr), 4))
    for i in range(1, len(tr)):
        p[i] = p[i - 1] + a * (tr[i, 1:] - p[i - 1])
    x, y, z, yaw = p.T
    v = np.gradient(p, dt, axis=0)
    vbx, vby = sa.to_body(v[:, 0], v[:, 1], yaw)
    q = np.stack([0 * yaw, 0 * yaw, np.sin(yaw / 2), np.cos(yaw / 2)], 1)
    n = lambda s: rng.normal(0, s, len(tt))
    turns = np.cumsum(np.r_[0, np.abs(np.diff(np.round(np.degrees(np.unwrap(tr[:, 4])) / 45)))])  # slip once per 45 deg setpoint step
    zed_yaw = yaw + np.radians(SLIP_DEG) * turns + np.radians(n(0.1))
    sel = lambda hz: slice(None, None, int(round(1 / (hz * dt))))

    rec = {}
    rec['att'] = np.column_stack([tt, q, 0 * tt, 0 * tt, v[:, 3]])[sel(50)]
    rec['truth'] = np.column_stack([tt, x, y, z, yaw])[sel(50)]
    rec['ekf'] = np.column_stack([tt, x + n(0.005), y + n(0.005), z + n(0.005), yaw])[sel(30)]
    rec['ekf_vb'] = np.column_stack([tt, vbx + n(0.01), vby + n(0.01), v[:, 2]])[sel(30)]
    rec['zed'] = np.column_stack([tt, x + n(0.01), y + n(0.01), z + n(0.02), zed_yaw])[sel(30)]
    cx, cy = x + ARM_M * np.cos(yaw), y + ARM_M * np.sin(yaw)      # the camera, ahead of the body
    k = int(round(LAG_S / dt))                    # vision stamped LAG_S late: the value at t is the motion at t - LAG_S
    late = lambda s: np.r_[np.full(k, s[0]), s[:-k]]
    rec['vision'] = np.column_stack([tt, late(cx), late(cy), late(z), late(zed_yaw)])[sel(30)]
    F = np.array([0.0, 20e-6, -40e-6])            # Earth field (ENU)
    c, s = np.cos(yaw), np.sin(yaw)
    mb = np.stack([c * F[0] + s * F[1], -s * F[0] + c * F[1], np.full_like(yaw, F[2])], 1) + rng.normal(0, 0.1e-6, (len(tt), 3))
    rec['mag'] = np.column_stack([tt, mb])[sel(50)]
    rec['baro'] = np.column_stack([tt, 101325 * (1 - (z + n(0.08)) / 44330.77) ** (1 / 0.190263)])[sel(20)]
    rg = np.where(z > 0.3, z + 0.06 + n(0.01), np.nan)
    rec['range'] = np.column_stack([tt, rg])[sel(20)]
    d = z + 0.06
    fdt = 0.1
    wx, wy = vby / d, -vbx / d                    # PX4 flow in MAVROS FLU: (vx, vy) = (-wy, wx) * distance
    rec['flow'] = np.column_stack([tt, wx * fdt, wy * fdt, 0 * tt, 0 * tt, np.full_like(tt, fdt), np.where(z > 0.3, 200, 0)])[sel(10)]
    rec['zimu'] = np.column_stack([tt, n(0.002), n(0.002), v[:, 3] + n(0.002), n(0.05), n(0.05), 9.81 + n(0.05)])[sel(100)]
    rec['imu_raw'] = np.column_stack([tt, n(0.1), n(0.1), 9.81 + n(0.1), n(0.003), n(0.003), v[:, 3] + n(0.003)])[sel(50)]
    rec['vib'] = np.column_stack([tt, 0 * tt + 1e-4, 0 * tt + 2e-3, 0 * tt + 0.3, 0 * tt, 0 * tt, 0 * tt])[sel(10)]
    rec['est'] = np.column_stack([tt, np.abs(n(0.1)), 0 * tt, np.abs(n(0.2)), 0 * tt, np.abs(n(0.1))])[sel(10)]
    meta = {'params': {'EKF2_MAG_TYPE': 5, 'EKF2_EV_CTRL': 12, 'EKF2_HGT_REF': 0, 'EKF2_RNG_CTRL': 2, 'EKF2_OF_CTRL': 1,
                       'EKF2_EV_DELAY': 0.0, 'EKF2_EV_POS_X': ARM_M, 'EKF2_EV_POS_Y': 0.0},
            'cam': {'fx': 520.0, 'fps': 30.0}}
    return rec, phases, meta


def test_flight():
    rec, phases, meta = flight()
    res = sa.analyse(rec, phases, meta)
    h = res['heading']
    assert h['steps'] == 16
    assert abs(h['sources']['zed']['step_err_deg'][0] - SLIP_DEG) < 0.3, h['sources']['zed']
    assert h['sources']['mag']['step_err_deg'][1] < 0.5, h['sources']['mag']
    assert h['gyro_step_err_deg'][1] < 0.2, h
    assert h['mag']['healthy'], h['mag']
    la = res['latency']
    assert abs(la['vision']['lag_ms'] - LAG_S * 1000) < 8, la
    assert abs(la['zed']['lag_ms']) < 8 and abs(la['truth']['lag_ms']) < 8, la
    arm = res['lever_arm']
    assert abs(arm['fit_flu_m'][0] - ARM_M) < 0.02 and abs(arm['fit_flu_m'][1]) < 0.02, arm
    fa = res['velocity']['flow_axes']
    assert fa['ok'] and abs(fa['rotation_deg']) < 3 and abs(fa['scale'] - 1) < 0.05, fa
    z = res['height']['sources']
    assert abs(z['baro']['sigma_m'] - 0.08) < 0.02 and z['range']['sigma_m'] < 0.03, z
    assert abs(z['ekf']['closure_m']) < 0.02, z
    kx = res['kinetix']
    assert all(d['settled'] == d['n'] for d in kx['settle_s'].values()), kx
    assert kx['pose_err_m'][1] < 0.05 and kx['blur_px'] is not None, kx
    assert res['imu']['vibration']['ok']
    rec_ = {d['param']: d for d in res['recommend']}
    assert rec_['EKF2_MAG_TYPE']['suggest'] == 1, rec_       # the healthy magnetometer beats a slipping ZED heading
    assert rec_['EKF2_EV_CTRL']['suggest'] & 8 == 0, rec_
    assert rec_['EKF2_HGT_REF']['suggest'] == 2, rec_        # range is the least noisy height
    assert rec_['EKF2_EV_DELAY']['suggest'] == 50, rec_
    tables = sa.tables(res)
    assert tables and all({'title', 'cols', 'rows'} <= set(t) for t in tables)


def test_no_truth_hand():
    rec, phases, meta = flight(1)
    rec.pop('truth')
    rec['zed'] = rec['zed'].copy()
    rec['zed'][:, 0] += 1.79e9                    # stamped on another clock: dropped, not a grid over decades
    res = sa.analyse(rec, phases, meta)
    assert res['off_clock'] == ['zed'] and 'zed' not in res['heading']['sources']
    rec, phases, meta = flight(1)
    rec.pop('truth')
    res = sa.analyse(rec, phases, meta)
    assert abs(res['heading']['sources']['zed']['step_err_deg'][0] - SLIP_DEG) < 0.3   # truth-free: the gyro reference
    assert res['lever_arm'] is None and 'pose_err_m' not in res['kinetix']
    hand = [(phases[0][0], phases[-1][1], 'hand', None)]
    res = sa.analyse(rec, hand, meta)
    assert res['kind'] == 'hand' and sa.tables(res)


if __name__ == '__main__':
    test_flight()
    test_no_truth_hand()
    print('ok')
