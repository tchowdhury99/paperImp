#!/usr/bin/env python3
"""
Baseline evaluation — no recovery (A/B comparison for Choi et al. RAID 2020).

This script runs the SAME attack as attack_injector.py but measures the
attitude error WITHOUT the recovery monitor active.  To get a true A/B:

  A (baseline): compile firmware with recovery disabled:
        add #define RECOVERY_DISABLED at top of AP_InertialSensor.cpp, rebuild
        bash /tmp/launch_sitl_copter346.sh && bash /tmp/launch_mavproxy_346.sh
        python3 eval_baseline.py          → saves /tmp/step9_baseline.log + .npy

  B (recovery): compile firmware with recovery enabled (default), rebuild,
        restart SITL, run attack_injector.py + eval_recovery.py
        → saves /tmp/step9_recovery.log + /tmp/eval_recovery_results.npy

Eq. 7 (paper §4): R_succ = |Y_t - Y_bar_t| ≤ epsilon for all t in [1..k]
epsilon = 3°, k = 10 s (paper §4 — NOT 30 s)
"""
import time
import math
import numpy as np
from pymavlink import mavutil

CONNECTION = 'udp:127.0.0.1:14550'
EPSILON    = 3.0    # degrees (paper Eq. 7)
K_SEC      = 10.0   # paper §4: k = 10 s evaluation window
ATTACK_LOG = '/tmp/attack_timeline.log'
RESULT_NPY = '/tmp/step9_baseline_results.npy'
LOG_FILE   = '/tmp/step9_baseline.log'


def quat_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def wait_for_attack_start():
    print("Waiting for attack_injector.py to signal attack start...")
    while True:
        try:
            with open(ATTACK_LOG) as f:
                lines = {l.split('=')[0]: l.split('=')[1].strip()
                         for l in f if '=' in l}
            if 'attack_start' in lines:
                t = float(lines['attack_start'])
                print(f"Attack started at {time.strftime('%H:%M:%S', time.localtime(t))}")
                return t
        except FileNotFoundError:
            pass
        time.sleep(0.5)


def evaluate():
    mav = mavutil.mavlink_connection(CONNECTION, source_system=216)
    mav.wait_heartbeat()
    print(f"Connected. epsilon={EPSILON}° k={K_SEC}s  [BASELINE — recovery disabled]")

    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)

    t_attack = wait_for_attack_start()

    errors_roll  = []
    errors_pitch = []
    timestamps   = []
    t_start      = None
    last_des_roll  = 0.0
    last_des_pitch = 0.0

    print(f"Recording ATTITUDE errors for {K_SEC}s (NO recovery)...")
    deadline = t_attack + K_SEC + 5

    with open(LOG_FILE, 'w') as logf:
        logf.write(f"mode=baseline  epsilon={EPSILON}  k_sec={K_SEC}\n")
        logf.write(f"attack_start={t_attack}\n")

        while time.time() < deadline:
            msg = mav.recv_match(type=['ATTITUDE', 'ATTITUDE_TARGET'],
                                 blocking=True, timeout=2)
            if msg is None:
                continue

            now = time.time()
            if msg.get_type() == 'ATTITUDE_TARGET':
                last_des_roll, last_des_pitch = quat_to_euler(msg.q)
                continue

            if now < t_attack:
                continue
            if t_start is None:
                t_start = now
                print("Recording started.")

            actual_roll  = math.degrees(msg.roll)
            actual_pitch = math.degrees(msg.pitch)
            err_roll     = abs(actual_roll  - last_des_roll)
            err_pitch    = abs(actual_pitch - last_des_pitch)

            errors_roll.append(err_roll)
            errors_pitch.append(err_pitch)
            t_rel = now - t_start
            timestamps.append(t_rel)
            logf.write(f"t={t_rel:.2f} roll_err={err_roll:.3f} pitch_err={err_pitch:.3f}\n")

            if t_rel > K_SEC:
                break

    if not errors_roll:
        print("ERROR: No ATTITUDE messages received.")
        return

    max_err_roll  = max(errors_roll)
    max_err_pitch = max(errors_pitch)
    success_roll  = max_err_roll  <= EPSILON
    success_pitch = max_err_pitch <= EPSILON
    success       = success_roll and success_pitch

    print(f"\n{'='*55}")
    print(f"  Eq. 7 BASELINE (no recovery)  (epsilon={EPSILON}°, k={K_SEC}s)")
    print(f"{'='*55}")
    print(f"  Roll  : max_err={max_err_roll:6.2f}°  {'PASS' if success_roll  else 'FAIL'}")
    print(f"  Pitch : max_err={max_err_pitch:6.2f}°  {'PASS' if success_pitch else 'FAIL'}")
    print(f"  OVERALL: {'SUCCESS' if success else 'FAILURE (expected without recovery)'}")
    print(f"{'='*55}")
    print(f"  Samples: {len(errors_roll)}  Duration: {timestamps[-1]:.1f}s")
    print(f"\nExpected: FAILURE (attack unmitigated → large roll/pitch excursion)")
    print(f"This is the 'A' side of the A/B comparison.")

    np.save(RESULT_NPY, {
        'errors_roll':  np.array(errors_roll),
        'errors_pitch': np.array(errors_pitch),
        'timestamps':   np.array(timestamps),
        'epsilon': EPSILON,
        'success': success,
        'max_err_roll':  max_err_roll,
        'max_err_pitch': max_err_pitch,
        'mode': 'baseline',
    })
    print(f"Saved: {RESULT_NPY}")
    print(f"Saved: {LOG_FILE}")


if __name__ == '__main__':
    evaluate()
