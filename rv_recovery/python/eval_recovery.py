#!/usr/bin/env python3
"""
Implements Eq. (7) from Choi et al. RAID 2020:
    R_succ := |Y_t - Y_bar_t| <= epsilon,  for all t in [1..k]

Y_t     = actual Roll/Pitch angle at time t  (MAVLink ATTITUDE.roll/pitch)
Y_bar_t = desired Roll/Pitch (from ATTITUDE_TARGET quaternion; at hover ≈ 0)
epsilon = 3 degrees (paper Section 4, Eq. 7 example)
k       = 10 seconds of evaluation window (paper Eq. 7 example)

Runs concurrently with attack_injector.py.
Uses MAVLink ATTITUDE messages (not Dataflash ATT).
"""
import time
import math
import numpy as np
from pymavlink import mavutil

CONNECTION = 'udp:127.0.0.1:14550'
EPSILON    = 3.0    # degrees (paper Eq. 7)
K_SEC      = 10.0   # evaluation window — paper §4 uses k=10 s
ATTACK_LOG = '/tmp/attack_timeline.log'


def quat_to_euler(q):
    """Convert quaternion [w,x,y,z] to (roll, pitch) in degrees."""
    w, x, y, z = q
    # Roll (x-axis rotation)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2*(w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def wait_for_attack_start():
    """Poll the attack timeline log until attack_start is recorded."""
    print("Waiting for attack to start (watching /tmp/attack_timeline.log)...")
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


def evaluate(connection=CONNECTION, epsilon=EPSILON, k_sec=K_SEC):
    mav = mavutil.mavlink_connection(connection, source_system=216)
    mav.wait_heartbeat()
    print(f"Connected. epsilon={epsilon}° k={k_sec}s")

    # Request ATTITUDE stream at 10Hz
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
    t_start = None
    last_des_roll = 0.0
    last_des_pitch = 0.0

    print(f"Recording ATTITUDE errors for {k_sec}s...")
    t0 = time.time()
    deadline = t_attack + k_sec + 5  # absolute wall-clock deadline

    while time.time() < deadline:
        msg = mav.recv_match(type=['ATTITUDE', 'ATTITUDE_TARGET'],
                             blocking=True, timeout=2)
        if msg is None:
            print(f"  timeout at {time.time()-t0:.1f}s")
            continue

        now = time.time()

        if msg.get_type() == 'ATTITUDE_TARGET':
            # Update desired attitude from quaternion [w,x,y,z]
            last_des_roll, last_des_pitch = quat_to_euler(msg.q)
            continue

        # ATTITUDE message — actual roll/pitch in radians
        if now < t_attack:
            continue
        if t_start is None:
            t_start = now
            print("Recording started.")

        actual_roll  = math.degrees(msg.roll)
        actual_pitch = math.degrees(msg.pitch)

        err_roll  = abs(actual_roll  - last_des_roll)
        err_pitch = abs(actual_pitch - last_des_pitch)
        errors_roll.append(err_roll)
        errors_pitch.append(err_pitch)
        timestamps.append(now - t_start)

        if now - t_start > k_sec:
            break

    if not errors_roll:
        print("ERROR: No ATTITUDE messages received.")
        return False

    max_err_roll  = max(errors_roll)
    max_err_pitch = max(errors_pitch)
    mean_err_roll = np.mean(errors_roll)

    success_roll  = max_err_roll  <= epsilon
    success_pitch = max_err_pitch <= epsilon
    success       = success_roll and success_pitch

    print(f"\n{'='*55}")
    print(f"  Eq. 7 Recovery Evaluation  (epsilon={epsilon}°, k={k_sec}s)")
    print(f"{'='*55}")
    print(f"  Roll  : max_err={max_err_roll:6.2f}°  mean={mean_err_roll:5.2f}°  "
          f"{'PASS' if success_roll  else 'FAIL'}")
    print(f"  Pitch : max_err={max_err_pitch:6.2f}°  "
          f"{'PASS' if success_pitch else 'FAIL'}")
    print(f"  OVERALL: {'SUCCESS' if success else 'FAILURE'}")
    print(f"{'='*55}")
    print(f"  Samples: {len(errors_roll)}  Duration: {timestamps[-1]:.1f}s")

    # Save results
    np.save('/tmp/eval_recovery_results.npy', {
        'errors_roll': np.array(errors_roll),
        'errors_pitch': np.array(errors_pitch),
        'timestamps': np.array(timestamps),
        'epsilon': epsilon,
        'success': success,
        'max_err_roll': max_err_roll,
        'max_err_pitch': max_err_pitch,
    })
    print("\nSaved: /tmp/eval_recovery_results.npy")
    return success


if __name__ == '__main__':
    evaluate()
