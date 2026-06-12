#!/usr/bin/env python3
"""
Gyroscope attack case study — paper §4.3 / Figure 17, evaluated with Eq. (7).

Procedure:
  1. Take off, hover at a fixed altitude (the §4.3 scenario starts from hover).
  2. Trigger the firmware §4.1 attack module via RC ch7 override: inject a constant
     value into the gyroscope roll-rate measurement.
  3. Record roll vs. desired roll for k seconds and apply the Eq. (7) success
     criterion:  |Y_t - Ybar_t| <= eps  for all t in [1..k]   (eps=3 deg, k=10 s).

Run against EITHER the recovery-on (B-side) or recovery-off (A-side) binary to get
the paper's A/B comparison (Figure 17a vs 17b).

Usage: case_study_gyro.py [--conn tcp:127.0.0.1:5760] [--label on|off]
"""
import argparse, math, time, sys
from pymavlink import mavutil

EPS = 3.0      # degrees (Eq. 7 example)
K_SEC = 10.0   # seconds (Eq. 7 example)
ATTACK_PWM = 1800   # ch7: (1800-1500)/500*2.0 = +1.2 rad/s constant roll-rate inject


def set_param(mav, name, val):
    mav.mav.param_set_send(mav.target_system, mav.target_component,
                           name.encode(), float(val),
                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)


def wait_ekf(mav, timeout=120):
    EKF_POS_HORIZ_ABS = (1 << 4)
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
        if m and (m.flags & EKF_POS_HORIZ_ABS):
            return True
    return False


def set_mode(mav, mode):
    mid = mav.mode_mapping()[mode]
    for _ in range(5):
        mav.mav.set_mode_send(mav.target_system,
                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)
        for _ in range(10):
            hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and hb.custom_mode == mid:
                return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='tcp:127.0.0.1:5760')
    ap.add_argument('--label', default='?')
    ap.add_argument('--alt', type=float, default=20.0)
    args = ap.parse_args()

    mav = mavutil.mavlink_connection(args.conn)
    mav.wait_heartbeat(timeout=120)
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
    set_param(mav, 'ARMING_CHECK', 0)
    # CRITICAL: default SITL uses AHRS_EKF_TYPE=10 (EKF_TYPE_SITL) which takes
    # attitude/gyro straight from the perfect simulator FDM, bypassing the real
    # INS. The §4.1 attack and the recovery substitution both act on the INS, so
    # the loop must use a real estimator (EKF2) that consumes the INS.
    set_param(mav, 'AHRS_EKF_TYPE', 2)
    print("waiting for EKF ...")
    wait_ekf(mav)
    set_mode(mav, 'GUIDED')

    armed = False
    for _ in range(30):
        mav.arducopter_arm()
        m = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if m and (m.base_mode & 128):
            armed = True; break
        time.sleep(2)
    if not armed:
        print("ARM FAILED"); sys.exit(1)
    print("armed; taking off")

    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, args.alt)
    t0 = time.time()
    while time.time() - t0 < 60:
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if m and m.relative_alt > args.alt * 900:
            break
    print("hovering; stabilizing 8 s")
    t0 = time.time()
    while time.time() - t0 < 8:
        mav.recv_match(type='ATTITUDE', blocking=True, timeout=2)

    # ── trigger the §4.1 gyro attack via RC ch7 override ─────────────────────
    print(f"[{args.label}] INJECTING gyro roll-rate attack (ch7={ATTACK_PWM})")
    t_attack = time.time()
    errs = []
    crashed = False
    attack_seen = 0
    while time.time() - t_attack < K_SEC:
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                          0,0,0,0,0,0, ATTACK_PWM, 0)
        m = mav.recv_match(type=['ATTITUDE', 'STATUSTEXT'], blocking=True, timeout=2)
        if m is None:
            continue
        if m.get_type() == 'STATUSTEXT':
            txt = m.text if isinstance(m.text, str) else m.text.decode(errors='ignore')
            if 'ATTACK' in txt:
                attack_seen += 1
            if 'Crash' in txt or 'crash' in txt:
                crashed = True
            continue
        roll = math.degrees(m.roll)
        err = abs(roll - 0.0)         # Ybar = 0 deg (hover target attitude)
        errs.append(err)
    print(f"  (attack confirmations seen: {attack_seen})")

    # clear attack
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                      0,0,0,0,0,0, 1500, 0)

    if not errs:
        print("no ATTITUDE samples"); sys.exit(1)
    max_err = max(errs)
    success = (max_err <= EPS) and not crashed
    print(f"\n{'='*56}")
    print(f"  Gyro attack case study — recovery {args.label.upper()}")
    print(f"  Eq.7:  |roll - 0| <= {EPS} deg for {K_SEC:.0f} s")
    print(f"  max attitude error = {max_err:.2f} deg   crashed={crashed}")
    print(f"  R_succ = {'SUCCESS' if success else 'FAILURE'}")
    print(f"{'='*56}")


if __name__ == '__main__':
    main()
