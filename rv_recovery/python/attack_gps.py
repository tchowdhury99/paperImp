#!/usr/bin/env python3
"""
GPS spoofing attack scripts — Choi et al. RAID 2020, Section 4.3.

Two attack scenarios from the paper:
  1. 20 m sudden offset attack: GPS position instantly jumps 20 m north.
     Triggers a strong EKF position divergence.
  2. Stealthy controlled carry-off: GPS position drifts slowly (0.5 m/s)
     to quietly move the vehicle away from its intended hover point.

SITL GPS injection mechanism:
  - SIM_GPS_POS_ERR_N/E add a constant North/East offset (metres) to the
    simulated GPS reading.  The firmware receives the corrupted position.
  - SIM_GPS_POS_DRIFT (if available) allows a slow ramp; otherwise we
    approximate by updating the offset in small increments.

Usage:
    # Scenario 1 — sudden 20 m offset:
    python3 attack_gps.py --scenario offset

    # Scenario 2 — stealthy carry-off:
    python3 attack_gps.py --scenario drift

Run AFTER SITL+MAVProxy are up and vehicle is hovering.
The eval script (eval_recovery.py or eval_gps.py) should run concurrently.
"""
import argparse
import time
import math
from pymavlink import mavutil

CONNECTION   = 'udp:127.0.0.1:14551'
ATTACK_DELAY = 15.0   # seconds to wait for stable hover before injecting
LOG_FILE     = '/tmp/attack_timeline.log'


def set_param(mav, name, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.2)
    print(f"  set {name} = {value:.3f}")


def attack_offset(mav):
    """Section 4.3 scenario: sudden 20 m GPS position offset.

    The paper tests a 20 m north offset that persists for the evaluation
    window.  Expected: without recovery, the vehicle tries to fly 20 m north;
    with recovery, the GPS software sensor (dead-reckoning) detects the jump
    and the flight controller uses the unattacked position estimate.
    """
    print(f"\n[GPS Attack] Scenario 1: 20 m sudden offset (Section 4.3)")
    print(f"  SIM_GPS_POS_ERR_N = 20 m at t=0")

    t_start = time.time()
    set_param(mav, 'SIM_GPS_POS_ERR_N', 20.0)   # 20 m north offset
    set_param(mav, 'SIM_GPS_POS_ERR_E',  0.0)

    with open(LOG_FILE, 'w') as f:
        f.write(f"attack_start={t_start}\n")
        f.write(f"scenario=offset\n")
        f.write(f"offset_n=20.0\n")
        f.write(f"offset_e=0.0\n")

    print(f"  Attack active for 15 s ...")
    time.sleep(15.0)

    print(f"  Clearing GPS offset")
    set_param(mav, 'SIM_GPS_POS_ERR_N', 0.0)
    set_param(mav, 'SIM_GPS_POS_ERR_E', 0.0)

    with open(LOG_FILE, 'a') as f:
        f.write(f"attack_end={time.time()}\n")


def attack_drift(mav):
    """Section 4.3 scenario: stealthy controlled carry-off.

    The GPS position drifts at 0.5 m/s northward.  The vehicle slowly
    follows the fake GPS position rather than staying at its hover point.
    The drift is kept small enough to avoid triggering sudden detection.
    """
    print(f"\n[GPS Attack] Scenario 2: stealthy carry-off (0.5 m/s drift, Section 4.3)")
    DRIFT_RATE = 0.5    # m/s northward
    DRIFT_MAX  = 10.0   # maximum offset (m) before clearing
    STEP_SEC   = 1.0    # update interval

    t_start = time.time()
    with open(LOG_FILE, 'w') as f:
        f.write(f"attack_start={t_start}\n")
        f.write(f"scenario=drift\n")
        f.write(f"drift_rate_mps={DRIFT_RATE}\n")

    offset_n = 0.0
    while offset_n < DRIFT_MAX:
        offset_n += DRIFT_RATE * STEP_SEC
        set_param(mav, 'SIM_GPS_POS_ERR_N', offset_n)
        print(f"  GPS north offset: {offset_n:.1f} m  (t={time.time()-t_start:.1f}s)")
        time.sleep(STEP_SEC)

    print(f"  Maximum drift reached ({DRIFT_MAX} m). Clearing.")
    set_param(mav, 'SIM_GPS_POS_ERR_N', 0.0)

    with open(LOG_FILE, 'a') as f:
        f.write(f"attack_end={time.time()}\n")
        f.write(f"max_offset_n={DRIFT_MAX}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['offset', 'drift'], default='offset',
                        help='offset = 20 m sudden; drift = stealthy carry-off')
    args = parser.parse_args()

    print(f"Connecting to {CONNECTION} ...")
    mav = mavutil.mavlink_connection(CONNECTION)
    mav.wait_heartbeat()
    print(f"Connected.  Waiting {ATTACK_DELAY}s for stable hover ...")
    time.sleep(ATTACK_DELAY)

    if args.scenario == 'offset':
        attack_offset(mav)
    else:
        attack_drift(mav)


if __name__ == '__main__':
    main()
