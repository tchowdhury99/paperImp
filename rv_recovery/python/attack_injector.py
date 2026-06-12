#!/usr/bin/env python3
"""
Attack injector for the Choi et al. RAID 2020 replication.
Simulates a gyro spoofing attack via SIM_GYRO_BIAS_X/Y/Z parameters.
This injects a bias at the SITL simulation level, which propagates through
the sensor driver into AP_InertialSensor — exactly what the recovery monitor defends against.

Usage: run AFTER SITL+MAVProxy are up and vehicle is flying.
"""
import time
from pymavlink import mavutil

CONNECTION    = 'udp:127.0.0.1:14551'
ATTACK_BIAS   = 2.0    # rad/s bias injected on GyrX — paper §4 gyro-spoofing scenario
ATTACK_DELAY  = 15.0   # seconds after connection to stabilize before injecting
ATTACK_HOLD   = 15.0   # hold attack for 15 s so the k=10 s eval window falls inside it
LOG_FILE      = '/tmp/attack_timeline.log'


def set_param(mav, name, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode('utf-8'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.2)
    print(f"  param set {name} = {value}")


def main():
    print(f"Connecting to {CONNECTION}...")
    mav = mavutil.mavlink_connection(CONNECTION)
    mav.wait_heartbeat()
    print(f"Connected. Waiting {ATTACK_DELAY}s for vehicle to stabilize...")
    time.sleep(ATTACK_DELAY)

    # Check vehicle is in the air
    pos = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if pos:
        alt = pos.relative_alt / 1000.0
        print(f"Current altitude: {alt:.1f} m")
        if alt < 3.0:
            print("WARNING: Vehicle appears to be on ground — attack may not be meaningful")

    t_attack_start = time.time()
    print(f"\n[{time.strftime('%H:%M:%S')}] INJECTING ATTACK: SIM_GYRO_BIAS_X = {ATTACK_BIAS} rad/s")
    set_param(mav, 'SIM_GYRO_BIAS_X', ATTACK_BIAS)
    set_param(mav, 'SIM_GYRO_BIAS_Y', ATTACK_BIAS * 0.3)  # smaller bias on Y

    with open(LOG_FILE, 'w') as f:
        f.write(f"attack_start={t_attack_start}\n")
        f.write(f"bias_x={ATTACK_BIAS}\n")
        f.write(f"bias_y={ATTACK_BIAS*0.3}\n")

    print(f"Attack active for {ATTACK_HOLD}s...")
    time.sleep(ATTACK_HOLD)

    t_attack_end = time.time()
    print(f"\n[{time.strftime('%H:%M:%S')}] CLEARING ATTACK")
    set_param(mav, 'SIM_GYRO_BIAS_X', 0.0)
    set_param(mav, 'SIM_GYRO_BIAS_Y', 0.0)

    with open(LOG_FILE, 'a') as f:
        f.write(f"attack_end={t_attack_end}\n")

    print("Attack cleared. Check eval_recovery.py results.")


if __name__ == '__main__':
    main()
