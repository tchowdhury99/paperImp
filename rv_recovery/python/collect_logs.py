#!/usr/bin/env python3
"""
Mission generator + operation-data collection — paper §3.1:
"Our mission generator produces random missions systematically based on Mavlink
commands" ... "the data is collected under different maneuvers to appropriately
capture various control properties and dynamics" ... missions are "a sequence of
primitive moves like straight fly, turns, etc." (§4.2.2).

Flies N_MISSIONS random missions in GUIDED mode against a running SITL.
The vehicle disarms between missions, so each mission produces its own
Dataflash .BIN log.

Usage:
  python collect_logs.py [--missions 20] [--conn tcp:127.0.0.1:5760] [--seed 7]

Run AFTER SITL is up. Sets LOG_BITMASK with ATTITUDE_FAST (bit 0) for 25 Hz
attitude/EKF/IMU logging.
"""
import argparse, math, random, time
from pymavlink import mavutil

EARTH_M_PER_DEG = 111194.9


def set_param(mav, name, value):
    mav.mav.param_set_send(mav.target_system, mav.target_component,
                           name.encode(), float(value),
                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)


def get_position(mav, timeout=10):
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if msg is None:
        return None
    return (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0,
            msg.time_boot_ms / 1000.0)


def sim_sleep(mav, seconds):
    """Sleep in SIM time (robust under --speedup)."""
    p = get_position(mav)
    if p is None:
        time.sleep(seconds); return
    t_end = p[3] + seconds
    while True:
        p = get_position(mav)
        if p is None or p[3] >= t_end:
            return


def set_mode(mav, mode):
    # ArduCopter 3.4 does not support COMMAND_LONG DO_SET_MODE (ACK result 3);
    # use the legacy SET_MODE message instead.
    mode_id = mav.mode_mapping()[mode]
    for attempt in range(5):
        mav.mav.set_mode_send(mav.target_system,
                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                              mode_id)
        for _ in range(10):
            hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and hb.custom_mode == mode_id:
                return True
    return False


def arm_and_takeoff(mav, alt):
    if not set_mode(mav, 'GUIDED'):
        return False
    # arming may be refused until EKF/GPS ready — retry
    for attempt in range(30):
        mav.arducopter_arm()
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
        time.sleep(2)
    else:
        print("  arm failed"); return False
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                              0, 0, 0, 0, 0, 0, 0, alt)
    # wait for altitude
    t0 = time.time()
    while time.time() - t0 < 120:
        p = get_position(mav)
        if p and p[2] > alt * 0.9:
            return True
    print("  takeoff timeout"); return False


def goto(mav, lat, lon, alt, tol=3.0, timeout_s=90):
    mav.mav.set_position_target_global_int_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                  # position only
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)
    p = get_position(mav)
    t_end = (p[3] if p else 0) + timeout_s
    while True:
        p = get_position(mav)
        if p is None:
            return False
        dn = (p[0] - lat) * EARTH_M_PER_DEG
        de = (p[1] - lon) * EARTH_M_PER_DEG * math.cos(math.radians(lat))
        dz = p[2] - alt
        if math.sqrt(dn*dn + de*de + dz*dz) < tol:
            return True
        if p[3] > t_end:
            print("    goto timeout"); return True   # continue mission anyway


def yaw_to(mav, heading_deg, rate_dps=30):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                              0, heading_deg, rate_dps, 0, 0, 0, 0, 0)


def run_mission(mav, mid, rng):
    print(f"[Mission {mid}]")
    alt = rng.uniform(10, 25)
    if not arm_and_takeoff(mav, alt):
        return False
    p = get_position(mav)
    if p is None:
        return False
    lat, lon = p[0], p[1]
    n_prim = rng.randint(6, 12)
    for i in range(n_prim):
        prim = rng.choice(['straight', 'straight', 'turn', 'climb', 'hover'])
        if prim == 'straight':
            hdg = rng.uniform(0, 360)
            dist = rng.uniform(20, 80)
            lat += dist * math.cos(math.radians(hdg)) / EARTH_M_PER_DEG
            lon += dist * math.sin(math.radians(hdg)) / (EARTH_M_PER_DEG *
                    math.cos(math.radians(lat)))
            print(f"  {i+1}/{n_prim} straight {dist:.0f} m hdg {hdg:.0f}")
            goto(mav, lat, lon, alt)
        elif prim == 'turn':
            hdg = rng.uniform(0, 360)
            print(f"  {i+1}/{n_prim} turn to {hdg:.0f} deg")
            yaw_to(mav, hdg)
            sim_sleep(mav, 6)
        elif prim == 'climb':
            alt = rng.uniform(8, 30)
            print(f"  {i+1}/{n_prim} alt -> {alt:.0f} m")
            goto(mav, lat, lon, alt)
        else:
            d = rng.uniform(3, 8)
            print(f"  {i+1}/{n_prim} hover {d:.0f} s")
            sim_sleep(mav, d)
    print("  landing")
    set_mode(mav, 'LAND')
    mav.motors_disarmed_wait()
    time.sleep(2)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--missions', type=int, default=20)
    ap.add_argument('--conn', default='tcp:127.0.0.1:5760')
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    print(f"Connecting to {args.conn} ...")
    mav = mavutil.mavlink_connection(args.conn)
    mav.wait_heartbeat(timeout=120)
    print(f"Heartbeat from sys {mav.target_system}")

    # direct TCP link (no MAVProxy): ask the autopilot to stream telemetry
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # high-rate logging (paper: "data is collected at a high sampling rate")
    set_param(mav, 'LOG_BITMASK', 131071)
    # guided arming needs the EKF to be consuming GPS, not just a GPS fix
    print("Waiting for EKF to use GPS ...")
    t0 = time.time()
    while time.time() - t0 < 300:
        msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
        if msg and b'using GPS' in (msg.text if isinstance(msg.text, bytes)
                                    else msg.text.encode()):
            print("EKF using GPS")
            break
    sim_sleep(mav, 10)

    done = 0
    for mid in range(1, args.missions + 1):
        if run_mission(mav, mid, rng):
            done += 1
    print(f"\n{done}/{args.missions} missions complete. "
          f"Logs are in the SITL working directory (logs/).")


if __name__ == '__main__':
    main()
