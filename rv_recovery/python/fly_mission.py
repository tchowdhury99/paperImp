#!/usr/bin/env python3
"""
Fly one clean GUIDED square mission and count RECOVERY activations (false positives,
paper §4.2.2). Also usable as the takeoff+hover driver for attack case studies.

Usage: fly_mission.py [--conn tcp:127.0.0.1:5760] [--alt 15] [--hover N]
"""
import argparse, math, time, sys
from pymavlink import mavutil

EARTH = 111194.9


def wait_ekf_gps(mav, timeout=120):
    """Wait until the EKF reports a usable horizontal position estimate.
    Polls EKF_STATUS_REPORT flags (robust to connecting after the one-shot
    'using GPS' STATUSTEXT has already been sent)."""
    EKF_POS_HORIZ_ABS = (1 << 4)
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
        if m and (m.flags & EKF_POS_HORIZ_ABS):
            return True
    return False


LAST_DIAG = {}
def collect_recovery(mav, msgs):
    """Drain any pending messages, recording RECOVERY activations + DIAG peaks."""
    while True:
        m = mav.recv_match(type='STATUSTEXT', blocking=False)
        if m is None:
            break
        txt = m.text if isinstance(m.text, str) else m.text.decode(errors='ignore')
        if 'RECOVERY' in txt:
            msgs.append(txt)
        elif txt.startswith('DIAG'):
            LAST_DIAG[txt.split()[1]] = txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='tcp:127.0.0.1:5760')
    ap.add_argument('--alt', type=float, default=15.0)
    ap.add_argument('--hover', type=float, default=0.0)
    args = ap.parse_args()

    mav = mavutil.mavlink_connection(args.conn)
    mav.wait_heartbeat(timeout=120)
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    # SITL automation: bypass pre-arm checks (standard for autotest); the
    # recovery evaluation is about sensor attacks, not pre-arm gating.
    mav.mav.param_set_send(mav.target_system, mav.target_component,
                           b'ARMING_CHECK', 0.0,
                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.5)
    print("waiting for EKF/GPS ...")
    wait_ekf_gps(mav)
    print("EKF ready")

    msgs = []
    mode_id = mav.mode_mapping()['GUIDED']
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    time.sleep(0.5)
    armed = False
    for _ in range(30):                      # proven collect_logs.py arming
        mav.arducopter_arm()
        m = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if m and (m.base_mode & 128):
            print("armed"); armed = True; break
        time.sleep(2)
    if not armed:
        print("ARM FAILED"); sys.exit(1)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, args.alt)
    p = None
    t0 = time.time()
    last_alt = 0.0
    while time.time() - t0 < 90:
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        collect_recovery(mav, msgs)
        if m:
            last_alt = m.relative_alt / 1000.0
            if m.relative_alt > args.alt * 900:
                p = (m.lat/1e7, m.lon/1e7); break
    if p is None:
        print(f"TAKEOFF FAILED  (last_alt={last_alt:.1f}m, recovery activations: {len(msgs)})")
        for r in msgs[:8]: print("  ", r)
        sys.exit(1)
    print(f"took off to {last_alt:.1f} m")

    if args.hover > 0:
        t0 = time.time()
        while time.time() - t0 < args.hover:
            mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            collect_recovery(mav, msgs)

    lat0, lon0 = p
    for dlat, dlon in [(0.0003,0),(0.0003,0.0003),(0,0.0003),(0,0)]:
        lat, lon = lat0+dlat, lon0+dlon
        mav.mav.set_position_target_global_int_send(
            0, mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
            int(lat*1e7), int(lon*1e7), args.alt, 0,0,0, 0,0,0, 0,0)
        tw = time.time()
        while time.time() - tw < 30:
            m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            collect_recovery(mav, msgs)
            if m:
                dn = (m.lat/1e7-lat)*EARTH
                de = (m.lon/1e7-lon)*EARTH*math.cos(math.radians(lat))
                if math.hypot(dn, de) < 4:
                    break

    print(f"\n=== {len(msgs)} RECOVERY activations during clean flight ===")
    for r in msgs[:12]:
        print("  ", r)
    print("FP CHECK:", "PASS (0 activations)" if not msgs else f"FAIL ({len(msgs)})")
    if LAST_DIAG:
        print("\n=== DIAG peak per-window residuals (latest) ===")
        for k in sorted(LAST_DIAG):
            print("  ", LAST_DIAG[k])


if __name__ == '__main__':
    main()
