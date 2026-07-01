#!/usr/bin/env python3
"""
mavlink_source.py  —  streams operation_data_50hz.mat (segment 0) as MAVLink traffic
for the ONLINE paper-faithful detector, with a SUSTAINED attack injected on the wire.

Transport: HEARTBEAT + NAMED_VALUE_FLOAT (name<=10 chars):
  U0..U6   control input u(t)      (drives y=Cx+Du, x=Ax+Bu)
  Y0..Y11  measurement vector m(t) (the attacked channel is corrupted on the wire)
  TICK     sample index (alignment marker; -1 = end)

Attacks (channel index / kind / value):
  baro -> ch2 alt  bias +3.0 m      gyro -> ch9 p set 0.8 rad/s      gps -> ch1 pE bias +20 m
Attack is SUSTAINED from --start to end of segment (faithful thresholds need a sustained attack).

Run:  python3 mavlink_source.py --attack gyro --rate 1500
"""
import argparse, time
import numpy as np
from pymavlink import mavutil
import faithful_core as fc

CASE_CH = {'baro': (2, 'bias', 3.0), 'gyro': (9, 'set', 0.8), 'gps': (1, 'bias', 20.0), 'none': (None, None, 0.0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='udpout:127.0.0.1:14580')
    ap.add_argument('--attack', choices=list(CASE_CH.keys()), default='gyro')
    ap.add_argument('--start', type=int, default=2000)     # 40.0 s
    ap.add_argument('--rate', type=float, default=1500.0)  # ticks/s on the wire (>50 = faster than real time)
    args = ap.parse_args()

    Y, U, EX = fc.load_segment(0)
    N = Y.shape[0]
    k, kind, val = CASE_CH[args.attack]

    m = mavutil.mavlink_connection(args.out, source_system=1, source_component=1)
    print(f"[source] {args.out}  attack={args.attack} (sustained from {args.start*fc.TS:.1f}s)  N={N}  rate={args.rate}")

    period = 1.0 / args.rate
    t0 = time.time(); last_hb = 0.0
    for n in range(N):
        now = time.time()
        if now - last_hb > 0.5:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0,
                                 mavutil.mavlink.MAV_STATE_ACTIVE)
            last_hb = now
        yv = Y[n].copy()
        if k is not None and n >= args.start:              # sustained attack on the wire
            if kind == 'bias':
                yv[k] += val
            else:
                yv[k] = val
        tb = int((now - t0) * 1000)
        for i in range(7):
            m.mav.named_value_float_send(tb, f'U{i}'.encode(), float(U[n, i]))
        for i in range(12):
            m.mav.named_value_float_send(tb, f'Y{i}'.encode(), float(yv[i]))
        m.mav.named_value_float_send(tb, b'TICK', float(n))
        tgt = t0 + (n + 1) * period
        s = tgt - time.time()
        if s > 0:
            time.sleep(s)
    for _ in range(5):
        m.mav.named_value_float_send(int((time.time()-t0)*1000), b'TICK', float(-1))
        time.sleep(0.05)
    print(f"[source] done in {time.time()-t0:.1f}s")


if __name__ == '__main__':
    main()
