#!/usr/bin/env python3
"""
mavlink_replay_source.py
========================
Streams the recorded 50 Hz flight dataset (operation_data_50hz.mat, segment 0) out over
a real MAVLink UDP link, optionally injecting a sensor attack on the wire, so that
`online_stl_mavlink.py` can monitor it in real time exactly as it would monitor a live
ArduPilot/PX4 SITL flight.

Why this exists
---------------
The paper's runtime recovery monitor (Choi et al., RAID 2020, Algorithm 1) is meant to run
ONLINE, reading sensors over MAVLink. A full SITL flight is non-deterministic and heavy; this
bridge replays REAL recorded SITL flight data (the same operation_data_50hz.mat used for the
offline STL work and for system identification) as genuine MAVLink traffic. The monitor's
receive path (pymavlink connection, heartbeat handshake, message decode) is identical to the
live-SITL path -- only the data origin differs.

Transport contract (all MAVLink NAMED_VALUE_FLOAT, name <= 10 chars, plus HEARTBEAT)
-----------------------------------------------------------------------------------
  HEARTBEAT                      -> lets the monitor's wait_heartbeat() succeed
  U0..U6   (7 floats)            -> control input u(t) = [phi_cmd theta_cmd psi_cmd thr
                                    tiltN tiltE const]  (Useg), needed to drive y=Cx+Du, x=Ax+Bu
  Y0..Y11  (12 floats)           -> model-state / sensor-output vector y(t) (Yseg); the monitor
                                    uses Y0..Y11 at the first tick to initialise the model state x
  BARO     (1 float)             -> barometer altitude AGL (EXTRAseg[:,1]); ATTACKED on the wire
  ALTEST   (1 float)             -> autopilot fused altitude estimate (Yseg[:,2]); the live
                                    analogue is GLOBAL_POSITION_INT.relative_alt
  PMEAS    (1 float)             -> measured roll rate p (Yseg[:,9]); ATTACKED on the wire
  TICK     (1 float)             -> sample index, so the monitor and source stay aligned

The attack is applied HERE (on the wire), modelling a compromised sensor, so the monitor sees
already-corrupted measurements -- exactly the live threat model.

Run:
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 mavlink_replay_source.py \
      --attack baro --offset 3.0 --start 2000 --end 2500 --rate 1000
"""
import argparse, time
import numpy as np
import scipy.io
from pymavlink import mavutil

DATA = '/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='udpout:127.0.0.1:14570',
                    help='MAVLink endpoint to send to (monitor listens on udpin:...:14570)')
    ap.add_argument('--seg', type=int, default=0)
    ap.add_argument('--attack', choices=['none', 'baro', 'gyro'], default='baro')
    ap.add_argument('--offset', type=float, default=3.0, help='baro bias (m) added during attack')
    ap.add_argument('--gyroval', type=float, default=0.8, help='roll-rate p set-value (rad/s) during attack')
    ap.add_argument('--start', type=int, default=2000, help='attack start sample (2000 = 40.0 s)')
    ap.add_argument('--end', type=int, default=2500, help='attack end sample (2500 = 50.0 s)')
    ap.add_argument('--rate', type=float, default=1000.0, help='ticks per second on the wire (>50 = faster than real time)')
    args = ap.parse_args()

    d = scipy.io.loadmat(DATA)
    Y = d['Yseg'][0][args.seg].astype(float)
    U = d['Useg'][0][args.seg].astype(float)
    EX = d['EXTRAseg'][0][args.seg].astype(float)
    N = Y.shape[0]

    m = mavutil.mavlink_connection(args.out, source_system=1, source_component=1)
    print(f"[source] sending on {args.out}; segment {args.seg}; {N} samples; "
          f"attack={args.attack} window[{args.start}:{args.end}] rate={args.rate} Hz")

    period = 1.0 / args.rate
    t0 = time.time()
    last_hb = 0.0
    for n in range(N):
        now = time.time()
        # heartbeat at ~2 Hz so the monitor can connect any time
        if now - last_hb > 0.5:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                 mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0,
                                 mavutil.mavlink.MAV_STATE_ACTIVE)
            last_hb = now

        attacking = args.start <= n < args.end
        baro = EX[n, 1] + (args.offset if (attacking and args.attack == 'baro') else 0.0)
        pmeas = (args.gyroval if (attacking and args.attack == 'gyro') else Y[n, 9])
        tboot = int((now - t0) * 1000)

        def nvf(name, val):
            m.mav.named_value_float_send(tboot, name.encode('ascii')[:10], float(val))

        for i in range(7):
            nvf(f'U{i}', U[n, i])
        for i in range(12):
            nvf(f'Y{i}', Y[n, i])
        nvf('BARO', baro)
        nvf('ALTEST', Y[n, 2])
        nvf('PMEAS', pmeas)
        nvf('TICK', n)

        # pace the stream
        target = t0 + (n + 1) * period
        sleep = target - time.time()
        if sleep > 0:
            time.sleep(sleep)

    # a few trailing heartbeats + end marker
    for _ in range(5):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                             mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0,
                             mavutil.mavlink.MAV_STATE_STANDBY)
        m.mav.named_value_float_send(int((time.time() - t0) * 1000), b'TICK', float(-1))
        time.sleep(0.05)
    print(f"[source] done; streamed {N} samples in {time.time()-t0:.1f}s")


if __name__ == '__main__':
    main()
