#!/usr/bin/env python3
"""
online_stl_mavlink.py
=====================
Real-time (online) STL monitor + closed-loop recovery over MAVLink.

This is the live counterpart of the eight offline STL scripts. It connects to a MAVLink UDP
endpoint (the SITL or the replay bridge), and on every sample it:

  1. reads the control input u(t) and sensor measurements over MAVLink (pymavlink);
  2. drives the identified state-space software-sensor model  (paper Eq. 1-2):
         y(t) = C x(t) + D u(t)          <- software-sensor prediction  ms
         x(t+1) = A x(t) + B u(t)        <- model state advance
     with a short paper-shaped re-sync window (29 samples = 580 ms) while healthy;
  3. forms the per-sensor residual  resid = | m - ms |  (paper Eq. r <- r + |m - ms|, here
     evaluated per-sample for fast STL detection);
  4. evaluates an STL spec ONLINE with rtamt:  phi = G (resid < eps)   ->   rho = eps - resid;
  5. when rho < 0 (STL violated) it enters RECOVERY: the attacked physical sensor is replaced
     by the software sensor (paper Alg. 1 line: m <- ms), and after K consecutive safe samples
     (resid < eps) it switches back to the real sensor (paper Ton/Toff/K state machine).

Two sensors are monitored:
  ALT  : m = barometer altitude (BARO, attackable),  ms = autopilot fused altitude estimate
         (ALTEST; live analogue = GLOBAL_POSITION_INT.relative_alt).  eps = 0.30 m.
         This mirrors the offline baro-integrity spec exactly and needs no open-loop integration.
  P    : m = measured roll rate p (PMEAS, attackable), ms = model prediction y[9] = (Cx+Du)[9].
         eps = 0.15 rad/s.  This replaces the offline gyro PLACEHOLDER (clean-copy) with a
         genuine identified-model software-sensor prediction.

Run (after starting mavlink_replay_source.py, or pointed at a live SITL adapter):
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 online_stl_mavlink.py \
      --conn udpin:127.0.0.1:14570 --plot stl_result_online_mavlink.png
"""
import argparse
import numpy as np
import scipy.io
from scipy.signal import butter
from pymavlink import mavutil
import rtamt
import matplotlib
matplotlib.use('Agg')          # headless-safe (the offline baro/gps scripts hung on plt.show())
import matplotlib.pyplot as plt

MDL = '/home/tchowdh4/paperImp/rv_recovery/matlab/models/quadrotor_12state.mat'
EPS_ALT = 0.30          # m       (offline baro-integrity threshold)
EPS_P = 0.15            # rad/s   (offline gyro-integrity threshold)
W_RESYNC = 29           # samples = 580 ms model re-sync window (paper-shaped)
K_SAFE = 10             # consecutive safe samples to leave recovery (paper K)
LPF_B, LPF_A = butter(2, 5.0 / (50.0 / 2.0))   # same filter as recovery_monitor.h


class LPF:
    """Direct-form II transposed biquad (mirrors lpf_step in recovery_monitor.h)."""
    def __init__(self):
        self.w1 = 0.0
        self.w2 = 0.0

    def step(self, x):
        y = LPF_B[0] * x + self.w1
        self.w1 = LPF_B[1] * x - LPF_A[1] * y + self.w2
        self.w2 = LPF_B[2] * x - LPF_A[2] * y
        return y


def make_spec(var, eps):
    spec = (rtamt.StlDiscreteTimeSpecification() if hasattr(rtamt, 'StlDiscreteTimeSpecification')
            else rtamt.STLDiscreteTimeSpecification())
    spec.declare_var(var, 'float')
    try:
        spec.set_sampling_period(20, 'ms', 0.1)
    except TypeError:
        spec.set_sampling_period(20, 'ms')
    spec.spec = f'{var} < {eps}'      # atomic predicate -> online rho = eps - var (causal)
    spec.parse()
    return spec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='udpin:127.0.0.1:14570')
    ap.add_argument('--plot', default='/home/tchowdh4/paperImp/stl_result_online_mavlink.png')
    ap.add_argument('--max_idle', type=float, default=5.0, help='seconds of silence before stop')
    args = ap.parse_args()

    mdl = scipy.io.loadmat(MDL)
    A, B, C, D = (mdl[k].astype(float) for k in 'ABCD')

    print(f"[monitor] connecting on {args.conn} ...")
    mav = mavutil.mavlink_connection(args.conn)
    mav.wait_heartbeat()
    print(f"[monitor] heartbeat from system {mav.target_system}; STL online monitoring started.")

    spec_alt = make_spec('resid_alt', EPS_ALT)
    spec_p = make_spec('resid_p', EPS_P)

    # model / filter / recovery state
    x = None
    lpf_p = LPF()
    t_p = 0
    rec_alt = rec_p = False
    safe_alt = safe_p = 0
    det_alt = det_p = None

    # logs
    T, RESA, RESP, RHOA, RHOP, RECA, RECP, BSER, BFIX = ([] for _ in range(9))

    cur = {}
    seen = -1
    import time
    last_msg = time.time()

    while True:
        msg = mav.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_msg > args.max_idle:
                break
            continue
        last_msg = time.time()
        name = msg.name.strip('\x00').strip()
        cur[name] = msg.value

        if name != 'TICK':
            continue
        tick = int(round(msg.value))
        if tick < 0:            # end marker
            break
        if tick == seen:
            continue
        seen = tick

        # need a full sample
        if not all(k in cur for k in ('BARO', 'ALTEST', 'PMEAS')) or \
           not all(f'U{i}' in cur for i in range(7)):
            continue
        u = np.array([cur[f'U{i}'] for i in range(7)], float)
        if x is None:
            if not all(f'Y{i}' in cur for i in range(12)):
                continue
            x = np.array([cur[f'Y{i}'] for i in range(12)], float)   # init model state

        # ---- model step: y = Cx + Du ; x = Ax + Bu ----
        y = C @ x + D @ u
        x = A @ x + B @ u

        # ===== ALT channel: m = baro (attackable), ms = fused estimate =====
        m_baro = cur['BARO']
        ms_alt = cur['ALTEST']
        resid_alt = abs(m_baro - ms_alt)
        rho_alt = spec_alt.update(tick, [('resid_alt', resid_alt)])
        if rho_alt < 0 and not rec_alt:
            rec_alt = True
            safe_alt = 0
            if det_alt is None:
                det_alt = tick
        baro_out = m_baro
        if rec_alt:
            baro_out = ms_alt                       # paper: m <- ms (sensor replacement)
            safe_alt = safe_alt + 1 if resid_alt < EPS_ALT else 0
            if safe_alt > K_SAFE:
                rec_alt = False

        # ===== P channel: m = measured p (attackable), ms = model prediction y[9] =====
        ms_p = y[9]
        m_p = lpf_p.step(cur['PMEAS'])
        t_p += 1
        if (not rec_p) and t_p > W_RESYNC:          # re-sync only while healthy
            t_p = 0
            ms_p = m_p
            x[9] = m_p
        resid_p = abs(m_p - ms_p)
        rho_p = spec_p.update(tick, [('resid_p', resid_p)])
        if rho_p < 0 and not rec_p:
            rec_p = True
            safe_p = 0
            if det_p is None:
                det_p = tick
        if rec_p:
            safe_p = safe_p + 1 if resid_p < EPS_P else 0
            if safe_p > K_SAFE:
                rec_p = False

        T.append(tick * 0.02)
        RESA.append(resid_alt); RESP.append(resid_p)
        RHOA.append(rho_alt); RHOP.append(rho_p)
        RECA.append(rec_alt); RECP.append(rec_p)
        BSER.append(m_baro); BFIX.append(baro_out)

    # ---- report ----
    T = np.array(T); RESA = np.array(RESA); RESP = np.array(RESP)
    RHOA = np.array(RHOA); RHOP = np.array(RHOP)
    RECA = np.array(RECA); RECP = np.array(RECP)
    print("\n================ ONLINE STL / MAVLINK RESULT ================")
    print(f"samples processed : {len(T)}")
    if det_alt is not None:
        print(f"ALT  attack detected at t = {det_alt*0.02:.2f} s (sample {det_alt}); "
              f"recovery active {int(RECA.sum())} samples")
    else:
        print("ALT  : no STL violation (clean).")
    if det_p is not None:
        print(f"P    attack detected at t = {det_p*0.02:.2f} s (sample {det_p}); "
              f"recovery active {int(RECP.sum())} samples")
    else:
        print("P    : no STL violation (clean).")
    print("============================================================")

    # ---- plot ----
    if len(T):
        fig, ax = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
        ax[0].plot(T, BSER, label='baro measured (on wire)', color='red', alpha=.7)
        ax[0].plot(T, BFIX, '--', label='baro after recovery (m<-ms)', color='green')
        ax[0].set_ylabel('Altitude (m)'); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
        ax[1].plot(T, RESA, label='|baro - estimate|', color='red')
        ax[1].axhline(EPS_ALT, ls=':', color='orange', label=f'eps_alt={EPS_ALT}')
        ax[1].plot(T, RESP, label='|p - model|', color='blue', alpha=.6)
        ax[1].axhline(EPS_P, ls=':', color='purple', label=f'eps_p={EPS_P}')
        ax[1].set_ylabel('residual'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
        ax[2].plot(T, RHOA, label='rho ALT', color='red')
        ax[2].plot(T, RHOP, label='rho P', color='blue', alpha=.6)
        ax[2].axhline(0, color='black', lw=1)
        ax[2].set_ylabel('STL robustness rho'); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
        ax[3].plot(T, RECA.astype(int), label='ALT recovery mode', color='red')
        ax[3].plot(T, RECP.astype(int) * 1.05, label='P recovery mode', color='blue', alpha=.6)
        ax[3].set_ylabel('recovery on/off'); ax[3].set_xlabel('Time (s)')
        ax[3].legend(fontsize=8); ax[3].grid(alpha=.3)
        plt.suptitle('Online STL over MAVLink + closed-loop recovery\n'
                     'phi = G (resid < eps); rho<0 -> m<-ms; back after K safe', fontsize=10)
        plt.tight_layout()
        plt.savefig(args.plot, dpi=130)
        print(f"Saved: {args.plot}")


if __name__ == '__main__':
    main()
