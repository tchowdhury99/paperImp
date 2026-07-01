#!/usr/bin/env python3
"""
online_faithful_stl.py  —  ONLINE (real-time, over MAVLink) paper-faithful STL detection.

Same faithful pipeline as the offline version, run one MAVLink sample at a time:
  - FaithfulMonitor.step(u, m) runs Algorithm 1 (software sensor + LPF + windowed
    re-seed suppressed during recovery + error compensation) and returns the paper's
    windowed accumulated residual R_{k,N}(t).
  - STL (rtamt ONLINE spec.update): phi = (R < T_on) -> rho = T_on - R, evaluated per
    sample; rho < 0  <=>  R > T_on  = the paper's detection rule.
  - Recovery state machine (m<-ms, back after K safe) inside FaithfulMonitor (Algorithm 1).

Thresholds T_on/T_off are selected on CLEAN data at startup (paper §3.3), identical to
the offline detector, so offline and online use the SAME algorithm and SAME parameters.

Run (two terminals):
  python3 online_faithful_stl.py --conn udpin:127.0.0.1:14580 --plot figures/online_faithful_gyro.png --label "gyro"
  python3 mavlink_source.py --out udpout:127.0.0.1:14580 --attack gyro --rate 1500
"""
import argparse, time, os
import numpy as np
from pymavlink import mavutil
import rtamt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import faithful_core as fc


def make_spec(T_on):
    spec = (rtamt.StlDiscreteTimeSpecification() if hasattr(rtamt, 'StlDiscreteTimeSpecification')
            else rtamt.STLDiscreteTimeSpecification())
    spec.declare_var('R', 'float')
    try:
        spec.set_sampling_period(20, 'ms', 0.1)
    except TypeError:
        spec.set_sampling_period(20, 'ms')
    spec.spec = f'R < {T_on:.6f}'          # rho = T_on - R  (causal, per-sample)
    spec.parse()
    return spec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='udpin:127.0.0.1:14580')
    ap.add_argument('--plot', default=os.path.join(fc.HERE, 'figures/online_faithful.png'))
    ap.add_argument('--label', default='')
    ap.add_argument('--max_idle', type=float, default=5.0)
    args = ap.parse_args()

    windows = fc.load_windows()
    monitored = list(fc.ATTACK_CASES.keys())          # alt(2), p(9), pE(1)
    th = fc.select_thresholds(monitored, seg=0)       # paper §3.3 thresholds on clean data
    print("[monitor] thresholds (paper §3.3):")
    for k in monitored:
        print(f"   {fc.CH_NAMES[k]:4s} N={th[k]['N']:5d}  T_on={th[k]['T_on']:.1f}  T_off={th[k]['T_off']:.1f}")

    mon = fc.FaithfulMonitor(windows, {k: th[k] for k in monitored})
    specs = {k: make_spec(th[k]['T_on']) for k in monitored}

    m = mavutil.mavlink_connection(args.conn)
    print(f"[monitor] connecting on {args.conn} ...")
    m.wait_heartbeat()
    print(f"[monitor] heartbeat from system {m.target_system}; ONLINE faithful STL started.")

    cur = {}
    seen = -1
    last_msg = time.time()
    T = []
    R_tr = {k: [] for k in monitored}
    rho_tr = {k: [] for k in monitored}
    rec_tr = {k: [] for k in monitored}
    det = {k: None for k in monitored}

    while True:
        msg = m.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_msg > args.max_idle:
                break
            continue
        last_msg = time.time()
        cur[msg.name.strip('\x00').strip()] = msg.value
        if msg.name.strip('\x00').strip() != 'TICK':
            continue
        tick = int(round(msg.value))
        if tick < 0:
            break
        if tick == seen:
            continue
        seen = tick
        if not all(f'U{i}' in cur for i in range(7)) or not all(f'Y{i}' in cur for i in range(12)):
            continue
        u = np.array([cur[f'U{i}'] for i in range(7)], float)
        y = np.array([cur[f'Y{i}'] for i in range(12)], float)

        res = mon.step(u, y)                          # Algorithm 1 one sample
        T.append(tick * fc.TS)
        for k in monitored:
            R = res[k]['R']
            rho = specs[k].update(tick, [('R', R)])   # ONLINE STL robustness = T_on - R
            R_tr[k].append(R); rho_tr[k].append(rho); rec_tr[k].append(res[k]['rec'])
            if rho < 0 and det[k] is None:
                det[k] = tick

    # ---- report ----
    print("\n============ ONLINE FAITHFUL STL RESULT ============")
    print(f"samples processed: {len(T)}")
    for k in monitored:
        if det[k] is not None:
            print(f"  {fc.CH_NAMES[k]:4s}: STL detect (R>T_on) at t={det[k]*fc.TS:.2f}s  "
                  f"latency={det[k]*fc.TS - 40.0:.2f}s  recovery_active={int(np.sum(rec_tr[k]))} samples")
        else:
            print(f"  {fc.CH_NAMES[k]:4s}: no detection")
    print("====================================================")

    # ---- plot ----
    if T:
        T = np.array(T)
        fig, ax = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
        for k in monitored:
            ax[0].plot(T, R_tr[k], label=f"R_N {fc.CH_NAMES[k]}")
            ax[0].axhline(th[k]['T_on'], ls=':', alpha=.5)
        ax[0].set_ylabel('R_N = Σ|m−ms|'); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
        ax[0].set_yscale('symlog')
        for k in monitored:
            ax[1].plot(T, rho_tr[k], label=f"ρ {fc.CH_NAMES[k]} = T_on − R_N")
        ax[1].axhline(0, color='black', lw=1.2, label='ρ=0 (detection)')
        ax[1].set_ylabel('STL robustness ρ'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
        ax[1].set_yscale('symlog')
        for k in monitored:
            ax[2].plot(T, np.array(rec_tr[k]).astype(int), label=f"recovery {fc.CH_NAMES[k]}")
        ax[2].set_ylabel('recovery on/off'); ax[2].set_xlabel('Time (s)')
        ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
        _c = f' — {args.label}' if args.label else ''
        plt.suptitle(f'ONLINE (real-time MAVLink) PAPER-FAITHFUL STL{_c}\n'
                     'Algorithm 1 R_N=Σ|m−ms| ; STL φ=G(R_N < T_on) ; rho<0 ⇒ recovery (m←ms)',
                     fontsize=10)
        plt.tight_layout()
        plt.savefig(args.plot, dpi=130)
        print(f"Saved: {args.plot}")


if __name__ == '__main__':
    main()
