#!/usr/bin/env python3
"""
offline_faithful_stl.py  —  OFFLINE paper-faithful STL attack detection.

Pipeline (faithful to Choi et al. Algorithm 1 + §3.3):
  1. Build the software sensor and the paper's windowed accumulated residual
     R_{k,N}(t) = sum_{last N} |m - ms|   (faithful_core, mirrors recovery_monitor.h).
  2. Select thresholds T_on = e_max + 10% margin, T_off = 0.8 T_on on CLEAN data (§3.3).
  3. DETECT WITH STL:  phi_detect = G (R < T_on)  ->  rho(t) = T_on - R(t);
     rho < 0  <=>  R > T_on  = the paper's detection rule.  (rtamt spec.evaluate)
  4. Verify the recovery property with STL:
     phi_recover = G[0:H] ( (R > T_on) -> F[0:10000ms] (R < T_off) ).
  5. Recovery state machine (Algorithm 1: m<-ms, back after K safe) for the plot.

Monitored sensors / attacks (sustained, so the faithful thresholds trip):
  alt (barometer) +3.0 m ; p (gyroscope) set 0.8 rad/s ; pE (GPS east) +20 m.

Interpreter: /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
Figures -> newImp/figures/offline_faithful_<name>.png
"""
import os
import numpy as np
import rtamt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import faithful_core as fc

ATTACK_START = 2000          # 40.0 s
ATTACK_END = None            # None = sustained to end of segment (thresholds need sustained attack)
H_MS = 120000                # outer horizon for the recovery spec (120 s covers the segment)


def make_pred_spec(threshold):
    """STL atomic-predicate monitor:  R < threshold  ->  rho = threshold - R."""
    spec = (rtamt.StlDiscreteTimeSpecification() if hasattr(rtamt, 'StlDiscreteTimeSpecification')
            else rtamt.STLDiscreteTimeSpecification())
    spec.declare_var('R', 'float')
    try:
        spec.set_sampling_period(20, 'ms', 0.1)
    except TypeError:
        spec.set_sampling_period(20, 'ms')
    spec.spec = f'R < {threshold:.6f}'
    spec.parse()
    return spec


def make_recovery_spec(T_on, T_off):
    spec = (rtamt.StlDiscreteTimeSpecification() if hasattr(rtamt, 'StlDiscreteTimeSpecification')
            else rtamt.STLDiscreteTimeSpecification())
    spec.declare_var('R', 'float')
    try:
        spec.set_sampling_period(20, 'ms', 0.1)
    except TypeError:
        spec.set_sampling_period(20, 'ms')
    spec.spec = f'G[0:{H_MS}ms] ((R > {T_on:.6f}) -> F[0:10000ms] (R < {T_off:.6f}))'
    spec.parse()
    return spec


def stl_trace(threshold, R, t_ms):
    """Evaluate rho(t) = T - R via rtamt offline (with dataset-format fallback)."""
    spec = make_pred_spec(threshold)
    ds = {'time': t_ms.tolist(), 'R': R.astype(float).tolist()}
    try:
        res = spec.evaluate(ds)
    except Exception:
        spec = make_pred_spec(threshold)
        res = spec.evaluate({'R': list(zip(t_ms.tolist(), R.astype(float).tolist()))})
    res = list(res)
    if res and isinstance(res[0], (list, tuple, np.ndarray)):
        tt = np.array([float(a[0]) for a in res]); rr = np.array([float(a[1]) for a in res])
    else:
        rr = np.array([float(a) for a in res]); tt = np.arange(len(rr)) * 20.0
    return tt, rr


def main():
    Y, U, EX = fc.load_segment(0)
    N = Y.shape[0]
    end = ATTACK_END if ATTACK_END is not None else N
    windows = fc.load_windows()
    t_sec = np.arange(N) * fc.TS
    t_ms = (np.arange(N) * 20).astype(int)

    monitored = list(fc.ATTACK_CASES.keys())
    th = fc.select_thresholds(monitored, seg=0)

    print("=" * 70)
    print("OFFLINE PAPER-FAITHFUL STL DETECTION (Algorithm 1 + STL over R_N)")
    print("=" * 70)
    Dclean = fc.instantaneous_residual(Y, U, windows)

    for k in monitored:
        case = fc.ATTACK_CASES[k]
        p = th[k]
        Ya = fc.inject_attack(Y, k, case['kind'], case['value'], ATTACK_START, end)

        # Runtime Algorithm 1 monitor (recovery-aware, re-seed suppressed in recovery)
        mon = fc.run_monitor(Ya, U, windows, {k: p})[k]
        R_att = mon['R']
        rec_trace = mon['rec']
        R_clean = fc.sliding_R(Dclean, k, p['N'])          # clean has no recovery

        # ---- STL detection on the attacked run:  φ = G(R < T_on) -> ρ = T_on - R ----
        _, rho_att = stl_trace(p['T_on'], R_att, t_ms)
        _, rho_clean = stl_trace(p['T_on'], R_clean, t_ms)
        M = min(len(rho_att), N)
        viol = np.where(rho_att[:M] < 0)[0]
        det = int(viol[0]) if len(viol) else None

        # ---- STL recovery property ----
        rec_spec = make_recovery_spec(p['T_on'], p['T_off'])
        try:
            rr = rec_spec.evaluate({'time': t_ms.tolist(), 'R': R_att.astype(float).tolist()})
            rr = list(rr)
            rho_rec = (rr[0][1] if isinstance(rr[0], (list, tuple, np.ndarray)) else rr[0]) if rr else float('nan')
        except Exception as e:
            rho_rec = float('nan')

        print(f"\n[{case['name']}]  N={p['N']} ({p['N']*fc.TS:.1f}s window)  "
              f"e_max={p['e_max']:.1f}  T_on={p['T_on']:.1f}  T_off={p['T_off']:.1f}")
        print(f"   attack: {case['kind']} {case['value']} {case['unit']} sustained from "
              f"{ATTACK_START*fc.TS:.1f}s")
        if det is not None:
            print(f"   STL DETECTION: rho<0 (R>T_on) at t = {det*fc.TS:.2f} s  "
                  f"(latency {det*fc.TS - ATTACK_START*fc.TS:.2f} s)")
        else:
            print("   STL DETECTION: no violation (R stayed below T_on)")
        print(f"   recovery property phi_recover robustness = {rho_rec:.3f} "
              f"({'holds' if rho_rec >= 0 else 'violated: attack persists (sustained)'} )")

        # ---- plot ----
        fig, ax = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
        ax[0].plot(t_sec, Y[:, k], label=f'{fc.CH_NAMES[k]} measured (clean)', color='green')
        ax[0].plot(t_sec, Ya[:, k], '--', label=f'{fc.CH_NAMES[k]} measured (attacked)', color='red', alpha=.7)
        ax[0].axvspan(ATTACK_START*fc.TS, end*fc.TS, color='red', alpha=.08, label='attack window')
        ax[0].set_ylabel(f"{fc.CH_NAMES[k]} ({case['unit']})"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)

        ax[1].plot(t_sec, R_clean, label='R_N clean', color='green')
        ax[1].plot(t_sec, R_att, '--', label='R_N attacked', color='red')
        ax[1].axhline(p['T_on'], ls=':', color='orange', label=f"T_on={p['T_on']:.0f}")
        ax[1].axhline(p['T_off'], ls=':', color='purple', label=f"T_off={p['T_off']:.0f}")
        ax[1].set_ylabel('R_N = Σ|m−ms|'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)

        ax[2].plot(t_sec[:M], rho_clean[:M], label='ρ clean', color='green')
        ax[2].plot(t_sec[:M], rho_att[:M], '--', label='ρ attacked = T_on − R_N', color='red')
        ax[2].axhline(0, color='black', lw=1.2, label='ρ = 0 (detection)')
        if det is not None:
            ax[2].axvline(det*fc.TS, ls=':', color='blue', label=f'detect t={det*fc.TS:.1f}s')
        ax[2].set_ylabel('STL robustness ρ'); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)

        ax[3].plot(t_sec, rec_trace.astype(int), color='red', label='recovery mode (m←ms)')
        ax[3].set_ylabel('recovery on/off'); ax[3].set_xlabel('Time (s)')
        ax[3].legend(fontsize=8); ax[3].grid(alpha=.3)

        plt.suptitle(
            f"OFFLINE PAPER-FAITHFUL STL — {case['name']} attack detection\n"
            f"Algorithm 1 residual R_N=Σ|m−ms| (N={p['N']}); STL φ=G(R_N < T_on={p['T_on']:.0f}); "
            f"T_on=e_max+10%", fontsize=10)
        plt.tight_layout()
        outp = os.path.join(fc.HERE, 'figures', f"offline_faithful_{case['eps_label']}.png")
        plt.savefig(outp, dpi=130)
        plt.close(fig)
        print(f"   saved: {outp}")

    print("\nOFFLINE_FAITHFUL_DONE")


if __name__ == '__main__':
    main()
