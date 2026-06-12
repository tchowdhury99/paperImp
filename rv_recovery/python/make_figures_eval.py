#!/usr/bin/env python3
"""
§4.2.2 "Effectiveness" demonstration figures — READ-ONLY.

Reads only:
  - rv_recovery/data/operation_data_50hz.mat      (recorded clean operation data)
  - rv_recovery/matlab/models/quadrotor_12state.mat (identified A,B,C,D)
  - rv_recovery/firmware_patch/recovery_params.h    (DTW N, T_on, T_off)

No stored data, model, thresholds, or firmware/monitor code is modified. The
software-sensor predictions are produced by a Python mirror of the firmware
Algorithm-1 monitor (recovery_monitor.h, unit-tested 25/25). Attacks are injected
*in memory* into copies of the recorded clean traces, exactly as the paper's §4
evaluation injects attacks to measure effectiveness / FN.

Outputs -> ~/paperImp/rv_recovery/figures/
  fig9_drift_correction.png      (paper Fig. 12: drift w/o vs w/ §3.3 synchronization)
  fig10_fp_fn_vs_threshold.png   (paper Fig. 14: FP (clean) + FN (attacked) vs threshold)
  fig11_error_vs_attack_scale.png(paper Fig. 16b: residual / detection vs attack scale)
"""
import os
import numpy as np
import scipy.io
from scipy.signal import butter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HOME = os.path.expanduser('~')
DATA  = f'{HOME}/paperImp/rv_recovery/data/operation_data_50hz.mat'
MODEL = f'{HOME}/paperImp/rv_recovery/matlab/models/quadrotor_12state.mat'
PARAMS = f'{HOME}/paperImp/rv_recovery/firmware_patch/recovery_params.h'
OUT = f'{HOME}/paperImp/rv_recovery/figures'
os.makedirs(OUT, exist_ok=True)

TS = 0.02
CH = ['pN','pE','alt','phi','theta','psi','vN','vE','vUp','p','q','r']
LPF_B, LPF_A = butter(2, 5.0/(50.0/2.0))


class LPF:
    def __init__(self): self.w1 = self.w2 = 0.0
    def step(self, x):
        y = LPF_B[0]*x + self.w1
        self.w1 = LPF_B[1]*x - LPF_A[1]*y + self.w2
        self.w2 = LPF_B[2]*x - LPF_A[2]*y
        return y


def parse_params():
    txt = open(PARAMS).read()
    def arr(name):
        s = txt.split(name)[1].split('{')[1].split('}')[0]
        return [float(v.replace('f','')) for v in s.split(',') if v.strip()]
    return [int(v) for v in arr('REC_N_DEFAULT')], arr('REC_TON_DEFAULT'), arr('REC_TOFF_DEFAULT')


def monitor(U, Y, A, B, C, D, windows, ton, correction=True,
            attack_ch=None, attack_t=None, attack_val=0.0):
    """Algorithm-1 mirror.
      correction=True : full §3.3 — window checkpoint resets r, updates e, re-seeds
                        the model state (bounded drift).
      correction=False: NO checkpoint — pure open-loop, error accumulates
                        monotonically (the paper's 'without correction' baseline).
    Optional in-memory constant attack on attack_ch from sample attack_t.
    Tracks max_r = the running maximum of the per-window residual r reached at any
    point (so a detection that latches mid-window is captured)."""
    N, NY = U.shape[0], C.shape[0]
    x = Y[0].astype(float).copy()
    lpf = [LPF() for _ in range(NY)]
    e = np.zeros(NY); r = np.zeros(NY); t = np.zeros(NY, int)
    eh = [np.zeros(w) for w in windows]
    pred_err = np.zeros((N, NY)); accum = np.zeros((N, NY))
    max_r = np.zeros(NY); acc = np.zeros(NY)
    for n in range(N):
        y = C @ x + D @ U[n]; x = A @ x + B @ U[n]
        for k in range(NY):
            m_raw = Y[n, k]
            if attack_ch == k and attack_t is not None and n >= attack_t:
                m_raw = m_raw + attack_val
            ms_raw = y[k]; w = windows[k]
            m = lpf[k].step(m_raw); t[k] += 1
            if correction and t[k] > w:
                t[k] = 0; r[k] = 0.0; e[k] = eh[k].mean(); ms_raw = m
                x[k] = m; acc[k] = 0.0
            ms = ms_raw - e[k]
            eh[k][t[k] % w] = ms_raw - m
            r[k] += abs(m - ms); acc[k] += abs(m - ms)
            if r[k] > max_r[k]:
                max_r[k] = r[k]
            pred_err[n, k] = abs(m - ms); accum[n, k] = acc[k]
    return dict(pred_err=pred_err, accum=accum, max_r=max_r)


def main():
    S = scipy.io.loadmat(DATA)
    Mm = scipy.io.loadmat(MODEL)
    A, B, C, D = (Mm[k].astype(float) for k in 'ABCD')
    Useg = [np.asarray(u, float) for u in S['Useg'].ravel()]
    Yseg = [np.asarray(y, float) for y in S['Yseg'].ravel()]
    N_, TON, TOFF = parse_params()

    # ── Fig 9: drift correction (paper Fig. 12) ──────────────────────────────
    # Use a position channel (pN), where open-loop integrator drift is visible.
    seg = 7
    U, Y = Useg[seg], Yseg[seg]
    n = min(len(U), 5000); U, Y = U[:n], Y[:n]
    tsec = np.arange(n)*TS
    k = 0  # pN — strongest open-loop drift (integrator state)
    no_corr = monitor(U, Y, A, B, C, D, N_, TON, correction=False)
    with_corr = monitor(U, Y, A, B, C, D, N_, TON, correction=True)
    fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(tsec, no_corr['accum'][:, k], 'r-', lw=1.0,
               label='without correction (open-loop, drift accumulates)')
    ax[0].plot(tsec, with_corr['accum'][:, k], 'b-', lw=1.0,
               label='with §3.3 synchronization + error reset (bounded)')
    ax[0].set_ylabel(f'accumulated prediction error  ({CH[k]})')
    ax[0].legend(fontsize=9); ax[0].grid(alpha=0.3)
    ax[0].set_title('Drift correction by windowed synchronization (paper Fig. 12 analogue)')
    ax[1].plot(tsec, no_corr['pred_err'][:, k], 'r-', lw=0.6, label='without correction')
    ax[1].plot(tsec, with_corr['pred_err'][:, k], 'b-', lw=0.6, label='with correction')
    ax[1].set_ylabel('per-sample error |m−ms|'); ax[1].set_xlabel('time (s)')
    ax[1].legend(fontsize=9); ax[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f'{OUT}/fig9_drift_correction.png', dpi=120)
    print('wrote fig9_drift_correction.png')

    # ── Fig 10: FP + FN vs threshold (paper Fig. 14) ─────────────────────────
    # PER-MISSION metric (paper §4.2.2: "how many times recovery activation is
    # missed"): for each mission take the max residual reached. FP = clean missions
    # whose max residual exceeds the threshold (false alarm); FN = attacked missions
    # whose max residual stays below the threshold (attack missed).
    k = 11  # r (yaw rate) channel
    w = N_[k]
    attack_val = 0.6  # constant injection magnitude (rad/s), paper §4.3 class
    clean_max, attacked_max = [], []
    for U2, Y2 in zip(Useg, Yseg):
        clean_max.append(monitor(U2, Y2, A, B, C, D, N_, TON)['max_r'][k])
        attacked_max.append(monitor(U2, Y2, A, B, C, D, N_, TON,
                                    attack_ch=k, attack_t=w,
                                    attack_val=attack_val)['max_r'][k])
    clean_max = np.array(clean_max); attacked_max = np.array(attacked_max)
    scales = np.linspace(0.1, 2.0, 30)
    fp = [np.mean(clean_max > TON[k]*s) for s in scales]
    fn = [np.mean(attacked_max <= TON[k]*s) for s in scales]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(scales, fp, 'b.-', label='false positive (clean missions)')
    ax.plot(scales, fn, 'r.-', label='false negative (attacked missions)')
    ax.axvline(1.0, color='g', ls='--', label='selected T_on')
    ax.set_xlabel('threshold / selected T_on'); ax.set_ylabel('rate')
    ax.set_title(f'Parameter selection: FP & FN vs threshold, channel {CH[k]} '
                 '(paper Fig. 14)')
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f'{OUT}/fig10_fp_fn_vs_threshold.png', dpi=120)
    print(f'wrote fig10_fp_fn_vs_threshold.png  '
          f'(at selected T_on: FP={np.mean(clean_max>TON[k]):.2f}, '
          f'FN={np.mean(attacked_max<=TON[k]):.2f})')

    # ── Fig 11: residual / detection vs attack scale (paper Fig. 16b) ────────
    # Max residual reached (running max, captures the latching window) vs injected
    # constant-bias magnitude, averaged over all missions; T_on marks detection.
    k = 11; w = N_[k]
    mags = np.linspace(0.0, 1.5, 16)   # rad/s injected constant bias
    peak_mean, peak_lo, peak_hi, det_frac = [], [], [], []
    for mag in mags:
        mr = [monitor(U2, Y2, A, B, C, D, N_, TON, attack_ch=k, attack_t=w,
                      attack_val=mag)['max_r'][k] for U2, Y2 in zip(Useg, Yseg)]
        mr = np.array(mr)
        peak_mean.append(mr.mean()); peak_lo.append(mr.min()); peak_hi.append(mr.max())
        det_frac.append(np.mean(mr > TON[k]))
    peak_mean = np.array(peak_mean)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.fill_between(mags, peak_lo, peak_hi, color='gray', alpha=0.2,
                    label='min–max over missions')
    ax.plot(mags, peak_mean, 'k.-', label='mean peak residual')
    ax.axhline(TON[k], color='orange', ls='--', label=f'T_on = {TON[k]:.1f}')
    det_mags = mags[np.array(det_frac) >= 1.0]
    if det_mags.size:
        ax.axvline(det_mags.min(), color='g', ls=':',
                   label=f'all missions detected for attack ≥ {det_mags.min():.2f} rad/s')
    ax.set_xlabel('attack scale (injected constant bias, rad/s)')
    ax.set_ylabel(f'peak residual ({CH[k]})')
    ax.set_title('Detection vs attack scale (paper Fig. 16b analogue)')
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f'{OUT}/fig11_error_vs_attack_scale.png', dpi=120)
    print('wrote fig11_error_vs_attack_scale.png')

    print(f'\nAll figures in: {OUT}')


if __name__ == '__main__':
    main()
