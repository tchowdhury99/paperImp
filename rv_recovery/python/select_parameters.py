#!/usr/bin/env python3
"""
Recovery parameter selection — paper §3.3.

  1. Window size N  = maximum time-displacement between the real sensor signal and
     the software-sensor signal, computed with dynamic time warping over the large
     set of clean operation data.
  2. Threshold T_on = e_max + margin, where e_max is the maximum accumulated error
     |m - ms| within any window of size N over clean data.  T_off < T_on.

The software-sensor signal is produced by predict_software_sensors(), which mirrors
recovery_monitor.h recovery_check_ms() LINE FOR LINE (same Butterworth filter, same
checkpoint gating, same e = avg(ms_raw - m) sign, same ms<-m sync + model-state
re-seed). Calibrating thresholds on any other predictor yields wrong sensitivities.

Documented choices where the paper is silent (validated by the Fig. 14 FP/FN sweep):
  margin = 0.10 * e_max          T_off = 0.80 * T_on          K_safe = 10

Paper reference (3DR Solo @ 400 Hz): N = 230 (575 ms), T_on = 38 (roll rate).
Ours runs at 50 Hz (sanctioned deviation).
"""
import os
import numpy as np
import scipy.io
from scipy.signal import butter
from dtaidistance import dtw as dtw_lib

HOME = os.path.expanduser('~')
DATA_MAT  = f'{HOME}/paperImp/rv_recovery/data/operation_data_50hz.mat'
MODEL_MAT = f'{HOME}/paperImp/rv_recovery/matlab/models/quadrotor_12state.mat'
OUT_H     = f'{HOME}/paperImp/rv_recovery/firmware_patch/recovery_params.h'
FW_H      = (f'{HOME}/paperImp/ardupilot_ws/arducopter-3.4/'
             'libraries/AP_InertialSensor/recovery_params.h')

TS = 0.02
CH_NAMES = ['pN','pE','alt','phi','theta','psi','vN','vE','vUp','p','q','r']

MARGIN_FRAC = 0.10     # margin = 10% of e_max (documented choice)
TOFF_FRAC   = 0.80     # T_off = 0.8 * T_on   (documented choice; paper: T_off < T_on)
KSAFE       = 10
DTW_CHUNK   = 5000     # piecewise DTW chunk (100 s) — memory bound; max over chunks
DTW_BAND    = 250      # Sakoe-Chiba band (5 s) — displacement search range

# same filter as recovery_monitor.h (butter(2, 5/(50/2)))
LPF_B, LPF_A = butter(2, 5.0 / (50.0 / 2.0))


class LPF:
    """Direct-form II transposed biquad — mirrors lpf_step()."""
    def __init__(self):
        self.w1 = 0.0; self.w2 = 0.0
    def step(self, x):
        y = LPF_B[0]*x + self.w1
        self.w1 = LPF_B[1]*x - LPF_A[1]*y + self.w2
        self.w2 = LPF_B[2]*x - LPF_A[2]*y
        return y


def predict_software_sensors(U, Y, A, B, C, D, windows):
    """
    Mirror of recovery_monitor.h over one clean segment, all NY channels.
    U [N x NU], Y [N x NY] (real measurements = real states, C = I).
    windows: per-channel window size N.
    Returns: M (filtered real), MS (compensated software sensor),
             RWIN (list per channel of completed per-window accumulated residuals).
    """
    N, NU_ = U.shape
    NY = C.shape[0]
    x = Y[0].astype(float).copy()          # init model state from real state
    lpf = [LPF() for _ in range(NY)]
    e   = np.zeros(NY)
    r   = np.zeros(NY)
    t   = np.zeros(NY, dtype=int)
    err_hist = [np.zeros(w) for w in windows]
    M  = np.zeros((N, NY))
    MS = np.zeros((N, NY))
    RWIN = [[] for _ in range(NY)]

    for n in range(N):
        # lines 6-7: y = C x + D u (before advance), then x = A x + B u
        y = C @ x + D @ U[n]
        x = A @ x + B @ U[n]

        for k in range(NY):
            m_raw  = Y[n, k]
            ms_raw = y[k]
            m = lpf[k].step(m_raw)                      # line 8
            t[k] += 1                                   # line 10
            if t[k] > windows[k]:                       # line 11 (never in recovery
                RWIN[k].append(r[k])                    #   on clean data)
                t[k] = 0                                # line 12
                r[k] = 0.0                              # line 13
                e[k] = err_hist[k].mean()               # line 14
                ms_raw = m                              # line 15: sync
                x[k] = m                                #   feed into model (C = I)
            ms = ms_raw - e[k]                          # line 16
            err_hist[k][t[k] % windows[k]] = ms_raw - m
            r[k] += abs(m - ms)                         # line 17
            M[n, k]  = m
            MS[n, k] = ms
    return M, MS, RWIN


def dtw_max_displacement(a, b):
    """Max |i-j| along the optimal DTW path, piecewise over chunks (memory bound)."""
    n = len(a)
    worst = 0
    for s in range(0, max(n - 100, 1), DTW_CHUNK):
        ea = a[s:s+DTW_CHUNK].astype(np.double)
        eb = b[s:s+DTW_CHUNK].astype(np.double)
        if len(ea) < 100:
            break
        # z-normalize per chunk for alignment only (displacement is what we read off)
        ea = (ea - ea.mean()) / (ea.std() + 1e-12)
        eb = (eb - eb.mean()) / (eb.std() + 1e-12)
        path = dtw_lib.warping_path(ea, eb, window=DTW_BAND, use_c=True)
        d = max(abs(i - j) for i, j in path)
        worst = max(worst, d)
    return worst


def main():
    S = scipy.io.loadmat(DATA_MAT)
    mdl = scipy.io.loadmat(MODEL_MAT)
    A = mdl['A'].astype(float); B = mdl['B'].astype(float)
    C = mdl['C'].astype(float); D = mdl['D'].astype(float)
    NY = C.shape[0]
    segs = [(np.asarray(u, float), np.asarray(y, float))
            for u, y in zip(S['Useg'].ravel(), S['Yseg'].ravel())]
    print(f"Model NX={A.shape[0]} NU={B.shape[1]} | {len(segs)} clean segment(s)")
    rho = max(abs(np.linalg.eigvals(A)))
    print(f"spectral radius(A) = {rho:.4f}")

    # ── pass 1: provisional window (paper-shaped: 575 ms -> 29 @ 50 Hz) for ms ──
    prov = np.full(NY, 29, dtype=int)
    N_ch = np.zeros(NY, dtype=int)
    for (U, Y) in segs:
        M, MS, _ = predict_software_sensors(U, Y, A, B, C, D, prov)
        for k in range(NY):
            d = dtw_max_displacement(M[:, k], MS[:, k])
            N_ch[k] = max(N_ch[k], d)
    N_ch = np.maximum(N_ch, 5)         # window of <5 samples is degenerate
    print("\nDTW max time-displacement per channel (samples @ 50 Hz):")
    for k in range(NY):
        print(f"  {CH_NAMES[k]:5s} N = {N_ch[k]:4d}  ({N_ch[k]*TS*1000:.0f} ms)"
              f"   [paper 3DR Solo: 230 @ 400 Hz = 575 ms]")

    # ── pass 2: rerun mirror with the selected windows; e_max per channel ───────
    e_max = np.zeros(NY)
    for (U, Y) in segs:
        _, _, RWIN = predict_software_sensors(U, Y, A, B, C, D, N_ch)
        for k in range(NY):
            if RWIN[k]:
                e_max[k] = max(e_max[k], max(RWIN[k]))

    T_on  = e_max * (1.0 + MARGIN_FRAC)
    T_off = T_on * TOFF_FRAC
    print("\nThresholds (T = e_max + margin, §3.3):")
    for k in range(NY):
        print(f"  {CH_NAMES[k]:5s} e_max={e_max[k]:10.4f}  "
              f"T_on={T_on[k]:10.4f}  T_off={T_off[k]:10.4f}")

    # ── emit recovery_params.h ───────────────────────────────────────────────────
    cap = 1 << int(np.ceil(np.log2(max(int(N_ch.max()), 64) + 1)))
    lines = [
        '// recovery_params.h — generated by select_parameters.py (paper §3.3)',
        '// Predictor: exact mirror of recovery_monitor.h (Alg. 1 + LPF + sync).',
        f'// N = max DTW displacement; T_on = e_max + {MARGIN_FRAC:.0%} margin; '
        f'T_off = {TOFF_FRAC:.0%} T_on.',
        '// Channels: [pN pE alt phi theta psi vN vE vUp p q r] @ 50 Hz (Ts=0.02 s)',
        '#pragma once',
        f'#define REC_WINDOW_CAP  {cap}',
        f'#define REC_KSAFE_DEFAULT  {KSAFE}',
        'static const int REC_N_DEFAULT[12] = {',
        '    ' + ', '.join(str(int(n)) for n in N_ch),
        '};',
        'static const float REC_TON_DEFAULT[12] = {',
        '    ' + ', '.join(f'{v:.6f}f' for v in T_on),
        '};',
        'static const float REC_TOFF_DEFAULT[12] = {',
        '    ' + ', '.join(f'{v:.6f}f' for v in T_off),
        '};',
    ]
    with open(OUT_H, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"\nWrote {OUT_H}")
    import shutil
    shutil.copy(OUT_H, FW_H)
    print(f"Synced {FW_H}")


if __name__ == '__main__':
    main()
