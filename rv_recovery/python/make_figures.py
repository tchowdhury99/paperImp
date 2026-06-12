#!/usr/bin/env python3
"""
Demonstration figures for the RAID 2020 replication — generated OFFLINE from the
real recorded operation data and the real identified 12-state model, driven by a
Python mirror of the firmware Algorithm-1 monitor (recovery_monitor.h, which is
unit-tested 25/25). These figures do NOT depend on the live SITL loop (blocked, see
REPLICATION_REPORT.md §6/L2); they demonstrate the software sensors and the recovery
logic directly, the way the paper's Figures 11 and 5/15 do.

Outputs (PNG) -> ~/paperImp/rv_recovery/figures/
  fig1_software_sensor_tracking.png   (paper Fig. 11 analogue)
  fig2_attack_recovery_gyro.png       (paper Fig. 5 / 15 analogue)
"""
import os
import numpy as np
import scipy.io
from scipy.signal import butter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HOME = os.path.expanduser('~')
DATA = f'{HOME}/paperImp/rv_recovery/data/operation_data_50hz.mat'
MODEL = f'{HOME}/paperImp/rv_recovery/matlab/models/quadrotor_12state.mat'
PARAMS = f'{HOME}/paperImp/rv_recovery/firmware_patch/recovery_params.h'
OUTDIR = f'{HOME}/paperImp/rv_recovery/figures'
os.makedirs(OUTDIR, exist_ok=True)

TS = 0.02
CH = ['pN','pE','alt','phi','theta','psi','vN','vE','vUp','p','q','r']
LPF_B, LPF_A = butter(2, 5.0 / (50.0 / 2.0))


class LPF:
    def __init__(self): self.w1 = self.w2 = 0.0
    def step(self, x):
        y = LPF_B[0]*x + self.w1
        self.w1 = LPF_B[1]*x - LPF_A[1]*y + self.w2
        self.w2 = LPF_B[2]*x - LPF_A[2]*y
        return y


def parse_params():
    """Read N/T_on/T_off arrays out of recovery_params.h."""
    txt = open(PARAMS).read()
    def arr(name):
        s = txt.split(name)[1].split('{')[1].split('}')[0]
        return [float(v.replace('f','')) for v in s.split(',') if v.strip()]
    return arr('REC_N_DEFAULT'), arr('REC_TON_DEFAULT'), arr('REC_TOFF_DEFAULT')


def run_monitor(U, Y, A, B, C, D, windows, ton, toff, ksafe=10,
                attack_ch=None, attack_t=None, attack_val=0.0):
    """Mirror of recovery_check_ms over one segment, all channels.
    Optionally inject a constant attack on attack_ch from sample attack_t.
    Returns dict of arrays for plotting."""
    N, NY = U.shape[0], C.shape[0]
    x = Y[0].astype(float).copy()
    lpf = [LPF() for _ in range(NY)]
    e = np.zeros(NY); r = np.zeros(NY); t = np.zeros(NY, int)
    rec = np.zeros(NY, bool); safe = np.zeros(NY, int)
    eh = [np.zeros(w) for w in windows]
    out = dict(m=np.zeros((N, NY)), ms=np.zeros((N, NY)), r=np.zeros((N, NY)),
               real=np.zeros((N, NY)), used=np.zeros((N, NY)), rec=np.zeros((N, NY)))
    for n in range(N):
        y = C @ x + D @ U[n]
        x = A @ x + B @ U[n]
        for k in range(NY):
            m_raw = Y[n, k]
            if attack_ch == k and attack_t is not None and n >= attack_t:
                m_raw = m_raw + attack_val          # §4.1 constant injection
            out['real'][n, k] = m_raw
            ms_raw = y[k]
            w = windows[k]
            m = lpf[k].step(m_raw)
            t[k] += 1
            if (not rec[k]) and t[k] > w:
                t[k] = 0; r[k] = 0.0
                e[k] = eh[k].mean(); ms_raw = m; x[k] = m
            ms = ms_raw - e[k]
            eh[k][t[k] % w] = ms_raw - m
            r[k] += abs(m - ms)
            if r[k] > ton[k]:
                rec[k] = True; safe[k] = 0
            used = m_raw
            if rec[k]:
                if r[k] < toff[k]:
                    safe[k] += 1
                if safe[k] > ksafe:
                    rec[k] = False
                used = ms
            out['m'][n, k] = m; out['ms'][n, k] = ms
            out['r'][n, k] = r[k]; out['used'][n, k] = used
            out['rec'][n, k] = rec[k]
    return out


def main():
    S = scipy.io.loadmat(DATA)
    M = scipy.io.loadmat(MODEL)
    A, B, C, D = (M[k].astype(float) for k in 'ABCD')
    Useg = [np.asarray(u, float) for u in S['Useg'].ravel()]
    Yseg = [np.asarray(y, float) for y in S['Yseg'].ravel()]
    N_, TON, TOFF = parse_params()
    N_ = [int(v) for v in N_]

    # use a representative segment
    U, Y = Useg[5], Yseg[5]
    n = min(len(U), 4000)
    U, Y = U[:n], Y[:n]
    tsec = np.arange(n) * TS

    # ── Figure 1: software-sensor tracking on clean data (paper Fig. 11) ──────
    res = run_monitor(U, Y, A, B, C, D, N_, TON, TOFF)
    chans = [('p', 9), ('q', 10), ('r', 11), ('phi', 3), ('theta', 4)]
    fig, axes = plt.subplots(len(chans), 1, figsize=(9, 11), sharex=True)
    for ax, (name, k) in zip(axes, chans):
        ax.plot(tsec, res['real'][:, k], 'r-', lw=0.8, label='real sensor')
        ax.plot(tsec, res['ms'][:, k], 'b-', lw=0.8, label='software sensor')
        ax.set_ylabel(name); ax.grid(alpha=0.3)
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].set_title('Software sensor vs. real sensor — clean flight '
                      '(paper Fig. 11 analogue)')
    axes[-1].set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(f'{OUTDIR}/fig1_software_sensor_tracking.png', dpi=120)
    print(f'wrote {OUTDIR}/fig1_software_sensor_tracking.png')

    # ── Figure 2: gyro attack -> detection -> recovery (paper Fig. 5/15) ──────
    k = 11  # r (yaw-rate) channel; clearest constant-bias demo
    atk_t = n // 2
    atk_val = 0.6     # rad/s constant injection (paper §4.3 magnitude class)
    res2 = run_monitor(U, Y, A, B, C, D, N_, TON, TOFF,
                       attack_ch=k, attack_t=atk_t, attack_val=atk_val)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.axvspan(atk_t*TS, tsec[-1], color='red', alpha=0.07, label='attack active')
    ax1.plot(tsec, res2['real'][:, k], 'r-', lw=0.8, label='real (attacked) sensor')
    ax1.plot(tsec, res2['ms'][:, k], 'b-', lw=0.8, label='software sensor')
    ax1.plot(tsec, res2['used'][:, k], 'g--', lw=1.0, label='value used by control loop')
    ax1.set_ylabel(f'{CH[k]} (rad/s)'); ax1.legend(loc='upper left', fontsize=8)
    ax1.grid(alpha=0.3)
    ax1.set_title('Gyro attack detection & recovery (Algorithm 1) '
                  '— paper Fig. 5/15 analogue')
    ax2.plot(tsec, res2['r'][:, k], 'k-', lw=0.9, label='accumulated residual r')
    ax2.axhline(TON[k], color='orange', ls='--', label=f'T_on = {TON[k]:.1f}')
    ax2.axhline(TOFF[k], color='purple', ls=':', label=f'T_off = {TOFF[k]:.1f}')
    det = np.argmax(res2['rec'][:, k] > 0) if res2['rec'][:, k].any() else None
    if det:
        ax2.axvline(det*TS, color='g', lw=1.0, label=f'detected @ {det*TS:.1f}s')
    ax2.set_ylabel('residual'); ax2.set_xlabel('time (s)')
    ax2.legend(loc='upper left', fontsize=8); ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{OUTDIR}/fig2_attack_recovery_gyro.png', dpi=120)
    print(f'wrote {OUTDIR}/fig2_attack_recovery_gyro.png')

    if det:
        print(f'attack injected at {atk_t*TS:.1f}s, detected at {det*TS:.1f}s '
              f'(latency {(det-atk_t)*TS:.1f}s); recovery substituted software sensor')
    else:
        print('NOTE: residual did not cross T_on for this channel/segment/magnitude')


if __name__ == '__main__':
    main()
