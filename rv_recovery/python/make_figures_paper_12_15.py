#!/usr/bin/env python3
"""
Close analogues of paper Figures 12, 13, 14, 15 — READ-ONLY.

Reads only existing artifacts; NOTHING is modified:
  - rv_recovery/data/operation_data_50hz.mat        (recorded operation data)
  - rv_recovery/matlab/models/quadrotor_12state.mat (identified A,B,C,D)
  - rv_recovery/firmware_patch/recovery_params.h    (DTW N, T_on, T_off)
  - rv_recovery/data/sitl_run1/logs/4.BIN           (raw log, for Fig 15 accel/mag)

The software-sensor predictions come from a Python mirror of the firmware
Algorithm-1 monitor (recovery_monitor.h, unit-tested 25/25). Wind (Fig 13) and
attacks (Fig 14/15) are injected IN MEMORY into copies of the recorded traces,
exactly as the paper's §4 evaluation injects disturbances/attacks.

Outputs -> ~/paperImp/rv_recovery/figures/
  fig12_roll_drift_correction.png   (paper Fig. 12: roll prediction + accum error)
  fig13_external_wind_correction.png(paper Fig. 13: constant & dynamic wind)
  fig14_param_selection_windows.png (paper Fig. 14: FP/FN vs threshold, multiple W)
  fig15_allgyro_compensation.png    (paper Fig. 15: all-gyro attack +/- compensation)
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
BIN   = f'{HOME}/paperImp/rv_recovery/data/sitl_run1/logs/4.BIN'
OUT = f'{HOME}/paperImp/rv_recovery/figures'
os.makedirs(OUT, exist_ok=True)

TS = 0.02
CH = ['pN','pE','alt','phi','theta','psi','vN','vE','vUp','p','q','r']
LPF_B, LPF_A = butter(2, 5.0/(50.0/2.0))
R2D = 180/np.pi


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


def run(U, Y, A, B, C, D, windows, ton, correction=True,
        attack_ch=None, attack_t=None, attack_val=0.0, disturb=None):
    """Algorithm-1 mirror. disturb[n,k]: optional external disturbance added to the
    real measurement (wind). Returns per-channel traces + max residual."""
    N, NY = U.shape[0], C.shape[0]
    x = Y[0].astype(float).copy()
    lpf = [LPF() for _ in range(NY)]
    e = np.zeros(NY); r = np.zeros(NY); t = np.zeros(NY, int)
    eh = [np.zeros(w) for w in windows]
    ms_orig = np.zeros((N, NY)); ms_corr = np.zeros((N, NY))
    real = np.zeros((N, NY)); accum = np.zeros((N, NY)); max_r = np.zeros(NY)
    acc = np.zeros(NY)
    for n in range(N):
        y = C @ x + D @ U[n]; x = A @ x + B @ U[n]
        for k in range(NY):
            m_raw = Y[n, k]
            if disturb is not None:
                m_raw = m_raw + disturb[n, k]
            if attack_ch is not None and k in np.atleast_1d(attack_ch) \
               and attack_t is not None and n >= attack_t:
                m_raw = m_raw + attack_val
            real[n, k] = m_raw
            ms_raw = y[k]; w = windows[k]
            m = lpf[k].step(m_raw); t[k] += 1
            if correction and t[k] > w:
                t[k] = 0; r[k] = 0.0; e[k] = eh[k].mean(); ms_raw = m; x[k] = m
                acc[k] = 0.0
            ms_orig[n, k] = ms_raw
            ms = ms_raw - e[k]
            ms_corr[n, k] = ms
            eh[k][t[k] % w] = ms_raw - m
            r[k] += abs(m - ms); acc[k] += abs(m - ms)
            max_r[k] = max(max_r[k], r[k]); accum[n, k] = acc[k]
    return dict(ms_orig=ms_orig, ms_corr=ms_corr, real=real, accum=accum, max_r=max_r)


def load():
    S = scipy.io.loadmat(DATA)
    Mm = scipy.io.loadmat(MODEL)
    A, B, C, D = (Mm[k].astype(float) for k in 'ABCD')
    Useg = [np.asarray(u, float) for u in S['Useg'].ravel()]
    Yseg = [np.asarray(y, float) for y in S['Yseg'].ravel()]
    return A, B, C, D, Useg, Yseg


# ── Fig 12: roll prediction + accumulated error, without vs with correction ──
def fig12(A, B, C, D, Useg, Yseg, N_, TON):
    U, Y = Useg[7], Yseg[7]
    n = min(len(U), 5000); U, Y = U[:n], Y[:n]
    ts = np.arange(n)*TS; k = 3  # roll phi
    nc = run(U, Y, A, B, C, D, N_, TON, correction=False)
    wc = run(U, Y, A, B, C, D, N_, TON, correction=True)
    fig, ax = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    ax[0,0].plot(ts, Y[:, k]*R2D, 'r-', lw=0.8, label='real roll')
    ax[0,0].plot(ts, nc['ms_orig'][:, k]*R2D, 'b-', lw=0.8, label='model (software sensor)')
    ax[0,0].set_title('(a) WITHOUT correction — roll prediction'); ax[0,0].set_ylabel('deg')
    ax[0,0].legend(fontsize=8); ax[0,0].grid(alpha=0.3)
    ax[0,1].plot(ts, nc['accum'][:, k], 'r-'); ax[0,1].set_title('accumulated error (no correction)')
    ax[0,1].grid(alpha=0.3)
    ax[1,0].plot(ts, wc['real'][:, k]*R2D, 'r-', lw=0.8, label='real roll')
    ax[1,0].plot(ts, wc['ms_corr'][:, k]*R2D, 'b-', lw=0.8, label='software sensor (corrected)')
    ax[1,0].set_title('(b) WITH §3.3 synchronization — roll prediction')
    ax[1,0].set_ylabel('deg'); ax[1,0].set_xlabel('time (s)'); ax[1,0].legend(fontsize=8)
    ax[1,0].grid(alpha=0.3)
    ax[1,1].plot(ts, wc['accum'][:, k], 'b-'); ax[1,1].set_title('accumulated error (with correction)')
    ax[1,1].set_xlabel('time (s)'); ax[1,1].grid(alpha=0.3)
    fig.suptitle('Drift correction with synchronization & error reset (paper Fig. 12)')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig12_roll_drift_correction.png', dpi=120)
    print('wrote fig12_roll_drift_correction.png')


# ── Fig 13: external (wind) error correction — constant & dynamic ────────────
def fig13(A, B, C, D, Useg, Yseg, N_, TON):
    U, Y = Useg[7], Yseg[7]
    n = min(len(U), 5000); U, Y = U[:n], Y[:n]
    ts = np.arange(n)*TS; k = 6  # vN (a translational channel wind acts on)
    NY = C.shape[0]
    # constant wind: persistent offset added to the real measurement
    dist_c = np.zeros((n, NY)); dist_c[n//4:, k] = 1.0
    # dynamic wind: slowly varying offset
    dist_d = np.zeros((n, NY))
    dist_d[:, k] = 1.0*np.sin(2*np.pi*ts/20.0) * (ts > ts[n//4])
    rc = run(U, Y, A, B, C, D, N_, TON, disturb=dist_c)
    rd = run(U, Y, A, B, C, D, N_, TON, disturb=dist_d)

    def smooth(a, w=25):
        return np.convolve(np.abs(a), np.ones(w)/w, mode='same')

    fig, ax = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for a, res, ttl in [(ax[0], rc, '(a) constant wind'), (ax[1], rd, '(b) dynamic wind')]:
        err_orig = res['real'][:, k] - res['ms_orig'][:, k]   # no disturbance term
        err_corr = res['real'][:, k] - res['ms_corr'][:, k]   # with disturbance term e
        a.axvspan(ts[len(ts)//4], ts[-1], color='red', alpha=0.06, label='wind active')
        a.plot(ts, smooth(err_orig), color='gray', lw=1.2,
               label='|error| WITHOUT compensation')
        a.plot(ts, smooth(err_corr), 'b-', lw=1.2,
               label='|error| WITH compensation (− e)')
        a.set_title(ttl); a.set_xlabel('time (s)'); a.grid(alpha=0.3); a.legend(fontsize=8)
    ax[0].set_ylabel(f'prediction error |m − ms|  ({CH[k]}, m/s)')
    fig.suptitle('External-force (wind) error reduction by the disturbance term e '
                 '(paper Fig. 13) — the e term absorbs the persistent external offset')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig13_external_wind_correction.png', dpi=120)
    print('wrote fig13_external_wind_correction.png')


# ── Fig 14: FP/FN vs threshold for multiple window sizes W (paper Fig. 14) ───
def fig14(A, B, C, D, Useg, Yseg, N_, TON):
    k = 11  # r channel
    Ws = [25, 50, 100, 200, 400]
    scales = np.linspace(0.1, 2.5, 25)
    attack_val = 0.4
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    for W in Ws:
        win = list(N_); win[k] = W
        clean_max, atk_max = [], []
        for U2, Y2 in zip(Useg, Yseg):
            clean_max.append(run(U2, Y2, A, B, C, D, win, TON)['max_r'][k])
            atk_max.append(run(U2, Y2, A, B, C, D, win, TON,
                               attack_ch=k, attack_t=W, attack_val=attack_val)['max_r'][k])
        clean_max = np.array(clean_max); atk_max = np.array(atk_max)
        # threshold here scales the selected T_on; the residual scale grows with W,
        # so this shows the paper's "larger window -> more FP / fewer FN" trend.
        thr = TON[k]*scales
        fp = [np.mean(clean_max > th) for th in thr]
        fn = [np.mean(atk_max <= th) for th in thr]
        ax[0].plot(thr, fp, '.-', label=f'W={W}')
        ax[1].plot(thr, fn, '.-', label=f'W={W}')
    ax[0].set_title('(a) false-positive rate vs threshold'); ax[0].set_xlabel('threshold')
    ax[0].set_ylabel('FP rate (clean missions)'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].set_title('(b) false-negative rate vs threshold'); ax[1].set_xlabel('threshold')
    ax[1].set_ylabel('FN rate (attacked missions)'); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    fig.suptitle('Recovery parameters and FP/FN rates (paper Fig. 14)')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig14_param_selection_windows.png', dpi=120)
    print('wrote fig14_param_selection_windows.png')


# ── Fig 15: all-gyros attack — roll w/o vs w/ supplementary compensation ─────
def fig15():
    from pymavlink import DFReader
    log = DFReader.DFReader_binary(BIN, zero_time_base=True)
    rows = {'t': [], 'ax': [], 'ay': [], 'az': [], 'roll': [], 'pitch': []}
    magrows = {'t': [], 'mx': [], 'my': [], 'mz': []}
    while True:
        m = log.recv_match(type=['IMU', 'ATT', 'MAG'])
        if m is None: break
        if m.get_type() == 'IMU':
            rows['t'].append(m._timestamp); rows['ax'].append(m.AccX)
            rows['ay'].append(m.AccY); rows['az'].append(m.AccZ)
        elif m.get_type() == 'ATT':
            # piggyback roll/pitch onto nearest by appending separate stream
            magrows.setdefault('att_t', []).append(m._timestamp)
            magrows.setdefault('roll', []).append(np.radians(m.Roll))
            magrows.setdefault('pitch', []).append(np.radians(m.Pitch))
        if len(rows['t']) > 60000:
            break
    t = np.array(rows['t']); ax_ = np.array(rows['ax'])
    ay = np.array(rows['ay']); az = np.array(rows['az'])
    att_t = np.array(magrows['att_t']); roll = np.array(magrows['roll'])
    # take a 30 s in-flight window
    t0 = t[len(t)//2]; sel = (t >= t0) & (t < t0+30)
    tt = t[sel] - t0
    ax_s, ay_s, az_s = ax_[sel], ay[sel], az[sel]
    roll_real = np.interp(t[sel], att_t, roll)

    # supplementary compensation Eq. 11: roll from accelerometer
    phi_acc = np.arctan2(ay_s, np.sqrt(ax_s**2 + az_s**2))
    # LPF the accel-derived roll (Appendix B: low-pass the outputs)
    f = LPF(); phi_acc_f = np.array([f.step(v) for v in phi_acc])

    # all-gyros attack at t=10s: the gyro-integrated roll (software sensor with all
    # gyros gone) drifts. Model it as integrating an attacked roll-rate (bias).
    dt = np.median(np.diff(tt))
    atk = tt >= 10.0
    roll_rate_attacked = np.gradient(roll_real, tt)
    roll_rate_attacked[atk] += 0.5     # all-gyros bias injection (rad/s)
    roll_gyro = roll_real[0] + np.cumsum(roll_rate_attacked)*dt  # drifts under attack

    # combined (supplementary): weighted sum of gyro-integrated and accel roll
    W = 0.1
    roll_comb = np.zeros_like(roll_gyro); roll_comb[0] = roll_real[0]
    for i in range(1, len(tt)):
        pred = roll_comb[i-1] + roll_rate_attacked[i]*dt
        roll_comb[i] = pred + W*(phi_acc_f[i] - pred)   # accel correction

    fig, ax = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    ax[0].axvspan(10, tt[-1], color='red', alpha=0.07, label='all-gyros attack')
    ax[0].plot(tt, roll_real*R2D, 'k-', lw=0.8, label='ground-truth roll')
    ax[0].plot(tt, roll_gyro*R2D, 'b-', lw=0.8, label='software sensor (gyro only)')
    ax[0].set_title('(a) without compensation'); ax[0].set_ylabel('roll (deg)')
    ax[0].set_xlabel('time (s)'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].axvspan(10, tt[-1], color='red', alpha=0.07, label='all-gyros attack')
    ax[1].plot(tt, roll_real*R2D, 'k-', lw=0.8, label='ground-truth roll')
    ax[1].plot(tt, roll_comb*R2D, 'b-', lw=0.8, label='software sensor + accel compensation')
    ax[1].set_title('(b) with supplementary compensation (Eq. 11)')
    ax[1].set_xlabel('time (s)'); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    fig.suptitle('All-gyroscopes attack recovery and compensation (paper Fig. 15)')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig15_allgyro_compensation.png', dpi=120)
    print('wrote fig15_allgyro_compensation.png')


def main():
    A, B, C, D, Useg, Yseg = load()
    N_, TON, TOFF = parse_params()
    fig12(A, B, C, D, Useg, Yseg, N_, TON)
    fig13(A, B, C, D, Useg, Yseg, N_, TON)
    fig14(A, B, C, D, Useg, Yseg, N_, TON)
    fig15()
    print(f'\nAll figures in: {OUT}')


if __name__ == '__main__':
    main()
