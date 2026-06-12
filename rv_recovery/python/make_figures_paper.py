#!/usr/bin/env python3
"""
Paper-aligned demonstration figures — READ-ONLY (no data is modified).

Generated from the existing artifacts:
  - rv_recovery/data/operation_data_50hz.mat   (U, Y, EXTRA streams; 21 segments)
  - rv_recovery/matlab/models/quadrotor_12state.mat  (A,B,C,D + per-block fit report)
  - rv_recovery/firmware_patch/recovery_params.h     (DTW N, T_on, T_off)

The software-sensor predictions are produced by a Python mirror of the firmware
Algorithm-1 monitor (recovery_monitor.h, unit-tested 25/25): open-loop model
x<-Ax+Bu, output y=Cx+Du, with the §3.3 windowed synchronization. Sensor
conversions use the paper's equations (Eq. 5 baro, Eq. 6 magnetometer heading).

Outputs -> ~/paperImp/rv_recovery/figures/
  fig3_sensor_prediction.png        (paper Fig. 11: GPS/baro/gyro/mag prediction)
  fig4_sysid_validation_fit.png     (system-identification validation fit per axis)
  fig5_model_eigenvalues.png        (open-loop model poles / stability)
  fig6_dtw_parameters.png           (§3.3 window N and thresholds per channel)
  fig7_false_positive_rate.png      (paper Fig. 14a: FP rate vs threshold, clean data)
  fig8_operation_data_overview.png  (§3.1: diverse maneuvers in the operation data)
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
EARTH = 111194.9
LPF_B, LPF_A = butter(2, 5.0/(50.0/2.0))


def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi


def parse_params():
    txt = open(PARAMS).read()
    def arr(name):
        s = txt.split(name)[1].split('{')[1].split('}')[0]
        return [float(v.replace('f','')) for v in s.split(',') if v.strip()]
    return [int(v) for v in arr('REC_N_DEFAULT')], arr('REC_TON_DEFAULT'), arr('REC_TOFF_DEFAULT')


def software_baro(z, P0, T0=288.15):
    g0, Mm, R = 9.87, 0.02896, 8.3143
    return P0 * np.exp(-g0*Mm*z/(R*T0))


def mag_heading(mx, my, mz, phi, theta):
    return np.arctan2(-my*np.cos(phi) + mz*np.sin(phi),
                      mx*np.cos(theta) + my*np.sin(theta)*np.sin(phi)
                      + mz*np.sin(theta)*np.cos(phi))


def model_predict(U, Y, A, B, C, D, windows):
    """Open-loop model + §3.3 windowed sync — software-sensor outputs (clean)."""
    N, NY = U.shape[0], C.shape[0]
    x = Y[0].astype(float).copy()
    lpf = [(_LPF()) for _ in range(NY)]
    e = np.zeros(NY); t = np.zeros(NY, int)
    eh = [np.zeros(w) for w in windows]
    MS = np.zeros((N, NY))
    for n in range(N):
        y = C @ x + D @ U[n]; x = A @ x + B @ U[n]
        for k in range(NY):
            m = lpf[k].step(Y[n, k]); w = windows[k]; t[k] += 1
            ms_raw = y[k]
            if t[k] > w:
                t[k] = 0; e[k] = eh[k].mean(); ms_raw = m; x[k] = m
            eh[k][t[k] % w] = ms_raw - m
            MS[n, k] = ms_raw - e[k]
    return MS


class _LPF:
    def __init__(self): self.w1 = self.w2 = 0.0
    def step(self, x):
        y = LPF_B[0]*x + self.w1
        self.w1 = LPF_B[1]*x - LPF_A[1]*y + self.w2
        self.w2 = LPF_B[2]*x - LPF_A[2]*y
        return y


def main():
    S = scipy.io.loadmat(DATA)
    Mm = scipy.io.loadmat(MODEL)
    A, B, C, D = (Mm[k].astype(float) for k in 'ABCD')
    Useg = [np.asarray(u, float) for u in S['Useg'].ravel()]
    Yseg = [np.asarray(y, float) for y in S['Yseg'].ravel()]
    Eseg = [np.asarray(x, float) for x in S['EXTRAseg'].ravel()]
    N_, TON, TOFF = parse_params()

    seg = 5
    U, Y, E = Useg[seg], Yseg[seg], Eseg[seg]
    n = min(len(U), 4000)
    U, Y, E = U[:n], Y[:n], E[:n]
    tsec = np.arange(n)*TS
    MS = model_predict(U, Y, A, B, C, D, N_)

    # ── Fig 3: sensor prediction (paper Fig. 11) ─────────────────────────────
    # extra cols: 0 BARO_Press,1 BARO_Alt,2 MagX,3 MagY,4 MagZ,5 Lat,6 Lng,7 GPS_Alt,8 Spd
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    # (a) GPS position North: real (from lat) vs model pN
    lat0 = E[0, 5]
    gpsN = (E[:, 5] - lat0) * EARTH
    ax[0,0].plot(tsec, gpsN, 'r-', lw=0.8, label='real GPS (North)')
    ax[0,0].plot(tsec, MS[:, 0], 'b-', lw=0.8, label='software sensor (model pN)')
    ax[0,0].set_title('(a) GPS position — North'); ax[0,0].set_ylabel('m')
    ax[0,0].legend(fontsize=8); ax[0,0].grid(alpha=0.3)
    # (b) barometer pressure: real BARO_Press vs Eq.5(model alt)
    P0 = E[0, 0]
    ms_press = software_baro(MS[:, 2] - MS[0, 2], P0)
    ax[0,1].plot(tsec, E[:, 0], 'r-', lw=0.8, label='real barometer')
    ax[0,1].plot(tsec, ms_press, 'b-', lw=0.8, label='software sensor (Eq. 5)')
    ax[0,1].set_title('(b) Barometer pressure'); ax[0,1].set_ylabel('Pa')
    ax[0,1].legend(fontsize=8); ax[0,1].grid(alpha=0.3)
    # (c) gyroscope roll rate p: real vs model
    ax[1,0].plot(tsec, Y[:, 9], 'r-', lw=0.8, label='real gyroscope (p)')
    ax[1,0].plot(tsec, MS[:, 9], 'b-', lw=0.8, label='software sensor (model p)')
    ax[1,0].set_title('(c) Gyroscope — roll rate'); ax[1,0].set_ylabel('rad/s')
    ax[1,0].set_xlabel('time (s)'); ax[1,0].legend(fontsize=8); ax[1,0].grid(alpha=0.3)
    # (d) magnetometer heading: Eq.6(real mag) vs model psi
    real_head = np.unwrap(mag_heading(E[:, 2], E[:, 3], E[:, 4], Y[:, 3], Y[:, 4]))
    real_head = real_head - real_head[0] + MS[0, 5]
    ax[1,1].plot(tsec, real_head, 'r-', lw=0.8, label='real magnetometer (Eq. 6)')
    ax[1,1].plot(tsec, MS[:, 5], 'b-', lw=0.8, label='software sensor (model psi)')
    ax[1,1].set_title('(d) Magnetometer — heading'); ax[1,1].set_ylabel('rad')
    ax[1,1].set_xlabel('time (s)'); ax[1,1].legend(fontsize=8); ax[1,1].grid(alpha=0.3)
    fig.suptitle('Software-sensor prediction vs. real sensors (paper Fig. 11 analogue)')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig3_sensor_prediction.png', dpi=120)
    print('wrote fig3_sensor_prediction.png')

    # ── Fig 4: system-identification validation fit per axis ─────────────────
    fr = Mm['fitrep']
    names = fr.dtype.names
    val_mean, der_mean = [], []
    for nm in names:
        f = np.asarray(fr[nm][0, 0], float).ravel()
        # per block: experiments x 2 outputs -> mean per output (value, derivative)
        f = f[np.isfinite(f)]
        half = len(f)//2
        val_mean.append(np.mean(f[:half]) if half else np.nan)
        der_mean.append(np.mean(f[half:]) if half else np.nan)
    x = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(x-0.2, val_mean, 0.4, label='value (angle/position)')
    ax.bar(x+0.2, der_mean, 0.4, label='derivative (rate/velocity)')
    ax.set_xticks(x); ax.set_xticklabels(names)
    ax.set_ylabel('open-loop sim fit (NRMSE %)')
    ax.set_title('System-identification validation fit per axis (PEM, K=0)')
    ax.legend(); ax.grid(alpha=0.3, axis='y'); ax.axhline(0, color='k', lw=0.5)
    fig.tight_layout(); fig.savefig(f'{OUT}/fig4_sysid_validation_fit.png', dpi=120)
    print('wrote fig4_sysid_validation_fit.png')

    # ── Fig 5: open-loop model eigenvalues (poles) ───────────────────────────
    ev = np.linalg.eigvals(A)
    fig, ax = plt.subplots(figsize=(6, 6))
    th = np.linspace(0, 2*np.pi, 200)
    ax.plot(np.cos(th), np.sin(th), 'k--', lw=0.8, label='unit circle')
    ax.scatter(ev.real, ev.imag, c='b', marker='x', s=60, label='model poles')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('Re'); ax.set_ylabel('Im')
    ax.set_title(f'Identified model poles (spectral radius = {max(abs(ev)):.3f})')
    ax.legend()
    fig.tight_layout(); fig.savefig(f'{OUT}/fig5_model_eigenvalues.png', dpi=120)
    print('wrote fig5_model_eigenvalues.png')

    # ── Fig 6: DTW window N and thresholds per channel (§3.3) ─────────────────
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
    x = np.arange(len(CH))
    a1.bar(x, np.array(N_)*TS*1000, color='teal')
    a1.set_xticks(x); a1.set_xticklabels(CH, rotation=45)
    a1.set_ylabel('window N (ms)'); a1.set_title('DTW window size per channel (§3.3)')
    a1.grid(alpha=0.3, axis='y')
    a2.bar(x-0.2, TON, 0.4, label='T_on', color='orange')
    a2.bar(x+0.2, TOFF, 0.4, label='T_off', color='purple')
    a2.set_xticks(x); a2.set_xticklabels(CH, rotation=45); a2.set_yscale('log')
    a2.set_ylabel('threshold (log)'); a2.set_title('Recovery thresholds  T = e_max + margin')
    a2.legend(); a2.grid(alpha=0.3, axis='y')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig6_dtw_parameters.png', dpi=120)
    print('wrote fig6_dtw_parameters.png')

    # ── Fig 7: false-positive rate vs threshold on clean data (paper Fig. 14a) ─
    # For each channel, sweep a threshold scale and count fraction of clean windows
    # whose accumulated residual exceeds it. At the selected T_on, FP should be ~0.
    fig, ax = plt.subplots(figsize=(8, 5))
    scales = np.linspace(0.1, 1.5, 25)
    for k, nm in [(9, 'p'), (10, 'q'), (11, 'r'), (3, 'phi')]:
        # accumulate per-window residuals across all segments
        res_windows = []
        for U2, Y2 in zip(Useg, Yseg):
            ms = model_predict(U2, Y2, A, B, C, D, N_)
            w = N_[k]
            r = np.abs(Y2[:, k] - ms[:, k])
            nb = len(r)//w
            for b in range(nb):
                res_windows.append(r[b*w:(b+1)*w].sum())
        res_windows = np.array(res_windows)
        fp = [np.mean(res_windows > TON[k]*s) for s in scales]
        ax.plot(scales, fp, marker='.', label=nm)
    ax.axvline(1.0, color='r', ls='--', label='selected T_on')
    ax.set_xlabel('threshold / selected T_on'); ax.set_ylabel('false-positive rate (clean data)')
    ax.set_title('False-positive rate vs. threshold (paper Fig. 14a analogue)')
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f'{OUT}/fig7_false_positive_rate.png', dpi=120)
    print('wrote fig7_false_positive_rate.png')

    # ── Fig 8: operation-data maneuver overview (§3.1) ────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axes[0].plot(tsec, Y[:, 3], label='roll φ'); axes[0].plot(tsec, Y[:, 4], label='pitch θ')
    axes[0].plot(tsec, Y[:, 5], label='yaw ψ'); axes[0].set_ylabel('attitude (rad)')
    axes[1].plot(tsec, Y[:, 0], label='pN'); axes[1].plot(tsec, Y[:, 1], label='pE')
    axes[1].plot(tsec, Y[:, 2], label='alt'); axes[1].set_ylabel('position (m)')
    axes[2].plot(tsec, Y[:, 6], label='vN'); axes[2].plot(tsec, Y[:, 7], label='vE')
    axes[2].plot(tsec, Y[:, 8], label='vUp'); axes[2].set_ylabel('velocity (m/s)')
    axes[3].plot(tsec, Y[:, 9], label='p'); axes[3].plot(tsec, Y[:, 10], label='q')
    axes[3].plot(tsec, Y[:, 11], label='r'); axes[3].set_ylabel('body rates (rad/s)')
    axes[3].set_xlabel('time (s)')
    for a in axes: a.legend(fontsize=8, ncol=3); a.grid(alpha=0.3)
    axes[0].set_title('Operation data — one mission across diverse maneuvers (§3.1)')
    fig.tight_layout(); fig.savefig(f'{OUT}/fig8_operation_data_overview.png', dpi=120)
    print('wrote fig8_operation_data_overview.png')

    print(f'\nAll figures in: {OUT}')


if __name__ == '__main__':
    main()
