#!/usr/bin/env python3
"""
faithful_core.py  —  shared core for the paper-faithful STL attack detector.

Faithful to: Choi et al., "Software-based Realtime Recovery from Sensor Attacks on
Robotic Vehicles" (RAID 2020), Algorithm 1 (Runtime Recovery Monitoring) and §3.3
(parameter selection).

What this module provides (used by both the offline and online detectors):

  1. The software sensor:  y = C x + D u ;  x = A x + B u          (paper Eq. 1-2)
     with the paper's per-window model re-seed (x[k] <- m) and error-compensation
     term e, and a Butterworth low-pass filter on the real measurement m.
     -> this reproduces predict_software_sensors() / recovery_monitor.h line for line.

  2. The paper's DETECTION STATISTIC — the windowed accumulated residual (paper §3.3):
        R_{k,N}(t) = sum_{i=t-N+1..t} | m_k(i) - ms_k(i) |
     computed as a *sliding* window of size N_k (DTW-derived, from recovery_params.npy).
     (§3.3 says "the maximum accumulated error within any window of size N"; a sliding
      window is the faithful reading and is what makes the T_off switch-back work.)

  3. The paper's THRESHOLDS (§3.3):  T_on = e_max + margin,  T_off = TOFF_FRAC * T_on,
     where e_max = max over clean-data windows of R_{k,N}.  margin = 10% ; K = 10.

STL itself cannot compute a windowed SUM (its temporal operators are min/max, not sum),
so the accumulation R is computed HERE (the paper's algorithm) and the STL layer in the
offline/online scripts monitors it as  G (R < T_on)  ->  rho = T_on - R.  rho < 0 is
exactly the paper's detection rule  R > T_on.
"""
import os
import numpy as np
import scipy.io
from scipy.signal import butter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # /home/tchowdh4/paperImp
DATA = os.path.join(ROOT, 'rv_recovery/data/operation_data_50hz.mat')
MDL  = os.path.join(ROOT, 'rv_recovery/matlab/models/quadrotor_12state.mat')
RPARAMS = os.path.join(ROOT, 'rv_recovery/data/recovery_params.npy')

TS = 0.02                                         # 50 Hz
CH_NAMES = ['pN', 'pE', 'alt', 'phi', 'theta', 'psi', 'vN', 'vE', 'vUp', 'p', 'q', 'r']

MARGIN_FRAC = 0.10        # T_on = e_max * (1 + margin)      (paper §3.3, documented choice)
TOFF_FRAC   = 0.80        # T_off = 0.80 * T_on              (T_off < T_on)
K_SAFE      = 10          # consecutive safe samples to leave recovery (paper K)

# same 2nd-order Butterworth filter as recovery_monitor.h / select_parameters.py
LPF_B, LPF_A = butter(2, 5.0 / (50.0 / 2.0))


class LPF:
    """Direct-form II transposed biquad — mirrors lpf_step() in recovery_monitor.h."""
    def __init__(self):
        self.w1 = 0.0
        self.w2 = 0.0

    def step(self, x):
        y = LPF_B[0] * x + self.w1
        self.w1 = LPF_B[1] * x - LPF_A[1] * y + self.w2
        self.w2 = LPF_B[2] * x - LPF_A[2] * y
        return y


def load_model():
    m = scipy.io.loadmat(MDL)
    return (m['A'].astype(float), m['B'].astype(float),
            m['C'].astype(float), m['D'].astype(float))


def load_segment(seg=0):
    d = scipy.io.loadmat(DATA)
    Y = d['Yseg'][0][seg].astype(float)
    U = d['Useg'][0][seg].astype(float)
    EX = d['EXTRAseg'][0][seg].astype(float)
    return Y, U, EX


def load_windows():
    """Per-channel window size N (samples), DTW-derived, from recovery_params.npy (paper §3.3)."""
    rp = np.load(RPARAMS, allow_pickle=True).item()
    return np.array([p[0] for p in rp['per_sensor']], dtype=int)


def instantaneous_residual(Ymeas, U, windows):
    """
    Paper Algorithm 1 software sensor, mirror of predict_software_sensors().
    Returns Dtr[N,12] = per-sample instantaneous error |m - ms| for every channel.
    `Ymeas` is the (possibly attacked) measurement; with C = I, m_k = Ymeas[:,k].
    """
    A, B, C, D = load_model()
    N = Ymeas.shape[0]
    NY = C.shape[0]
    x = Ymeas[0].astype(float).copy()             # init model state from first measurement
    lpf = [LPF() for _ in range(NY)]
    e = np.zeros(NY)
    t = np.zeros(NY, dtype=int)
    err_hist = [np.zeros(w) for w in windows]
    Dtr = np.zeros((N, NY))
    for n in range(N):
        y = C @ x + D @ U[n]                       # y = C x + D u   (software sensor, pre-advance)
        x = A @ x + B @ U[n]                       # x = A x + B u
        for k in range(NY):
            m = lpf[k].step(Ymeas[n, k])          # m <- filter(m)
            ms_raw = y[k]
            t[k] += 1
            if t[k] > windows[k]:                  # per-window checkpoint (while healthy)
                t[k] = 0
                e[k] = err_hist[k].mean()          # e <- error_estimation(prev window)
                ms_raw = m                         # ms <- m  (sync)
                x[k] = m                            # re-seed model state (C = I)
            ms = ms_raw - e[k]                      # ms <- ms - e   (compensation)
            err_hist[k][t[k] % windows[k]] = ms_raw - m
            Dtr[n, k] = abs(m - ms)                # instantaneous |m - ms|
    return Dtr


def sliding_R(Dtr, k, Nk):
    """Windowed accumulated residual R_{k,N}(t) = sum of |m-ms| over the last N_k samples."""
    c = np.cumsum(Dtr[:, k])
    R = c.copy()
    R[Nk:] = c[Nk:] - c[:-Nk]
    return R


def select_thresholds(monitored, seg=0):
    """
    Paper §3.3 threshold selection on CLEAN data.
    Returns {k: dict(N, e_max, T_on, T_off, name)} for each channel k in `monitored`.
    e_max is recomputed with the SAME sliding-window statistic used at runtime, so the
    thresholds are self-consistent with detection (this is the faithful T = e_max + margin).
    """
    Y, U, _ = load_segment(seg)
    windows = load_windows()
    Dclean = instantaneous_residual(Y, U, windows)
    out = {}
    for k in monitored:
        Nk = int(windows[k])
        R = sliding_R(Dclean, k, Nk)
        e_max = float(R[Nk:].max()) if len(R) > Nk else float(R.max())
        T_on = e_max * (1.0 + MARGIN_FRAC)
        T_off = T_on * TOFF_FRAC
        out[k] = dict(N=Nk, e_max=e_max, T_on=T_on, T_off=T_off, name=CH_NAMES[k])
    return out


def inject_attack(Y, k, kind, value, start, end):
    """Offline attack simulation ONLY: corrupt physical measurement m_k in [start:end).
       kind='bias' adds `value`; kind='set' overwrites with `value`.  Returns a copy."""
    Ya = Y.copy()
    if kind == 'bias':
        Ya[start:end, k] += value
    elif kind == 'set':
        Ya[start:end, k] = value
    else:
        raise ValueError(kind)
    return Ya


def run_monitor(Ymeas, U, windows, params, seg_note=''):
    """
    FULL Algorithm 1 runtime monitor (stateful), recovery-aware.

    Faithful to Choi et al. Algorithm 1: software sensor + LPF + per-window checkpoint
    re-seed + error compensation e, PLUS the crucial rule that the checkpoint re-seed is
    SUPPRESSED while a channel is in recovery (Alg. 1: "if not recovery and t>window").
    This stops a sustained attack from being absorbed into the model, so the windowed
    residual R stays high and detection/recovery persist while the attack persists.

    `params` = {k: {N,T_on,T_off}} for the monitored channels.
    Returns dict per monitored channel k:
        R      : sliding accumulated residual trace  R_{k,N}(t) = Σ_last-N |m-ms|
        rec    : recovery on/off trace (bool)
        det    : first detection index (R>T_on) or None
        m_out  : output measurement (m<-ms while in recovery) — the recovered signal
    """
    A, B, C, D = load_model()
    N = Ymeas.shape[0]
    NY = C.shape[0]
    x = Ymeas[0].astype(float).copy()
    lpf = [LPF() for _ in range(NY)]
    e = np.zeros(NY)
    t = np.zeros(NY, dtype=int)
    err_hist = [np.zeros(w) for w in windows]

    mon = list(params.keys())
    # sliding-sum ring buffers for the monitored channels
    buf = {k: np.zeros(params[k]['N']) for k in mon}
    bpos = {k: 0 for k in mon}
    Rrun = {k: 0.0 for k in mon}
    recovery = {k: False for k in mon}
    safe = {k: 0 for k in mon}
    det = {k: None for k in mon}
    R_tr = {k: np.zeros(N) for k in mon}
    rec_tr = {k: np.zeros(N, dtype=bool) for k in mon}
    mout_tr = {k: np.zeros(N) for k in mon}

    for n in range(N):
        y = C @ x + D @ U[n]
        x = A @ x + B @ U[n]
        for k in range(NY):
            m = lpf[k].step(Ymeas[n, k])
            ms_raw = y[k]
            t[k] += 1
            in_recovery = recovery.get(k, False)
            if t[k] > windows[k] and not in_recovery:      # checkpoint (suppressed in recovery)
                t[k] = 0
                e[k] = err_hist[k].mean()
                ms_raw = m
                x[k] = m
            ms = ms_raw - e[k]
            err_hist[k][t[k] % windows[k]] = ms_raw - m
            d = abs(m - ms)                                 # instantaneous |m - ms|

            if k in params:                                 # monitored channel
                Nk = params[k]['N']
                Rrun[k] += d - buf[k][bpos[k]]              # O(1) sliding sum update
                buf[k][bpos[k]] = d
                bpos[k] = (bpos[k] + 1) % Nk
                R = Rrun[k]
                R_tr[k][n] = R
                if R > params[k]['T_on'] and not recovery[k]:   # STL detection rule R>T_on
                    recovery[k] = True
                    safe[k] = 0
                    if det[k] is None:
                        det[k] = n
                m_out = m
                if recovery[k]:
                    m_out = ms                              # sensor replacement m <- ms
                    safe[k] = safe[k] + 1 if R < params[k]['T_off'] else 0
                    if safe[k] > K_SAFE:
                        recovery[k] = False
                rec_tr[k][n] = recovery[k]
                mout_tr[k][n] = m_out
    return {k: dict(R=R_tr[k], rec=rec_tr[k], det=det[k], m_out=mout_tr[k]) for k in mon}


class FaithfulMonitor:
    """
    Stateful, per-sample version of run_monitor for ONLINE use (identical Algorithm 1
    logic).  Call step(u, y_meas) once per incoming sample.
    """
    def __init__(self, windows, params):
        self.A, self.B, self.C, self.D = load_model()
        self.NY = self.C.shape[0]
        self.windows = windows
        self.params = params
        self.x = None
        self.lpf = [LPF() for _ in range(self.NY)]
        self.e = np.zeros(self.NY)
        self.t = np.zeros(self.NY, dtype=int)
        self.err_hist = [np.zeros(w) for w in windows]
        mon = list(params.keys())
        self.buf = {k: np.zeros(params[k]['N']) for k in mon}
        self.bpos = {k: 0 for k in mon}
        self.Rrun = {k: 0.0 for k in mon}
        self.recovery = {k: False for k in mon}
        self.safe = {k: 0 for k in mon}
        self.n = 0

    def step(self, u, y_meas):
        if self.x is None:
            self.x = np.asarray(y_meas, float).copy()      # init state from first measurement
        u = np.asarray(u, float)
        y = self.C @ self.x + self.D @ u
        self.x = self.A @ self.x + self.B @ u
        out = {}
        for k in range(self.NY):
            m = self.lpf[k].step(y_meas[k])
            ms_raw = y[k]
            self.t[k] += 1
            in_rec = self.recovery.get(k, False)
            if self.t[k] > self.windows[k] and not in_rec:
                self.t[k] = 0
                self.e[k] = self.err_hist[k].mean()
                ms_raw = m
                self.x[k] = m
            ms = ms_raw - self.e[k]
            self.err_hist[k][self.t[k] % self.windows[k]] = ms_raw - m
            d = abs(m - ms)
            if k in self.params:
                Nk = self.params[k]['N']
                self.Rrun[k] += d - self.buf[k][self.bpos[k]]
                self.buf[k][self.bpos[k]] = d
                self.bpos[k] = (self.bpos[k] + 1) % Nk
                R = self.Rrun[k]
                if R > self.params[k]['T_on'] and not self.recovery[k]:
                    self.recovery[k] = True
                    self.safe[k] = 0
                m_out = m
                if self.recovery[k]:
                    m_out = ms
                    self.safe[k] = self.safe[k] + 1 if R < self.params[k]['T_off'] else 0
                    if self.safe[k] > K_SAFE:
                        self.recovery[k] = False
                out[k] = dict(R=R, rec=self.recovery[k], m_out=m_out, m=m, ms=ms)
        self.n += 1
        return out


def recovery_fsm(R, T_on, T_off, k_safe=K_SAFE):
    """
    Paper Algorithm 1 recovery state machine driven by the STL detection (R > T_on).
    Returns (recovery_bool_trace, detect_index or None).
      enter recovery when R > T_on ; leave after k_safe consecutive samples with R < T_off.
    (During recovery the real sensor would be replaced by the software sensor: m <- ms.)
    """
    n = len(R)
    rec = np.zeros(n, dtype=bool)
    on = False
    safe = 0
    det = None
    for i in range(n):
        if R[i] > T_on and not on:
            on = True
            safe = 0
            if det is None:
                det = i
        if on:
            safe = safe + 1 if R[i] < T_off else 0
            if safe > k_safe:
                on = False
        rec[i] = on
    return rec, det


# ── Monitored channels and their paper attack cases (baro / gyro / GPS) ──────────
#   value = attack magnitude ; kind = how the physical measurement is corrupted.
ATTACK_CASES = {
    2: dict(name='alt (barometer)', kind='bias', value=3.0,  unit='m',    eps_label='baro'),
    9: dict(name='p (gyroscope)',   kind='set',  value=0.8,  unit='rad/s',eps_label='gyro'),
    1: dict(name='pE (GPS east)',   kind='bias', value=20.0, unit='m',    eps_label='gps'),
}
