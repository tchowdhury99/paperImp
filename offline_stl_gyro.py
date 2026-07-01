#!/usr/bin/env python3
"""
Offline STL monitor for gyroscope rate attack detection.

Interpreter:
    /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Dataset:
    /home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat

Formula:
    G[0:580ms] (
        gyro_residual_x < 0.15
        and gyro_residual_y < 0.15
        and gyro_residual_z < 0.15
    )

Compatibility note:
    This installed rtamt version does not support bounded always
    through the online spec.update() monitor.

    Therefore this script uses the offline spec.evaluate() monitor.
    This is only an execution-level compatibility correction.
    It does not change the STL formula, threshold, dataset, attack,
    attack window, or workflow.
"""

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import rtamt


# ── 1. Paths ────────────────────────────────────────────────────────────────
DATA_PATH = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_gyro.png"


# ── 2. Fixed guide parameters ───────────────────────────────────────────────
SEG_IDX = 0

GYR_X_COL = 9     # Y[:, 9]  = p, roll rate, rad/s
GYR_Y_COL = 10    # Y[:, 10] = q, pitch rate, rad/s
GYR_Z_COL = 11    # Y[:, 11] = r, yaw rate, rad/s

EPS_GYR = 0.15    # rad/s

# Paper example window: 575ms.
# Smallest compatibility correction for 50 Hz dataset:
# 580ms = 29 samples × 20ms.
WINDOW_MS = 580

ATTACK_START = 2000
ATTACK_END = 2500
GYR_X_ATTACK_VALUE = 0.8    # rad/s


# ── 3. RTAMT compatibility helper ───────────────────────────────────────────
def make_discrete_spec():
    """
    Use the guide's RTAMT class name first.
    If the installed rtamt package uses alternate capitalization,
    fall back to that class only as a package-version compatibility correction.
    """
    if hasattr(rtamt, "STLDiscreteTimeSpecification"):
        return rtamt.STLDiscreteTimeSpecification()

    if hasattr(rtamt, "StlDiscreteTimeSpecification"):
        return rtamt.StlDiscreteTimeSpecification()

    raise AttributeError(
        "Could not find STLDiscreteTimeSpecification or "
        "StlDiscreteTimeSpecification in installed rtamt."
    )


# ── 4. Load dataset exactly from the guide structure ─────────────────────────
d = scipy.io.loadmat(DATA_PATH)

Yseg = d["Yseg"][0]
Ts = float(d["Ts"].flat[0])
fs = float(d["fs"].flat[0])

Y = Yseg[SEG_IDX]
N = Y.shape[0]

t_sec = np.arange(N) * Ts
t_ms = (np.arange(N) * Ts * 1000).astype(int)


# ── 5. Software-sensor prediction ms_gyr (paper §6.2: m_gyro,s = [p q r]) ─────
# Paper-aligned residual:  gyro_error_j(t) = |m_gyr_j(t) - ms_gyr_j(t)|
#   m_gyr_j  = actual physical gyroscope measurement (roll/pitch/yaw rate)
#   ms_gyr_j = software-sensor prediction = model angular-rate state [p q r]
# Dataset approximation:  ms_gyr = model states Y[:,9:12]. The dataset has no
# separate raw-gyro channel distinct from the model rate state, so the clean
# physical measurement m_gyr is approximated by the same model-state rate on
# clean data (=> clean residual ~ 0), and is corrupted during the attack window.
# STL below uses the INSTANTANEOUS error |m - ms| (guide-based simplification of
# the paper's accumulated residual r <- r + |m - ms|).
gyr_x_predicted = Y[:, GYR_X_COL].astype(float)    # ms_gyr_x = p / roll rate
gyr_y_predicted = Y[:, GYR_Y_COL].astype(float)    # ms_gyr_y = q / pitch rate
gyr_z_predicted = Y[:, GYR_Z_COL].astype(float)    # ms_gyr_z = r / yaw rate
ms_gyr_x, ms_gyr_y, ms_gyr_z = gyr_x_predicted, gyr_y_predicted, gyr_z_predicted

m_gyr_x_clean = ms_gyr_x.copy()                    # clean physical measurement
m_gyr_y_clean = ms_gyr_y.copy()
m_gyr_z_clean = ms_gyr_z.copy()
gyr_x_measured_clean = m_gyr_x_clean               # aliases kept for downstream
gyr_y_measured_clean = m_gyr_y_clean
gyr_z_measured_clean = m_gyr_z_clean


# ── 6. Offline attack simulation ONLY: corrupt physical measurement m_gyr_x ───
# The attacked variable only models a compromised gyroscope corrupting m_gyr_x(t)
# during the attack window; it is not a theoretical residual variable.
m_gyr_x = m_gyr_x_clean.copy()
m_gyr_y = m_gyr_y_clean.copy()
m_gyr_z = m_gyr_z_clean.copy()
m_gyr_x[ATTACK_START:ATTACK_END] = GYR_X_ATTACK_VALUE   # roll-rate spoof ~0.8 rad/s
gyr_x_measured_attacked = m_gyr_x                       # aliases kept for plotting
gyr_y_measured_attacked = m_gyr_y
gyr_z_measured_attacked = m_gyr_z


# ── 7. Compute residuals  gyro_error_j = |m_gyr_j - ms_gyr_j| ─────────────────
gyro_residual_x_clean = np.abs(m_gyr_x_clean - ms_gyr_x)
gyro_residual_y_clean = np.abs(m_gyr_y_clean - ms_gyr_y)
gyro_residual_z_clean = np.abs(m_gyr_z_clean - ms_gyr_z)

gyro_residual_x_attacked = np.abs(m_gyr_x - ms_gyr_x)
gyro_residual_y_attacked = np.abs(m_gyr_y - ms_gyr_y)
gyro_residual_z_attacked = np.abs(m_gyr_z - ms_gyr_z)


# ── 8. Define STL spec ──────────────────────────────────────────────────────
def build_spec():
    spec = make_discrete_spec()

    spec.declare_var("gyro_residual_x", "float")
    spec.declare_var("gyro_residual_y", "float")
    spec.declare_var("gyro_residual_z", "float")

    spec.set_sampling_period(int(Ts * 1000), "ms")

    spec.spec = (
        "G[0:580ms] ("
        "gyro_residual_x < 0.15 and "
        "gyro_residual_y < 0.15 and "
        "gyro_residual_z < 0.15"
        ")"
    )

    spec.parse()
    return spec


# ── 9. Offline robustness evaluation ────────────────────────────────────────
def evaluate_offline(rx, ry, rz):
    """
    Offline evaluation is used because this rtamt version does not support
    bounded always through online spec.update().

    Compatibility correction:
    This installed rtamt offline evaluator expects a separate 'time' key
    and each variable as a value list, not as list(zip(time, value)).
    """

    spec = build_spec()

    dataset = {
        "time": t_ms.astype(float).tolist(),
        "gyro_residual_x": rx.astype(float).tolist(),
        "gyro_residual_y": ry.astype(float).tolist(),
        "gyro_residual_z": rz.astype(float).tolist(),
    }

    rho = spec.evaluate(dataset)
    rho = np.array(rho, dtype=float)

    # Handle both possible rtamt output formats:
    # 1. [[time, robustness], ...]
    # 2. [robustness0, robustness1, ...]
    if rho.ndim == 2 and rho.shape[1] >= 2:
        rho_time_ms = rho[:, 0]
        rho_values = rho[:, 1]
    elif rho.ndim == 1:
        rho_time_ms = t_ms[:len(rho)].astype(float)
        rho_values = rho
    else:
        raise ValueError(f"Unexpected rtamt robustness output shape: {rho.shape}")

    return rho_time_ms, rho_values


rho_time_ms_clean, rho_clean = evaluate_offline(
    gyro_residual_x_clean,
    gyro_residual_y_clean,
    gyro_residual_z_clean,
)

rho_time_ms_attacked, rho_attacked = evaluate_offline(
    gyro_residual_x_attacked,
    gyro_residual_y_attacked,
    gyro_residual_z_attacked,
)

rho_time_sec = rho_time_ms_attacked / 1000.0


# ── 10. Find detection point ────────────────────────────────────────────────
violations = np.where(rho_attacked < 0)[0]

if len(violations):
    det_array_idx = int(violations[0])
    det_time = float(rho_time_sec[det_array_idx])
    det_sample = int(round(det_time / Ts))
else:
    det_array_idx = None
    det_time = None
    det_sample = None


# ── 11. Plot result ─────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t_sec, gyr_x_predicted, label="GyrX predicted / p model")
axes[0].plot(t_sec, gyr_x_measured_clean, label="GyrX measured clean", alpha=0.7)
axes[0].plot(
    t_sec,
    gyr_x_measured_attacked,
    label="GyrX measured attacked",
    linestyle="--",
    alpha=0.8,
)
axes[0].axvspan(
    t_sec[ATTACK_START],
    t_sec[ATTACK_END],
    alpha=0.15,
    label="attack window",
)
axes[0].set_ylabel("Roll rate p (rad/s)")
axes[0].legend(fontsize=8)
axes[0].grid(alpha=0.3)

axes[1].plot(t_sec, gyro_residual_x_clean, label="gyro_residual_x clean")
axes[1].plot(
    t_sec,
    gyro_residual_x_attacked,
    label="gyro_residual_x attacked",
    linestyle="--",
)
axes[1].plot(t_sec, gyro_residual_y_attacked, label="gyro_residual_y attacked")
axes[1].plot(t_sec, gyro_residual_z_attacked, label="gyro_residual_z attacked")
axes[1].axhline(EPS_GYR, linestyle=":", label="threshold ε_gyr=0.15 rad/s")
axes[1].set_ylabel("Gyro residual (rad/s)")
axes[1].legend(fontsize=8)
axes[1].grid(alpha=0.3)

axes[2].plot(rho_time_sec, rho_clean, label="ρ clean")
axes[2].plot(rho_time_sec, rho_attacked, label="ρ attacked", linestyle="--")
axes[2].axhline(0, linewidth=1.5, label="ρ = 0 boundary")

if det_time is not None:
    axes[2].axvline(
        det_time,
        linestyle=":",
        label=f"detection t={det_time:.2f}s",
    )

axes[2].set_ylabel("Robustness ρ")
axes[2].set_xlabel("Time (s)")
axes[2].legend(fontsize=8)
axes[2].grid(alpha=0.3)

plt.suptitle(
    "OFFLINE STL — Gyroscope Rate Attack Detection (recorded 50 Hz dataset)\n"
    "Spec: G[0:580ms] "
    "(gyro_residual_x < 0.15 and gyro_residual_y < 0.15 and gyro_residual_z < 0.15)",
    fontsize=10,
)

plt.tight_layout()
plt.savefig(OUTPUT_PLOT, dpi=130)
plt.close()


# ── 12. Terminal report ─────────────────────────────────────────────────────
print(f"Loaded dataset: {DATA_PATH}")
print(f"Using segment: {SEG_IDX}")
print("Using Y[:, 9]  as p / GyrX roll rate")
print("Using Y[:, 10] as q / GyrY pitch rate")
print("Using Y[:, 11] as r / GyrZ yaw rate")
print(f"Y shape: {Y.shape}")
print(f"Sampling period Ts: {Ts:.6f} s")
print(f"Sampling rate fs: {fs:.2f} Hz")
print(f"Attack samples: {ATTACK_START} to {ATTACK_END}")
print(f"Attack time window: {t_sec[ATTACK_START]:.2f} s to {t_sec[ATTACK_END]:.2f} s")
print(f"Gyroscope roll-rate attack value: {GYR_X_ATTACK_VALUE} rad/s")
print(f"Threshold epsilon_gyr: {EPS_GYR} rad/s")
print(
    "STL formula: "
    "G[0:580ms] "
    "(gyro_residual_x < 0.15 and gyro_residual_y < 0.15 and gyro_residual_z < 0.15)"
)
print("RTAMT compatibility correction: used offline spec.evaluate(), not online spec.update()")
print(f"Saved: {OUTPUT_PLOT}")

if det_time is None:
    print("No violation detected")
else:
    print(f"Attack detected at t = {det_time:.2f} s  (sample {det_sample})")
    print(f"Attack started at t = {t_sec[ATTACK_START]:.2f} s")
    print(f"Detection latency  = {det_time - t_sec[ATTACK_START]:.2f} s")
