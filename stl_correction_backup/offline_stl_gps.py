#!/usr/bin/env python3
"""
Step 8: Offline STL monitor for GPS Integrity.

Project path:
  /home/tchowdh4/paperImp/

Interpreter:
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Formula:
  φ_gps = G[0:580ms] (gps_north_res < 0.169349 and gps_east_res < 0.169349)

Dataset:
  /home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat
"""

import numpy as np
import scipy.io
import rtamt
import matplotlib
matplotlib.use('Agg')   # headless-safe: avoid blocking # plt.show() removed: blocks on headless machines. Plot is saved to PNG above. on machines without a display
import matplotlib.pyplot as plt


DATASET = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_gps.png"

EPS_GPS = 0.169349
WINDOW_MS = 580

ATTACK_START = 2000
ATTACK_END = 2500
GPS_EAST_ATTACK_OFFSET_M = 20.0


def extract_rho_values(rtamt_result):
    """
    Convert rtamt evaluate() output into a 1D numpy array of robustness values.
    This keeps the STL logic unchanged and only handles rtamt output format.
    """
    if isinstance(rtamt_result, list):
        if len(rtamt_result) > 0 and isinstance(rtamt_result[0], (list, tuple)):
            return np.array([float(row[1]) for row in rtamt_result], dtype=float)
        return np.array(rtamt_result, dtype=float)

    arr = np.array(rtamt_result)

    if arr.ndim == 2 and arr.shape[1] >= 2:
        return arr[:, 1].astype(float)

    return arr.astype(float).reshape(-1)


print("Loading dataset:")
print(DATASET)

d = scipy.io.loadmat(DATASET)

Yseg = d["Yseg"][0]
Useg = d["Useg"][0]
EXTRAseg = d["EXTRAseg"][0]
Ts = float(d["Ts"].flat[0])
fs = float(d["fs"].flat[0])

seg_idx = 0
Y = Yseg[seg_idx]
U = Useg[seg_idx]
EXTR = EXTRAseg[seg_idx]
N = Y.shape[0]

print("\nDataset loaded successfully")
print(f"Segment index: {seg_idx}")
print(f"Y shape: {Y.shape}")
print(f"U shape: {U.shape}")
print(f"EXTRA shape: {EXTR.shape}")
print(f"Sampling time Ts: {Ts}")
print(f"Sampling frequency fs: {fs}")

# Guide channel layout:
# Yseg[i]: 0:pN, 1:pE
pN_model = Y[:, 0]
pE_model = Y[:, 1]

# Guide channel layout:
# EXTRAseg[i]: 5:GPS_Lat, 6:GPS_Lng
gps_lat = EXTR[:, 5]
gps_lng = EXTR[:, 6]

# The guide's GPS STL formula needs GPS_North and GPS_East.
# The dataset gives GPS_Lat and GPS_Lng, so convert them to local meters.
# Use first GPS sample as local origin.
lat0 = gps_lat[0]
lng0 = gps_lng[0]

meters_per_deg_lat = 111320.0
meters_per_deg_lng = 111320.0 * np.cos(np.deg2rad(lat0))

GPS_North = (gps_lat - lat0) * meters_per_deg_lat
GPS_East = (gps_lng - lng0) * meters_per_deg_lng

# Align model pN/pE to the same local origin.
pN_model_local = pN_model - pN_model[0]
pE_model_local = pE_model - pE_model[0]

gps_north_res = np.abs(GPS_North - pN_model_local)
gps_east_res = np.abs(GPS_East - pE_model_local)

# Simulated GPS position attack following the paper GPS case idea:
# corrupt longitude / east-position information by a position offset.
GPS_East_attacked = GPS_East.copy()
GPS_East_attacked[ATTACK_START:ATTACK_END] += GPS_EAST_ATTACK_OFFSET_M

gps_north_res_attacked = gps_north_res.copy()
gps_east_res_attacked = np.abs(GPS_East_attacked - pE_model_local)

# Time axis
t_sec = np.arange(N) * Ts
t_ms = (np.arange(N) * 1000 * Ts).astype(int)

# STL specification
spec = rtamt.StlDiscreteTimeSpecification()
spec.declare_var("gps_north_res", "float")
spec.declare_var("gps_east_res", "float")
spec.set_sampling_period(int(Ts * 1000), "ms")

spec.spec = f"G[0:{WINDOW_MS}ms] (gps_north_res < {EPS_GPS:.6f} and gps_east_res < {EPS_GPS:.6f})"
spec.parse()

print(f"\nSTL formula: {spec.spec}")
print(f"GPS threshold epsilon_gps: {EPS_GPS:.6f} m")
print(f"GPS STL window W: {WINDOW_MS} ms")
print(f"Attack window: sample {ATTACK_START} to {ATTACK_END}")
print(f"Attack start time: {t_sec[ATTACK_START]:.2f} s")
print(f"Attack end time: {t_sec[ATTACK_END]:.2f} s")
print(f"GPS East attack offset: {GPS_EAST_ATTACK_OFFSET_M:.2f} m")

# rtamt offline evaluation format used for this installed rtamt version
dataset_clean = {
    "time": t_ms.tolist(),
    "gps_north_res": gps_north_res.tolist(),
    "gps_east_res": gps_east_res.tolist(),
}

dataset_attacked = {
    "time": t_ms.tolist(),
    "gps_north_res": gps_north_res_attacked.tolist(),
    "gps_east_res": gps_east_res_attacked.tolist(),
}

rho_clean = extract_rho_values(spec.evaluate(dataset_clean))
rho_attacked = extract_rho_values(spec.evaluate(dataset_attacked))

print(f"\nClean robustness trace length: {len(rho_clean)}")
print(f"Attacked robustness trace length: {len(rho_attacked)}")

# Align traces defensively in case rtamt returns a shortened horizon at the end.
M = min(N, len(rho_clean), len(rho_attacked))
t_plot = t_sec[:M]
rho_clean = rho_clean[:M]
rho_attacked = rho_attacked[:M]

violations = np.where(rho_attacked < 0)[0]

if len(violations):
    det_idx = int(violations[0])
    print(f"Attack detected at t = {t_plot[det_idx]:.2f} s")
    print(f"Detection sample: {det_idx}")
    print(f"Detection latency = {t_plot[det_idx] - t_sec[ATTACK_START]:.2f} s")
else:
    print("No GPS attack detected")

# Plot
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t_sec, pN_model_local, label="pN model")
axes[0].plot(t_sec, GPS_North, label="GPS North clean", alpha=0.7)
axes[0].plot(t_sec, pE_model_local, label="pE model")
axes[0].plot(t_sec, GPS_East, label="GPS East clean", alpha=0.7)
axes[0].plot(t_sec, GPS_East_attacked, label="GPS East attacked", linestyle="--", alpha=0.8)
axes[0].axvspan(t_sec[ATTACK_START], t_sec[ATTACK_END], alpha=0.1, label="attack window")
axes[0].set_ylabel("Position (m)")
axes[0].legend(fontsize=8)
axes[0].grid(alpha=0.3)

axes[1].plot(t_sec, gps_north_res, label="north residual clean")
axes[1].plot(t_sec, gps_east_res, label="east residual clean")
axes[1].plot(t_sec, gps_east_res_attacked, label="east residual attacked", linestyle="--")
axes[1].axhline(EPS_GPS, linestyle=":", label=f"threshold ε={EPS_GPS:.6f}m")
axes[1].set_ylabel("GPS residual (m)")
axes[1].legend(fontsize=8)
axes[1].grid(alpha=0.3)

axes[2].plot(t_plot, rho_clean, label="ρ clean")
axes[2].plot(t_plot, rho_attacked, label="ρ attacked", linestyle="--")
axes[2].axhline(0, linewidth=1.5, label="ρ = 0 boundary")
if len(violations):
    axes[2].axvline(t_plot[det_idx], linestyle=":", label=f"detection t={t_plot[det_idx]:.2f}s")
axes[2].set_ylabel("Robustness ρ")
axes[2].set_xlabel("Time (s)")
axes[2].legend(fontsize=8)
axes[2].grid(alpha=0.3)

plt.suptitle(
    "STL Robustness — GPS Position Attack Detection\n"
    f"Spec: {spec.spec}",
    fontsize=10
)

plt.tight_layout()
plt.savefig(OUTPUT_PLOT, dpi=130)

print(f"Saved: {OUTPUT_PLOT}")
# plt.show() removed: blocks on headless machines. Plot is saved to PNG above.
