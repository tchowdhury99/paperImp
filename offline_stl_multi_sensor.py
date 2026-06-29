import os
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import rtamt


# ============================================================
# Paths and fixed guide parameters
# ============================================================

PROJECT_DIR = "/home/tchowdh4/paperImp"
DATASET_PATH = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_multi_sensor.png"

SEGMENT_INDEX = 0

Ts = 0.02
fs = 50

ATTACK_START = 2000
ATTACK_END = 2500

ATTACK_START_TIME = ATTACK_START * Ts
ATTACK_END_TIME = ATTACK_END * Ts

# Dataset altitude bounds option from the guide
H_MIN = 0.97
H_MAX = 29.70

# Already completed threshold values
EPS_BARO = 0.30
EPS_GPS = 0.169349
EPS_GYR = 0.15

# Already established compatibility window
STL_WINDOW_MS = 580

# Attack values from completed single-sensor STL steps
BARO_ATTACK_OFFSET = 3.0
GPS_EAST_ATTACK_OFFSET = 20.0
GYRO_X_ATTACK_VALUE = 0.8


# ============================================================
# Load dataset exactly using the guide's segment structure
# ============================================================

if not os.path.exists(DATASET_PATH):
    raise FileNotFoundError(f"Dataset not found: {DATASET_PATH}")

d = sio.loadmat(DATASET_PATH)

Yseg = d["Yseg"][0]
EXTRAseg = d["EXTRAseg"][0]
Y = Yseg[0]
EXTR = EXTRAseg[0]

Y = np.asarray(Y, dtype=float)
EXTR = np.asarray(EXTR, dtype=float)

n = Y.shape[0]
t = np.arange(n) * Ts


# ============================================================
# Dataset channels from the guide
# ============================================================

# Altitude
alt = Y[:, 2].copy()

# Barometer
baro_alt = EXTR[:, 1].copy()

# GPS/model position
gps_north = Y[:, 0].copy()
gps_east = Y[:, 1].copy()
model_north = Y[:, 0].copy()
model_east = Y[:, 1].copy()

# Gyroscope
gyro_x_model = Y[:, 9].copy()
gyro_y_model = Y[:, 10].copy()
gyro_z_model = Y[:, 11].copy()

gyro_x_measured = Y[:, 9].copy()
gyro_y_measured = Y[:, 10].copy()
gyro_z_measured = Y[:, 11].copy()


# ============================================================
# Apply compound attacks using the same completed definitions
# ============================================================

baro_alt_attacked = baro_alt.copy()
gps_north_attacked = gps_north.copy()
gps_east_attacked = gps_east.copy()
gyro_x_measured_attacked = gyro_x_measured.copy()
gyro_y_measured_attacked = gyro_y_measured.copy()
gyro_z_measured_attacked = gyro_z_measured.copy()

baro_alt_attacked[ATTACK_START:ATTACK_END] = (
    baro_alt_attacked[ATTACK_START:ATTACK_END] + BARO_ATTACK_OFFSET
)

gps_east_attacked[ATTACK_START:ATTACK_END] = (
    gps_east_attacked[ATTACK_START:ATTACK_END] + GPS_EAST_ATTACK_OFFSET
)

gyro_x_measured_attacked[ATTACK_START:ATTACK_END] = GYRO_X_ATTACK_VALUE


# ============================================================
# Residuals from the completed STL steps
# ============================================================

baro_residual = np.abs(baro_alt_attacked - alt)

gps_north_residual = np.abs(gps_north_attacked - model_north)
gps_east_residual = np.abs(gps_east_attacked - model_east)

gyro_residual_x = np.abs(gyro_x_measured_attacked - gyro_x_model)
gyro_residual_y = np.abs(gyro_y_measured_attacked - gyro_y_model)
gyro_residual_z = np.abs(gyro_z_measured_attacked - gyro_z_model)


# ============================================================
# RTAMT compatibility class fallback
# ============================================================

try:
    SpecClass = rtamt.STLDiscreteTimeSpecification
except AttributeError:
    SpecClass = rtamt.StlDiscreteTimeSpecification


# ============================================================
# Multi-Sensor Compound STL formula
# ============================================================

stl_formula_display = (
    "G[0:580ms] ("
    "(alt > 0.97) and (alt < 29.70) "
    "and (baro_residual < 0.30) "
    "and (gps_north_residual < 0.169349) "
    "and (gps_east_residual < 0.169349) "
    "and (gyro_residual_x < 0.15) "
    "and (gyro_residual_y < 0.15) "
    "and (gyro_residual_z < 0.15)"
    ")"
)

rtamt_formula = (
    "always[0:580ms] ("
    "(alt > 0.97) and (alt < 29.70) "
    "and (baro_residual < 0.30) "
    "and (gps_north_residual < 0.169349) "
    "and (gps_east_residual < 0.169349) "
    "and (gyro_residual_x < 0.15) "
    "and (gyro_residual_y < 0.15) "
    "and (gyro_residual_z < 0.15)"
    ")"
)

spec = SpecClass()
spec.name = "Multi-Sensor Compound Spec — All at once"
spec.set_sampling_period(20, "ms", 0.1)

spec.declare_var("alt", "float")
spec.declare_var("baro_residual", "float")
spec.declare_var("gps_north_residual", "float")
spec.declare_var("gps_east_residual", "float")
spec.declare_var("gyro_residual_x", "float")
spec.declare_var("gyro_residual_y", "float")
spec.declare_var("gyro_residual_z", "float")

spec.spec = rtamt_formula
spec.parse()


# ============================================================
# Offline evaluation with separate "time" key
# ============================================================

dataset = {
    "time": t,
    "alt": alt,
    "baro_residual": baro_residual,
    "gps_north_residual": gps_north_residual,
    "gps_east_residual": gps_east_residual,
    "gyro_residual_x": gyro_residual_x,
    "gyro_residual_y": gyro_residual_y,
    "gyro_residual_z": gyro_residual_z,
}

robustness = spec.evaluate(dataset)

robustness_array = np.asarray(robustness, dtype=float)

if robustness_array.ndim == 2 and robustness_array.shape[1] >= 2:
    rob_time = robustness_array[:, 0]
    rob_value = robustness_array[:, 1]
else:
    rob_value = robustness_array.reshape(-1)
    rob_time = t[: len(rob_value)]


# ============================================================
# Detection
# ============================================================

violation_indices = np.where(rob_value < 0)[0]

if len(violation_indices) > 0:
    first_violation_index = violation_indices[0]
    detection_time = rob_time[first_violation_index]
    detection_latency = detection_time - ATTACK_START_TIME
    detected = True
else:
    detection_time = None
    detection_latency = None
    detected = False


# ============================================================
# Terminal output
# ============================================================

print(f"Loaded dataset: {DATASET_PATH}")
print(f"Using segment: {SEGMENT_INDEX}")
print(f"Ts = {Ts:.2f} s")
print(f"fs = {fs} Hz")
print()
print("Formula name: Multi-Sensor Compound Spec — All at once")
print("STL formula:")
print(stl_formula_display)
print()
print(f"Altitude bounds: h_min = {H_MIN:.2f} m, h_max = {H_MAX:.2f} m")
print(f"Thresholds: epsilon_baro = {EPS_BARO:.2f} m, epsilon_gps = {EPS_GPS:.6f} m, epsilon_gyr = {EPS_GYR:.2f} rad/s")
print(f"Window: G[0:{STL_WINDOW_MS}ms]")
print()
print(f"Attack samples: {ATTACK_START}:{ATTACK_END}")
print(f"Attack time window: {ATTACK_START_TIME:.2f} s to {ATTACK_END_TIME:.2f} s")
print(f"BARO_Alt attacked = BARO_Alt + {BARO_ATTACK_OFFSET:.1f} m")
print(f"GPS East attacked = GPS East + {GPS_EAST_ATTACK_OFFSET:.1f} m")
print(f"GyrX measured attacked = {GYRO_X_ATTACK_VALUE:.1f} rad/s")
print()

if detected:
    print(f"Attack detected at t = {detection_time:.2f} s")
    print(f"Detection latency relative to attack start = {detection_latency:.2f} s")
else:
    print("No attack detected")


# ============================================================
# Plot
# ============================================================

plt.figure(figsize=(12, 10))

plt.subplot(5, 1, 1)
plt.plot(t, alt, linewidth=1.0)
plt.axhline(H_MIN, linestyle="--", linewidth=1.0)
plt.axhline(H_MAX, linestyle="--", linewidth=1.0)
plt.axvspan(ATTACK_START_TIME, ATTACK_END_TIME, alpha=0.2)
plt.ylabel("alt (m)")
plt.title("Multi-Sensor Compound STL Inputs and Robustness")

plt.subplot(5, 1, 2)
plt.plot(t, baro_residual, linewidth=1.0)
plt.axhline(EPS_BARO, linestyle="--", linewidth=1.0)
plt.axvspan(ATTACK_START_TIME, ATTACK_END_TIME, alpha=0.2)
plt.ylabel("baro res")

plt.subplot(5, 1, 3)
plt.plot(t, gps_north_residual, linewidth=1.0, label="GPS North residual")
plt.plot(t, gps_east_residual, linewidth=1.0, label="GPS East residual")
plt.axhline(EPS_GPS, linestyle="--", linewidth=1.0)
plt.axvspan(ATTACK_START_TIME, ATTACK_END_TIME, alpha=0.2)
plt.ylabel("gps res")
plt.legend(loc="upper right")

plt.subplot(5, 1, 4)
plt.plot(t, gyro_residual_x, linewidth=1.0, label="Gyro X residual")
plt.plot(t, gyro_residual_y, linewidth=1.0, label="Gyro Y residual")
plt.plot(t, gyro_residual_z, linewidth=1.0, label="Gyro Z residual")
plt.axhline(EPS_GYR, linestyle="--", linewidth=1.0)
plt.axvspan(ATTACK_START_TIME, ATTACK_END_TIME, alpha=0.2)
plt.ylabel("gyro res")
plt.legend(loc="upper right")

plt.subplot(5, 1, 5)
plt.plot(rob_time, rob_value, linewidth=1.0)
plt.axhline(0.0, linestyle="--", linewidth=1.0)
plt.axvspan(ATTACK_START_TIME, ATTACK_END_TIME, alpha=0.2)
if detected:
    plt.axvline(detection_time, linestyle="--", linewidth=1.0)
plt.ylabel("robustness")
plt.xlabel("time (s)")

plt.tight_layout()
plt.savefig(OUTPUT_PLOT, dpi=150)
plt.close()

print(f"Saved plot: {OUTPUT_PLOT}")
