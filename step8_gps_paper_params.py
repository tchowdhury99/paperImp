#!/usr/bin/env python3
"""
Step 8 GPS parameter selection for STL.

Project path:
  /home/tchowdh4/paperImp/

Interpreter:
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Purpose:
  Compute GPS residuals and paper-based GPS threshold:
      ε_gps = emax + m
  where:
      m = 0.10 * emax
"""

import numpy as np
import scipy.io

DATASET = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"

print("Loading dataset:")
print(DATASET)

d = scipy.io.loadmat(DATASET)

Yseg = d["Yseg"][0]
EXTRAseg = d["EXTRAseg"][0]
Ts = float(d["Ts"].flat[0])
fs = float(d["fs"].flat[0])

seg_idx = 0
Y = Yseg[seg_idx]
EXTR = EXTRAseg[seg_idx]
N = Y.shape[0]

print("\nDataset loaded successfully")
print(f"Segment index: {seg_idx}")
print(f"Y shape: {Y.shape}")
print(f"EXTRA shape: {EXTR.shape}")
print(f"Sampling time Ts: {Ts}")
print(f"Sampling frequency fs: {fs}")

# Model position channels from guide:
# Yseg[i]: 0:pN, 1:pE
pN_model = Y[:, 0]
pE_model = Y[:, 1]

# GPS channels from guide:
# EXTRAseg[i]: 5:GPS_Lat, 6:GPS_Lng
gps_lat = EXTR[:, 5]
gps_lng = EXTR[:, 6]

# Convert GPS latitude/longitude to local meters.
# The STL guide requires GPS_North/GPS_East, but the dataset gives GPS_Lat/GPS_Lng.
# We use the first GPS sample as the local origin so GPS and pN/pE are compared in meters.
lat0 = gps_lat[0]
lng0 = gps_lng[0]

meters_per_deg_lat = 111320.0
meters_per_deg_lng = 111320.0 * np.cos(np.deg2rad(lat0))

GPS_North = (gps_lat - lat0) * meters_per_deg_lat
GPS_East = (gps_lng - lng0) * meters_per_deg_lng

# Align model pN/pE to the same first-sample origin.
pN_model_local = pN_model - pN_model[0]
pE_model_local = pE_model - pE_model[0]

gps_north_residual = np.abs(GPS_North - pN_model_local)
gps_east_residual = np.abs(GPS_East - pE_model_local)

# Paper-style threshold:
# T = emax + m
# Use one ε_gps for both north and east predicates, so choose max over both axes.
emax_north = float(np.max(gps_north_residual))
emax_east = float(np.max(gps_east_residual))
emax = max(emax_north, emax_east)

margin = 0.10 * emax
epsilon_gps = emax + margin

print("\nGPS local conversion:")
print(f"lat0: {lat0}")
print(f"lng0: {lng0}")
print(f"meters_per_deg_lat: {meters_per_deg_lat}")
print(f"meters_per_deg_lng: {meters_per_deg_lng}")

print("\nGPS residual stats:")
print(f"max gps_north_residual: {emax_north:.6f} m")
print(f"max gps_east_residual:  {emax_east:.6f} m")
print(f"emax = max(north, east): {emax:.6f} m")
print(f"margin m = 0.10 * emax: {margin:.6f} m")
print(f"epsilon_gps = emax + m: {epsilon_gps:.6f} m")

# Paper example window:
# Paper gives 575 ms as an example selected window for 3DR Solo quadrotor.
# At 50 Hz dataset sampling, one sample is 20 ms.
paper_window_ms = 575
samples_exact = paper_window_ms / (Ts * 1000.0)
samples_rounded = int(round(samples_exact))
dataset_window_ms = int(samples_rounded * Ts * 1000.0)

print("\nWindow W:")
print(f"Paper example W: {paper_window_ms} ms")
print(f"At Ts={Ts}s, this is {samples_exact:.3f} samples")
print(f"Nearest 50 Hz dataset-compatible window: {samples_rounded} samples")
print(f"Nearest dataset-compatible W: {dataset_window_ms} ms")

print("\nUse for Step 8 GPS STL:")
print(f"STL formula: G[0:{dataset_window_ms}ms] (gps_north_res < {epsilon_gps:.6f} and gps_east_res < {epsilon_gps:.6f})")
