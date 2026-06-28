# STL Implementation Guide
## For: Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles (Choi et al., RAID 2020)
## Platform: ArduPilot SITL + PX4 SITL at `/home/tchowdh4/paperImp/`

---

## Table of Contents
1. [Dataset Validation Results](#1-dataset-validation-results)
2. [STL Fundamentals](#2-stl-fundamentals)
3. [STL Operators Reference](#3-stl-operators-reference)
4. [Robustness (Quantitative Semantics)](#4-robustness-quantitative-semantics)
5. [Ready-to-Use STL Formulas for This Paper](#5-ready-to-use-stl-formulas-for-this-paper)
6. [YOUR Formulas — Template](#6-your-formulas--template)
7. [rtamt Library: How to Use](#7-rtamt-library-how-to-use)
8. [How to Load the Datasets](#8-how-to-load-the-datasets)
9. [Starting ArduPilot SITL (Separate Terminal)](#9-starting-ardupilot-sitl-separate-terminal)
10. [Starting PX4 SITL (Separate Terminal)](#10-starting-px4-sitl-separate-terminal)
11. [Connecting MAVLink to Read Live Sensor Data](#11-connecting-mavlink-to-read-live-sensor-data)
12. [Offline STL on Dataset — Implementation Skeleton](#12-offline-stl-on-dataset--implementation-skeleton)
13. [Online (Real-Time) STL via SITL — Implementation Skeleton](#13-online-real-time-stl-via-sitl--implementation-skeleton)
14. [Known Issues and Notes](#14-known-issues-and-notes)

---

## 1. Dataset Validation Results

> Run with: `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`

### `operation_data_50hz.mat` ← PRIMARY DATASET

| Property | Value |
|---|---|
| File path | `rv_recovery/data/operation_data_50hz.mat` |
| Sampling rate | **50 Hz** (Ts = 0.02 s) |
| Segments | **21 independent flight segments** |
| Total flight time | **2452 s (40.9 minutes)** |
| Longest segment | 139.8 s (Segment 0) |
| NaN / Inf | **NONE — data is clean** |
| Format | MATLAB struct with `Yseg`, `Useg`, `EXTRAseg` cell arrays |

**Channel layout:**

| Variable | Shape | Channels (index: name) |
|---|---|---|
| `Yseg[i]` | `(N, 12)` | 0:pN, 1:pE, 2:**alt**, 3:phi, 4:theta, 5:psi, 6:vN, 7:vE, 8:vUp, 9:p, 10:q, 11:r |
| `Useg[i]` | `(N, 7)` | 0:phi_cmd, 1:theta_cmd, 2:psi_cmd, 3:thr, 4:tiltN, 5:tiltE, 6:const |
| `EXTRAseg[i]` | `(N, 9)` | 0:BARO_Press, 1:**BARO_Alt**, 2:MagX, 3:MagY, 4:MagZ, 5:GPS_Lat, 6:GPS_Lng, 7:**GPS_Alt**, 8:GPS_Spd |

**Altitude channel stats (all 21 segments combined):**

| Channel | Min (m) | Max (m) | Mean (m) | Std (m) | Unit |
|---|---|---|---|---|---|
| `alt` (model state, col 2 of Yseg) | 0.97 | 29.70 | 15.93 | 7.30 | m AGL |
| `BARO_Alt` (col 1 of EXTRAseg) | 0.92 | 29.72 | 15.93 | 7.31 | m AGL |
| `GPS_Alt` (col 7 of EXTRAseg) | 1583.97 | 1611.36 | 1598.25 | 6.95 | m MSL |

**Model vs Barometer residual (alt − BARO_Alt):**
- Mean: **−0.0006 m** (essentially 0 bias)
- Std: **0.064 m**
- Max absolute: **0.226 m**
- ✅ **Excellent fit — model predicts barometer within 0.23 m**

**⚠️ GPS_Alt is MSL, not AGL.**
The flights are at ~1600 m elevation (Colorado, USA). GPS_Alt ≈ BARO_Alt + 1597 m.
For altitude STL specs, use `alt` or `BARO_Alt` (both are AGL).

**⚠️ Barometric Pressure (Eq. 5) offset note:**
The actual ground pressure is ~83,600 Pa (not 101,325 Pa sea-level standard).
Eq. 5 with P0=101325, h0=0 gives predictions offset by ~17,587 Pa.
Fix: calibrate P0 and h0 to local ground conditions, or use `BARO_Alt` directly.

---

### `sitl_run1/logs/4.BIN` ← ARDUPILOT DATAFLASH LOG

| Property | Value |
|---|---|
| File path | `rv_recovery/data/sitl_run1/logs/4.BIN` |
| Size | 55 MB |
| Messages extracted | > 300,000 |
| Format | ArduPilot DataFlash binary |

**Available message types and sample counts:**

| Type | Count | Contains |
|---|---|---|
| `ATT` | 12,218 | Roll, Pitch, Yaw (degrees) |
| `BARO` | ~22,221 | Alt (m AGL), Press (Pa) |
| `GPS` | ~11,113 | Alt (m MSL), Lat, Lng, Spd |
| `IMU` | ~12,217 | GyrX, GyrY, GyrZ (rad/s); AccX, AccY, AccZ |
| `NKF1-9` | ~12,218 each | EKF state estimates |
| `POS` | ~12,217 | Position NED |

**Channel ranges from BIN log:**

| Channel | Min | Max | Std | Unit |
|---|---|---|---|---|
| BARO_Alt | -0.03 | 29.72 | 7.79 | m AGL |
| GPS_Alt | 1583.07 | 1611.36 | 7.41 | m MSL |
| Roll | -18.03 | 18.11 | 4.13 | degrees |
| Pitch | -18.29 | 17.93 | 7.80 | degrees |
| GyrX | -0.476 | 0.571 | 0.036 | rad/s |
| GyrY | -0.577 | 0.691 | 0.043 | rad/s |
| GyrZ | -1.672 | 1.461 | 0.275 | rad/s |

✅ **Both datasets are validated and ready for STL implementation.**

---

## 2. STL Fundamentals

Signal Temporal Logic (STL) lets you write **formal specifications** over continuous-time signals and check whether a signal satisfies them at time t.

A signal here is a time series: `x(t)` — e.g., altitude measured every 0.02 s.

### 2.1 What STL gives you

**Boolean verdict:** at time t, does signal `x` satisfy spec `φ`?
```
(x, t) ⊨ φ      ← TRUE: signal satisfies φ at time t
(x, t) ⊭ φ      ← FALSE: signal violates φ at time t
```

**Robustness ρ (quantitative):** HOW MUCH does the signal satisfy/violate φ?
```
ρ(φ, x, t) > 0   ← satisfied, by margin ρ
ρ(φ, x, t) < 0   ← violated, by magnitude |ρ|
ρ(φ, x, t) = 0   ← on the boundary
```

### 2.2 A signal in STL context

```
time:   t₀   t₁   t₂   t₃   ...   tₙ
alt:    5.1  5.4  5.3  5.0  ...  15.0
```

At sampling rate 50 Hz, t₀ = 0.00 s, t₁ = 0.02 s, t₂ = 0.04 s, etc.

---

## 3. STL Operators Reference

### 3.1 Atomic Predicate
The basic building block. A comparison on a signal value.

```
μ := (f(x(t)) > c)   or   (f(x(t)) < c)   or   (f(x(t)) = c)
```

Examples:
```
altitude(t) > 5.0         ← "altitude is above 5 m"
|baro_real(t) - baro_pred(t)| < 0.5    ← "baro residual is small"
gyro_x(t) < 0.3           ← "gyro x-rate is bounded"
```

---

### 3.2 Boolean Operators

| Operator | Notation | Meaning |
|---|---|---|
| Negation | `¬φ` or `NOT φ` | φ is NOT satisfied |
| Conjunction | `φ₁ ∧ φ₂` or `φ₁ AND φ₂` | BOTH φ₁ and φ₂ |
| Disjunction | `φ₁ ∨ φ₂` or `φ₁ OR φ₂` | EITHER φ₁ or φ₂ |
| Implication | `φ₁ → φ₂` | IF φ₁ THEN φ₂ |

---

### 3.3 Temporal Operators

#### G — Globally (Always)
```
G[a,b] φ
```
"At ALL times in window [a, b], φ must hold."

```
G[0,10] (altitude > 5)
= "For all t ∈ [0, 10] s, altitude is above 5 m"
```

Unbounded (over entire signal):
```
G φ   =   G[0,∞] φ
```

---

#### F — Finally (Eventually)
```
F[a,b] φ
```
"At SOME time in window [a, b], φ must hold."

```
F[0,30] (altitude > 10)
= "At some point within the first 30 s, altitude exceeds 10 m"
```

---

#### U — Until
```
φ₁ U[a,b] φ₂
```
"φ₁ holds continuously until φ₂ becomes true, and φ₂ must occur in [a,b]."

```
(altitude < 20) U[0,60] (altitude > 15)
= "Altitude stays below 20 m until it climbs above 15 m, within 60 s"
```

---

#### X — Next (discrete-time)
```
X φ
```
"φ holds at the very NEXT time step."

```
X (baro_error < 0.5)
= "At the next sample (t + 0.02 s), barometer error is below 0.5"
```

---

### 3.4 Combining Operators

```
G[0,T] (F[0,W] (altitude > h_min))
= "At all times in [0,T], eventually within W seconds, altitude exceeds h_min"

G[0,T] (altitude > h_min ∧ altitude < h_max)
= "Altitude always stays between h_min and h_max for T seconds"

G[0,T] (|baro_alt - baro_predicted| < ε → F[0,K] recovery_success)
= "Whenever the barometer error is small, recovery succeeds within K seconds"
```

---

## 4. Robustness (Quantitative Semantics)

### 4.1 Why robustness matters

Instead of TRUE/FALSE, robustness gives you a **number**:
- `ρ > 0` → spec satisfied, ρ = margin before violation
- `ρ < 0` → spec violated, |ρ| = how badly

This is what makes STL useful for anomaly detection: when ρ drops below 0, an attack is detected. The rate of ρ dropping tells you how severe the attack is.

### 4.2 Robustness formulas for each operator

**Atomic predicate `(f(x) > c)`:**
```
ρ((f(x) > c), x, t) = f(x(t)) - c
```
Example: if altitude = 7.0 m and spec is `altitude > 5.0`:
```
ρ = 7.0 - 5.0 = +2.0   ← satisfied, 2 m of margin
```

**Atomic predicate `(f(x) < c)`:**
```
ρ((f(x) < c), x, t) = c - f(x(t))
```
Example: if altitude = 3.0 m and spec is `altitude < 5.0`:
```
ρ = 5.0 - 3.0 = +2.0   ← satisfied
```

**Negation `¬φ`:**
```
ρ(¬φ, x, t) = -ρ(φ, x, t)
```

**Conjunction `φ₁ ∧ φ₂`:**
```
ρ(φ₁ ∧ φ₂, x, t) = min(ρ(φ₁, x, t), ρ(φ₂, x, t))
```
The robustness of AND is the MINIMUM — weakest link.

**Disjunction `φ₁ ∨ φ₂`:**
```
ρ(φ₁ ∨ φ₂, x, t) = max(ρ(φ₁, x, t), ρ(φ₂, x, t))
```

**Globally `G[a,b] φ`:**
```
ρ(G[a,b] φ, x, t) = min_{τ ∈ [t+a, t+b]} ρ(φ, x, τ)
```
The robustness of G = MINIMUM robustness in the window. If ANY point violates, ρ < 0.

**Finally `F[a,b] φ`:**
```
ρ(F[a,b] φ, x, t) = max_{τ ∈ [t+a, t+b]} ρ(φ, x, τ)
```
The robustness of F = MAXIMUM robustness in the window. If ANY point satisfies, ρ > 0.

### 4.3 Example trace

Signal: `baro_error(t) = |baro_measured(t) - baro_predicted(t)|`

```
t(s):         0.00  0.02  0.04  0.06  0.08  0.10  0.12  0.14
baro_error:   0.05  0.06  0.08  0.22  0.45  0.91  1.20  1.50  ← attack starts ~0.06s

Spec φ: G[0,0.14] (baro_error < 0.30)
ρ at each t: 0.25  0.24  0.22  0.08  -0.15  -0.61  -0.90  -1.20

ρ(φ, x, 0) = min over all = -1.20  ← spec violated globally
```

At t=0.08 s, ρ first crosses 0 → **attack detected at t=0.08 s**.

---

## 5. Ready-to-Use STL Formulas for This Paper

These map directly to the paper's sensor attack scenarios. The channels refer to the dataset layout in Section 1.

### 5.1 Altitude Bounds (Mission Spec S3/S4 equivalent)

```
φ_alt_lower = G[0,T] (alt > h_min)
φ_alt_upper = G[0,T] (alt < h_max)
φ_alt = φ_alt_lower ∧ φ_alt_upper
      = G[0,T] (alt > h_min ∧ alt < h_max)
```

**Parameters from dataset:** `h_min = 0.97 m`, `h_max = 29.70 m`
**Reasonable mission bounds:** `h_min = 2.0 m`, `h_max = 25.0 m`

Robustness:
```
ρ(φ_alt, x, t) = min over τ∈[t, t+T] of: min(alt(τ) - h_min,  h_max - alt(τ))
```

---

### 5.2 Barometer Integrity (Sensor Attack Detection)

The paper checks whether the barometric reading matches the model prediction.

```
baro_residual(t) = |BARO_Alt(t) - alt(t)|

φ_baro = G[0,W] (baro_residual < ε_baro)
```

**Threshold from dataset:** residual std = 0.064 m, max = 0.226 m
**Recommended ε_baro:** `0.30 m` to `0.50 m` (= max_abs + 2×std margin)

Or using the paper's T = e_max + margin:
```
ε_baro = 0.226 + 0.10 × 0.226 = 0.249 m   (10% margin, per paper §3.3)
```

When ρ(φ_baro) < 0 → barometer attack detected.

---

### 5.3 GPS Integrity (Position Attack Detection)

```
gps_north_residual(t) = |GPS_North(t) - pN_model(t)|
gps_east_residual(t)  = |GPS_East(t)  - pE_model(t)|

φ_gps = G[0,W] (gps_north_residual < ε_gps ∧ gps_east_residual < ε_gps)
```

**Note:** GPS_Alt is MSL; pN/pE are local NED in meters.

---

### 5.4 Gyroscope Integrity (Rate Attack Detection)

```
gyro_residual_x(t) = |GyrX_measured(t) - GyrX_predicted(t)|
gyro_residual_y(t) = |GyrY_measured(t) - GyrY_predicted(t)|
gyro_residual_z(t) = |GyrZ_measured(t) - GyrZ_predicted(t)|

φ_gyro = G[0,W] (gyro_residual_x < ε_gyr ∧
                  gyro_residual_y < ε_gyr ∧
                  gyro_residual_z < ε_gyr)
```

**Threshold from dataset:** GyrX std = 0.036 rad/s → `ε_gyr = 0.15 rad/s`

---

### 5.5 Multi-Sensor Compound Spec (All at once)

```
φ_all = G[0,T] (
    (alt > h_min) ∧ (alt < h_max)      ← altitude bounds
  ∧ (baro_residual < ε_baro)           ← barometer integrity
  ∧ (gps_north_residual < ε_gps)       ← GPS North integrity
  ∧ (gps_east_residual < ε_gps)        ← GPS East integrity
  ∧ (gyro_residual_x < ε_gyr)          ← Gyro X integrity
  ∧ (gyro_residual_y < ε_gyr)          ← Gyro Y integrity
  ∧ (gyro_residual_z < ε_gyr)          ← Gyro Z integrity
)
```

---

### 5.6 Temporal Attack Pattern Detection

STL's advantage over the paper's simple threshold: detect patterns over time.

```
# Attack must persist for 1 s before flagging (reduces false positives):
φ_baro_persistent = G[0,T] (G[0,1.0] (baro_residual < ε_baro))
```

```
# After attack detected, recovery must happen within 10 s (Eq. 7):
φ_recovery = G[0,T] (
    (baro_residual > ε_baro) → F[0,10.0] (baro_residual < ε_baro)
)
```

```
# Multi-sensor: if ANY sensor is attacked, ALL must recover:
φ_any_attack = (baro_residual > ε_baro) ∨ (gyro_residual_x > ε_gyr)
φ_recovery_any = G[0,T] (φ_any_attack → F[0,10.0] ¬φ_any_attack)
```

---

## 6. YOUR Formulas — Template

Write your own STL specifications here. Use the channel names from Section 1.

### Your Formula 1
```
Name: ___________________
Spec: φ₁ = 
      G[_, _] (  )

Channels used: ___________________
Threshold values: ___________________
Window size (s): ___________________

Robustness interpretation:
  ρ > 0 means: ___________________
  ρ < 0 means: ___________________
```

### Your Formula 2
```
Name: ___________________
Spec: φ₂ = 
      

Channels used: ___________________
Threshold values: ___________________
```

### Your Formula 3
```
Name: ___________________
Spec: φ₃ = 
      

Channels used: ___________________
```

### Combined Spec
```
φ_combined = φ₁ ∧ φ₂ ∧ φ₃
```

---

## 7. rtamt Library: How to Use

### 7.1 Python interpreter to use
```bash
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
```
Do NOT use `python3` directly — that is Python 3.13 (miniconda) and rtamt has a compatibility issue with it.

### 7.2 Import and create a spec

```python
import rtamt

# Discrete-time STL (for sampled data like the .mat dataset)
spec = rtamt.STLDiscreteTimeSpecification()

# Declare signal variables (names must match what you feed in)
spec.declare_var('alt', 'float')
spec.declare_var('baro_alt', 'float')
spec.declare_var('baro_residual', 'float')

# Set sampling period (must match your data)
spec.set_sampling_period(20, 'ms')   # 50 Hz = 20 ms period

# Write your STL formula
spec.spec = 'G[0:5000ms] (alt > 2.0 and alt < 25.0)'   # G over 5 seconds

spec.parse()
```

### 7.3 Evaluate over a dataset

```python
import numpy as np

# Prepare signals as lists of (time_ms, value) pairs
times_ms = [int(i * 20) for i in range(N)]   # 0, 20, 40, ... (milliseconds)

dataset = {
    'alt':          list(zip(times_ms, alt_signal)),
    'baro_alt':     list(zip(times_ms, baro_signal)),
    'baro_residual':list(zip(times_ms, np.abs(alt_signal - baro_signal))),
}

# Compute robustness at t=0 (over the full signal from t=0)
rho = spec.evaluate(dataset)
print(f"Robustness ρ = {rho}")
```

### 7.4 Compute robustness trace (ρ at every t)

```python
spec2 = rtamt.STLDiscreteTimeSpecification()
spec2.declare_var('alt', 'float')
spec2.set_sampling_period(20, 'ms')
spec2.spec = 'G[0:100ms] (alt > 2.0)'   # 100ms = 5-sample sliding window
spec2.parse()

# Online evaluation — feed sample by sample
rho_trace = []
for i in range(len(alt_signal)):
    sample = {'alt': [(times_ms[i], alt_signal[i])]}
    rho_i = spec2.update(i, [('alt', alt_signal[i])])   # time step index
    rho_trace.append(rho_i)
```

### 7.5 Time notation in rtamt

| You write | Meaning |
|---|---|
| `G[0:100ms]` | Globally over next 100 milliseconds |
| `G[0:5000ms]` | Globally over next 5 seconds |
| `F[0:10000ms]` | Eventually within next 10 seconds |
| `G[1000ms:5000ms]` | Globally from 1 s to 5 s ahead |

---

## 8. How to Load the Datasets

### 8.1 Load `operation_data_50hz.mat`

```python
import scipy.io
import numpy as np

d = scipy.io.loadmat('/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat')

Yseg     = d['Yseg'][0]      # list of 21 segments, each shape (N, 12)
Useg     = d['Useg'][0]      # list of 21 segments, each shape (N, 7)
EXTRAseg = d['EXTRAseg'][0]  # list of 21 segments, each shape (N, 9)
Ts       = float(d['Ts'].flat[0])   # 0.02 s
fs       = float(d['fs'].flat[0])   # 50 Hz

# Pick one segment
seg_idx  = 0
Y    = Yseg[seg_idx]      # (6990, 12)
U    = Useg[seg_idx]      # (6990, 7)
EXTR = EXTRAseg[seg_idx]  # (6990, 9)
N    = Y.shape[0]

# Extract channels by index
alt       = Y[:, 2]    # altitude AGL (m)  — model state
phi       = Y[:, 3]    # roll (rad)
theta     = Y[:, 4]    # pitch (rad)
psi       = Y[:, 5]    # yaw (rad)
vN        = Y[:, 6]    # velocity North (m/s)
vE        = Y[:, 7]    # velocity East (m/s)
gyr_p     = Y[:, 9]    # roll rate (rad/s)
gyr_q     = Y[:, 10]   # pitch rate (rad/s)
gyr_r     = Y[:, 11]   # yaw rate (rad/s)

baro_press = EXTR[:, 0]  # barometric pressure (Pa)
baro_alt   = EXTR[:, 1]  # barometric altitude AGL (m)
mag_x      = EXTR[:, 2]  # magnetometer X (mGauss)
mag_y      = EXTR[:, 3]  # magnetometer Y
mag_z      = EXTR[:, 4]  # magnetometer Z
gps_lat    = EXTR[:, 5]  # GPS latitude (degrees)
gps_lng    = EXTR[:, 6]  # GPS longitude (degrees)
gps_alt    = EXTR[:, 7]  # GPS altitude MSL (m)  ← NOT AGL
gps_spd    = EXTR[:, 8]  # GPS speed (m/s)

# Time axis
t_sec = np.arange(N) * Ts   # seconds: 0, 0.02, 0.04, ...
t_ms  = np.arange(N) * 20   # milliseconds: 0, 20, 40, ...
```

### 8.2 Load ArduPilot DataFlash BIN log

```python
from pymavlink import mavutil
import numpy as np

log = mavutil.mavlink_connection(
    '/home/tchowdh4/paperImp/rv_recovery/data/sitl_run1/logs/4.BIN',
    dialect='ardupilotmega'
)

baro_alt=[]; att_roll=[]; att_pitch=[]; gyr_x=[]; baro_t=[]; att_t=[]; gyr_t=[]

while True:
    msg = log.recv_match(type=['BARO','ATT','IMU'])
    if msg is None: break
    t = msg.get_type()
    ts = msg._timestamp   # seconds since epoch

    if t == 'BARO':
        baro_alt.append(msg.Alt)      # m AGL
        baro_t.append(ts)
    elif t == 'ATT':
        att_roll.append(msg.Roll)     # degrees
        att_pitch.append(msg.Pitch)
        att_t.append(ts)
    elif t == 'IMU':
        gyr_x.append(msg.GyrX)       # rad/s
        gyr_t.append(ts)

baro_alt  = np.array(baro_alt)
att_roll  = np.array(att_roll)
gyr_x     = np.array(gyr_x)
```

### 8.3 Compute derived signals for STL

```python
# Barometer residual (attack signal)
baro_residual = np.abs(baro_alt - alt)   # shape (N,), same dataset

# Injected attack: add 5 m spike to baro starting at sample 1000
baro_attacked = baro_alt.copy()
baro_attacked[1000:1500] += 5.0           # 10-second attack at 50 Hz
baro_residual_attacked = np.abs(baro_attacked - alt)

# Time in milliseconds for rtamt
t_ms = (np.arange(len(baro_alt)) * 20).tolist()
```

---

## 9. Starting ArduPilot SITL (Separate Terminal)

Open a **new terminal** (not this Claude terminal) and run:

### Terminal 1 — Start ArduPilot SITL
```bash
cd /home/tchowdh4/paperImp/ardupilot_ws/arducopter-3.4

# Using the existing pre-built binary directly:
/home/tchowdh4/paperImp/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad \
    --model quad \
    --speedup 1 \
    --home 40.0713,-105.2300,1584,0 \
    --defaults /home/tchowdh4/paperImp/ardupilot_ws/arducopter-3.4/Tools/autotest/default_params/copter.parm

# OR: use sim_vehicle.py (easier, manages everything automatically):
PATH=/home/tchowdh4/.pyenv/versions/3.10.14/bin:$PATH \
    python /home/tchowdh4/paperImp/ardupilot_ws/arducopter-3.4/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    --no-rebuild \
    --console \
    --map
```

Wait for output: `APM: Calibration complete`

**Default ports after startup:**
- MAVLink: `udp:127.0.0.1:14550` (GCS) and `udp:127.0.0.1:14551`
- TCP: `tcp:127.0.0.1:5762`

### Terminal 2 — Arm and takeoff (MAVProxy)
```bash
/home/tchowdh4/.pyenv/shims/mavproxy.py --master=udp:127.0.0.1:14550

# In MAVProxy console:
mode GUIDED
arm throttle
takeoff 10
```

### What to log (for your STL script to read live):
```bash
# MAVProxy can log to a file:
/home/tchowdh4/.pyenv/shims/mavproxy.py \
    --master=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14560 \
    --streamrate=50
# Your Python STL script connects to udp:127.0.0.1:14560
```

---

## 10. Starting PX4 SITL (Separate Terminal)

Open a **new terminal**:

### Terminal 1 — Start PX4 SITL
```bash
cd /home/tchowdh4/paperImp/PX4-Autopilot

PATH=/home/tchowdh4/.pyenv/versions/3.10.14/bin:$PATH \
    make px4_sitl_default none_iris

# PX4 will print:
# INFO  [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560
```

PX4 SITL without a physics simulator will wait for an external simulator on TCP port 4560.
For basic data collection without a sim, connect QGroundControl or just use the MAVLink port.

### Terminal 1 (Alternative) — PX4 with jMAVSim (Full Physics)
```bash
cd /home/tchowdh4/paperImp/PX4-Autopilot

PATH=/home/tchowdh4/.pyenv/versions/3.10.14/bin:$PATH \
    make px4_sitl_default jmavsim

# Wait for: "Waiting for initial data link"
# PX4 exposes MAVLink on UDP port 14540 (GCS) and 14557
```

**PX4 default MAVLink ports:**
- `udp:127.0.0.1:14540` — GCS port (QGroundControl default)
- `udp:127.0.0.1:14557` — onboard port

### Terminal 2 — Connect QGroundControl or MAVProxy
```bash
# MAVProxy to PX4:
/home/tchowdh4/.pyenv/shims/mavproxy.py --master=udp:127.0.0.1:14540

# Or pymavlink in your Python script:
# from pymavlink import mavutil
# mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
```

---

## 11. Connecting MAVLink to Read Live Sensor Data

This code goes in **your Python STL script** running in a third terminal.

```python
from pymavlink import mavutil
import time

# Connect to ArduPilot SITL
mav = mavutil.mavlink_connection('udp:127.0.0.1:14551')
# OR for PX4:
# mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')

mav.wait_heartbeat()
print("Connected. System:", mav.target_system)

# Request data streams at 50 Hz
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    50,   # Hz
    1     # start
)

# Live reading loop
alt_live = []
baro_live = []
t_live = []

while True:
    msg = mav.recv_match(type=['GLOBAL_POSITION_INT','SCALED_PRESSURE','RAW_IMU'], blocking=True, timeout=1.0)
    if msg is None:
        continue

    t = msg.get_type()
    ts = time.time()

    if t == 'GLOBAL_POSITION_INT':
        alt_m = msg.relative_alt / 1000.0   # mm → m AGL
        alt_live.append(alt_m)
        t_live.append(ts)

    elif t == 'SCALED_PRESSURE':
        press_pa = msg.press_abs * 100.0     # hPa → Pa
        baro_live.append(press_pa)

    elif t == 'RAW_IMU':
        gyr_x = msg.xgyro / 1000.0   # mrad/s → rad/s (ArduPilot)
        gyr_y = msg.ygyro / 1000.0
        gyr_z = msg.zgyro / 1000.0

    # === Run your STL monitor here ===
    # rho = your_stl_monitor(alt_live, baro_live)
    # if rho < 0:
    #     print(f"ATTACK DETECTED at t={ts:.2f}s, rho={rho:.3f}")
```

**MAVLink message types and what they carry:**

| Message | Field | Unit | What it is |
|---|---|---|---|
| `GLOBAL_POSITION_INT` | `relative_alt` | mm | Altitude AGL (÷ 1000 → m) |
| `GLOBAL_POSITION_INT` | `lat`, `lon` | 1e-7 deg | GPS lat/lng |
| `SCALED_PRESSURE` | `press_abs` | hPa | Barometric pressure |
| `SCALED_PRESSURE` | `temperature` | cdeg | Temperature |
| `RAW_IMU` | `xgyro`, `ygyro`, `zgyro` | mrad/s | Gyroscope |
| `RAW_IMU` | `xacc`, `yacc`, `zacc` | mG | Accelerometer |
| `ATTITUDE` | `roll`, `pitch`, `yaw` | rad | Euler angles |
| `LOCAL_POSITION_NED` | `x`, `y`, `z` | m | Position NED |

---

## 12. Offline STL on Dataset — Implementation Skeleton

Use this as your starting point. Fill in your own STL spec in the `DEFINE YOUR STL SPEC` section.

```python
#!/usr/bin/env python3
"""
Offline STL monitor on operation_data_50hz.mat
Interpreter: /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
"""
import scipy.io, numpy as np, rtamt, matplotlib.pyplot as plt

# ── 1. Load dataset ──────────────────────────────────────────────────────────
d    = scipy.io.loadmat('/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat')
Yseg = d['Yseg'][0];  EXTRAseg = d['EXTRAseg'][0]
Ts   = float(d['Ts'].flat[0])

# Pick one segment (0 = longest, 139.8 s)
Y    = Yseg[0];  EXTR = EXTRAseg[0];  N = Y.shape[0]

alt        = Y[:, 2]       # m AGL
baro_alt   = EXTR[:, 1]   # m AGL  (measured sensor)
gps_alt_msl= EXTR[:, 7]   # m MSL  (subtract ~1597 for AGL)

# Residuals (model vs sensor)
baro_res = np.abs(alt - baro_alt)

# ── 2. Simulate an attack (for testing STL detection) ────────────────────────
baro_attacked   = baro_alt.copy()
attack_start    = 2000     # sample index (= 40 s)
attack_end      = 2500     # sample index (= 50 s)
baro_attacked[attack_start:attack_end] += 3.0    # 3 m spoofing offset

baro_res_attacked = np.abs(alt - baro_attacked)

# ── 3. Time axis ─────────────────────────────────────────────────────────────
t_sec = np.arange(N) * Ts
t_ms  = (np.arange(N) * 1000 * Ts).astype(int)

# ── 4. DEFINE YOUR STL SPEC ─────────────────────────────────────────────────
spec = rtamt.STLDiscreteTimeSpecification()
spec.declare_var('alt', 'float')
spec.declare_var('baro_res', 'float')

spec.set_sampling_period(int(Ts * 1000), 'ms')   # 20 ms = 50 Hz

# ===== PUT YOUR FORMULA HERE =================================================
spec.spec = 'G[0:2000ms] (baro_res < 0.30)'    # ← REPLACE with your formula
# =============================================================================

spec.parse()

# ── 5. Evaluate robustness trace ─────────────────────────────────────────────
rho_clean    = []
rho_attacked = []

for i in range(N):
    r1 = spec.update(i, [('alt', alt[i]),
                          ('baro_res', baro_res[i])])
    r2 = spec.update(i, [('alt', alt[i]),
                          ('baro_res', baro_res_attacked[i])])
    rho_clean.append(r1)
    rho_attacked.append(r2)

rho_clean    = np.array(rho_clean)
rho_attacked = np.array(rho_attacked)

# ── 6. Find detection point ──────────────────────────────────────────────────
violations = np.where(rho_attacked < 0)[0]
if len(violations):
    det_idx = violations[0]
    print(f"Attack detected at t = {t_sec[det_idx]:.2f} s  (sample {det_idx})")
    print(f"  Attack started at t = {t_sec[attack_start]:.2f} s")
    print(f"  Detection latency  = {t_sec[det_idx] - t_sec[attack_start]:.2f} s")
else:
    print("No violation detected (threshold may be too large)")

# ── 7. Plot ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t_sec, alt, label='alt (model)', color='blue')
axes[0].plot(t_sec, baro_alt, label='BARO_Alt (clean)', color='green', alpha=0.6)
axes[0].plot(t_sec, baro_attacked, label='BARO_Alt (attacked)', color='red', alpha=0.6, ls='--')
axes[0].axvspan(t_sec[attack_start], t_sec[attack_end], color='red', alpha=0.1, label='attack window')
axes[0].set_ylabel('Altitude (m AGL)')
axes[0].legend(fontsize=8); axes[0].grid(alpha=0.3)

axes[1].plot(t_sec, baro_res, label='residual (clean)', color='green')
axes[1].plot(t_sec, baro_res_attacked, label='residual (attacked)', color='red', ls='--')
axes[1].axhline(0.30, color='orange', ls=':', label='threshold ε=0.30m')
axes[1].set_ylabel('|alt - BARO| (m)')
axes[1].legend(fontsize=8); axes[1].grid(alpha=0.3)

axes[2].plot(t_sec, rho_clean, label='ρ (clean)', color='green')
axes[2].plot(t_sec, rho_attacked, label='ρ (attacked)', color='red', ls='--')
axes[2].axhline(0, color='black', lw=1.5, ls='-', label='ρ = 0 boundary')
if len(violations):
    axes[2].axvline(t_sec[det_idx], color='purple', ls=':', label=f'detection t={t_sec[det_idx]:.1f}s')
axes[2].set_ylabel('Robustness ρ'); axes[2].set_xlabel('Time (s)')
axes[2].legend(fontsize=8); axes[2].grid(alpha=0.3)

plt.suptitle('STL Robustness — Barometer Attack Detection\n'
             f'Spec: {spec.spec}', fontsize=10)
plt.tight_layout()
plt.savefig('/home/tchowdh4/paperImp/stl_result.png', dpi=130)
print("Saved: /home/tchowdh4/paperImp/stl_result.png")
plt.show()
```

---

## 13. Online (Real-Time) STL via SITL — Implementation Skeleton

This runs while the SITL is running. Open it in a **separate terminal** from the SITL.

```python
#!/usr/bin/env python3
"""
Online (real-time) STL monitor connected to ArduPilot SITL
Interpreter: /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
Connect AFTER the SITL is running and the vehicle is armed.
"""
import rtamt, time, numpy as np
from pymavlink import mavutil

# ── 1. Connect to SITL ───────────────────────────────────────────────────────
mav = mavutil.mavlink_connection('udp:127.0.0.1:14551')
mav.wait_heartbeat()
print(f"Connected to system {mav.target_system}")

# ── 2. Define STL spec ───────────────────────────────────────────────────────
spec = rtamt.STLDiscreteTimeSpecification()
spec.declare_var('alt', 'float')
spec.declare_var('baro_res', 'float')
spec.set_sampling_period(20, 'ms')   # 50 Hz

# ===== YOUR FORMULA HERE =====================================================
spec.spec = 'G[0:2000ms] (baro_res < 0.30)'
# =============================================================================
spec.parse()

# ── 3. State for model prediction ────────────────────────────────────────────
# Load the system model (A, B, C, D matrices from matlab)
import scipy.io
m = scipy.io.loadmat('/home/tchowdh4/paperImp/rv_recovery/matlab/models/model_BARO_Alt.mat')
# Adjust key names to what your model file uses
# A = m['A']; B = m['B']; C = m['C']; D = m['D']

# ── 4. Request data streams ──────────────────────────────────────────────────
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1
)

# ── 5. Real-time loop ────────────────────────────────────────────────────────
step = 0
alt_prev  = 0.0
baro_prev = 0.0

print("Monitoring started. Ctrl-C to stop.")
try:
    while True:
        msg = mav.recv_match(
            type=['GLOBAL_POSITION_INT','SCALED_PRESSURE'],
            blocking=True, timeout=0.5
        )
        if msg is None:
            continue

        t = msg.get_type()

        if t == 'GLOBAL_POSITION_INT':
            alt_live  = msg.relative_alt / 1000.0   # mm → m AGL
            alt_prev  = alt_live

        elif t == 'SCALED_PRESSURE':
            baro_pa   = msg.press_abs * 100.0       # hPa → Pa

            # Simple baro→altitude conversion using hypsometric formula:
            # alt_from_baro = 44330 * (1 - (press/P0)^(1/5.255))
            P0 = 83600.0   # local ground pressure (Pa) from dataset
            baro_alt_live = 44330.0 * (1.0 - (baro_pa / P0) ** (1.0/5.255))

            baro_res = abs(alt_prev - baro_alt_live)

            # Evaluate STL robustness
            rho = spec.update(step, [('alt',     alt_prev),
                                     ('baro_res', baro_res)])
            step += 1

            status = "OK" if rho >= 0 else "*** ATTACK DETECTED ***"
            print(f"[{step:6d}] t={step*0.02:7.2f}s  alt={alt_prev:6.2f}m  "
                  f"baro_res={baro_res:.3f}m  ρ={rho:+.3f}  {status}")

            if rho < 0:
                print(f"  → Spec violated: {spec.spec}")
                # ↑ Trigger your recovery action here

except KeyboardInterrupt:
    print("Monitor stopped.")
```

---

## 14. Known Issues and Notes

### Python interpreter
Always use `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3` for STL work.
The system `python3` (3.13 miniconda) has an rtamt incompatibility.

### Barometric pressure offset
The dataset flights are at ~1600 m MSL (Colorado). Ground pressure is ~83,600 Pa.
Eq. 5 from the paper uses P0 = 101,325 Pa (sea level). You must either:
- Set `h0` = ground elevation (~1597 m) in Eq. 5
- Or use `BARO_Alt` directly (already AGL, calibrated)

### GPS_Alt is MSL not AGL
`EXTRAseg[:, 7]` is GPS altitude in meters above sea level (~1584–1611 m).
To get AGL: `gps_alt_agl = gps_alt_msl - ground_elevation`
From dataset: ground_elevation ≈ 1584 m.

### ArduPilot SITL rebuild (if needed)
```bash
cd /home/tchowdh4/paperImp/ardupilot_ws/arducopter-3.4
PATH=/home/tchowdh4/.pyenv/versions/3.10.14/bin:$PATH python ./waf copter
```

### PX4 SITL rebuild (if needed)
```bash
cd /home/tchowdh4/paperImp/PX4-Autopilot
PATH=/home/tchowdh4/.pyenv/versions/3.10.14/bin:$PATH make px4_sitl_default none_iris
```

### rtamt time units
rtamt time windows are in milliseconds as integers, NOT seconds.
`G[0:5000ms]` = "Globally over 5 seconds" at 50 Hz = 250 samples.

### Window size selection
- Too small (< 1 s): many false positives from sensor noise
- Too large (> 30 s): slow detection
- Recommended starting point: 1–5 s window (`1000ms` to `5000ms`)

---

*Guide created for `/home/tchowdh4/paperImp/` — 2026-06-28*
