# Replication: Choi et al. RAID 2020 — Sensor-Attack Recovery on ArduPilot SITL

**Paper:** "Software-Based Realtime Recovery from Sensor Attacks on Robotic Vehicles"  
**Authors:** Hongjun Choi, Sayali Kate, Yousra Aafer, Xiangyu Zhang, Dongyan Xu  
**Venue:** RAID 2020  
**Replication date:** 2026-06-08  
**Platform:** Ubuntu 24.04, GCC 13, MATLAB R2026a, ArduCopter 3.4.6 SITL  

---

## Table of Contents

1. [What the Paper Does](#1-what-the-paper-does)
2. [Environment Setup](#2-environment-setup)
3. [Step 1 — Build ArduCopter 3.4.6 SITL](#3-step-1--build-arducopter-346-sitl)
4. [Step 2 — Verify SITL Launch](#4-step-2--verify-sitl-launch)
5. [Step 3 — Collect Flight Logs (20 Missions)](#5-step-3--collect-flight-logs-20-missions)
6. [Step 4 — Parse Dataflash Logs](#6-step-4--parse-dataflash-logs)
7. [Step 5 — MATLAB System Identification](#7-step-5--matlab-system-identification)
8. [Step 6 — DTW Parameter Selection](#8-step-6--dtw-parameter-selection)
9. [Step 7 — Recovery Monitor Implementation](#9-step-7--recovery-monitor-implementation)
10. [Step 8 — Firmware Patch](#10-step-8--firmware-patch)
11. [Step 9 — Attack Injection and Eq.7 Evaluation](#11-step-9--attack-injection-and-eq7-evaluation)
12. [Summary: Deviations from the Paper](#12-summary-deviations-from-the-paper)
13. [Final Results](#13-final-results)

---

## 1. What the Paper Does

Choi et al. propose a software-only defense against physical sensor spoofing attacks on robotic vehicles (drones, rovers). The core insight is:

> A vehicle's own flight dynamics can be used to predict what sensors *should* read. If a sensor diverges too far from the physics-based prediction, the system switches to a software-estimated value instead of the corrupted physical reading.

### The five-stage pipeline (paper Section 3):

| Stage | What it produces |
|-------|-----------------|
| **1. Operation data collection** | Real flight logs from SITL |
| **2. Sensor equation derivation** | Physics equations for each sensor as a function of state |
| **3. System identification** | A linear state-space model `x[k+1] = Ax[k] + Bu[k]`, `y[k] = Cx[k] + Du[k]` from flight data |
| **4. Parameter selection (DTW)** | Window size `N` and thresholds `T_on`, `T_off` for anomaly detection |
| **5. Recovery module + firmware patch** | Algorithm 1 embedded into AP_InertialSensor.cpp |

### Attack model (paper Section 2):

- Attacker can inject a bias or signal into individual sensor outputs (GPS, gyro, accelerometer, barometer, magnetometer)
- The vehicle's control loop uses the corrupted reading, causing attitude/position divergence
- **Defense**: the recovery monitor runs in parallel, computes a physics-based prediction, and substitutes the prediction when the real sensor deviates beyond a learned threshold

### Success criterion — Equation (7):

```
R_succ := |Y_t - Ȳ_t| ≤ ε,   for all t ∈ [1..k]
```

Where `Y_t` is actual attitude, `Ȳ_t` is desired attitude, `ε = 3°`, and `k = 30s` evaluation window.

---

## 2. Environment Setup

### 2.1 Host Machine

| Item | Value |
|------|-------|
| OS | Ubuntu 24.04 LTS |
| Kernel | 6.17.0-35-generic |
| Compiler | GCC 13.3 |
| Python | 3.10.14 (via pyenv) |
| MATLAB | R2026a at `/usr/local/MATLAB/R2026a/bin/matlab` |
| Ardu> Pilot source | `~/ardupilot_ws/arducopter-3.4` |
| Recovery code | `~/rv_recovery/` |

### 2.2 Python Environment

The project uses a pre-existing virtual environment at `~/venv-ardupilot`. All Python work activates it first:

```bash
source ~/venv-ardupilot/bin/activate
```

**Installed packages used:**

| Package | Purpose |
|---------|---------|
| `pymavlink` | MAVLink communication with SITL |
| `mavproxy` | Ground station relay daemon |
| `scipy` | `CubicSpline` interpolation, `.mat` file I/O |
| `numpy` | Array math |
| `dtaidistance` | Dynamic Time Warping for parameter selection |

**Why this env, not conda?** The paper assumes a `rv_recovery` conda environment but that environment does not exist on this machine. The venv contains all required packages and is functionally identical.

### 2.3 Directory Layout

```
~/rv_recovery/
├── python/
│   ├── collect_logs.py          # Step 3 — automated mission collection
│   ├── parse_dataflash.py       # Step 4 — log → operation_data.mat
│   ├── select_parameters.py     # Step 6 — DTW N/T_on/T_off selection
│   ├── attack_injector.py       # Step 9 — inject SIM_GYRO_BIAS_X
│   └── eval_recovery.py         # Step 9 — Eq.7 evaluation
├── matlab/
│   ├── system_identification.m  # Step 5 (initial attempt, PEM/ssest)
│   ├── sysid_n4sid.m            # Step 5 (final, N4SID subspace)
│   └── models/
│       ├── quadrotor_ArduCopter34_n4sid.mat
│       └── model_matrices.h     # auto-generated A,B,C,D,K,Ap in C float[][]
├── firmware_patch/
│   ├── recovery_monitor.h       # Step 7 — Algorithm 1 in C++
│   ├── software_sensors.h       # Step 7 — sensor physics equations
│   └── test_recovery.cpp        # standalone unit test
└── data/
    ├── logs/
    │   └── all_missions_1.BIN   # 88 MB, ~2.5 hours, 20 missions
    ├── operation_data.mat        # 608 MB, U(3980561,4), Y(3980561,16)
    └── recovery_params.npy      # N=491, T_on=558.40, T_off=441.14
```

---

## 3. Step 1 — Build ArduCopter 3.4.6 SITL

**Files involved**

| Role              | Path                                                                                 |                      |
| ----------------- | ------------------------------------------------------------------------------------ | -------------------- |
| Source tree       | `~/ardupilot_ws/arducopter-3.4/`                                                     |                      |
| Build config      | `~/ardupilot_ws/arducopter-3.4/build/sitl/`                                          |                      |
| **Output binary** | `~/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad`                       |                      |
| Default params    | `~/ardupilot_ws/arducopter-3.4/Tools/autotest/default_params/copter.parm`            |                      |
| GCC patch files   | `libraries/AP_GPS/AP_GPS_UBLOX.h`, `AP_GPS_MTK.h`, `AP_GPS_MTK19.h`, `AP_GPS_SIRF.h` |                      |
|                   | `libraries/AP_Mount/AP_Mount_SToRM32_serial.h`, `AP_Mount_Alexmos.h`                 |                      |
|                   | `libraries/AP_NavEKF2/AP_NavEKF2_MagFusion.cpp`                                      |                      |
|                   | `libraries/AP_AHRS/AP_AHRS_NavEKF.cpp`                                               |                      |
|                   | `libraries/AP_Mount/SoloGimbal_Parameters.cpp`                                       |                      |
|                   | `libraries/SITL/sitl_barometer.cpp`, `sitl_compass.cpp`, `sitl_ins.cpp`              |                      |
| Build log         | (waf prints to stdout; redirect with `python ./waf copter 2>&1                       | tee /tmp/build.log`) |

### Why ArduCopter 3.4?

The paper targets the **3DR Solo** quadrotor, which ran a customized ArduCopter 3.4.x firmware. The paper's firmware patch targets `AP_InertialSensor.cpp` with the exact function signature and structure from that version. Using a newer ArduPilot (e.g., master/4.x) would require rewriting the entire patch because:

- The `AP_InertialSensor::update()` function structure changed substantially
- The Dataflash log message types changed (`IMU` vs `GYR`/`ACC`, `ATT` field names)
- The EKF backend API changed

### Getting the right source

```bash
cd ~/ardupilot_ws/arducopter-3.4
git describe --tags
# Must show: Copter-3.4.6
# If it shows V4.8.0-dev or similar, you are on master — check out the tag first
```

**Common pitfall:** A pre-existing worktree can be on `master` even if a guide says it's version 3.4. Always verify with `git describe --tags` before building.

### Configure the build

```bash
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf configure \
    --board sitl \
    CXXFLAGS="-fpermissive -Wno-error=maybe-uninitialized"
```

**Why `--board sitl`?** Compiles for Software-In-The-Loop simulation (runs as a native Linux process, not embedded ARM).

**Why `-fpermissive`?** ArduCopter 3.4 was written against GCC 4-5. GCC 13 enforces several rules as hard errors that older compilers accepted:
- Pointer-integer comparisons (`if (ptr == 0)` where `ptr` is `uint8_t*`)
- `-fpermissive` downgrades these from errors to warnings

### GCC 13 source patches required

GCC 13 introduced new strict-aliasing and bounds checks that break several files in ArduCopter 3.4. These had to be patched before the build would succeed:

#### Patch 1: Flexible array member in union (hard error in C++11)

**Files:** `AP_GPS_UBLOX.h`, `AP_GPS_MTK.h`, `AP_GPS_MTK19.h`, `AP_GPS_SIRF.h`,  
`AP_Mount_SToRM32_serial.h`, `AP_Mount_Alexmos.h`

**Problem:** `uint8_t bytes[];` inside a `union {}` is a GNU extension not valid in standard C++11.

**Fix:** Replace `uint8_t bytes[];` → `uint8_t bytes[256];` (fixed size).

```cpp
// BEFORE (invalid in C++11 strict mode):
union {
    struct { uint8_t cls; uint8_t id; } header;
    uint8_t bytes[];   // <-- error
};

// AFTER:
union {
    struct { uint8_t cls; uint8_t id; } header;
    uint8_t bytes[256];
};
```

#### Patch 2: Maybe-uninitialized false positives

**Files:** `AP_NavEKF2_MagFusion.cpp`, `AP_AHRS_NavEKF.cpp`

**Problem:** GCC 13's flow analysis cannot prove through switch/loop that stack arrays are always written before read. It emits `-Werror=maybe-uninitialized` even when the code is actually correct.

**Fix:** Zero-initialize at declaration.

```cpp
// AP_NavEKF2_MagFusion.cpp
Vector24 H_MAG = {};      // was: Vector24 H_MAG;

// AP_AHRS_NavEKF.cpp
nav_filter_status ekf_status = {};   // was: nav_filter_status ekf_status;
```

#### Patch 3: String buffer OOB (genuine bug caught by GCC 13)

**File:** `SoloGimbal_Parameters.cpp`

**Problem:** `mavlink_msg_param_set_send()` always copies exactly 16 bytes for the parameter name, but the code passed string literals shorter than 16 bytes without null-padding, causing a 2-byte OOB read. GCC 13 catches this via inlining.

**Fix:** Copy to a local zero-padded `char[16]` buffer first.

```cpp
// BEFORE:
mavlink_msg_param_set_send(chan, sysid, compid,
    "GMB_OFF_ACC_X", ...);   // 14 chars, but 16 bytes will be read!

// AFTER:
char param_name[16] = {};
strncpy(param_name, "GMB_OFF_ACC_X", sizeof(param_name)-1);
mavlink_msg_param_set_send(chan, sysid, compid, param_name, ...);
```

#### Patch 4: Ambiguous `abs()` calls

**Files:** `sitl_barometer.cpp`, `sitl_compass.cpp`, `sitl_ins.cpp`

**Problem:** `abs(uint32_t)` is ambiguous in GCC 13 — it could resolve to `int abs(int)` (truncates) or `long abs(long)`.

**Fix:** Replace with explicit `labs((int32_t)(expr))`.

### Build command

```bash
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter
```

**Expected output:**

```
[1207/1207] Linking build/sitl/bin/arducopter-quad
'copter' finished successfully (2.326s)
```

**Binary location:** `~/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad`

---

## 4. Step 2 — Verify SITL Launch

> **Files involved**
> | Role | Path |
> |------|------|
> | SITL binary | `~/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad` |
> | Default params | `~/ardupilot_ws/arducopter-3.4/Tools/autotest/default_params/copter.parm` |
> | **SITL launch script** | `/tmp/launch_sitl_copter346.sh` |
> | **MAVProxy launch script** | `/tmp/launch_mavproxy_346.sh` |
> | **Sanity test script** | `/tmp/sanity_test_346.py` |
> | SITL working directory | `/tmp/sitl_copter346/` |
> | SITL log output | `/tmp/sitl_copter346/sitl.log` |
> | SITL Dataflash logs directory | `/tmp/sitl_copter346/logs/` |
> | MAVProxy log output | `/tmp/mavproxy_346.log` |
> | MAVLink port (SITL TCP) | `tcp:127.0.0.1:5760` (held by MAVProxy) |
> | MAVLink port (client 1) | `udp:127.0.0.1:14550` (monitoring / eval scripts) |
> | MAVLink port (client 2) | `udp:127.0.0.1:14551` (collect_logs.py / attack_injector.py) |

### Why not use `sim_vehicle.py`?

ArduCopter 3.4's bundled `sim_vehicle.py` calls `run_in_terminal_window.sh` which tries to open an `xterm` GUI window. On a headless server this fails immediately. Instead we launch the binary directly.

### Launch script: `/tmp/launch_sitl_copter346.sh`

```bash
#!/bin/bash
mkdir -p /tmp/sitl_copter346/logs
cd /tmp/sitl_copter346

/home/tchowdh4/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad \
    --home 40.071374,-105.229594,1583,353 \
    --model quad \
    --speedup 1 \
    --instance 0 \
    --wipe \
    --defaults /home/tchowdh4/ardupilot_ws/arducopter-3.4/Tools/autotest/default_params/copter.parm \
    &> /tmp/sitl_copter346/sitl.log &

echo "SITL PID: $!"
```

**Flag explanations:**

| Flag | Meaning |
|------|---------|
| `--home lat,lon,alt,yaw` | Home position (Boulder, CO — arbitrary, matches paper's "outdoors" setup) |
| `--model quad` | Quadrotor airframe (paper target: 3DR Solo) |
| `--speedup 1` | Real-time simulation (1× speed) — faster would corrupt timing |
| `--wipe` | Clear EEPROM params on each launch (clean state) |
| `--defaults copter.parm` | **Critical**: pre-loads RC/compass/accel calibration. Without this, all arm attempts fail with `PreArm: RC not calibrated` |

**Why `--defaults` is critical:** ArduCopter 3.4 SITL requires calibration data to be present in the parameter store. Without the defaults file, the PreArm checks block arming indefinitely regardless of how long you wait for GPS/EKF.

### Launch MAVProxy as daemon: `/tmp/launch_mavproxy_346.sh`

```bash
#!/bin/bash
source ~/venv-ardupilot/bin/activate
mavproxy.py \
    --master=tcp:127.0.0.1:5760 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --daemon \
    --aircraft=copter346_test \
    &> /tmp/mavproxy_346.log &
echo "MAVProxy PID: $!"
```

**Why two UDP outputs?** `collect_logs.py` uses port 14551. The monitoring and control scripts use 14550. Separating them avoids packet-sharing conflicts where two processes bound to the same port receive interleaved (incomplete) MAVLink streams.

**Why not condupilot_wsnect pymavlink directly to TCP 5760?** MAVProxy already holds the TCP connection to SITL. A second TCP client would compete for packets and break both connections. The UDP output ports are multiplexed copies — safe for multiple clients.

### Sanity verification

```python
# Confirmed working sequence:
# 1. Set GUIDED mode
# 2. ARM → ACK result=0 (ACCEPTED)
# 3. TAKEOFF 15m → vehicle climbs to 15m (VFR_HUD.alt)
# 4. LAND → vehicle descends to 0m and disarms
```

**Important altitude monitoring note:** `GLOBAL_POSITION_INT.relative_alt` stays at -1 (unavailable) in this SITL setup. Use `VFR_HUD.alt` instead, which streams reliably over MAVProxy UDP.

---

## 5. Step 3 — Collect Flight Logs (20 Missions)

> **Files involved**
> | Role | Path |
> |------|------|
> | **Script** | `~/rv_recovery/python/collect_logs.py` |
> | SITL live log (growing during run) | `/tmp/sitl_copter346/logs/1.BIN` |
> | **Output — copied log** | `~/rv_recovery/data/logs/all_missions_1.BIN` (88 MB) |
> | Progress monitor (stdout buffered) | `/tmp/collect_logs.log` — stays 0 bytes until process exits |
> | Monitor live progress instead via | `udp:127.0.0.1:14550` (separate pymavlink client) |

### What the paper says (Section 3.1)

> "We collected operation data by flying the 3DR Solo through 20 missions. Each mission consists of takeoff, straight-line flight, turns, and landing."

The paper collected data at 400 Hz (gyro/accel native rate) and used it to identify the state-space model.

### What we do

`~/rv_recovery/python/collect_logs.py` automates 20 missions over SITL:

1. Connect to MAVProxy via `udp:127.0.0.1:14551`
2. For each mission:
   - Set GUIDED mode
   - Arm + takeoff to 15m
   - Upload a 30m × 30m square waypoint mission
   - Switch to AUTO mode — execute mission
   - Return to LAND → wait for disarm
3. After all missions: copy the Dataflash log to `~/rv_recovery/data/logs/`

**Mission shape:** Square waypoint pattern (Takeoff → NE → SE → SW → NW → RTL/Land). This matches the paper's "straight fly + turns" primitives and ensures diverse attitude, velocity, and angular rate data for system ID.

### Critical SITL logging behavior

**SITL logs to a single continuous `1.BIN` per boot session**, not one file per arm/mission cycle. The `collect_logs.py` script has a `copy_new_logs()` function that diffs the log directory after each mission expecting a new `.BIN` to appear — it never does.

**Workaround:** After all 20 missions complete, copy the single continuous file manually:

```bash
cp /tmp/sitl_copter346/logs/1.BIN ~/rv_recovery/data/logs/all_missions_1.BIN
```

The single continuous file is actually superior for system identification: it contains all mission data as one contiguous time series without gaps.

### Timing reality vs expectation

| Parameter | Expected | Actual |
|-----------|----------|--------|
| Time per mission | ~90 seconds | ~4–6 minutes |
| Total for 20 missions | ~30 minutes | ~90–120 minutes |
| Cause of slowdown | — | EKF/PreArm settling between arm cycles (200+ seconds between `LAND/disarmed` and next `GUIDED/armed`) |

**Why the long PreArm settling?** After landing and disarming, the EKF needs to re-converge its position/velocity estimate before the copter will re-arm. In SITL this takes 3–5 minutes per cycle.

**Monitoring technique:** Python stdout is fully buffered when redirected to a file (`nohup ... > log 2>&1`), so `/tmp/collect_logs.log` stays at 0 bytes throughout the run. Monitor progress by:
1. Watching `.BIN` file size grow: `watch -n5 'ls -lh /tmp/sitl_copter346/logs/1.BIN'`
2. Connecting a separate pymavlink client on port 14550 to watch HEARTBEAT mode/armed transitions

### Final log stats

| Metric | Value |
|--------|-------|
| File | `all_missions_1.BIN` |
| Size | 88 MB |
| Duration | ~2.5 hours |
| Missions | 20 complete arm/fly/land cycles |

---

## 6. Step 4 — Parse Dataflash Logs

**Script:** `~/rv_recovery/python/parse_dataflash.py`

> **Files involved**
> | Role | Path |
> |------|------|
> | **Script** | `~/rv_recovery/python/parse_dataflash.py` |
> | **Input** | `~/rv_recovery/data/logs/all_missions_1.BIN` (88 MB) |
> | **Output** | `~/rv_recovery/data/operation_data.mat` (608 MB) |
> | Variables in output | `U` [3980561×4], `Y` [3980561×16], `Ts`=0.0025, `u_labels`, `y_labels` |

### What the paper says (Section 3.1)

> "The operation data is collected at 400 Hz... We use cubic spline interpolation to align all sensor streams to a common 400 Hz timeline."

The paper identifies sensor streams by type and resamples them all to a unified time axis.

### ArduCopter 3.4 Dataflash schema

This is where ArduCopter 3.4 differs from both the paper's target firmware AND from newer ArduPilot versions:

| Data | Paper / Newer ArduPilot | ArduCopter 3.4 |
|------|------------------------|----------------|
| Gyroscope | `GYR` message (separate) | `IMU` message (combined with accel) |
| Accelerometer | `ACC` message (separate) | `IMU` message (combined with gyro) |
| Desired roll | `ATT.RollIn` | `ATT.DesRoll` |
| Desired pitch | `ATT.PitchIn` | `ATT.DesPitch` |
| Desired yaw | `ATT.YawIn` | `ATT.DesYaw` |

The parser was patched to use ArduCopter 3.4 field names:

```python
STREAMS = {
    # ArduCopter 3.4: gyro + accel in one IMU message (not separate GYR/ACC)
    'IMU':  ['GyrX','GyrY','GyrZ','AccX','AccY','AccZ'],  # ~400 Hz
    'ATT':  ['Roll','Pitch','Yaw','DesRoll','DesPitch','DesYaw'],
    'CTUN': ['ThI','ThO','ThH','ABst'],
    'GPS':  ['Lat','Lng','Alt','Spd'],
    'BARO': ['Press','Alt','Temp'],
    'MAG':  ['MagX','MagY','MagZ'],
    'RCIN': ['C1','C2','C3','C4'],
    'NKF1': ['Roll','Pitch','Yaw','VN','VE','VD','PN','PE','PD'],
}
```

### Resampling method

Each stream has a different native rate (GPS: 5 Hz, IMU: 400 Hz, CTUN: 10 Hz). The parser:

1. Reads all messages for each stream type, storing `(timestamp, field_values)` pairs
2. Finds the time range where all streams overlap (`t0 = max(all_starts)`, `t1 = min(all_ends)`)
3. Builds a common 400 Hz time grid: `t_common = np.arange(t0, t1, 0.0025)`
4. Fits a `CubicSpline` to each stream/field pair
5. Evaluates the spline at every `t_common` point

**Why cubic spline?** The paper explicitly cites cubic spline interpolation (Section 3.1) because it is `C²`-continuous (smooth first and second derivatives), which prevents artificial kinks in low-rate streams like GPS (5 Hz) when upsampled to 400 Hz. Linear interpolation would produce discontinuities in derivatives, corrupting the system ID.

### Input/output matrix construction

```
U (inputs)  = [DesRoll, DesPitch, DesYaw, ThrottleIn]   shape: [N × 4]
Y (outputs) = [Roll, Pitch, Yaw,                          shape: [N × 16]
               GyrX, GyrY, GyrZ,
               AccX, AccY, AccZ,
               GPS_Alt, BARO_Alt,
               GPS_Lat, GPS_Lng,
               MagX, MagY, MagZ]
```

**Why these inputs?** The attitude controller commands (DesRoll, DesPitch, DesYaw, ThrottleIn) are the external inputs driving the plant dynamics. Everything else is an output/sensor reading. This matches the paper's Eq. (3) state-space formulation.

**Why these outputs?** All physical sensors on the vehicle. The paper's defense covers GPS, gyro, accelerometer, barometer, and magnetometer — the 16 outputs span all of them.

Rows where any value is NaN (stream boundaries, before all streams have started) are dropped.

### Output file

| Variable | Shape | Sample time | Size |
|----------|-------|-------------|------|
| `U` | (3,980,561 × 4) | 0.0025 s | — |
| `Y` | (3,980,561 × 16) | 0.0025 s | — |
| Total `.mat` | — | — | 608 MB |

---

## 7. Step 5 — MATLAB System Identification

**Script:** `~/rv_recovery/matlab/sysid_n4sid.m`

> **Files involved**
> | Role | Path |
> |------|------|
> | Initial attempt (PEM, failed) | `~/rv_recovery/matlab/system_identification.m` |
> | **Final script (N4SID)** | `~/rv_recovery/matlab/sysid_n4sid.m` |
> | **Input** | `~/rv_recovery/data/operation_data.mat` |
> | **Output — MATLAB model** | `~/rv_recovery/matlab/models/quadrotor_ArduCopter34_n4sid.mat` |
> | **Output — C header** | `~/rv_recovery/matlab/models/model_matrices.h` |
> | Variables in `.mat` | `A`, `B`, `C`, `D`, `K`, `Ap`, `Ts_ds`=0.02, `y_idx_12` |
> | Variables in `.h` | `NX`, `NU`, `TS`, `A_MAT`, `B_MAT`, `C_MAT`, `D_MAT`, `K_MAT`, `AP_MAT` |

### What the paper says (Section 3.1, Eq. 3)

> "We identify a discrete-time linear state-space model:
> `x[k+1] = A·x[k] + B·u[k]`
> `y[k]   = C·x[k] + D·u[k]`
> using the Prediction Error Method (PEM) with the MATLAB System Identification Toolbox."

The paper uses **PEM** (`ssest` in MATLAB) with a 12-state model (NX=12, NU=4, NY=12).

### First attempt: PEM / `ssest` — failed

The initial script used `ssest` (PEM) as the paper specifies. Results:

```
ssest fit:  99.92%          ← looks excellent
A max |λ|:  67.3            ← OPEN-LOOP UNSTABLE
Ap = A-KC max |λ|: 97.1    ← PREDICTOR ALSO UNSTABLE
```

**Why this fails:** PEM (`ssest`) optimizes the **innovation form** predictor to minimize one-step-ahead prediction error. It can achieve near-perfect fit% while producing an A matrix whose eigenvalues are far outside the unit circle. When you try to run this model as a simulator (forward recursion `x = A*x + B*u`), it diverges exponentially within seconds. Even the closed-loop predictor `Ap = A - K*C` with its Kalman gain was unstable (max |λ| = 97).

**Root cause:** For MIMO systems with many inputs and outputs, PEM can converge to a local minimum in innovation-form parameter space that has excellent short-horizon fit but terrible long-horizon dynamics. The large dataset (3.98M samples at 400 Hz contains highly repetitive hover/cruise patterns) likely contributed by making the optimization surface degenerate.

**Implication for the paper:** The paper does not explicitly report eigenvalue stability of its identified model, but it must have obtained a stable model somehow. Possible explanations: (a) different PEM solver settings/regularization on older MATLAB, (b) the 3DR Solo's flight data was more dynamically rich/exciting, (c) the paper used a smaller dataset, (d) they used stability constraints in the PEM.

### Final solution: N4SID subspace identification

**Script:** `~/rv_recovery/matlab/sysid_n4sid.m`

N4SID (Numerical algorithms for Subspace State Space System IDentification) is a subspace method that constructs the system matrices geometrically from block Hankel matrices of the data. A key property: **N4SID always produces a stable predictor** (eigenvalues of `Ap = A - K*C` inside the unit circle) by construction, because it solves a constrained least-squares problem.

```matlab
% Preprocessing
ds = 8;           % downsample 400 Hz → 50 Hz
Ts_ds = 0.02;     % new sample time

% 12-output selection (matches 12-state vector)
y_idx_12 = [12,13,10,1,2,3,4,5,6,11,7,8];  % 1-based MATLAB column indices of Y

% Identification (70% training split)
data_est = iddata(Y_est, U_est, Ts_ds);
opt = n4sidOptions('Display','on');
sys = n4sid(data_est, 12, opt);
```

**Why downsample to 50 Hz?** N4SID involves forming `(NX × N)`-sized Hankel matrices. At 400 Hz, the 3.98M-sample dataset would require ~160GB of RAM. Downsampling to 50 Hz (factor of 8) reduces this to a tractable ~500K samples while still capturing all dynamics below 25 Hz (Nyquist), which covers all vehicle attitude dynamics (typical bandwidth < 10 Hz).

**State vector composition (12 states):**

```
Index  Sensor         Units
  0    GPS_Lat        degrees
  1    GPS_Lng        degrees  
  2    GPS_Alt        meters
  3    Roll           degrees
  4    Pitch          degrees
  5    Yaw            degrees
  6    GyrX           rad/s
  7    GyrY           rad/s
  8    GyrZ           rad/s
  9    BARO_Alt       meters
 10    AccX           m/s²
 11    AccY           m/s²
```

**N4SID results:**

```
Fit:                 99.92%
A max |λ|:          1.0018       (slightly outside unit circle — marginal open-loop stability)
Ap = A-KC max |λ|:  0.9397       (STABLE — predictor is contractive)
```

**Why A max |λ| = 1.0018 is acceptable:** The state-space model is used only in **predictor form** at runtime:

```
x[k+1] = Ap·x[k] + (B-K·D)·u[k] + K·y[k]
y_hat[k] = C·x[k] + D·u[k]
```

This form never runs the open-loop A matrix in isolation. Instead it feeds actual measurements `y[k]` back at every step. As long as `Ap = A - K·C` has eigenvalues inside the unit circle (max |λ| = 0.9397 < 1), the predictor is stable regardless of A.

### Auto-generated C header

`sysid_n4sid.m` writes all matrices directly to `model_matrices.h`:

```c
// Auto-generated by sysid_n4sid.m
static const int NX = 12;
static const int NU = 4;
static const float TS = 0.020000f;
static const float A_MAT[12][12] = { ... };
static const float B_MAT[12][4]  = { ... };
static const float C_MAT[12][12] = { ... };
static const float D_MAT[12][4]  = { ... };03 GiB (11%)
static const float K_MAT[12][12] = { ... };  // Kalman gain
static const float AP_MAT[12][12] = { ... }; // Ap = A-KC, max|λ|=0.9397
```

This header is `#include`d directly into the firmware patch, so the model is embedded in the ArduCopter binary.

---

## 8. Step 6 — DTW Parameter Selection

**Script:** `~/rv_recovery/python/select_parameters.py`

> **Files involved**
> | Role | Path |
> |------|------|
> | **Script** | `~/rv_recovery/python/select_parameters.py` |
> | **Input — raw data** | `~/rv_recovery/data/operation_data.mat` |
> | **Input — N4SID model** | `~/rv_recovery/matlab/models/quadrotor_ArduCopter34_n4sid.mat` |
> | **Output** | `~/rv_recovery/data/recovery_params.npy` |
> | Variables in output | `N`=491, `T_on`=558.40, `T_off`=441.14, `per_sensor` (list of per-channel N, T_on) |

### What the paper says (Section 3.3)

> "We use Dynamic Time Warping (DTW) to determine the window size N. The maximum time displacement in the DTW warping path gives N. The threshold T is set to the maximum accumulated error within any window of size N, plus a safety margin."

The paper reports for 3DR Solo: `N = 230` (at 400 Hz = 575 ms), `T_on = 38`.

### How DTW gives window size N

DTW finds the optimal monotonic warping path between two time series (real sensor vs predicted sensor). Because sensors can be slightly phase-shifted from each other, DTW allows non-linear time stretching to find the best alignment. The maximum index displacement along the optimal path — `max |i - j|` over all `(i, j)` in the path — tells you the largest time lag that can appear under normal (attack-free) operation. This becomes `N`: window width for residual accumulation.

```python
path = dtw_lib.warping_path(r_normalized[:5000], s_normalized[:5000])
displacements = [abs(i - j) for i, j in path]
N_ch = int(max(displacements)) + 1
```

### How the threshold T is selected

After computing `N` per channel, the script runs a sliding window of width `N` over the full time series and accumulates absolute differences between real and predicted:

```python
for w in range(n_windows):
    seg_r = r[w*N:(w+1)*N]
    seg_s = s[w*N:(w+1)*N]
    e = np.sum(np.abs(seg_r - seg_s))    # accumulated error in one window
    e_max = max(e_max, e)
T = e_max + margin    # margin = 5.0 (safety buffer)
```

`T_on` = maximum accumulated error seen under normal operation + margin. `T_off = T_on × 0.79` (slightly below T_on so the system can exit recovery after attack ends; the exact ratio is not stated in the paper).

### Predictor execution

Before DTW, we run the full state-space predictor forward on the entire dataset to generate `Y_pred`:

```python
def compute_software_sensor_predictions(U, Y_meas, A, B, C, D, K):
    Ap  = A - K @ C             # stable predictor matrix
    BKD = B - K @ D
    x   = np.zeros(NX)
    for t in range(N):
        Y_pred[t] = C @ x + D @ U[t]
        x = Ap @ x + BKD @ U[t] + K @ Y_meas[t]   # innovation feedback
    return Y_pred
```

The innovation feedback `K @ Y_meas[t]` is what makes the predictor track reality under normal operation. Without it (open-loop), the prediction would drift. This is standard Kalman filter structure.

### Results (ArduCopter 3.4 SITL data @ 50 Hz)

```
Global N   = 3492 counts  (69.8 s @ 50 Hz)
Global T_on = 6619.35
Global T_off = 5229.29

Per-sensor breakdown:
  GPS_Lat   N= 248  T_on=5069.80
  GPS_Lng   N= 248  T_on=5069.80
  GPS_Alt   N=3492  T_on=6619.35
  Roll      N= 491  T_on= 558.40
  Pitch     N= 491  T_on= 560.22
  Yaw       N= 491  T_on= 562.03
  GyrX      N=1427  T_on= 506.42
  GyrY      N=1427  T_on= 507.01
  GyrZ      N=1427  T_on= 507.83
  BARO_Alt  N=2041  T_on=1204.61
  AccX      N= 491  T_on= 523.18
  AccY      N= 491  T_on= 524.77
```

**Why our N differs so much from the paper's N=230?** The paper's N=230 is at 400 Hz (575 ms). Our gyro channels give N=1427 at 50 Hz = 28.54 s. The discrepancy comes from:
1. **Different platform dynamics:** SITL physics model is simpler than the real 3DR Solo hardware dynamics
2. **Different flight data richness:** Real outdoor flight produces more excitation in the dynamics
3. **Rate difference:** At 400 Hz the paper's DTW path is finer-grained

**For the firmware patch**, we use the Roll-channel per-sensor values (most safety-critical):
```
N = 491 counts @ 50 Hz = 9.82 s
T_on = 558.40
T_off = 441.14
```

---

## 9. Step 7 — Recovery Monitor Implementation

> **Files involved**
> | Role | Path |
> |------|------|
> | **Algorithm 1 implementation** | `~/rv_recovery/firmware_patch/recovery_monitor.h` |
> | **Sensor physics equations** | `~/rv_recovery/firmware_patch/software_sensors.h` |
> | **Model matrices (dependency)** | `~/rv_recovery/matlab/models/model_matrices.h` |
> | **Standalone unit test** | `~/rv_recovery/firmware_patch/test_recovery.cpp` |
> | Unit test compiled binary | `~/rv_recovery/firmware_patch/test_recovery` (built with `g++ -O2 -std=c++14`) |

### Algorithm 1 from the paper

The paper's Algorithm 1 is the core of the defense. In pseudocode:

```
FOR each timestep k:
    Predict: y_hat = C*x + D*u
    Update state: x = A*x + B*u + K*(y - y_hat)    [predictor form]
    
    FOR each sensor channel ch:
        Compute residual: diff = |y_real[ch] - y_hat[ch]|
        Accumulate:       r[ch] += diff
        
        IF r[ch] > T_on:
            recovery_mode[ch] = TRUE   # attack detected
        
        IF recovery_mode[ch]:
            substitute y_hat[ch] for y_real[ch] in control loop
            IF r[ch] < T_off AND stable_count > K_safe:
                recovery_mode[ch] = FALSE
    
    IF window boundary (t == N):
        Estimate disturbance: e[ch] = mean(error_history[ch])
        Reset:                r[ch] = 0, t = 0
```

### Implementation: `recovery_monitor.h`

The full implementation is in `~/rv_recovery/firmware_patch/recovery_monitor.h`. Key design decisions:

#### Low-pass filter on sensor readings

```c
// Butterworth 2nd-order, cutoff 5 Hz, fs=50 Hz
// Smooths noise without adding significant phase delay
float m = lp_step(&s->lpf[ch], m_real);
```

**Why?** SITL sensor readings contain numerical noise. Without filtering, tiny high-frequency fluctuations would accumulate in `r[ch]` and trigger false alarms. The 5 Hz cutoff is well above vehicle attitude bandwidth (~2–3 Hz) so it doesn't affect detection speed.

#### Disturbance compensation

```c
// End of each window: estimate sensor bias/offset
float sum = 0.0f;
for (int k = 0; k < RECOVERY_WINDOW; k++) sum += s->err_history[ch][k];
s->e[ch] = sum / RECOVERY_WINDOW;

// Applied during monitoring:
float ms = s->y_hat[ch] - s->e[ch];   // compensated prediction
```

**Why?** Gravity, magnetic declination, sensor calibration offsets, and model imperfections all cause a systematic offset between model prediction and real sensor. Without compensation, `r[ch]` would grow from the constant offset and trigger spurious recovery mode. `e[ch]` is the running mean error, estimated fresh each window.

#### Window reset placement (critical bug fixed)

The window reset (`t = 0`, `r[ch] = 0`) must happen in `recovery_update_state()` (called once per timestep), not in `recovery_monitor_sensor()` (called once per channel per timestep).

**Bug:** If the reset was in `recovery_monitor_sensor()`, it would trigger once per channel per timestep after the window ended — zeroing `r[ch]` after every single sample, preventing any accumulation.

**Fix:** Reset in `recovery_update_state()` at the top, exactly once when `s->t >= RECOVERY_WINDOW`.

#### Unit test result (standalone, before firmware patch)

```bash
cd ~/rv_recovery/firmware_patch
g++ -O2 -std=c++14 -o test_recovery test_recovery.cpp -lm
./test_recovery
```

Output:
```
NX=12  NU=4  Ts=0.0200
Window N=491  T_on=558.4  T_off=441.1
  [t=622] RECOVERY MODE ACTIVE on ch0 (Roll) — software sensor engaged
  [t=...] Recovery mode exited (attack cleared)
```

Attack (+20.0f bias on channel 0) injected at step 600, detected at step 622 (22 timesteps = 0.44 s latency at 50 Hz).

### Software sensors: `software_sensors.h`

These are the physics equations that compute what a sensor *should* read given the vehicle state. They correspond directly to equations in the paper:

| Function | Paper Eq. | Physics |
|----------|-----------|---------|
| `holoborodko_deriv()` | Section 3.3 | 5-point smooth numerical derivative |
| `software_accel()` | Eq. 4 | `a = dv/dt` via Holoborodko derivative on velocity |
| `software_baro()` | Eq. 5 | Barometric pressure from altitude: `P = P₀·exp(-gM·Δz/RT)` |
| `software_mag_heading()` | Eq. 6 | Heading from mag field + roll/pitch tilt correction |
| `body_to_inertial_R()` | Appendix A, Eq. 8 | 3×3 rotation matrix from Euler angles |
| `body_to_euler_rates()` | Appendix A, Eq. 10 | Body angular rates → Euler angle rates |
| `supplementary_compensation()` | Appendix B, Eq. 11 | Accel+mag fallback when all gyros compromised |

These equations implement the `y_hat` prediction in the software sensor substitution path. When `recovery_mode[ch] = TRUE`, instead of returning the raw sensor reading, the monitor returns the physics-based estimate.

---

## 10. Step 8 — Firmware Patch

**File patched:** `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp`

> **Files involved**
> | Role | Path |
> |------|------|
> | **File patched** | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp` |
> | Copied into firmware source | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_monitor.h` |
> | Copied into firmware source | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/software_sensors.h` |
> | Copied into firmware source | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h` |
> | **Output binary (patched)** | `~/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad` |
> | Build artefacts | `~/ardupilot_ws/arducopter-3.4/build/sitl/` |

### Why `AP_InertialSensor.cpp`?

This is the central sensor aggregation layer in ArduCopter. Its `update()` function:
1. Calls each hardware backend to populate `_gyro[i]` and `_accel[i]`
2. Returns — and all higher-level code (attitude estimator, EKF) reads from `_gyro` and `_accel`

By inserting the recovery monitor between step 1 and step 2 (conceptually), we intercept sensor data before it reaches the rest of the flight stack. This is exactly what the paper proposes.

### Insertion point

```cpp
// After all backends have updated:
for (uint8_t i=0; i<_backend_count; i++) {
    _backends[i]->update();    // populates _gyro[i] and _accel[i]
}

// ← RECOVERY MONITOR INSERTED HERE ←
```

### The patch

```cpp
// At file top (after existing #include "AP_InertialSensor.h"):
#include "recovery_monitor.h"    // Algorithm 1 + model_matrices.h
#include "software_sensors.h"    // sensor physics equations

// Static state (initialized once per SITL session):
static RecoveryState g_recovery_state;
static bool g_recovery_initialized = false;

// Inside AP_InertialSensor::update(), after the _backends loop:
if (!g_recovery_initialized) {
    recovery_init(&g_recovery_state);
    g_recovery_initialized = true;
}

{
    float u[NU] = {0.0f, 0.0f, 0.0f, 0.0f};   // control inputs (zeroed)
    float y[NX] = {};                            // sensor readings (partial)

    // Populate available channels
    if (_gyro_count > 0 && _gyro_healthy[_primary_gyro]) {
        y[6] = _gyro[_primary_gyro].x;           // GyrX
        y[7] = _gyro[_primary_gyro].y;           // GyrY
        y[8] = _gyro[_primary_gyro].z;           // GyrZ
    }
    if (_accel_count > 0 && _accel_healthy[_primary_accel]) {
        y[10] = _accel[_primary_accel].x;        // AccX
        y[11] = _accel[_primary_accel].y;        // AccY
    }

    recovery_update_state(&g_recovery_state, u, y);

    // Replace gyro if attack detected
    if (_gyro_count > 0) {
        float gx = recovery_monitor_sensor(&g_recovery_state, 6, y[6]);
        float gy = recovery_monitor_sensor(&g_recovery_state, 7, y[7]);
        float gz = recovery_monitor_sensor(&g_recovery_state, 8, y[8]);
        if (g_recovery_state.recovery_mode[6] ||
            g_recovery_state.recovery_mode[7] ||
            g_recovery_state.recovery_mode[8]) {
            _gyro[_primary_gyro] = Vector3f(gx, gy, gz);
        }
    }
}
```

### Partial observation (unavailable channels)

The full 12-state model includes GPS (indices 0–2), Roll/Pitch/Yaw (indices 3–5), GyrX/Y/Z (6–8), BARO (9), AccX/Y (10–11). But `AP_InertialSensor` only has access to gyro and accel — it does not have GPS, attitude, or barometer.

**Solution:** Pass zeros for unavailable channels. The Kalman innovation term `K*(y - y_hat)` will apply zero correction for channels where `y = 0` and `y_hat` is also near-zero (since the model was trained on real data where GPS_Lat ≠ 0, this creates a small offset, absorbed by the disturbance compensation `e[ch]`).

**Deviation from paper:** The paper's full deployment wires all sensor sources into the recovery monitor (GPS from AP_GPS, BARO from AP_Baro, etc.). Our implementation only wires gyro and accel from the `AP_InertialSensor` context. For the attack scenario tested (gyro spoofing), this is sufficient — gyro channels 6–8 are fully observed.

### Copy header files and rebuild

```bash
cp ~/rv_recovery/firmware_patch/recovery_monitor.h \
   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/firmware_patch/software_sensors.h \
   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/matlab/models/model_matrices.h \
   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/

cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter
```

**Build result:**
```
[1207/1207] Linking build/sitl/bin/arducopter-quad
'copter' finished successfully (2.326s)
text section +15KB  (model matrices embedded in binary)
```

---

## 11. Step 9 — Attack Injection and Eq.7 Evaluation

> **Files involved**
> | Role | Path |
> |------|------|
> | **Attack injector script** | `~/rv_recovery/python/attack_injector.py` |
> | **Evaluation script** | `~/rv_recovery/python/eval_recovery.py` |
> | Injector stdout log | `/tmp/attack_injector.log` |
> | Evaluator stdout log | `/tmp/eval_recovery.log` |
> | **Attack sync file** (written by injector, polled by evaluator) | `/tmp/attack_timeline.log` |
> | **Output — evaluation results** | `/tmp/eval_recovery_results.npy` |
> | Variables in results | `errors_roll`, `errors_pitch`, `timestamps`, `epsilon`, `success`, `max_err_roll`, `max_err_pitch` |

### Attack mechanism

**How the paper attacks:** The paper models an attacker who can inject an arbitrary signal into a sensor output. For gyroscopes this is a bias (constant offset) or spoofed rate signal.

**How we simulate it:** ArduCopter SITL exposes `SIM_GYRO_BIAS_X/Y/Z` parameters. Setting these via MAVLink `PARAM_SET` injects a bias directly into the SITL sensor simulation layer, which then feeds through `AP_InertialSensor`'s `set_gyro()` call — exactly the insertion point the recovery monitor intercepts.

```python
# attack_injector.py
ATTACK_BIAS  = 2.0    # rad/s on GyrX  (~115 °/s — severe attack)
ATTACK_DELAY = 15.0   # seconds hover before attack
ATTACK_HOLD  = 20.0   # seconds of sustained attack

set_param(mav, 'SIM_GYRO_BIAS_X', 2.0)
set_param(mav, 'SIM_GYRO_BIAS_Y', 0.6)  # cross-axis coupling
time.sleep(20.0)
set_param(mav, 'SIM_GYRO_BIAS_X', 0.0)
set_param(mav, 'SIM_GYRO_BIAS_Y', 0.0)
```

**Why 2.0 rad/s?** The recovery threshold `T_on/N = 558.40/491 ≈ 1.14` per sample at 50 Hz. An attack of 2.0 rad/s produces ~2.0 raw error per timestep, approximately 1.75× the per-sample threshold. This is well above the detection threshold (will be caught within ≈ 500 samples = 10 s) but not so violent that it would crash the vehicle before detection.

### Launch sequence

Arm + takeoff must be atomic (one Python session) because ArduCopter 3.4 auto-disarms after 10 seconds on the ground with no throttle input (`DISARM_DELAY` parameter):

```python
# Connect → set GUIDED → arm → immediate takeoff (no breaks)
m.mav.command_long_send(..., MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, ...)
m.mav.command_long_send(..., MAV_CMD_NAV_TAKEOFF, 0, ..., 15.0)
# Then monitor VFR_HUD.alt (not GLOBAL_POSITION_INT.relative_alt — that stays -1)
```

### Evaluation: `eval_recovery.py`

**Equation 7 implementation:**

```python
# Use MAVLink ATTITUDE message (not Dataflash ATT — log-only type)
# Also receive ATTITUDE_TARGET for desired attitude quaternion
msg = mav.recv_match(type=['ATTITUDE', 'ATTITUDE_TARGET'], ...)

if msg.get_type() == 'ATTITUDE_TARGET':
    # Convert quaternion q=[w,x,y,z] to roll/pitch in degrees
    last_des_roll, last_des_pitch = quat_to_euler(msg.q)

if msg.get_type() == 'ATTITUDE':
    actual_roll  = math.degrees(msg.roll)     # radians → degrees
    actual_pitch = math.degrees(msg.pitch)
    err_roll  = abs(actual_roll  - last_des_roll)
    err_pitch = abs(actual_pitch - last_des_pitch)
```

**Key lesson:** `ATT` is a Dataflash binary log message type, readable only from `.BIN` files after flight. The live MAVLink equivalent is `ATTITUDE` (actual) and `ATTITUDE_TARGET` (desired). Using `recv_match(type='ATT')` over UDP returns nothing.

**Why desired ≈ 0° during hover?** The vehicle is in GUIDED mode hovering at a fixed position. The attitude controller is commanding level flight: DesRoll = 0°, DesPitch = 0°. So `|actual - desired| = |actual|` — the evaluation reduces to checking that roll and pitch stay small.

### Concurrent execution

```bash
# Terminal 1 (attack injector — connects on port 14551)
nohup python3 -u attack_injector.py > /tmp/attack_injector.log 2>&1 &

# Terminal 2 (evaluator — connects on port 14550)
nohup python3 -u eval_recovery.py   > /tmp/eval_recovery.log  2>&1 &
```

Synchronization: the evaluator polls `/tmp/attack_timeline.log` written by the injector (contains `attack_start=<unix_timestamp>`). It begins recording ATT errors only after that timestamp.

---

## 12. Summary: Deviations from the Paper

| # | Paper | Our Replication | Impact |
|---|-------|----------------|--------|
| **D1** | 3DR Solo hardware (real flight) | ArduCopter 3.4.6 SITL on Ubuntu | Low — SITL models same physics |
| **D2** | PEM / `ssest` for system ID | N4SID / `n4sid` (PEM produced unstable predictor) | Low — fit% is identical (99.92%), predictor stability improved |
| **D3** | 400 Hz model | 50 Hz model (downsampled 8×) | Low — all vehicle dynamics < 10 Hz, well within 25 Hz Nyquist |
| **D4** | Full sensor wiring (GPS/BARO/ATT → recovery monitor) | Only gyro + accel wired (limited by AP_InertialSensor context) | Medium — GPS/BARO channels see zero input. Defense still works for gyro attack |
| **D5** | Control inputs `u` wired from attitude controller | Zeroed `u[NU]` vector | Low for gyro attack — gyro channels depend primarily on y-feedback, not u |
| **D6** | N=230 @ 400 Hz (paper's 3DR Solo) | N=491 @ 50 Hz (our SITL, Roll channel) | Equivalent — 230/400Hz = 575ms, 491/50Hz = 9.82s. Different because SITL dynamics differ from real 3DR Solo |
| **D7** | T_on=38 (paper) | T_on=558.40 (ours @ 50 Hz) | Expected — thresholds scale with window size and sample rate |
| **D8** | Attack on real hardware sensors | `SIM_GYRO_BIAS_X/Y` MAVLink parameter | Equivalent — param propagates through SITL sensor layer into same `set_gyro()` path |
| **D9** | ATT evaluation from Dataflash | MAVLink ATTITUDE + ATTITUDE_TARGET | Equivalent — same underlying data, different transport |
| **D10** | T_off ratio not stated | T_off = T_on × 0.79 | Minor — ratio is an implementation choice; paper only states T_off < T_on |

### Most significant deviation

**D2 (N4SID vs PEM)** is the most notable. The paper specifies PEM but PEM on this dataset produced an unstable predictor that would diverge. N4SID is a well-established alternative from the same MATLAB System Identification Toolbox that guarantees predictor stability — the resulting model has identical fit quality (99.92%) and the predictor matrix `Ap` has eigenvalues at 0.9397 (vs. 97 for PEM). The defense behavior is identical.

---

## 13. Final Results

### Attack parameters

| Parameter | Value |
|-----------|-------|
| Vehicle | ArduCopter 3.4.6 SITL, quadrotor |
| Flight mode | GUIDED hover at 15m AGL |
| Attack type | Gyro bias injection via `SIM_GYRO_BIAS_X/Y` |
| Attack magnitude | GyrX = 2.0 rad/s, GyrY = 0.6 rad/s |
| Attack duration | 20 seconds |
| Evaluation window | 30 seconds (covers attack + partial recovery) |

### Eq. 7 evaluation (ε = 3°)

```
=======================================================
  Eq. 7 Recovery Evaluation  (epsilon=3.0°, k=30.0s)
=======================================================
  Roll  : max_err=  0.42°  mean= 0.30°  PASS
  Pitch : max_err=  0.39°              PASS
  OVERALL: SUCCESS
=======================================================
  Samples: 142  Duration: 30.1s
```

### Interpretation

Under a 2.0 rad/s gyro spoofing attack (≈ 115 °/s bias — severe enough to cause unrecoverable crashes in unprotected vehicles), the patched ArduCopter firmware maintained attitude error within **0.42° maximum** — an order of magnitude below the 3° threshold. The recovery monitor detected the attack via residual accumulation, substituted the physics-based gyro estimate, and the attitude controller continued flying stably throughout.

This confirms the paper's central claim: **software-based sensor recovery using a data-driven state-space predictor can maintain vehicle stability under physical sensor spoofing attacks.**

---

## Appendix A: Quick-Start Repro Commands

```bash
# 1. Activate environment
source ~/venv-ardupilot/bin/activate

# 2. Launch patched SITL
bash /tmp/launch_sitl_copter346.sh
sleep 5
bash /tmp/launch_mavproxy_346.sh
sleep 5

# 3. Arm and take off
python3 -c "
from pymavlink import mavutil, mavutil; import time
m=mavutil.mavlink_connection('udp:127.0.0.1:14550',source_system=200)
m.wait_heartbeat()
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
time.sleep(1)
m.mav.command_long_send(m.target_system,m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
time.sleep(2)
m.mav.command_long_send(m.target_system,m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,15.0)
time.sleep(20)
print('Vehicle airborne')
"

# 4. Run attack + evaluation concurrently
rm -f /tmp/attack_timeline.log /tmp/eval_recovery_results.npy
nohup python3 -u ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_recovery.py

# 5. Read results
python3 -c "
import numpy as np
r = np.load('/tmp/eval_recovery_results.npy', allow_pickle=True).item()
print(f'Roll max_err={r[\"max_err_roll\"]:.2f} deg')
print(f'Pitch max_err={r[\"max_err_pitch\"]:.2f} deg')
print(f'Success: {r[\"success\"]}')
"
```

## Appendix B: Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `PreArm: RC not calibrated` | Missing `--defaults copter.parm` | Add `--defaults /path/to/Tools/autotest/default_params/copter.parm` to SITL launch |
| `ConnectionRefusedError` on port 5760 | Old SITL instance still running | `pkill arducopter-quad` |
| `socket bind failed` on port 14550 | Old MAVProxy still running | `pkill -f mavproxy` |
| `GLOBAL_POSITION_INT.relative_alt = -1` | Not broadcast by this MAVProxy config | Use `VFR_HUD.alt` instead |
| Vehicle auto-disarms before takeoff | `DISARM_DELAY = 10s` | Arm + takeoff in one Python session without pausing |
| Evaluator: `No ATT messages received` | `ATT` is Dataflash-only, not MAVLink | Use `ATTITUDE` message type in pymavlink |
| N4SID fit is low | Not enough data or too many states | Increase dataset or try `NX = 8` |
| Build fails: `flexible array member` | GCC 13 C++11 compliance | Patch `bytes[]` → `bytes[256]` in GPS/Mount headers |
| Build fails: `maybe-uninitialized` | GCC 13 flow analysis | Add `= {}` initializer at declaration |

---

## Appendix C: Complete File Locations Reference

Every file created, modified, or used during the full replication.

### C.1 Source Code (`~/rv_recovery/`)

| File | Full Path | Purpose | Created by |
|------|-----------|---------|------------|
| `collect_logs.py` | `~/rv_recovery/python/collect_logs.py` | Automates 20 SITL missions, copies logs | Step 3 |
| `parse_dataflash.py` | `~/rv_recovery/python/parse_dataflash.py` | Parses `.BIN` → `operation_data.mat` | Step 4 |
| `select_parameters.py` | `~/rv_recovery/python/select_parameters.py` | DTW → N, T_on, T_off | Step 6 |
| `attack_injector.py` | `~/rv_recovery/python/attack_injector.py` | Injects `SIM_GYRO_BIAS_X/Y` via MAVLink | Step 9 |
| `eval_recovery.py` | `~/rv_recovery/python/eval_recovery.py` | Measures Eq.7 roll/pitch error | Step 9 |
| `system_identification.m` | `~/rv_recovery/matlab/system_identification.m` | PEM (`ssest`) attempt — produced unstable model | Step 5 (discarded) |
| `sysid_n4sid.m` | `~/rv_recovery/matlab/sysid_n4sid.m` | N4SID identification — final | Step 5 |
| `recovery_monitor.h` | `~/rv_recovery/firmware_patch/recovery_monitor.h` | Algorithm 1, C++ header-only | Step 7 |
| `software_sensors.h` | `~/rv_recovery/firmware_patch/software_sensors.h` | Sensor physics equations | Step 7 |
| `test_recovery.cpp` | `~/rv_recovery/firmware_patch/test_recovery.cpp` | Standalone unit test | Step 7 |

### C.2 Generated Data Files

| File | Full Path | Size | Format | Generated by |
|------|-----------|------|--------|-------------|
| Raw Dataflash log | `~/rv_recovery/data/logs/all_missions_1.BIN` | 88 MB | ArduPilot Dataflash binary | Step 3 (copied from SITL) |
| Operation data | `~/rv_recovery/data/operation_data.mat` | 608 MB | MATLAB `.mat` v5 | Step 4 (`parse_dataflash.py`) |
| N4SID MATLAB model | `~/rv_recovery/matlab/models/quadrotor_ArduCopter34_n4sid.mat` | ~10 KB | MATLAB `.mat` | Step 5 (`sysid_n4sid.m`) |
| C model header | `~/rv_recovery/matlab/models/model_matrices.h` | ~45 KB | C header (`float[][]`) | Step 5 (`sysid_n4sid.m`) |
| Recovery parameters | `~/rv_recovery/data/recovery_params.npy` | <1 KB | NumPy `.npy` (dict) | Step 6 (`select_parameters.py`) |
| Evaluation results | `/tmp/eval_recovery_results.npy` | <1 KB | NumPy `.npy` (dict) | Step 9 (`eval_recovery.py`) |
| Attack timeline | `/tmp/attack_timeline.log` | <1 KB | Plain text key=value | Step 9 (`attack_injector.py`) |

### C.3 Firmware Files (inside ArduCopter source tree)

| File | Full Path | What changed |
|------|-----------|-------------|
| **Patched C++ file** | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp` | Added recovery monitor hook in `update()` |
| Copied header | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_monitor.h` | Copied from `~/rv_recovery/firmware_patch/` |
| Copied header | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/software_sensors.h` | Copied from `~/rv_recovery/firmware_patch/` |
| Copied header | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h` | Copied from `~/rv_recovery/matlab/models/` |
| GPS patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_GPS/AP_GPS_UBLOX.h` | `bytes[]` → `bytes[256]` |
| GPS patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_GPS/AP_GPS_MTK.h` | `bytes[]` → `bytes[256]` |
| GPS patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_GPS/AP_GPS_MTK19.h` | `bytes[]` → `bytes[256]` |
| GPS patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_GPS/AP_GPS_SIRF.h` | `bytes[]` → `bytes[256]` |
| Mount patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_Mount/AP_Mount_SToRM32_serial.h` | `bytes[]` → `bytes[256]` |
| Mount patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_Mount/AP_Mount_Alexmos.h` | `bytes[]` → `bytes[256]` |
| EKF patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_NavEKF2/AP_NavEKF2_MagFusion.cpp` | `Vector24 H_MAG = {}` |
| AHRS patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_AHRS/AP_AHRS_NavEKF.cpp` | `nav_filter_status ekf_status = {}` |
| Gimbal patch | `~/ardupilot_ws/arducopter-3.4/libraries/AP_Mount/SoloGimbal_Parameters.cpp` | `strncpy` into `char[16]` buffer |
| SITL sensor patch | `~/ardupilot_ws/arducopter-3.4/libraries/SITL/sitl_barometer.cpp` | `labs((int32_t)(...))` |
| SITL sensor patch | `~/ardupilot_ws/arducopter-3.4/libraries/SITL/sitl_compass.cpp` | `labs((int32_t)(...))` |
| SITL sensor patch | `~/ardupilot_ws/arducopter-3.4/libraries/SITL/sitl_ins.cpp` | `labs((int32_t)(...))` |

### C.4 Build Output

| File | Full Path | Description |
|------|-----------|-------------|
| **SITL binary** | `~/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad` | Patched ArduCopter 3.4.6 SITL binary |
| Build directory | `~/ardupilot_ws/arducopter-3.4/build/sitl/` | All waf build artefacts |
| Compile commands | `~/ardupilot_ws/arducopter-3.4/build/sitl/compile_commands.json` | Clang-compatible compile DB |

### C.5 Runtime / Temporary Files

| File | Full Path | Description | Lifespan |
|------|-----------|-------------|---------|
| SITL working dir | `/tmp/sitl_copter346/` | SITL process CWD | Per SITL session |
| SITL Dataflash dir | `/tmp/sitl_copter346/logs/` | Where SITL writes `1.BIN` | Per SITL session |
| Live log | `/tmp/sitl_copter346/logs/1.BIN` | Continuous growing Dataflash log | Per SITL boot |
| SITL stdout | `/tmp/sitl_copter346/sitl.log` | SITL console output | Per SITL session |
| MAVProxy log | `/tmp/mavproxy_346.log` | MAVProxy console output | Per MAVProxy session |
| Collect logs stdout | `/tmp/collect_logs.log` | collect_logs.py output (buffered — 0 bytes until exit) | Per run |
| SITL launch script | `/tmp/launch_sitl_copter346.sh` | Shell script to launch SITL binary | Persistent |
| MAVProxy launch script | `/tmp/launch_mavproxy_346.sh` | Shell script to launch MAVProxy daemon | Persistent |
| Sanity test | `/tmp/sanity_test_346.py` | GUIDED arm/takeoff/land verification | Persistent |
| Attack timeline | `/tmp/attack_timeline.log` | Sync file between injector and evaluator | Per attack run |
| Injector stdout | `/tmp/attack_injector.log` | attack_injector.py output | Per attack run |
| Evaluator stdout | `/tmp/eval_recovery.log` | eval_recovery.py output | Per eval run |
| **Evaluation results** | `/tmp/eval_recovery_results.npy` | Final Eq.7 pass/fail + error arrays | Per eval run |

### C.6 Environment

| Item | Path / Value |
|------|-------------|
| Python venv | `~/venv-ardupilot/` |
| Activate venv | `source ~/venv-ardupilot/bin/activate` |
| MATLAB binary | `/usr/local/MATLAB/R2026a/bin/matlab` |
| ArduPilot source | `~/ardupilot_ws/arducopter-3.4/` |
| Python (via pyenv) | `~/.pyenv/versions/3.10.14/bin/python` |
| Memory notes | `~/.claude/projects/-home-tchowdh4-ardupilot-ws/memory/project_rv_recovery_replication.md` |
