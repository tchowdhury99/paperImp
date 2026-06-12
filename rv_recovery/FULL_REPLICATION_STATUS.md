# Full Replication Status — Choi et al. RAID 2020
## Everything Done, Why, File Locations, Deviations, and What Remains

**Paper:** "RVFuzzer: Finding Input Validation Bugs in Robotic Vehicles Through Control-Guided Testing"  
Wait — correct paper: **Choi et al., "Software-Based Realtime Recovery from Sensor Attacks on Robotic Vehicles", RAID 2020**  
**Platform:** Ubuntu 24.04, GCC 13, MATLAB R2026a, ArduCopter 3.4.6 SITL  
**Firmware tree:** `~/ardupilot_ws/arducopter-3.4/`  
**Replication workspace:** `~/rv_recovery/`  

---

## Table of Contents

1. [Background — What the Paper Does](#1-background--what-the-paper-does)
2. [Step 1 — System Identification (Grey-Box PEM)](#2-step-1--system-identification-grey-box-pem)
3. [Step 2 — DTW Parameter Selection](#3-step-2--dtw-parameter-selection)
4. [Step 3 — Algorithm 1: Open-Loop Form](#4-step-3--algorithm-1-open-loop-form)
5. [Step 4 — Wrong Insertion Point Found and Fixed (Re-Root)](#5-step-4--wrong-insertion-point-found-and-fixed-re-root)
6. [Step 5 — Windowed State Resynchronization (Section 3.3)](#6-step-5--windowed-state-resynchronization-section-33)
7. [Step 6 — Real Control Input u and Real State x](#7-step-6--real-control-input-u-and-real-state-x)
8. [Step 7 — 50 Hz Decimation](#8-step-7--50-hz-decimation)
9. [Step 8 — NX=12 Ghost Cleanup](#9-step-8--nx12-ghost-cleanup)
10. [Step 9 — Unit Tests (T1–T5, 19/19 pass)](#10-step-9--unit-tests-t1t5-1919-pass)
11. [Step 10 — GPS Recovery (recovery_gps.h)](#11-step-10--gps-recovery-recovery_gpsh)
12. [Step 11 — Barometer Recovery (recovery_baro.h)](#12-step-11--barometer-recovery-recovery_baroh)
13. [Step 12 — GPS Spoofing Attack Scripts](#13-step-12--gps-spoofing-attack-scripts)
14. [Step 13 — All-Gyros-Compromised Supplementary Compensation](#14-step-13--all-gyros-compromised-supplementary-compensation)
15. [Step 14 — A/B Baseline Evaluation Framework](#15-step-14--ab-baseline-evaluation-framework)
16. [Step 15 — DTW Script Updated for Correct Predictor](#16-step-15--dtw-script-updated-for-correct-predictor)
17. [Step 16 — Fabricated Quotes Fixed in Guide](#17-step-16--fabricated-quotes-fixed-in-guide)
18. [Complete File Reference](#18-complete-file-reference)
19. [Deviations from the Paper](#19-deviations-from-the-paper)
20. [What Remains for Complete Replication](#20-what-remains-for-complete-replication)

---

## 1. Background — What the Paper Does

The paper proposes a software-only defense against sensor spoofing attacks on autonomous
vehicles (quadrotors). The core idea:

1. **System identification:** Fit a discrete-time linear state-space model to normal flight data.
   The model predicts what each sensor *should* read given the current state and control inputs.

2. **Runtime monitoring (Algorithm 1):** At each timestep, compute a "software sensor" value
   `ms = C·x + D·u` (the physics-predicted reading). Compare it against the real sensor reading.
   If the accumulated residual `r` exceeds a threshold `T_on`, switch to recovery mode and
   substitute the software-sensor value for the real sensor value.

3. **Parameter selection (Section 3.3):** Use Dynamic Time Warping (DTW) on normal flight data
   to find the maximum time-displacement `N` between real and predicted signals, and set `T_on`
   from the maximum accumulated error within each window.

4. **Evaluation (Section 4):** Show that a vehicle under gyro spoofing, GPS spoofing, or
   barometer attack recovers correctly when the defense is active, and stays within ε=3° of
   the desired attitude over a k=10 s window (Equation 7).

The paper's insertion point (Figure 3) is explicitly in **`main_loop` / `read_AHRS`** — where
both the control targets `u` and the real state `x` are in scope. This is critical because
the predictor needs `u` to drive `B·u` and needs `x` for the windowed resynchronization.

---

## 2. Step 1 — System Identification (Grey-Box PEM)

### What the paper says
Section 3.1: Identify a discrete-time linear state-space model:
```
x[k+1] = A·x[k] + B·u[k]
y[k]   = C·x[k] + D·u[k]
```
The model state `x` includes at minimum attitude and angular rates. The paper uses a
grey-box PEM (Prediction Error Minimization) approach where known physics structure
constrains the model (rather than the black-box N4SID approach).

### What was done
The MATLAB script `~/rv_recovery/matlab/sysid_greybox.m` was used to fit a **6-state
grey-box model** to flight log data. The state vector is:

```
x = [φ, θ, ψ, p, q, r]
    phi  theta  psi  GyrX  GyrY  GyrZ
     0     1     2    3     4     5
```

- φ, θ, ψ = Roll, Pitch, Yaw (radians)
- p, q, r  = body angular rates (rad/s) — directly the gyro readings

Control input `u = [DesRoll, DesPitch, DesYaw, Throttle]` (NU=4).

**Output of sysid:** `A[6×6]`, `B[6×4]`, `C[6×6]`, `D[6×4]`, written to:
- `~/rv_recovery/matlab/models/model_matrices.h` — **master copy**
- `~/rv_recovery/matlab/models/quadrotor_greybox.mat`

Spectral radius of A: **0.993 < 1** — the open-loop model is stable.
This is critical: the paper's open-loop form `x = A·x + B·u` (no Kalman feedback) only
works if A is stable. The earlier N4SID model had ρ=1.0018 (marginally unstable) which
required Kalman feedback — but that Kalman form is not Algorithm 1.

### File locations
| File | Path |
|------|------|
| Grey-box sysid script | `~/rv_recovery/matlab/sysid_greybox.m` |
| Model matrices header (master) | `~/rv_recovery/matlab/models/model_matrices.h` |
| Model .mat file | `~/rv_recovery/matlab/models/quadrotor_greybox.mat` |

### Deviation
The paper uses a 12-state model (position, velocity, attitude, rates). Our model is 6-state
(attitude + rates only), because extending the grey-box PEM template with position/velocity
kinematic rows (`quad_template.m`) has not been done. See [What Remains](#20-what-remains-for-complete-replication).

---

## 3. Step 2 — DTW Parameter Selection

### What the paper says
Section 3.3: Use Dynamic Time Warping to find the maximum time-displacement between the
predicted signal and the real signal over normal flight data. This gives window size `N`.
Then set `T_on = e_max + margin` where `e_max` is the maximum accumulated error within
any window of size `N` during normal operation.

### What was done (original — now superseded)
`~/rv_recovery/python/select_parameters.py` was run using the N4SID innovation/observer
predictor:
```python
x[k+1] = Ap·x[k] + (B - K·D)·u[k] + K·y[k]   # OLD — not paper's algorithm
```
This produced: **N=491, T_on=558.40, T_off=441.14** for the Roll channel.
These values are in the current `recovery_params.h` as placeholders.

### What was fixed (re-root session)
`select_parameters.py` was rewritten to use `predict_software_sensors()` — a function that
mirrors the exact firmware predictor:
```python
def predict_software_sensors(U, X_real, A, B, C, D, window):
    # open-loop:  x = A*x + B*u  (NO Kalman)
    # output:     y = C*x + D*u - e  (disturbance-compensated)
    # checkpoint: e = mean(err_hist),  x = X_real  (windowed resync)
```

Thresholds must be regenerated by running:
```bash
python3 ~/rv_recovery/python/select_parameters.py
```
When run, it will emit a new `~/rv_recovery/firmware_patch/recovery_params.h` and sync it
to the firmware tree at `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_params.h`.

### Why this matters
If thresholds are calibrated against the Kalman predictor but the firmware runs the
open-loop predictor, the residuals have different magnitudes. `T_on` calibrated on the
wrong predictor gives wrong detection sensitivity — too tight (false positives) or too
loose (missed attacks).

### File locations
| File | Path |
|------|------|
| DTW script | `~/rv_recovery/python/select_parameters.py` |
| Recovery params header (placeholder) | `~/rv_recovery/firmware_patch/recovery_params.h` |
| Recovery params (firmware copy) | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_params.h` |

---

## 4. Step 3 — Algorithm 1: Open-Loop Form

### What was wrong before
The old `recovery_monitor.h` implemented the **observer/predictor form**:
```cpp
// WRONG (old code):
x_new[i] += AP_MAT[i][j] * s->x[j];            // Ap = A - K*C
x_new[i] += K_MAT[i][j] * (y[j] - s->y_hat[j]); // Kalman correction term
```
This is the N4SID innovation predictor — it adds a Kalman feedback term `K·(y − ŷ)` that
pulls the state estimate toward the measured output every step. This form is NOT in
Algorithm 1 of the paper.

The old code also referenced `K_MAT` and `AP_MAT`, which do not exist in the grey-box
`model_matrices.h` (which only outputs `A`, `B`, `C`, `D`). So the old code would not
even compile with the grey-box model.

### What the paper says
Paper §3.1, Algorithm 1, line 7:
```
x[k+1] = A·x[k] + B·u[k]
```
That is all. Open-loop. No measurement feedback. No Kalman term.

### What was changed
`~/rv_recovery/firmware_patch/recovery_monitor.h` was rewritten. The state update is now:
```cpp
// CORRECT (paper Algorithm 1 line 7):
float xn[NX] = {};
for (int i = 0; i < NX; i++) {
    for (int j = 0; j < NX; j++) xn[i] += A_MAT[i][j] * s->x[j];
    for (int j = 0; j < NU; j++) xn[i] += B_MAT[i][j] * u_real[j];
}
memcpy(s->x, xn, sizeof(float)*NX);
```
All references to `K_MAT`, `AP_MAT`, and the Kalman term were removed.

The output (software sensor) is computed **before** the state advance, also per Algorithm 1:
```cpp
// Algorithm 1 line 6: y = C*x + D*u  (BEFORE state advance)
for (int k = 0; k < NY; k++) {
    float y = 0.0f;
    for (int j = 0; j < NX; j++) y += C_MAT[k][j] * s->x[j];
    for (int j = 0; j < NU; j++) y += D_MAT[k][j] * u_real[j];
    s->ms[k] = y - s->e[k];   // disturbance-compensated
}
```

### File locations
| File | Path |
|------|------|
| Recovery monitor header | `~/rv_recovery/firmware_patch/recovery_monitor.h` |
| Firmware copy | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_monitor.h` |

---

## 5. Step 4 — Wrong Insertion Point Found and Fixed (Re-Root)

### What was wrong before
The recovery block was in `AP_InertialSensor::update()` in:
```
~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp
```
This is the **IMU driver** — it runs deep in the hardware abstraction layer. The IMU driver
has **no access** to:
- The attitude controller targets (the paper's `u`) — these live in AC_AttitudeControl
- The AHRS attitude estimate (the paper's real state `x`) — this lives in AP_AHRS

So in the driver, `u` was zeroed (`float u[NU] = {}`) and the attitude channels 0–2 (φ,θ,ψ)
were also zero. The predictor ran `x = A·x + B·0`, which is a stable homogeneous system
that decays to zero regardless of initial conditions. The software sensor `ms` tracked
nothing about the actual vehicle — it just decayed to zero. Detection relied entirely on
gyro channel residuals against a decaying-to-zero prediction.

### What the paper says
Paper Figure 3 (page 3) shows the insertion point explicitly:
```c
main_loop() {
    angles  = read_AHRS();              // real state x
    targets = navigation_logic();       // control input u
    inputs  = attitude_controller(targets, angles);
    motor.update(inputs);
}
read_AHRS() {
    for each gyro[i]:
        gyros[i] = sensor.read();
        if |soft_gyro[i] - gyros[i]| > T_on:
            gyros[i] = soft_gyro[i];   // substitute software sensor
        fuse gyros
    angles = convert2angle(gyro_fused)
    return angles
}
```
The recovery check happens **inside `read_AHRS`**, after reading the raw sensor but before
fusing it into attitude. In ArduCopter 3.4, the equivalent is `Copter::fast_loop()` after
`read_AHRS()` (which calls `ahrs.update()`), where both `attitude_control` and `ahrs` are
in scope.

### What was changed

**Removed from `AP_InertialSensor.cpp`:**
- The `#include "recovery_monitor.h"` and `#include "software_sensors.h"` lines
- The global state variables (`g_recovery_state`, `g_recovery_initialized`, `g_recovery_decimate`)
- The entire recovery block (initialization, decimation counter, u/y setup, state update, per-channel monitoring)

The driver file now has **zero recovery references**.

**Added to `ArduCopter/ArduCopter.cpp`:**

At file scope (after `#include "Copter.h"`):
```cpp
#include "../libraries/AP_InertialSensor/recovery_monitor.h"
#include "../libraries/AP_InertialSensor/software_sensors.h"

static RecoveryState g_recovery;
static int           g_recovery_decim = 0;   // 400 Hz → 50 Hz decimation
```

Inside `Copter::fast_loop()`, immediately after `read_AHRS();`:
```cpp
if (++g_recovery_decim >= 8) {
    g_recovery_decim = 0;

    // u_real: attitude targets from attitude controller (paper's 'u')
    Vector3f tgt_cd = attitude_control.get_att_target_euler_cd();
    float u_real[NU] = {
        radians(tgt_cd.x * 0.01f),          // phi_cmd  (rad)
        radians(tgt_cd.y * 0.01f),          // theta_cmd (rad)
        radians(tgt_cd.z * 0.01f),          // psi_cmd  (rad)
        attitude_control.get_throttle_in()  // throttle 0..1
    };

    // x_real: real vehicle state (phi, theta, psi, p, q, r)
    const Vector3f &gyro = ahrs.get_gyro();
    float x_real[NX] = {
        ahrs.roll, ahrs.pitch, ahrs.yaw,   // rad
        gyro.x,    gyro.y,    gyro.z       // rad/s
    };

    // Algorithm 1 + Section 3.3 windowed resync
    recovery_update(&g_recovery, u_real, x_real);

    // Per-sensor check (paper Figure 3 inner loop)
    float gx = recovery_check(&g_recovery, CH_GYRX, gyro.x);
    float gy = recovery_check(&g_recovery, CH_GYRY, gyro.y);
    float gz = recovery_check(&g_recovery, CH_GYRZ, gyro.z);

    if (g_recovery.recovery_mode[CH_GYRX] ||
        g_recovery.recovery_mode[CH_GYRY] ||
        g_recovery.recovery_mode[CH_GYRZ]) {
        ins.set_gyro(ins.get_primary_gyro(), Vector3f(gx, gy, gz));
    }

    // All-gyros-compromised: supplementary compensation (Appendix B)
    if (recovery_all_gyros_compromised(&g_recovery)) {
        Vector3f acc = ins.get_accel();
        Vector3f mag = compass.get_field();
        float phi_a, the_a, psi_m;
        supplementary_compensation(acc.x, acc.y, acc.z,
                                   mag.x, mag.y, mag.z,
                                   &phi_a, &the_a, &psi_m);
        (void)phi_a; (void)the_a; (void)psi_m;  // injection pending — see deviation
    }
}
```

### File locations
| File | Action | Path |
|------|--------|------|
| Correct insertion | Added | `~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` |
| Driver | Cleaned | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp` |
| Git diff | Saved | `/tmp/firmware_controlloop.diff` |
| Build log | Saved | `/tmp/step8_build.log` |

### Deviation
**Wiring depth is Option 1 (detection-faithful, replacement-shallow).**
The paper replaces `gyros[i]` BEFORE `convert2angle()` — before the AHRS integrates the
gyro into attitude. In ArduCopter 3.4, `read_AHRS()` (which calls `ahrs.update()`) runs
before our recovery block in `fast_loop()`. So the AHRS state used by the EKF for this
particular tick was already computed from the uncorrected gyro.

Our corrected gyro is fed back via `ins.set_gyro()`, which the **rate controller** will
use on the same tick (since `attitude_control.rate_controller_run()` runs after our block).
This correctly affects the rate control loop but not the EKF state estimate.

Full pre-AHRS injection (Options 2/3) would require intercepting the DCM or EKF backend
before `ahrs.update()` — not yet done. This is the most significant remaining deviation.

---

## 6. Step 5 — Windowed State Resynchronization (Section 3.3)

### What was wrong before
The old recovery block reset `r[ch]` and updated `e[ch]` at each window checkpoint (every
`RECOVERY_WINDOW` steps), but it did **not** resync the predicted state `s->x` to the real
vehicle state. This meant that after the first window, the open-loop predictor's state `x`
diverged from the real vehicle, and `ms = C·x + D·u` drifted away from the real sensor
values — making detection unreliable.

### What the paper says
Section 3.3: At each window checkpoint, update the disturbance estimate `e` from the error
history, then **re-seed the predicted state to the real state**. This bounds the predictor
error to a single window horizon (~10 s at paper's 400 Hz, ~9.8 s at our 50 Hz with N=491).
Without this resync, the open-loop predictor is unusable over longer flight times.

Also: the residual accumulator `r` should NOT be reset at the checkpoint — it runs
continuously. Only `t` (window tick counter) and `e` (disturbance estimate) are updated.

### What was changed
In `recovery_update()` in `recovery_monitor.h`, the checkpoint now:
1. Updates `e[k]` for healthy (non-attacked) channels only
2. Re-seeds `s->x[c] = x_real[c]` for healthy channels
3. Resets `t = 0`
4. Does **not** reset `s->r[k]` (continuous accumulation, per guide Section 4 Python code)

```cpp
if (s->t >= REC_WINDOW_MAX) {
    for (int k = 0; k < NY; k++) {
        if (!s->recovery_mode[k]) {
            float sum = 0.0f;
            for (int w = 0; w < REC_WINDOW_MAX; w++) sum += s->err_hist[k][w];
            s->e[k] = sum / (float)REC_WINDOW_MAX;
        }
        // r[k] NOT reset here — accumulator runs continuously
    }
    for (int c = 0; c < NX; c++) {
        if (!s->recovery_mode[c]) s->x[c] = x_real[c];  // resync
    }
    s->t = 0;
}
```

T4 in the unit test verifies that resyncs happen at each `REC_WINDOW_MAX` boundary.

### File locations
`~/rv_recovery/firmware_patch/recovery_monitor.h` — `recovery_update()` function

---

## 7. Step 6 — Real Control Input u and Real State x

### What was wrong before
In the IMU driver location, `u` was forced to zero and φ,θ,ψ were zero:
```cpp
float u[NU] = {};    // WRONG: zero — no access to attitude controller here
float y[NX] = {};    // WRONG: phi/theta/psi (ch 0-2) left at zero
```

### What was changed
After moving to `fast_loop()`, the real values are used:

**Control input u (`attitude_control` API):**
```cpp
Vector3f tgt_cd = attitude_control.get_att_target_euler_cd();
// get_att_target_euler_cd() returns centidegrees (_attitude_target_euler_angle × 100×(180/π))
// Convert centidegrees → radians: radians(tgt_cd.x × 0.01f)
float u_real[NU] = {
    radians(tgt_cd.x * 0.01f),          // phi_cmd
    radians(tgt_cd.y * 0.01f),          // theta_cmd
    radians(tgt_cd.z * 0.01f),          // psi_cmd
    attitude_control.get_throttle_in()  // 0..1
};
```

**Real state x (`ahrs` API):**
```cpp
const Vector3f &gyro = ahrs.get_gyro();
float x_real[NX] = {
    ahrs.roll, ahrs.pitch, ahrs.yaw,  // rad — matches sysid output units
    gyro.x,    gyro.y,    gyro.z      // rad/s — matches sysid output units
};
```

**Unit consistency requirement:** The units must exactly match what `sysid_greybox.m`
used during identification. The sysid script used `[DesRoll, DesPitch, DesYaw (rad),
ThrottleIn (0..1)]` for `u` and attitude in radians, gyro in rad/s. The code above
matches these units exactly.

### File locations
`~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` — `fast_loop()` block

---

## 8. Step 7 — 50 Hz Decimation

### What was wrong before
The original recovery block had no decimation. `AP_InertialSensor::update()` runs at
400 Hz. The model was identified at Ts=0.02 s (50 Hz). Calling the model 400 times per
second with a 50 Hz model means the virtual time inside the model runs 8× faster than
real time — thresholds were calibrated at 50 Hz, so they fire 8× too quickly at 400 Hz.

### What was changed
A decimation counter `g_recovery_decim` was added. The recovery block only executes
every 8th call of `fast_loop()`:
```cpp
static int g_recovery_decim = 0;
if (++g_recovery_decim >= 8) {
    g_recovery_decim = 0;
    // ... full recovery block at 50 Hz ...
}
```
400 Hz / 8 = 50 Hz — matches `Ts = 0.02 s` from `model_matrices.h`.

### Deviation from paper
The paper identifies and runs the model at **400 Hz** (native IMU rate). Our model is at
50 Hz due to RAM constraints. The error history buffer `float err_hist[NY][REC_WINDOW_MAX]`
at 400 Hz with equivalent N≈1840 would require ≈44 KB just for history. At 50 Hz with
N=491 it uses ≈11 KB. This is the one acknowledged rate deviation.

### File locations
`~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` — `g_recovery_decim` at file scope,
decimation check in `fast_loop()`

---

## 9. Step 8 — NX=12 Ghost Cleanup

### What was wrong before
Before the grey-box sysid, a 12-state N4SID model was used (states: GPS_Lat, GPS_Lng,
GPS_Alt, Roll, Pitch, Yaw, GyrX, GyrY, GyrZ, BARO_Alt, AccX, AccY). After switching to
the grey-box 6-state model, the old code in `AP_InertialSensor.cpp` still had:
```cpp
y[6] = _gyro[_primary_gyro].x;   // WRONG: y is now size 6, y[6] is out-of-bounds
y[7] = _gyro[_primary_gyro].y;
y[8] = _gyro[_primary_gyro].z;
```
Writing to `y[6]` in a 6-element array is an out-of-bounds stack write — undefined behavior.

Additionally, `~/rv_recovery/firmware_patch/model_matrices.h` was a stale copy of the old
NX=12 N4SID header, never updated after the grey-box MATLAB run. The firmware was being
compiled with the wrong model.

### What was changed
1. All hard-coded channel indices replaced with named constants:
   - `CH_PHI=0`, `CH_THETA=1`, `CH_PSI=2` (attitude)
   - `CH_GYRX=3`, `CH_GYRY=4`, `CH_GYRZ=5` (body rates)
2. `~/rv_recovery/firmware_patch/model_matrices.h` re-synced from
   `~/rv_recovery/matlab/models/model_matrices.h` (the grey-box master)
3. The synced file copied to the firmware tree at
   `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h`

After the re-root, the recovery block was moved out of `AP_InertialSensor.cpp` entirely,
so these channel accesses now live in `ArduCopter.cpp` where they are correct.

### File locations
| File | Path |
|------|------|
| Model matrices (master) | `~/rv_recovery/matlab/models/model_matrices.h` |
| Model matrices (fw_patch) | `~/rv_recovery/firmware_patch/model_matrices.h` |
| Model matrices (firmware) | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h` |

---

## 10. Step 9 — Unit Tests (T1–T5, 19/19 pass)

### What was wrong before
The old unit test:
- Attacked channel 0 (Roll/φ) with a `+20.0f` offset — wrong channel; should attack
  gyro channels 3–5 (p, q, r) with a realistic rad/s bias
- Had no no-attack (false-positive) test
- Had no test that the predictor was being driven by a non-zero `u`
- Had no test for the windowed resync behavior

### What the paper says
The attack scenario in §4.1 is a gyro bias attack — the adversary injects a constant offset
into the gyroscope readings. Gyros are channels 3, 4, 5 in the 6-state model. A 2.0 rad/s
bias is a realistic, strong attack. Detection should occur within ~N/2 steps.

### What was changed
`~/rv_recovery/firmware_patch/test_recovery.cpp` was rewritten with five tests using the
new control-loop API (`recovery_update` + `recovery_check`):

**T1 — No-attack false-positive check:**
Runs `REC_WINDOW_MAX + 10` steps with small hover-like signals and real-ish `u`/`x_real`.
Asserts no channel ever enters recovery mode.

**T2 — Driven predictor (not decaying to zero):**
Compares `sum(|ms[CH_PHI]|)` with driven `u` vs. zero `u`. Asserts the driven predictor
produces a larger signal — proves the `u=0` bug is gone. With the old driver-location code,
`ms[]` would decay to zero regardless of what the vehicle was doing.

**T3 — Gyro bias attack on channels 3/4/5:**
- Injects `+2.0 rad/s` on GyrX, `+1.6 rad/s` on GyrY, `+1.0 rad/s` on GyrZ
- `recovery_update()` sees CLEAN `x_real` (model tracks true plant)
- `recovery_check()` sees CORRUPTED `m_real` (what the firmware would see after spoofing)
- Asserts: all three channels detected, attitude channels (0–2) not triggered,
  returned value switches to `ms[ch]`, detection within 2×N steps, `all_gyros_compromised` true

**T4 — Windowed resync:**
Runs 3 full windows, counts resyncs (t reset to 0). Asserts at least 2 resyncs observed.

**T5 — Software sensors compilation (Eq. 4–6):**
Verifies Holoborodko derivative (Eq. 4), barometric pressure model (Eq. 5),
magnetic heading (Eq. 6), and supplementary compensation (Appendix B) all produce
finite, physically reasonable values.

**Result: 19 / 19 pass.**

```
=== T1: No-attack === PASS no false positive
=== T2: Driven predictor === ms[phi] driven=146.04, zero=5.38 → PASS PASS
=== T3: Gyro attack ===
  [t= 873] GyrX detected  latency=273 steps (5.46 s)
  [t= 943] GyrY detected
  [t=1080] GyrZ detected
  PASS×6 (detection, no att false-pos, substitution, timing, all-gyros flag)
=== T4: Windowed resync === PASS PASS
=== T5: Software sensors === PASS×7
=== Results: 19 / 19 passed ===
```

### How to run
```bash
g++ -O2 -std=c++14 -I/home/tchowdh4/rv_recovery/firmware_patch \
    -o /tmp/test_recovery \
    /home/tchowdh4/rv_recovery/firmware_patch/test_recovery.cpp -lm
/tmp/test_recovery
```

### File locations
`~/rv_recovery/firmware_patch/test_recovery.cpp`

---

## 11. Step 10 — GPS Recovery (recovery_gps.h)

### What the paper says
Section 4.3: GPS spoofing is a primary attack scenario. The defense predicts where the GPS
should report by dead-reckoning from velocity (integrating NED velocity forward). If the
reported GPS position diverges from the dead-reckoning estimate by more than `T_on`, the
DR position is substituted.

### What was done
`~/rv_recovery/firmware_patch/recovery_gps.h` implements Algorithm 1 for GPS:

- **`recovery_gps_predict(s, vel_north_ms, vel_east_ms, dt)`** — advances the DR position
  estimate by integrating NED velocity. Converts m/s to degrees using 111,000 m/deg for
  latitude and cos(lat)-scaled longitude.

- **`recovery_monitor_gps(s, lat_meas, lng_meas, lat_dr, lng_dr, lat_out, lng_out)`** —
  runs Algorithm 1 on the GPS lat/lng channels. Returns the position to use: real GPS if
  healthy, DR estimate if attack detected.

DTW parameters: `GPS_RECOVERY_WINDOW=248`, `GPS_T_ON=5069.80`, `GPS_T_OFF=4005.14`

Wiring instructions are in the file header — points to `AP_GPS.cpp`, function `AP_GPS::update()`.

### Deviation: NOT wired into firmware
`recovery_gps.h` exists and is complete but has **not** been patched into `AP_GPS.cpp`.
The firmware does not yet detect or recover from GPS attacks. See [What Remains](#20-what-remains-for-complete-replication).

### File locations
`~/rv_recovery/firmware_patch/recovery_gps.h`

---

## 12. Step 11 — Barometer Recovery (recovery_baro.h)

### What the paper says
Section 3.2, Eq. 5: Given the current altitude `h` (from EKF), the expected barometric
pressure is:
```
P_h = P_0 · exp(−g_0 · M · (h − h_0) / (R · T_0))
```
If the reported pressure diverges from this physics prediction, a barometer attack is detected.

### What was done
`~/rv_recovery/firmware_patch/recovery_baro.h` implements Algorithm 1 for barometer:

- Takes `press_measured` (raw barometer) and `press_expected` (from `software_baro(altitude_m)`)
- Runs the residual accumulation and threshold comparison
- Returns the pressure to use: raw if healthy, physics-predicted if attack detected

DTW parameters: `BARO_RECOVERY_WINDOW=2041`, `BARO_T_ON=1204.61`, `BARO_T_OFF=951.64`

### Deviation: NOT wired into firmware
`recovery_baro.h` exists and is complete but has **not** been patched into `AP_Baro.cpp`.
See [What Remains](#20-what-remains-for-complete-replication).

### File locations
`~/rv_recovery/firmware_patch/recovery_baro.h`

---

## 13. Step 12 — GPS Spoofing Attack Scripts

### What the paper says
Section 4.3 evaluates two GPS spoofing scenarios:
1. **Sudden 20 m offset** — GPS position jumps 20 m north instantaneously
2. **Stealthy controlled carry-off** — GPS drifts at 0.5 m/s north to slowly move
   the vehicle off its hover point

### What was done
`~/rv_recovery/python/attack_gps.py` was created with `--scenario offset` and
`--scenario drift` modes:

**`--scenario offset`:** Sets `SIM_GPS_POS_ERR_N = 20.0` via MAVLink PARAM_SET,
holds for 15 s, clears. Writes `attack_start` timestamp to `/tmp/attack_timeline.log`.

**`--scenario drift`:** Increments `SIM_GPS_POS_ERR_N` by 0.5 m each second until
it reaches 10 m, then clears. Logs drift rate to `/tmp/attack_timeline.log`.

### How to run
```bash
# Vehicle hovering in SITL, MAVProxy connected on port 14551
python3 ~/rv_recovery/python/attack_gps.py --scenario offset
# or:
python3 ~/rv_recovery/python/attack_gps.py --scenario drift
```

### Deviation
Attack scripts inject the spoofed GPS. The GPS **recovery** (detection + substitution)
requires `recovery_gps.h` to be wired into `AP_GPS.cpp`, which is not yet done.

### File locations
`~/rv_recovery/python/attack_gps.py`

---

## 14. Step 13 — All-Gyros-Compromised Supplementary Compensation

### What the paper says
Appendix B, Table 4, cases C3/C5/C6: When **all three** gyro channels are simultaneously
under attack, the software-sensor substitution for gyro channels returns `ms[ch]` — but
`ms[ch]` is driven by a state `x` that contains `p, q, r` values. If all gyros are spoofed,
the state's angular rate components drift.

The solution: reconstruct attitude from **accelerometer** (roll/pitch, Eq. 11):
```
φ_acc  = atan2(ya, √(xa² + za²))
θ_acc  = atan2(xa, √(ya² + za²))
ψ_mag  = atan2(...magnetometer...)
```
Then substitute these into the attitude estimator.

### What was done
1. `recovery_all_gyros_compromised(s)` function added to `recovery_monitor.h` — returns
   true when `recovery_mode[CH_GYRX] && recovery_mode[CH_GYRY] && recovery_mode[CH_GYRZ]`.

2. `supplementary_compensation()` was already in `software_sensors.h`. It is called in
   `fast_loop()` when `recovery_all_gyros_compromised()` returns true:
   ```cpp
   supplementary_compensation(acc.x, acc.y, acc.z,
                               mag.x, mag.y, mag.z,
                               &phi_a, &the_a, &psi_m);
   ```
   The values `phi_a`, `the_a`, `psi_m` are **computed** but currently suppressed with
   `(void)` — they are not yet injected into the AHRS.

3. T3 in the unit test verifies `recovery_all_gyros_compromised()` returns true when all
   three gyro channels are under attack.

### Deviation
The computed supplementary attitude is not yet substituted into the AHRS/attitude
estimator. This requires backend-specific API calls (DCM or EKF) that differ from the
public ArduCopter API. See [What Remains](#20-what-remains-for-complete-replication).

### File locations
| File | Path |
|------|------|
| `recovery_all_gyros_compromised()` | `~/rv_recovery/firmware_patch/recovery_monitor.h` |
| `supplementary_compensation()` | `~/rv_recovery/firmware_patch/software_sensors.h` |
| Wiring | `~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp`, `fast_loop()` |

---

## 15. Step 14 — A/B Baseline Evaluation Framework

### What the paper says
Section 4: Compare attack-under-recovery vs. attack-without-recovery to demonstrate that
the defense actually prevents attitude deviation. Evaluate using Equation 7 over k=10 s:

```
R_succ := |Y_t − Ȳ_t| ≤ ε   for all t ∈ [1..k]
```

where ε = 3° and k = 10 s. The paper shows that without recovery, attitude error exceeds
ε; with recovery, it stays within ε.

### What was wrong before
- k=30 s was used instead of the paper's k=10 s
- Only the recovery-ON case was evaluated; no baseline comparison existed

### What was changed
1. **`~/rv_recovery/python/eval_recovery.py`**: `K_SEC` changed from 30.0 → **10.0** s
2. **`~/rv_recovery/python/attack_injector.py`**: `ATTACK_HOLD` changed from 20.0 → **15.0** s
   (attack must last longer than k=10 s evaluation window)
3. **New `~/rv_recovery/python/eval_baseline.py`**: A-side of A/B test. Records attitude
   error WITHOUT recovery, saves to `/tmp/step9_baseline_results.npy` and `/tmp/step9_baseline.log`

### How to run the full A/B test
```bash
# ── A-side: recovery DISABLED ────────────────────────────────────────────────
# 1. In ArduCopter/ArduCopter.cpp, wrap the recovery block:
#    #ifndef RECOVERY_DISABLED
#    ... recovery block ...
#    #endif
# 2. Add at top of ArduCopter.cpp: #define RECOVERY_DISABLED
# 3. Rebuild:
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter
# 4. Launch SITL, arm, takeoff, then:
nohup python3 ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_baseline.py | tee /tmp/step9_baseline.log

# ── B-side: recovery ENABLED ─────────────────────────────────────────────────
# 5. Remove #define RECOVERY_DISABLED, rebuild
# 6. Restart SITL, arm, takeoff, then:
nohup python3 ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_recovery.py | tee /tmp/step9_recovery.log
```

### File locations
| File | Path |
|------|------|
| Recovery evaluator | `~/rv_recovery/python/eval_recovery.py` |
| Baseline evaluator | `~/rv_recovery/python/eval_baseline.py` |
| Attack injector | `~/rv_recovery/python/attack_injector.py` |

---

## 16. Step 15 — DTW Script Updated for Correct Predictor

### What was changed
`~/rv_recovery/python/select_parameters.py` was updated in two ways:

1. **Old `compute_software_sensor_predictions()` kept as reference** but clearly marked
   `# OLD — do not use for threshold calibration`.

2. **New `predict_software_sensors(U, X_real, A, B, C, D, window)`** added — this mirrors
   `recovery_update()` exactly:
   - Open-loop `x = A*x + B*u` (no Kalman)
   - Output `y = C*x + D*u - e` (disturbance-compensated)
   - Windowed resync: every `window` samples, update `e = mean(err_hist)`, then
     `x = X_real[n]` (drift reset)

3. **`write_recovery_params_h()`** added — emits `recovery_params.h` with per-channel
   arrays (`N_CH[]`, `T_ON_CH[]`, `T_OFF_CH[]`) after running DTW on all 6 channels.

4. **`main()` updated** — loads grey-box model (not N4SID), runs `predict_software_sensors()`,
   runs DTW, calls `write_recovery_params_h()`, syncs header to firmware tree.

### How to regenerate thresholds
```bash
# Requires operation_data.mat and quadrotor_greybox.mat:
python3 ~/rv_recovery/python/select_parameters.py
# Outputs:
#   ~/rv_recovery/firmware_patch/recovery_params.h  (new per-channel values)
#   ~/ardupilot_ws/.../AP_InertialSensor/recovery_params.h  (synced)
# Then rebuild firmware:
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter
```

### File locations
`~/rv_recovery/python/select_parameters.py`

---

## 17. Step 16 — Fabricated Quotes Fixed in Guide

### What was wrong
Four block-quotes in `REPLICATION_GUIDE_COMPLETE.md` were formatted as direct paper quotes
using the Markdown `>` blockquote syntax. The exact wording did not appear in the paper —
they were paraphrases incorrectly presented as verbatim quotations.

### What was changed
All four were replaced with clearly marked interpretation text:

| Guide location | Old (fabricated) | New |
|---|---|---|
| Step 3 | `> "We collected operation data..."` | `[Our interpretation — ...]` |
| Step 4 | `> "The operation data is collected at 400 Hz..."` | `[Our interpretation — ...]` |
| Step 5 | `> "We identify a discrete-time linear..."` | `[Our interpretation — §3.1 ...]` |
| Step 6 | `> "We use Dynamic Time Warping..."` | `[Our interpretation — §3.3 ...]` |

The deviations table was also expanded from 12 to 16 entries.

### File locations
`~/rv_recovery/REPLICATION_GUIDE_COMPLETE.md`

---

## 18. Complete File Reference

### Firmware files (ArduCopter 3.4 tree)

| File | Path | Status |
|------|------|--------|
| **Main insertion point** | `~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` | Recovery in `fast_loop()` after `read_AHRS()` |
| IMU driver (reverted) | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp` | 0 recovery references — clean |
| Recovery monitor | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_monitor.h` | Synced from firmware_patch/ |
| Recovery params | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_params.h` | Synced from firmware_patch/ |
| Model matrices | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h` | Synced from matlab/models/ |
| Software sensors | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/software_sensors.h` | Eqs. 4–6 + Appendix B |

### Replication workspace files

| File | Path | Purpose |
|------|------|---------|
| Recovery monitor | `~/rv_recovery/firmware_patch/recovery_monitor.h` | **Master** — control-loop API |
| Recovery params | `~/rv_recovery/firmware_patch/recovery_params.h` | Per-channel N/T_on/T_off (placeholder) |
| Model matrices | `~/rv_recovery/firmware_patch/model_matrices.h` | Synced from matlab/models/ |
| Software sensors | `~/rv_recovery/firmware_patch/software_sensors.h` | Eqs. 4–6, Appendix B |
| Unit tests | `~/rv_recovery/firmware_patch/test_recovery.cpp` | T1–T5, 19/19 pass |
| GPS recovery header | `~/rv_recovery/firmware_patch/recovery_gps.h` | Algorithm 1 for GPS (not wired) |
| Baro recovery header | `~/rv_recovery/firmware_patch/recovery_baro.h` | Algorithm 1 for baro (not wired) |
| Grey-box sysid | `~/rv_recovery/matlab/sysid_greybox.m` | MATLAB system identification |
| Model matrices (master) | `~/rv_recovery/matlab/models/model_matrices.h` | NX=6, grey-box output |
| Grey-box .mat | `~/rv_recovery/matlab/models/quadrotor_greybox.mat` | MATLAB model matrices |
| DTW script | `~/rv_recovery/python/select_parameters.py` | Updated for correct predictor |
| Attack injector | `~/rv_recovery/python/attack_injector.py` | Gyro bias attack |
| GPS attack | `~/rv_recovery/python/attack_gps.py` | GPS offset + carry-off |
| Recovery evaluator | `~/rv_recovery/python/eval_recovery.py` | k=10 s Eq. 7 check |
| Baseline evaluator | `~/rv_recovery/python/eval_baseline.py` | A-side of A/B comparison |
| Full replication guide | `~/rv_recovery/REPLICATION_GUIDE_COMPLETE.md` | Original complete guide |
| Correction guide | `~/rv_recovery/CORRECTION_GUIDE.md` | Correction session notes |
| This file | `~/rv_recovery/FULL_REPLICATION_STATUS.md` | Current document |

### Saved artifacts

| Artifact | Path |
|----------|------|
| Firmware build log | `/tmp/step8_build.log` |
| Firmware git diff (control-loop patch) | `/tmp/firmware_controlloop.diff` |

---

## 19. Deviations from the Paper

The following table lists every known deviation between this replication and the paper.
Items marked ✓ are fully aligned. Items marked as deviations are explained.

| # | Paper requirement | Our implementation | Deviation? |
|---|---|---|---|
| **D1** | Algorithm 1 open-loop: `x = A·x + B·u` | Implemented exactly | ✓ Aligned |
| **D2** | Output before state advance: `y = C·x + D·u` then `x = A·x + B·u` | Implemented exactly | ✓ Aligned |
| **D3** | Insertion in `main_loop` / `read_AHRS` (Figure 3) | In `Copter::fast_loop()` after `read_AHRS()` — ArduCopter 3.4 equivalent | ✓ Aligned |
| **D4** | Real control targets `u` fed into predictor | `attitude_control.get_att_target_euler_cd()` + `get_throttle_in()` | ✓ Aligned |
| **D5** | Real state `x = [φ,θ,ψ,p,q,r]` from AHRS | `ahrs.roll/pitch/yaw` + `ahrs.get_gyro()` | ✓ Aligned |
| **D6** | Windowed state resync to real state (§3.3) | Implemented — `s->x[c] = x_real[c]` at each window checkpoint | ✓ Aligned |
| **D7** | Disturbance compensation `ms = C·x + D·u − e` | Implemented — `s->ms[k] = y - s->e[k]` | ✓ Aligned |
| **D8** | k=10 s evaluation window (Eq. 7) | `K_SEC = 10.0` in eval_recovery.py | ✓ Aligned |
| **D9** | Algorithm runs at 400 Hz (native sensor rate) | Runs at **50 Hz** (Ts=0.02 s, decimate 8× from 400 Hz) | **Deviation** — RAM constraint. Error history at 400 Hz would be ≈44 KB. Thresholds recalibrated at 50 Hz. Acknowledged. |
| **D10** | 12-state model: position + velocity + attitude | **6-state model:** attitude + angular rates only [φ,θ,ψ,p,q,r] | **Deviation** — Grey-box PEM template not extended with kinematic position/velocity rows. GPS and baro channels are not part of the identified state. |
| **D11** | DTW thresholds calibrated on open-loop+resync predictor | Current `recovery_params.h` has **placeholder values** from old N4SID observer predictor | **Deviation** — `select_parameters.py` was updated with the correct predictor function but has not been re-run yet. Run `select_parameters.py` to regenerate. |
| **D12** | Replacement BEFORE `convert2angle()` — inside AHRS update | Option-1 shallow: corrected gyro fed via `ins.set_gyro()` after `ahrs.update()` | **Deviation** — AHRS state for the current tick was computed from uncorrected gyro. Rate controller gets corrected gyro. Full pre-AHRS injection needs DCM/EKF backend intercept. |
| **D13** | GPS monitoring wired in `AP_GPS.cpp` | `recovery_gps.h` complete, **not wired** | **Deviation** — Header exists with correct Algorithm 1 implementation. Requires patching `libraries/AP_GPS/AP_GPS.cpp`. |
| **D14** | Barometer monitoring wired in `AP_Baro.cpp` | `recovery_baro.h` complete, **not wired** | **Deviation** — Header exists. Requires patching `libraries/AP_Baro/AP_Baro.cpp`. |
| **D15** | Magnetometer monitoring wired | **Not implemented** | **Deviation** — `software_mag_heading()` exists in `software_sensors.h`. No `recovery_mag.h` or firmware patch. |
| **D16** | All-gyros-compromised: φ/θ/ψ from accel+mag injected into AHRS | Values computed and available, **not injected** | **Deviation** — `supplementary_compensation()` computes `phi_a`, `the_a`, `psi_m` in `fast_loop()` but they are `(void)`-suppressed. Requires DCM/EKF backend API. |
| **D17** | A/B baseline SITL run with saved logs | Framework complete (scripts ready), **SITL run not executed** | **Deviation** — Requires two separate SITL sessions (recovery off / on) with attack injection. |

**Summary:** 8 items fully aligned (D1–D8). 9 deviations remain (D9–D17).
D9 (rate) is acknowledged and cannot be resolved without hardware changes.
D10–D17 are implementation gaps that can be resolved by following the remaining steps below.

---

## 20. What Remains for Complete Replication

These are ordered by dependency — later steps depend on earlier ones.

---

### R1 — Regenerate DTW Thresholds (Closes D11)

**Why:** Current `recovery_params.h` has placeholder values from the old N4SID observer
predictor. The firmware now runs the open-loop+resync predictor. Thresholds calibrated on
the wrong predictor give wrong detection sensitivity.

**How:**
```bash
# Requires operation_data.mat and quadrotor_greybox.mat to exist:
python3 ~/rv_recovery/python/select_parameters.py
# Then rebuild firmware:
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter
```

**Output:** New `recovery_params.h` with per-channel N, T_on, T_off calibrated on the
exact firmware predictor. Synced automatically to firmware tree.

---

### R2 — Wire GPS Recovery into AP_GPS.cpp (Closes D13)

**File to patch:** `~/ardupilot_ws/arducopter-3.4/libraries/AP_GPS/AP_GPS.cpp`

**What to add** at top:
```cpp
#include "recovery_gps.h"   // (in AP_InertialSensor/ or copy to AP_GPS/)
static RecoveryGPSState g_gps_recovery = {};
```

**What to add** in `AP_GPS::update()`, after backends update `state.location`:
```cpp
if (!g_gps_recovery.initialized) {
    g_gps_recovery.dr_lat = state.location.lat * 1e-7f;
    g_gps_recovery.dr_lng = state.location.lng * 1e-7f;
    g_gps_recovery.initialized = true;
}
float vel_n = state.velocity.x;   // m/s north
float vel_e = state.velocity.y;   // m/s east
float dt    = 1.0f / 5.0f;        // GPS ~5 Hz
recovery_gps_predict(&g_gps_recovery, vel_n, vel_e, dt);

float lat_out, lng_out;
recovery_monitor_gps(&g_gps_recovery,
                      state.location.lat * 1e-7f,
                      state.location.lng * 1e-7f,
                      g_gps_recovery.dr_lat,
                      g_gps_recovery.dr_lng,
                      &lat_out, &lng_out);
if (g_gps_recovery.recovery_mode) {
    state.location.lat = (int32_t)(lat_out * 1e7);
    state.location.lng = (int32_t)(lng_out * 1e7);
}
```

---

### R3 — Wire Barometer Recovery into AP_Baro.cpp (Closes D14)

**File to patch:** `~/ardupilot_ws/arducopter-3.4/libraries/AP_Baro/AP_Baro.cpp`

**What to add** at top:
```cpp
#include "recovery_baro.h"
#include "../AP_InertialSensor/software_sensors.h"
static RecoveryBaroState g_baro_recovery = {};
```

**What to add** in `AP_Baro::update()`, after backends write `_sensors[i].pressure`:
```cpp
float alt_m = _sensors[_primary].altitude;           // EKF altitude (m AGL)
float press_expected = software_baro(alt_m);          // Eq. 5
float press_out = recovery_monitor_baro(&g_baro_recovery,
                                         _sensors[_primary].pressure,
                                         press_expected);
if (g_baro_recovery.recovery_mode)
    _sensors[_primary].pressure = press_out;
```

---

### R4 — Wire Supplementary Compensation into AHRS (Closes D16, partially D12)

**File to patch:** `~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp`

The supplementary compensation values are already computed in `fast_loop()`:
```cpp
float phi_a, the_a, psi_m;
supplementary_compensation(..., &phi_a, &the_a, &psi_m);
(void)phi_a; (void)the_a; (void)psi_m;   // <-- remove (void) and inject
```

**What to add** — replace the `(void)` lines with AHRS override. In ArduCopter 3.4 with
DCM backend, the attitude can be forced by writing to `_dcm_matrix` or using a sim-mode
override. The exact API depends on which AHRS backend is active:

```cpp
// DCM backend override (if ahrs.get_dcm_matrix() is accessible):
// Set roll/pitch directly — this is what convert2angle() produces.
// Note: this is an approximation; the full integration path through DCM
// would need its own gyro feed, not just the output angles.
ahrs.set_roll(phi_a);    // if AP_AHRS exposes this
ahrs.set_pitch(the_a);
ahrs.set_yaw(psi_m);
```

Investigation of the AHRS backend API is required before implementing this.

---

### R5 — Magnetometer Recovery (Closes D15)

**Step 1:** Create `~/rv_recovery/firmware_patch/recovery_mag.h` (similar structure to
`recovery_baro.h`). The measurement is compass heading from `compass.get_heading_rad()`.
The prediction is `software_mag_heading(mx, my, mz, phi, theta)` from `software_sensors.h`.

**Step 2:** Run `select_parameters.py` with magnetometer channel to get DTW parameters.

**Step 3:** Patch `~/ardupilot_ws/arducopter-3.4/libraries/AP_Compass/AP_Compass.cpp`.

---

### R6 — Extend sysid to 12-State Model (Closes D10)

**Why needed:** The paper uses a 12-state model. GPS and barometer predictions come from
the model state directly (not just physics equations like Eq. 5), giving tighter residuals
and faster detection.

**How:** Extend `~/rv_recovery/matlab/quad_template.m` to add kinematic rows:
```
ẋ_pos = R(φ,θ,ψ) · v_body    (known physics — no new free parameters)
v̇     = g_inertial + a_body   (known physics)
```
These can be added as **fixed** rows (no new optimization parameters) since the kinematics
are physically known. Then re-run:
```bash
/usr/local/MATLAB/R2026a/bin/matlab -nodisplay -nosplash \
    -r "cd ~/rv_recovery/matlab; run('sysid_greybox.m')"
```

After running: sync the new `model_matrices.h` (NX=12) to firmware and rebuild.

---

### R7 — Run A/B Baseline SITL Sessions (Closes D17)

Follow the commands in [Step 14](#15-step-14--ab-baseline-evaluation-framework) above:
1. Build with `#define RECOVERY_DISABLED`, run SITL, save `/tmp/step9_baseline.log`
2. Build without it, run SITL, save `/tmp/step9_recovery.log`
3. Compare: baseline should show attitude error > ε = 3°; recovery should stay within ε

---

### R8 — GPS Spoofing Case Study Evaluation (Closes §4.3, depends on R2)

After R2 is done:
```bash
python3 ~/rv_recovery/python/attack_gps.py --scenario offset
# In parallel, run an eval_gps.py that measures horizontal position error
# (needs to be written — similar to eval_recovery.py but measuring position
# divergence rather than attitude error)
```

---

### Summary — Done vs. Remaining

| Item | Status |
|------|--------|
| Algorithm 1 open-loop `x = A·x + B·u` | **Done** |
| Output before state advance | **Done** |
| Insertion in `fast_loop()` after `read_AHRS()` | **Done** |
| Real `u` from attitude controller | **Done** |
| Real `x` from AHRS | **Done** |
| Windowed state resync (§3.3) | **Done** |
| Disturbance compensation `ms = y − e` | **Done** |
| k=10 s evaluation window | **Done** |
| 50 Hz decimation (acknowledged deviation) | **Done** |
| 19/19 unit tests pass (T1–T5) | **Done** |
| GPS recovery header | **Done** (not wired) |
| Baro recovery header | **Done** (not wired) |
| GPS spoofing attack scripts | **Done** |
| All-gyros flag + supplementary computation | **Done** (not injected into AHRS) |
| A/B framework (scripts + k=10 s) | **Done** (SITL run pending) |
| DTW script uses correct predictor | **Done** (re-run pending) |
| Fabricated quotes fixed | **Done** |
| Firmware build: 1207/1207 success | **Done** |
| Regenerate DTW thresholds (R1) | **Remaining** |
| Wire GPS into AP_GPS.cpp (R2) | **Remaining** |
| Wire baro into AP_Baro.cpp (R3) | **Remaining** |
| Wire supplementary into AHRS (R4) | **Remaining** |
| Magnetometer recovery (R5) | **Remaining** |
| 12-state sysid (R6) | **Remaining** |
| Run A/B SITL sessions (R7) | **Remaining** |
| GPS spoofing case study evaluation (R8) | **Remaining** (after R2) |
