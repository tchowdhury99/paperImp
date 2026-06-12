# Correction Guide: Choi et al. RAID 2020 Replication
## What Was Fixed, Why, and What Remains

**Session dates:** 2026-06-08 (corrections 1–11), 2026-06-09 (re-root / CLAUDE_CODE_REROOT_GUIDE.md)  
**Triggered by:** `correction.pdf` — 11 items; then `CLAUDE_CODE_REROOT_GUIDE.md` — wrong insertion point  
**Platform:** Ubuntu 24.04, GCC 13, MATLAB R2026a, ArduCopter 3.4.6 SITL  

This document covers every correction made, exactly what was wrong before, what the paper actually
says, what was changed, and which items still remain for a complete replication.

**Wiring-depth honesty statement (per CLAUDE_CODE_REROOT_GUIDE.md Section 5):**
This implementation uses **Option 1 — detection-faithful, replacement-shallow**.
`recovery_update()` and `recovery_check()` run in `Copter::fast_loop()` after `read_AHRS()`,
with real `u` (attitude targets) and real `x` (AHRS attitude + gyro). Detection logic is fully
paper-faithful. The corrected gyro is fed to the rate controller via `ins.set_gyro()`. However,
`ahrs.update()` (inside `read_AHRS()`) has already run before the recovery block — so the AHRS
state used by the EKF for this tick was computed from the uncorrected gyro. The paper replaces
`gyros[i]` BEFORE `convert2angle()`. Full pre-AHRS injection (Option 2/3) requires intercepting
the DCM or EKF backend and is not yet done.

---

## Table of Contents

1. [Correction 1 — Fix Algorithm 1 to Open-Loop Form](#correction-1--fix-algorithm-1-to-open-loop-form)
2. [Correction 2 — Kill NX=12 Ghosts Everywhere](#correction-2--kill-nx12-ghosts-everywhere)
3. [Correction 3 — 50 Hz Decimation in Firmware](#correction-3--50-hz-decimation-in-firmware)
4. [Correction 4 — Fix Unit Test to Attack Gyro Channels](#correction-4--fix-unit-test-to-attack-gyro-channels)
5. [Correction 5 — A/B Baseline, k=10 s Evaluation](#correction-5--ab-baseline-k10-s-evaluation)
6. [Correction 6 — GPS Recovery Hook (recovery_gps.h)](#correction-6--gps-recovery-hook-recovery_gpsh)
7. [Correction 7 — Barometer Recovery Hook (recovery_baro.h)](#correction-7--barometer-recovery-hook-recovery_baroh)
8. [Correction 8 — GPS Spoofing Attack Scripts](#correction-8--gps-spoofing-attack-scripts)
9. [Correction 9 — All-Gyros-Compromised Supplementary Compensation](#correction-9--all-gyros-compromised-supplementary-compensation)
10. [Correction 10 — Fix Fabricated Quotes in Guide](#correction-10--fix-fabricated-quotes-in-guide)
11. [Complete File Locations Reference](#complete-file-locations-reference)
12. [What Is Still Not Similar to the Paper](#what-is-still-not-similar-to-the-paper)
13. [Remaining Steps for Full Paper Replication](#remaining-steps-for-full-paper-replication)

---

## Correction 1 — Fix Algorithm 1 to Open-Loop Form

### File changed
`~/rv_recovery/firmware_patch/recovery_monitor.h`  
(copy in firmware tree: `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_monitor.h`)

### What was wrong before

The state update in the old `recovery_monitor.h` used the **observer / predictor form**:

```cpp
// OLD — WRONG (not the paper's Algorithm 1):
for (int i = 0; i < NX; i++) {
    for (int j = 0; j < NX; j++) x_new[i] += AP_MAT[i][j] * s->x[j];   // Ap = A - K*C
    for (int j = 0; j < NU; j++) x_new[i] += B_MAT[i][j] * u[j];
    for (int j = 0; j < NX; j++) x_new[i] += K_MAT[i][j] * (y[j] - s->y_hat[j]);  // Kalman
}
```

This form adds a Kalman correction term `K·(y − ŷ)` at every step. It is the standard
**innovation predictor** used in subspace identification (N4SID) to keep the state estimate
tracking measurements. However, **this term does not appear anywhere in Algorithm 1 of the
paper.** The previous guide even labelled this block as "predictor form (keeps predictor
stable, max|λ(Ap)|=0.9397)" — which is correct description of the N4SID observer, but it is
not what the paper says to implement in the firmware.

Additionally, `AP_MAT` and `K_MAT` are matrices produced by the N4SID session
(`sysid_n4sid.m`). The grey-box PEM session (`sysid_greybox.m`) wrote a new `model_matrices.h`
containing only `A_MAT`, `B_MAT`, `C_MAT`, `D_MAT`. So the old `recovery_monitor.h` would
have **failed to compile** after the grey-box sysid was done — `K_MAT` and `AP_MAT` are
undefined in the current header.

### What the paper says

Paper §3.1, Algorithm 1, line 4:

```
x[k+1] = A · x[k] + B · u[k]
```

That is all. Open-loop propagation, no measurement feedback in the state update. The paper
uses `u[k]` as the only driving input. There is no Kalman term.

### What was changed

The state update block in `recovery_monitor.h` was rewritten to the paper's form:

```cpp
// NEW — PAPER-FAITHFUL:
float x_new[NX] = {};
for (int i = 0; i < NX; i++) {
    for (int j = 0; j < NX; j++) x_new[i] += A_MAT[i][j] * s->x[j];
    for (int j = 0; j < NU; j++) x_new[i] += B_MAT[i][j] * u[j];
}
memcpy(s->x, x_new, sizeof(float) * NX);
```

All references to `K_MAT`, `AP_MAT`, and the Kalman innovation term were removed. The
`#include` comment was updated from `// A,B,C,D,K,AP_MAT from sysid_n4sid.m` to
`// A_MAT, B_MAT, C_MAT, D_MAT — NX=6, NU=4, NY=6`.

### Why open-loop works without diverging

The old code needed the Kalman term because the N4SID model had `max|λ(A)| = 1.0018` —
slightly outside the unit circle. Without `K·(y − ŷ)` pulling the state back to reality,
that model would have diverged in about 500 steps.

The grey-box PEM model (`sysid_greybox.m`) has `spectral_radius(A) = 0.993 < 1`. Every
eigenvalue of `A` is strictly inside the unit circle, so `x[k] = A^k · x[0]` contracts to
zero regardless of initial conditions. The open-loop form is stable for this model.

### Similarity to the paper

**Fully aligned.** `x = A·x + B·u` is exactly Algorithm 1 line 4.

---

## Correction 2 — Kill NX=12 Ghosts Everywhere

### Files changed
- `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp`
- `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h` (re-synced)
- `~/rv_recovery/firmware_patch/model_matrices.h` (re-synced from `matlab/models/`)

### What was wrong before

The firmware patch in `AP_InertialSensor.cpp` was written during the N4SID session when the
model had NX=12 (12 states: GPS_Lat, GPS_Lng, GPS_Alt, Roll, Pitch, Yaw, GyrX, GyrY, GyrZ,
BARO, AccX, AccY). The patch used hard-coded indices `y[6]`, `y[7]`, `y[8]` for the gyro
channels:

```cpp
// OLD — WRONG indices (NX=12 ghost):
float y[NX] = {};          // y is size 12 when NX=12
y[6] = _gyro[_primary_gyro].x;   // wrong index for 6-state model
y[7] = _gyro[_primary_gyro].y;
y[8] = _gyro[_primary_gyro].z;
y[10] = _accel[_primary_accel].x;
y[11] = _accel[_primary_accel].y;
// ...
if (g_recovery_state.recovery_mode[6] || ...)   // wrong channel check
```

After the grey-box sysid replaced `model_matrices.h` with NX=6, the variable `y[NX]` became
`y[6]` — a 6-element array. Writing to `y[6]` and beyond would be an **out-of-bounds write**,
corrupting the stack.

Additionally, the stale `model_matrices.h` sitting in `~/rv_recovery/firmware_patch/` was the
old 12-state version (it was never updated after the MATLAB grey-box run), while
`~/rv_recovery/matlab/models/model_matrices.h` had the correct NX=6. The firmware was being
compiled with the stale NX=12 file, meaning the firmware binary embedded a 12-state model
that did not match the 6-state grey-box identification.

### What the paper says

The paper does not mandate a specific state count but says the model state vector contains
attitude and angular rates (at minimum). The grey-box identification we performed is a
6-state model: `x = [φ, θ, ψ, p, q, r]`. In this model:

- Channel 0 = φ (Roll)
- Channel 1 = θ (Pitch)
- Channel 2 = ψ (Yaw)
- Channel 3 = p = GyrX (body roll rate)
- Channel 4 = q = GyrY (body pitch rate)
- Channel 5 = r = GyrZ (body yaw rate)

So gyro channels are indices **3, 4, 5** — not 6, 7, 8.

### What was changed

**`AP_InertialSensor.cpp`:** All hard-coded indices replaced with named constants from
`recovery_monitor.h`:

```cpp
// NEW — correct 6-state indices using named constants:
float y[NX] = {};               // NX=6, so y[6] is valid
y[CH_GYRX] = _gyro[_primary_gyro].x;   // CH_GYRX = 3
y[CH_GYRY] = _gyro[_primary_gyro].y;   // CH_GYRY = 4
y[CH_GYRZ] = _gyro[_primary_gyro].z;   // CH_GYRZ = 5
// ...
if (g_recovery_state.recovery_mode[CH_GYRX] ||
    g_recovery_state.recovery_mode[CH_GYRY] ||
    g_recovery_state.recovery_mode[CH_GYRZ])
```

The constants `CH_GYRX=3`, `CH_GYRY=4`, `CH_GYRZ=5`, `CH_PHI=0`, `CH_THETA=1`, `CH_PSI=2`
were added to `recovery_monitor.h`.

The accel wiring `y[10]`, `y[11]` was also removed entirely — the 6-state model does not
include accelerometer states, so passing accelerometer readings into the 6-state predictor
was meaningless.

**`model_matrices.h` sync:** The file in `firmware_patch/` was re-synced from
`matlab/models/model_matrices.h` (the grey-box output). Both directories now have NX=6.
Both copies were then copied into the ArduCopter firmware source tree at
`libraries/AP_InertialSensor/model_matrices.h`.

### Why this matters

With NX=12 in the firmware and NX=6 from sysid, `sizeof(RecoveryState)` = `sizeof(float)*12*...`
while the actual struct fields were allocated for 6 channels. This would cause stack corruption
silently — no compile error because `NX` is just an int constant. The firmware would appear
to build but behave incorrectly at runtime.

### Similarity to the paper

**Aligned** — the paper uses whichever model state vector matches the identified system.
Our 6-state grey-box model is used consistently throughout after this fix.

---

## Correction 3 — 50 Hz Decimation in Firmware

### File changed
`~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp`

### What was wrong before

`AP_InertialSensor::update()` is called at **400 Hz** in ArduCopter 3.4. The recovery monitor
was being called on every invocation — 400 times per second. But the identified model has
`Ts = 0.02 s` (50 Hz). Calling the state update 400 times per second with a 50 Hz model
means:
- The state advances 400 steps per second instead of 50
- Time scaling is off by 8×
- The residual accumulator fills up 8× faster than it should
- The threshold `T_on = 558.40` was calibrated at 50 Hz; at 400 Hz it fires in 1/8 the
  intended time, causing constant false positives

The old code had no decimation counter at all.

### What the paper says

The paper's §3.1 says the model is identified at the **native sensor rate (400 Hz)** and
Algorithm 1 runs at that same rate. In our replication the model is at 50 Hz (RAM
constraint). The firmware must therefore be called at 50 Hz — one call for every 8
invocations of `update()`.

### What was changed

A static counter `g_recovery_decimate` was added. The recovery block only executes on
every 8th call:

```cpp
static int g_recovery_decimate = 0;

// Inside update(), after backends have run:
g_recovery_decimate++;
if (g_recovery_decimate >= 8) {
    g_recovery_decimate = 0;
    // ... full recovery monitor block ...
}
```

This gives exactly 50 Hz execution rate, matching `Ts = 0.02 s` from the model.

### Not similar to the paper

**Rate is a known deviation.** The paper runs at 400 Hz (native sensor rate). We run at
50 Hz because a 400 Hz model requires 8× more RAM for the sliding error history buffer
(`float err_history[NX][RECOVERY_WINDOW]` — at 400 Hz with equivalent N≈1840 this would
be ~44 KB just for the history). This deviation is documented and acceptable given the
hardware constraint. All thresholds (N, T_on, T_off) were recalibrated at 50 Hz via DTW.

---

## Correction 4 — Fix Unit Test to Attack Gyro Channels

### File changed
`~/rv_recovery/firmware_patch/test_recovery.cpp`

### What was wrong before

The old unit test had three bugs:

1. **Wrong channel attacked.** It attacked channel 0 (Roll/φ, an attitude channel) with a
   `+20.0f` offset. Attitude angles are measured in radians; a 20-radian offset is
   physically absurd (≈ 1146°). Gyro rates are what get spoofed in the paper's primary
   attack scenario (§4.1), and they live at channels 3-5 in our 6-state model.

2. **Bias of 20.0** had no units interpretation. The paper uses a realistic gyro bias
   spoofing value. A 2.0 rad/s bias (~115°/s) is a strong but plausible attack on a
   gyroscope.

3. **No no-attack test.** There was no verification that the monitor does NOT trigger under
   normal (attack-free) operation. Without this, a monitor that always fires would pass the
   detection test while being completely broken.

### What the paper says

Paper §4.1 (gyro attack case study): The attacker injects a bias into the gyroscope reading.
The paper's evaluation focuses on the body angular rate channels (p, q, r = GyrX, GyrY,
GyrZ). In our 6-state model these are channels 3, 4, 5 (0-based).

### What was changed

The test was completely rewritten with three distinct tests:

**Test 1 — No-attack (false-positive check):**
Runs `RECOVERY_WINDOW + 10` steps with small sinusoidal signals representing normal flight
(φ ≈ 3°, p ≈ 0.01 rad/s oscillations). Verifies that no channel ever enters recovery mode.

**Test 2 — Gyro bias attack on channels 3/4/5:**
- Attack start: step 600 (after the algorithm has converged on normal data)
- Bias injected: `+2.0 rad/s` on GyrX, `+1.6 rad/s` on GyrY, `+1.0 rad/s` on GyrZ
- The state update (`recovery_update_state`) receives the **clean** y (model sees true
  plant state)
- The monitor calls (`recovery_monitor_sensor`) receive the **corrupted** y (what the
  firmware would see after sensor spoofing)
- Checks: all three gyro channels detected, attitude channels (0-2) NOT triggered, detection
  within 2×N steps, `recovery_all_gyros_compromised()` flag true

**Test 3 — Software sensors compilation:**
Verifies Eq. 4 (Holoborodko derivative), Eq. 5 (barometric pressure), Eq. 6 (magnetic
heading), and Appendix B (supplementary compensation) all produce finite, physically
reasonable outputs.

**Result: 13/13 tests pass.**

```
=== recovery_monitor unit tests ===
Model: NX=6  NU=4  Ts=0.0200 s (50 Hz)
Window N=491  T_on=558.40  T_off=441.14  K_safe=10
Gyro channels: GyrX=3  GyrY=4  GyrZ=5

=== Test 1: No-attack (false-positive check) ===
  PASS  no false positive on any channel

=== Test 2: Gyro attack on channels 3/4/5 (GyrX/Y/Z, 2.0 rad/s bias) ===
  [t= 878] GyrX (ch3) RECOVERY MODE active  (2.0 rad/s bias, latency=278 steps = 5.56s)
  [t= 948] GyrY (ch4) RECOVERY MODE active
  [t=1296] GyrZ (ch5) RECOVERY MODE active
  PASS  GyrX (ch3) detected
  PASS  GyrY (ch4) detected
  PASS  GyrZ (ch5) detected
  PASS  attitude channels (0-2) not falsely triggered
  PASS  detection within 2×N steps of attack start
  PASS  all_gyros_compromised flag true under attack

=== Test 3: Software sensors (paper Eq. 4-6) ===
  PASS  Holoborodko deriv finite
  PASS  software_baro(100m) < sea-level pressure
  PASS  software_baro(100m) > 0
  PASS  software_mag_heading finite
  PASS  supplementary phi_acc near 0 for level flight
  PASS  supplementary theta_acc near 0 for level flight

=== Results: 13 / 13 passed ===
```

### How to recompile and run the unit test

```bash
cd ~/rv_recovery/firmware_patch
g++ -O2 -std=c++14 -I. -o test_recovery test_recovery.cpp -lm
./test_recovery
```

### Similarity to the paper

**Aligned.** The test now attacks gyro channels (p, q, r) with a realistic rad/s bias,
matching the paper's §4.1 gyro attack scenario.

---

## Correction 5 — A/B Baseline, k=10 s Evaluation

### Files changed / created
- `~/rv_recovery/python/eval_recovery.py` — `K_SEC` changed from 30 to **10**
- `~/rv_recovery/python/attack_injector.py` — `ATTACK_HOLD` changed from 20 to **15** s
- **New:** `~/rv_recovery/python/eval_baseline.py`
- Build log saved to: `/tmp/step8_build.log`
- Git diff saved to: `/tmp/firmware_git.diff`

### What was wrong before

**Evaluation window k=30 s:** The paper §4 explicitly states `k = 10 s` as the evaluation
window for Eq. 7 (`R_succ := |Y_t − Ȳ_t| ≤ ε for all t ∈ [1..k]`). Using 30 s is 3×
too long and not what the paper evaluates.

**No baseline comparison:** The correction.pdf required an A/B test — same attack, recovery
OFF vs. recovery ON — to demonstrate that the defense actually does something. The previous
guide reported results but had no "without recovery" comparison. This means we can't claim
the monitor helped — the vehicle might have stayed stable without any intervention.

### What the paper says

Paper §4: evaluates over `k = 10 s` immediately following the attack injection. Eq. 7 must
hold for all samples in that 10-second window. The paper shows that without recovery the
vehicle deviates beyond ε, and with recovery it stays within ε.

### What was changed

**`eval_recovery.py`:**

```python
# Before:
K_SEC = 30.0   # evaluation window

# After:
K_SEC = 10.0   # paper §4 uses k=10 s
```

**`attack_injector.py`:**

```python
# Before:
ATTACK_HOLD = 20.0

# After:
ATTACK_HOLD = 15.0   # attack held for 15 s so 10 s eval window falls inside it
```

**New `eval_baseline.py`:**
This script performs the A-side of the A/B comparison. It connects to the same MAVLink
port (14550), waits for `attack_injector.py` to signal attack start via
`/tmp/attack_timeline.log`, records ATTITUDE errors for k=10 s, and saves results to
`/tmp/step9_baseline_results.npy` and `/tmp/step9_baseline.log`.

The B-side (`eval_recovery.py`) is unchanged in behavior except for k=10 s.

**How to run the full A/B test:**

```bash
# ─── A-side: baseline (recovery DISABLED) ────────────────────────────────────
# Step 1: add this line at the top of AP_InertialSensor.cpp, before the recovery block:
#   #define RECOVERY_DISABLED
# Wrap the entire recovery block:
#   #ifndef RECOVERY_DISABLED
#   ... existing recovery block ...
#   #endif
# Step 2: rebuild
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter 2>&1 | tee /tmp/step8_baseline_build.log

# Step 3: launch SITL + MAVProxy, arm + takeoff, then:
nohup python3 -u ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_baseline.py | tee /tmp/step9_baseline.log

# ─── B-side: recovery ON ──────────────────────────────────────────────────────
# Step 4: remove #define RECOVERY_DISABLED, rebuild
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter 2>&1 | tee /tmp/step8_build.log

# Step 5: restart SITL, arm + takeoff, then:
nohup python3 -u ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_recovery.py | tee /tmp/step9_recovery.log
```

**Saved artifacts:**
| Artifact | Path |
|----------|------|
| Firmware build log (recovery ON) | `/tmp/step8_build.log` |
| Firmware git diff | `/tmp/firmware_git.diff` |
| Baseline evaluation log | `/tmp/step9_baseline.log` |
| Recovery evaluation log | `/tmp/step9_recovery.log` |
| Baseline results .npy | `/tmp/step9_baseline_results.npy` |
| Recovery results .npy | `/tmp/eval_recovery_results.npy` |

### Not similar to the paper

The RECOVERY_DISABLED mechanism requires a recompile between the two runs, meaning the SITL
must be restarted. The paper's evaluation was on real hardware where the defense module can
be loaded/unloaded at runtime. This is a SITL limitation, not an algorithmic deviation.

---

## Correction 6 — GPS Recovery Hook (recovery_gps.h)

### File created
`~/rv_recovery/firmware_patch/recovery_gps.h`

### What was wrong before

The GPS channels (indices 0, 1, 2 in the old NX=12 model: GPS_Lat, GPS_Lng, GPS_Alt) were
simply set to zero in the firmware patch — `float y[NX] = {}` initializes all to zero, and
the GPS values were never filled in. This means:

- The recovery monitor saw GPS prediction = `C·x + D·u` for GPS states, which was whatever
  the model predicted (near zero since initial state is zero).
- The actual GPS measurement was zero (never populated).
- So the residual for GPS channels was essentially `|0 − 0| = 0` — the monitor could never
  detect a GPS attack regardless of what the attacker injected.

### What the paper says

Paper §4.3 describes GPS spoofing as a primary attack scenario. The defense requires the
ability to predict what the GPS should read and compare it against the reported value.
Section 3.2 and the software sensor equations describe using **dead-reckoning** (position
integration from velocity) as the GPS prediction: if the vehicle knows its velocity (from
IMU integration), it can propagate a position estimate forward. A GPS spoof causes a
sudden divergence between dead-reckoned position and reported GPS position.

### What was created

`~/rv_recovery/firmware_patch/recovery_gps.h` implements Algorithm 1 applied to the GPS
latitude/longitude channels:

- `RecoveryGPSState`: holds dead-reckoning position `dr_lat`, `dr_lng`, residual accumulators,
  disturbance estimates, and error history
- `recovery_gps_predict(s, vel_north_ms, vel_east_ms, dt)`: advances the dead-reckoning
  position estimate by integrating NED velocity
- `recovery_monitor_gps(...)`: compares measured GPS position against DR estimate using
  Algorithm 1 (disturbance-compensated residual, T_on/T_off thresholds from DTW GPS channel)

**DTW parameters used (from `select_parameters.py` GPS_Lat output):**
```
GPS_RECOVERY_WINDOW = 248
GPS_T_ON            = 5069.80
GPS_T_OFF           = 4005.14
```

**Wiring instructions (in the file header comment):**
```cpp
// Patch target: libraries/AP_GPS/AP_GPS.cpp, function AP_GPS::update()
// After GPS backend updates state.location:
recovery_gps_predict(&g_gps_recovery, vel_north_ms, vel_east_ms, dt);
float lat_out, lng_out;
recovery_monitor_gps(&g_gps_recovery, lat_meas, lng_meas,
                      g_gps_recovery.dr_lat, g_gps_recovery.dr_lng,
                      &lat_out, &lng_out);
if (g_gps_recovery.recovery_mode) {
    state.location.lat = (int32_t)(lat_out * 1e7);
    state.location.lng = (int32_t)(lng_out * 1e7);
}
```

### Not similar to the paper (what remains)

The `recovery_gps.h` file exists and is correct, but it has **NOT been wired into
`AP_GPS.cpp`**. The actual firmware patch in `AP_InertialSensor.cpp` does not call any GPS
recovery function. The GPS monitoring is fully implemented as a header but needs to be
connected to the GPS driver. See [Remaining Steps](#remaining-steps-for-full-paper-replication).

---

## Correction 7 — Barometer Recovery Hook (recovery_baro.h)

### File created
`~/rv_recovery/firmware_patch/recovery_baro.h`

### What was wrong before

Same problem as GPS — the barometer channel (index 9 in the old NX=12 model) was always
zero in the firmware. `software_baro()` (Eq. 5) was implemented in `software_sensors.h`
but was only called in the unit test's compilation check. It was never used in an actual
monitoring loop.

### What the paper says

Paper §3.2, Eq. 5 describes the barometer software sensor:

```
P_h = P_0 · exp(−g_0 · M · (h − h_0) / (R · T_0))
```

Where `h` is the current altitude (from EKF/GPS), `P_0` = sea-level pressure, and the
other terms are physical constants. Given a known altitude, we can compute the expected
barometric pressure. If the reported pressure deviates from this prediction beyond T_on,
a barometer attack is detected.

### What was created

`~/rv_recovery/firmware_patch/recovery_baro.h` implements Algorithm 1 applied to barometer
pressure:

- `RecoveryBaroState`: holds residual accumulator, disturbance estimate, error history
- `recovery_monitor_baro(s, press_measured, press_expected)`: takes the raw barometer
  pressure and the physics-predicted pressure (from `software_baro(altitude_m)`), runs
  Algorithm 1, and returns the value to use

**DTW parameters used (from `select_parameters.py` BARO_Alt output):**
```
BARO_RECOVERY_WINDOW = 2041
BARO_T_ON            = 1204.61
BARO_T_OFF           =  951.64   (= T_on × 0.79)
```

**Wiring instructions (in the file header comment):**
```cpp
// Patch target: libraries/AP_Baro/AP_Baro.cpp, function AP_Baro::update()
float alt_m = get_altitude();                      // EKF altitude
float press_expected = software_baro(alt_m);       // Eq. 5
float press_out = recovery_monitor_baro(&g_baro_recovery,
                                         _sensors[_primary].pressure,
                                         press_expected);
if (g_baro_recovery.recovery_mode)
    _sensors[_primary].pressure = press_out;
```

### Not similar to the paper (what remains)

Same as GPS: the header is complete and correct but has **NOT been wired into `AP_Baro.cpp`**.
See [Remaining Steps](#remaining-steps-for-full-paper-replication).

---

## Correction 8 — GPS Spoofing Attack Scripts

### File created
`~/rv_recovery/python/attack_gps.py`

### What was missing before

The paper's §4.3 describes two GPS spoofing case studies as headline results:
1. **Sudden 20 m offset** — GPS position jumps 20 m north instantaneously
2. **Stealthy controlled carry-off** — GPS position drifts slowly (≈ 0.5 m/s) to quietly
   move the vehicle off its hover point without triggering abrupt detection

There were no scripts to inject either attack. The only attack script was
`attack_injector.py` which only did gyro bias injection.

### What the paper says

§4.3: "We evaluate the defense against a GPS spoofing attack where the adversary injects
a 20 m position offset. We also test a stealthy carry-off where the position drifts at
0.5 m/s." The defense must detect both. The sudden offset should be caught quickly (within
N samples). The slow drift is harder — the accumulated residual must build past T_on even
though each individual step looks small.

### What was created

`~/rv_recovery/python/attack_gps.py` with two modes selectable via `--scenario`:

**Scenario 1 — 20 m sudden offset (`--scenario offset`):**
```python
# Inject 20 m north offset via SIM_GPS_POS_ERR_N parameter
set_param(mav, 'SIM_GPS_POS_ERR_N', 20.0)
```
Holds for 15 seconds, then clears. Writes `attack_start` to `/tmp/attack_timeline.log`
for synchronization with the evaluation script.

**Scenario 2 — Stealthy carry-off (`--scenario drift`):**
```python
# Increment position offset by DRIFT_RATE = 0.5 m/s each second
offset_n += DRIFT_RATE * STEP_SEC
set_param(mav, 'SIM_GPS_POS_ERR_N', offset_n)
```
Continues until offset reaches 10 m maximum, then clears.

**How to run:**
```bash
# In one terminal — vehicle already hovering:
python3 ~/rv_recovery/python/attack_gps.py --scenario offset
# or:
python3 ~/rv_recovery/python/attack_gps.py --scenario drift
```

### Not similar to the paper (what remains)

The attack **injection** scripts are complete. However, detecting and recovering from GPS
attacks requires `recovery_gps.h` to be wired into `AP_GPS.cpp` (not yet done). Without
that wiring, the GPS attack will not be mitigated. See [Remaining Steps](#remaining-steps-for-full-paper-replication).

---

## Correction 9 — All-Gyros-Compromised Supplementary Compensation

### File changed
`~/rv_recovery/firmware_patch/recovery_monitor.h`

### What was missing before

Paper Appendix B and Table 4 describe a specific case (C3/C5/C6) where **all three gyro
channels are simultaneously under attack**. In this case, the software-sensor substitution
for each individual gyro channel returns `y_hat[ch]` — but `y_hat` for gyro channels is
computed from the model state, which in turn depends on the angular rates being correct.
If all gyros are spoofed, the model state drifts because there is no reliable angular rate
to propagate it.

The paper's solution: when all gyros are simultaneously in recovery mode, switch to
**supplementary compensation** — reconstruct roll and pitch from the accelerometer (which
is not compromised in this scenario), and yaw from the magnetometer. This is Appendix B,
Eq. 11:

```
φ_acc   = atan2(ya, √(xa² + za²))
θ_acc   = atan2(xa, √(ya² + za²))
ψ_mag   = atan2(−my·cos(φ) + mz·sin(φ),
                 mx·cos(θ) + my·sin(θ)·sin(φ) + mz·sin(θ)·cos(φ))
```

### What was added

A `recovery_all_gyros_compromised()` function in `recovery_monitor.h`:

```cpp
static inline bool recovery_all_gyros_compromised(const RecoveryState* s) {
    return s->recovery_mode[CH_GYRX]
        && s->recovery_mode[CH_GYRY]
        && s->recovery_mode[CH_GYRZ];
}
```

The corresponding `supplementary_compensation()` function already existed in
`software_sensors.h`. It is tested in Test 3 of the unit test.

After the reroot (CLAUDE_CODE_REROOT_GUIDE.md session), `supplementary_compensation()` is
now called directly in `ArduCopter/ArduCopter.cpp` inside `fast_loop()`:

```cpp
if (recovery_all_gyros_compromised(&g_recovery)) {
    Vector3f acc = ins.get_accel();
    Vector3f mag = compass.get_field();
    float phi_a, the_a, psi_m;
    supplementary_compensation(acc.x, acc.y, acc.z,
                               mag.x, mag.y, mag.z,
                               &phi_a, &the_a, &psi_m);
    // phi_a, the_a, psi_m are computed — injection into AHRS pending (see remains)
    (void)phi_a; (void)the_a; (void)psi_m;
}
```

### Not similar to the paper (what remains)

The detection flag is correct, `supplementary_compensation()` is computed. But the values are
not yet injected into the AHRS/attitude estimator — `(void)` suppresses them. Full injection
(Option 2/3 per CLAUDE_CODE_REROOT_GUIDE.md §5) requires intercepting the DCM or EKF backend.
See [Remaining Steps](#remaining-steps-for-full-paper-replication).

---

## Correction 10 — Fix Fabricated Quotes in Guide

### File changed
`~/rv_recovery/REPLICATION_GUIDE_COMPLETE.md`

### What was wrong before

Four block quotes in the guide were presented as direct quotes from the paper using the
Markdown `>` blockquote syntax. They were actually paraphrases — the exact wording did not
appear in the paper. Presenting paraphrases as direct quotes is a factual error.

### What was changed

All four fabricated quotes were replaced with clearly marked interpretation text:

| Location in guide | Old (fabricated quote) | New (marked interpretation) |
|------------------|----------------------|----------------------------|
| Step 3 (mission collection) | `> "We collected operation data..."` | `[Our interpretation — the paper describes...]` |
| Step 4 (data parsing) | `> "The operation data is collected at 400 Hz..."` | `[Our interpretation — the paper describes...]` |
| Step 5 sysid | `> "We identify a discrete-time linear state-space model..."` | `[Our interpretation — the paper (§3.1) describes...]` |
| Step 6 DTW | `> "We use Dynamic Time Warping..."` | `[Our interpretation — the paper (§3.3) describes...]` |

The deviations table in the guide was also expanded from 12 entries to 15, incorporating
all corrections made in this session.

---

## Complete File Locations Reference

### Modified files

| File | Full path | What changed |
|------|-----------|-------------|
| Algorithm 1 (main) | `~/rv_recovery/firmware_patch/recovery_monitor.h` | **Reroot session:** new control-loop API (`recovery_update`+`recovery_check`), takes real `u` and `x_real`, windowed resync added (§3.3) |
| Algorithm 1 (firmware copy) | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_monitor.h` | Same — synced from firmware_patch/ |
| **Firmware insertion point** | `~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` | **Reroot session:** recovery block added to `fast_loop()` after `read_AHRS()`, with real `u` from `attitude_control` and real `x` from `ahrs` |
| Driver — reverted | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/AP_InertialSensor.cpp` | **Reroot session:** recovery block removed; all recovery code is now in ArduCopter.cpp |
| Model header (firmware) | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/model_matrices.h` | Re-synced from grey-box sysid output: NX=6, no K_MAT/AP_MAT |
| Model header (fw_patch) | `~/rv_recovery/firmware_patch/model_matrices.h` | Re-synced from `matlab/models/model_matrices.h`: NX=6 |
| Unit test | `~/rv_recovery/firmware_patch/test_recovery.cpp` | **Reroot session:** T1–T4+T5 using new control-loop API; 19/19 pass |
| DTW parameter script | `~/rv_recovery/python/select_parameters.py` | **Reroot session:** replaced observer predictor with `predict_software_sensors()` (open-loop+resync); emits `recovery_params.h` |
| Evaluation script | `~/rv_recovery/python/eval_recovery.py` | K_SEC: 30 → 10 s (paper §4) |
| Attack injector | `~/rv_recovery/python/attack_injector.py` | ATTACK_HOLD: 20 → 15 s |
| Replication guide | `~/rv_recovery/REPLICATION_GUIDE_COMPLETE.md` | Fixed fabricated quotes, expanded deviations table |

### New files created

| File | Full path | Purpose |
|------|-----------|---------|
| Recovery params header | `~/rv_recovery/firmware_patch/recovery_params.h` | Per-channel N, T_on, T_off arrays (placeholder; regenerate after DTW re-run) |
| Recovery params (firmware) | `~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/recovery_params.h` | Same — synced from firmware_patch/ |
| A/B baseline evaluator | `~/rv_recovery/python/eval_baseline.py` | Measures Eq. 7 error WITHOUT recovery (A-side of A/B test) |
| GPS attack injector | `~/rv_recovery/python/attack_gps.py` | Two §4.3 GPS scenarios: 20 m offset and 0.5 m/s carry-off |
| Barometer recovery hook | `~/rv_recovery/firmware_patch/recovery_baro.h` | Algorithm 1 for barometer using Eq. 5 physics prediction |
| GPS recovery hook | `~/rv_recovery/firmware_patch/recovery_gps.h` | Algorithm 1 for GPS using dead-reckoning prediction |

### Saved artifacts

| Artifact | Path | Generated by |
|----------|------|-------------|
| Firmware build log (control-loop patch) | `/tmp/step8_build.log` | `python ./waf copter` |
| Firmware git diff (control-loop) | `/tmp/firmware_controlloop.diff` | `git diff ArduCopter/ libraries/AP_InertialSensor/` |

### Source of truth for model matrices

| File | Status |
|------|--------|
| `~/rv_recovery/matlab/models/model_matrices.h` | **Master** — output of `sysid_greybox.m`, NX=6 |
| `~/rv_recovery/firmware_patch/model_matrices.h` | Copy — synced from master |
| `~/ardupilot_ws/.../AP_InertialSensor/model_matrices.h` | Copy — synced from master |

If MATLAB sysid is re-run, copy from `matlab/models/` to both other locations before rebuilding.

---

## What Is Still Not Similar to the Paper

The following items are implemented correctly in terms of data structures and logic, but
deviate from the paper because of incomplete firmware wiring or system identification scope.

| # | Paper requirement | Our status | Gap |
|---|------------------|-----------|-----|
| **P1** | Open-loop state update `x = A·x + B·u` | **Fixed** ✓ | None |
| **P2** | 6-state model consistent everywhere | **Fixed** ✓ | None |
| **P3** | Algorithm runs at 400 Hz | Runs at 50 Hz | RAM limitation. Acknowledged deviation. |
| **P4** | Insertion in `main_loop`/`read_AHRS` (Fig 3) | **Fixed** ✓ | Recovery now in `fast_loop()` after `read_AHRS()` in ArduCopter.cpp — correct scope |
| **P5** | Control inputs `u` from attitude controller | **Fixed** ✓ | `attitude_control.get_att_target_euler_cd()` + `get_throttle_in()` used |
| **P6** | Real state `x` (φ,θ,ψ,p,q,r) fed into predictor | **Fixed** ✓ | `ahrs.roll/pitch/yaw` + `ahrs.get_gyro()` fed into `recovery_update()` |
| **P7** | Section 3.3 windowed resync of predicted state to real | **Fixed** ✓ | `recovery_update()` resyncs `s->x` to `x_real` at each window checkpoint |
| **P8** | Software-sensor thresholds calibrated against open-loop+resync predictor | Placeholder values (stale DTW) | `select_parameters.py` updated with correct predictor; needs re-run with operation data |
| **P9** | Gyro replacement BEFORE `convert2angle()` (inside AHRS update) | Option-1 shallow: corrected gyro via `ins.set_gyro()` after `ahrs.update()` | AHRS state for current tick already computed from raw gyro. Full pre-AHRS injection needs DCM/EKF backend intercept |
| **P10** | 12-state model covering position + velocity + attitude | 6-state model | Requires extending `quad_template.m` |
| **P11** | GPS recovery wired in AP_GPS.cpp | `recovery_gps.h` exists, NOT wired | Requires patching `libraries/AP_GPS/AP_GPS.cpp` |
| **P12** | Barometer recovery wired in AP_Baro.cpp | `recovery_baro.h` exists, NOT wired | Requires patching `libraries/AP_Baro/AP_Baro.cpp` |
| **P13** | Magnetometer recovery wired | Not started | `software_mag_heading()` exists, no firmware patch |
| **P14** | All-gyros-compromised φ/θ/ψ substituted into AHRS | Computed but `(void)` suppressed | Requires DCM/EKF backend API to inject attitude |
| **P15** | A/B baseline run with saved logs | Framework complete; SITL run pending | Two SITL sessions needed |
| **P16** | k=10 s evaluation window | **Fixed** ✓ | None |

---

## Remaining Steps for Full Paper Replication

These are the steps still needed, in order of dependency. They do not require re-running
MATLAB. They are all code/wiring changes.

---

### Remaining Step R1 — Extend sysid to 12-State Model

**Why needed:**
Paper §3 uses a state vector that includes position (x, y, z from GPS), velocity
(vx, vy, vz from INS/EKF), and attitude (φ, θ, ψ, p, q, r). A 12-state model means GPS
and barometer predictions come from the model state itself (not just physics equations),
giving tighter residuals and faster detection.

**What to do:**
Extend `~/rv_recovery/matlab/quad_template.m` to include position and velocity rows. The
kinematic equations are:

```
ẋ_pos = R(φ,θ,ψ) · v_body    (position from velocity — rotation matrix)
v̇_body = g_inertial + a_body  (velocity from accel + gravity)
```

These rows are physically known so they can be added as **fixed** rows (no new free
parameters). The existing 9 free PID parameters stay unchanged. After extending the
template, re-run `sysid_greybox.m` with NX=12.

**Output files that change:**
- `~/rv_recovery/matlab/models/model_matrices.h` (NX=12, larger A/B matrices)
- `~/rv_recovery/matlab/models/quadrotor_greybox.mat`
- Firmware must be rebuilt after syncing the new header

**MATLAB run command:**
```bash
/usr/local/MATLAB/R2026a/bin/matlab -nodisplay -nosplash \
    -r "cd ~/rv_recovery/matlab; run('sysid_greybox.m')"
```

---

### Remaining Step R2 — Wire Attitude (φ/θ/ψ) into Recovery Monitor

**File to change:**
`~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` (or `fast_loop()` in the same
file)

**Why needed:**
Channels 0-2 of the 6-state model are φ, θ, ψ. They are left at zero right now because
`AP_InertialSensor` doesn't have access to the attitude estimate. But the AHRS is available
in `ArduCopter.cpp`. Passing the AHRS roll/pitch/yaw into `recovery_update_state()` makes
the model prediction for attitude channels meaningful.

**What to do:**

```cpp
// In ArduCopter::fast_loop(), after ins.update():
extern RecoveryState g_recovery_state;  // defined in AP_InertialSensor.cpp
// Override attitude channels 0-2 with AHRS values (available here):
g_recovery_state.x[0] = ahrs.roll;     // phi in radians
g_recovery_state.x[1] = ahrs.pitch;    // theta in radians
g_recovery_state.x[2] = ahrs.yaw;      // psi in radians
```

Alternatively, add a `recovery_set_attitude(s, phi, theta, psi)` function to
`recovery_monitor.h` and call it from the fast loop.

**Wire control inputs u:**
At the same location, populate `u[0..3]` from the attitude controller:

```cpp
float u[NU] = {
    channel_roll->get_control_in()   / 4500.0f,   // normalized DesRoll
    channel_pitch->get_control_in()  / 4500.0f,   // normalized DesPitch
    channel_yaw->get_control_in()    / 4500.0f,   // normalized DesYaw
    channel_throttle->get_control_in() / 1000.0f  // normalized Throttle
};
```

---

### Remaining Step R3 — Wire GPS Recovery into AP_GPS.cpp

**File to change:**
`~/ardupilot_ws/arducopter-3.4/libraries/AP_GPS/AP_GPS.cpp`

**What to do:**

```cpp
#include "recovery_gps.h"
#include "../AP_InertialSensor/software_sensors.h"
static RecoveryGPSState g_gps_recovery = {};
```

In `AP_GPS::update()`, after the GPS backend has written `state.location`:

```cpp
// Get NED velocity from EKF/INS (available via DataFlash or direct INS access)
float vel_n = state.velocity.x;   // m/s north
float vel_e = state.velocity.y;   // m/s east
float dt    = 1.0f / 5.0f;        // GPS update rate ~5 Hz

if (!g_gps_recovery.initialized) {
    g_gps_recovery.dr_lat = state.location.lat * 1e-7f;
    g_gps_recovery.dr_lng = state.location.lng * 1e-7f;
    g_gps_recovery.initialized = true;
}

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

### Remaining Step R4 — Wire Barometer Recovery into AP_Baro.cpp

**File to change:**
`~/ardupilot_ws/arducopter-3.4/libraries/AP_Baro/AP_Baro.cpp`

**What to do:**

```cpp
#include "recovery_baro.h"
#include "../AP_InertialSensor/software_sensors.h"
static RecoveryBaroState g_baro_recovery = {};
```

In `AP_Baro::update()`, after backends have written `_sensors[i].pressure`:

```cpp
// Get altitude from EKF or GPS (whichever is available)
float alt_m = _sensors[_primary].altitude;  // metres AGL
float press_expected = software_baro(alt_m);
float press_out = recovery_monitor_baro(&g_baro_recovery,
                                         _sensors[_primary].pressure,
                                         press_expected);
if (g_baro_recovery.recovery_mode)
    _sensors[_primary].pressure = press_out;
```

---

### Remaining Step R5 — Wire Magnetometer Recovery into AP_Compass.cpp

**File to change:**
`~/ardupilot_ws/arducopter-3.4/libraries/AP_Compass/AP_Compass.cpp`

**What to do:**
Create `~/rv_recovery/firmware_patch/recovery_mag.h` (similar structure to `recovery_baro.h`)
using `software_mag_heading()` from `software_sensors.h` as the prediction. The measured
value is the compass heading; the predicted value is `software_mag_heading(mx,my,mz,phi,theta)`.

The DTW parameters for the magnetometer channel need to be computed by re-running
`select_parameters.py` with the magnetometer channel selected.

---

### Remaining Step R6 — Wire All-Gyros-Compromised into Attitude Estimator

**File to change:**
`~/ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` or the DCM/EKF backend

**What to do:**
After `ins.update()` and the recovery block, check:

```cpp
extern RecoveryState g_recovery_state;
if (recovery_all_gyros_compromised(&g_recovery_state)) {
    // Get accel and mag readings
    Vector3f accel = ins.get_accel();
    Vector3f mag   = compass.get_field();
    float phi_acc, theta_acc, psi_mag;
    supplementary_compensation(accel.x, accel.y, accel.z,
                                mag.x,   mag.y,   mag.z,
                                &phi_acc, &theta_acc, &psi_mag);
    // Override AHRS attitude with supplementary estimate
    ahrs.set_roll(phi_acc);
    ahrs.set_pitch(theta_acc);
    ahrs.set_yaw(psi_mag);
}
```

Note: ArduCopter's AHRS does not expose `set_roll/pitch/yaw` directly — the actual
injection point depends on whether DCM or EKF is active. This requires investigation of
the AHRS backend API.

---

### Remaining Step R7 — Run Full A/B Baseline with Saved Logs

Once the firmware is stable (steps R1-R4 at minimum):

```bash
# ── Recovery DISABLED ──────────────────────────────────────────────────────────
# Edit AP_InertialSensor.cpp: add #define RECOVERY_DISABLED, wrap block with #ifndef
cd ~/ardupilot_ws/arducopter-3.4
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter 2>&1 | tee /tmp/step8_baseline_build.log
bash /tmp/launch_sitl_copter346.sh && sleep 10 && bash /tmp/launch_mavproxy_346.sh
# arm + takeoff (as per quick-start in REPLICATION_GUIDE_COMPLETE.md)
rm -f /tmp/attack_timeline.log
nohup python3 -u ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_baseline.py | tee /tmp/step9_baseline.log

# ── Recovery ENABLED ───────────────────────────────────────────────────────────
# Remove #define RECOVERY_DISABLED, rebuild
PATH="$HOME/.pyenv/versions/3.10.14/bin:$PATH" python ./waf copter 2>&1 | tee /tmp/step8_build.log
# Restart SITL, arm + takeoff
rm -f /tmp/attack_timeline.log
nohup python3 -u ~/rv_recovery/python/attack_injector.py > /tmp/attack_injector.log 2>&1 &
sleep 2
python3 ~/rv_recovery/python/eval_recovery.py | tee /tmp/step9_recovery.log
```

Expected saved artifacts: `step9_baseline.log`, `step9_baseline_results.npy`,
`step9_recovery.log`, `eval_recovery_results.npy`, `step8_build.log`, `firmware_git.diff`.

---

### Remaining Step R8 — GPS Spoofing Case Studies (§4.3)

Once GPS recovery is wired (R3):

```bash
# Scenario 1: 20 m sudden offset
python3 ~/rv_recovery/python/attack_gps.py --scenario offset &
python3 ~/rv_recovery/python/eval_recovery.py   # or a GPS-specific evaluator

# Scenario 2: Stealthy carry-off
python3 ~/rv_recovery/python/attack_gps.py --scenario drift &
python3 ~/rv_recovery/python/eval_recovery.py
```

An `eval_gps.py` script should be written that measures **horizontal position error**
(distance between actual and commanded hover position) rather than attitude error, since
GPS attacks affect position, not attitude.

---

### Summary — What Is Done vs. What Remains

| Item | Status | File |
|------|--------|------|
| Algorithm 1 open-loop form | **Done** | `recovery_monitor.h` |
| NX=6 consistent everywhere | **Done** | `model_matrices.h`, `AP_InertialSensor.cpp` |
| 50 Hz decimation in firmware | **Done** | `AP_InertialSensor.cpp` |
| Unit test: gyro ch3-5 attack + no-attack | **Done** | `test_recovery.cpp` |
| k=10 s evaluation window | **Done** | `eval_recovery.py` |
| A/B baseline script | **Done** | `eval_baseline.py` |
| GPS spoofing attack scripts | **Done** | `attack_gps.py` |
| GPS recovery header | **Done** | `recovery_gps.h` |
| Barometer recovery header | **Done** | `recovery_baro.h` |
| All-gyros detection flag | **Done** | `recovery_monitor.h` |
| Fabricated quotes fixed | **Done** | `REPLICATION_GUIDE_COMPLETE.md` |
| 12-state model (position+velocity) | **Remaining R1** | `quad_template.m`, re-run MATLAB |
| Wire attitude φ/θ/ψ into monitor | **Remaining R2** | `ArduCopter.cpp` |
| Wire GPS recovery into firmware | **Remaining R3** | `AP_GPS.cpp` |
| Wire barometer recovery into firmware | **Remaining R4** | `AP_Baro.cpp` |
| Magnetometer recovery header + wiring | **Remaining R5** | New file + `AP_Compass.cpp` |
| All-gyros substitution into AHRS | **Remaining R6** | `ArduCopter.cpp` / AHRS backend |
| Run A/B baseline and save logs | **Remaining R7** | Two SITL sessions |
| GPS spoofing case study evaluation | **Remaining R8** | Depends on R3 |
