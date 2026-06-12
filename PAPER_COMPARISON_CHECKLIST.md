# Paper vs. Our Implementation — Similarity Checklist

**Paper:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic
Vehicles*, RAID 2020.
**Purpose:** checkpoint-by-checkpoint comparison of the paper's procedure against what we
built, rated by **process similarity** (does our process match the paper's process).

**Legend**
- ✅ **Match** — same process as the paper, implemented faithfully.
- 🟡 **Match w/ noted difference** — same process; one documented choice or the 50 Hz rate.
- 🔵 **Process done, not closed-loop** — the procedure is implemented and demonstrated
  offline on recorded data, but not run live in the closed loop (blocked by the SITL
  sensor-path issue, see L2).
- ⬜ **Not done** — process not implemented.

Similarity column: H = high (same algorithm/equations), M = medium (same idea, different
detail), L = low.

---

## 1. Offline data pipeline (paper §3.1)

- [x] ✅ **Collect normal-operation logs under many maneuvers** — *Similarity: H*
  Paper: random missions from MAVLink commands (straight, turns, etc.).
  Ours: `python/collect_logs.py` flies random GUIDED missions (straight/turn/climb/hover);
  20 missions collected. → `data/sitl_run1/logs/*.BIN`
- [x] ✅ **Collect at high sampling rate** — *Similarity: H*
  Paper: high-rate Dataflash logging. Ours: `LOG_BITMASK` with ATTITUDE_FAST (25 Hz logging).
- [x] ✅ **Resample heterogeneous streams to one rate via spline interpolation** — *Similarity: H*
  Paper: spline interpolation to a common frequency. Ours: `python/parse_dataflash.py`
  CubicSpline per contiguous segment. → `data/operation_data_50hz.mat`
- [x] 🟡 **Target frequency** — *Similarity: M (rate differs)*
  Paper: 400 Hz. Ours: **50 Hz** (sanctioned RAM deviation; everything downstream consistent).
- [x] ✅ **Define x(t), u(t), y(t)** — *Similarity: H*
  Paper Eq. (3): `x=[x y z φ θ ψ ẋ ẏ ż p q r]`, `u` = target states, `y` = sensor outputs.
  Ours: identical 12-state vector; `u=[φ_cmd θ_cmd ψ_cmd thr tiltN tiltE const]` (target
  states + Appendix-A frame-canonicalized tilt). → `parse_dataflash.py`, `quad_template`

## 2. System identification (paper §3.1)

- [x] ✅ **Use MATLAB SI Toolbox / prediction-error minimization** — *Similarity: H*
  Paper: SI Toolbox, iterative PEM. Ours: `matlab/sysid_12state.m` uses `ssest` (PEM).
- [x] ✅ **Per-variable model order (dominant 2nd-order dynamic)** — *Similarity: H*
  Paper: "for each variable we specify a model order"; dominant dynamic is 2nd order.
  Ours: six per-axis **2nd-order** blocks (roll, pitch, yaw, N, E, Up).
- [x] ✅ **Discrete state-space, open-loop form (no observer term)** — *Similarity: H*
  Paper Alg. 1 line 7: `x ← Ax + Bu` (open loop). Ours: `K = 0` fixed in identification.
- [x] ✅ **Model encodes controller + dynamics (closed-loop plant)** — *Similarity: H*
  Paper: template encodes a PID controller + dynamics. Ours: per-axis closed-loop 2nd-order
  blocks identified from flight data (controller already in the loop). → `quad_template.m`
- [x] ✅ **Output the A,B,C,D model** — *Similarity: H*
  → `matlab/models/quadrotor_12state.mat`, `model_matrices.h` (NX=12)
- [x] 🟡 **Model accuracy** — *Similarity: M (data coverage)*
  Roll/pitch fit well; translational/yaw sub-models under-excited by hover-dominant data
  (limitation L3). Same *procedure*, weaker *data coverage*.

## 3. Software-sensor conversions (paper §3.2, Appendix A/B)

- [x] ✅ **Gyroscope = angular-rate states** — *Similarity: H* → `software_sensors.h`, monitor
- [x] ✅ **Accelerometer = Eq. (4) finite difference of velocity** — *Similarity: H*
  With the Holoborodko smooth noise-robust differentiator (paper §3.3). → `software_accel()`
- [x] ✅ **Barometer = Eq. (5) altitude→pressure** — *Similarity: H* → `software_baro()`
- [x] ✅ **Magnetometer heading = Eq. (6)** — *Similarity: H* → `software_mag_heading()`
- [x] ✅ **GPS = position/velocity states** — *Similarity: H* (monitor uses model pN,pE,alt,vN,vE,vUp)
- [x] ✅ **Frame transforms R, Rᵀ, W_η, W_η⁻¹ (Eq. 8–10)** — *Similarity: H*
  → `body_to_inertial_R()`, `body_to_euler_rates()` in `software_sensors.h`
- [x] ✅ **Supplementary compensation Eq. (11) (roll/pitch from accel, yaw from mag)** — *Similarity: H*
  → `supplementary_compensation()`

## 4. Error correction (paper §3.3)

- [x] ✅ **Low-pass filter the real measurement (Alg. 1 line 8)** — *Similarity: H*
  Paper: standard LPF, pre-selected cutoff. Ours: 2nd-order Butterworth (5 Hz @ 50 Hz, documented).
- [x] ✅ **Conversion-error handling via noise-robust differentiator** — *Similarity: H* (Holoborodko)
- [x] ✅ **Model-error handling: periodic synchronization + error reset** — *Similarity: H*
  Paper §3.3: partition into windows, sync software sensors to real at window start.
  Ours: window checkpoint re-seeds model state + resets residual. Demonstrated in **Fig. 12**.
- [x] ✅ **External-error compensation `e = avg(ms − m)` over previous window** — *Similarity: H*
  Paper §3.3. Ours: exact (and the sign bug from prior work was corrected). Demo **Fig. 13**.

## 5. Recovery parameter selection (paper §3.3)

- [x] ✅ **Window size N = max DTW time-displacement over clean data** — *Similarity: H*
  Paper: dynamic time warping. Ours: `python/select_parameters.py` uses `dtaidistance` DTW.
- [x] ✅ **Threshold T = e_max + margin, T_off < T_on** — *Similarity: H*
  → `recovery_params.h` (per-channel N, T_on, T_off)
- [x] ✅ **Predictor used for calibration matches the runtime monitor** — *Similarity: H*
  Same LPF, sign, gating, sync as the firmware monitor.
- [x] 🔵 **FP/FN parameter study (paper Fig. 14)** — *Similarity: H, offline*
  Paper: 20 clean + 20 attacked missions, FP/FN vs (window, threshold); zero FP/FN at chosen
  params. Ours: replicated **offline on recorded data** — `fig10`, `fig14`; FP≈0, FN≈0 at the
  selected T_on. Process matches; not a live closed-loop run.

## 6. Runtime recovery monitor — Algorithm 1 (paper §3.4)

- [x] ✅ **Line 6: y ← Cx + Du (output before state advance)** — *Similarity: H*
- [x] ✅ **Line 7: x ← Ax + Bu (open loop)** — *Similarity: H*
- [x] ✅ **Line 8: m ← filter(m)** — *Similarity: H*
- [x] ✅ **Line 9: ms ← convert(y)** — *Similarity: H*
- [x] ✅ **Lines 11–15: window checkpoint (gated on !recovery_mode), e update, ms←m, sync** — *Similarity: H*
- [x] ✅ **Line 16: ms ← ms − e** — *Similarity: H*
- [x] ✅ **Line 17: r ← r + |m − ms|** — *Similarity: H*
- [x] ✅ **Lines 18–22: T_on detect, T_off + safe-count K switch-back** — *Similarity: H*
- [x] ✅ **Line 20: replace compromised sensor (m ← ms)** — *Similarity: H*
- [x] ✅ **Line 23: optional recovery_action()** — *Similarity: H* (GCS-alert hook)
- [x] ✅ **Verification** — 25/25 conformance tests pin each line. → `firmware_patch/test_recovery.cpp`
  → all of the above in `firmware_patch/recovery_monitor.h`

## 7. Firmware integration (paper §3.4, Fig. 3)

- [x] ✅ **Insert recovery right after sensor acquisition, before fusion** — *Similarity: H*
  Paper Fig. 3: inside `read_AHRS()`, replace `gyros[i]` before `convert2angle()`.
  Ours: in `Copter::read_AHRS()` before `ahrs.update()`. → `ArduCopter/ArduCopter.cpp`
- [x] ✅ **Monitor each physical sensor instance (not the fused one)** — *Similarity: H*
  Per-instance gyro + accel; plus GPS, baro, magnetometer channels.
- [x] ✅ **Driver substitution hooks** — *Similarity: H*
  `AP_Baro::recovery_set_pressure`, `AP_GPS::recovery_override`, `AP_Compass::recovery_set_field`.
- [x] ✅ **Firmware compiles (recovery-on, recovery-off, attack module)** — *Similarity: H*
- [ ] 🔵 **Substitution propagates to the control loop in SITL** — *Similarity: H (process), blocked (live)*
  ArduCopter 3.4 SITL takes attitude from the perfect physics model and regenerates the IMU on
  an async backend thread, so the `read_AHRS`-level substitution (attack AND recovery, identical)
  is overwritten before the estimator reads it. Same insertion point as the paper; the live
  closed-loop effect needs real hardware or a non-faithful SITL workaround (L2).

## 8. Sensor-replacement / fusion logic (paper §2.2, §11)

- [x] ✅ **Replace if |software − real| > threshold, else keep real** — *Similarity: H* (Alg. 1 l.18–20)
- [x] ✅ **Healthy passthrough; substitution only under recovery** — *Similarity: H* (Fig. 3 semantics; test T7)

## 9. Supplementary compensation — all gyros compromised (paper §3.3, Appendix B)

- [x] ✅ **Detect all-gyros-compromised condition** — *Similarity: H*
- [x] ✅ **Reconstruct roll/pitch from accel, yaw from mag (Eq. 11), LPF, weighted sum** — *Similarity: H*
- [x] 🔵 **Demonstration** — *Similarity: H, offline*
  Replicated offline from the raw log: gyro-only roll diverges (~575°) vs bounded with
  compensation — **Fig. 15**. Process matches; not a live closed-loop run.

## 10. Attack model (paper §4.1)

- [x] ✅ **Attack code in the firmware sensor interface, MAVLink-triggered** — *Similarity: H*
  Paper: malicious code injects constant/sine/random into sensor readings, MAVLink-triggered.
  Ours: firmware attack hook (constant/sine/random on the gyro), triggered by RC override.
  → `ArduCopter/ArduCopter.cpp` (compiles into A-side and B-side builds)
- [x] 🔵 **Attack reaches the control loop in SITL** — *Similarity: H (process), blocked (live)*
  Fires correctly (GCS-confirmed) but does not propagate, for the same L2 reason as recovery.
- [x] ✅ **GPS spoofing attack scripts (offset / carry-off, §4.3)** — *Similarity: H* → `python/attack_gps.py`

## 11. Recovery success criterion (paper Eq. 7)

- [x] ✅ **R_succ: |Y_t − Ȳ_t| ≤ ε for t∈[1..k] (ε=3°, k=10 s)** — *Similarity: H*
  → `python/eval_recovery.py`, `python/case_study_gyro.py`

## 12. Effectiveness evaluation (paper §4.2.2)

- [x] 🔵 **(1) Software sensors predict real readings under maneuvers (Fig. 11)** — *Similarity: H, offline*
  → `fig3_sensor_prediction.png` (GPS/baro/gyro/mag)
- [x] 🔵 **(2) Error correction attenuates prediction errors (Fig. 12, 13)** — *Similarity: H, offline*
  → `fig12` (drift/sync), `fig13` (wind/external `e`)
- [x] 🔵 **(3) Parameter selection effective (Fig. 14)** — *Similarity: H, offline*
  → `fig10`, `fig14` (FP/FN vs threshold & window; ~0 FP/FN at selected params)
- [x] 🔵 **(5) Various attack scales (Fig. 16b)** — *Similarity: H, offline*
  → `fig11_error_vs_attack_scale.png`
- [ ] 🔵 **(4) Recover from multiple/all-sensor attacks in flight (Fig. 15, Tables 3/4)** — *Similarity: H (offline Fig. 15), blocked (live)*
  Supplementary-compensation process replicated offline (`fig15`); live multi-attack recovery
  blocked by L2.

## 13. Attack matrix & case studies (paper Tables 3/4, §4.3, Figs. 16/17/18)

- [ ] 🔵 **Table 3 (GPS/baro/gyro combinations)** — live A/B blocked by L2; scripts/builds ready.
- [ ] 🔵 **Table 4 (1/2/3-of-3 gyros + TMR comparison)** — offline all-gyro demo done (`fig15`); live blocked.
- [ ] 🔵 **Gyro case study (Fig. 17)** — runner ready (`case_study_gyro.py`); live A/B blocked by L2.
- [ ] 🔵 **GPS case study (Fig. 18)** — attack scripts ready (`attack_gps.py`); live blocked by L2.
- [ ] ⬜ **Wind-speed / recovery-duration sweep (Fig. 16a)** — not done (needs live wind runs).

## 14. Vehicles & platform (paper Table 1)

- [x] ✅ **Simulated quadrotor (ArduCopter)** — *Similarity: H* (full pipeline end-to-end)
- [ ] 🟡 **Simulated rover (APMrover2)** — built, **not patched** (paper also evaluates a rover).
- [ ] ⬜ **Simulated hexrotor** — not done.
- [ ] ⬜ **Real vehicles (3DR Solo, Erle-Rover)** — out of scope (no hardware).

---

## Summary scoreboard

| Process group | Status |
|---|---|
| Data pipeline (§3.1) | ✅ Match (50 Hz rate aside) |
| System identification (§3.1) | ✅ Match (procedure); 🟡 model data-coverage |
| Software-sensor equations (§3.2, App. A/B) | ✅ Match (all equations) |
| Error correction (§3.3) | ✅ Match |
| Parameter selection / DTW (§3.3) | ✅ Match; FP/FN study offline |
| Algorithm 1 runtime monitor (§3.4) | ✅ Match (25/25 tests) |
| Firmware insertion point (Fig. 3) | ✅ Match (process); 🔵 live propagation blocked |
| Attack module (§4.1) | ✅ Match (process); 🔵 live propagation blocked |
| Success criterion (Eq. 7) | ✅ Match |
| Effectiveness figures (§4.2.2) | 🔵 Match offline (Figs. 11–16b reproduced) |
| Live A/B + attack matrix (Tables 3/4, §4.3) | 🔵 Blocked by L2 (SITL sensor path) |
| Rover / hexrotor / real vehicles | 🟡 / ⬜ remaining |

**Bottom line on similarity:** every *offline* process of the paper — data collection,
spline resampling, per-variable PEM identification (K=0), the Eq. 4–6/8–11 software sensors,
§3.3 error correction, DTW parameter selection, and the Algorithm-1 monitor — matches the
paper's procedure at high similarity and is demonstrated on recorded data (Figs. 11–16b
reproduced). The only processes not matched *live* are the closed-loop attack/recovery runs,
which are blocked by an ArduCopter 3.4 SITL sensor-path detail (L2), not by any divergence
from the paper's method. The single intentional deviation is the 50 Hz vs 400 Hz rate.

(See `REPLICATION_REPORT.md` for full detail, file locations, deviations, and limitations
L1–L4; `EXECUTION_PLAN.md` for the running execution log.)
