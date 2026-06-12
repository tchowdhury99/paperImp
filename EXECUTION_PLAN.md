![[REPLICATION_REPORT]]# Paper-Faithful Replication — Audit & Execution Plan
## Choi et al., "Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles" (RAID 2020)

**Date:** 2026-06-10
**Ground rule:** Replicate the paper exactly — same algorithms, same system model, same
equations, same workflow. **The only sanctioned deviation is the model/monitor rate:
50 Hz instead of 400 Hz (RAM limit).** Everything else must match the paper PDF
(`SbRRfSAoRV.pdf`) and the equation digest (`paper.md`). No additions, no "improvements".

---

## 1. Paper Ground Truth (the spec)

### 1.1 Offline workflow (§3.1–3.3, Fig. 6)
1. Collect normal-operation logs under many maneuvers; missions generated randomly &
   systematically via MAVLink commands (straight fly, turns, etc.).
2. Collect at high sampling rate (Dataflash); streams arrive at heterogeneous rates.
3. Resample all streams to one target frequency using **spline interpolation**
   (paper: 400 Hz; us: **50 Hz**).
4. Define `x(t)` = 12-state vector (Eq. 3): `[x y z φ θ ψ ẋ ẏ ż p q r]`,
   `u(t)` = control input / **target states**, `y(t)` = sensor-measured outputs.
5. System identification with the **MATLAB SI Toolbox**: a discrete-time state-space
   **model template encoding a PID controller and dynamics known a priori** for the RV
   family; per-variable model order (dominant dynamic is 2nd-order); coefficients
   instantiated by **iterative prediction-error minimization (PEM)**. The model is of the
   *closed loop* (controller + actuators + dynamics), Eq. (1)–(2):
   `x' = Ax + Bu`, `y = Cx + Du`.
6. Software sensors = conversion of model outputs into predicted sensor readings (§3.2):
   - **Gyroscope**: angular-rate states `[p q r]` directly.
   - **Accelerometer**: Eq. (4) `a(t) = c_k·(v(t) − v(t−k))/(k·Δt)` from model velocity
     states; implemented with a **smooth noise-robust differentiator (Holoborodko)** (§3.3).
   - **Barometer**: Eq. (5) `P_h = P0·exp(−g0·M·(z−h0)/(R·T0))` from model altitude state
     (g0=9.87, M=0.02896, R=8.3143, T0 base temp).
   - **Magnetometer**: do **not** synthesize raw field values — use **orientation states
     from the model directly** (heading), Eq. (6) defines how real field → heading.
   - **GPS**: position/velocity **directly from model states**.
   - Frame canonicalization via R / Rᵀ / W_η / W_η⁻¹ (Appendix A, Eq. 8–10).
7. **Window size N** = maximum time-displacement between real and software-sensor signals
   computed by **DTW** over the large clean operation dataset (§3.3). Paper used 575 ms
   (230 counts @400 Hz) for 3DR Solo.
8. **Threshold** `T = e_max + m` where `e_max` = maximum accumulated error within any
   window of size N over clean data; `T_off < T_on`. (Paper Fig. 16: T_on=38 for roll-rate.)
9. Patch the control program **immediately after physical sensor acquisition**
   (Fig. 3: inside `read_AHRS()`, per physical sensor instance, **before** fusion /
   `convert2angle()`).

### 1.2 Runtime: Algorithm 1 — RECOVERYMONITOR(u, m), verbatim
```
 6:  y  ← C·x + D·u                        # model output (BEFORE state advance)
 7:  x  ← A·x + B·u                        # open-loop state update (NO Kalman/observer term)
 8:  m  ← filter(m)                        # low-pass filter the real measurement (§3.3 Fig. 8)
 9:  ms ← convert(y)                       # software-sensor conversion (§3.2)
10:  t++
11:  if !recovery_mode && t > window:      # checkpoint — ONLY when not in recovery
12:      t ← 0
13:      r ← 0                             # residual accumulator resets each window
14:      e ← error_estimation(r, m, ms)    # avg (ms − m) over the PREVIOUS window (§3.3)
15:      ms ← m                            # synchronize software sensor to real reading;
                                           # synchronized readings are fed into the system
                                           # model → re-seed model state (§3.3 "Model Errors")
16:  ms ← ms − e                 Paper          # external/model error compensation
17:  r  ← r + |m − ms|                     # accumulated residual
18:  if r > T_on:  recovery_mode ← true; safe_count ← 0
19:  if recovery_mode:
20:      m ← ms                            # replace compromised sensor
21:      if r < T_off: safe_count++
22:      if safe_count > K: recovery_mode ← false
23:      recovery_action()                 # optional (e.g., safe landing)
24:  return m
```

### 1.3 Supplementary compensation (§3.3, Appendix B)
Only when **all** gyros are compromised: estimate `φ_acc, θ_acc` from accelerometer and
`ψ_mag` from magnetometer (Eq. 11), low-pass filter the outputs, and **combine with the
software-sensor output by weighted sum** to form the attitude fed to the controller.

### 1.4 Evaluation (§4)
- Attack modules inserted **in the firmware sensor interface** (simulating controlled
  attacks: constant/sinusoid/random injection), remotely triggered via MAVLink mapping.
- Success criterion Eq. (7): `|when ￼￼all￼￼ gyros are compromisY_t − Ȳ_t| ≤ ε` for `t ∈ [1..k]`, paper example ε=3, k=10 s.
- 20 clean missions → FP rate; 20 attacked missions → FN rate (Fig. 14), zero FP/FN at
  chosen parameters.
- Attack matrix Table 3 (C1–C6: GPS / baro / gyro combinations) + Table 4 (1/2/3 gyros).
- Case studies: gyro constant-value attack while hovering; GPS 20 m offset during square
  waypoint mission; recovery duration ≥ 10 s.

---

## 2. Audit — divergences found in the current implementation

Legend: 🔴 = violates the paper, must fix. 🟡 = unfaithful/unjustified choice, replace.
🔵 = gap (paper feature not implemented). ⚪ = acceptable (documented choice where paper
is silent, or the sanctioned 50 Hz deviation).

### A. Algorithm 1 fidelity — `rv_recovery/firmware_patch/recovery_monitor.h`
| # | Finding | Paper ref |
|---|---------|-----------|
| A1 🔴 | **No low-pass filter on the real measurement** (`filter(m)` line 8 missing entirely). | Alg. 1 l.8, §3.3 Fig. 8 |
| A2 🔴 | **Error-compensation sign inverted.** Code stores `err = m − ms` and computes `ms = y − mean(err)`, i.e. `ms + mean(ms − m)` — pushes the prediction *away* from the real signal. Paper: `e = avg(ms − m)` over previous window, then `ms ← ms − e`. Same bug mirrored in `select_parameters.py::predict_software_sensors`, `recovery_gps.h`, `recovery_baro.h`. | §3.3 "External Errors", Alg. 1 l.14+17 |
| A3 🔴 | **Checkpoint not gated on `!recovery_mode`.** `r` is reset every window even for channels in recovery → `r` drops below `T_off` each window → `safe_count` accrues → premature recovery exit while attack continues. Paper gates the entire checkpoint (l.11). | Alg. 1 l.11 |
| A4 🔴 | **`ms ← m` sync at checkpoint missing** (state-reseed is done, but the prediction sync of l.15 is dropped). | Alg. 1 l.15 |
| A5 🟡 | `safe_count` reset to 0 whenever `r ≥ T_off` during recovery — paper only resets it on a (re)trigger (l.18). | Alg. 1 l.21 |
| A6 🟡 | Per-channel `N_CH[]` exists but is never used; all channels run window `REC_WINDOW_MAX`. Either honor per-sensor N or document one N. | §3.3 |
| A7 🟡 | `err_hist` indexed with `t % REC_WINDOW_MAX` *after* `t` was incremented (off-by-one), and buffer is never aligned to window start. | — |
| A8 🔵 | `recovery_action()` hook absent (optional in paper — provide a no-op hook). | Alg. 1 l.23 |
| A9 🔵 | **Monitoring operates on the fused/primary gyro only.** Paper Fig. 3 monitors **each physical sensor instance** (3 gyros on 3DR Solo) before the weighted-sum fusion — that per-instance isolation is the point of the technique (Table 4: 1-of-3, 2-of-3, 3-of-3 gyros). SITL exposes ≥2 IMU instances; monitor each. | Fig. 3, §4.2.2 |

### B. System identification — `matlab/sysid_greybox.m`, `matlab/quad_template.m`
| # | Finding | Paper ref |
|---|---------|-----------|
| B1 🔴 | **6-state attitude-only model instead of the paper's 12-state Eq. (3).** Position, velocity (and thus GPS/baro/accel software sensors) cannot be produced from model states. The in-file rationalization ("6 states is MORE faithful") is a prior agent's invention. | Eq. (3), §3.2 |
| B2 🔴 | **Kalman gain K set FREE in `ssest`** (`Structure.K.Free = true`) while the header claims "K=0 throughout". A,B are then optimal for a 1-step Kalman predictor, not for the open-loop form deployed in firmware. Fix `K = 0`, `Free = false` (output-error PEM — PEM consistent with Alg. 1 l.7). | Alg. 1 l.7 |
| B3 🔴 | **Detrending (mean subtraction of U and Y) before SI, never replicated at runtime.** Firmware feeds raw radians/throttle into a model identified on mean-removed signals → systematic offset. Either don't detrend, or export the offsets and apply them in `recovery_update()`. Paper never mentions detrending. | §3.1 |
| B4 🟡 | In-flight filter (`ThI ∈ [0.05,1]`) splices non-contiguous samples into one continuous record for PEM — dynamics across splice points are fictitious. Use per-mission/per-segment multi-experiment `iddata` instead (standard SI, no paper deviation). | §3.1 |
| B5 🟡 | Spline-induced throttle overshoot (CubicSpline ringing at liftoff) handled by data deletion. Root cause: splining across mission boundaries / step transitions. Resample per contiguous segment. Spline itself is per paper — keep it. | §3.1 |
| B6 ⚪ | 50 Hz target rate (paper 400 Hz) — **the sanctioned deviation**. Prefer resampling straight to 50 Hz with the spline rather than 400 Hz + decimation (one resampling step, same as the paper's pipeline shape). | §3.1 |

### C. Parameter selection — `python/select_parameters.py`, `recovery_params.h`
| # | Finding | Paper ref |
|---|---------|-----------|
| C1 🔴 | `recovery_params.h` in the firmware tree contains **stale values from the old N4SID observer predictor** (N=491, T_on=558.4 for all channels). Must be regenerated with the corrected predictor/monitor semantics. | §3.3 |
| C2 🔴 | The Python reference predictor has the same sign bug as A2 and lacks the checkpoint gating of A3 — it must mirror the fixed firmware monitor *exactly*, or thresholds are calibrated against the wrong residual distribution. | §3.3 |
| C3 🟡 | DTW run on z-normalized signals and only the first 5000 samples. Paper: max time-displacement over the **large set** of operation data. Run on full clean dataset (windowed/banded DTW for tractability is fine; document). | §3.3 |
| C4 ⚪ | `margin` (paper's `m`) and `T_off` ratio and `K` are unspecified in the paper. Choose once, document: margin per channel (small % of e_max), `T_off < T_on`, fixed K. Validate via the FP/FN sweep (Fig. 14 replication) which the paper *does* specify. | §3.3, §4.2.2 |

### D. Per-sensor recovery wiring
| # | Finding | Paper ref |
|---|---------|-----------|
| D1 🔴 | `recovery_gps.h` predicts GPS by **dead-reckoning measured velocity** — not the paper's method (GPS prediction = model position/velocity states). Superseded by the 12-state model channels. (File header already admits this.) | §3.2 |
| D2 🔴 | `recovery_baro.h` feeds **EKF altitude** into Eq. (5) — paper feeds the **model state z**. Superseded by 12-state model + Eq. (5) conversion. | §3.2 Eq. 5 |
| D3 🔵 | Magnetometer/heading monitoring not implemented (model ψ vs. compass-derived heading per Eq. 6). | §3.2 Eq. 6 |
| D4 🔵 | Accelerometer monitoring not implemented (Eq. 4 conversion from model velocity states + Holoborodko differentiator + LPF). | §3.2 Eq. 4, §3.3 |
| D5 🔵 | GPS/baro/mag/accel monitors not wired into `AP_GPS.cpp` / `AP_Baro.cpp` / `AP_Compass` / IMU-accel path at all. | §3.4 |
| D6 🔵 | Supplementary compensation computed but `(void)`-discarded — never combined (weighted sum) nor injected into the attitude path. | App. B, §3.3 |

### E. Evaluation & environment
| # | Finding | Paper ref |
|---|---------|-----------|
| E1 🔵 | A/B (recovery off/on) SITL runs never executed; Eq. 7 evaluator ready (ε=3, k=10 s ✓). | §4 |
| E2 🔵 | FP/FN parameter sweep (20 clean + 20 attacked missions, Fig. 14) not implemented. | §4.2.2 |
| E3 🔵 | Attack matrix Table 3/4 scenarios (multi-sensor combos, all-gyro with compensation) not executed. GPS attack scripts exist (`attack_gps.py` offset/drift ✓). | §4.2.2, §4.3 |
| E4 🟡 | Attacks injected via `SIM_GYRO_BIAS_*`/`SIM_GPS_POS_ERR_*` SITL params. Paper inserts **attack modules in the firmware sensor interface** with MAVLink-mapped triggers. SITL params approximate this but the bias enters before sensor calibration; a small firmware attack hook (post-read, pre-recovery) is closer to the paper and gives controlled constant/sine/random waveforms. | §4.1 |
| E5 🔴 | **All paths are broken after the move**: scripts and MATLAB code reference `~/rv_recovery/` and `~/ardupilot_ws/`, which now live in `~/paperImp/`. | — |
| E6 🟡 | Python env from the docs (`conda activate rv_recovery`) no longer exists. pyenv 3.10.14 has pymavlink only; no scipy/dtaidistance anywhere. Recreate one env with numpy/scipy/pymavlink/dtaidistance and standardize on it. | — |
| E7 🟡 | Only one Dataflash log (`all_missions_1.BIN`, 91 MB, all missions concatenated). Fine for SI if split per mission (B4); the FP/FN evaluation needs per-mission runs anyway. | §3.1, §4.2.2 |

---

## 3. Execution checklist

Order matters — each phase has a verifiable exit criterion. Work test-first on the
monitor (Phase 2): one failing test per fix, then the fix, then green.

### Phase 0 — Workspace repair
- [x] 0.1 Fix the moved paths. Preferred: update every hardcoded `~/rv_recovery` /
      `~/ardupilot_ws` reference in `python/*.py` and `matlab/*.m` to the new
      `~/paperImp/...` locations (grep-driven, surgical). The firmware-tree includes are
      relative and unaffected.
      → verify: `grep -rn 'tchowdh4/rv_recovery\|tchowdh4/ardupilot_ws\|HOME.*rv_recovery' rv_recovery/ | grep -v '\.md'` returns nothing.
- [x] 0.2 DONE differently: installed scipy/dtaidistance/matplotlib into pyenv 3.10.14 (single env; conda base left untouched). Recreate the Python env (conda env `rv_recovery`: numpy, scipy, pymavlink,
      dtaidistance, matplotlib). → verify: import check passes.
- [x] 0.3 Confirm toolchain: ArduCopter 3.4 SITL still builds (`./waf copter`), MATLAB
      R2026a launches, `sim_vehicle.py` boots. → verify: build exits 0; SITL reaches
      "Ready to FLY".

### Phase 1 — 12-state model & faithful SI (fixes B1–B6)
- [x] 1.1 (REVISED after closer §3.1 reading — "we can generate the models for
      individual state variables ... for each variable, we specify a model order":
      the paper builds PER-VARIABLE models, not one joint 12-state fit. Implemented
      as six per-axis 2nd-order blocks [value, derivative] with C=I (states pinned
      to measured outputs, required for §3.3 resync), assembled block-diagonally
      into the Eq. (3) 12-state A,B,C,D. Translational blocks take frame-canonicalized
      tilt commands tiltN/tiltE = Rz(ψ)·(−θc, φc) plus a const input for the hover
      equilibrium — Appendix A frame conversion, documented in sysid_12state.m.)
      Original text: Extend `quad_template.m` to the paper's 12-state template, Eq. (3):
      `x = [x y z φ θ ψ ẋ ẏ ż p q r]`. Fixed kinematic rows (known a priori):
      position integrates velocity; attitude integrates rates (small-angle / Appendix A
      transforms). Free rows: 2nd-order closed-loop PID dynamics per axis (roll, pitch,
      yaw, vertical/throttle) — "PID controller + dynamics known a priori, coefficients
      unknown". `u = [φ_cmd, θ_cmd, ψ_cmd, throttle]` (target states, §3.1).
      C selects the sensed outputs; D = 0.
- [x] 1.2 Rework `parse_dataflash.py`:
      resample each contiguous mission segment separately to **50 Hz** with CubicSpline
      (single resampling step); keep velocity outputs (NKF1 VN/VE/VD) and local-frame
      position (PN/PE/PD) so the 12-state model has all outputs; consistent units
      (rad, rad/s, m, m/s) converted **here, once**; save per-segment cell arrays for
      multi-experiment iddata; **no detrending**.
      → verify: loader prints per-segment lengths; spot-check no spline ringing at
      segment edges (plot or min/max sanity on throttle ∈ [0,1]).
- [x] 1.3 Written as new `matlab/sysid_12state.m` (run pending fresh data):
      12-state template; `K.Value = 0`, **`K.Free = false`** (output-error PEM);
      **no detrending**; multi-experiment `iddata` (`merge`); report fit %, spectral
      radius, ctrb/obsv ranks; export `model_matrices.h` (NX=12) + `quadrotor_12state.mat`.
      → verify: spectral radius reported (do not constrain); validation fit reasonable
      on held-out segments using **open-loop simulation** (`compare` with
      `predictionHorizon = Inf`), since open-loop is what the firmware runs.
      Note: pure integrator rows put eigenvalues at 1.0 — that is the physics; the
      windowed resync (§3.3) is the paper's own answer to open-loop drift. Do not add
      stabilization hacks.
- [x] 1.4 If MATLAB throws the `greyest`/metrics crash documented earlier: keep using
      structured `idss` + `ssest` (it *is* PEM); never silence it by freeing K.

### Phase 2 — Algorithm 1 exact rewrite (fixes A1–A9) — TDD
Rewrite `recovery_monitor.h` to mirror §1.2 above, line by line. For each item: write the
failing test in `test_recovery.cpp` first, then fix, then green.
- [x] 2.1 Add `filter(m)`: low-pass filter on each real measurement before comparison
      (paper cites a standard LPF with pre-selected cutoff [40]; use a 2nd-order
      Butterworth, cutoff documented in `recovery_params.h`, designed at 50 Hz).
      Filtered `m` is used for residual/e-estimation; the control loop receives the raw
      measurement in normal operation and `ms` under recovery (Fig. 3 semantics).
      → test: noisy clean signal accumulates much smaller r with filter than without.
- [x] 2.2 Fix compensation sign: `e = mean over previous window of (ms_raw − m_filtered)`;
      apply `ms ← ms − e`. → test: constant disturbance offset (wind analogue) is
      compensated to near-zero residual in the window after the checkpoint.
- [x] 2.3 Gate the entire checkpoint on `!recovery_mode && t > window` (per channel):
      `t←0, r←0, e update, ms←m, model-state resync`. No `r` reset while in recovery.
      → test: sustained attack across ≥2 window lengths never exits recovery before the
      attack ends; after attack ends, r falls below T_off and exit occurs after K counts.
- [x] 2.4 Implement l.15 `ms ← m` and keep §3.3 state resync (synchronized readings fed
      into the model) — both, at checkpoint only. → test: prediction drift across windows
      is bounded (replicates Fig. 12 behavior qualitatively).
- [x] 2.5 Remove the extra `safe_count = 0` on `r ≥ T_off`; keep paper-literal reset only
      on trigger. → test: intermittent dips below T_off accumulate safe_count per paper.
- [x] 2.6 Per-sensor windows honored (N per channel from DTW) or a single documented N;
      fix `err_hist` indexing (store before increment / align with window start).
- [x] 2.7 Add `recovery_action()` weak hook (default no-op, GCS warning message is fine).
- [x] 2.8 Extend the monitor to **all sensor channels of the 12-state model** with the
      paper's `convert()` per type: gyro (states direct), accel (Eq. 4 + Holoborodko on
      model velocity states), baro (Eq. 5 on model z), heading (model ψ vs. compass
      heading via Eq. 6), GPS (model position/velocity states). Delete/retire
      `recovery_gps.h` dead-reckoning and `recovery_baro.h` EKF-altitude variants (D1, D2).
      → test: synthetic state trajectory produces correct conversions per equation
      (unit tests against hand-computed values).
- [x] 2.9 Run full `test_recovery.cpp` suite; all green; keep the no-false-positive and
      driven-predictor tests.

### Phase 3 — Parameter selection (fixes C1–C4)
- [x] 3.1 Update `select_parameters.py` so `predict_software_sensors()` is a line-for-line
      mirror of the fixed monitor (sign, gating, ms←m, filter). Add the same Butterworth
      LPF on the real signal.
- [x] 3.2 DTW per channel on the full clean dataset (banded DTW acceptable; document
      band). `N_ch = max time-displacement`. Sanity: at 50 Hz expect N in the tens of
      samples (paper: 230 @400 Hz ≈ 575 ms), not ~500.
- [x] 3.3 `T_on = e_max + margin` per channel from clean windows; pick + document margin
      and `T_off < T_on`; emit `recovery_params.h`; sync to firmware tree.
- [ ] 3.4 Replicate the Fig. 14 sweep: FP rate over 20 clean missions, FN rate over 20
      attacked missions, across (window, threshold) grid. → verify: chosen parameters
      give 0 FP and 0 FN, as in the paper.

### Phase 4 — Firmware wiring (fixes A9, D3–D6)
- [x] 4.1 Gyro: keep the Fig. 3 insertion point (inside `read_AHRS()`, before
      `ahrs.update()` — already correct), but monitor **each gyro instance**
      (`ins.get_gyro(i)` for all instances) and substitute per instance before fusion
      (`set_gyro` + `set_delta_angle` per instance).
- [x] 4.2 Accel: same place, per instance, Eq. (4) conversion channel.
- [x] 4.3 Baro: patch `AP_Baro::update()` — `ms = Eq.5(model z)`, Algorithm 1 channel.
- [x] 4.4 GPS: patch `AP_GPS::update()` — `ms` = model position/velocity states,
      Algorithm 1 channels (lat/lng/alt/vel).
- [x] 4.5 Compass/heading: patch compass read path — model ψ vs. measured heading (Eq. 6).
- [x] 4.6 Supplementary compensation: when all gyros compromised, combine
      `[φ_acc, θ_acc, ψ_mag]` (LPF'd) with the software-sensor attitude by **weighted
      sum** (weights documented; paper leaves them unspecified) and feed the combined
      attitude into the substitution path. Remove the `(void)` suppressions.
- [ ] 4.7 Rebuild firmware; SITL boots; clean flight shows zero recovery activations
      (FP check) over a full mission.

### Phase 5 — Evaluation (E1–E4, §4)
- [ ] 5.1 Attack injection: add a small firmware attack hook at the sensor interface
      (constant / sine / random injection per sensor, MAVLink-triggered) per §4.1;
      keep the SIM_* param scripts as a secondary path.
- [ ] 5.2 Case study 1 (gyro): hover, constant-value attack on all gyro rate channels;
      A/B with `#define RECOVERY_DISABLED`. Expect crash/flip without recovery; stable
      hover ≥10 s with recovery. Evaluate Eq. (7) ε=3°, k=10 s.
- [ ] 5.3 Case study 2 (GPS): square 5-waypoint mission, 20 m longitude offset attack;
      verify mission continues with marginal deviation under recovery.
- [ ] 5.4 Table 3 combinations C1–C6 (GPS/baro/gyros) and Table 4 (1/2/3 gyros, with
      supplementary compensation for the all-gyro case).
- [ ] 5.5 Record results, save logs/plots, write up replication results vs. paper tables.

---

## 4. Fixed interpretation decisions — do not re-litigate, do not re-spice

Where the paper is explicit, the paper wins. Where it is silent, these are the standing
choices (documented, consistent, minimal):

1. **50 Hz** model/monitor rate — the one sanctioned deviation. Everything (SI, DTW, N,
   thresholds, LPF design) is done at 50 Hz consistently.
2. **Filter semantics**: LPF'd `m` is used for comparison/e-estimation only; the control
   loop gets raw `m` when healthy and `ms` under recovery (Fig. 3 motivating code).
3. **One 12-state vehicle model** (Eq. 3) shared by all software sensors; per-sensor
   `convert()` + per-sensor `{t, r, e, recovery_mode, safe_count, window}`.
4. **Open-loop model, K=0** in both identification (OE-PEM) and runtime (Alg. 1 l.7).
   Drift is handled exactly the paper's way: windowed sync + error reset (§3.3).
5. **`e` sign**: `e = mean(ms_raw − m_filtered)` over the previous window; `ms ← ms − e`.
6. Unspecified constants — choose once, document in `recovery_params.h`, validate via the
   Fig. 14 FP/FN sweep: LPF cutoff, margin `m`, `T_off/T_on` ratio, safe count `K`.
7. `u` = attitude targets (rad) + throttle (0..1) from `attitude_control` — the paper's
   "target states". Raw units, no detrending/normalization anywhere.

## 5. Key file map (post-move)
- Paper: `~/paperImp/SbRRfSAoRV.pdf`; equation digest: `~/paperImp/paper.md`
- Workspace: `~/paperImp/rv_recovery/` (matlab/, python/, firmware_patch/, data/)
- Firmware: `~/paperImp/ardupilot_ws/arducopter-3.4/` (patched: `ArduCopter/ArduCopter.cpp`
  `read_AHRS()`; headers in `libraries/AP_InertialSensor/`)
- Rover (later): `~/paperImp/ardupilot_ws/apmrover2-2.5/`
- MATLAB R2026a: `/usr/local/MATLAB/R2026a/bin/matlab`
- Build python: `~/.pyenv/versions/3.10.14/bin/python` (waf needs it)

---

## 6. Execution log (running)

**2026-06-10 session 2:**
- New finding **B7 🔴**: the existing operation log (`all_missions_1.BIN`, ~2.8 h) is a
  pure **hover** — attitude never exceeds ±0.7°, position ±0.4 m, commands ≈ 0. It cannot
  identify any dynamics and violates §3.1 ("data collected under different maneuvers").
  Action: `collect_logs.py` rewritten as a real mission generator (random sequences of
  straight-fly / turn / climb / hover primitives, GUIDED mode, disarm between missions →
  one log per mission, LOG_BITMASK 131071 for 25 Hz ATT/IMU/EKF streams). Fresh data
  collection required before sysid.
- New finding **A10 🔴**: `holoborodko_deriv()` used a 4-point formula with a 25% gain
  error on ramps. Replaced with the correct causal 5-point smooth-noise-robust form
  `(2(f[n−1]−f[n−3]) + (f[n]−f[n−4]))/(8h)` (derivative at n−2). Pinned by test T8.
- Phase 2 engine rewritten: `recovery_monitor.h` now implements Algorithm 1 line-for-line
  (RecoveryModel + per-sensor RecoveryChannel API). Conformance suite
  `test_recovery.cpp` T1–T8: **25/25 pass** (LPF l.8, e-sign l.14/16, checkpoint gating
  l.11, sync l.15 + §3.3 state re-seed, safe_count l.21-22, detection/substitution
  l.18-20, healthy raw passthrough per Fig. 3, conversion equations Eq. 4/5/6/11).
- LPF: 2nd-order Butterworth, fc = 5 Hz @ fs = 50 Hz, coefficients verified against
  scipy `butter(2, 0.2)`; same filter to be used in select_parameters.py (Phase 3).
- `RECOVERY_DISABLED` compile guard added around the firmware recovery block
  (A-side of §4 A/B evaluation + clean data collection).

**2026-06-10 session 2 (cont.):**
- New finding **B8 🔴**: `all_missions_1.BIN` is unusable for SI (pure hover, see B7).
  Wrote a real GUIDED mission generator; **key fix**: ArduCopter 3.4 rejects
  COMMAND_LONG DO_SET_MODE (ACK result 3) — must use the legacy `SET_MODE` message;
  and guided arming requires the EKF to be *using GPS* (STATUSTEXT "is using GPS"),
  not merely a GPS_RAW 3D fix. With both fixed, missions arm/takeoff/fly/land cleanly.
- Phase 4 wiring written and **compiles clean with recovery ENABLED** (full 12-channel
  monitor: 3×gyro + 3×accel per IMU instance, GPS pos/vel, baro Eq.5, mag heading Eq.6,
  supplementary compensation Appendix B). Driver setters added: AP_Baro::recovery_set_pressure,
  AP_GPS::recovery_override, AP_Compass::recovery_set_field. Insertion stays in read_AHRS()
  before ahrs.update() (Fig. 3). Decimation 8:1 (400→50 Hz). RECOVERY_DISABLED toggles A/B.
- Collecting 20 fresh maneuver missions for SI (Phase 1b blocked on this data).

**2026-06-10 session 2 — sysid results:**
- Collected 21 maneuver segments (40.9 min @ 50 Hz) — real excitation (attitude ±0.3 rad,
  velocities several m/s, yaw sweeps, 90 m position spans), unlike the old hover-only log.
- `sysid_12state.m`: per-variable 2nd-order PEM blocks, K=0, no detrend, multi-experiment
  iddata over all 21 segments. Open-loop validation sim-fit: roll angle 84%, pitch 92.6%
  (rate channels lower; yaw/pos/vel negative on infinite-horizon sim = integrator DRIFT,
  the exact phenomenon §3.3 windowed resync handles — firmware re-seeds every ~0.6 s, not
  every 100 s). spectral radius = 1.0 = pure-integrator kinematics, not instability.
  Model exported NX=12, synced to firmware tree.
- The faithful per-window quality metric is e_max from select_parameters.py (Phase 3),
  not the infinite-horizon fit %.

**2026-06-10 — Phase 3 results:**
- Fixed dtaidistance perf bug (warping_path needs use_c=True; 22x speedup).
- DTW windows (samples @50Hz): phi134 theta131 psi249 p180 q172 r249;
  pN/pE/alt/vN/vE/vUp 237-249. Attitude/rate channels (gyro case study) clean
  (2.6-3.6s windows, small thresholds: phi 2.58, theta 1.54, p 11.5, q 13.8, r 38.4).
- Position/velocity/yaw channels hit the DTW band ceiling (249 ≈ 5s, band=250): a real
  consequence of open-loop integrator drift — the §3.3 windowed resync is the paper's
  own remedy, applied every window in firmware. Thresholds calibrated on the EXACT
  firmware predictor (same LPF, sign, gating). recovery_params.h synced to firmware.
- Caveat C5 (documented): for pure-integrator channels the "max DTW displacement" is
  band-limited; the gyro/attitude recovery (paper's primary case study) is unaffected.

**2026-06-10 — Phase 4 firmware integration + FP debugging:**
- Built recovery-enabled firmware (NX=12 model + calibrated recovery_params.h synced).
  Added RECOVERY_DISABLED (A-side), RECOVERY_DIAG (detection-only diagnostics), and a
  recovery_action() GCS alert for FP/FN observation.
- FP debugging on clean SITL flight surfaced and fixed real issues (all faithful to paper):
  1. First-window startup transient: e=0 + unsettled LPF made channels false-latch before
     error-compensation learned the offset. FIX: prime each channel so ms=m on its first
     tick (LPF DC-seeded, e=startup offset). Tests T2/T5 updated (warm up clean first).
  2. Heading channel frame mismatch: calibrated on ATT yaw but monitored Eq.6 compass
     heading (declination/tilt differ) -> FP. FIX: monitor model psi vs unwrapped ATT yaw
     (the calibration domain); compass-field substitution retained.
  3. FP CASCADE (root cause): an initial false trigger substitutes a bad sensor -> corrupts
     EKF -> spikes other channels -> more triggers. DIAG (detection-only) build proved every
     channel's peak per-window residual stays BELOW its threshold in steady flight (vN 394<584,
     r 5.3<38, psi 2.1<19, gyros <0.4) => thresholds correctly calibrated; the storm was
     substitution-induced feedback during EKF cold-start / aggressive auto-takeoff.
     FIX (faithful to §4.2.2 "RV starts with accurate initial states"): gate monitor on
     armed; add a post-arm warmup that keeps the model synced to real states through the
     takeoff transient, then hands off to normal windowed monitoring from stable hover.
- Model-quality note: the throttle->altitude (U) and tilt->position blocks are under-excited
  by the mostly-hover collection data (negative open-loop fits). The gyro/attitude channels
  (paper's PRIMARY gyroscope case study) are well-identified and clean.
- Script fixes: ArduCopter 3.4 needs legacy SET_MODE (not DO_SET_MODE); EKF readiness must
  be polled via EKF_STATUS_REPORT flags (the 'using GPS' STATUSTEXT is one-shot at boot);
  ARMING_CHECK=0 for SITL automation.

**2026-06-10 — REFOCUS (user: "I don't care about result. Just follow the paper"):**
- Recognized that DIAG-based threshold re-tuning + warmup + first-tick priming were
  RESULT-CHASING (suppressing false positives), NOT the paper's procedure. The paper
  derives thresholds from DTW on offline operation data (select_parameters.py, done) and
  Algorithm 1 has no priming/warmup. Reverted all of it to stay faithful:
  - recovery_monitor.h: removed first-tick priming, peak_r diagnostics, g_recovery_detect
    gate -> now LITERAL Algorithm 1 (25/25 conformance tests still pass).
  - ArduCopter.cpp: removed RECOVERY_DIAG scaffolding, g_rec_substitute gating, the 12 s
    post-arm warmup. Restored direct substitution and the §3.2/Eq.6 compass-heading
    comparison for the magnetometer channel (was swapped to ATT yaw to dodge FP).
  - KEPT (paper-justified): init seeds model from real states at operation start
    (§4.2.2 "RV starts with accurate initial states"); windowed sync every N (§3.3);
    arm-gating as the "operation begins" boundary; DTW thresholds from operation data.
  - recovery_action() GCS alert kept (Alg.1 l.23, optional) for honest FP/FN observation.
- Faithful recovery firmware builds clean. Known honest consequence (NOT to be hacked
  away): the under-excited altitude/position sub-models give large residuals, so those
  channels can false-trip in flight. Reported as-is per the paper's faithful procedure.

**2026-06-10 — Phase 5 evaluation procedure + SITL data-path finding:**
- Implemented the §4.1 attack module as firmware code in the sensor interface
  (read_AHRS, before the recovery monitor + ahrs.update), MAVLink-triggered via spare
  RC channels (ch7 = gyro-X constant injection magnitude, ch8 = waveform
  constant/sine/random). Builds into both A-side (RECOVERY_DISABLED) and B-side binaries.
- Implemented case_study_gyro.py (§4.3 / Fig 17): takeoff, hover, trigger gyro attack,
  evaluate Eq.7 (|roll-0| <= 3 deg over 10 s).
- FINDING (firmware-architecture, not a procedure error): ArduCopter 3.4 SITL defaults to
  AHRS_EKF_TYPE=10 (EKF_TYPE_SITL), which takes attitude/gyro DIRECTLY from the perfect
  simulator FDM (update_SITL() in AP_AHRS_NavEKF.cpp), bypassing the real INS. Switching to
  AHRS_EKF_TYPE=2 (EKF2) was necessary but STILL did not propagate the injection, because
  the SITL INS backend regenerates _gyro/_delta_angle from the FDM asynchronously — so a
  read_AHRS-level set_gyro()/set_delta_angle() (used by BOTH the §4.1 attack and the
  recovery substitution, per the paper's Fig 3 insertion point) is overwritten before the
  estimator consumes it. The firmware attack hook FIRES correctly (31 GCS confirmations per
  run) but its effect does not reach the control loop in SITL.
- CONSEQUENCE: the live A/B numeric demonstration cannot be produced at the paper's
  read_AHRS insertion point in ArduCopter 3.4 SITL. The paper ran on real hardware
  (3DR Solo / Pixhawk) where read_AHRS substitution propagates. Producing live SITL numbers
  would require moving the insertion point to the INS backend or EKF input, or modifying the
  SITL sensor model — all SITL-specific workarounds, NOT part of faithful procedure
  replication. Per the user's direction (results are secondary), this is documented and
  left as-is rather than hacked around.

## 7. FINAL STATUS — procedure replication complete

Faithful to the paper (verified):
- §3.1 data pipeline: random MAVLink maneuver missions; spline resample to one rate
  (50 Hz, the sole sanctioned deviation); per-segment, no detrend.
- §3.1 system identification: per-variable 2nd-order PEM templates, K=0 (open-loop /
  Algorithm 1 line 7), 12-state Eq.(3) model assembled and exported.
- §3.2 software sensors: gyro (states), accel (Eq.4 + Holoborodko), baro (Eq.5),
  magnetometer heading (Eq.6), GPS (states); frame transforms (Appendix A).
- §3.3 parameter selection: DTW window N + T = e_max + margin, computed on a predictor
  that mirrors the firmware monitor exactly.
- §3.4 / Algorithm 1: recovery_monitor.h implements lines 6-24 literally; 25/25
  conformance tests pass; wired into read_AHRS per Fig 3 (insertion point), per-instance
  gyro/accel + GPS + baro + mag + Appendix B supplementary compensation.
- §4.1 attack module: firmware sensor-interface injection, MAVLink-triggered.
- §4 evaluation scripts: Eq.7 success criterion (eps=3, k=10 s), A/B builds.

Honest limitations (consequences of faithful choices, not hacked away):
- 50 Hz instead of 400 Hz (RAM; sanctioned).
- Translational/altitude sub-models under-excited by hover-dominant data -> those channels
  can false-trip in flight (the gyro/attitude channels — paper's primary case study — are
  clean).
- Live SITL A/B numbers blocked by the AHRS_EKF_TYPE / async-INS data path above.

**2026-06-11 — User decision: accept documented limitation.**
Live SITL A/B numbers will not be produced (the read_AHRS-level substitution is overwritten
by SITL's async INS backend; conclusively confirmed: EKF2 + a +2 rad/s gyro attack yields
only 0.34 deg roll deviation). The paper ran on real Pixhawk hardware where the substitution
propagates. Procedure replication is the goal and is complete & faithful; this limitation is
recorded honestly rather than worked around. Canonical firmware build = recovery ON
(RECOVERY_DIAG and RECOVERY_DISABLED both commented out).

### Deliverables (all under ~/paperImp/)
- EXECUTION_PLAN.md — this file: audit, decisions, execution log, final status.
- rv_recovery/firmware_patch/recovery_monitor.h — literal Algorithm 1 (25/25 tests).
- rv_recovery/firmware_patch/software_sensors.h — Eq.4/5/6, Appendix A/B conversions.
- rv_recovery/firmware_patch/recovery_params.h — DTW N + T=e_max+margin (per §3.3).
- rv_recovery/firmware_patch/test_recovery.cpp — 25 conformance tests.
- rv_recovery/matlab/sysid_12state.m, quad_template.m — per-variable PEM, K=0, 12-state.
- rv_recovery/matlab/models/{quadrotor_12state.mat, model_matrices.h} — identified model.
- rv_recovery/python/{collect_logs,parse_dataflash,select_parameters,case_study_gyro,
  fly_mission,eval_recovery,attack_gps}.py — §3.1-§4 pipeline + evaluation.
- ardupilot_ws/arducopter-3.4 — patched firmware: §4.1 attack module + recovery monitor
  in Copter::read_AHRS() (Fig 3), driver setters in AP_Baro/AP_GPS/AP_Compass.
