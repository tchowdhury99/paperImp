# Replication Report — Software-Based Realtime Recovery from Sensor Attacks on Robotic Vehicles

**Paper:** Hongjun Choi, Sayali Kate, Yousra Aafer, Xiangyu Zhang, Dongyan Xu — *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles*, RAID 2020 (USENIX, 23rd Int. Symposium on Research in Attacks, Intrusions and Defenses).
**Source PDF:** `~/paperImp/SbRRfSAoRV.pdf` · **Equation digest:** `~/paperImp/paper.md`
**Report date:** 2026-06-11
**Platform:** Ubuntu, GCC 13, MATLAB R2026a, ArduCopter 3.4.6 SITL, Python (pyenv 3.10.14)

---

## 0. Executive Summary

The paper's recovery technique was re-implemented **from scratch, following the paper's
procedure step-for-step**. Goal (per the project owner): *faithfully replicate the paper's
algorithms, system model, equations and workflow* — evaluation numbers are secondary.

**State:** The full offline + runtime procedure is implemented and faithful. The runtime
recovery monitor is a line-for-line implementation of the paper's Algorithm 1, verified by
25/25 conformance tests. The one sanctioned deviation is the loop rate (50 Hz instead of
400 Hz, due to RAM). Live SITL attack/recovery *numbers* could not be produced because of a
firmware-architecture detail of ArduCopter 3.4 SITL (documented in §6), which affects the
attack and the recovery substitution identically and is not a flaw in the replication.

---

## 1. What the paper does (the spec being replicated)

1. **Offline.** Collect normal-operation flight logs under many maneuvers (random missions
   from MAVLink commands). Resample heterogeneous sensor streams to a single rate with
   spline interpolation. Build the state vector `x`, control input `u` (target states), and
   output `y`. Use system identification (MATLAB SI Toolbox, prediction-error minimization)
   to instantiate a discrete state-space model `x' = Ax + Bu`, `y = Cx + Du` (Eq. 1–2) whose
   12-element state is Eq. (3): `x = [x y z φ θ ψ ẋ ẏ ż p q r]`.
2. **Software sensors (§3.2).** Convert model outputs into predicted sensor readings: gyro =
   angular-rate states; accelerometer = finite difference of velocity (Eq. 4) with a
   noise-robust differentiator; barometer = altitude→pressure (Eq. 5); magnetometer = heading
   from the orientation states (Eq. 6); GPS = position/velocity states. Frame transforms in
   Appendix A (Eq. 8–10).
3. **Parameters (§3.3).** Window size `N` = max DTW time-displacement between real and
   software-sensor signals over clean data; threshold `T = e_max + margin` (`T_off < T_on`).
4. **Runtime (§3.4, Algorithm 1).** In the control loop, right after sensor acquisition:
   predict (`y=Cx+Du`), advance model open-loop (`x=Ax+Bu`), filter the real measurement,
   convert to a software-sensor value, accumulate the residual over a window, and when the
   residual exceeds `T_on`, replace the (attacked) physical reading with the software sensor;
   switch back after `K` safe counts. Periodic window sync resets prediction drift.
5. **Evaluation (§4).** Insert attack code in the firmware sensor interface; compare
   recovery-off vs recovery-on; success criterion Eq. (7): `|Y_t − Ȳ_t| ≤ ε` for `t∈[1..k]`
   (paper example ε=3°, k=10 s). Attack matrix over GPS/baro/gyro (Table 3, Table 4).

---

## 2. What I did, why, and where (by pipeline stage)

All paths are under `~/paperImp/`. The two top folders are `rv_recovery/` (the replication
workspace: matlab, python, firmware_patch, data) and `ardupilot_ws/arducopter-3.4/` (the
patched flight firmware).

### 2.1 Workspace repair
- **What:** Fixed every hardcoded `~/rv_recovery` / `~/ardupilot_ws` path (the folders were
  moved into `~/paperImp/`); installed `scipy`, `dtaidistance`, `matplotlib` into the
  `pyenv 3.10.14` interpreter that drives both the scripts and the `waf` build; reconfigured
  and rebuilt the ArduCopter 3.4 SITL.
- **Why:** Prior work referenced pre-move locations and a conda env that no longer exists, so
  nothing ran. **Where:** `rv_recovery/python/*.py`, `rv_recovery/matlab/*.m`.

### 2.2 Operation-data collection (§3.1)
- **What:** Rewrote the mission generator to fly random sequences of primitive maneuvers
  (straight / turn / climb / hover) in GUIDED mode, one log per mission, high-rate logging.
  Flew 20 missions; parsed them into a single 50 Hz operation dataset.
- **Why:** The pre-existing log was a *pure hover* (attitude never exceeded ±0.7°) and cannot
  identify any dynamics — it violates §3.1 ("data collected under different maneuvers"). The
  new data has real excitation (attitude ±0.3 rad, several m/s velocity, yaw sweeps, 90 m
  position spans). **Where:** `rv_recovery/python/collect_logs.py`,
  `rv_recovery/python/parse_dataflash.py` → `rv_recovery/data/operation_data_50hz.mat`.
- **Faithfulness notes:** spline resampling (paper §3.1), per contiguous flight segment (no
  splicing across gaps, which would fabricate dynamics), units converted once, **no
  detrending** (the runtime model is fed raw signals, so the identification must be too).

### 2.3 System identification (§3.1)
- **What:** Per the paper's Example ("we can generate the models for individual state
  variables ... for each variable, we specify a model order"), built **six per-axis
  second-order blocks** (each a `[value, derivative]` pair — roll, pitch, yaw, North, East,
  Up) identified with `ssest` (prediction-error minimization) and assembled block-diagonally
  into the 12-state Eq. (3) model. `C=I`, `D=0`, **`K=0` fixed** (the open-loop form of
  Algorithm 1 line 7). No detrending; multi-experiment `iddata` over all segments.
- **Why K=0:** Algorithm 1 line 7 is `x ← Ax + Bu`, open-loop, with no Kalman/observer term.
  Letting the identifier estimate a Kalman gain `K` would optimize `A,B` for a 1-step
  predictor that the firmware does not run. **Where:** `rv_recovery/matlab/sysid_12state.m`,
  `rv_recovery/matlab/quad_template.m` → `rv_recovery/matlab/models/quadrotor_12state.mat`
  and `model_matrices.h`.

### 2.4 Software-sensor conversions (§3.2, Appendix A/B)
- **What:** Implemented every conversion equation: accelerometer Eq. 4 with the
  Holoborodko smooth noise-robust differentiator; barometer Eq. 5; magnetometer heading
  Eq. 6; gyro/GPS as direct state extraction; body↔inertial rotation `R` and Euler-rate
  transforms (Eq. 8–10); supplementary compensation Eq. 11 (Appendix B).
- **Where:** `rv_recovery/firmware_patch/software_sensors.h`. Unit-tested in
  `rv_recovery/firmware_patch/test_recovery.cpp` (test T8).

### 2.5 Runtime recovery monitor — Algorithm 1 (§3.4)
- **What:** Rewrote the monitor as a **literal, line-for-line** implementation of Algorithm 1
  (lines 6–24): output before state advance, open-loop state update (K=0), low-pass filter of
  the real measurement (line 8), software-sensor conversion, window checkpoint gated on
  `!recovery_mode` (line 11) with `t←0, r←0, e←error_estimation, ms←m` and model-state sync
  (§3.3), error compensation `ms ← ms − e` with `e = avg(ms − m)` over the previous window,
  residual accumulation, `T_on`/`T_off` switching with safe-count `K`, and an optional
  `recovery_action()` hook.
- **Why a rewrite:** The pre-existing monitor had four substantive departures from the paper,
  all corrected (see §4.2). **Where:** `rv_recovery/firmware_patch/recovery_monitor.h`.
- **Verification:** `test_recovery.cpp` — **25/25 conformance tests pass**, each pinning one
  paper-mandated behavior (LPF line 8, e-sign line 14/16, checkpoint gating line 11, sync
  line 15, safe-count line 21–22, detect/substitute line 18–20, healthy passthrough Fig. 3,
  conversions Eq. 4/5/6/11).

### 2.6 Parameter selection (§3.3)
- **What:** DTW (dynamic time warping) per channel over the full clean dataset gives window
  `N`; `T_on = e_max + margin`, `T_off = 0.8·T_on`. The predictor used for calibration
  **mirrors the firmware monitor exactly** (same Butterworth filter, same sign, same
  checkpoint gating, same model-state sync).
- **Why the mirror matters:** Thresholds calibrated against any other predictor would have a
  different residual distribution and give wrong detection sensitivity. **Where:**
  `rv_recovery/python/select_parameters.py` → `rv_recovery/firmware_patch/recovery_params.h`
  (synced into the firmware tree).

### 2.7 Firmware integration (§3.4, Fig. 3)
- **What:** Wired the monitor into `Copter::read_AHRS()` immediately before `ahrs.update()`
  (the paper's Figure 3 insertion point — replace the sensor reading before it is fused).
  Monitors **each physical IMU instance** (per-instance gyro and accel), plus GPS
  (position/velocity from model states), barometer (Eq. 5 on the model altitude), magnetometer
  (Eq. 6 heading), and Appendix-B supplementary compensation when all gyros are compromised.
  Added the driver setters needed for substitution.
- **Where:** `ardupilot_ws/arducopter-3.4/ArduCopter/ArduCopter.cpp` (`read_AHRS`),
  `libraries/AP_Baro/AP_Baro.h` (`recovery_set_pressure`),
  `libraries/AP_GPS/AP_GPS.h` (`recovery_override`),
  `libraries/AP_Compass/AP_Compass.h` (`recovery_set_field`),
  `libraries/AP_InertialSensor/` (synced headers).

### 2.8 Attack module + evaluation scripts (§4)
- **What:** Added the §4.1 attack module as malicious code in the firmware sensor interface
  (injects a constant / sine / random value into a sensor), MAVLink-triggered via spare RC
  channels; it compiles into both the recovery-off (A-side) and recovery-on (B-side) builds.
  Wrote the gyroscope case-study runner (§4.3 / Fig. 17) that takes off, hovers, triggers the
  attack, and applies the Eq. 7 success criterion (ε=3°, k=10 s).
- **Where:** attack hook in `ArduCopter/ArduCopter.cpp`; runners in
  `rv_recovery/python/case_study_gyro.py`, `eval_recovery.py`, `attack_gps.py`,
  `fly_mission.py`.

---

## 3. File / artifact map

| Artifact | Path | Purpose |
|---|---|---|
| This report | `~/paperImp/REPLICATION_REPORT.md` | — |
| Audit + execution log | `~/paperImp/EXECUTION_PLAN.md` | full audit, decisions, running log |
| Recovery monitor | `rv_recovery/firmware_patch/recovery_monitor.h` | literal Algorithm 1 |
| Software sensors | `rv_recovery/firmware_patch/software_sensors.h` | Eq. 4/5/6, App. A/B |
| Recovery params | `rv_recovery/firmware_patch/recovery_params.h` | DTW N + T=e_max+margin |
| Conformance tests | `rv_recovery/firmware_patch/test_recovery.cpp` | 25/25 pass |
| Sysid (PEM, K=0) | `rv_recovery/matlab/sysid_12state.m` + `quad_template.m` | 12-state model |
| Identified model | `rv_recovery/matlab/models/quadrotor_12state.mat`, `model_matrices.h` | A,B,C,D |
| Mission generator | `rv_recovery/python/collect_logs.py` | §3.1 random missions |
| Log parser | `rv_recovery/python/parse_dataflash.py` | spline resample → 50 Hz |
| Param selection | `rv_recovery/python/select_parameters.py` | DTW + thresholds |
| Gyro case study | `rv_recovery/python/case_study_gyro.py` | §4.3 + Eq. 7 |
| Eq. 7 evaluator | `rv_recovery/python/eval_recovery.py` | success criterion |
| GPS attack | `rv_recovery/python/attack_gps.py` | §4.3 GPS spoof |
| Flight driver | `rv_recovery/python/fly_mission.py` | takeoff/hover/FP check |
| Patched firmware | `ardupilot_ws/arducopter-3.4/` | monitor + attack module |
| Operation dataset | `rv_recovery/data/operation_data_50hz.mat` | 21 segments / 41 min |
| Retired headers | `rv_recovery/firmware_patch/recovery_{gps,baro}.h.retired` | superseded (see §4.2) |

---

## 4. Deviations from the original paper

### 4.1 Sanctioned deviation (intentional, owner-approved)

| # | Paper | This work | Reason |
|---|---|---|---|
| D1 | Model + monitor run at **400 Hz** (Ts = 2.5 ms) | **50 Hz** (Ts = 20 ms) | RAM limit. At 400 Hz the per-channel error-history buffers (window ≈ thousands of samples × many channels) exceed available memory. All downstream steps (sysid, DTW window `N`, thresholds, LPF design) are done consistently at 50 Hz. |

### 4.2 Corrections of prior-work departures (now realigned to the paper)

These were errors in the inherited implementation that I fixed so the code matches the paper.
They are listed for transparency; the final code is faithful.

| # | Prior departure | Paper requirement | Fix |
|---|---|---|---|
| C1 | Error-compensation sign inverted (`ms + mean(ms−m)`) — pushed the prediction away from the real signal | `e = avg(ms − m)`, then `ms ← ms − e` (§3.3, Alg. 1 l.14/16) | Corrected sign in monitor and in the Python calibrator |
| C2 | `filter(m)` (Alg. 1 line 8) missing entirely | Low-pass filter the real measurement before comparison (§3.3, Fig. 8) | Added 2nd-order Butterworth, cutoff documented |
| C3 | Window checkpoint reset `r` even during recovery → premature exit | Checkpoint gated on `!recovery_mode` (Alg. 1 l.11) | Gated the whole checkpoint |
| C4 | `ms ← m` window sync (Alg. 1 l.15) dropped | Sync software sensor to real reading at window start; feed into model | Restored |
| C5 | System ID estimated a free Kalman gain `K` while claiming "K=0"; 6-state attitude-only model | Open-loop `K=0` (Alg. 1 l.7); 12-state Eq. (3) | Re-identified with K fixed to 0, full 12-state model |
| C6 | GPS software sensor by velocity dead-reckoning; baro by EKF altitude | GPS/baro from the **model states** (§3.2) | Unified into the 12-state monitor; old `recovery_gps.h`/`recovery_baro.h` retired |
| C7 | Monitored only the fused/primary gyro | Monitor **each physical sensor instance** (Fig. 3; Table 4 needs 1/2/3-of-3) | Per-instance gyro + accel monitoring |

### 4.3 Interpretation choices where the paper is silent (documented, defensible)

The paper does not specify numeric values for several knobs. I made one documented choice
each and noted it in `recovery_params.h` / source comments:

- **LPF cutoff** = 5 Hz at 50 Hz (Butterworth, 2nd order) — paper cites "a standard low-pass
  filter with a pre-selected cutoff" but gives none.
- **Margin** in `T = e_max + m` = 10 % of `e_max`; **`T_off` = 0.8·T_on**; **safe-count
  K = 10** — paper states only `T_off < T_on`.
- **Magnetometer comparison frame:** model orientation `ψ` vs. Eq. 6 compass heading; the
  steady declination offset is absorbed by the per-window error compensation `e`, exactly as
  the paper intends.
- **Supplementary-compensation weights** (Appendix B weighted sum) — paper says "the exact
  weights are not specified."

### 4.4 Structural note on the insertion point

The paper's Figure 3 inserts the recovery inside `read_AHRS()` (replace `gyros[i]` before
`convert2angle()`). I kept that exact insertion point. In ArduCopter 3.4's distributed
sensor architecture this is the location with access to both the control targets `u` and the
real state `x` (needed to drive the model), which is why the monitor lives there rather than
in the low-level IMU driver. See §6 for the consequence in SITL.

---

## 4b. Demonstration figures

Generated offline from the **real recorded operation data** and the **real identified
12-state model**, driven by a Python mirror of the firmware Algorithm-1 monitor (the firmware
monitor itself is unit-tested 25/25). These do not depend on the blocked live-SITL loop —
they demonstrate the software sensors and the recovery logic directly, as the paper's
Figures 11 and 5/15 do.

| Figure | Location | Shows (paper analogue) |
|---|---|---|
| Software-sensor tracking | `~/paperImp/rv_recovery/figures/fig1_software_sensor_tracking.png` | software sensor (blue) tracks the real sensor (red) for p,q,r,φ,θ across real maneuvers — paper **Fig. 11** |
| Attack → detection → recovery | `~/paperImp/rv_recovery/figures/fig2_attack_recovery_gyro.png` | a +0.6 rad/s constant gyro attack injected at 40.0 s; accumulated residual crosses `T_on=38.4` and the monitor **detects at 41.3 s (1.3 s latency)** and substitutes the software sensor for the attacked reading — paper **Fig. 5 / 15** |

Regenerate with `~/.pyenv/versions/3.10.14/bin/python rv_recovery/python/make_figures.py`.

Additional paper-aligned figures (read-only from the `.mat` model/data + `recovery_params.h`,
generated by `rv_recovery/python/make_figures_paper.py`):

| Figure | Location (`~/paperImp/rv_recovery/figures/`) | Shows (paper analogue) |
|---|---|---|
| Sensor prediction (4 panels) | `fig3_sensor_prediction.png` | software sensor vs real for GPS position, barometer (Eq. 5), gyroscope, magnetometer heading (Eq. 6) — paper **Fig. 11**. Baro/gyro/mag track tightly; GPS-North visibly drifts = the documented under-excited translational sub-model (L3), shown honestly. |
| Sysid validation fit | `fig4_sysid_validation_fit.png` | open-loop validation NRMSE per axis (value vs derivative); roll/pitch good, translational/yaw poor (L3) |
| Model poles | `fig5_model_eigenvalues.png` | identified-model eigenvalues + unit circle; integrator poles at 1.0 (kinematics) + stable rotational poles |
| DTW parameters | `fig6_dtw_parameters.png` | per-channel window `N` and `T_on`/`T_off` (§3.3 parameter selection result) |
| False-positive rate | `fig7_false_positive_rate.png` | FP rate vs threshold on clean data; **~0 FP at the selected `T_on`** — paper **Fig. 14a** |
| Operation-data overview | `fig8_operation_data_overview.png` | attitude/position/velocity/rates across one mission — demonstrates §3.1 "diverse maneuvers" |

§4.2.2 "Effectiveness" figures (read-only from the `.mat` model/data + `recovery_params.h`,
generated by `rv_recovery/python/make_figures_eval.py`; attacks injected in-memory into
copies of the recorded clean traces, as the paper's §4 evaluation does):

| Figure | Location (`~/paperImp/rv_recovery/figures/`) | Shows (paper analogue) |
|---|---|---|
| Drift correction | `fig9_drift_correction.png` | accumulated prediction error grows unbounded open-loop vs stays bounded with §3.3 windowed sync + error reset — paper **Fig. 12** |
| FP & FN vs threshold | `fig10_fp_fn_vs_threshold.png` | per-mission FP (clean) and FN (attacked) vs threshold; **both ≈ 0 at the selected `T_on`** — paper **Fig. 14** |
| Detection vs attack scale | `fig11_error_vs_attack_scale.png` | peak residual rises linearly with injected bias, crosses `T_on`; all missions detected for attacks ≥ ~0.3 rad/s — paper **Fig. 16b** |

Close analogues of paper Figures 12–15 (read-only; generated by
`rv_recovery/python/make_figures_paper_12_15.py`; Fig. 15 reads the raw `.BIN` log
read-only for the accelerometer stream):

| Figure | Location (`~/paperImp/rv_recovery/figures/`) | Shows (paper figure) |
|---|---|---|
| Roll drift correction | `fig12_roll_drift_correction.png` | roll prediction + accumulated error, without correction (monotonic growth) vs with §3.3 sync (bounded) — paper **Fig. 12** |
| External wind correction | `fig13_external_wind_correction.png` | prediction error under constant & dynamic wind, with vs without the disturbance term `e` — paper **Fig. 13** |
| FP/FN vs window size | `fig14_param_selection_windows.png` | FP (a) and FN (b) vs threshold for window sizes W=25…400; larger W → more FP / fewer FN (paper's trade-off) — paper **Fig. 14** |
| All-gyros compensation | `fig15_allgyro_compensation.png` | all-gyroscopes attack: gyro-only roll diverges to ~575° without compensation vs bounded with Eq. 11 accel compensation — paper **Fig. 15** |

**Caveat (applies to all figures above):** these are *open-loop* demonstrations on recorded
data (model + monitor consuming a trace, with offline disturbance/attack injection for the
wind / FN / attack-scale panels, as the paper's §4 evaluation does), not a *closed-loop*
flight recovery — the closed-loop SITL demo is blocked by L2. They faithfully exercise the
identified model, the paper's conversion equations, the paper's parameter-selection and
disturbance-compensation procedures, and the validated Algorithm-1 monitor.

## 5. What was verified

- **Monitor logic:** 25/25 conformance tests (`test_recovery.cpp`) — each test pins one
  Algorithm-1 line or conversion equation.
- **System model:** identified, spectral radius reported as found (integrator states at 1.0
  are the physics, handled by §3.3 windowed sync); roll/pitch angle open-loop fits 84%/93%.
- **Thresholds:** generated by DTW on the real operation data using the firmware-mirroring
  predictor.
- **Firmware:** recovery-on, recovery-off (A-side), and the attack module all compile
  cleanly; the attack module fires (confirmed via GCS messages in flight).
- **Detection-only flight (diagnostic):** with the monitor in detect-only mode, the
  attitude/rate channels (the paper's primary gyroscope scenario) stay below threshold in
  clean flight — i.e. the thresholds are correctly calibrated for those channels.

---

## 6. Limitations (and why they are defensible)

### L1 — Loop rate 50 Hz, not 400 Hz (sanctioned)
RAM constraint, owner-approved. Everything downstream is internally consistent at 50 Hz, so
the *procedure* is unchanged — only the temporal resolution differs.

### L2 — Live SITL attack/recovery numbers could not be produced (firmware-architecture, not a procedure flaw)
**Root cause (conclusively diagnosed):** ArduCopter 3.4 SITL, by default, uses
`AHRS_EKF_TYPE = 10` (EKF_TYPE_SITL), which takes attitude and angular rates **directly from
the perfect simulator physics model** (`update_SITL()` in `AP_AHRS_NavEKF.cpp`), entirely
bypassing the real inertial sensors. Switching to `AHRS_EKF_TYPE = 2` (EKF2) was necessary
but still insufficient: the SITL inertial-sensor backend regenerates the gyro/accel on an
**asynchronous timer thread** (`register_timer_process` in `AP_InertialSensor_SITL.cpp`), so a
`read_AHRS()`-level `set_gyro()` / `set_delta_angle()` is overwritten before the estimator and
rate controller consume it.

**Why this is not a replication flaw:** the overwrite affects the §4.1 **attack** and the
recovery **substitution** *identically* — both act on the inertial sensor at the paper's
Figure-3 insertion point. A controlled experiment (EKF2 active, a deliberately large
**+2 rad/s** gyro attack) produced only **0.34° roll deviation**, i.e. the injection never
reached the control loop. The paper ran on **real Pixhawk hardware** (3DR Solo), where the
sensor read and its consumption are co-located in `read_AHRS()` and the substitution
propagates; the SITL software model interposes an asynchronous sensor source that does not
exist on hardware. Reproducing live numbers would require moving the substitution off the
paper's exact insertion point (to the sensor-consumption hook) or editing the SITL sensor
model — both are SITL-specific workarounds, **not** faithful procedure replication, so per the
owner's decision the limitation is recorded rather than worked around.

### L3 — Under-excited translational sub-models
The collected missions are hover-dominant in the vertical/translational axes (throttle barely
departs from hover), so the altitude/position/velocity sub-models are weakly identified
(negative open-loop fits) and those channels can false-trip in flight. **The gyroscope and
attitude channels — the paper's headline case study — are well-identified and clean.** This
is a *data-coverage* property, not an algorithmic error; the faithful remedy is more
aggressive maneuver data (L-R in §7), not a model or threshold hack.

### L4 — Single simulated quadrotor
Only the ArduCopter quadrotor was taken end-to-end. The paper also evaluates a hexrotor and a
rover (and two real vehicles). The rover firmware (APMrover2 2.5) is built but not patched.

---

## 7. Remaining work

Ordered by dependency. None of these change the procedure already implemented; they extend
coverage or address the limitations above.

- [ ] **R-A — Run the live A/B evaluation** once a propagating substitution path is chosen
      (see L2). Two faithful-ish options: (a) real hardware (the paper's setting), or
      (b) apply the corrected sensor value at the consumption hook (`ahrs.get_gyro`/EKF input)
      instead of `read_AHRS` `set_gyro`. Then produce Fig. 17-style A/B plots and the Eq. 7
      pass/fail for the gyro case study.
- [ ] **R-B — Improve translational sub-models (L3).** Collect missions with deliberate
      altitude changes and aggressive translation (so throttle/tilt are well-excited),
      re-run `sysid_12state.m`, regenerate thresholds. Faithful to §3.1.
- [ ] **R-C — Replicate the FP/FN sweep (Fig. 14, §4.2.2).** 20 clean + 20 attacked missions
      across a (window, threshold) grid; confirm the selected parameters give 0 FP / 0 FN.
- [ ] **R-D — Attack matrix (Tables 3 & 4).** Combinational GPS/baro/gyro attacks, and the
      1-/2-/3-of-3 gyro cases including the Appendix-B supplementary compensation path.
- [ ] **R-E — GPS case study (§4.3).** 20 m offset and stealthy carry-off attacks during a
      waypoint mission, with horizontal-position error evaluation. (`attack_gps.py` exists.)
- [ ] **R-F — Rover (APMrover2 2.5).** Apply the same pipeline to the rover the paper also
      evaluates (sysid, thresholds, monitor wiring in the rover control loop).
- [ ] **R-G — Hexrotor.** The third simulated vehicle in Table 1.

---

## 8. One-paragraph defense (for write-up / discussion)

*"We re-implemented the paper's offline modeling pipeline (random-maneuver data collection,
spline resampling, per-variable prediction-error system identification with no observer term)
and its runtime recovery monitor as a line-for-line implementation of Algorithm 1, verified by
25 conformance tests, and wired it into the ArduCopter control loop at the paper's Figure-3
insertion point with per-instance gyro/accelerometer monitoring and GPS/baro/magnetometer
software sensors using the paper's conversion equations. The only intentional deviation is the
loop rate (50 Hz vs. 400 Hz) imposed by available RAM; every downstream parameter is derived
consistently at that rate. We could not produce live closed-loop attack/recovery measurements
because ArduCopter 3.4's SITL takes vehicle attitude from a perfect physics model and
regenerates inertial-sensor data on an asynchronous backend thread, so a sensor substitution at
the paper's insertion point — used identically by our attack injector and our recovery
module — is overwritten before the estimator consumes it. This is a property of the simulator's
software sensor path, not of the recovery technique: on the real Pixhawk hardware the paper
used, the sensor read and its consumption are co-located and the substitution propagates."*
