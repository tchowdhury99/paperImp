# STL Remaining Work — Completed: What, Why, Equations, Deviations, Remaining

**Project:** `/home/tchowdh4/paperImp/`
**Base paper:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles*, RAID 2020 (`SbRRfSAoRV.pdf`)
**STL idea reference:** SpecGuard (monitor a vehicle against a formal spec)
**Dataset:** `rv_recovery/data/operation_data_50hz.mat`, segment 0, 50 Hz (Ts = 0.02 s)
**Model:** `rv_recovery/matlab/models/quadrotor_12state.mat` (identified A,B,C,D)
**Interpreter:** `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`
**Date:** 2026-06-30
**Companion file:** `MAVLINK_STEPS.md` (the MAVLink part, step by step)

---

## 0. What was outstanding, and what is now done

The previous report (`STL_WORK_EXPLAINED.md`) listed the offline STL as 100% done but the
"full STL monitor" at ~85%, with this remaining list. Status now:

| Remaining item (from STL_WORK_EXPLAINED.md §3) | Status now |
|---|---|
| 1. Real-time / online STL via MAVLink/SITL | ✅ **Done** — `online_stl_mavlink.py`, verified |
| 2. Close the loop (trigger recovery when ρ<0) | ✅ **Done** — m←ms + K-safe switch-back |
| 3. Replace gyro placeholder with a real model prediction | ✅ **Done** — gyro ms = (Cx+Du)[9] |
| 4. Live-autopilot MAVLink connectivity | ✅ **Done** — real ArduPilot SITL heartbeat + streams |
| 5. Fix blocking `plt.show()` in baro/gps scripts | ✅ **Done** — Agg backend, show() removed |
| 6. Report latency honestly | ✅ **Done** — online latency ≈ 0 (causal); offline artifact explained |
| Live *armed flight* driving the model's `u` vector | ⬜ Remaining (see §7) |

**New completion: offline STL 100%; online STL monitor + closed-loop recovery built and
verified; full "live armed-flight closed loop" ~95%** (only a live-flight adapter remains).

---

## 1. What I built (files)

```
mavlink_replay_source.py        # streams operation_data_50hz.mat as MAVLink (with attack injection)
online_stl_mavlink.py           # ONLINE STL monitor + closed-loop recovery over MAVLink
stl_result_online_baro.png      # baro-spoof run     (ALT detected 40.00 s, recovered)
stl_result_online_gyro.png      # gyro-spoof run     (P detected 40.02 s, recovered)
stl_result_online_clean.png     # clean run          (no false positives)
offline_stl_baro.py / _gps.py   # patched: Agg backend, removed blocking plt.show()
MAVLINK_STEPS.md                # the MAVLink part explained step by step
STL_REMAINING_WORK_COMPLETED.md # this file
```

---

## 2. Why — connecting the paper, the offline STL, and the live monitor

The paper builds a **software sensor**: an identified model predicts what each physical sensor
*should* read; a large discrepancy means the sensor is under attack, and during recovery the
predicted value replaces the real sensor. The offline STL work wrapped those discrepancies in
formal specs but ran on a recorded file *after the fact*. The missing piece was running this
**online**, over MAVLink, and actually **acting** on a detection. That is what these two scripts
add: STL becomes the live detector, and the paper's sensor-replacement becomes the live actuator.

---

## 3. The equations implemented (and where, in code)

### 3.1 Software-sensor model — paper Eq. (1)–(2)

```
y(t)   = C x(t) + D u(t)        # software-sensor prediction  ms
x(t+1) = A x(t) + B u(t)        # model state advance
```

`online_stl_mavlink.py`: `y = C @ x + D @ u ; x = A @ x + B @ u`.
`A (12×12), B (12×7), C = I₁₂, D = 0` from `quadrotor_12state.mat`. State vector (paper §3):
`x = [pN pE alt φ θ ψ vN vE vUp p q r]`.

### 3.2 Low-pass filter on the real measurement — paper Alg. 1 `m ← filter(m)`

Second-order Butterworth, cutoff 5 Hz at 50 Hz (`butter(2, 5/(50/2))`), identical to
`recovery_monitor.h` / `select_parameters.py`. Applied to the roll-rate measurement `p`.

### 3.3 Residual — paper Alg. 1 `r ← r + |m − ms|`

Evaluated **per sample** for fast STL detection (the paper accumulates it over a window for its
plain-threshold detector; STL gives instantaneous robustness instead):

```
ALT : resid_alt = | baro − alt_estimate |        eps_alt = 0.30 m
P   : resid_p   = | filter(p) − (Cx+Du)[9] |      eps_p   = 0.15 rad/s
```

### 3.4 STL robustness — quantitative semantics

Online causal spec `φ = (resid < ε)`, so each tick rtamt returns

```
ρ(φ, t) = ε − resid(t)        ρ ≥ 0 satisfied (healthy) ; ρ < 0 violated (attack)
```

`spec.update(tick, [('resid', value)])` — verified to return `ε − resid` per step.

### 3.5 Closed-loop recovery — paper Alg. 1 (Ton/Toff/K state machine)

```
if ρ < 0 and not recovery:  recovery ← true ; safe ← 0        # detect (STL threshold = ε)
if recovery:
    m ← ms                                                    # sensor replacement
    safe ← safe+1 if resid < ε else 0                         # count stable samples
    if safe > K:  recovery ← false                            # switch back (K = 10)
```

STL's `ε` plays the role of the paper's detection threshold `T_on`; the switch-back uses the
same `resid < ε` condition held for `K` consecutive samples (the paper's `T_off`/`K` idea).

### 3.6 Re-sync window — why the model tracks in real time

The open-loop model integrates, so `alt`/`pN`/`pE` drift if run unattended (measured RMSE ≈ 7.8 m
for `alt` over 140 s). The paper handles this by periodically re-seeding the model state from the
real sensor inside each window. The monitor re-syncs the model channel to the filtered real
measurement every **W = 29 samples (580 ms)** *while healthy*, and **stops re-syncing during
recovery** (so it never re-seeds from the attacked sensor). 580 ms is the paper-shaped window
(3DR Solo used 575 ms; 580 ms = 29×20 ms is the nearest 50 Hz multiple — same value used by the
offline STL specs).

### 3.7 Why ALT uses the estimate, not the open-loop integrator

For altitude the *autopilot's fused estimate* is itself the trustworthy reference and is directly
available live (`GLOBAL_POSITION_INT.relative_alt`); comparing the raw barometer to it is exactly
the offline baro-integrity spec and avoids open-loop integration drift. For the gyro the body rate
`p` is a (near-)directly-observed state, so the identified model predicts it well (clean residual
≤ 0.024 rad/s ≪ 0.15) — which is why the gyro can use a true model prediction and no longer needs
the old clean-copy placeholder.

---

## 4. Verification (measured, honest)

All three runs went over the MAVLink UDP link (monitor `udpin:14570`, source `udpout:14570`):

| Run | ALT result | P result | False positives? |
|---|---|---|---|
| baro spoof +3.0 m, samples 2000–2500 | **detected t = 40.00 s**, recovery 510 samples | clean | none |
| gyro spoof p=0.8 rad/s, samples 2000–2500 | clean | **detected t = 40.02 s**, recovery 513 samples | none |
| clean (no attack) | no violation | no violation | none |

Clean-window peak residuals: ALT 0.202 m (< 0.30), P 0.024 rad/s (< 0.15) → margin before any
false alarm. Detection occurs **at the true attack onset (40.0 s), latency ≈ 0**, and recovery
clears a few samples after the attack ends (attack 500 samples + the K-safe tail).

Live-autopilot check: real ArduPilot SITL (`arducopter-quad`) produced a valid `HEARTBEAT`
(type 2 QUADROTOR, autopilot 3) and streamed `SCALED_PRESSURE` (`press_abs` 836.9 hPa ≈ the
dataset's ~83.6 kPa ground pressure), `RAW_IMU`, `ATTITUDE`, and `GLOBAL_POSITION_INT` — the
exact channels the monitor consumes. Details in `MAVLINK_STEPS.md`.

---

## 5. The negative-latency artifact — resolved

The offline scripts printed *negative* "detection latency" (e.g. baro −2.0 s) because the
offline specs use a forward-looking bounded operator `G[0:W]`, whose robustness at time `t`
already sees up to `t+W`; the first ρ<0 lands at `onset − W`. That is **not** early detection.
The **online** monitor uses a causal instantaneous predicate (`ρ = ε − resid` now), so its
reported latency is the true value, ≈ 0. The artifact is therefore both *explained* (offline) and
*eliminated* (online).

---

## 6. Deviations

### 6.1 From the paper (overall setup)
1. **50 Hz, not 400 Hz** — the one sanctioned project-wide deviation; all params consistent at 50 Hz.
2. **STL detector in place of the plain accumulated-residual threshold.** The paper detects with
   `r > T_on` over a window; here STL robustness `ρ = ε − resid` per sample is the detector. This
   is intentional (the whole point of the STL layer) and the recovery back-end (m←ms, K-safe) is
   the paper's. ε replaces `T_on`.
3. **Per-sample residual** rather than the paper's window-accumulated `r` for detection — gives
   faster, sample-resolved detection. The accumulated-`r` + DTW-derived `N`, `T_on`, `T_off`
   already exist in the project (`recovery_params.npy`: N=3492, T_on=6619, T_off=5229) and remain
   available; they are large because the DTW window for integrating channels is long, which makes
   the accumulated detector insensitive to small spoofs — another reason STL's per-sample ε is used.
4. **Attacks are synthetic injections** (baro +3 m; p=0.8 rad/s; samples 2000–2500), applied on
   the MAVLink wire to model a compromised sensor.
5. **Closed loop is demonstrated on replayed flight data**, not on a live *armed* SITL flight
   (the live link itself is proven; see §7).

### 6.2 From the offline STL guide — none of substance
Same dataset, segment, thresholds (ε_baro 0.30, ε_gyr 0.15), and 580 ms window. The online spec
is the causal form of the same predicate. Mechanical only: `matplotlib.use('Agg')` + removal of
blocking `plt.show()`; rtamt online `update()` for the causal predicate.

### 6.3 Honest caveat that is now *fixed*
The offline gyro "prediction" was a clean-signal copy (placeholder). The **online** gyro uses the
genuine identified-model output `(Cx+Du)[9]`, which tracks the real rate (clean residual ≤ 0.024)
— so the deployed monitor no longer relies on the placeholder.

---

## 7. Remaining work

1. **Live armed-flight closed loop.** Swap the replay source for a thin live-SITL adapter
   (read `SCALED_PRESSURE`/`GLOBAL_POSITION_INT`/`RAW_IMU` instead of `NAMED_VALUE_FLOAT`), fly an
   armed GUIDED mission, and drive the model with the live control vector. The MAVLink link and the
   STL+recovery core are already proven; this is an integration/adapter task plus deriving the
   project's `u = [φ_cmd θ_cmd ψ_cmd thr tiltN tiltE const]` from live MAVLink setpoints.
2. **Push recovery into the firmware.** The project already has `recovery_monitor.h` /
   `recovery_params.h` in the ArduPilot tree; wiring the STL verdict to the in-loop sensor path
   (rather than an external monitor) would match the paper's "patch immediately after sensor read."
3. **Multi-sensor online specs.** Extend the online monitor from {ALT, P} to GPS and the remaining
   gyro/accel/mag channels (the offline multi-sensor spec already exists; replicate it causally).
4. **Real attack traces** instead of synthetic injections, if available.
5. **Eq. 7 recovery-success scoring online** (`|Yt − Ȳt| ≤ ε for t ∈ [1..k]`, ε=3, k=10) — the
   project's `eval_recovery.py` already computes this over MAVLink ATTITUDE and can be linked to
   the monitor's recovery episodes.

---

## 8. How to reproduce

```bash
cd /home/tchowdh4/paperImp
PY=/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

# terminal A — monitor
$PY online_stl_mavlink.py --conn udpin:127.0.0.1:14570 --plot stl_result_online_baro.png
# terminal B — attacked replay (baro)
$PY mavlink_replay_source.py --out udpout:127.0.0.1:14570 --attack baro --offset 3.0 --rate 1000
# (repeat with --attack gyro --gyroval 0.8, and --attack none for the clean check)
```

Live-autopilot link check:
```bash
ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad --model quad \
    --home 40.0713,-105.2300,1584,0 \
    --defaults ardupilot_ws/arducopter-3.4/Tools/autotest/default_params/copter.parm
# then connect: mavutil.mavlink_connection('tcp:127.0.0.1:5760'); wait_heartbeat()
```
