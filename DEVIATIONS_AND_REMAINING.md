# Deviations from the Paper & Remaining Work — `newImp/` faithful STL detector

**Scope:** the paper-faithful STL detector in `/home/tchowdh4/paperImp/newImp/`
(`faithful_core.py`, `offline_faithful_stl.py`, `online_faithful_stl.py`, `mavlink_source.py`).
**Reference:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic
Vehicles* (RAID 2020), `../SbRRfSAoRV.pdf`.
**Date:** 2026-07-01

This file lists, honestly, **every way this implementation differs from the paper** and **what is
left to do**. Companion: `README_FAITHFUL_STL.md` (what/why/equations/results).

---

## PART A — Deviations from the paper

Grouped by kind. Severity: **H** = changes what is being demonstrated, **M** = moderate, **L** = minor.

### A1. Sanctioned platform deviation

1. **50 Hz instead of the paper's 400 Hz.** *(H, project-wide sanctioned)*
   Direct consequence here: the DTW windows are very large (barometer N≈3492 = 70 s, GPS N≈2639 =
   53 s), the open-loop model drifts more between re-seeds, `e_max`/`T_on` are inflated, and so
   barometer/GPS detection is **slow** (25 s / 3 s) while the gyroscope detects in 0.1 s. This is a
   faithful consequence of the rate change, not tuning — and it is exactly why the paper used 400 Hz.

### A2. Algorithmic / interpretation deviations

2. **STL is a monitor over the statistic, not the accumulator itself.** *(M)*
   STL's temporal operators are `min`/`max`; there is no windowed-sum operator. The paper's detector
   *is* a sum (`R_N = Σ|m−ms|`). So the accumulation is computed in `faithful_core` (the paper's
   algorithm) and STL monitors it: `G(R_N < T_on)` with `ρ = T_on − R_N`. The **detection decision is
   identical** to the paper (`ρ<0 ⇔ R_N>T_on`); the deviation is only that STL is a layer on top, not
   the whole detector.

3. **Sliding window vs. the literal tumbling accumulator.** *(M)*
   Algorithm 1 (as transcribed) accumulates `r` and **resets it at each window checkpoint**
   (tumbling). We instead use a **sliding** window `R_N(t) = Σ_{last N}|m−ms|` for the runtime
   statistic. Reason: it matches §3.3's wording ("accumulated error within *any* window of size N")
   and it lets the `T_off` switch-back actually work (a tumbling `r` never decreases during recovery,
   so it could never fall below `T_off`). This is a documented interpretation choice, not the literal
   pseudo-code. `faithful_core.sliding_R`.

4. **Threshold constants where the paper is silent.** *(L)*
   `T_on = e_max·(1+margin)` with **margin = 10%**, `T_off = 0.80·T_on`, `K = 10`. The paper gives the
   *form* `T = e_max + margin` and the `T_on/T_off/K` roles but not exact values; these are documented
   choices (inherited from the earlier `select_parameters.py`).

5. **Low-pass filter choice.** *(L)*
   `m ← filter(m)` is a 2nd-order Butterworth at 5 Hz (`butter(2, 5/(50/2))`). The paper applies a
   filter but does not specify coefficients; this is a documented choice.

6. **`e_max` calibrated on one clean segment.** *(L)*
   Thresholds are selected on **segment 0** (`select_thresholds(seg=0)`), not the full clean dataset.
   The paper calibrates over the large clean set. Extending to all 21 segments is a one-line change
   and would only raise `T_on` slightly.

### A3. Scope deviations (parts of the paper not implemented here)

7. **Sensor-specific software-sensor conversions (paper §6) are not implemented.** *(H)*
   The paper converts the model output into *raw sensor units* — barometer **altitude→pressure**
   (Eq. 5), magnetometer **heading**, accelerometer **finite-difference of velocity**. Here the model
   has `C = I`, so we compare the **model state directly to the measurement** in engineering units
   (altitude in m, rate in rad/s, position in m). Faithful in spirit (residual of measurement vs
   prediction) but it skips the explicit `convert()` step for pressure/heading/acceleration.

8. **Only 3 sensors monitored (barometer, gyroscope roll-rate, GPS-east).** *(M)*
   The paper monitors all sensors. Accelerometer, magnetometer, and gyro `q`/`r` are not monitored
   here (`ATTACK_CASES` = channels {2, 9, 1}). The framework generalizes to all 12 channels, but the
   accel/mag software sensors (item 7) would be needed first.

9. **Supplementary attitude compensation (paper §13) not implemented.** *(L)*
   The paper's fallback (roll/pitch from accelerometer, yaw from magnetometer when all gyros are
   compromised) is not built.

### A4. Attack & evaluation deviations

10. **Synthetic, sustained attacks — not real physical attacks.** *(H)*
    The paper uses real GPS spoofing / acoustic / optical attacks on real vehicles. Here attacks are
    software injections (baro +3 m, gyro set 0.8 rad/s, GPS +20 m), applied **sustained** from t=40 s
    (sustained is required for the accumulated thresholds to trip).

11. **Recovery is monitored/simulated, not closed-loop on a flying vehicle.** *(H)*
    We implement the `m ← ms` replacement and the `T_on/T_off/K` state machine, and express the
    recovery-time property in STL (`G((R>T_on) → F[0:10s](R<T_off))`), but the replaced sensor value
    is **not fed back into a controller/flight dynamics**. So we do not measure the paper's recovery
    *success* metric (Eq. 7: `|Yt − Ȳt| ≤ ε` for `t∈[1..k]`, ε=3, k=10) on an actual trajectory.

12. **Online = MAVLink replay of recorded data, not a live armed flight.** *(M)*
    `online_faithful_stl.py` runs the real MAVLink receive path, but it is fed by
    `mavlink_source.py` replaying the recorded segment with an injected attack — not a live armed
    SITL/real vehicle under recovery control.

13. **No detection-quality evaluation.** *(H)*
    One attack instance per sensor on one segment. No false-positive rate over the 21 clean segments,
    no ROC / detection-vs-magnitude sweep, no stealthy-attack case, no comparison against a plain
    threshold or the instantaneous-residual detector.

---

## PART B — Remaining work (prioritized)

### B1. To strengthen the *result* (highest value for a review/defense)
1. **Evaluation, not just demonstration.** Report false-positive rate across all 21 clean segments;
   sweep attack magnitude down toward the noise floor and plot detection vs. false alarms (ROC);
   include a **stealthy/gradual** attack and show the accumulated `R_N` detector catches it where an
   instantaneous threshold would not. This is what turns "it works" into "it works *and* is better".
2. **Compare detectors** on the same attacks: instantaneous `|m−ms|` (the earlier work) vs. the
   paper's accumulated `R_N` (this work) — quantify the trade-off (latency vs. false alarms).

### B2. To increase paper fidelity
3. **Implement the §6 software-sensor conversions** (barometer Eq. 5 altitude→pressure with local
   `P0/h0`, magnetometer heading, accelerometer finite-difference) and monitor in raw sensor units.
4. **Monitor all sensors** (accel, mag, gyro `q`/`r`), not just barometer/gyro-x/GPS-east.
5. **Supplementary attitude compensation (§13)** for the all-gyros-compromised case.
6. **Recompute `N` (DTW) and `e_max`** inside `newImp/` over the full clean dataset for
   self-containment (currently `N` is reused from `../rv_recovery/data/recovery_params.npy` and
   `e_max` from segment 0).
7. **Reconcile the sliding vs. tumbling window** — either implement the literal Algorithm 1 tumbling
   reset with an explicit recovery-exit rule, or formally justify the sliding choice.

### B3. To close the loop (make recovery real)
8. **Closed-loop recovery evaluation (Eq. 7).** Feed the recovered sensor (`m←ms`) back into a
   controller / SITL and measure `|Yt − Ȳt| ≤ ε` for `t∈[1..k]` (ε=3, k=10) on the actual
   trajectory — the paper's recovery-success metric. (`../rv_recovery/python/eval_recovery.py`
   already computes Eq. 7 over MAVLink and can be linked.)
9. **Live armed SITL / real flight** for the online path (swap `mavlink_source.py` for a live
   ArduPilot SITL adapter; the MAVLink receive path is already proven).

### B4. Data / realism
10. **Real attack traces** instead of synthetic injections.
11. **Higher-rate data (≈400 Hz)** to match the paper and reduce barometer/GPS detection latency
    (the 50 Hz large-window slowness in A1/§5).

---

## Summary

| | Status |
|---|---|
| Detection algorithm (Algorithm 1 + §3.3 statistic/thresholds) | **faithfully reproduced** |
| STL used for detection (`G(R_N < T_on)`) | **yes**, offline + online |
| Biggest honest gaps | §6 sensor conversions; closed-loop recovery (Eq. 7); detection-quality evaluation; real/stealthy attacks; 400 Hz |
| Nothing tuned to flatter results | confirmed (slow baro/GPS reported openly) |
```
