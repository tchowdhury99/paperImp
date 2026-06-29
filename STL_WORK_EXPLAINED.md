# STL Work — Completion, Deviation, Remaining Work, and Full Explanation

**Project:** `/home/tchowdh4/paperImp/`
**Base paper:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles*, RAID 2020 (`SbRRfSAoRV.pdf`)
**STL tooling reference:** SpecGuard (`/home/tchowdh4/SpecGuard_Implementation/...pdf`) — used for the idea of monitoring a vehicle against a formal specification
**Guide followed:** `/home/tchowdh4/paperImp/STL_IMPLEMENTATION_GUIDE.md`
**Prior reports:** `STL_COMPLETION_STATUS.md`, `STL_FINAL_REPORT.md`
**This document written:** 2026-06-29
**Verification:** every script below was re-executed on 2026-06-29 with `MPLBACKEND=Agg` and the pyenv 3.10.14 interpreter. All 8 ran to `exit 0` and regenerated their plots. Results are reproduced verbatim in Section 6.

---

## 0. TL;DR

- **Your stated goal** — "monitor a drone with STL so it does not go beyond the altitude threshold" — **is implemented and working.** It is the *Altitude Bounds* spec (`offline_stl_altitude_bounds.py`), formula `G[0:580ms] ((alt > 0.97) and (alt < 29.70))`.
- On top of that, **all 8 ready-to-use STL formulas** from the guide (Section 5) are implemented, run cleanly, and produce plots.
- **Completion: 100% of the offline STL formula set; ~85% of "STL as a deployed monitor"** — because the work is **offline on recorded data only**. The real-time / closed-loop SITL monitor (guide Section 13) is the missing piece.
- **Deviation from the guide's STL formulas: effectively none** — only mechanical compatibility fixes (e.g. `575ms → 580ms` window, offline `evaluate()`).
- **Honest caveats you should know:** (1) "detection latency" prints as **negative** in several scripts — this is *not* early detection, it is the STL forward-looking window horizon (explained in Section 7); (2) the gyroscope "prediction" is a copy of the clean signal, not a true model output, so its clean residual is identically zero (Section 8).

---

## 1. What "completion percentage" means here

The honest answer depends on what you count, so here are three framings:

| Scope | What it covers | Completion |
|---|---|---|
| **A. Your stated goal** (altitude threshold monitor) | One STL altitude-bounds monitor over drone altitude | **100%** |
| **B. Guide Section 5 "ready-to-use" STL formulas** | 8 offline STL specs (baro/GPS/gyro/multi/persistent/recovery/any-attack/altitude) | **100% (8/8)** |
| **C. "STL as a real monitor" for the paper** | Offline specs **+** real-time SITL monitoring **+** closed-loop recovery trigger | **~85%** (offline done; online/closed-loop not done) |

**Bottom line: 100% of what the guide told you to build offline is done; the remaining ~15% is the live/closed-loop deployment that the guide describes in Section 13 but you have not implemented.**

---

## 2. Deviation

### 2.1 Deviation from the STL guide formulas — essentially none

No STL formula, threshold, dataset, segment, attack value, or attack window was changed from the guide. The only changes were mechanical compatibility corrections required by the installed `rtamt` version and the 50 Hz sampling rate:

| # | Correction | Why | Affects logic? |
|---|---|---|---|
| 1 | `575ms` example window → **`580ms`** | 50 Hz = 20 ms/sample; 575 is not a multiple of 20, 580 = 29 samples | No (window granularity only) |
| 2 | `set_sampling_period(20, "ms", 0.1)` | installed `rtamt` wanted the operator bound | No |
| 3 | Offline `spec.evaluate()` instead of online `spec.update()` | `rtamt` online path was unreliable for bounded/nested temporal ops | No (same robustness, computed offline) |
| 4 | Dataset shape fallback (`"time"` key vs `(t,val)` pairs) | different `rtamt` versions accept different shapes | No |
| 5 | Class-name fallback `STL...`/`Stl...` | capitalization differs by version | No |
| 6 | Used `BARO_Alt` directly instead of pressure Eq. 5 | dataset pressure has a ~17.6 kPa local ground offset; guide explicitly says to use `BARO_Alt` | No (guide-sanctioned) |
| 7 | Altitude from `Y[:,2]`/`BARO_Alt`, not `GPS_Alt` | `GPS_Alt` is MSL (~1600 m), not AGL; guide says don't use it | No (guide-sanctioned) |

### 2.2 Deviation from the *papers* (bigger picture — be aware)

These are deviations of the **overall STL setup** relative to the RAID 2020 paper and SpecGuard, not deviations from the guide:

1. **Sampling rate: 50 Hz, not the paper's 400 Hz.** This is the one sanctioned deviation already recorded for the whole project (memory: *RV recovery replication project*). Everything downstream is consistent at 50 Hz.
2. **Offline, not online.** STL is evaluated over a recorded `.mat` segment, not over a live MAVLink stream from a running SITL. The paper/guide envision a real-time monitor.
3. **Attacks are synthetic injections**, added in software to a clean segment (e.g. `+3.0 m` baro, `0.8 rad/s` gyro, `+20 m` GPS east, over samples 2000–2500 = 40–50 s). They are not captured real attacks.
4. **No closed-loop recovery.** STL detects/flags; it does not trigger the paper's software-sensor recovery switch (Ton/Toff/K logic). It is a *monitor*, not yet an *actuator* in the loop.
5. **Gyro "prediction" is a clean-signal copy, not a model output** (Section 8). Baro and GPS residuals *do* use the model state (`alt`, `pN_model`, `pE_model`) as the prediction, which is closer to the paper's software-sensor idea.

---

## 3. Remaining work

In priority order:

1. **Real-time / online STL via SITL (guide Section 13).** Connect `rtamt` to a live ArduPilot/PX4 MAVLink stream and evaluate robustness sample-by-sample while the drone flies. This is the largest remaining item and the one that turns "offline analysis" into "monitoring a drone."
2. **Close the loop.** When `ρ < 0`, trigger the paper's recovery (switch real sensor → software sensor; apply Ton/Toff/K). Currently nothing is triggered.
3. **Replace the gyro placeholder prediction with a real model.** Use an actual A,B,C,D software-sensor prediction for the gyro so the clean residual is meaningful rather than identically zero (Section 8).
4. **Use real attack traces** instead of synthetic injections, if available, to validate detection on genuine data.
5. **Fix the two blocking `plt.show()` calls** in `offline_stl_baro.py` (line 150) and `offline_stl_gps.py` (line 212). They hang on a headless machine; run with `MPLBACKEND=Agg` or add `matplotlib.use("Agg")` (as `offline_stl_baro_recovery.py` already does). Cosmetic, but it is why a plain run appears to "freeze."
6. **Report latency honestly.** Re-label the misleading negative "detection latency" (Section 7).

Everything in guide **Section 5 is complete**; the remaining work lives in guide **Sections 11–13** (live MAVLink + online monitor) plus the closed-loop hookup.

---

## 4. The 8 STL specifications (what each one checks and why)

All use: dataset `rv_recovery/data/operation_data_50hz.mat`, **segment 0**, `Ts = 0.02 s` (50 Hz). Robustness convention: **ρ > 0 satisfied, ρ < 0 violated, ρ = 0 on boundary.**

| # | Name | Script | Plot | Formula |
|---|---|---|---|---|
| 1 | Barometer Integrity | `offline_stl_baro.py` | `stl_result_baro.png` | `G[0:2000ms] (baro_res < 0.30)` |
| 2 | GPS Integrity | `offline_stl_gps.py` | `stl_result_gps.png` | `G[0:580ms] (gps_north_res < 0.169349 and gps_east_res < 0.169349)` |
| 3 | Gyroscope Integrity | `offline_stl_gyro.py` | `stl_result_gyro.png` | `G[0:580ms] (gyro_res_x<0.15 and gyro_res_y<0.15 and gyro_res_z<0.15)` |
| 4 | Multi-Sensor Compound | `offline_stl_multi_sensor.py` | `stl_result_multi_sensor.png` | `G[0:580ms] (alt>0.97 and alt<29.70 and baro_res<0.30 and gps_n<.169349 and gps_e<.169349 and gyro_x<.15 and gyro_y<.15 and gyro_z<.15)` |
| 5 | Persistent Baro Pattern | `offline_stl_baro_persistent.py` | `stl_result_baro_persistent.png` | `G[0:580ms] (G[0:1000ms] (baro_res < 0.30))` |
| 6 | Baro Recovery ≤10 s | `offline_stl_baro_recovery.py` | `stl_result_baro_recovery.png` | `G[0:580ms] ((baro_res>0.30) -> F[0:10000ms] (baro_res<0.30))` |
| 7 | Any-Attack Recovery | `offline_stl_any_attack_recovery.py` | `stl_result_any_attack_recovery.png` | `G[0:580ms] (((baro_res>0.30) or (gyro_x>0.15)) -> F[0:10000ms] (not(...)))` |
| 8 | **Altitude Bounds (your goal)** | `offline_stl_altitude_bounds.py` | `stl_result_altitude_bounds.png` | `G[0:580ms] ((alt>0.97) and (alt<29.70))` |

**Why these formulas:** The RAID 2020 paper compares each real physical sensor against a model/software-sensor prediction; a large residual means the sensor is compromised. STL wraps those residuals (and the altitude state bounds) in formal temporal specifications so a single robustness number ρ tells you, at every instant, whether the safety/integrity property holds and by how much. Specs 1–4 are integrity checks, 5 adds temporal persistence (fewer false positives), 6–7 check that an attack *clears within 10 s* (the paper's recovery requirement), and 8 is the mission-level altitude envelope.

### Thresholds and attacks (per guide)
- ε_baro = 0.30 m, ε_gps = 0.169349 m, ε_gyr = 0.15 rad/s; altitude bounds h_min = 0.97 m, h_max = 29.70 m (dataset min/max).
- Injected attacks (samples 2000–2500 = 40.00–50.00 s): baro `+3.0 m`; gyro X `set to 0.8 rad/s`; GPS east `+20 m`.

---

## 5. How the residuals are built (so the numbers are interpretable)

- **Barometer:** `baro_res(t) = |BARO_Alt(t) − alt(t)|` where `alt = Y[:,2]` is the **model state** (genuine model-vs-sensor residual).
- **GPS:** `gps_north_res = |GPS_North − pN_model_local|`, `gps_east_res = |GPS_East − pE_model_local|`. GPS lat/lng are converted to local metres about the first sample; model `pN/pE = Y[:,0:1]` re-based to the same origin (genuine model-vs-sensor residual).
- **Gyro:** `gyro_res = |GyrX_measured − GyrX_predicted|`, but **`GyrX_predicted` is a copy of the clean `Y[:,9]`** and the clean "measured" is the same copy ⇒ clean residual ≡ 0; attacked residual = `|0.8 − clean|`. See Section 8 — this is a placeholder, not a real software sensor.

---

## 6. Verified run output (re-run 2026-06-29, all exit 0)

```
offline_stl_baro            : Attack detected at t = 38.00 s   latency = -2.00 s
offline_stl_gps             : Attack detected at t = 39.42 s   latency = -0.58 s
offline_stl_gyro            : Attack detected at t = 39.42 s   latency = -0.58 s
offline_stl_multi_sensor    : Attack detected at t = 39.42 s   latency = -0.58 s
offline_stl_baro_persistent : Attack detected at t = 38.42 s   latency = -1.58 s
offline_stl_baro_recovery   : No recovery STL violation detected
offline_stl_any_attack_recov: No recovery STL violation detected
offline_stl_altitude_bounds : No altitude bounds STL violation detected
```

All 8 plots were regenerated. The three "No ... violation" results are expected: recovery specs are satisfied because the injected attack ends at 50 s and the residual returns below threshold well within the 10 s window; the altitude spec is satisfied because the clean segment never leaves [0.97, 29.70] m.

---

## 7. ⚠️ Important honest caveat: the "negative detection latency" is not early detection

Several scripts print a **negative** latency (e.g. baro −2.00 s, gyro/gps/multi −0.58 s, persistent −1.58 s). This looks like the monitor detected the attack *before it happened*. It did not. It is an artifact of the **forward-looking bounded-Globally operator**:

`ρ(G[0:W] φ, t) = min over τ ∈ [t, t+W] of ρ(φ, τ)`

So at time `t`, `G[0:W]` already "sees" the worst point up to `t+W`. The first index where ρ goes negative is therefore the attack onset **minus the window width W**:

| Spec | Window horizon W | Attack onset | First ρ<0 | onset − W |
|---|---|---|---|---|
| baro | 2000 ms | 40.00 s | 38.00 s | 38.00 s ✔ |
| gyro/gps/multi | 580 ms | 40.00 s | 39.42 s | 39.42 s ✔ |
| persistent | 580 + 1000 ms ≈ 1580 ms | 40.00 s | 38.42 s | 38.42 s ✔ |

The match is exact, which confirms the mechanism. **True detection latency relative to the window is ~0 (the monitor flags the attack as soon as it enters the look-ahead horizon).** The printed "latency = onset − first_violation" is mislabeled and should be reported as ~0 with a stated horizon, not as negative. This does not indicate a bug in the STL logic — only in how the latency line is phrased.

---

## 8. ⚠️ Important honest caveat: the gyroscope "prediction" is a placeholder

In `offline_stl_gyro.py`, `gyr_x_predicted = Y[:,9].copy()` and the clean "measured" signal is the **same** array. The clean gyro residual is therefore identically zero, and only the injected attack creates a non-zero residual. This means the gyro spec demonstrates the *STL mechanism* correctly but does **not** exercise a real software-sensor model the way the baro and GPS specs do (which use the independent model state). To make the gyro check faithful to the paper, replace the copy with an actual A,B,C,D model prediction of the gyro rates. This is listed in Remaining Work (Section 3, item 3).

---

## 9. Files

**STL scripts (8 final + intermediates):**
```
offline_stl_baro.py                         # spec 1
offline_stl_gps.py                          # spec 2   (+ step8_gps_paper_params.py helper)
offline_stl_gyro.py                         # spec 3
offline_stl_multi_sensor.py                 # spec 4
offline_stl_baro_persistent.py              # spec 5
offline_stl_baro_recovery.py                # spec 6
offline_stl_any_attack_recovery.py          # spec 7
offline_stl_altitude_bounds.py              # spec 8  (your altitude-threshold monitor)
# intermediates kept from development (safe to archive):
offline_stl_baro_before_rtamt_eval_fix.py
offline_stl_gyro_failed_no_time_key.py
offline_stl_gyro_failed_online_update.py
```

**Plots:** `stl_result_baro.png`, `stl_result_gps.png`, `stl_result_gyro.png`, `stl_result_multi_sensor.png`, `stl_result_baro_persistent.png`, `stl_result_baro_recovery.png`, `stl_result_any_attack_recovery.png`, `stl_result_altitude_bounds.png`

**Reports:** `STL_IMPLEMENTATION_GUIDE.md` (the brief), `STL_COMPLETION_STATUS.md`, `STL_FINAL_REPORT.md`, and **this file**.

**How to reproduce (headless-safe):**
```bash
cd /home/tchowdh4/paperImp
MPLBACKEND=Agg /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 offline_stl_altitude_bounds.py
```

---

## 10. Final status

| Item | Status |
|---|---|
| Your altitude-threshold STL monitor | ✅ Done & verified |
| Guide Section 5 offline STL formulas (8/8) | ✅ Done & verified (all run exit 0) |
| Deviation from guide formulas | None (compatibility fixes only) |
| Deviation from papers | 50 Hz (sanctioned); offline-only; synthetic attacks; no closed loop; gyro prediction is a placeholder |
| Real-time SITL monitor (guide §13) | ❌ Not done |
| Closed-loop recovery trigger | ❌ Not done |
| **Overall** | **100% of offline STL; ~85% of full "STL monitor" deployment** |
