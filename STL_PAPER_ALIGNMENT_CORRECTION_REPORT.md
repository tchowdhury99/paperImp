# STL Paper Alignment Correction Report

**Project:** `/home/tchowdh4/paperImp/`
**Paper:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles* (RAID 2020) — `SbRRfSAoRV.pdf`
**Guide:** `STL_IMPLEMENTATION_GUIDE.md`
**Interpreter:** `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`
**Date:** 2026-07-01
**Scope:** naming / documentation alignment only. **No STL formula, threshold, window, dataset, attack value, or numerical result was changed.**

---

## 1. What was wrong

The scripts and `STL_FINAL_REPORT.md` wrote the theoretical residual using attack-simulation
variables as if they were the residual variables, e.g.:

```text
baro_residual(t)  = |BARO_Alt_attacked(t) - alt(t)|
gps_east_residual = |GPS_East_attacked   - pE_model|
gyro_residual_x   = |GyrX_attacked       - GyrX_predicted|
```

In code the same pattern appeared as `baro_res_attacked = np.abs(alt - baro_attacked)`, etc. This
presents `*_attacked` as a first-class residual quantity, which is conceptually incorrect.

## 2. Why it was wrong according to Choi et al.

The paper's **Algorithm 1 (Runtime Recovery Monitoring)** defines the residual strictly between
the **actual physical measurement `m`** and the **software-sensor prediction `ms`**:

```text
r ← r + | m − ms |
```

`m` is a single physical measurement that is *clean during normal operation* and *corrupted
during an attack*. The attack does not create a new residual variable; it merely changes the
value of `m` inside the attack window. So the theoretical residual must always be written
`|m_sensor − ms_sensor|`, and `*_attacked` is only an offline device for producing `m_sensor`
during the attack window.

## 3. Correct residual equation from the paper

```text
Instantaneous (per-sample) error:
    e_sensor(t) = | m_sensor(t) − ms_sensor(t) |

Paper accumulated residual (Algorithm 1):
    r(t) = r(t−1) + | m_sensor(t) − ms_sensor(t) |

Windowed form:
    R_sensor,N(t) = Σ (i = t−N+1 … t) | m_sensor(i) − ms_sensor(i) |
```

## 4. Difference between `m_sensor` and the attacked simulation variable

```text
m_sensor(t)        = the physical measurement the monitor receives.
                     Normal time  : clean sensor value.
                     Attack window : artificially corrupted value.
SENSOR_attacked    = an OFFLINE array used only to build m_sensor(t) inside the attack
                     window (e.g. BARO_Alt + 3.0 m). It is NOT a theoretical residual
                     variable and never appears in the residual definition.
Residual is ALWAYS : e_sensor(t) = | m_sensor(t) − ms_sensor(t) |.
```

## 5. Corrected equations for barometer, GPS, gyroscope

```text
Barometer:
    m_baro(t)  = BARO_Alt(t)          (physical; corrupted in attack window)
    ms_baro(t) = alt(t)               (software-sensor / model prediction)
    baro_error(t) = | m_baro(t) − ms_baro(t) |

GPS:
    m_gps_north/east  = GPS position from Lat/Lng in local metres (east corrupted in window)
    ms_gps_north/east = model states pN/pE aligned to the same origin
    gps_north_error(t) = | m_gps_north(t) − ms_gps_north(t) |
    gps_east_error(t)  = | m_gps_east(t)  − ms_gps_east(t)  |

Gyroscope (paper §6.2: m_gyro,s = [p q r] from model state):
    m_gyr_j  = measured angular rate  (roll x corrupted in window)
    ms_gyr_j = model angular-rate state prediction
    gyro_error_j(t) = | m_gyr_j(t) − ms_gyr_j(t) | ,  j ∈ {x, y, z}
```

Dataset note (pre-existing, unchanged): the dataset has no raw-gyro channel distinct from the
model rate state, so on clean data `m_gyr ≈ ms_gyr` (clean gyro residual ≈ 0). This is a dataset
limitation inherited from the guide, not introduced by this correction.

## 6. Corrected STL formulas, if any were changed

**No STL formula was changed.** All eight STL specifications keep their exact structure,
thresholds, windows, and monitored-variable names:

| Spec | STL formula (unchanged) |
|---|---|
| Barometer Integrity | `G[0:2000ms] (baro_res < 0.30)` |
| GPS Integrity | `G[0:580ms] (gps_north_res < 0.169349 and gps_east_res < 0.169349)` |
| Gyroscope Integrity | `G[0:580ms] (gyro_residual_x < 0.15 and gyro_residual_y < 0.15 and gyro_residual_z < 0.15)` |
| Multi-Sensor Compound | `G[0:580ms] (alt>0.97 and alt<29.70 and baro_residual<0.30 and gps_*<0.169349 and gyro_*<0.15)` |
| Persistent Baro | `G[0:580ms] (G[0:1000ms] (baro_residual < 0.30))` |
| Baro Recovery ≤10 s | `G[0:580ms] ((baro_residual > 0.30) -> F[0:10000ms] (baro_residual < 0.30))` |
| Any-Attack Recovery | `G[0:580ms] (((baro_residual>0.30) or (gyro_residual_x>0.15)) -> F[0:10000ms] (not(...)))` |
| Altitude Bounds | `G[0:580ms] ((alt > 0.97) and (alt < 29.70))` |

Only the **construction** of the residual arrays that feed these formulas was rewritten in terms
of `m_sensor` / `ms_sensor`; the arrays are numerically identical to before.

## 7. Scripts modified

Restructured / renamed only (mathematically identical):

```text
offline_stl_baro.py
offline_stl_gps.py
offline_stl_gyro.py
offline_stl_multi_sensor.py
offline_stl_baro_persistent.py
offline_stl_baro_recovery.py
offline_stl_any_attack_recovery.py
```

**Not modified:** `offline_stl_altitude_bounds.py` — it is a state-bound spec
(`G[0:580ms]((alt>0.97) and (alt<29.70))`), not a sensor residual, so `m/ms` notation does not
apply (per the correction brief).

## 8. Reports modified

```text
STL_FINAL_REPORT.md        — added §0 paper-aligned residual definition; rewrote every
                             residual equation block from |SENSOR_attacked − pred| to |m − ms|.
STL_COMPLETION_STATUS.md   — added a residual-variable-meaning note (paper-aligned |m − ms|).
```

## 9. Backup location

```text
/home/tchowdh4/paperImp/stl_correction_backup/
```

Contains the pre-correction copies of all 8 `offline_stl_*.py` scripts and
`STL_FINAL_REPORT.md`, `STL_COMPLETION_STATUS.md`, `STL_WORK_EXPLAINED.md`.

## 10. Verification commands and outputs

Ran with `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3` (MPLBACKEND=Agg). All exit 0;
detection results **identical to before the correction** (proving math unchanged):

```text
offline_stl_baro                : Attack detected t = 38.00 s   (latency −2.00 s)
offline_stl_gps                 : Attack detected t = 39.42 s   (latency −0.58 s)
offline_stl_gyro                : Attack detected t = 39.42 s   (latency −0.58 s)
offline_stl_multi_sensor        : Attack detected t = 39.42 s   (latency −0.58 s)
offline_stl_baro_persistent     : Attack detected t = 38.42 s   (latency −1.58 s)
offline_stl_baro_recovery       : No recovery STL violation detected
offline_stl_any_attack_recovery : No recovery STL violation detected
offline_stl_altitude_bounds     : No altitude bounds STL violation detected
```

Plot existence / non-empty check (`test -s`): all 8 PASS —
`stl_result_{baro,gps,gyro,multi_sensor,baro_persistent,baro_recovery,any_attack_recovery,altitude_bounds}.png`.

(The negative "latency" values are the forward-looking `G[0:W]` window horizon, i.e. the spec sees
up to `t+W`; they are unchanged by this correction and are explained in `STL_WORK_EXPLAINED.md`.)

## 11. Remaining deviations, if any

1. **Instantaneous vs accumulated residual.** The STL specs monitor `|m − ms|` per sample, not the
   paper's accumulated `R_sensor,N(t) = Σ|m − ms|`. This is a **guide-based simplification**
   (`STL_IMPLEMENTATION_GUIDE.md` §5), now explicitly disclosed. Not changed, because the guide
   mandates the instantaneous form and changing it would alter results and deviate from the guide.
2. **Gyro dataset approximation.** No raw-gyro channel distinct from the model rate state exists in
   the dataset, so clean gyro residual ≈ 0. Pre-existing, inherited from the guide.
3. **50 Hz vs the paper's 400 Hz.** The one sanctioned project-wide sampling deviation; unchanged.

No new deviation was introduced by this correction.

## 12. Final status

```text
Correction type            : naming / documentation alignment only
Scripts mathematically changed : NONE (7 restructured/renamed; 1 untouched)
STL formulas changed       : NONE
Instantaneous residual used: YES (guide-based simplification, disclosed)
Cumulative residual added  : NO (paper's accumulated form documented, not implemented)
Matches paper              : residual DEFINITION now matches Algorithm 1 (|m − ms|);
                             STL uses the guide's instantaneous simplification
Scripts run                : 8 / 8 exit 0
Plots non-empty            : 8 / 8
Numerical results          : identical to pre-correction
Backup                     : /home/tchowdh4/paperImp/stl_correction_backup/
Remaining deviations       : instantaneous-vs-accumulated (guide), gyro dataset approx, 50 Hz
```
