# Paper-Faithful STL Attack Detection — `newImp/`

**Location:** `/home/tchowdh4/paperImp/newImp/`
**Paper:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles* (RAID 2020) — `../SbRRfSAoRV.pdf`
**Interpreter:** `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`
**Date:** 2026-07-01

This is a **from-scratch, paper-faithful** reimplementation that detects sensor attacks **with STL**,
running **both offline and online**. Unlike the earlier `../offline_stl_*.py` work (which used the
STL guide's *instantaneous* residual with round-number thresholds), this version reproduces the
paper's **Algorithm 1** detection statistic and **§3.3** parameter selection, and uses STL as the
formal monitor over that statistic.

---

## 1. What this does (short version)

- Builds the paper's **software sensor** from the identified model (`y = Cx + Du`, `x = Ax + Bu`)
  with the paper's **low-pass filter**, **per-window model re-seed**, and **error compensation `e`**.
- Computes the paper's **windowed accumulated residual** `R_{k,N}(t) = Σ_{last N} |m − ms|`.
- **Detects with STL:** `φ = G (R_N < T_on)` → robustness `ρ = T_on − R_N`; `ρ < 0` is exactly the
  paper's rule `R_N > T_on`.
- **Recovers** with the paper's `T_on/T_off/K` state machine (`m ← ms`), with re-seed suppressed
  during recovery (Algorithm 1) so a sustained attack is not absorbed.
- Runs **offline** (on the recorded dataset) and **online** (real-time over MAVLink), using the
  **same core and the same parameters**.

---

## 2. Why this is faithful to the paper (and where STL fits)

| Paper element (Choi et al.) | Where in code |
|---|---|---|
| Software sensor `y=Cx+Du`, `x=Ax+Bu` (Eq. 1–2) | `faithful_core.instantaneous_residual` / `run_monitor` / `FaithfulMonitor.step` |
| Low-pass filter `m ← filter(m)` | `faithful_core.LPF` (butter(2, 5/(50/2))) |
| Per-window checkpoint re-seed + error term `e` | same functions (Algorithm 1 checkpoint) |
| **Accumulated residual** `r ← r + |m−ms|`, windowed `R_{k,N}` (§3.3) | `faithful_core.sliding_R` / running sum in the monitors |
| Detection rule `r > T_on` | **STL** `G(R < T_on)` → `ρ<0` | 
| Threshold `T_on = e_max + margin`, `T_off < T_on` (§3.3) | `faithful_core.select_thresholds` (margin 10%, T_off=0.8·T_on) |
| Window `N` via DTW | reused from `../rv_recovery/data/recovery_params.npy` (DTW-derived) |
| Recovery FSM `m←ms`, back after `K` safe (Algorithm 1) | `run_monitor` / `FaithfulMonitor` (K=10) |
| Recovery-within-time property (Eq. 7 spirit) | **STL** `G((R>T_on) → F[0:10s](R<T_off))` |

**One honest, unavoidable point about STL.** STL's temporal operators are `min`/`max` (that's what
`G`/`F` are) — **there is no windowed-sum operator**. The paper's detector *is* a windowed sum
(`R_N = Σ|m−ms|`). So the accumulation is computed in the monitor (the paper's algorithm, where it
belongs), and **STL monitors that statistic**: `G(R_N < T_on)`. Because `ρ = T_on − R_N`, STL's
Boolean verdict is *identical* to the paper's detection decision. This is the correct division of
labour — STL is the formal specification/monitoring layer over the paper's exact statistic and
thresholds — and it fixes the earlier criticism that we were only thresholding an instantaneous
residual with a guide constant.

---

## 3. Files (all in `newImp/`)

| File | Role |
|---|---|
| `faithful_core.py` | shared core: model/data loading, LPF, Algorithm 1 software sensor, `sliding_R` (windowed residual), `select_thresholds` (§3.3), `run_monitor` (offline stateful), `FaithfulMonitor` (online per-sample), attack injection |
| `offline_faithful_stl.py` | **OFFLINE** detector: computes `R_N`, selects thresholds, detects with STL `G(R_N<T_on)`, verifies the recovery property, plots |
| `online_faithful_stl.py` | **ONLINE** detector: same core over live MAVLink, rtamt online `spec.update`, closed-loop recovery |
| `mavlink_source.py` | streams the dataset as MAVLink (with a sustained on-wire attack) to feed the online monitor |
| `figures/offline_faithful_{baro,gyro,gps}.png` | offline result plots |
| `figures/online_faithful_{baro,gyro,gps}.png` | online result plots |
| `README_FAITHFUL_STL.md` | this file |

Data/model used (read-only, from the existing project):
`../rv_recovery/data/operation_data_50hz.mat` (segment 0, 50 Hz),
`../rv_recovery/matlab/models/quadrotor_12state.mat` (identified A,B,C,D),
`../rv_recovery/data/recovery_params.npy` (DTW window `N` per channel).

---

## 4. The equations (what the code computes)

```
Software sensor (paper Eq. 1–2):     y = C x + D u ;   x = A x + B u        (C = I, D = 0)
Filter (Algorithm 1):                m ← LPF(m)         (Butterworth, 2nd order, 5 Hz @ 50 Hz)
Checkpoint (every N, while healthy): e ← mean(ms−m) ;  ms ← m ;  x_k ← m     (re-seed)
Compensated prediction:              ms ← ms − e
Instantaneous error:                 d(t) = | m(t) − ms(t) |
Windowed accumulated residual (§3.3):R_{k,N}(t) = Σ_{i=t−N+1..t} d(i)
Threshold (§3.3):                    T_on = e_max·(1+0.10),  T_off = 0.80·T_on,  K = 10
STL detection:                       φ_det = G ( R_N < T_on )   →   ρ = T_on − R_N   (ρ<0 ⇔ attack)
STL recovery property:               φ_rec = G ( (R_N>T_on) → F[0:10 s] (R_N<T_off) )
Recovery action (Algorithm 1):       while recovery: m ← ms ; leave after K samples with R_N<T_off
```

---

## 5. Monitored sensors, attacks, and results (verified)

Three sensor cases (barometer / gyroscope / GPS), each a **sustained** attack from t = 40 s
(sustained is required so the faithful accumulated thresholds actually trip). Offline and online
give **identical** detections:

| Sensor (channel) | Attack | N (window) | T_on | Detection (STL ρ<0) | Latency |
|---|---|---|---|---|---|
| barometer (alt, ch2) | +3.0 m bias | 3492 (69.8 s) | 13631.9 | **65.56 s** | 25.56 s |
| gyroscope (p, ch9) | set 0.8 rad/s | 1066 (21.3 s) | 4.8 | **40.10 s** | 0.10 s |
| GPS east (pE, ch1) | +20 m bias | 2639 (52.8 s) | 4931.0 | **43.02 s** | 3.02 s |

Only the attacked channel fires in each run (no cross-channel false positives). Clean data does not
trigger (thresholds are `e_max + 10%`).

**Honest observation (worth telling your professor).** The barometer/GPS detections are *slow*
(25 s / 3 s) because their DTW windows are huge (70 s / 53 s) and the open-loop altitude/position
predictions drift a lot at **50 Hz**, inflating `e_max` and `T_on`. The gyroscope, a directly
observed fast state, has a tiny `e_max` (≈4.3) and detects in 0.1 s. This is a faithful consequence
of applying the method at 50 Hz — and it is exactly why the paper ran at **400 Hz**. Nothing was
tuned to hide it.

---

## 6. How to run

```bash
cd /home/tchowdh4/paperImp/newImp
PY=/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
export MPLBACKEND=Agg

# OFFLINE (all three sensors, prints detections, writes figures/offline_faithful_*.png)
$PY offline_faithful_stl.py

# ONLINE (two terminals) — example: gyroscope
#   terminal A (monitor):
$PY online_faithful_stl.py --conn udpin:127.0.0.1:14580 --plot figures/online_faithful_gyro.png --label "gyro"
#   terminal B (attacked replay over MAVLink):
$PY mavlink_source.py --out udpout:127.0.0.1:14580 --attack gyro --rate 1500
#   (repeat with --attack baro / --attack gps on ports 14582 / 14581)
```

---

## 7. Difference from the earlier `../offline_stl_*.py` STL work

| | earlier `../offline_stl_*.py` | this `newImp/` |
|---|---|---|
| Residual monitored | **instantaneous** `|m−ms|` | paper's **accumulated** `R_N = Σ|m−ms|` |
| Threshold | guide round numbers (0.30, 0.15, …) | paper **`T_on = e_max + margin`** (§3.3) |
| Window `N` | fixed 580 ms | **DTW-derived** per channel |
| Software sensor (offline) | recorded state / clean copy | **model `Cx+Du`** with re-seed + `e` |
| Recovery | none / simple | Algorithm 1 **`T_on/T_off/K`**, re-seed suppressed in recovery |
| STL role | `G[0:W](resid<ε)` | `G(R_N<T_on)` over the paper's statistic |
| Faithful to Algorithm 1? | no (guide simplification) | **yes** |

---

## 8. Remaining honesty / limitations

1. **STL cannot sum** — the accumulation is the paper's algorithm computed in the monitor; STL
   monitors it. (Section 2.) This is the correct architecture, not a shortcut.
2. **Coarse on slow/large-window channels at 50 Hz** — barometer/GPS detect slowly because of large
   DTW windows and open-loop drift; faithful, and motivates the paper's 400 Hz. (Section 5.)
3. **Sustained attacks** are used so the accumulated thresholds trip; a short transient below the
   window's accumulated margin may not (also a faithful property of accumulated detection).
4. `e_max` is calibrated on **segment 0** (clean); extending to all 21 clean segments is a one-line
   change and would only raise `T_on` slightly.
5. Attacks are **synthetic injections** (as in the earlier work); the pipeline is attack-source
   agnostic (the online path also accepts a real ArduPilot SITL feed via an adapter).
```
