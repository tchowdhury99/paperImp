# STL Implementation — Presentation Walkthrough (for the professor)

**Project directory:** `/home/tchowdh4/paperImp/`
**Paper implemented:** Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles* (RAID 2020) — `SbRRfSAoRV.pdf`
**STL tooling source:** SpecGuard (`/home/tchowdh4/SpecGuard_Implementation/…pdf`) — we reuse its idea of monitoring a vehicle against a formal specification, implemented with the `rtamt` STL library.
**Python interpreter (must use):** `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`
**Date:** 2026-07-01

> This document is a guided tour you can present top-to-bottom. It explains **the dataset**, **the
> STL method**, **every figure**, and **every code file — pointing to the exact lines that
> implement STL** — plus a suggested talking track and how to run each piece live.

---

## 0. One-paragraph summary to open with

> "I reproduced Choi et al.'s sensor-attack detection idea and added a **formal Signal Temporal
> Logic (STL) monitoring layer** on top of it. STL lets me write each safety/integrity property
> as a temporal-logic formula and get a **robustness value ρ** at every instant: ρ > 0 means the
> property holds, ρ < 0 means it is violated (an attack). I built **8 offline STL monitors** on
> recorded 50 Hz flight data, and then a **real-time STL monitor over MAVLink** that also triggers
> the paper's closed-loop recovery. Everything runs and is reproduced in the figures."

---

## 1. Suggested presentation flow (what to show, in order)

| Step | Show | File(s) |
|---|---|---|
| 1 | The dataset & where it comes from | `rv_recovery/data/operation_data_50hz.mat` |
| 2 | What STL is + robustness ρ | this doc §3; `STL_IMPLEMENTATION_GUIDE.md` §2–4 |
| 3 | Barometer detection figure + its code | `stl_result_baro.png` + `offline_stl_baro.py` |
| 4 | GPS, gyro, multi-sensor figures | the corresponding `stl_result_*.png` |
| 5 | Temporal specs (persistent + recovery) | `stl_result_baro_persistent.png`, `_recovery.png` |
| 6 | Altitude-bounds (mission spec) figure | `stl_result_altitude_bounds.png` |
| 7 | **Real-time STL over MAVLink + recovery** | `online_stl_mavlink.py` + `stl_result_online_*.png` |
| 8 | Paper-alignment correction (rigor) | `STL_PAPER_ALIGNMENT_CORRECTION_REPORT.md` |

---

## 2. The dataset

**Location:** `/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat`
(21.5 MB; there is also a larger raw `operation_data.mat` and the ArduPilot log `sitl_run1/logs/4.BIN`.)

**What it is:** real ArduPilot SITL quad-copter flight logs, resampled by cubic spline to a single
**50 Hz** rate (Ts = 0.02 s), split into **21 clean flight segments**. All STL work uses
**segment 0** (the longest, ~140 s, ~6990 samples). It is the same data used for the system
identification that produced the software-sensor model.

**Structure (MATLAB struct of cell arrays):**

| Variable | Shape | Channels (index: name) |
|---|---|---|
| `Yseg[i]` | (N, 12) | 0:pN 1:pE **2:alt** 3:phi 4:theta 5:psi 6:vN 7:vE 8:vUp **9:p 10:q 11:r** |
| `Useg[i]` | (N, 7) | control inputs: phi_cmd theta_cmd psi_cmd thr tiltN tiltE const |
| `EXTRAseg[i]` | (N, 9) | 0:BARO_Press **1:BARO_Alt** 2-4:Mag 5:GPS_Lat 6:GPS_Lng 7:GPS_Alt 8:GPS_Spd |

**Channels we actually monitor with STL:** `alt = Y[:,2]` (model/estimate altitude AGL),
`BARO_Alt = EXTRA[:,1]` (barometer altitude AGL), `p,q,r = Y[:,9:12]` (gyro body rates),
`pN,pE = Y[:,0:2]` and `GPS_Lat/Lng = EXTRA[:,5:7]` (position).

**Important dataset facts to mention** (from `STL_IMPLEMENTATION_GUIDE.md` §1):
- `GPS_Alt` is **MSL** (~1600 m, Colorado) not AGL → we use `alt`/`BARO_Alt` for altitude.
- Barometer pressure has a local ground-pressure offset → we use `BARO_Alt` (already AGL) directly.
- Model-vs-barometer residual on clean data: mean ≈ 0, std ≈ 0.064 m, max ≈ 0.23 m → this is why
  the barometer threshold ε = 0.30 m is meaningful (just above clean noise).

**How the code loads it** (identical pattern in every script), e.g. `offline_stl_baro.py`:
```python
d = scipy.io.loadmat('.../operation_data_50hz.mat')
Y   = d['Yseg'][0][0]      # segment 0, shape (N,12)
EXTR= d['EXTRAseg'][0][0]  # segment 0, shape (N,9)
alt      = Y[:, 2]         # model altitude
baro_alt = EXTR[:, 1]      # barometer altitude
```

---

## 3. The STL method (what to explain before the figures)

**Signal Temporal Logic** describes properties of a time-series signal. We use the `rtamt` library.

- **Atomic predicate:** `x < c`. Robustness `ρ = c − x` (how far below the bound).
- **G[a,b] φ (Globally / Always):** φ holds at every time in the window; `ρ = min` over the window.
- **F[a,b] φ (Finally / Eventually):** φ holds somewhere in the window; `ρ = max`.
- **→ (implies), and, or, not:** Boolean combinations (min/max/negate on robustness).
- **Robustness ρ:** `ρ > 0` satisfied, `ρ < 0` violated, `ρ = 0` on the boundary. **ρ crossing 0 = attack detected.**

**The residual we feed into STL** is the paper's (Choi et al., Algorithm 1) sensor error:
```
e_sensor(t) = | m_sensor(t) − ms_sensor(t) |
   m_sensor  = actual physical measurement (corrupted only during a simulated attack)
   ms_sensor = software-sensor / model prediction
```
Dataset mapping: `m_baro=BARO_Alt, ms_baro=alt`; `m_gps=GPS pos, ms_gps=model pN/pE`;
`m_gyr=measured rate, ms_gyr=model rate state`. (See `STL_PAPER_ALIGNMENT_CORRECTION_REPORT.md`.)

**Guide-based simplification to disclose honestly:** the paper accumulates the residual over a
window (`r ← r + |m−ms|`); the STL guide simplifies this to the **instantaneous** `|m−ms|` inside a
short `G[0:W]` window. We kept the guide's instantaneous form and documented it.

**rtamt units:** windows are milliseconds; at 50 Hz, `580ms` = 29 samples (the guide's 575 ms
rounded to a 20 ms multiple), `2000ms` = 100 samples, `10000ms` = 500 samples.

---

## 4. The figures — one section each (location + how to read them)

All plots are in `/home/tchowdh4/paperImp/`. Each offline figure has **three stacked panels**:
**(top)** the sensor signal clean vs attacked, **(middle)** the residual `|m−ms|` vs the threshold ε,
**(bottom)** the STL robustness ρ with the ρ = 0 line and the detection instant.

> Reading tip for the professor: "Follow the red dashed line down through the panels — when the
> residual (middle) crosses ε, the robustness ρ (bottom) crosses 0, and that is the detection."

### 4.1 `stl_result_baro.png` — Barometer Integrity
- **Spec:** `G[0:2000ms] (baro_res < 0.30)`; ε = 0.30 m; attack = BARO_Alt **+3.0 m** over 40–50 s.
- **Result:** ρ goes negative → **barometer attack detected at t = 38.00 s**.
- **Talking point:** the residual is ~0 while clean (model tracks baro to <0.23 m), then jumps to ~3 m under attack; ρ = 0.30 − residual goes sharply negative.

### 4.2 `stl_result_gps.png` — GPS Integrity
- **Spec:** `G[0:580ms] (gps_north_res < 0.169349 and gps_east_res < 0.169349)`; attack = GPS East **+20 m**.
- **Result:** **detected at t = 39.42 s** (east residual breaks the bound).

### 4.3 `stl_result_gyro.png` — Gyroscope Integrity
- **Spec:** `G[0:580ms] (gyro_residual_x<0.15 and _y<0.15 and _z<0.15)`; ε = 0.15 rad/s; attack = roll-rate **set to 0.8 rad/s**.
- **Result:** **detected at t = 39.42 s**.

### 4.4 `stl_result_multi_sensor.png` — Multi-Sensor Compound Spec
- **Spec:** one big AND over altitude bounds + baro + GPS(N/E) + gyro(x/y/z). A single ρ for the whole vehicle's health.
- **Result:** **detected at t = 39.42 s** (whichever sensor breaks first pulls the min-robustness negative).

### 4.5 `stl_result_baro_persistent.png` — Persistent Barometer Pattern
- **Spec:** `G[0:580ms] (G[0:1000ms] (baro_residual < 0.30))` — the residual must stay low over a **sustained 1 s** inner window (reduces false positives from momentary spikes).
- **Result:** **detected at t = 38.42 s**. Demonstrates STL's *temporal* power beyond a plain threshold.

### 4.6 `stl_result_baro_recovery.png` — Barometer Recovery within 10 s
- **Spec:** `G[0:580ms] ((baro_residual > 0.30) -> F[0:10000ms] (baro_residual < 0.30))` — "whenever an attack appears, it must clear within 10 s."
- **Result:** **no violation** — the (transient) attack clears within 10 s, so the recovery property holds. This encodes the paper's recovery-time requirement (Eq. 7 idea).

### 4.7 `stl_result_any_attack_recovery.png` — Any-Attack Recovery
- **Spec:** `G[0:580ms] (((baro_residual>0.30) or (gyro_residual_x>0.15)) -> F[0:10000ms] (not(...)))` — if *either* baro or gyro is attacked, the system must recover within 10 s.
- **Result:** **no violation** (recovers in time).

### 4.8 `stl_result_altitude_bounds.png` — Altitude Bounds (mission spec S3/S4)
- **Spec:** `G[0:580ms] ((alt > 0.97) and (alt < 29.70))` — altitude must stay in the flight envelope. **This is a state-bound spec, not a sensor residual.**
- **Result:** **no violation** (clean flight stays in bounds). This is the direct answer to "monitor the drone so it does not exceed an altitude threshold."

### 4.9 Online / MAVLink figures (the live demo)
- `stl_result_online_baro.png` — baro spoof over MAVLink → **ALT detected at 40.00 s**, then recovery engages (baro replaced by the estimate) and releases after the attack ends.
- `stl_result_online_gyro.png` — gyro spoof over MAVLink → **P detected at 40.02 s**, recovery engages/releases.
- `stl_result_online_clean.png` — no attack → **no false positives** on either channel.
- These have **four panels**: sensor (measured vs recovered), residuals vs ε, robustness ρ, and the **recovery on/off** state. Point at the bottom panel: "ρ<0 flips recovery ON; K safe samples later it flips OFF — that is the closed loop."

---

## 5. The code — where STL is actually implemented

**Location of all code:** the offline monitors are the 8 `offline_stl_*.py` files directly in
`/home/tchowdh4/paperImp/`; the live monitor is `online_stl_mavlink.py` + `mavlink_replay_source.py`
there too. Each offline script has the **same 4-stage skeleton**: (1) load data, (2) build residual
`|m−ms|` + simulate attack, (3) **define & evaluate the STL spec ← this is the STL part**, (4) plot.

### 5.1 The exact "STL lines" per offline script (point the professor here)

For every script the STL implementation is the block: **declare variables → set 50 Hz sampling →
assign the STL formula string → parse → evaluate robustness**.

| Script | Residual `|m−ms|` (line) | Threshold (line) | STL spec object + formula (lines) | Evaluate ρ (line) | Output plot |
|---|---|---|---|---|---|
| `offline_stl_baro.py` | 49, 63 | `0.30` in formula (77) | `70` spec, `72–73` declare, `75` 20 ms, **`77` `G[0:2000ms](baro_res<0.30)`**, `79` parse | `89` `spec.evaluate` | `stl_result_baro.png` (163) |
| `offline_stl_gps.py` | 119–130 | `EPS_GPS=0.169349` (29) | `137` spec, `138–139` declare, `140` 20 ms, **`142` formula**, `143` parse | `166–167` evaluate | `stl_result_gps.png` (226) |
| `offline_stl_gyro.py` | 127–133 | `EPS_GYR=0.15` (46) | `140–142` declare, **`146–150` formula**, `144` 20 ms | `spec.evaluate` (in `evaluate_offline`) | `stl_result_gyro.png` |
| `offline_stl_multi_sensor.py` | 134–141 | `EPS_* ` (32–34) | **`160–166` compound formula** | `spec.evaluate` | `stl_result_multi_sensor.png` |
| `offline_stl_baro_persistent.py` | 77 | `EPSILON_BARO=0.30` (20) | `82` declare, **`84–87` nested `G[0:580ms](G[0:1000ms](…))`**, `91` parse | `98` evaluate | `stl_result_baro_persistent.png` (163) |
| `offline_stl_baro_recovery.py` | 136–137 | `EPS_BARO=0.30` (28) | `148` declare, **`153–155` `…-> F[0:10000ms]…`**, `158` parse | `92`/`101` evaluate | `stl_result_baro_recovery.png` |
| `offline_stl_any_attack_recovery.py` | 189–193 | `EPS_BARO,EPS_GYR` (39–40) | `65–66` declare, **`45` `STL_FORMULA`, `74` assign**, `75` parse | `126`/`137` evaluate | `stl_result_any_attack_recovery.png` |
| `offline_stl_altitude_bounds.py` | — (state bound) | `0.97 / 29.70` (26–27) | `45` declare, **`31` `STL_FORMULA`, `52` assign**, `53` parse | `98`/`108` evaluate | `stl_result_altitude_bounds.png` (201) |

**Canonical STL block to show on screen** (from `offline_stl_baro.py`, lines 70–89):
```python
spec = rtamt.StlDiscreteTimeSpecification()      # 70  create STL monitor
spec.declare_var('alt', 'float')                 # 72  signals used
spec.declare_var('baro_res', 'float')            # 73  residual = |m_baro - ms_baro|
spec.set_sampling_period(int(Ts*1000), 'ms')     # 75  50 Hz -> 20 ms
spec.spec = 'G[0:2000ms] (baro_res < 0.30)'      # 77  <-- THE STL FORMULA
spec.parse()                                     # 79  compile
...
out = spec.evaluate(dataset)                     # 89  <-- robustness trace ρ(t)
```
And the **paper-aligned residual** that feeds it (lines 46–63):
```python
ms_baro      = alt                 # software-sensor prediction   ms
m_baro_clean = baro_alt            # clean physical measurement    m
m_baro = m_baro_clean.copy()
m_baro[2000:2500] += 3.0           # OFFLINE attack simulation: corrupt m only in the window
baro_res_attacked = np.abs(m_baro - ms_baro)   # residual = |m - ms|  (paper Algorithm 1)
```

### 5.2 The real-time STL monitor — `online_stl_mavlink.py` (the strongest part)

This is where STL becomes a **live drone monitor with closed-loop recovery**. Key lines:

| What | Lines |
|---|---|
| Load identified software-sensor model A,B,C,D (`quadrotor_12state.mat`) | 44, 100–101 |
| Butterworth low-pass filter on the real measurement (paper `m ← filter(m)`) | 49–63 |
| **Build & parse the online STL spec** `resid < ε` (per channel) | **65–74**, `93–94` |
| MAVLink connect + handshake (`wait_heartbeat`) | 88–90 |
| MAVLink receive loop (`recv_match`) | 113 |
| **Software-sensor model step** `y=Cx+Du ; x=Ax+Bu` (paper Eq. 1–2) | **142–143** |
| Residual `|m−ms|` for alt and gyro | 147, 158 |
| **Online STL robustness** `spec.update(...)` → ρ per sample | **149, 165** |
| **Closed-loop recovery** (ρ<0 → `m←ms`; back after K safe samples) | ~150–175 |
| Save figure | end of `main()` |

**The line to emphasize:** `rho_alt = spec_alt.update(tick, [('resid_alt', resid_alt)])` (149) —
this is STL evaluated **in real time, one MAVLink sample at a time**, and the very next lines flip
the vehicle into recovery when ρ crosses 0.

**Companion `mavlink_replay_source.py`** streams the dataset as genuine MAVLink traffic (with the
attack injected on the wire) so the loop is reproducible; line 44 = dataset path, line 88 =
`named_value_float_send` (the MAVLink send). See `MAVLINK_STEPS.md` for the step-by-step MAVLink
story and the proof it also connects to a real ArduPilot SITL.

---

## 6. How to run everything live (copy-paste)

```bash
cd /home/tchowdh4/paperImp
PY=/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
export MPLBACKEND=Agg          # headless-safe (no display needed)

# ---- 8 offline STL monitors (each prints detection time + saves its PNG) ----
$PY offline_stl_baro.py
$PY offline_stl_gps.py
$PY offline_stl_gyro.py
$PY offline_stl_multi_sensor.py
$PY offline_stl_baro_persistent.py
$PY offline_stl_baro_recovery.py
$PY offline_stl_any_attack_recovery.py
$PY offline_stl_altitude_bounds.py

# ---- live MAVLink STL + recovery (two terminals) ----
# terminal A: the monitor listens
$PY online_stl_mavlink.py --conn udpin:127.0.0.1:14570 --plot stl_result_online_baro.png
# terminal B: replay the flight with a barometer attack
$PY mavlink_replay_source.py --out udpout:127.0.0.1:14570 --attack baro --offset 3.0 --rate 1000
```

Expected console: baro → detect 38.00 s (offline) / 40.00 s (online), gps/gyro/multi → 39.42 s,
persistent → 38.42 s, the two recovery specs + altitude bounds → no violation, online clean → no
false positive.

---

## 7. Honest points to raise (shows rigor)

1. **Instantaneous vs accumulated residual.** STL monitors `|m−ms|` per sample; the paper accumulates
   it (`r ← r + |m−ms|`). Ours follows the STL guide's instantaneous simplification — disclosed, not hidden.
2. **Offline "detection latency" prints negative** (e.g. baro −2.0 s). That is **not** early detection —
   the forward-looking `G[0:W]` window sees up to `t+W`, so the first ρ<0 lands at `attack_onset − W`.
   The **online** monitor is causal, so its latency is ≈ 0 (real detection at 40.0 s). Explained in
   `STL_WORK_EXPLAINED.md` §7.
3. **Gyro software-sensor.** Offline gyro uses the model rate state as prediction (clean residual ≈ 0
   because the dataset has no separate raw-gyro channel); the **online** monitor uses the true model
   output `(Cx+Du)[9]` — a genuine software-sensor prediction.
4. **Paper-alignment correction.** Residual variables were renamed to `|m_sensor − ms_sensor|`
   (Algorithm 1); math and results are unchanged — see `STL_PAPER_ALIGNMENT_CORRECTION_REPORT.md`.

---

## 8. Full file map (hand this to the professor)

**Dataset**
```
rv_recovery/data/operation_data_50hz.mat        # 50 Hz, 21 segments — the data all STL uses
rv_recovery/matlab/models/quadrotor_12state.mat # identified A,B,C,D software-sensor model (online)
```

**Offline STL code (STL implemented in the declare→spec→parse→evaluate block of each)**
```
offline_stl_baro.py  offline_stl_gps.py  offline_stl_gyro.py  offline_stl_multi_sensor.py
offline_stl_baro_persistent.py  offline_stl_baro_recovery.py
offline_stl_any_attack_recovery.py  offline_stl_altitude_bounds.py
```

**Online STL + recovery code**
```
online_stl_mavlink.py        # real-time STL monitor + closed-loop recovery (spec.update at line 149/165)
mavlink_replay_source.py     # MAVLink data source (dataset -> MAVLink, with attack injection)
```

**Figures**
```
stl_result_baro.png  stl_result_gps.png  stl_result_gyro.png  stl_result_multi_sensor.png
stl_result_baro_persistent.png  stl_result_baro_recovery.png
stl_result_any_attack_recovery.png  stl_result_altitude_bounds.png
stl_result_online_baro.png  stl_result_online_gyro.png  stl_result_online_clean.png
```

**Reports / docs (context for the professor)**
```
STL_IMPLEMENTATION_GUIDE.md            # STL background, operators, dataset facts, skeletons
STL_FINAL_REPORT.md                    # per-spec detail (now with §0 paper-aligned residual)
STL_COMPLETION_STATUS.md               # one-page list of the 8 specs
STL_WORK_EXPLAINED.md                  # completion %, deviations, the negative-latency explanation
MAVLINK_STEPS.md                       # the MAVLink part, step by step
STL_REMAINING_WORK_COMPLETED.md        # online monitor + recovery: what/why/equations
STL_PAPER_ALIGNMENT_CORRECTION_REPORT.md  # the residual naming correction (rigor)
stl_correction_backup/                 # pre-correction backups
STL_PRESENTATION_WALKTHROUGH.md        # <-- THIS FILE
```

---

## 9. If the professor asks "where exactly is the STL?"

Point to two lines:
- **Offline:** `offline_stl_baro.py:77` → `spec.spec = 'G[0:2000ms] (baro_res < 0.30)'`, evaluated at
  `offline_stl_baro.py:89` (`spec.evaluate`). Every other offline script is the same pattern.
- **Online (real-time):** `online_stl_mavlink.py:149` → `rho_alt = spec_alt.update(tick, [('resid_alt', resid_alt)])`,
  with recovery triggered on the next lines. That single line is STL running live on a MAVLink sample.
