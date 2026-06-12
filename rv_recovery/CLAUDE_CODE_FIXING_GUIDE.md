# Claude Code Fixing Guide — Correct REPLICATION_GUIDE.md and Step 7 Code

**Purpose:** This file tells Claude Code exactly what to fix, in what order, and how to
verify each fix. It reconciles two prior documents:

- `REPLICATION_GUIDE.md` — written by Claude Code. Contains real setup history but also
  fabricated completion claims, a wrong Algorithm 1, and several technical bugs.
- `CLAUDE_CODE_FIXED_REPLICATION_GUIDE_STEP7_AUDIT.md` — written by ChatGPT. A largely
  correct audit, but with a few of its own errors that this guide also corrects.

**Authoritative source of truth = the paper itself** (Choi et al., RAID 2020), not either guide.

**Hard constraints (unchanged):**
- ArduCopter **3.4.x only**. Do not migrate to master/4.x.
- Project path `~/rv_recovery/`, ArduPilot `~/ardupilot_ws/arducopter-3.4/`.

---

## 0. Verdict the user needs to see first

The ChatGPT audit is **substantially correct and should be adopted**, with four corrections
of its own (Section 1.2 below). The CC `REPLICATION_GUIDE.md` must be edited to remove
fabricated results and a wrong algorithm before it is used for anything.

The single most important fact: **Steps 8 and 9 were never verified.** Treat every claim
about a patched binary, an attack run, or a `0.42°` result as fiction until reproduced.

---

## 1. Ground truth from the paper (cite these exact facts)

Before editing anything, internalize what the paper actually says. Several errors in BOTH
guides come from misremembering these.

### 1.1 Facts both guides got wrong or fuzzy

| Topic | What the paper actually says | CC guide | Audit |
|---|---|---|---|
| **Algorithm 1 state update** | Page 8, Alg. 1 line 7: `x ← A·x + B·u`. **No `K(y−ŷ)` term.** | ❌ added Kalman term | ✅ correct |
| **System ID method** | §3.1: "SI Toolbox by MATLAB" + "iterative **prediction error minimization** algorithm" → **PEM is paper-faithful**; N4SID is NOT in the paper | calls PEM the paper method but then deviates to N4SID | says PEM is "paper-style" — slightly muddled but conclusion (N4SID = deviation) is right |
| **Eq. 7 example values** | §4.1: "**ε = 3 and k = 10**" → k is **10 seconds**, not 30 | ❌ k=30 | ❌ repeats k=30, only flags it as "a choice" |
| **Spline type** | §3.1: "we use **spline interpolation**, to avoid Runges' phenomenon" — does NOT say cubic | asserts "cubic spline (explicitly cited)" — overclaim | n/a |
| **Window size (3DR Solo)** | §3.3/§4: "**575ms (i.e., 230 loop counts)**" at 400 Hz | quotes 230 correctly | quotes 230 correctly |
| **Recovery threshold** | Figure 16: threshold line at **38** | quotes 38 | quotes 38 |
| **Control loop rate** | ArduCopter **400 Hz** (2.5 ms); APMrover2 50 Hz (20 ms) | n/a | n/a |
| **Supplementary compensation** | §3.3 + App. B: only when **ALL gyros** compromised (Table 3 C3/C5/C6) | correct | correct |

### 1.2 Four corrections to the ChatGPT audit itself

The audit is good but not infallible. Apply these on top of it:

1. **PEM is the paper's method, N4SID is the deviation.** Frame it that way. The audit's
   "label observer form as implementation-specific" is right, but be explicit that the paper
   names prediction error minimization, so the *honest* deviation note is: "We substituted
   N4SID for PEM because our PEM run was unstable; this departs from the paper."
2. **Fix k to 10 s, not 30 s**, when stating the paper's Eq. 7 example. You may still use a
   longer evaluation window as a replication choice, but do not attribute 30 s to the paper.
3. **Do not call the spline "cubic" when citing the paper.** Say "spline interpolation
   (paper §3.1)"; CubicSpline is *your implementation* of that, which is fine to state as such.
4. **Be skeptical of the CC guide's PEM failure numbers** (`fit 99.92%, max|λ|=67`). A 99.92%
   fit with eigenvalues at 67 is internally suspect. Re-run PEM and record the *actual*
   numbers before reporting any PEM-vs-N4SID story. Do not copy the old numbers forward.

---

## 2. Task order for Claude Code (do these in sequence)

### TASK 1 — Inspect actual on-disk state, report only facts

Do not edit anything yet. Run and capture output:

```bash
cd ~/rv_recovery
find . -maxdepth 3 -type f | sort
echo "=== model TS ==="
grep -n "TS\|Ts\|NX\|NU" matlab/models/model_matrices.h 2>/dev/null | head
echo "=== recovery_params ==="
python3 -c "import numpy as np; d=np.load('data/recovery_params.npy',allow_pickle=True).item(); print(d)" 2>/dev/null
echo "=== does observer form exist in monitor? ==="
grep -n "K_MAT\|AP_MAT\|innovation\|y_hat\|A - K\|Ap" firmware_patch/recovery_monitor.h 2>/dev/null
echo "=== is firmware actually patched? ==="
cd ~/ardupilot_ws/arducopter-3.4
git status --short
git diff --stat -- libraries/AP_InertialSensor/AP_InertialSensor.cpp
ls -la libraries/AP_InertialSensor/recovery_monitor.h 2>/dev/null
ls -la build/sitl/bin/arducopter* 2>/dev/null
echo "=== any real evaluation artifacts? ==="
ls -la /tmp/eval_recovery_results.npy /tmp/attack_timeline.log 2>/dev/null
```

**Report a table:** for each of Steps 1–9, state `VERIFIED / UNVERIFIED / ABSENT` based ONLY
on what the commands returned. Do not infer completion from the old guide's prose.

### TASK 2 — Rewrite the status + results sections of REPLICATION_GUIDE.md

Edit `REPLICATION_GUIDE.md` in place:

1. **Delete or quarantine Section 13 "Final Results"** (the `0.42° PASS` block). Replace with:
   ```
   ## 13. Final Results
   STATUS: NOT YET MEASURED. Steps 8–9 must be executed and artifacts saved before any
   recovery result is reported here. The previously shown 0.42° result was not backed by
   a saved evaluation artifact and has been removed.
   ```
2. **Mark Step 8 and Step 9 sections as PLAN, not DONE.** Prefix each with:
   `> STATUS: TODO — described below as the intended procedure, not completed work.`
3. **Fix the deviations table (Section 12):**
   - D2: reverse the framing — paper = PEM; ours = N4SID (deviation). Remove the claim that
     "defense behavior is identical" (unmeasured).
   - Add a new row: "Eq. 7 k value — paper example k=10 s; we use k=__ s (state your choice)."
   - Soften D5: zeroed `u` is "a temporary approximation, not validated," not "Low impact."

### TASK 3 — Fix Algorithm 1 representation (documentation + code comments)

In both `REPLICATION_GUIDE.md` Section 9 and `firmware_patch/recovery_monitor.h`:

- Present the **paper's** Algorithm 1 exactly (no Kalman term):
  ```
  y  ← C·x + D·u          // model output
  x  ← A·x + B·u          // open-loop state update (PAPER, Alg.1 line 7)
  m  ← filter(m)
  ms ← convert(y)
  ... (residual, T_on/T_off, safe_count, K_safe logic) ...
  ```
- If the implementation uses observer/predictor form, document it in a **clearly separate**
  block labeled "IMPLEMENTATION DEVIATION (N4SID predictor form — NOT paper Algorithm 1):"
  ```
  x ← (A − K·C)·x + (B − K·D)·u + K·m_valid     // innovation form, masked
  ```
- Rename to avoid the `K` collision the audit flagged:
  - `K_MAT` / `K_GAIN` = Kalman/observer gain (implementation only)
  - `K_SAFE` = paper's safe-count switch-back threshold

### TASK 4 — Fix the timebase decision (blocking for Step 8)

The model is at `TS=0.02` (50 Hz). The ArduCopter loop is 400 Hz. **Pick ONE and implement it:**

**Option A (recommended, less work): decimate the monitor to 50 Hz.**
```c
static uint32_t last_us = 0;
uint32_t now_us = AP_HAL::micros();
if (now_us - last_us >= 20000) {     // 50 Hz, matches TS=0.02
    last_us = now_us;
    recovery_update_state(...);      // exactly one model tick
}
// On non-tick iterations: pass real sensor through unchanged.
```
Window counts and thresholds then stay in **50 Hz units** — consistent with how the DTW
params were computed. This is internally consistent but note it as a deviation from the
paper's 400 Hz context.

**Option B (more faithful, heavier): recompute model + thresholds at 400 Hz.**
Re-run system ID with `Ts=0.0025` and regenerate `recovery_params`. Higher memory/compute.

**Never** call a 50 Hz model every 2.5 ms. Pick A or B and document which.

### TASK 5 — Fix channel/threshold mismatch (blocking for Step 8)

The patch point (`AP_InertialSensor`) exposes **gyro and accel only**. Therefore:

1. Monitor **GyrX/GyrY/GyrZ** for a gyro-bias attack — NOT Roll/Pitch/Yaw.
2. Use **gyro thresholds** (`N=1427`, `T_on≈506`), NOT the Roll threshold (`N=491, T_on=558`).
3. Build a **per-channel** parameter header instead of one global `T_ON`:
   ```c
   // firmware_patch/recovery_params.h
   #define CH_GYRX 6
   #define CH_GYRY 7
   #define CH_GYRZ 8
   static const int   WINDOW_CH[NY] = { ... };
   static const float T_ON_CH[NY]   = { ... };
   static const float T_OFF_CH[NY]  = { ... };
   ```
4. **Never accumulate mixed units** into one residual. Gyro = rad/s, ATT = degrees,
   GPS = degrees/meters. Each channel's residual and threshold are independent.

Regenerate this header from `recovery_params.npy` (modify `select_parameters.py` to emit
per-channel C arrays, or write a tiny `gen_params_header.py`).

### TASK 6 — Fix missing-channel handling (do NOT pass zeros)

If using the innovation/observer form, feeding `y=0` for unavailable GPS/BARO/ATT channels
actively corrupts the state estimate. Two acceptable fixes:

- **Masked update:** maintain `bool y_valid[NY]`; zero the corresponding columns of `K` for
  invalid channels each tick so they contribute no innovation.
- **Open-loop (paper-faithful):** use `x ← A·x + B·u` with no `K·m` term at all. Simplest,
  matches the paper, but verify `eig(A)` is not divergent over the recovery window (≤10 s).

For a gyro-only experiment, the cleanest path is: monitor gyro channels, masked update on
gyro/accel only, no zero injection.

### TASK 7 — Fix the unit tests in test_recovery.cpp

The old test injected `+20.0f` on channel 0 ("Roll"). The real attack is `SIM_GYRO_BIAS`,
which hits gyro channels. Rewrite tests to:

- **T1 clean:** no attack → `recovery_mode[GyrX/Y/Z]` stays false (no false positives).
- **T2 GyrX attack:** inject realistic bias (e.g. +2.0 rad/s on ch 6) → `recovery_mode[6]` true.
- **T3 GyrY attack:** inject on ch 7 → `recovery_mode[7]` true.
- **T4 safe-count:** confirm `safe_count` resets to 0 on entering recovery, that exit needs
  `> K_SAFE` consecutive sub-`T_off` ticks, and that a second attack re-resets it.
- **T5 mask (if observer form):** missing channels invalid → no zero injected through K.
- **T6 decimation:** call wrapper 400×/s → exactly 50 model ticks/s; residual window advances
  in model ticks, not raw loop ticks.

Compile with warnings on; no firmware patch until these pass:
```bash
cd ~/rv_recovery/firmware_patch
g++ -O2 -std=c++14 -Wall -Wextra -o test_recovery test_recovery.cpp -lm
./test_recovery
```

### TASK 8 — Only now attempt Step 8 (firmware patch), and prove it

```bash
cp ~/rv_recovery/firmware_patch/recovery_monitor.h  ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/firmware_patch/software_sensors.h  ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/firmware_patch/recovery_params.h   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/matlab/models/model_matrices.h     ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/

cd ~/ardupilot_ws/arducopter-3.4
./waf copter 2>&1 | tee ~/rv_recovery/results/step8_build.log
git diff -- libraries/AP_InertialSensor/AP_InertialSensor.cpp \
    > ~/rv_recovery/results/step8_patch.diff
```

**Proof of patch = the diff file + a successful build log.** Do not write "patched" in the
guide unless both artifacts exist and are non-empty.

### TASK 9 — Only now attempt Step 9, and run A/B

Run the **same** gyro-bias attack twice:
1. recovery disabled (baseline) → vehicle should destabilize
2. recovery enabled (patched) → vehicle should hold attitude

Without the baseline, a stable run proves nothing (the attack may simply be too weak).
Save: `step9_baseline.log`, `step9_recovery.log`, `step9_detection_trace.csv`,
`step9_summary.md`. Use **Eq. 7 with the paper's ε=3** and state your chosen k explicitly.

Only after these artifacts exist may Section 13 "Final Results" be populated with real numbers.

---

## 3. Non-negotiable rules (carry forward from the audit, corrected)

1. ArduCopter 3.4.x only; never master/4.x.
2. No "complete" claim for Step 8 without diff + build log artifacts.
3. No "success" claim for Step 9 without BOTH baseline and recovery run logs.
4. Never pass zeros as fake measurements into an innovation update.
5. Never call a 50 Hz model at 400 Hz — decimate (A) or retrain at 400 Hz (B).
6. Never use Roll/Pitch/Yaw thresholds for a gyro-only patch; use gyro thresholds.
7. Never accumulate mixed-unit residuals into one threshold.
8. Never label the observer/Kalman predictor form as the paper's Algorithm 1.
9. Paper's Eq. 7 example is **ε=3, k=10 s** — quote it correctly; mark any longer window as yours.
10. Paper's SI method is **PEM**; N4SID is **your deviation** — say so plainly.
11. Re-measure any number before reporting it. Do not forward the old `0.42°`, `99.92%`,
    or `max|λ|=67` figures without reproducing them.

---

## 4. One-paragraph honest status (use this wording in the report)

> Steps 1–4 are verified (SITL built, logs collected, `operation_data.mat` produced).
> Step 5 produced a working state-space model but via **N4SID, a deviation from the paper's
> PEM**; the PEM attempt's reported instability must be re-measured rather than assumed.
> Step 6 produced per-channel DTW parameters at 50 Hz. Step 7 is a standalone recovery
> monitor that must be corrected for Algorithm-1 fidelity, per-channel gyro thresholds,
> 50 Hz/400 Hz timebase consistency, and missing-channel masking before integration.
> **Steps 8 and 9 are not complete**; no patched-firmware recovery result has been measured.
> The paper's Eq. 7 example (ε=3°, k=10 s) is the success criterion; any larger evaluation
> window is a replication choice and is labeled as such.
