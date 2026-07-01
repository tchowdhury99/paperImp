# Detailed Explanation of `README_FAITHFUL_STL.md`

**Source report:** `README_FAITHFUL_STL.md`  
**Project location described in source:** `/home/tchowdh4/paperImp/newImp/`  
**Topic:** Paper-faithful STL attack detection for Choi et al., *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles*  
**Purpose of this explanation:** Explain every part of the report, how each formula was formed, how each threshold was obtained, why each value was used, and how to interpret the results with examples.

---

## 1. What this report is about

The report describes a newer implementation under:

```text
/home/tchowdh4/paperImp/newImp/
```

This implementation is called **paper-faithful** because it tries to follow the Choi et al. paper's actual runtime detection logic instead of only using a simplified STL residual threshold.

The key difference is this:

```text
Earlier STL guide implementation:
    Monitor instantaneous residual:
    |m(t) − ms(t)| < ε

New paper-faithful implementation:
    First compute accumulated paper statistic:
    R_N(t) = Σ |m(i) − ms(i)| over the last N samples

    Then monitor it with STL:
    G(R_N < T_on)
```

So this report is not only about writing STL formulas. It is about combining:

1. the paper's software sensor,
2. the paper's accumulated residual,
3. the paper's threshold-selection rule,
4. the paper's recovery state machine,
5. STL monitoring over the paper's detection statistic.

---

## 2. Important terms

### 2.1 Physical sensor measurement: `m(t)`

`m(t)` means the measurement received from the actual sensor.

Examples:

```text
m_baro(t)    = physical barometer altitude measurement
m_gyr_x(t)   = physical gyro-x roll-rate measurement
m_gps_east(t)= physical GPS east-position measurement
```

If there is no attack, `m(t)` is clean.

If an attack is injected, `m(t)` becomes corrupted.

For example:

```text
barometer attack:
m_baro(t) = clean_BARO_Alt(t) + 3.0 m

gyro attack:
m_gyr_x(t) = 0.8 rad/s

GPS east attack:
m_gps_east(t) = clean_GPS_east(t) + 20 m
```

---

### 2.2 Software sensor prediction: `ms(t)`

`ms(t)` is the value predicted by the software sensor/model.

The report says the software sensor uses the identified state-space model:

```text
y = Cx + Du
x = Ax + Bu
```

In this implementation:

```text
C = I
D = 0
```

So the output prediction is essentially based on the model state.

Meaning:

```text
ms(t) = software/model prediction of what the sensor should read
```

---

### 2.3 Instantaneous error: `d(t)`

The first error quantity is the absolute difference between the physical measurement and the software-sensor prediction:

```text
d(t) = |m(t) − ms(t)|
```

This is the per-sample error.

Example:

```text
m(t)  = 13.0
ms(t) = 10.0

d(t) = |13.0 − 10.0|
d(t) = 3.0
```

This tells us that at that sample, the physical sensor and software sensor disagree by `3.0`.

---

### 2.4 Windowed accumulated residual: `R_N(t)`

The paper does not only check one sample at a time. It accumulates the last `N` instantaneous errors:

```text
R_N(t) = Σ_{i=t−N+1}^{t} d(i)
```

Expanded:

```text
R_N(t) = d(t−N+1) + d(t−N+2) + ... + d(t)
```

Because:

```text
d(i) = |m(i) − ms(i)|
```

we can write:

```text
R_N(t) = Σ_{i=t−N+1}^{t} |m(i) − ms(i)|
```

This is the paper's accumulated detection statistic.

---

## 3. Why the paper uses an accumulated residual

A single-sample residual may be noisy. For example, one sample may have a small spike because of noise, filtering delay, or model mismatch. The paper's method accumulates residuals across a window to detect sustained disagreement between the real sensor and the software sensor.

Simplified example:

```text
N = 5
d values over last five samples:
0.10, 0.12, 0.09, 0.11, 0.10

R_N = 0.10 + 0.12 + 0.09 + 0.11 + 0.10
R_N = 0.52
```

If the threshold is:

```text
T_on = 1.00
```

then:

```text
R_N = 0.52 < 1.00
```

No attack.

Now suppose an attack causes larger sustained errors:

```text
d values:
0.60, 0.70, 0.80, 0.75, 0.90

R_N = 0.60 + 0.70 + 0.80 + 0.75 + 0.90
R_N = 3.75
```

Now:

```text
3.75 > 1.00
```

Attack detected.

So the accumulated residual detects sustained disagreement.

---

## 4. Where STL fits

### 4.1 Important limitation of STL

STL temporal operators such as `G` and `F` are based on min/max semantics. They do not directly compute a summation like:

```text
Σ |m − ms|
```

Therefore, the implementation does this:

```text
Step 1: Compute R_N(t) in the monitor using the paper's algorithm.
Step 2: Give R_N(t) to STL as a signal.
Step 3: STL monitors whether R_N(t) remains below T_on.
```

So STL does not calculate the accumulated residual. The paper's algorithm calculates it. STL formally monitors the result.

This is the correct division of labor:

```text
Paper algorithm: compute detection statistic R_N
STL: formally specify and monitor condition R_N < T_on
```

---

### 4.2 STL detection formula

The paper's detection rule is:

```text
Attack if R_N(t) > T_on
```

The safe condition is the opposite:

```text
R_N(t) < T_on
```

The STL formula is:

```text
φ_det = G(R_N < T_on)
```

Meaning:

```text
Globally / always, the accumulated residual must stay below the attack threshold.
```

---

### 4.3 STL robustness

For the predicate:

```text
R_N < T_on
```

the robustness is:

```text
ρ = T_on − R_N
```

Interpretation:

```text
ρ > 0  → safe
ρ = 0  → exactly at threshold
ρ < 0  → attack / violation
```

This means:

```text
ρ < 0
```

is equivalent to:

```text
T_on − R_N < 0
```

which implies:

```text
R_N > T_on
```

That is exactly the paper's detection rule.

---

## 5. How the threshold was obtained

The paper-faithful implementation does not use fixed guide thresholds such as:

```text
0.30
0.15
0.169349
```

Instead, it uses the paper-style clean-data threshold calculation.

The report states:

```text
T_on = e_max · (1 + 0.10)
T_off = 0.80 · T_on
K = 10
```

---

### 5.1 What is `e_max`?

`e_max` is the maximum accumulated residual observed on clean data.

The process is:

```text
1. Run the monitor on clean data.
2. Compute d(t) = |m(t) − ms(t)|.
3. Compute R_N(t) = Σ d(i) over the last N samples.
4. Find the maximum R_N(t) over the clean run.
5. Call that maximum e_max.
```

Mathematically:

```text
e_max = max_t R_N(t) on clean data
```

---

### 5.2 Why add 10% margin?

The clean data already has model mismatch, filtering delay, sensor noise, and normal system variation.

If we set:

```text
T_on = e_max
```

then the system would trigger at the exact largest clean residual. That is too sensitive.

So the implementation adds 10% margin:

```text
T_on = e_max · (1 + 0.10)
T_on = 1.10 · e_max
```

This means the attack threshold is slightly above the largest clean residual.

---

### 5.3 Example threshold calculation

Suppose clean accumulated residual values are:

```text
R_N values:
15, 18, 21, 20, 17
```

Then:

```text
e_max = 21
```

Add 10% margin:

```text
T_on = 21 × 1.10
T_on = 23.1
```

So attack detection rule becomes:

```text
Attack if R_N > 23.1
```

STL formula becomes:

```text
G(R_N < 23.1)
```

Robustness:

```text
ρ = 23.1 − R_N
```

If:

```text
R_N = 20
ρ = 23.1 − 20 = +3.1
```

safe.

If:

```text
R_N = 30
ρ = 23.1 − 30 = −6.9
```

attack detected.

---

## 6. Why there are two thresholds: `T_on` and `T_off`

The report uses:

```text
T_on  = attack-entry threshold
T_off = recovery/exit threshold
```

The values are:

```text
T_on  = 1.10 · e_max
T_off = 0.80 · T_on
```

Because `T_off` is smaller than `T_on`, the monitor has hysteresis.

---

### 6.1 Why hysteresis matters

Without separate thresholds, the monitor may rapidly switch between attack and normal if the residual stays close to one threshold.

Example without hysteresis:

```text
Threshold = 100

R_N values:
99, 101, 100, 102, 98
```

This can cause repeated switching:

```text
99  → normal
101 → attack
100 → boundary
102 → attack
98  → normal
```

With hysteresis:

```text
T_on  = 100
T_off = 80
```

then:

```text
Enter attack mode when R_N > 100.
Leave attack mode only when R_N < 80.
```

This prevents unstable switching.

---

### 6.2 Example of `T_on` and `T_off`

Suppose:

```text
e_max = 200
```

Then:

```text
T_on = 1.10 × 200 = 220
```

Recovery threshold:

```text
T_off = 0.80 × 220 = 176
```

Detection:

```text
R_N > 220
```

Recovery:

```text
R_N < 176
```

So if:

```text
R_N = 230
```

attack mode starts.

If later:

```text
R_N = 190
```

it is below `T_on` but not below `T_off`, so recovery is not fully accepted yet.

If:

```text
R_N = 170
```

then:

```text
170 < 176
```

recovery condition is satisfied.

---

## 7. What `K = 10` means

The report says:

```text
K = 10
```

This means the system does not leave recovery mode after only one safe sample. It requires several safe samples.

The report states recovery leaves after:

```text
K samples with R_N < T_off
```

So with:

```text
K = 10
```

the monitor needs 10 consecutive safe samples before leaving recovery.

Because the dataset is 50 Hz:

```text
1 sample = 0.02 s
```

then:

```text
10 samples × 0.02 s = 0.20 s
```

So `K = 10` means about `0.20 s` of stable safe residual at 50 Hz.

---

## 8. What `N` means and where it came from

`N` is the residual accumulation window length.

The report says `N` is reused from:

```text
../rv_recovery/data/recovery_params.npy
```

and is DTW-derived per channel.

That means `N` was not randomly selected. It came from the paper/recovery parameter setup.

The formula is:

```text
R_N(t) = Σ_{i=t−N+1}^{t} |m(i) − ms(i)|
```

So if:

```text
N = 3492
```

the monitor sums the last 3492 instantaneous errors.

Because the data is 50 Hz:

```text
Ts = 0.02 s
```

the time duration is:

```text
window duration = N × 0.02 s
```

---

## 9. Sampling-time conversions

The report uses 50 Hz data.

```text
fs = 50 Hz
Ts = 1/fs = 1/50 = 0.02 s
```

So:

```text
1 sample = 0.02 s = 20 ms
```

For each channel:

### Barometer

```text
N = 3492
window = 3492 × 0.02
window = 69.84 s
```

The report rounds this to about:

```text
69.8 s
```

### Gyroscope

```text
N = 1066
window = 1066 × 0.02
window = 21.32 s
```

The report rounds this to about:

```text
21.3 s
```

### GPS east

```text
N = 2639
window = 2639 × 0.02
window = 52.78 s
```

The report rounds this to about:

```text
52.8 s
```

---

## 10. Software sensor equations

The report says the implementation builds the software sensor from the identified model:

```text
y = Cx + Du
x = Ax + Bu
```

These are state-space equations.

---

### 10.1 Meaning of `x`

`x` is the state vector.

For a quadrotor, the state can include quantities like:

```text
position
velocity
attitude
angular rates
```

The exact model is loaded from:

```text
../rv_recovery/matlab/models/quadrotor_12state.mat
```

---

### 10.2 Meaning of `u`

`u` is the input vector.

It represents control or input signals used by the state-space model.

---

### 10.3 Meaning of `A`, `B`, `C`, `D`

In a state-space model:

```text
x_next = A x + B u
y      = C x + D u
```

where:

```text
A = state transition matrix
B = input matrix
C = output matrix
D = direct feedthrough matrix
```

The report writes:

```text
y = Cx + Du
x = Ax + Bu
```

The usual discrete-time form is:

```text
x(k+1) = A x(k) + B u(k)
y(k)   = C x(k) + D u(k)
```

The report indicates:

```text
C = I
D = 0
```

So:

```text
y = x
```

for the selected output states.

---

## 11. Low-pass filter

The report says the monitor applies:

```text
m ← LPF(m)
```

using:

```text
Butterworth, 2nd order, 5 Hz @ 50 Hz
```

This means the raw physical measurement is filtered before the residual is computed.

Purpose:

```text
reduce high-frequency noise
make the physical measurement smoother
match the paper's monitoring pipeline
```

A second-order Butterworth low-pass filter with 5 Hz cutoff means:

```text
signals below 5 Hz mostly pass
signals above 5 Hz are attenuated
```

Since the data is 50 Hz:

```text
Nyquist frequency = 50/2 = 25 Hz
```

The normalized cutoff is:

```text
5 / 25 = 0.2
```

That matches the report's filter expression:

```text
butter(2, 5/(50/2))
```

---

## 12. Checkpoint, re-seed, and error compensation

The report says:

```text
Checkpoint every N, while healthy:
e ← mean(ms − m)
ms ← m
x_k ← m
```

and:

```text
Compensated prediction:
ms ← ms − e
```

This means that during healthy operation, the monitor periodically corrects the software sensor so that normal model drift does not accumulate forever.

---

### 12.1 Why re-seed is needed

A model can drift away from the real system over time.

If the monitor never corrects it, then the residual could grow even without attack:

```text
R_N grows because the model is drifting, not because the sensor is attacked.
```

So the paper uses a checkpoint/re-seed idea.

---

### 12.2 What `e` means

The error term is:

```text
e = mean(ms − m)
```

This is the average difference between the model prediction and physical measurement over a healthy window.

Then the compensated prediction is:

```text
ms ← ms − e
```

This compensates for systematic offset.

---

### 12.3 Why re-seed is suppressed during recovery

The report states that re-seed is suppressed during recovery so a sustained attack is not absorbed.

This is important.

If the sensor is attacked and the monitor keeps re-seeding the model to the attacked measurement, then the software sensor could learn the attack as if it were normal.

Example:

```text
Real altitude should be 10 m.
Attacked barometer says 13 m.
If the model is re-seeded to 13 m during attack, then ms becomes close to 13 m.
Then |m − ms| becomes small.
Attack disappears from the residual.
```

That is bad.

So during recovery:

```text
do not re-seed using attacked sensor measurements
```

This keeps the software sensor independent of the corrupted physical measurement.

---

## 13. Recovery action

The report states:

```text
while recovery: m ← ms
leave after K samples with R_N < T_off
```

Meaning:

1. If attack is detected, the system enters recovery mode.
2. During recovery, the physical measurement is replaced by the software-sensor value:

```text
m ← ms
```

3. The system stays in recovery until the accumulated residual is safely below `T_off` for `K` samples.

So recovery is not just detection. It also defines what to do with the corrupted measurement.

---

## 14. STL recovery formula

The report gives:

```text
φ_rec = G((R_N > T_on) → F[0:10s](R_N < T_off))
```

Breakdown:

### 14.1 Attack condition

```text
R_N > T_on
```

This means the accumulated residual crossed the attack threshold.

### 14.2 Recovery condition

```text
R_N < T_off
```

This means the accumulated residual dropped below the recovery threshold.

### 14.3 Eventually operator

```text
F[0:10s](R_N < T_off)
```

This means:

```text
within 10 seconds, R_N must become smaller than T_off
```

### 14.4 Full implication

```text
(R_N > T_on) → F[0:10s](R_N < T_off)
```

This means:

```text
if attack is detected, recovery must happen within 10 seconds
```

### 14.5 Global operator

```text
G(...)
```

This means the implication must hold at every monitored time point.

---

### 14.6 Recovery example

Suppose:

```text
T_on  = 100
T_off = 80
```

At time `t = 40 s`:

```text
R_N = 120
```

Attack detected because:

```text
120 > 100
```

Now the formula requires:

```text
Within 10 seconds, R_N < 80
```

If at `t = 47 s`:

```text
R_N = 70
```

then:

```text
70 < 80
```

Recovery property is satisfied.

If from `t = 40 s` to `t = 50 s`, `R_N` never goes below `80`, then the recovery STL property is violated.

---

## 15. File-by-file explanation

The report lists these files.

### 15.1 `faithful_core.py`

Role:

```text
shared core implementation
```

It contains:

```text
model/data loading
low-pass filter
Algorithm 1 software sensor
sliding_R calculation
threshold selection
offline run_monitor
online FaithfulMonitor
attack injection
```

This is the main logic.

---

### 15.2 `offline_faithful_stl.py`

Role:

```text
offline detector
```

It:

```text
loads recorded dataset
computes R_N
selects thresholds
detects with STL G(R_N < T_on)
checks recovery property
generates plots
```

This is for experiments on existing recorded data.

---

### 15.3 `online_faithful_stl.py`

Role:

```text
online detector
```

It:

```text
reads live sample stream through MAVLink
updates the monitor per sample
uses rtamt online spec.update
performs closed-loop recovery
```

This makes the monitor closer to real-time use.

---

### 15.4 `mavlink_source.py`

Role:

```text
dataset replay source
```

It:

```text
streams the dataset as MAVLink
injects sustained on-wire attack
feeds the online monitor
```

This lets the online monitor be tested with controlled attacked data.

---

### 15.5 Figure files

Offline plots:

```text
figures/offline_faithful_baro.png
figures/offline_faithful_gyro.png
figures/offline_faithful_gps.png
```

Online plots:

```text
figures/online_faithful_baro.png
figures/online_faithful_gyro.png
figures/online_faithful_gps.png
```

These show the detection results.

---

## 16. Inputs used

The report says the data/model inputs are read-only from the existing project:

```text
../rv_recovery/data/operation_data_50hz.mat
../rv_recovery/matlab/models/quadrotor_12state.mat
../rv_recovery/data/recovery_params.npy
```

Meaning:

### 16.1 `operation_data_50hz.mat`

This is the dataset:

```text
segment 0
50 Hz
recorded operation data
```

### 16.2 `quadrotor_12state.mat`

This contains:

```text
identified A, B, C, D matrices
```

used for the state-space software sensor.

### 16.3 `recovery_params.npy`

This contains:

```text
DTW-derived window N per channel
```

used to define the accumulated residual window.

---

# 17. Monitored sensors, attacks, thresholds, and results

The report monitors three sensor cases:

```text
barometer
gyroscope
GPS east
```

All attacks are sustained from:

```text
t = 40 s
```

The reason for sustained attacks is that an accumulated residual detector may not trip from a very short attack. The attack must contribute enough accumulated error to cross `T_on`.

---

## 18. Barometer report explained

### 18.1 Sensor

The report says:

```text
barometer (alt, ch2)
```

This means the monitored channel is altitude/barometer related.

### 18.2 Attack

The attack is:

```text
+3.0 m bias
```

That means during the attack:

```text
m_baro_attacked(t) = m_baro_clean(t) + 3.0
```

The physical measurement is artificially shifted upward by 3 meters.

---

### 18.3 Window

The report gives:

```text
N = 3492
```

At 50 Hz:

```text
window = 3492 × 0.02 s
window = 69.84 s
```

So:

```text
R_baro,N(t) = Σ over last 3492 samples |m_baro(i) − ms_baro(i)|
```

---

### 18.4 Threshold

The report gives:

```text
T_on = 13631.9
```

Threshold formation:

```text
T_on = e_max × 1.10
```

So the approximate `e_max` can be recovered as:

```text
e_max = T_on / 1.10
e_max = 13631.9 / 1.10
e_max ≈ 12392.64
```

Recovery threshold:

```text
T_off = 0.80 × T_on
T_off = 0.80 × 13631.9
T_off ≈ 10905.52
```

---

### 18.5 STL formula

```text
φ_baro = G(R_baro,N < 13631.9)
```

Robustness:

```text
ρ_baro = 13631.9 − R_baro,N
```

Attack detection:

```text
ρ_baro < 0
```

Equivalent:

```text
R_baro,N > 13631.9
```

---

### 18.6 Detection result

The report gives:

```text
Attack start: 40.00 s
Detection:    65.56 s
Latency:      25.56 s
```

Latency calculation:

```text
latency = detection time − attack start time
latency = 65.56 − 40.00
latency = 25.56 s
```

---

### 18.7 Why barometer detection is slow

The window is large:

```text
69.84 s
```

The threshold is also large:

```text
T_on = 13631.9
```

The report explains that slow barometer/GPS detection happens because large DTW windows and open-loop prediction drift inflate `e_max` and therefore inflate `T_on`.

If `T_on` is high, it takes longer for the attack to push `R_N` above the threshold.

---

### 18.8 Barometer example

Simplified example:

```text
N = 5
T_on = 20
T_off = 16
```

Before attack:

```text
d values = 2, 3, 4, 3, 2
R_N = 2 + 3 + 4 + 3 + 2 = 14
```

Robustness:

```text
ρ = T_on − R_N
ρ = 20 − 14
ρ = +6
```

Safe.

During attack:

```text
d values = 5, 5, 6, 6, 5
R_N = 5 + 5 + 6 + 6 + 5 = 27
```

Robustness:

```text
ρ = 20 − 27
ρ = −7
```

Attack detected.

---

## 19. Gyroscope report explained

### 19.1 Sensor

The report says:

```text
gyroscope (p, ch9)
```

Here:

```text
p = roll rate
```

So this monitors the x-axis/roll-rate gyroscope channel.

---

### 19.2 Attack

The attack is:

```text
set 0.8 rad/s
```

That means during the attack:

```text
m_gyr_x(t) = 0.8 rad/s
```

Instead of using the clean physical gyro value, the attacked measurement is forced to `0.8 rad/s`.

---

### 19.3 Window

The report gives:

```text
N = 1066
```

At 50 Hz:

```text
window = 1066 × 0.02
window = 21.32 s
```

So:

```text
R_gyro,N(t) = Σ over last 1066 samples |m_gyr_x(i) − ms_gyr_x(i)|
```

---

### 19.4 Threshold

The report gives:

```text
T_on = 4.8
```

Threshold formation:

```text
T_on = e_max × 1.10
```

Approximate clean maximum:

```text
e_max = 4.8 / 1.10
e_max ≈ 4.36
```

Recovery threshold:

```text
T_off = 0.80 × 4.8
T_off = 3.84
```

---

### 19.5 STL formula

```text
φ_gyro = G(R_gyro,N < 4.8)
```

Robustness:

```text
ρ_gyro = 4.8 − R_gyro,N
```

Attack detection:

```text
ρ_gyro < 0
```

Equivalent:

```text
R_gyro,N > 4.8
```

---

### 19.6 Detection result

The report gives:

```text
Attack start: 40.00 s
Detection:    40.10 s
Latency:      0.10 s
```

Latency:

```text
40.10 − 40.00 = 0.10 s
```

---

### 19.7 Why gyroscope detection is fast

The gyroscope is a directly observed fast state.

The report says the gyroscope has a small `e_max` of about `4.3`, so `T_on` is also small:

```text
T_on = 4.8
```

The attack value:

```text
0.8 rad/s
```

creates enough error to push `R_N` above `T_on` very quickly.

---

### 19.8 Gyroscope example

Suppose:

```text
T_on = 4.8
```

Before attack:

```text
R_gyro,N = 3.0
```

Robustness:

```text
ρ = 4.8 − 3.0
ρ = +1.8
```

Safe.

During attack:

```text
R_gyro,N = 5.5
```

Robustness:

```text
ρ = 4.8 − 5.5
ρ = −0.7
```

Attack detected.

---

## 20. GPS east report explained

### 20.1 Sensor

The report says:

```text
GPS east (pE, ch1)
```

This monitors the east-position component.

---

### 20.2 Attack

The attack is:

```text
+20 m bias
```

That means:

```text
m_gps_east_attacked(t) = m_gps_east_clean(t) + 20
```

The GPS east measurement is shifted by 20 meters.

---

### 20.3 Window

The report gives:

```text
N = 2639
```

At 50 Hz:

```text
window = 2639 × 0.02
window = 52.78 s
```

So:

```text
R_gps_east,N(t) = Σ over last 2639 samples |m_gps_east(i) − ms_gps_east(i)|
```

---

### 20.4 Threshold

The report gives:

```text
T_on = 4931.0
```

Threshold formation:

```text
T_on = e_max × 1.10
```

Approximate clean maximum:

```text
e_max = 4931.0 / 1.10
e_max ≈ 4482.73
```

Recovery threshold:

```text
T_off = 0.80 × 4931.0
T_off = 3944.8
```

---

### 20.5 STL formula

```text
φ_gps = G(R_gps_east,N < 4931.0)
```

Robustness:

```text
ρ_gps = 4931.0 − R_gps_east,N
```

Attack detection:

```text
ρ_gps < 0
```

Equivalent:

```text
R_gps_east,N > 4931.0
```

---

### 20.6 Detection result

The report gives:

```text
Attack start: 40.00 s
Detection:    43.02 s
Latency:      3.02 s
```

Latency:

```text
43.02 − 40.00 = 3.02 s
```

---

### 20.7 Why GPS is slower than gyroscope but faster than barometer

GPS has a large window:

```text
52.78 s
```

and a large threshold:

```text
T_on = 4931.0
```

So it is not instant.

But the attack is also large:

```text
+20 m
```

That large bias increases the accumulated residual enough to cross threshold after about `3.02 s`.

---

### 20.8 GPS example

Suppose:

```text
T_on = 100
```

Before attack:

```text
R_gps,N = 80
```

Robustness:

```text
ρ = 100 − 80
ρ = +20
```

Safe.

During attack:

```text
R_gps,N = 120
```

Robustness:

```text
ρ = 100 − 120
ρ = −20
```

Attack detected.

---

## 21. Summary of all monitored results

| Sensor | Attack | N | Window duration | T_on | Approx. e_max | T_off | Detection | Latency |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Barometer | +3.0 m bias | 3492 | 69.84 s | 13631.9 | 12392.64 | 10905.52 | 65.56 s | 25.56 s |
| Gyroscope | set 0.8 rad/s | 1066 | 21.32 s | 4.8 | 4.36 | 3.84 | 40.10 s | 0.10 s |
| GPS east | +20 m bias | 2639 | 52.78 s | 4931.0 | 4482.73 | 3944.8 | 43.02 s | 3.02 s |

Notes:

```text
Approx. e_max = T_on / 1.10
T_off = 0.80 × T_on
Latency = detection time − 40.00 s
```

---

## 22. Why only the attacked channel fires

The report says:

```text
Only the attacked channel fires in each run.
No cross-channel false positives.
Clean data does not trigger.
```

This means:

- in the barometer attack experiment, the barometer monitor detects;
- in the gyro attack experiment, the gyro monitor detects;
- in the GPS attack experiment, the GPS monitor detects;
- unrelated channels do not incorrectly detect attacks.

This is important because it means the thresholds are not so low that everything triggers.

Clean data does not trigger because:

```text
T_on = e_max + 10%
```

or more exactly:

```text
T_on = 1.10 × e_max
```

Since `e_max` is the largest clean accumulated residual, clean data should stay below `T_on`.

---

## 23. Why the report says barometer/GPS detections are slow

The report says barometer and GPS are slower because:

```text
DTW windows are huge
open-loop altitude/position predictions drift at 50 Hz
large drift inflates e_max
large e_max inflates T_on
large T_on delays threshold crossing
```

This is not hidden. The report explicitly says this is a faithful consequence of applying the method at 50 Hz, while the paper ran at 400 Hz.

Important wording:

```text
Nothing was tuned to hide it.
```

Meaning the implementation did not artificially lower thresholds to get better detection times.

---

## 24. Why sustained attacks are used

The report says attacks are sustained from `t = 40 s`.

A sustained attack is necessary because the paper-faithful detector uses accumulated residual:

```text
R_N = Σ |m − ms|
```

If the attack is too short, it may not add enough total error to exceed `T_on`.

Example:

```text
T_on = 100
```

Short attack:

```text
R_N only reaches 90
90 < 100
No detection
```

Sustained attack:

```text
R_N reaches 130
130 > 100
Attack detected
```

So sustained attack is consistent with accumulated detection.

---

## 25. Difference from earlier STL work

The report compares the old implementation and the new one.

### 25.1 Residual monitored

Earlier:

```text
instantaneous |m − ms|
```

New:

```text
accumulated R_N = Σ|m − ms|
```

### 25.2 Threshold

Earlier:

```text
guide round numbers:
0.30, 0.15, etc.
```

New:

```text
T_on = e_max + margin
T_on = e_max × 1.10
```

### 25.3 Window

Earlier:

```text
fixed 580 ms
```

New:

```text
DTW-derived N per channel
```

### 25.4 Software sensor

Earlier:

```text
recorded state / clean copy approximation
```

New:

```text
model Cx + Du with re-seed and compensation e
```

### 25.5 Recovery

Earlier:

```text
none or simple STL recovery property
```

New:

```text
Algorithm 1 T_on/T_off/K state machine
```

### 25.6 STL role

Earlier:

```text
G[0:W](residual < ε)
```

New:

```text
G(R_N < T_on)
```

The new version is more faithful to Algorithm 1.

---

## 26. Offline vs online

The report says both offline and online are supported.

### 26.1 Offline

Offline means:

```text
Use recorded dataset
Compute R_N
Select thresholds
Detect with STL
Generate plots
```

Run command:

```bash
cd /home/tchowdh4/paperImp/newImp
PY=/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
export MPLBACKEND=Agg
$PY offline_faithful_stl.py
```

### 26.2 Online

Online means:

```text
Read data sample-by-sample from MAVLink
Update monitor in real time
Use rtamt online spec.update
Apply closed-loop recovery
```

Example monitor terminal:

```bash
$PY online_faithful_stl.py --conn udpin:127.0.0.1:14580 --plot figures/online_faithful_gyro.png --label "gyro"
```

Example source terminal:

```bash
$PY mavlink_source.py --out udpout:127.0.0.1:14580 --attack gyro --rate 1500
```

The report says offline and online give identical detections for the verified cases.

---

## 27. Formula formation step-by-step

This section summarizes the formation of the main formula from the paper to STL.

### Step 1: Physical measurement and software prediction

```text
m(t)  = physical sensor measurement
ms(t) = software-sensor prediction
```

### Step 2: Instantaneous error

```text
d(t) = |m(t) − ms(t)|
```

### Step 3: Windowed accumulated residual

```text
R_N(t) = Σ_{i=t−N+1}^{t} d(i)
```

or:

```text
R_N(t) = Σ_{i=t−N+1}^{t} |m(i) − ms(i)|
```

### Step 4: Clean-data maximum

```text
e_max = max_t R_N(t) on clean segment
```

### Step 5: Attack threshold

```text
T_on = e_max × (1 + 0.10)
T_on = 1.10 × e_max
```

### Step 6: Recovery threshold

```text
T_off = 0.80 × T_on
```

### Step 7: Detection rule

Paper rule:

```text
Attack if R_N > T_on
```

### Step 8: STL safety formula

```text
φ_det = G(R_N < T_on)
```

### Step 9: Robustness

```text
ρ = T_on − R_N
```

### Step 10: Attack verdict

```text
ρ < 0 ⇔ R_N > T_on ⇔ attack
```

### Step 11: Recovery property

```text
φ_rec = G((R_N > T_on) → F[0:10s](R_N < T_off))
```

---

## 28. Full numerical example from start to finish

Assume one sensor has:

```text
N = 5
clean d(t) values:
1, 2, 1, 2, 1, 3, 2
```

Compute rolling accumulated residuals:

```text
R_N at first full window:
1 + 2 + 1 + 2 + 1 = 7

next:
2 + 1 + 2 + 1 + 3 = 9

next:
1 + 2 + 1 + 3 + 2 = 9
```

So:

```text
e_max = 9
```

Threshold:

```text
T_on = 1.10 × 9 = 9.9
```

Recovery threshold:

```text
T_off = 0.80 × 9.9 = 7.92
```

Now attack happens and errors become:

```text
4, 4, 5, 5, 4
```

Accumulated residual:

```text
R_N = 4 + 4 + 5 + 5 + 4 = 22
```

STL robustness:

```text
ρ = T_on − R_N
ρ = 9.9 − 22
ρ = −12.1
```

Since:

```text
ρ < 0
```

attack is detected.

Recovery condition requires:

```text
R_N < 7.92
```

within 10 seconds.

If later errors become:

```text
1, 1, 2, 1, 1
```

then:

```text
R_N = 6
```

Since:

```text
6 < 7.92
```

recovery condition is satisfied.

If this safe condition stays true for:

```text
K = 10 samples
```

the recovery state machine can leave recovery mode.

---

## 29. Limitations honestly stated in the report

### 29.1 STL cannot sum

STL monitors `R_N`, but it does not compute the sum internally.

The sum is computed by the paper's algorithm.

### 29.2 Slow detection at 50 Hz for some sensors

Barometer and GPS are slow because:

```text
large N
large drift
large e_max
large T_on
```

### 29.3 Sustained attacks are needed

Short attacks may not accumulate enough residual to cross `T_on`.

### 29.4 `e_max` is calibrated on segment 0

The report says extending calibration to all 21 clean segments is possible but would likely raise `T_on` slightly.

### 29.5 Attacks are synthetic injections

The attacks are simulated/injected, but the online pipeline can accept a real ArduPilot SITL feed through an adapter.

---

## 30. Final interpretation

This report shows a stronger and more paper-faithful STL implementation than the earlier guide-based one.

The most important final statement is:

```text
The STL formula is not just checking instantaneous residual < fixed threshold.
It is checking the paper's accumulated residual R_N against the threshold T_on.
```

The final detection formula is:

```text
φ_det = G(R_N < T_on)
```

with:

```text
R_N(t) = Σ_{i=t−N+1}^{t} |m(i) − ms(i)|
T_on = 1.10 × e_max
ρ = T_on − R_N
ρ < 0 ⇔ attack detected
```

The recovery formula is:

```text
φ_rec = G((R_N > T_on) → F[0:10s](R_N < T_off))
```

with:

```text
T_off = 0.80 × T_on
K = 10 safe samples
```

The verified attacks and detections are:

```text
barometer +3.0 m bias:
    N = 3492
    T_on = 13631.9
    detection = 65.56 s
    latency = 25.56 s

gyroscope set 0.8 rad/s:
    N = 1066
    T_on = 4.8
    detection = 40.10 s
    latency = 0.10 s

GPS east +20 m bias:
    N = 2639
    T_on = 4931.0
    detection = 43.02 s
    latency = 3.02 s
```

This means the implementation follows the paper's detection statistic and threshold-selection method, then uses STL as the formal monitoring layer over that statistic.

---

## 31. Short presentation-ready explanation

Use this if you need to explain it quickly:

```text
In this paper-faithful STL implementation, the monitor first computes the paper's software-sensor residual d(t)=|m(t)−ms(t)|, where m(t) is the physical sensor measurement and ms(t) is the model/software-sensor prediction. Instead of thresholding only one instantaneous residual, it computes the paper's accumulated residual R_N(t)=Σ|m−ms| over a DTW-derived window N. The clean segment is used to find e_max, the maximum clean accumulated residual. The attack threshold is then T_on=1.10e_max, and the recovery threshold is T_off=0.80T_on. STL monitors the paper statistic using φ_det=G(R_N<T_on), whose robustness is ρ=T_on−R_N. Therefore, ρ<0 is exactly equivalent to the paper rule R_N>T_on, so STL detection matches the paper's attack decision.
```
