# Algorithms and Equations Needed to Implement

# *Software-based Realtime Recovery from Sensor Attacks on Robotic Vehicles*

## 1. Core Runtime Variables

```text
u       = control input / target state of the real vehicle
m       = real physical sensor measurement
ms      = software-sensor measurement / predicted sensor measurement
x       = control-state vector of the real vehicle / model state
y       = model output
A,B,C,D = state-space system matrices learned by system identification
r       = accumulated residual / accumulated prediction error
e       = model/external error compensation term
N       = historical window size
Ton     = recovery-on threshold
Toff    = recovery-off threshold, normally Toff < Ton
K       = safe-count threshold for switching back to real sensor
Δt      = sampling interval
k       = number of equidistant sample points for finite differentiation
```

---

## 2. Offline Implementation Workflow

```text
Algorithm: Offline Software-Sensor Construction

Input:
    - Model template for the RV family
    - Normal operation logs
    - Target-state/control-input logs
    - Sensor/state-output logs

Steps:
    1. Collect normal operation data under multiple maneuvers.
    2. Generate missions systematically using MAVLink commands.
    3. Collect data at high sampling rate.
    4. Align heterogeneous streams by resampling them to the same target frequency.
    5. Use spline interpolation for resampling.
    6. Define state vector x(t), control input u(t), and output y(t).
    7. Use system identification to instantiate A, B, C, D.
    8. Build software sensors by converting model outputs into predicted physical sensor readings.
    9. Select window size N using dynamic time warping.
    10. Select recovery thresholds Ton and Toff from clean operation data.
    11. Patch the control program immediately after physical sensor acquisition.
    12. Run runtime recovery monitoring in the main control loop.
```

---

## 3. State-Space System Model

### 3.1 Continuous / Template State Equation

```math
x'(t) = A x(t) + B u(t)
```

### 3.2 Output Equation

```math
y(t) = C x(t) + D u(t)
```

Where:

```text
x(t) = physical/control state vector
u(t) = system input / target state / reference command
y(t) = system output measured by sensors
A,B,C,D = learned system matrices
```

---

## 4. Runtime Discrete Model Update

The runtime algorithm applies the model in iterative loop form:

```math
y \leftarrow Cx + Du
```

```math
x \leftarrow Ax + Bu
```

Implementation form:

```text
y_pred = C @ x + D @ u
x_next = A @ x + B @ u
```

---

## 5. Quadrotor State Vector

For the quadrotor model, the state vector is:

```math
x =
\begin{bmatrix}
x & y & z & \phi & \theta & \psi & \dot{x} & \dot{y} & \dot{z} & p & q & r
\end{bmatrix}
```

Where:

```text
[x, y, z]              = position vector
[φ, θ, ψ]              = attitude angles: roll, pitch, yaw
[ẋ, ẏ, ż]             = linear velocity
[p, q, r]              = angular velocity
```

---

## 6. Software-Sensor Conversion Equations

Software sensors convert model outputs into predicted sensor readings.

---

### 6.1 Accelerometer Software Sensor

The accelerometer measures linear acceleration. Since the state vector contains velocity but not direct acceleration, acceleration is computed from velocity difference:

```math
a(t) = c_k \frac{v(t) - v(t-k)}{k \cdot \Delta t}
```

Where:

```text
a(t)  = predicted acceleration
v(t)  = current velocity
v(t-k)= velocity k samples earlier
Δt    = sampling interval
ck    = constant coefficient
k     = number of equidistant sample points
```

Implementation form:

```text
a_pred[t] = ck * (v[t] - v[t-k]) / (k * dt)
```

---

### 6.2 Gyroscope Software Sensor

The gyroscope measures angular velocity. For the quadrotor state vector:

```math
\omega =
\begin{bmatrix}
p \\
q \\
r
\end{bmatrix}
```

Where:

```text
p = roll angular velocity
q = pitch angular velocity
r = yaw angular velocity
```

Direct software gyroscope prediction:

```math
m_{gyro,s} =
\begin{bmatrix}
p \\
q \\
r
\end{bmatrix}
```

---

### 6.3 Barometer Software Sensor

The barometer measures atmospheric pressure. Pressure is calculated from altitude:

```math
P_h = P_0 \cdot \exp
\left[
\frac{-g_0 \cdot M \cdot (z - h_0)}
{R \cdot T_0}
\right]
```

Where:

```text
Ph = predicted air pressure at altitude z
P0 = base air pressure
g0 = gravitational acceleration, 9.87 m/s²
M  = molar mass of Earth air, 0.02896 kg/mol
z  = current altitude from model state
h0 = base altitude
R  = universal gas constant, 8.3143 N·m/(mol·K)
T0 = base temperature in Kelvin
```

Implementation form:

```text
Ph = P0 * exp((-g0 * M * (z - h0)) / (R * T0))
```

---

### 6.4 Magnetometer / Compass Heading Conversion

The magnetometer measures magnetic field values along three axes. Heading is computed as:

```math
H =
atan2
\left(
-m_y \cos\phi + m_z \sin\phi,\;
m_x \cos\theta
+ m_y \sin\theta \sin\phi
+ m_z \sin\theta \cos\phi
\right)
```

Where:

```text
H          = heading / yaw direction
mx, my, mz = magnetic field measurements along x, y, z axes
φ          = roll angle
θ          = pitch angle
```

Implementation form:

```text
H = atan2(
    -my * cos(phi) + mz * sin(phi),
     mx * cos(theta)
   + my * sin(theta) * sin(phi)
   + mz * sin(theta) * cos(phi)
)
```

---

### 6.5 GPS Software Sensor

GPS measures position and velocity. In the paper’s model, GPS measurements are directly acquired from the system-model state.

```math
m_{GPS,s} =
\begin{bmatrix}
x & y & z & \dot{x} & \dot{y} & \dot{z}
\end{bmatrix}
```

Implementation form:

```text
gps_position_pred = [x, y, z]
gps_velocity_pred = [xdot, ydot, zdot]
```

---

## 7. Coordinate-System Transformation

The paper uses two frames:

```text
Inertial frame = earth-fixed/global frame
Body frame     = vehicle-attached/local frame
```

Notation:

```text
ξ = (x, y, z)       = linear position in inertial frame
η = (φ, θ, ψ)       = attitude in inertial frame
VB                 = linear velocity in body frame
ω = (p, q, r)       = angular velocity in body frame
Sx = sin(x)
Cx = cos(x)
Tx = tan(x)
```

---

### 7.1 Body-to-Inertial Rotation Matrix

```math
R =
\begin{bmatrix}
C_\psi C_\theta
&
C_\psi S_\theta S_\phi - S_\psi C_\phi
&
C_\psi S_\theta C_\phi + S_\psi S_\phi
\\
S_\psi C_\theta
&
S_\psi S_\theta S_\phi + C_\psi C_\phi
&
S_\psi S_\theta C_\phi - C_\psi S_\phi
\\
-S_\theta
&
C_\theta S_\phi
&
C_\theta C_\phi
\end{bmatrix}
```

Orthogonality property:

```math
R^{-1} = R^T
```

Therefore:

```text
R   = body frame → inertial frame
Rᵀ  = inertial frame → body frame
```

---

### 7.2 Angular Velocity Transformation: Euler Rate to Body Rate

```math
\omega = W_\eta \dot{\eta}
```

Expanded:

```math
\begin{bmatrix}
p \\
q \\
r
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & -S_\theta \\
0 & C_\phi & C_\theta S_\phi \\
0 & -S_\phi & C_\phi C_\theta
\end{bmatrix}
\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
```

Implementation form:

```text
p = phidot - sin(theta) * psidot

q = cos(phi) * thetadot
  + cos(theta) * sin(phi) * psidot

r = -sin(phi) * thetadot
  + cos(phi) * cos(theta) * psidot
```

---

### 7.3 Angular Velocity Transformation: Body Rate to Euler Rate

```math
\dot{\eta} = W_\eta^{-1} \omega
```

Expanded:

```math
\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
1 & S_\phi T_\theta & C_\phi T_\theta \\
0 & C_\phi & -S_\phi \\
0 & S_\phi / C_\theta & C_\phi / C_\theta
\end{bmatrix}
\begin{bmatrix}
p \\
q \\
r
\end{bmatrix}
```

Implementation form:

```text
phidot = p
       + sin(phi) * tan(theta) * q
       + cos(phi) * tan(theta) * r

thetadot = cos(phi) * q
         - sin(phi) * r

psidot = (sin(phi) / cos(theta)) * q
       + (cos(phi) / cos(theta)) * r
```

---

## 8. Error Correction Equations

The paper separates software-sensor error into:

```text
1. conversion error
2. model error
3. external error
```

---

### 8.1 Filtering Real Measurements

The paper applies a filter to the real sensor measurement:

```math
m \leftarrow filter(m)
```

Implementation block:

```text
m_filtered = low_pass_filter(m_raw)
```

The paper does not provide exact low-pass filter coefficients.

---

### 8.2 Software Sensor Conversion

The model output is converted into sensor prediction:

```math
m_s \leftarrow convert(y)
```

Implementation block:

```text
ms = convert_model_output_to_sensor_prediction(y)
```

Where `convert()` depends on sensor type:

```text
accelerometer → finite-difference velocity equation
gyroscope     → angular velocity state or angular-rate transform
barometer     → altitude-to-pressure equation
magnetometer  → heading equation
GPS           → position/velocity state extraction
```

---

### 8.3 Window-Based Residual Accumulation

At runtime, residual accumulates the absolute difference between the real filtered measurement and the corrected software-sensor prediction:

```math
r \leftarrow r + |m - m_s|
```

Implementation form:

```text
r = r + abs(m - ms)
```

For vector sensors:

```math
r \leftarrow r + \|m - m_s\|
```

or component-wise:

```math
r_j \leftarrow r_j + |m_j - m_{s,j}|
```

---

### 8.4 External / Model Error Compensation

At the start of a new window, the error compensation term is estimated from the previous window:

```math
e \leftarrow error\_estimation(r, m, m_s)
```

The paper describes this as estimating the average error between real state/measurement and model prediction in the previous window.

Direct implementation form:

```math
e_j =
\frac{1}{N}
\sum_{i=t-N}^{t-1}
\left(
m_{s,j}[i] - m_j[i]
\right)
```

Then compensate the software-sensor prediction:

```math
m_s \leftarrow m_s - e
```

Implementation form:

```text
e = average(ms_previous_window - m_previous_window)
ms_corrected = ms - e
```

---

## 9. Recovery Parameter Selection

---

### 9.1 Window Size Selection

The paper selects the window size `N` using dynamic time warping:

```math
N =
\max
\left(
\text{time-displacement between real sensor signal and software-sensor signal}
\right)
```

Implementation interpretation:

```text
For each clean operation log:
    1. Align real sensor sequence m with software-sensor sequence ms using DTW.
    2. Compute the maximum time displacement between aligned points.
Set:
    N = maximum displacement observed over the clean operation dataset.
```

---

### 9.2 Threshold Selection

After choosing `N`, compute the maximum accumulated error between real and software-sensor signals within windows from clean data.

```math
T = e_{max} + m
```

Where:

```text
T     = recovery threshold
emax  = maximum accumulated clean residual/error within the selected window
m     = margin parameter
```

Implementation form:

```text
for each clean log:
    for each window of size N:
        residual_window = sum(abs(m - ms))
emax = max(residual_window over all clean windows)
T = emax + margin
```

For recovery switching:

```math
T_{off} < T_{on}
```

---

## 10. Runtime Recovery Monitoring Algorithm

```text
Algorithm: Runtime Recovery Monitoring

Inputs:
    u = control input of the real vehicle
    m = physical sensor measurement

Persistent / internal states:
    x             = model/control state
    t             = elapsed loop count inside current window
    r             = accumulated residual
    e             = error compensation term
    recovery_mode = boolean flag
    safe_count    = counter for stable recovery-off condition

Procedure RECOVERY_MONITOR(u, m):

    1. Compute model output:
           y ← Cx + Du

    2. Update model state:
           x ← Ax + Bu

    3. Filter real sensor measurement:
           m ← filter(m)

    4. Convert model output to software-sensor prediction:
           ms ← convert(y)

    5. Increment window timer:
           t ← t + 1

    6. If not in recovery mode and current window ended:
           if recovery_mode == false and t > window:
               t ← 0
               r ← 0
               e ← error_estimation(r, m, ms)
               ms ← m

    7. Apply error compensation:
           ms ← ms - e

    8. Accumulate residual:
           r ← r + |m - ms|

    9. Turn recovery on if residual exceeds recovery-on threshold:
           if r > Ton:
               recovery_mode ← true
               safe_count ← 0

    10. If recovery mode is active:
           if recovery_mode == true:

               Replace real sensor with software sensor:
                   m ← ms

               Check if residual is below recovery-off threshold:
                   if r < Toff:
                       safe_count ← safe_count + 1

               Switch back to real sensor after K safe counts:
                   if safe_count > K:
                       recovery_mode ← false

               Trigger optional recovery action:
                   recovery_action()

    11. Return corrected sensor measurement:
           return m

```

---

## 11. Minimal Sensor-Replacement Logic

The paper’s motivating code inserts recovery immediately after sensor read:

```text
for each sensor i:
    real_i = physical_sensor[i].read()

    if abs(software_sensor_i - real_i) > threshold:
        real_i = software_sensor_i

    fused_sensor_value += weight_i * real_i
```

Equivalent equation:

```math
m_i =
\begin{cases}
m_{s,i}, & |m_i - m_{s,i}| > k \\
m_i, & |m_i - m_{s,i}| \le k
\end{cases}
```

Weighted sensor fusion:

```math
m_{fused} =
\sum_i w_i m_i
```

---

## 12. Recovery Success Criterion

Recovery is successful when the real output remains within an error margin of the predicted/expected output for a specified duration:

```math
R_{succ} :=
|Y_t - \bar{Y}_t|
\le
\epsilon,
\quad

t \in [1 \ldots k]
```

Where:

```text
Yt      = real output
Ȳt      = predicted / expected output
ε       = allowed error margin
t       = timestamp in recovery mode
k       = maximum time horizon for deciding recovery success
```

Example from the paper:

```text
ε = 3
k = 10
```

Meaning:

```text
The vehicle must remain within 3 meters error for 10 seconds under recovery mode.
```

---

## 13. Supplementary Compensation Equations

The paper uses supplementary compensation when software gyroscope alone is not accurate enough, especially when all gyroscopes are compromised.

The compensation estimates attitude from accelerometer and magnetometer.

---

### 13.1 Roll from Accelerometer

```math
\phi_{acc}
=
atan2
\left(
y_{acc},
\sqrt{x_{acc}^2 + z_{acc}^2}
\right)
```

---

### 13.2 Pitch from Accelerometer

```math
\theta_{acc}
=
atan2
\left(
x_{acc},
\sqrt{y_{acc}^2 + z_{acc}^2}
\right)
```

---

### 13.3 Yaw / Heading from Magnetometer

```math
\psi_{mag}
=
atan2
\left(
-y_{mag}\cos\phi + z_{mag}\sin\phi,\;
x_{mag}\cos\theta
+ y_{mag}\sin\theta\sin\phi
+ z_{mag}\sin\theta\cos\phi

\right)
```

---

### 13.4 Supplementary Weighted Combination

The paper states that the supplementary outputs and the software-sensor output are combined using a weighted sum.

Generic implementation form:

```math
m_{combined}
=
\sum_i w_i m_i
```

For attitude compensation:

```math
\eta_{combined}
=
weighted\_sum
\left(
\eta_{software},
\eta_{acc/mag}
\right)
```

Where:


```text
ηsoftware = software-sensor attitude estimate
ηacc/mag  = attitude estimate from accelerometer and magnetometer
```

The exact weights are not specified in the paper.

---

## 14. Required Implementation Functions

```text
system_identification(data, template)
    → learns A, B, C, D

resample_with_spline(raw_streams, target_frequency)
    → aligned streams at common frequency

convert(y, sensor_type)
    → software-sensor prediction ms

filter(m)
    → filtered real sensor measurement

error_estimation(previous_window)
    → compensation term e

dynamic_time_warping(m, ms)
    → maximum time displacement for N

recovery_action()
    → optional emergency action, e.g., safe landing / manual transition
```

---

## 15. Implementation Checklist

```text
[ ] Collect normal operation data under multiple maneuvers.
[ ] Resample all logs to a common target frequency using spline interpolation.
[ ] Build x(t), u(t), y(t).
[ ] Use system identification to learn A, B, C, D.
[ ] Implement state-space update:
        y ← Cx + Du
        x ← Ax + Bu
[ ] Implement accelerometer conversion.
[ ] Implement gyroscope prediction.
[ ] Implement barometer conversion.
[ ] Implement magnetometer heading conversion.
[ ] Implement GPS state extraction.
[ ] Implement coordinate transforms R, Rᵀ, Wη, Wη⁻¹.
[ ] Implement low-pass filtering for real measurements.
[ ] Implement residual accumulation:
        r ← r + |m - ms|
[ ] Select N using DTW.
[ ] Select threshold:
        T = emax + margin
[ ] Use Ton and Toff with Toff < Ton.

[ ] Implement recovery mode switching.
[ ] Replace attacked physical sensor measurement with software-sensor value.
[ ] Implement optional recovery action.
[ ] Evaluate recovery with:
        |Yt - Ȳt| ≤ ε for t ∈ [1...k]
```
