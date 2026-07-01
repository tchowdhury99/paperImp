
## Thresholds and Bounds Used in STL Monitoring

| Quantity | Threshold / Bound | Meaning |
|---|---:|---|
| Barometer residual | $\varepsilon_{\text{baro}} = 0.30\ \text{m}$ | Barometer error must stay below 0.30 m |
| GPS residual | $\varepsilon_{\text{gps}} = 0.169349$ | GPS north/east residual must stay below 0.169349 |
| Gyroscope residual | $\varepsilon_{\text{gyr}} = 0.15\ \text{rad/s}$ | Gyro residual must stay below 0.15 rad/s |
| Altitude lower bound | $h_{\min} = 0.97\ \text{m}$ | Altitude must stay above 0.97 m |
| Altitude upper bound | $h_{\max} = 29.70\ \text{m}$ | Altitude must stay below 29.70 m |

## Formula 1 — Barometer Integrity

**Script:** `offline_stl_baro.py`  
**Plot:** `stl_result_baro.png`

### Formula

$$
G_{[0,2000\mathrm{ms}]}\left(\mathrm{baro\_res} < 0.30\right)
$$

### Example

Suppose the true altitude/software-sensor estimate is:

$$
\mathrm{alt} = 10.0\mathrm{m}
$$

and the barometer reading is:

$$
\mathrm{BARO\_Alt} = 10.1\mathrm{m}
$$

Normal residual:

$$
\mathrm{baro\_error} = \left|10.1 - 10.0\right| = 0.1\mathrm{m}
$$

Since:

$$
0.1 < 0.30
$$

the barometer is safe.

During the simulated attack:

$$
m_{\mathrm{baro}} = 10.1 + 3.0 = 13.1\mathrm{m}
$$

and:

$$
m_{s,\mathrm{baro}} = 10.0\mathrm{m}
$$

Residual:

$$
\mathrm{baro\_error} = \left|13.1 - 10.0\right| = 3.1\mathrm{m}
$$

Since:

$$
3.1 > 0.30
$$

the STL predicate is violated.

Robustness:

$$
\rho = 0.30 - 3.1 = -2.8
$$

So STL reports an attack/violation.


## Formula 2 — GPS Integrity

**Script:** `offline_stl_gps.py`  
**Plot:** `stl_result_gps.png`

### Formula

$$
G_{[0,580\,\mathrm{ms}]}
\left(
e_{\mathrm{GPS,N}} < 0.169349
\land
e_{\mathrm{GPS,E}} < 0.169349
\right)
$$

### Threshold

$$
\varepsilon_{\mathrm{GPS}} = 0.169349
$$

### Formula Formation

Start with GPS north safety:

$$
e_{\mathrm{GPS,N}}(t) < \varepsilon_{\mathrm{GPS}}
$$

Substitute the threshold:

$$
e_{\mathrm{GPS,N}}(t) < 0.169349
$$

For both GPS north and GPS east residuals, the STL formula becomes:

$$
G_{[0,580\,\mathrm{ms}]}
\left(
e_{\mathrm{GPS,N}} < 0.169349
\land
e_{\mathrm{GPS,E}} < 0.169349
\right)
$$

### Example

Suppose:

$$
e_{\mathrm{GPS,N}} = 0.05
$$

$$
e_{\mathrm{GPS,E}} = 0.08
$$

Both are below the threshold:

$$
0.05 < 0.169349
$$

$$
0.08 < 0.169349
$$

So the GPS formula is satisfied.

Now suppose:

$$
e_{\mathrm{GPS,N}} = 0.20
$$

$$
e_{\mathrm{GPS,E}} = 0.08
$$

North violates the threshold:

$$
0.20 > 0.169349
$$

Robustness for north:

$$
\rho_{\mathrm{N}} = 0.169349 - 0.20
$$

$$
\rho_{\mathrm{N}} = -0.030651
$$

East is safe:

$$
\rho_{\mathrm{E}} = 0.169349 - 0.08
$$

$$
\rho_{\mathrm{E}} = +0.089349
$$

For the AND operator, STL uses the weaker condition:

$$
\rho_{\mathrm{total}} =
\min\left(\rho_{\mathrm{N}}, \rho_{\mathrm{E}}\right)
$$

$$
\rho_{\mathrm{total}} =
\min\left(-0.030651, +0.089349\right)
$$

$$
\rho_{\mathrm{total}} = -0.030651
$$

So the full GPS formula is violated.

## Formula 3 — Gyroscope Integrity

**Script:** `offline_stl_gyro.py`  
**Plot:** `stl_result_gyro.png`

### Formula

$$G_{[0,580\,\mathrm{ms}]}\left(e_{\mathrm{gyr},x}<0.15 \land e_{\mathrm{gyr},y}<0.15 \land e_{\mathrm{gyr},z}<0.15\right)$$

### Correct Residual Formation

$$e_{\mathrm{gyr},x}(t)=\left|m_{\mathrm{gyr},x}(t)-m_{s,\mathrm{gyr},x}(t)\right|$$

$$e_{\mathrm{gyr},y}(t)=\left|m_{\mathrm{gyr},y}(t)-m_{s,\mathrm{gyr},y}(t)\right|$$

$$e_{\mathrm{gyr},z}(t)=\left|m_{\mathrm{gyr},z}(t)-m_{s,\mathrm{gyr},z}(t)\right|$$

### Threshold

$$\varepsilon_{\mathrm{gyr}}=0.15\,\mathrm{rad/s}$$

### Formula Formation

For each axis:

$$e_{\mathrm{gyr},x}(t)<\varepsilon_{\mathrm{gyr}}$$

$$e_{\mathrm{gyr},y}(t)<\varepsilon_{\mathrm{gyr}}$$

$$e_{\mathrm{gyr},z}(t)<\varepsilon_{\mathrm{gyr}}$$

Substitute the threshold:

$$e_{\mathrm{gyr},x}(t)<0.15$$

$$e_{\mathrm{gyr},y}(t)<0.15$$

$$e_{\mathrm{gyr},z}(t)<0.15$$

All three axes must be safe:

$$e_{\mathrm{gyr},x}<0.15 \land e_{\mathrm{gyr},y}<0.15 \land e_{\mathrm{gyr},z}<0.15$$

Then add the STL global operator:

$$G_{[0,580\,\mathrm{ms}]}\left(e_{\mathrm{gyr},x}<0.15 \land e_{\mathrm{gyr},y}<0.15 \land e_{\mathrm{gyr},z}<0.15\right)$$

### Example

Suppose:

$$m_{s,\mathrm{gyr},x}=0.05\,\mathrm{rad/s}$$

$$m_{\mathrm{gyr},x}=0.80\,\mathrm{rad/s}$$

Then:

$$e_{\mathrm{gyr},x}=\left|0.80-0.05\right|=0.75\,\mathrm{rad/s}$$

Compare:

$$0.75>0.15$$

So this is a violation.

Robustness:

$$\rho_x=0.15-0.75=-0.60$$

If:

$$e_{\mathrm{gyr},y}=0.02$$

$$e_{\mathrm{gyr},z}=0.01$$

then:

$$\rho_y=0.15-0.02=+0.13$$

$$\rho_z=0.15-0.01=+0.14$$

For the AND operator:

$$\rho_{\mathrm{total}}=\min(-0.60,+0.13,+0.14)$$

$$\rho_{\mathrm{total}}=-0.60$$

So the gyroscope formula is violated because the X-axis is attacked.

## Formula 4 — Multi-Sensor Compound Spec

**Script:** `offline_stl_multi_sensor.py`  
**Plot:** `stl_result_multi_sensor.png`

### Formula

$$G_{[0,580\,\mathrm{ms}]}\left(h>0.97 \land h<29.70 \land e_{\mathrm{baro}}<0.30 \land e_{\mathrm{GPS,N}}<0.169349 \land e_{\mathrm{GPS,E}}<0.169349 \land e_{\mathrm{gyr},x}<0.15 \land e_{\mathrm{gyr},y}<0.15 \land e_{\mathrm{gyr},z}<0.15\right)$$

## Formula 5 — Persistent Barometer Attack Pattern

**Script:** `offline_stl_baro_persistent.py`  
**Plot:** `stl_result_baro_persistent.png`

### Formula

$$G_{[0,580\,\mathrm{ms}]}\left(G_{[0,1000\,\mathrm{ms}]}\left(e_{\mathrm{baro}}<0.30\right)\right)$$

### Purpose

This detects whether the barometer residual remains safe over a longer temporal pattern, not just at one instant.

### Full Formula

$$G_{[0,580\,\mathrm{ms}]}\left(G_{[0,1000\,\mathrm{ms}]}\left(e_{\mathrm{baro}}<0.30\right)\right)$$
## Formula 6 — Barometer Recovery Within 10 s

**Script:** `offline_stl_baro_recovery.py`  
**Plot:** `stl_result_baro_recovery.png`

### Formula

$$G_{[0,580\,\mathrm{ms}]}\left(e_{\mathrm{baro}}>0.30 \rightarrow F_{[0,10000\,\mathrm{ms}]}\left(e_{\mathrm{baro}}<0.30\right)\right)$$

### Purpose

This checks whether a barometer attack is followed by recovery within 10 seconds.

### Formula Formation

Attack condition:

$$e_{\mathrm{baro}}>0.30$$

This means the barometer residual is above the safe threshold.

Recovery condition:

$$e_{\mathrm{baro}}<0.30$$

This means the barometer residual has returned to the safe range.

Recovery within 10 seconds:

$$F_{[0,10000\,\mathrm{ms}]}\left(e_{\mathrm{baro}}<0.30\right)$$

Attack implies recovery:

$$e_{\mathrm{baro}}>0.30 \rightarrow F_{[0,10000\,\mathrm{ms}]}\left(e_{\mathrm{baro}}<0.30\right)$$

Monitor this rule globally over 580 ms:

$$G_{[0,580\,\mathrm{ms}]}\left(e_{\mathrm{baro}}>0.30 \rightarrow F_{[0,10000\,\mathrm{ms}]}\left(e_{\mathrm{baro}}<0.30\right)\right)$$
## Formula 8 — Altitude Bounds / Mission Spec S3-S4 Equivalent

**Script:** `offline_stl_altitude_bounds.py`  
**Plot:** `stl_result_altitude_bounds.png`

### Formula

$$G_{[0,580\,\mathrm{ms}]}\left(h>0.97 \land h<29.70\right)$$

### Purpose

This checks whether altitude remains within the guide-defined mission bounds.

### Example 1 — Safe Altitude

Suppose:

$$h=10.0\,\mathrm{m}$$

Then:

$$10.0>0.97$$

$$10.0<29.70$$

So the formula is satisfied.

Robustness for the lower bound:

$$\rho_{\mathrm{lower}}=h-0.97=10.0-0.97=9.03$$

Robustness for the upper bound:

$$\rho_{\mathrm{upper}}=29.70-h=29.70-10.0=19.70$$

For the AND operator:

$$\rho_{\mathrm{total}}=\min(9.03,19.70)$$

$$\rho_{\mathrm{total}}=9.03$$

Positive robustness means the altitude formula is satisfied.

### Example 2 — Unsafe Altitude

Suppose:

$$h=30.0\,\mathrm{m}$$

Then the upper-bound condition is false:

$$30.0<29.70$$

Upper-bound robustness:

$$\rho_{\mathrm{upper}}=29.70-30.0=-0.30$$

So the altitude formula is violated.


