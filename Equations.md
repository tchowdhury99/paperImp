
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
