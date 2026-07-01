
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
\mathrm{gps\_north\_res} < 0.169349
\;\land\;
\mathrm{gps\_east\_res} < 0.169349
\right)
$$

### Threshold

$$
\varepsilon_{\mathrm{gps}} = 0.169349
$$

### Formula Formation

Start with GPS north safety:

$$
\mathrm{gps\_north\_error}(t) < \varepsilon_{\mathrm{gps}}
$$

Substitute the threshold:

$$
\mathrm{gps\_north\_error}(t) < 0.169349
$$

For both GPS north and GPS east residuals, the STL formula becomes:

$$
G_{[0,580\,\mathrm{ms}]}
\left(
\mathrm{gps\_north\_res} < 0.169349
\;\land\;
\mathrm{gps\_east\_res} < 0.169349
\right)
$$

### Example

Suppose:

$$
\mathrm{gps\_north\_error} = 0.05
$$

$$
\mathrm{gps\_east\_error} = 0.08
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
\mathrm{gps\_north\_error} = 0.20
$$

$$
\mathrm{gps\_east\_error} = 0.08
$$

North violates the threshold:

$$
0.20 > 0.169349
$$

Robustness for north:

$$
\rho_{\mathrm{north}} = 0.169349 - 0.20
$$

$$
\rho_{\mathrm{north}} = -0.030651
$$

East is safe:

$$
\rho_{\mathrm{east}} = 0.169349 - 0.08
$$

$$
\rho_{\mathrm{east}} = +0.089349
$$

For the AND operator, STL uses the weaker condition:

$$
\rho_{\mathrm{total}}
=
\min
\left(
\rho_{\mathrm{north}},
\rho_{\mathrm{east}}
\right)
$$

$$
\rho_{\mathrm{total}}
=
\min
\left(
-0.030651,
+0.089349
\right)
$$

$$
\rho_{\mathrm{total}} = -0.030651
$$

So the full GPS formula is violated.
