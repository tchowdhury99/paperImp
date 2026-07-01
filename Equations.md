
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

### STL Formula

$$
G_{[0,2000\text{ ms}]}\left(\text{baro\_res} < 0.30\right)
$$


