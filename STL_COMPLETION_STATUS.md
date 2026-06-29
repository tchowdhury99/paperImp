# STL Implementation Completion Status

Project path:

`/home/tchowdh4/paperImp/`

Python interpreter used:

`/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`

Primary dataset:

`/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat`

Sampling:

`Ts = 0.02 s`, `fs = 50 Hz`, `20 ms per sample`

Segment used:

`segment 0`

## Completed STL formulas

1. Barometer Integrity

- Script: `offline_stl_baro.py`
- Plot: `stl_result_baro.png`
- Formula: `G[0:2000ms] (baro_res < 0.30)`

2. GPS Integrity

- Script: `offline_stl_gps.py`
- Plot: `stl_result_gps.png`
- Formula: `G[0:580ms] (gps_north_res < 0.169349 and gps_east_res < 0.169349)`

3. Gyroscope Integrity

- Script: `offline_stl_gyro.py`
- Plot: `stl_result_gyro.png`
- Formula: `G[0:580ms] (gyro_residual_x < 0.15 and gyro_residual_y < 0.15 and gyro_residual_z < 0.15)`

4. Multi-Sensor Compound Spec

- Script: `offline_stl_multi_sensor.py`
- Plot: `stl_result_multi_sensor.png`
- Formula: `G[0:580ms] ((alt > 0.97) and (alt < 29.70) and (baro_residual < 0.30) and (gps_north_residual < 0.169349) and (gps_east_residual < 0.169349) and (gyro_residual_x < 0.15) and (gyro_residual_y < 0.15) and (gyro_residual_z < 0.15))`

5. Persistent Barometer Attack Pattern

- Script: `offline_stl_baro_persistent.py`
- Plot: `stl_result_baro_persistent.png`
- Formula: `G[0:580ms] (G[0:1000ms] (baro_residual < 0.30))`

6. Barometer Recovery Within 10 s

- Script: `offline_stl_baro_recovery.py`
- Plot: `stl_result_baro_recovery.png`
- Formula: `G[0:580ms] ((baro_residual > 0.30) -> F[0:10000ms] (baro_residual < 0.30))`

7. Multi-Sensor Any-Attack Recovery

- Script: `offline_stl_any_attack_recovery.py`
- Plot: `stl_result_any_attack_recovery.png`
- Formula: `G[0:580ms] (((baro_residual > 0.30) or (gyro_residual_x > 0.15)) -> F[0:10000ms] (not ((baro_residual > 0.30) or (gyro_residual_x > 0.15))))`

8. Altitude Bounds / Mission Spec S3-S4 equivalent

- Script: `offline_stl_altitude_bounds.py`
- Plot: `stl_result_altitude_bounds.png`
- Formula: `G[0:580ms] ((alt > 0.97) and (alt < 29.70))`

## Final verification

All 8 scripts exist.

All 8 result plots exist.

Remaining formulas:

`0`
