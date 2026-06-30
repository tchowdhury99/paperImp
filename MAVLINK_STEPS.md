# How the MAVLink Part Was Completed — Step by Step

**Goal of this part:** turn the offline STL work into a *real-time* monitor that reads a drone's
sensors over **MAVLink**, evaluates STL on every sample, and triggers the paper's closed-loop
recovery — exactly what Choi et al. (RAID 2020) Algorithm 1 is meant to do at runtime.

**Interpreter:** `/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3`
**Library:** `pymavlink 2.4.49` (already installed)
**Date completed:** 2026-06-30

Two MAVLink paths were built and verified:

1. **Live ArduPilot SITL** — proves the monitor's MAVLink receive path works against a *real
   autopilot* (genuine heartbeat + live sensor streams).
2. **MAVLink replay bridge** — streams the recorded 50 Hz dataset as MAVLink traffic so the full
   STL-detect → recover loop runs deterministically and reproducibly on known data.

---

## Step 1 — Confirm the MAVLink toolchain is present

```bash
/home/tchowdh4/.pyenv/versions/3.10.14/bin/python3 -c "import pymavlink; print(pymavlink.__version__)"
# -> 2.4.49
```

`pymavlink` provides `mavutil.mavlink_connection(...)` (the UDP/TCP endpoint), `wait_heartbeat()`
(the connection handshake), `recv_match(type=...)` (message decode), and `mav.*_send(...)`
(message encode). These are the only MAVLink primitives needed.

---

## Step 2 — Prove a genuine live autopilot link (ArduPilot SITL)

Launch the pre-built SITL quadcopter binary (it simulates flight physics internally and exposes
MAVLink on TCP 5760):

```bash
ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter-quad \
    --model quad \
    --home 40.0713,-105.2300,1584,0 \
    --defaults ardupilot_ws/arducopter-3.4/Tools/autotest/default_params/copter.parm
# -> "bind port 5760 for 0 ... Serial port 0 on TCP port 5760 ... Waiting for connection"
```

Connect with pymavlink and read live data:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()                       # -> system 1, type 2 (QUADROTOR), autopilot 3 (ArduPilotMega)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
```

**Verified live messages (≈23 Hz each):** `RAW_IMU` (xgyro/ygyro/zgyro), `SCALED_PRESSURE`
(`press_abs = 836.9 hPa` ≈ the dataset's ~83.6 kPa ground pressure), `ATTITUDE`
(roll/pitch/yaw + rollspeed = body rates), `GLOBAL_POSITION_INT` (`relative_alt` = fused
altitude estimate). These are **exactly** the channels the STL monitor needs:

| Monitor needs | Live MAVLink source |
|---|---|
| barometer altitude (m, attackable) | `SCALED_PRESSURE.press_abs` → altitude |
| fused altitude estimate (ms for ALT) | `GLOBAL_POSITION_INT.relative_alt` |
| roll rate p (m, attackable) | `RAW_IMU.xgyro` or `ATTITUDE.rollspeed` |

This is the key result: **the monitor's MAVLink path is real-autopilot-ready.**

---

## Step 3 — Why a replay bridge, not a full SITL flight, for the closed-loop test

The identified software-sensor model needs the project's specific control-input vector
`u = [phi_cmd, theta_cmd, psi_cmd, thr, tiltN, tiltE, const]` (the `Useg` produced by system
identification). A live SITL flight does not emit that exact vector, and a full armed mission is
non-deterministic and heavy. To exercise the *complete* detect-and-recover loop on **known data
with a known injected attack**, the recorded `operation_data_50hz.mat` (itself real SITL flight
data) is **replayed as MAVLink traffic**. The monitor's receive code is identical either way.

---

## Step 4 — Build the MAVLink replay source (`mavlink_replay_source.py`)

Transport: standard MAVLink `HEARTBEAT` + `NAMED_VALUE_FLOAT` (a built-in message carrying a
≤10-char name and a float — the canonical way to stream arbitrary named scalars over MAVLink).

Per 50 Hz tick the source sends:

```
HEARTBEAT                 (≈2 Hz)  -> lets the monitor's wait_heartbeat() succeed
U0..U6   (7 floats)                -> control input u(t)            (drives y=Cx+Du, x=Ax+Bu)
Y0..Y11  (12 floats)               -> state/output y(t)             (monitor inits model state x)
BARO     (1 float)                 -> barometer altitude  [ATTACKED on the wire]
ALTEST   (1 float)                 -> fused altitude estimate (= Yseg[:,2])
PMEAS    (1 float)                 -> measured roll rate p [ATTACKED on the wire]
TICK     (1 float)                 -> sample index (sample-alignment marker)
```

The **attack is injected here, on the wire** (a compromised sensor), so the monitor receives
already-corrupted measurements — the real threat model:

```bash
# barometer spoof: +3.0 m bias over samples 2000–2500 (40–50 s)
python3 mavlink_replay_source.py --out udpout:127.0.0.1:14570 --attack baro --offset 3.0 --rate 1000
# gyro spoof: roll-rate p set to 0.8 rad/s over the same window
python3 mavlink_replay_source.py --out udpout:127.0.0.1:14570 --attack gyro --gyroval 0.8 --rate 1000
```

`--rate 1000` streams 1000 ticks/s (20× real time) so a 140 s flight replays in ≈7 s; STL is
causal per-sample, so wire speed does not change the result.

---

## Step 5 — Build the online STL monitor (`online_stl_mavlink.py`)

```python
mav = mavutil.mavlink_connection('udpin:127.0.0.1:14570')
mav.wait_heartbeat()                                   # MAVLink handshake
while True:
    msg = mav.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=1.0)
    cur[msg.name] = msg.value                          # assemble one sample, keyed by TICK
    # on each completed TICK:
    y = C @ x + D @ u ;  x = A @ x + B @ u             # software-sensor model step
    resid_alt = abs(BARO - ALTEST)                     # ALT residual
    resid_p   = abs(lpf(PMEAS) - y[9])                 # P residual (model prediction)
    rho_alt = spec_alt.update(tick, [('resid_alt', resid_alt)])   # rtamt ONLINE STL
    rho_p   = spec_p.update(tick,  [('resid_p',   resid_p)])
    # rho<0 -> enter recovery: m <- ms ; leave after K consecutive safe samples
```

The STL specs are causal atomic predicates evaluated online:
`resid_alt < 0.30` and `resid_p < 0.15`, so `rho = eps − resid` each tick (no future-window
artifact). When `rho < 0` the monitor enters **recovery** and replaces the attacked sensor with
the software sensor (`m ← ms`), leaving recovery after `K = 10` consecutive safe samples.

---

## Step 6 — Run the loop and verify (monitor first, then source)

```bash
# terminal A — monitor listens on UDP 14570
python3 online_stl_mavlink.py --conn udpin:127.0.0.1:14570 --plot stl_result_online_baro.png
# terminal B — source streams the attacked flight
python3 mavlink_replay_source.py --out udpout:127.0.0.1:14570 --attack baro --offset 3.0 --rate 1000
```

**Verified results (all over the MAVLink link):**

| Case | ALT (baro) | P (gyro) |
|---|---|---|
| baro spoof +3.0 m | **detected t = 40.00 s**, recovery 510 samples | clean |
| gyro spoof 0.8 rad/s | clean | **detected t = 40.02 s**, recovery 513 samples |
| clean (no attack) | no violation | no violation |

Detection lands at the true attack onset (40.0 s) with **~0 latency** — a real online monitor,
not the backward-looking offline analysis. No false positives on clean data. Plots saved:
`stl_result_online_baro.png`, `stl_result_online_gyro.png`, `stl_result_online_clean.png`.

---

## Step 7 — Map back to a live autopilot

Because Step 2 proved `SCALED_PRESSURE`, `GLOBAL_POSITION_INT.relative_alt`, and
`RAW_IMU.xgyro`/`ATTITUDE.rollspeed` all stream from the real ArduPilot SITL, swapping the
replay source for a live SITL is a thin adapter (read those messages instead of the
`NAMED_VALUE_FLOAT` channels). The STL + recovery core is unchanged. That adapter — plus driving
a live *armed* flight with the project's `u` vector — is the remaining online step (see
`STL_REMAINING_WORK_COMPLETED.md`, "Remaining work").

---

## Files produced in this part

```
mavlink_replay_source.py            # MAVLink data-source bridge (dataset -> MAVLink, with attack)
online_stl_mavlink.py               # online STL monitor + closed-loop recovery over MAVLink
stl_result_online_baro.png          # baro-spoof run
stl_result_online_gyro.png          # gyro-spoof run
stl_result_online_clean.png         # clean run (false-positive check)
```
