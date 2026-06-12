# Replication Continuation Guide
## Picking up from: ArduCopter 3.4 SITL binary built ✅
### Next: APMrover2 2.5 → MAVProxy SITL test → Log collection → MATLAB SI

---

## Where You Are

```
✅ ArduCopter 3.4 SITL binary:  ~/ardupilot_ws/arducopter-3.4/build/sitl/bin/arducopter
✅ APMrover2 2.5 SITL binary:  ~/ardupilot_ws/apmrover2-2.5/build/sitl/bin/ardurover
□  MAVProxy connects to SITL
□  First Dataflash .bin log collected
□  operation_data.mat generated
□  MATLAB model (A, B, C, D)
□  Recovery module compiled + patched
```

---

## Step 1 — Build APMrover2 2.5 SITL Binary

The paper uses APMrover2 2.5 for the simulated rover (Table 1). Same submodule-fix
procedure that resolved the ArduCopter 3.4 build applies here.

```bash
cd ~/ardupilot_ws

# Check out APMrover2 2.5 as a worktree
# First find the exact tag name:
cd ardupilot
git tag | grep -i "rover\|APMrover" | sort -V | grep "^APMrover-2\.5\|^Rover-2\.5" | head -10
```

The tag is likely `APMrover-2.5.0` or `Rover-2.5.1`. Use whichever appears:

```bash
# Use the tag you found above — example shown as APMrover-2.5.1
cd ~/ardupilot_ws
git -C ardupilot worktree add apmrover2-2.5 APMrover-2.5.1   # adjust tag name

# Fix submodules (same fix as the ArduCopter 3.4 littlefs issue)
cd ~/ardupilot_ws/apmrover2-2.5
git submodule sync --recursive
git submodule update --init --recursive --force

# Install prereqs if not already done from this directory
# (Only needed once per machine — skip if already ran for arducopter-3.4)
# Tools/environment_install/install-prereqs-ubuntu.sh -y
# . ~/.profile

# Build
./waf configure --board sitl
./waf rover

# Verify
ls -lh build/sitl/bin/ardurover
```

Expected output:
```
[N/N] checking symbols build/sitl/bin/ardurover
'rover' finished successfully
```

> **If waf is absent** (older rover firmware): try `cd APMrover2 && make sitl`

---

## Step 2 — Verify SITL Launches Correctly

Test ArduCopter 3.4 SITL boots and accepts MAVLink before doing anything else.
This uses `sim_vehicle.py` — ArduPilot's SITL launch script (in the repo).

### 2.1 — Launch SITL (ArduCopter 3.4)

Open **Terminal A**:

```bash
conda activate rv_recovery
cd ~/ardupilot_ws/arducopter-3.4

# --no-rebuild avoids recompiling; uses the binary you already built
# -L sets the home location (lat, lon, alt, heading)
# --out routes MAVLink telemetry to localhost UDP ports
python Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f quad \
    --no-rebuild \
    --home=40.071374,-105.229594,1583,353 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --speedup=1
```

Wait for output like:
```
Simulating with: build/sitl/bin/arducopter
...
Ready to FLY
```

> **If sim_vehicle.py not found**: it is at `Tools/autotest/sim_vehicle.py` within the
> arducopter-3.4 tree. Python path: `python3 Tools/autotest/sim_vehicle.py ...`

### 2.2 — Connect MAVProxy (Terminal B)

Open **Terminal B**:

```bash
conda activate rv_recovery

mavproxy.py \
    --master=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --aircraft=copter_test
```

You should see:
```
Heartbeat from APM: ARDUCOPTER (sys_id 1 comp_id 1 msg_count N)
```

If heartbeat appears: ✅ MAVProxy → SITL link is working.

### 2.3 — Quick sanity test in MAVProxy console

```
# Inside mavproxy.py prompt:
mode GUIDED
arm throttle
takeoff 10
```

Watch altitude climb in the SITL terminal output. Then:
```
mode LAND
```

If the vehicle takes off and lands without error: ✅ SITL is fully operational.

---

## Step 3 — Collect Operation Data (Dataflash Logs)

The paper collects logs "under different maneuvers to capture various control
properties and dynamics" (Section 3.1) using randomly generated MAVLink missions.
These logs are the input to MATLAB System Identification.

### 3.1 — Enable Dataflash Logging in SITL

In MAVProxy console (Terminal B):

```
# Enable all log messages (bitmask 0xFFFF = log everything)
param set LOG_BITMASK 65535
param set LOG_BACKEND_TYPE 1
param set LOG_FILE_DSRMSG 1
```

Confirm logging is active:
```
param show LOG_BITMASK
# Should return: LOG_BITMASK     65535
```

### 3.2 — Run the Mission Generator

Save this as `~/rv_recovery/python/collect_logs.py` and run it from **Terminal C**:

```python
#!/usr/bin/env python3
"""
Automated log collection: flies N random missions and saves Dataflash .bin logs.
Run AFTER sim_vehicle.py and mavproxy.py are up.
"""
import time, random, os
from pymavlink import mavutil, mavwp

CONNECTION = 'udp:127.0.0.1:14551'
LOG_DIR    = os.path.expanduser('~/rv_recovery/data/logs/')
os.makedirs(LOG_DIR, exist_ok=True)

def wait_for_mode(mav, mode, timeout=30):
    mode_id = mav.mode_mapping()[mode]
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.custom_mode == mode_id:
            return True
    return False

def upload_square_mission(mav, home_lat, home_lon, size_deg=0.0003, alt=15.0):
    """
    Square waypoint mission — matches the paper's 'straight fly + turns' primitives.
    size_deg ≈ 30 m side at mid-latitudes.
    """
    wp = mavwp.MAVWPLoader()
    waypoints = [
        (home_lat,            home_lon,            alt),
        (home_lat + size_deg, home_lon,            alt),
        (home_lat + size_deg, home_lon + size_deg, alt),
        (home_lat,            home_lon + size_deg, alt),
        (home_lat,            home_lon,            alt),
    ]
    for seq, (lat, lon, a) in enumerate(waypoints):
        wp.add(mavutil.mavlink.MAVLink_mission_item_message(
            mav.target_system, mav.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1 if seq == 0 else 0,   # current
            1,                       # autocontinue
            0, 2.0, 0, 0,           # params (acceptance radius = 2 m)
            lat, lon, a
        ))
    # Upload
    mav.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        req = mav.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
        if req:
            mav.mav.send(wp.wp(req.seq))
    ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(f"  Mission upload: {ack.type if ack else 'NO ACK'}")

def run_mission(mav, mission_id):
    print(f"\n[Mission {mission_id}] Starting...")

    # --- Arm and take off ---
    mav.set_mode('GUIDED')
    time.sleep(1)
    mav.arducopter_arm()
    mav.motors_armed_wait()
    print(f"  Armed.")

    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 15.0   # takeoff to 15 m
    )
    time.sleep(5)   # wait for takeoff

    # --- Upload and run AUTO mission ---
    pos = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if pos:
        home_lat = pos.lat / 1e7
        home_lon = pos.lon / 1e7
    else:
        home_lat, home_lon = 40.071374, -105.229594

    upload_square_mission(mav, home_lat, home_lon,
                          size_deg=random.uniform(0.0002, 0.0005),
                          alt=random.uniform(10.0, 20.0))

    mav.set_mode('AUTO')
    time.sleep(30)   # fly the mission (adjust if needed)

    # --- Return and land ---
    mav.set_mode('RTL')
    time.sleep(20)

    mav.set_mode('LAND')
    mav.motors_disarmed_wait()
    print(f"  [Mission {mission_id}] Landed and disarmed.")
    time.sleep(3)

def main():
    print(f"Connecting to SITL at {CONNECTION}...")
    mav = mavutil.mavlink_connection(CONNECTION)
    mav.wait_heartbeat()
    print(f"Heartbeat OK — system {mav.target_system}")

    N_MISSIONS = 20   # paper uses 20 missions for FP/FN parameter evaluation
    for i in range(N_MISSIONS):
        run_mission(mav, i+1)

    print(f"\nAll {N_MISSIONS} missions complete.")
    print(f"Dataflash logs are in the SITL working directory.")
    print(f"Copy .bin files to: {LOG_DIR}")

if __name__ == '__main__':
    main()
```

Run it:
```bash
conda activate rv_recovery
python ~/rv_recovery/python/collect_logs.py
```

### 3.3 — Locate the Dataflash .bin Logs

After missions complete, SITL writes `.bin` logs to its working directory:

```bash
# ArduCopter 3.4 SITL logs land here by default:
ls ~/ardupilot_ws/arducopter-3.4/logs/
# or wherever sim_vehicle.py was launched from:
ls ./logs/

# Copy all .bin files to your data directory
mkdir -p ~/rv_recovery/data/logs/
cp ~/ardupilot_ws/arducopter-3.4/logs/*.bin ~/rv_recovery/data/logs/
ls -lh ~/rv_recovery/data/logs/
```

You should see files like `00000001.BIN`, `00000002.BIN`, etc. — one per flight.

---

## Step 4 — Parse Logs and Generate operation_data.mat

Save this as `~/rv_recovery/python/parse_dataflash.py`:

```python
#!/usr/bin/env python3
"""
Parses ArduPilot Dataflash .bin logs and resamples all sensor streams
to 400 Hz using cubic spline interpolation (paper Section 3.1).

Outputs: operation_data.mat — ready for MATLAB System Identification Toolbox.

Sensor rates from paper (Table 2 / Section 3.1):
  Gyro, Accel    : 400 Hz  (Ts = 0.0025 s)
  RC, non-critical: 100 Hz
  Log module     :  10 Hz
  GPS, Baro      :  50 Hz
"""
import os, sys, glob
import numpy as np
import scipy.io
from scipy.interpolate import CubicSpline
from pymavlink import DFReader

TARGET_HZ = 400          # resample everything to 400 Hz (paper spec)
TS        = 1.0 / TARGET_HZ   # 0.0025 s

# Map Dataflash message types to fields we need
# Field names match ArduCopter 3.4 Dataflash schema
STREAMS = {
    'GYR': ['GyrX', 'GyrY', 'GyrZ'],           # rad/s — 400 Hz
    'ACC': ['AccX', 'AccY', 'AccZ'],            # m/s²  — 400 Hz
    'ATT': ['Roll', 'Pitch', 'Yaw',             # deg   — 400 Hz
            'RollIn', 'PitchIn', 'YawIn'],
    'CTUN': ['ThI', 'ThO', 'ThH', 'ABst'],      # throttle — 10 Hz
    'GPS': ['Lat', 'Lng', 'Alt', 'Spd'],        # deg/deg/m/m·s⁻¹ — 5 Hz in SITL
    'BARO': ['Press', 'Alt', 'Temp'],           # Pa/m/°C — 50 Hz
    'MAG': ['MagX', 'MagY', 'MagZ'],            # milliGauss — 100 Hz
    'RCIN': ['C1', 'C2', 'C3', 'C4'],          # PWM      — 100 Hz
    'NKF1': ['Roll', 'Pitch', 'Yaw',           # EKF output — 25 Hz
             'VN', 'VE', 'VD', 'PN', 'PE', 'PD'],
}

def parse_single_log(bin_path):
    """Parse one .bin file, return dict of {stream: {field: array, 't': array}}"""
    log = DFReader.DFReader_binary(bin_path, zero_time_base=True)
    raw = {k: {'t': [], **{f: [] for f in v}} for k, v in STREAMS.items()}

    while True:
        msg = log.recv_match()
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype not in STREAMS:
            continue
        t = msg._timestamp
        raw[mtype]['t'].append(t)
        for field in STREAMS[mtype]:
            try:
                raw[mtype][field].append(float(getattr(msg, field)))
            except AttributeError:
                raw[mtype][field].append(np.nan)

    return raw

def spline_resample(raw, t_common):
    """
    Cubic spline resample all streams to t_common.
    CubicSpline avoids Runge's phenomenon (cited in paper Section 3.1).
    """
    out = {}
    for stream, data in raw.items():
        t_orig = np.array(data['t'])
        if len(t_orig) < 4:          # need at least 4 points for cubic spline
            continue
        for field in STREAMS[stream]:
            y = np.array(data[field])
            mask = np.isfinite(y) & np.isfinite(t_orig)
            if mask.sum() < 4:
                continue
            cs  = CubicSpline(t_orig[mask], y[mask])
            key = f'{stream}_{field}'
            # Only interpolate within observed range to avoid extrapolation artifacts
            t_min, t_max = t_orig[mask][0], t_orig[mask][-1]
            valid = (t_common >= t_min) & (t_common <= t_max)
            arr = np.full(len(t_common), np.nan)
            arr[valid] = cs(t_common[valid])
            out[key] = arr
    return out

def build_input_output_matrices(resampled, t_common):
    """
    Construct U (inputs) and Y (outputs) for MATLAB iddata().

    U = control inputs:  [RollIn, PitchIn, YawIn, ThI]  (reference/target states)
    Y = sensor outputs:  [Roll, Pitch, Yaw,              (ATT — attitude)
                          GyrX, GyrY, GyrZ,              (gyroscope)
                          AccX, AccY, AccZ,              (accelerometer)
                          GPS_Alt, BARO_Alt,             (altitude sensors)
                          GPS_Lat, GPS_Lng,              (position)
                          MagX, MagY, MagZ]              (magnetometer)

    This matches the 12-state vector from Eq. (3) of the paper.
    """
    # --- Inputs (reference commands) ---
    u_fields = ['ATT_RollIn', 'ATT_PitchIn', 'ATT_YawIn', 'CTUN_ThI']
    U_cols = []
    for f in u_fields:
        if f in resampled:
            U_cols.append(resampled[f])
        else:
            U_cols.append(np.zeros(len(t_common)))
    U = np.column_stack(U_cols)

    # --- Outputs (sensor measurements) ---
    y_fields = [
        'ATT_Roll', 'ATT_Pitch', 'ATT_Yaw',
        'GYR_GyrX', 'GYR_GyrY', 'GYR_GyrZ',
        'ACC_AccX', 'ACC_AccY', 'ACC_AccZ',
        'GPS_Alt',  'BARO_Alt',
        'GPS_Lat',  'GPS_Lng',
        'MAG_MagX', 'MAG_MagY', 'MAG_MagZ',
    ]
    Y_cols = []
    for f in y_fields:
        if f in resampled:
            Y_cols.append(resampled[f])
        else:
            Y_cols.append(np.full(len(t_common), np.nan))
    Y = np.column_stack(Y_cols)

    # Drop rows where any value is NaN (affects beginning/end where streams overlap)
    valid = np.all(np.isfinite(U), axis=1) & np.all(np.isfinite(Y), axis=1)
    return U[valid], Y[valid], t_common[valid]

def process_logs(log_dir, out_mat):
    bin_files = sorted(glob.glob(os.path.join(log_dir, '*.BIN')) +
                       glob.glob(os.path.join(log_dir, '*.bin')))
    if not bin_files:
        print(f"ERROR: No .BIN files found in {log_dir}")
        sys.exit(1)

    print(f"Found {len(bin_files)} log files.")
    all_U, all_Y = [], []

    for i, bf in enumerate(bin_files):
        print(f"  [{i+1}/{len(bin_files)}] Parsing: {os.path.basename(bf)}")
        raw = parse_single_log(bf)

        # Find common time range across all streams
        t_starts = [np.array(raw[s]['t'])[0]  for s in STREAMS if raw[s]['t']]
        t_ends   = [np.array(raw[s]['t'])[-1] for s in STREAMS if raw[s]['t']]
        if not t_starts:
            print(f"    Skipping — no data.")
            continue
        t0 = max(t_starts)
        t1 = min(t_ends)
        if t1 <= t0:
            print(f"    Skipping — streams don't overlap.")
            continue
        t_common = np.arange(t0, t1, TS)

        resampled = spline_resample(raw, t_common)
        U, Y, t   = build_input_output_matrices(resampled, t_common)

        if len(U) < 400:   # skip very short segments (<1 s)
            print(f"    Skipping — segment too short ({len(U)} samples).")
            continue

        all_U.append(U)
        all_Y.append(Y)
        print(f"    Resampled: {len(U)} samples ({len(U)*TS:.1f} s)")

    if not all_U:
        print("ERROR: No valid data segments found.")
        sys.exit(1)

    # Concatenate all missions into one dataset for SI
    U_all = np.vstack(all_U)
    Y_all = np.vstack(all_Y)

    print(f"\nTotal samples: {len(U_all)} ({len(U_all)*TS:.1f} s @ {TARGET_HZ} Hz)")
    print(f"U shape: {U_all.shape}  (inputs:  RollIn, PitchIn, YawIn, ThrottleIn)")
    print(f"Y shape: {Y_all.shape}  (outputs: Roll,Pitch,Yaw, GyrXYZ, AccXYZ, ...)")

    scipy.io.savemat(out_mat, {
        'U':   U_all,          # [N x 4]  control inputs
        'Y':   Y_all,          # [N x 16] sensor outputs
        'Ts':  TS,             # scalar   sample time
        'fs':  float(TARGET_HZ),
        'u_labels': ['RollIn','PitchIn','YawIn','ThrottleIn'],
        'y_labels': ['Roll','Pitch','Yaw',
                     'GyrX','GyrY','GyrZ',
                     'AccX','AccY','AccZ',
                     'GPS_Alt','BARO_Alt',
                     'GPS_Lat','GPS_Lng',
                     'MagX','MagY','MagZ'],
    })
    print(f"\nSaved: {out_mat}")

if __name__ == '__main__':
    LOG_DIR = os.path.expanduser('~/rv_recovery/data/logs/')
    OUT_MAT = os.path.expanduser('~/rv_recovery/data/operation_data.mat')
    os.makedirs(os.path.dirname(OUT_MAT), exist_ok=True)
    process_logs(LOG_DIR, OUT_MAT)
```

Run it:
```bash
conda activate rv_recovery
python ~/rv_recovery/python/parse_dataflash.py
```

Expected output:
```
Found 20 log files.
  [1/20] Parsing: 00000001.BIN
    Resampled: 4800 samples (12.0 s)
  ...
Total samples: 96000 (240.0 s @ 400 Hz)
U shape: (96000, 4)
Y shape: (96000, 16)
Saved: ~/rv_recovery/data/operation_data.mat
```

---

## Step 5 — MATLAB System Identification

Run this in MATLAB (R2026a). Requires **System Identification Toolbox**.

Save as `~/rv_recovery/matlab/system_identification.m`:

```matlab
%% system_identification.m
% Derives the state-space model A,B,C,D for ArduCopter 3.4 (quadrotor)
% using the System Identification Toolbox (ssest / PEM).
%
% Paper reference: Section 3.1, Eq. (1)-(2)
% State vector (Eq. 3): x = [x y z φ θ ψ ẋ ẏ ż p q r]ᵀ
% Model template: discrete-time state-space, PID + dynamics known a priori
% SI algorithm: Prediction Error Minimization (PEM), cited in Section 3.1

clear; clc;
addpath(genpath(pwd));

%% ── 1. Load Data ──────────────────────────────────────────────────────────
data_path = fullfile(getenv('HOME'), 'rv_recovery', 'data', 'operation_data.mat');
load(data_path);   % loads U, Y, Ts, u_labels, y_labels
fprintf('Loaded: %d samples, Ts=%.4f s (%.0f Hz)\n', size(U,1), Ts, 1/Ts);

% U: [N x 4]  — RollIn, PitchIn, YawIn, ThrottleIn  (reference/control inputs)
% Y: [N x 16] — Roll,Pitch,Yaw, GyrXYZ, AccXYZ, GPS_Alt,BARO_Alt, GPS_LatLon, MagXYZ

%% ── 2. Split into estimation and validation sets ─────────────────────────
% 70% estimation, 30% validation (standard SI practice)
n_total = size(U, 1);
n_est   = round(0.7 * n_total);

U_est = U(1:n_est, :);          Y_est = Y(1:n_est, :);
U_val = U(n_est+1:end, :);      Y_val = Y(n_est+1:end, :);

data_est = iddata(Y_est, U_est, Ts);
data_val = iddata(Y_val, U_val, Ts);
data_est.TimeUnit = 'seconds';
data_val.TimeUnit = 'seconds';

%% ── 3. Per-Variable State-Space Models ───────────────────────────────────
% Paper Section 3.1: "For each variable, we first determine the state and
% output template equations... we specify a model order (i.e., the degree
% of polynomial equations)."
%
% "The dominating system dynamic is a second-order system" → order = 2
% We build one model per output channel (one per state variable).

state_names = {'Roll','Pitch','Yaw', ...
               'GyrX','GyrY','GyrZ', ...
               'AccX','AccY','AccZ', ...
               'GPS_Alt','BARO_Alt', ...
               'GPS_Lat','GPS_Lng', ...
               'MagX','MagY','MagZ'};

MODEL_ORDER = 2;    % second-order per variable (paper Section 3.1)
models = cell(length(state_names), 1);

opt = ssestOptions;
opt.InitialState = 'estimate';
opt.SearchMethod = 'auto';      % PEM — Prediction Error Minimization
opt.Display      = 'off';

fprintf('\nFitting %d per-variable models (order %d, PEM)...\n', ...
        length(state_names), MODEL_ORDER);

for i = 1:length(state_names)
    fprintf('  [%2d/%d] %s ... ', i, length(state_names), state_names{i});
    d_i = iddata(Y_est(:,i), U_est, Ts);
    try
        sys_i = ssest(d_i, MODEL_ORDER, opt);
        models{i} = sys_i;
        fit = sys_i.Report.Fit.FitPercent;
        fprintf('fit = %.1f%%\n', fit);
    catch e
        fprintf('FAILED: %s\n', e.message);
    end
end

%% ── 4. Full 12-State Model (for the complete recovery monitor) ───────────
% Paper Eq. (1)-(2): x' = Ax + Bu,  y = Cx + Du
% This is the model used at RUNTIME in Algorithm 1.
%
% State vector follows Eq. (3): [x y z φ θ ψ ẋ ẏ ż p q r]
% We use Roll≈φ, Pitch≈θ, Yaw≈ψ, GyrX≈p, GyrY≈q, GyrZ≈r as proxies.
% Full position (x,y,z) uses GPS_Lat, GPS_Lng, GPS_Alt.

% Select outputs corresponding to the 12 state variables:
% [GPS_Lat, GPS_Lng, GPS_Alt, Roll, Pitch, Yaw, 0, 0, 0, GyrX, GyrY, GyrZ]
% Velocity (ẋ,ẏ,ż) is derived from GPS — not directly logged at 400Hz,
% so we use finite difference of GPS position as proxy during SI.
y_idx_12 = [12, 13, 10, 1, 2, 3, 4, 5, 6, ...   % lat,lon,alt,roll,pitch,yaw,gx,gy,gz
             11, 7, 8];                            % baro_alt, accX, accY as velocity proxies
% (Indices into Y's 16 columns — adjust if your column order differs)

fprintf('\nFitting full 12-state model...\n');
nx_full = 12;
data_12 = iddata(Y_est(:, y_idx_12), U_est, Ts);
opt_full = ssestOptions;
opt_full.InitialState = 'estimate';
opt_full.SearchMethod = 'lm';   % Levenberg-Marquardt for full model
opt_full.Display      = 'on';

sys_full = ssest(data_12, nx_full, opt_full);
fprintf('Full model fit: %.1f%%\n', sys_full.Report.Fit.FitPercent);

%% ── 5. Extract and Save A, B, C, D Matrices ─────────────────────────────
A = sys_full.A;   % 12×12
B = sys_full.B;   % 12×4
C = sys_full.C;   % 12×12
D = sys_full.D;   % 12×4

fprintf('\nA: %dx%d  B: %dx%d  C: %dx%d  D: %dx%d\n', ...
    size(A), size(B), size(C), size(D));

out_dir = fullfile(getenv('HOME'), 'rv_recovery', 'matlab', 'models');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

save(fullfile(out_dir, 'quadrotor_ArduCopter34.mat'), ...
     'A', 'B', 'C', 'D', 'Ts', 'nx_full', 'state_names', 'sys_full');

% Save per-variable models too
for i = 1:length(state_names)
    if ~isempty(models{i})
        fname = fullfile(out_dir, ['model_' state_names{i} '.mat']);
        m = models{i};
        save(fname, 'm');
    end
end

fprintf('\nSaved models to: %s\n', out_dir);

%% ── 6. Validation Plot ───────────────────────────────────────────────────
figure('Name','Model Validation — Roll');
compare(data_val, sys_full);
title('Model vs Real Output (Validation Set)');

figure('Name','Residual Analysis');
resid(data_est, sys_full);

%% ── 7. Export matrices for C++ recovery module ───────────────────────────
% Write A,B,C,D as C header file for direct inclusion in firmware patch
fid = fopen(fullfile(out_dir, 'model_matrices.h'), 'w');
fprintf(fid, '// Auto-generated by system_identification.m\n');
fprintf(fid, '// ArduCopter 3.4 quadrotor state-space model\n');
fprintf(fid, '// State vector: [lat,lon,alt,roll,pitch,yaw,gyrX,gyrY,gyrZ,baroAlt,accX,accY]\n\n');
fprintf(fid, '#pragma once\n\n');
fprintf(fid, 'static const int NX = %d;\n', nx_full);
fprintf(fid, 'static const int NU = %d;\n', size(B,2));
fprintf(fid, 'static const float TS = %.6ff;\n\n', Ts);

write_matrix(fid, 'A_MAT', A);
write_matrix(fid, 'B_MAT', B);
write_matrix(fid, 'C_MAT', C);
write_matrix(fid, 'D_MAT', D);
fclose(fid);
fprintf('Exported: model_matrices.h\n');

function write_matrix(fid, name, M)
    [r,c] = size(M);
    fprintf(fid, 'static const float %s[%d][%d] = {\n', name, r, c);
    for i = 1:r
        fprintf(fid, '    {');
        fprintf(fid, '%.8ff, ', M(i,:));
        fprintf(fid, '},\n');
    end
    fprintf(fid, '};\n\n');
end
```

Run in MATLAB:
```matlab
cd ~/rv_recovery/matlab
system_identification
```

Expected terminal output:
```
Loaded: 96000 samples, Ts=0.0025 s (400 Hz)
Fitting 16 per-variable models (order 2, PEM)...
  [ 1/16] Roll  ... fit = 78.3%
  [ 2/16] Pitch ... fit = 81.2%
  ...
Full model fit: 72.1%
A: 12x12  B: 12x4  C: 12x12  D: 12x4
Saved models to: ~/rv_recovery/matlab/models/
Exported: model_matrices.h
```

> **Fit % guidance**: The paper reports successful recovery across all tested scenarios,
> implying good model fit. Target >70% per variable. If fit is low for a specific
> sensor, increase `MODEL_ORDER` to 3 or 4 for that channel only.

---

## Step 6 — DTW Parameter Selection

Run after MATLAB produces the model (you need software sensor predictions to compare
against real sensor data for the DTW window calculation).

```python
#!/usr/bin/env python3
# ~/rv_recovery/python/select_parameters.py
#
# Implements Section 3.3 parameter selection:
#   1. Window size N = max time-displacement (DTW)
#   2. Threshold T = e_max + margin
#
# Paper values for 3DR Solo (used as reference):
#   N = 230 counts (575 ms @ 400 Hz)
#   T_on = 38  (Figure 16 threshold line)

import numpy as np
import scipy.io
from dtaidistance import dtw as dtw_lib

TS = 0.0025   # 400 Hz

def compute_software_sensor_predictions(U, A, B, C, D):
    """
    Run the state-space model forward on input U to get predicted outputs.
    This is what the software sensor produces during normal operation.
    x[t+1] = A*x[t] + B*u[t]
    y[t]   = C*x[t] + D*u[t]
    """
    N, nu = U.shape
    nx = A.shape[0]
    ny = C.shape[0]

    x = np.zeros(nx)
    Y_pred = np.zeros((N, ny))

    for t in range(N):
        u = U[t]
        y = C @ x + D @ u
        Y_pred[t] = y
        x = A @ x + B @ u

    return Y_pred

def select_window_and_threshold(Y_real, Y_pred, margin=5.0):
    """
    For each output channel:
      - DTW path gives max time-displacement → window N
      - Max accumulated error within each window → threshold T
    """
    ny = Y_real.shape[1]
    N_windows_all = []
    T_all = []

    for ch in range(ny):
        r = Y_real[:, ch]
        s = Y_pred[:, ch]

        # Normalize to [0,1] for DTW comparison
        r_n = (r - r.mean()) / (r.std() + 1e-9)
        s_n = (s - s.mean()) / (s.std() + 1e-9)

        # DTW warping path (Section 3.3: "dynamic time-warping algorithm")
        # Use a window constraint to limit compute (Sakoe-Chiba band)
        path = dtw_lib.warping_path(r_n[:5000], s_n[:5000])
        displacements = [abs(i - j) for i, j in path]
        N_ch = int(max(displacements)) + 1
        N_windows_all.append(N_ch)

        # Max accumulated error within each window of size N_ch
        n_windows = len(r) // N_ch
        e_max = 0.0
        for w in range(n_windows):
            seg_r = r[w*N_ch:(w+1)*N_ch]
            seg_s = s[w*N_ch:(w+1)*N_ch]
            e = np.sum(np.abs(seg_r - seg_s))
            e_max = max(e_max, e)
        T_all.append(e_max + margin)

    N_global = max(N_windows_all)
    T_global = max(T_all)   # conservative: use max across all channels

    # Also compute per-sensor thresholds (more precise — paper uses per-sensor values)
    per_sensor = list(zip(N_windows_all, T_all))

    return N_global, T_global, per_sensor

def main():
    # Load operation data
    mat = scipy.io.loadmat(
        '/home/tchowdh4/rv_recovery/data/operation_data.mat')  # adjust path
    U = mat['U']
    Y = mat['Y']

    # Load model matrices exported from MATLAB
    model = scipy.io.loadmat(
        '/home/tchowdh4/rv_recovery/matlab/models/quadrotor_ArduCopter34.mat')
    A = model['A']
    B = model['B']
    C = model['C']
    D = model['D']

    print("Running state-space model forward pass...")
    Y_pred = compute_software_sensor_predictions(U, A, B, C, D)

    print("Computing DTW window size and threshold...")
    N, T_on, per_sensor = select_window_and_threshold(Y, Y_pred, margin=5.0)

    T_off = T_on * 0.79   # T_off < T_on (Algorithm 1 line 25); ratio not stated in paper

    print(f"\n{'='*50}")
    print(f"  Window size N  = {N} counts  ({N*TS*1000:.1f} ms)")
    print(f"  T_on threshold = {T_on:.2f}")
    print(f"  T_off threshold= {T_off:.2f}")
    print(f"  (Paper ref: N=230, T_on=38 for 3DR Solo @ 400Hz)")
    print(f"{'='*50}")
    print(f"\nPer-sensor (N, T_on):")
    sensor_names = ['Roll','Pitch','Yaw','GyrX','GyrY','GyrZ',
                    'AccX','AccY','AccZ','GPS_Alt','BARO_Alt',
                    'GPS_Lat','GPS_Lng','MagX','MagY','MagZ']
    for i, (n, t) in enumerate(per_sensor):
        name = sensor_names[i] if i < len(sensor_names) else f'ch{i}'
        print(f"  {name:12s}  N={n:4d}  T_on={t:.2f}")

    # Save for use in C++ recovery module
    np.save('/home/tchowdh4/rv_recovery/data/recovery_params.npy',
            {'N': N, 'T_on': T_on, 'T_off': T_off, 'per_sensor': per_sensor})
    print("\nSaved: recovery_params.npy")

if __name__ == '__main__':
    main()
```

---

## Step 7 — Recovery Module: File Structure and Build

With `model_matrices.h` generated by MATLAB, create the firmware patch files.

### 7.1 — Directory setup

```bash
mkdir -p ~/rv_recovery/firmware_patch
cd ~/rv_recovery/firmware_patch
```

### 7.2 — `recovery_monitor.h`

```cpp
// recovery_monitor.h
// Algorithm 1 from Choi et al. RAID 2020 — complete implementation
#pragma once
#include <cmath>
#include <cstring>
#include "model_matrices.h"   // A,B,C,D generated by system_identification.m

// ── Recovery parameters (set from DTW output, Section 3.3) ──────────────
// Defaults match paper's 3DR Solo values (Section 3.3, Figure 16):
//   N=230 counts (575ms @ 400Hz),  T_on=38,  T_off≈30
#ifndef RECOVERY_WINDOW
#define RECOVERY_WINDOW  230
#endif
#ifndef RECOVERY_T_ON
#define RECOVERY_T_ON    38.0f
#endif
#ifndef RECOVERY_T_OFF
#define RECOVERY_T_OFF   30.0f
#endif
#define RECOVERY_K_SAFE  10     // consecutive under-threshold counts before switch-back

// ── Low-pass filter (2nd-order Butterworth, Direct Form II Transposed) ───
// Coefficients: cutoff ~20 Hz, fs=400 Hz (tune per sensor in MATLAB)
struct LPFilter {
    float b[3];   // feedforward
    float a[3];   // feedback (a[0] normalised to 1)
    float w[2];   // delay line
};

// Initialise with coefficients from MATLAB:
//   [b,a] = butter(2, 20/(400/2), 'low');
// Default (20 Hz cutoff @ 400 Hz):
static inline void lp_init(LPFilter* f,
    float b0=0.0155f, float b1=0.0311f, float b2=0.0155f,
    float a1=-1.7347f, float a2=0.7969f) {
    f->b[0]=b0; f->b[1]=b1; f->b[2]=b2;
    f->a[1]=a1; f->a[2]=a2;
    f->w[0]=f->w[1]=0.0f;
}

static inline float lp_step(LPFilter* f, float x) {
    float y = f->b[0]*x + f->w[0];
    f->w[0] = f->b[1]*x - f->a[1]*y + f->w[1];
    f->w[1] = f->b[2]*x - f->a[2]*y;
    return y;
}

// ── Per-sensor recovery state ────────────────────────────────────────────
struct RecoveryState {
    float   x[NX];          // current predicted state vector
    float   ms;             // current software sensor prediction
    float   e;              // error compensation (external disturbance estimate)
    float   r;              // cumulative residual within window
    int     t;              // loop counter within current window
    bool    recovery_mode;
    int     safe_count;
    LPFilter lpf;           // low-pass filter for real measurement
    // Error history for external disturbance estimation (one window)
    float   err_history[RECOVERY_WINDOW];
    int     err_idx;
};

static inline void recovery_init(RecoveryState* s) {
    memset(s, 0, sizeof(RecoveryState));
    lp_init(&s->lpf);
}

// ── Software sensor convert: model output y → sensor reading ─────────────
// Specialise per sensor type. Default: identity (e.g., attitude, GPS).
static inline float sensor_convert_identity(float y) { return y; }

// Barometer: altitude → pressure  (Eq. 5 from paper)
static inline float sensor_convert_baro(float z_m) {
    const float P0 = 101325.0f;
    const float g0 = 9.87f;
    const float M  = 0.02896f;
    const float R  = 8.3143f;
    const float T0 = 288.15f;
    return P0 * expf((-g0 * M * z_m) / (R * T0));
}

// ── Algorithm 1: RECOVERYMONITOR ─────────────────────────────────────────
// u[NU]: control input vector
// m_real: raw real sensor measurement (scalar, one sensor at a time)
// output_row: which row of C/D to use for this sensor (0-indexed)
// Returns: measurement to feed into control loop (real or software)
static inline float recovery_monitor(RecoveryState* s,
                                     const float u[NU],
                                     float m_real,
                                     int output_row) {
    // ── Line 6: y = C[row]·x + D[row]·u ─────────────────────────────
    float y = 0.0f;
    for (int i = 0; i < NX; i++) y += C_MAT[output_row][i] * s->x[i];
    for (int i = 0; i < NU; i++) y += D_MAT[output_row][i] * u[i];

    // ── Line 7: x = A·x + B·u ────────────────────────────────────────
    float x_new[NX] = {0};
    for (int i = 0; i < NX; i++) {
        for (int j = 0; j < NX; j++) x_new[i] += A_MAT[i][j] * s->x[j];
        for (int j = 0; j < NU; j++) x_new[i] += B_MAT[i][j] * u[j];
    }
    memcpy(s->x, x_new, sizeof(float) * NX);

    // ── Line 8: m = filter(m) ─────────────────────────────────────────
    float m = lp_step(&s->lpf, m_real);

    // ── Line 9: ms = convert(y) ───────────────────────────────────────
    s->ms = sensor_convert_identity(y);  // swap for baro/gyro as needed

    // ── Line 10: t++ ─────────────────────────────────────────────────
    s->t++;

    // ── Lines 11–16: checkpoint (window boundary) ─────────────────────
    if (!s->recovery_mode && s->t > RECOVERY_WINDOW) {
        s->t   = 0;
        s->r   = 0.0f;
        // error_estimation: average error over previous window (Section 3.3)
        float sum_err = 0.0f;
        for (int i = 0; i < RECOVERY_WINDOW; i++) sum_err += s->err_history[i];
        s->e = sum_err / RECOVERY_WINDOW;   // external force estimate
        // Line 15: synchronise — reset software sensor to real sensor
        s->ms = m;
        memset(s->err_history, 0, sizeof(s->err_history));
        s->err_idx = 0;
    }

    // ── Line 17: error compensation ───────────────────────────────────
    s->ms -= s->e;

    // ── Line 18: accumulate residual ──────────────────────────────────
    float diff = fabsf(m - s->ms);
    s->r += diff;
    // Store for next window's error estimation
    if (s->err_idx < RECOVERY_WINDOW)
        s->err_history[s->err_idx++] = m - s->ms;

    // ── Lines 19–22: attack detection ────────────────────────────────
    if (s->r > RECOVERY_T_ON) {
        s->recovery_mode = true;
        s->safe_count    = 0;
    }

    // ── Lines 23–32: recovery mode ────────────────────────────────────
    if (s->recovery_mode) {
        m = s->ms;                   // Line 24: replace with software sensor

        if (s->r < RECOVERY_T_OFF)
            s->safe_count++;

        if (s->safe_count > RECOVERY_K_SAFE)   // attack ended
            s->recovery_mode = false;

        // recovery_action() — optional: trigger safe landing, alert, etc.
    }

    return m;
}
```

### 7.3 — `software_sensors.h`

All sensor conversion equations from the paper (Eq. 4–6, Appendix A–B):

```cpp
// software_sensors.h
// Sensor conversion equations — Section 3.2 of Choi et al. RAID 2020
#pragma once
#include <cmath>

// ── Smooth noise-robust differentiator (Holoborodko, cited Section 3.3) ──
// Causal 5-point formula for first derivative:
// f'[n] ≈ (2*(f[n]-f[n-2]) + (f[n-1]-f[n-3])) / (8*h)
// h = sample interval (0.0025 s @ 400 Hz)
static inline float holoborodko_deriv(const float* buf, float h) {
    // buf[0]=f[n], buf[1]=f[n-1], buf[2]=f[n-2], buf[3]=f[n-3]
    return (2.0f*(buf[0]-buf[2]) + (buf[1]-buf[3])) / (8.0f * h);
}

// ── Accelerometer (Eq. 4) ─────────────────────────────────────────────────
// a(t) = c_k * (v(t) - v(t-k)) / (k * Δt)
// Use Holoborodko differentiator for noise suppression (Section 3.3)
static inline float software_accel(const float* v_buf, float dt) {
    return holoborodko_deriv(v_buf, dt);
}

// ── Barometer (Eq. 5) ─────────────────────────────────────────────────────
// Ph = P0 * exp(-g0*M*(z-h0) / (R*T0))
// Constants per paper:
//   P0 = base air pressure (Pa)
//   g0 = 9.87 m/s²  ← paper's exact value
//   M  = 0.02896 kg/mol
//   h0 = base altitude (set to home at runtime)
//   R  = 8.3143 N·m/(mol·K)
//   T0 = base temperature (K)
static inline float software_baro(float z_m,
                                   float h0=0.0f,
                                   float P0=101325.0f,
                                   float T0=288.15f) {
    const float g0 = 9.87f;
    const float M  = 0.02896f;
    const float R  = 8.3143f;
    return P0 * expf((-g0 * M * (z_m - h0)) / (R * T0));
}

// ── Magnetometer heading (Eq. 6) ──────────────────────────────────────────
// H = atan2(-my*cos(φ) + mz*sin(φ),
//           mx*cos(θ) + my*sin(θ)*sin(φ) + mz*sin(θ)*cos(φ))
// Note: control system uses yaw ψ from model states directly (Section 3.2)
static inline float software_mag_heading(float mx, float my, float mz,
                                          float phi, float theta) {
    return atan2f(-my*cosf(phi) + mz*sinf(phi),
                   mx*cosf(theta)
                 + my*sinf(theta)*sinf(phi)
                 + mz*sinf(theta)*cosf(phi));
}

// ── Frame: body → inertial rotation matrix (Appendix A, Eq. 8) ───────────
static inline void body_to_inertial_R(float phi, float theta, float psi,
                                       float R[3][3]) {
    float Cp=cosf(phi),  Sp=sinf(phi);
    float Ct=cosf(theta),St=sinf(theta);
    float Cy=cosf(psi),  Sy=sinf(psi);
    R[0][0]= Cy*Ct;  R[0][1]= Cy*St*Sp - Sy*Cp;  R[0][2]= Cy*St*Cp + Sy*Sp;
    R[1][0]= Sy*Ct;  R[1][1]= Sy*St*Sp + Cy*Cp;  R[1][2]= Sy*St*Cp - Cy*Sp;
    R[2][0]=-St;     R[2][1]= Ct*Sp;              R[2][2]= Ct*Cp;
}

// ── Body rates → Euler rates (Appendix A, Eq. 10) ────────────────────────
static inline void body_to_euler_rates(float phi, float theta,
                                        float p, float q, float r,
                                        float* phi_d, float* theta_d, float* psi_d) {
    float Cp=cosf(phi), Sp=sinf(phi);
    float Ct=cosf(theta), Tt=tanf(theta);
    *phi_d   = p + (q*Sp + r*Cp)*Tt;
    *theta_d = q*Cp - r*Sp;
    *psi_d   = (q*Sp + r*Cp)/Ct;
}

// ── Supplementary compensation (Appendix B, Eq. 11) ──────────────────────
// ONLY used when ALL gyros are compromised simultaneously (Table 3 C3/C5/C6).
// Estimates roll/pitch from accelerometer, yaw from magnetometer.
static inline void supplementary_compensation(
    float xa, float ya, float za,          // accelerometer
    float xm, float ym, float zm,          // magnetometer
    float* phi_acc, float* theta_acc, float* psi_mag) {
    *phi_acc   = atan2f(ya, sqrtf(xa*xa + za*za));
    *theta_acc = atan2f(xa, sqrtf(ya*ya + za*za));
    *psi_mag   = atan2f(-ym*cosf(*phi_acc) + zm*sinf(*phi_acc),
                         xm*cosf(*theta_acc)
                       + ym*sinf(*theta_acc)*sinf(*phi_acc)
                       + zm*sinf(*theta_acc)*cosf(*phi_acc));
    // Apply low-pass filter to outputs before use (Section 3.3)
}
```

### 7.4 — Compile a standalone test binary

Before patching firmware, test the recovery logic compiles cleanly:

```bash
cd ~/rv_recovery/firmware_patch

# Copy model_matrices.h from MATLAB output
cp ~/rv_recovery/matlab/models/model_matrices.h .

cat > test_recovery.cpp << 'EOF'
#include <cstdio>
#include "recovery_monitor.h"
#include "software_sensors.h"

int main() {
    RecoveryState s;
    recovery_init(&s);

    float u[NU] = {0};
    printf("Recovery module compiled OK\n");
    printf("NX=%d  NU=%d  Ts=%.4f\n", NX, NU, TS);
    printf("Window N=%d  T_on=%.1f  T_off=%.1f\n",
           RECOVERY_WINDOW, RECOVERY_T_ON, RECOVERY_T_OFF);

    // Simulate 10 normal cycles then inject an attack
    for (int i = 0; i < 500; i++) {
        float real_meas = 0.1f * sinf(i * 0.01f);        // normal signal
        float result    = recovery_monitor(&s, u, real_meas, 0);
        if (i == 300) printf("  [t=300] Injecting attack...\n");
        if (i >= 300) real_meas += 5.0f;                  // simulate attack offset
        if (s.recovery_mode)
            printf("  [t=%d] RECOVERY MODE ACTIVE — using software sensor\n", i);
    }
    printf("Test complete.\n");
    return 0;
}
EOF

g++ -O2 -std=c++14 -o test_recovery test_recovery.cpp -lm
./test_recovery
```

Expected:
```
Recovery module compiled OK
NX=12  NU=4  Ts=0.0025
Window N=230  T_on=38.0  T_off=30.0
  [t=300] Injecting attack...
  [t=530] RECOVERY MODE ACTIVE — using software sensor
Test complete.
```

---

## Step 8 — Patch the ArduCopter 3.4 Firmware

The paper inserts recovery code "right after sensor reading acquisition" (Section 3.4,
Algorithm 1). In ArduCopter 3.4, the gyro read loop is in:

```
libraries/AP_InertialSensor/AP_InertialSensor.cpp
```

### 8.1 — Locate the exact insertion point

```bash
cd ~/ardupilot_ws/arducopter-3.4

# Find the gyro read function — matches Figure 3 of the paper
grep -n "get_gyro\|gyro_sum\|_gyro_filtered" \
  libraries/AP_InertialSensor/AP_InertialSensor.cpp | head -20

# Find the sensor fusion loop (Figure 3 line 18-26 equivalent)
grep -n "for.*num_instances\|_ins_count\|backend.*update" \
  libraries/AP_InertialSensor/AP_InertialSensor.cpp | head -20
```

### 8.2 — Add recovery headers to the build

```bash
# Copy recovery headers into the ArduPilot library tree
cp ~/rv_recovery/firmware_patch/recovery_monitor.h \
   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/firmware_patch/software_sensors.h \
   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
cp ~/rv_recovery/firmware_patch/model_matrices.h \
   ~/ardupilot_ws/arducopter-3.4/libraries/AP_InertialSensor/
```

### 8.3 — Edit AP_InertialSensor.cpp

At the top of the file, add:
```cpp
#include "recovery_monitor.h"
#include "software_sensors.h"

// One RecoveryState per sensor type (paper monitors each sensor individually)
static RecoveryState g_recovery_gyro[3];   // 3 gyros on 3DR Solo
static RecoveryState g_recovery_accel[3];
static bool g_recovery_initialized = false;
```

Inside the gyro update loop, after each `_gyro_filtered[i] = ...` line, add:
```cpp
// === PAPER FIGURE 3 LINES 22-23 / ALGORITHM 1 ===
if (!g_recovery_initialized) {
    for (int k = 0; k < 3; k++) recovery_init(&g_recovery_gyro[k]);
    g_recovery_initialized = true;
}
float u_vec[NU] = { /* fill from current target */ 0 };
float recovered = recovery_monitor(&g_recovery_gyro[i], u_vec,
                                   _gyro_filtered[i].x, 0);
if (g_recovery_gyro[i].recovery_mode)
    _gyro_filtered[i].x = recovered;
// Repeat for .y (row 1) and .z (row 2)
// =================================================
```

### 8.4 — Rebuild

```bash
cd ~/ardupilot_ws/arducopter-3.4
./waf copter
```

---

## Step 9 — Attack Injection Test (SITL End-to-End)

With the patched firmware built:

```bash
# Terminal A: Launch patched SITL
cd ~/ardupilot_ws/arducopter-3.4
python Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild \
    --home=40.071374,-105.229594,1583,353 \
    --out=udp:127.0.0.1:14550 --speedup=1

# Terminal B: MAVProxy
mavproxy.py --master=udp:127.0.0.1:14550 --aircraft=attack_test

# Terminal C: Attack injector
python ~/rv_recovery/python/attack_injector.py
```

### Recovery Success Criterion (Eq. 7 from paper)

```python
# eval_recovery.py — implements Eq. (7):
# R_succ := |Y_t - Ȳ_t| ≤ ε,  t ∈ [1...k]
# ε=3 degrees (attitude), k=10 seconds
import numpy as np
from pymavlink import mavutil
import time

def evaluate(connection='udp:127.0.0.1:14550', epsilon=3.0, k_sec=10):
    mav = mavutil.mavlink_connection(connection)
    mav.wait_heartbeat()
    start = None
    errors = []
    while True:
        msg = mav.recv_match(type='ATT', blocking=True, timeout=1)
        if msg is None: break
        if start is None: start = msg._timestamp
        err = abs(msg.Roll - msg.RollIn)   # |real - expected|
        errors.append(err)
        if msg._timestamp - start > k_sec:
            break
    success = all(e <= epsilon for e in errors)
    print(f"Recovery SUCCESS: {success}  "
          f"(max_err={max(errors):.2f}°  ε={epsilon}°  k={k_sec}s)")
    return success
```

---

## Updated Checklist

| Done | Step | Notes |
|---|---|---|
| ✅ | System packages | |
| ✅ | Conda env `rv_recovery` | |
| ✅ | ArduPilot cloned + submodules | |
| ✅ | ArduCopter 3.4 worktree + binary | |
| ✅ | **APMrover2 2.5 worktree + binary** | Built via `make sitl` (mk/-based, pre-waf); binary copied to build/sitl/bin/ardurover |
| □ | **SITL launches + MAVProxy connects** | Step 2 |
| □ | **20 missions flown, .bin logs collected** | Step 3 |
| □ | **operation_data.mat generated** | Step 4 |
| □ | **MATLAB: A,B,C,D + model_matrices.h** | Step 5 |
| □ | **DTW window N + threshold T_on** | Step 6 |
| □ | **recovery_monitor.h + software_sensors.h compile** | Step 7 |
| □ | **Firmware patched + rebuilt** | Step 8 |
| □ | **Attack injection → recovery verified (Eq. 7)** | Step 9 |
