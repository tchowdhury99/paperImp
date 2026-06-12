#!/usr/bin/env python3
"""
Parse ArduPilot Dataflash .BIN logs and resample all sensor streams to ONE target
frequency using spline interpolation — paper §3.1 ("we convert various data streams
to the same target frequency using a resampling technique ... we use spline
interpolation, to avoid Runge's phenomenon").

Paper target: 400 Hz. Ours: 50 Hz (the one sanctioned deviation — RAM limit).

Output: operation_data_50hz.mat with PER-SEGMENT cell arrays (each contiguous flight
segment is one identification experiment — no splicing across gaps), raw units,
NO detrending.

Per-segment matrices:
  U [N x 7]:  [phi_cmd, theta_cmd, psi_cmd (rad, unwrapped), thr (0..1),
               tiltN, tiltE (rad, frame-canonicalized commands, Appendix A),
               const=1 (gravity/hover equilibrium term of the template)]
  Y [N x 12]: Eq. (3) state variables
              [pN, pE, alt, phi, theta, psi(unwrapped, rad),
               vN, vE, vUp, p, q, r]   (m, rad, m/s, rad/s)
  EXTRA [N x 9]: sensor streams for software-sensor calibration/eval
              [BARO_Press(Pa), BARO_Alt(m), MagX, MagY, MagZ (mG),
               GPS_Lat(deg), GPS_Lng(deg), GPS_Alt(m), GPS_Spd(m/s)]

tiltN/tiltE: the closed-loop translational dynamics are LTI only in a consistent
frame; the attitude commands (body frame) are rotated to the inertial N/E frame with
the yaw rotation (paper Appendix A frame canonicalization, Eq. 8):
  tiltN = (-theta_cmd)*cos(psi) - (phi_cmd)*sin(psi)
  tiltE = (-theta_cmd)*sin(psi) + (phi_cmd)*cos(psi)
The same conversion is applied at runtime by the firmware monitor.
"""
import os, sys, glob
import numpy as np
import scipy.io
from scipy.interpolate import CubicSpline
from pymavlink import DFReader

TARGET_HZ = 50.0            # sanctioned deviation: 50 Hz instead of paper's 400 Hz
TS        = 1.0 / TARGET_HZ # 0.02 s

D2R = np.pi / 180.0

# Dataflash streams and fields (ArduCopter 3.4 schema, verified against the log)
STREAMS = {
    'IMU':  ['GyrX', 'GyrY', 'GyrZ', 'AccX', 'AccY', 'AccZ'],       # rad/s, m/s^2
    'ATT':  ['Roll', 'Pitch', 'Yaw', 'DesRoll', 'DesPitch', 'DesYaw'],  # deg
    'CTUN': ['ThI', 'ThO', 'ThH'],                                   # 0..1
    'NKF1': ['VN', 'VE', 'VD', 'PN', 'PE', 'PD'],                    # m/s, m (NED)
    'BARO': ['Press', 'Alt'],                                        # Pa, m
    'MAG':  ['MagX', 'MagY', 'MagZ'],                                # milliGauss
    'GPS':  ['Lat', 'Lng', 'Alt', 'Spd'],                            # deg, m, m/s
}

# In-flight segmentation on raw CTUN.ThO samples
THR_FLYING   = 0.05   # throttle-out above this = motors driving
GAP_MERGE_S  = 5.0    # merge gaps shorter than this
EDGE_TRIM_S  = 2.0    # trim segment edges (takeoff/landing transients)
MIN_SEG_S    = 60.0   # discard segments shorter than this


def parse_single_log(bin_path):
    log = DFReader.DFReader_binary(bin_path, zero_time_base=True)
    raw = {k: {'t': [], **{f: [] for f in v}} for k, v in STREAMS.items()}
    while True:
        msg = log.recv_match()
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype not in STREAMS:
            continue
        raw[mtype]['t'].append(msg._timestamp)
        for field in STREAMS[mtype]:
            try:
                raw[mtype][field].append(float(getattr(msg, field)))
            except AttributeError:
                raw[mtype][field].append(np.nan)
    for k in raw:
        for f in raw[k]:
            raw[k][f] = np.asarray(raw[k][f])
    return raw


def flight_segments(raw):
    """Contiguous in-flight intervals from raw CTUN.ThO (hysteresis + merge + trim)."""
    t   = raw['CTUN']['t']
    tho = raw['CTUN']['ThO']
    flying = tho > THR_FLYING
    if not flying.any():
        return []
    # raw intervals where flying
    idx = np.flatnonzero(np.diff(np.concatenate(([0], flying.view(np.int8), [0]))))
    starts, ends = idx[0::2], idx[1::2] - 1
    ivals = [(t[s], t[e]) for s, e in zip(starts, ends)]
    # merge short gaps
    merged = [list(ivals[0])]
    for s, e in ivals[1:]:
        if s - merged[-1][1] < GAP_MERGE_S:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    # trim edges, enforce min length
    out = []
    for s, e in merged:
        s += EDGE_TRIM_S
        e -= EDGE_TRIM_S
        if e - s >= MIN_SEG_S:
            out.append((s, e))
    return out


def resample_segment(raw, t0, t1):
    """Spline-resample every stream/field onto the common 50 Hz grid of [t0, t1].
    Returns dict {'STREAM_Field': array} or None if any stream lacks coverage."""
    t_common = np.arange(t0, t1, TS)
    out = {'t': t_common}
    for stream, data in raw.items():
        t_orig = data['t']
        m = (t_orig >= t0 - 1.0) & (t_orig <= t1 + 1.0)
        if m.sum() < 4:
            return None
        ts = t_orig[m]
        # guard duplicate timestamps (spline needs strictly increasing x)
        keep = np.concatenate(([True], np.diff(ts) > 0))
        ts = ts[keep]
        if ts[0] > t0 or ts[-1] < t1 - TS:
            return None                      # stream doesn't cover the segment
        for field in STREAMS[stream]:
            y = data[field][m][keep]
            if not np.all(np.isfinite(y)):
                return None
            if stream == 'ATT' and field in ('Yaw', 'DesYaw'):
                y = np.unwrap(y * D2R) / D2R  # unwrap BEFORE splining (deg kept)
            cs = CubicSpline(ts, y)
            out[f'{stream}_{field}'] = cs(t_common)
    # align DesYaw unwrap branch with Yaw at segment start
    dy = out['ATT_DesYaw'] * D2R
    yw = out['ATT_Yaw'] * D2R
    k = np.round((dy[0] - yw[0]) / (2 * np.pi))
    out['ATT_DesYaw'] = (dy - 2 * np.pi * k) / D2R
    return out


def build_matrices(r):
    """Per-segment U [N x 7], Y [N x 12] (Eq. 3 order), EXTRA [N x 9]."""
    phi_c   = r['ATT_DesRoll']  * D2R
    theta_c = r['ATT_DesPitch'] * D2R
    psi_c   = r['ATT_DesYaw']   * D2R
    thr     = r['CTUN_ThI']
    psi     = r['ATT_Yaw'] * D2R          # measured yaw for frame canonicalization
    tiltN   = (-theta_c) * np.cos(psi) - phi_c * np.sin(psi)
    tiltE   = (-theta_c) * np.sin(psi) + phi_c * np.cos(psi)
    ones    = np.ones_like(thr)
    U = np.column_stack([phi_c, theta_c, psi_c, thr, tiltN, tiltE, ones])

    Y = np.column_stack([
        r['NKF1_PN'], r['NKF1_PE'], -r['NKF1_PD'],            # pN, pE, alt(up)
        r['ATT_Roll'] * D2R, r['ATT_Pitch'] * D2R, psi,        # phi, theta, psi
        r['NKF1_VN'], r['NKF1_VE'], -r['NKF1_VD'],             # vN, vE, vUp
        r['IMU_GyrX'], r['IMU_GyrY'], r['IMU_GyrZ'],           # p, q, r
    ])

    EXTRA = np.column_stack([
        r['BARO_Press'], r['BARO_Alt'],
        r['MAG_MagX'], r['MAG_MagY'], r['MAG_MagZ'],
        r['GPS_Lat'], r['GPS_Lng'], r['GPS_Alt'], r['GPS_Spd'],
    ])
    return U, Y, EXTRA


def process_logs(log_dir, out_mat):
    bin_files = sorted(glob.glob(os.path.join(log_dir, '*.BIN')) +
                       glob.glob(os.path.join(log_dir, '*.bin')))
    if not bin_files:
        print(f"ERROR: no .BIN files in {log_dir}")
        sys.exit(1)
    print(f"Found {len(bin_files)} log file(s).")

    Us, Ys, Es, lens = [], [], [], []
    for bf in bin_files:
        print(f"Parsing {os.path.basename(bf)} ...")
        raw = parse_single_log(bf)
        segs = flight_segments(raw)
        print(f"  {len(segs)} flight segment(s)")
        for (s, e) in segs:
            r = resample_segment(raw, s, e)
            if r is None:
                print(f"    [{s:9.1f},{e:9.1f}] skipped (stream coverage)")
                continue
            U, Y, EX = build_matrices(r)
            Us.append(U); Ys.append(Y); Es.append(EX); lens.append(len(U))
            print(f"    [{s:9.1f},{e:9.1f}] {len(U)} samples ({len(U)*TS:.0f} s)")

    if not Us:
        print("ERROR: no usable segments.")
        sys.exit(1)

    total = sum(lens)
    print(f"\nSegments: {len(Us)}  |  total {total} samples = {total*TS/60:.1f} min @ {TARGET_HZ:.0f} Hz")

    cellU = np.empty(len(Us), dtype=object); cellU[:] = Us
    cellY = np.empty(len(Ys), dtype=object); cellY[:] = Ys
    cellE = np.empty(len(Es), dtype=object); cellE[:] = Es
    scipy.io.savemat(out_mat, {
        'Useg': cellU, 'Yseg': cellY, 'EXTRAseg': cellE,
        'Ts': TS, 'fs': TARGET_HZ,
        'u_labels': ['phi_cmd','theta_cmd','psi_cmd','thr','tiltN','tiltE','const'],
        'y_labels': ['pN','pE','alt','phi','theta','psi','vN','vE','vUp','p','q','r'],
        'extra_labels': ['BARO_Press','BARO_Alt','MagX','MagY','MagZ',
                         'GPS_Lat','GPS_Lng','GPS_Alt','GPS_Spd'],
    }, do_compression=True)
    print(f"Saved: {out_mat}")


if __name__ == '__main__':
    import sys
    default_dir = '~/paperImp/rv_recovery/data/sitl_run1/logs/'
    LOG_DIR = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else default_dir)
    OUT_MAT = os.path.expanduser('~/paperImp/rv_recovery/data/operation_data_50hz.mat')
    process_logs(LOG_DIR, OUT_MAT)
