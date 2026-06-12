#!/usr/bin/env python3
"""
Figures derived ONLY from the recorded operation data:
    rv_recovery/data/operation_data_50hz.mat

These characterize the §3.1 operation data used for system identification — the inputs,
states, extra sensor streams, flight paths, and coverage. No model, thresholds, or monitor
are involved. READ-ONLY: nothing is modified.

Outputs -> ~/paperImp/rv_recovery/figures/
  figD1_flight_paths.png        top-down flight paths (pN vs pE) for several missions
  figD2_input_commands.png      control inputs u over one mission
  figD3_state_coverage_hist.png distribution of each state across ALL segments
  figD4_extra_sensors.png       barometer / magnetometer / GPS streams over one mission
  figD5_command_tracking.png    attitude command vs response (roll/pitch/yaw)
  figD6_segment_durations.png   duration of each recorded flight segment
"""
import os
import numpy as np
import scipy.io
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HOME = os.path.expanduser('~')
DATA = f'{HOME}/paperImp/rv_recovery/data/operation_data_50hz.mat'
OUT = f'{HOME}/paperImp/rv_recovery/figures'
os.makedirs(OUT, exist_ok=True)

TS = 0.02
R2D = 180/np.pi
YL = ['pN','pE','alt','phi','theta','psi','vN','vE','vUp','p','q','r']
UL = ['phi_cmd','theta_cmd','psi_cmd','thr','tiltN','tiltE','const']
EL = ['BARO_Press','BARO_Alt','MagX','MagY','MagZ','GPS_Lat','GPS_Lng','GPS_Alt','GPS_Spd']


def main():
    S = scipy.io.loadmat(DATA)
    U = [np.asarray(u, float) for u in S['Useg'].ravel()]
    Y = [np.asarray(y, float) for y in S['Yseg'].ravel()]
    E = [np.asarray(x, float) for x in S['EXTRAseg'].ravel()]
    nseg = len(Y)
    print(f'{nseg} segments loaded from operation_data_50hz.mat')

    # ── figD1: top-down flight paths (pN vs pE) for several missions ─────────
    fig, ax = plt.subplots(figsize=(8, 8))
    for i in range(min(nseg, 8)):
        yi = Y[i]
        ax.plot(yi[:, 1], yi[:, 0], lw=1.0, alpha=0.8, label=f'seg {i+1}')
        ax.plot(yi[0, 1], yi[0, 0], 'go', ms=5)   # start
        ax.plot(yi[-1, 1], yi[-1, 0], 'rs', ms=5)  # end
    ax.set_xlabel('East position pE (m)'); ax.set_ylabel('North position pN (m)')
    ax.set_title('Recorded flight paths — top-down (operation data)')
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3); ax.set_aspect('equal', 'box')
    fig.tight_layout(); fig.savefig(f'{OUT}/figD1_flight_paths.png', dpi=120)
    print('wrote figD1_flight_paths.png')

    # ── figD2: control inputs u over one mission ─────────────────────────────
    seg = 5; u = U[seg]; n = len(u); t = np.arange(n)*TS
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, u[:, 0]*R2D, label='roll cmd'); ax[0].plot(t, u[:, 1]*R2D, label='pitch cmd')
    ax[0].set_ylabel('attitude cmd (deg)'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].plot(t, u[:, 2]*R2D, 'g', label='yaw cmd (deg)')
    ax[1].set_ylabel('yaw cmd (deg)'); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    ax[2].plot(t, u[:, 3], 'm', label='throttle (0–1)')
    ax[2].set_ylabel('throttle'); ax[2].set_xlabel('time (s)')
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)
    ax[0].set_title('Control inputs u over one mission (target states)')
    fig.tight_layout(); fig.savefig(f'{OUT}/figD2_input_commands.png', dpi=120)
    print('wrote figD2_input_commands.png')

    # ── figD3: distribution of each state across ALL segments ────────────────
    allY = np.vstack(Y)
    fig, axes = plt.subplots(3, 4, figsize=(14, 8))
    units = ['m','m','m','rad','rad','rad','m/s','m/s','m/s','rad/s','rad/s','rad/s']
    for k, axk in enumerate(axes.ravel()):
        axk.hist(allY[:, k], bins=60, color='steelblue', alpha=0.85)
        axk.set_title(f'{YL[k]} ({units[k]})', fontsize=10); axk.grid(alpha=0.3)
    fig.suptitle('State-variable coverage across all recorded segments '
                 '(excitation envelope for system ID)')
    fig.tight_layout(); fig.savefig(f'{OUT}/figD3_state_coverage_hist.png', dpi=120)
    print('wrote figD3_state_coverage_hist.png')

    # ── figD4: extra sensor streams over one mission ─────────────────────────
    e = E[seg]; n = len(e); t = np.arange(n)*TS
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, e[:, 0], 'r'); ax[0].set_ylabel('baro pressure (Pa)'); ax[0].grid(alpha=0.3)
    ax[0].set_title('Extra sensor streams over one mission (barometer / magnetometer / GPS)')
    ax[1].plot(t, e[:, 2], label='MagX'); ax[1].plot(t, e[:, 3], label='MagY')
    ax[1].plot(t, e[:, 4], label='MagZ'); ax[1].set_ylabel('magnetometer (mG)')
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    lat0, lng0 = e[0, 5], e[0, 6]
    ax[2].plot(t, (e[:, 5]-lat0)*111194.9, label='GPS North (m)')
    ax[2].plot(t, (e[:, 6]-lng0)*111194.9*np.cos(np.radians(lat0)), label='GPS East (m)')
    ax[2].set_ylabel('GPS local (m)'); ax[2].set_xlabel('time (s)')
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f'{OUT}/figD4_extra_sensors.png', dpi=120)
    print('wrote figD4_extra_sensors.png')

    # ── figD5: attitude command vs response (closed-loop tracking) ───────────
    u = U[seg]; y = Y[seg]; n = len(y); t = np.arange(n)*TS
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for axk, ci_u, ci_y, name in [(ax[0], 0, 3, 'roll'), (ax[1], 1, 4, 'pitch'),
                                   (ax[2], 2, 5, 'yaw')]:
        axk.plot(t, u[:, ci_u]*R2D, 'r--', lw=0.9, label=f'{name} command')
        axk.plot(t, y[:, ci_y]*R2D, 'b-', lw=0.9, label=f'{name} response')
        axk.set_ylabel(f'{name} (deg)'); axk.legend(fontsize=8); axk.grid(alpha=0.3)
    ax[2].set_xlabel('time (s)')
    ax[0].set_title('Attitude command vs. response (closed-loop tracking in the data)')
    fig.tight_layout(); fig.savefig(f'{OUT}/figD5_command_tracking.png', dpi=120)
    print('wrote figD5_command_tracking.png')

    # ── figD6: duration of each recorded flight segment ──────────────────────
    durs = [len(y)*TS for y in Y]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.bar(np.arange(1, nseg+1), durs, color='teal')
    ax.set_xlabel('segment #'); ax.set_ylabel('duration (s)')
    ax.set_title(f'Recorded flight-segment durations '
                 f'(total {sum(durs)/60:.1f} min over {nseg} segments)')
    ax.grid(alpha=0.3, axis='y')
    fig.tight_layout(); fig.savefig(f'{OUT}/figD6_segment_durations.png', dpi=120)
    print('wrote figD6_segment_durations.png')

    print(f'\nAll data figures in: {OUT}')


if __name__ == '__main__':
    main()
