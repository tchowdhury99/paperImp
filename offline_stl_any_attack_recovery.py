#!/usr/bin/env python3
"""
Offline STL monitor: Multi-Sensor Any-Attack Recovery

Interpreter:
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Dataset:
  /home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat

Formula:
  G[0:580ms] (
      ((baro_residual > 0.30) or (gyro_residual_x > 0.15))
      ->
      F[0:10000ms] (not ((baro_residual > 0.30) or (gyro_residual_x > 0.15)))
  )
"""

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import rtamt


DATASET_PATH = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_any_attack_recovery.png"

SEG_IDX = 0

TS_EXPECTED = 0.02
FS_EXPECTED = 50

ATTACK_START = 2000
ATTACK_END = 2500

BARO_ATTACK_OFFSET_M = 3.0
GYRX_ATTACK_VALUE_RAD_S = 0.8

EPS_BARO = 0.30
EPS_GYR = 0.15

OUTER_T_MS = 580
RECOVERY_WINDOW_MS = 10000

STL_FORMULA = (
    "G[0:580ms] "
    "(((baro_residual > 0.30) or (gyro_residual_x > 0.15)) "
    "-> "
    "F[0:10000ms] "
    "(not ((baro_residual > 0.30) or (gyro_residual_x > 0.15))))"
)


def make_stl_spec():
    """
    Create RTAMT discrete-time STL spec using the smallest compatibility fallback:
      1. Try rtamt.STLDiscreteTimeSpecification
      2. Fall back to rtamt.StlDiscreteTimeSpecification
    """
    if hasattr(rtamt, "STLDiscreteTimeSpecification"):
        spec = rtamt.STLDiscreteTimeSpecification()
    else:
        spec = rtamt.StlDiscreteTimeSpecification()

    spec.declare_var("baro_residual", "float")
    spec.declare_var("gyro_residual_x", "float")

    # Compatibility rule already used in completed STL steps.
    try:
        spec.set_sampling_period(20, "ms", 0.1)
    except TypeError:
        spec.set_sampling_period(20, "ms")

    spec.spec = STL_FORMULA
    spec.parse()
    return spec


def normalize_rtamt_trace(result):
    """
    Convert RTAMT offline output to two arrays:
      times_ms, rho_values

    Handles common RTAMT output shapes:
      [(time, rho), ...]
      [[time, rho], ...]
      [rho0, rho1, ...]
    """
    if isinstance(result, (float, int, np.floating, np.integer)):
        return np.array([0]), np.array([float(result)])

    result_list = list(result)

    if len(result_list) == 0:
        raise RuntimeError("RTAMT returned an empty robustness trace.")

    first = result_list[0]

    if isinstance(first, (tuple, list, np.ndarray)) and len(first) >= 2:
        times_ms = np.array([float(x[0]) for x in result_list], dtype=float)
        rho = np.array([float(x[1]) for x in result_list], dtype=float)
        return times_ms, rho

    rho = np.array(result_list, dtype=float)
    times_ms = np.arange(len(rho), dtype=float) * 20.0
    return times_ms, rho


def evaluate_offline(baro_residual, gyro_residual_x, t_ms):
    """
    Prefer offline spec.evaluate().

    First try dataset with a separate 'time' key.
    If this installed RTAMT version expects time-value pairs instead,
    use the smallest dataset-format fallback only.
    """
    spec = make_stl_spec()

    dataset_with_time_key = {
        "time": t_ms.tolist(),
        "baro_residual": baro_residual.astype(float).tolist(),
        "gyro_residual_x": gyro_residual_x.astype(float).tolist(),
    }

    try:
        result = spec.evaluate(dataset_with_time_key)
        return normalize_rtamt_trace(result)
    except Exception as first_error:
        spec = make_stl_spec()

        dataset_pairs = {
            "baro_residual": list(zip(t_ms.tolist(), baro_residual.astype(float).tolist())),
            "gyro_residual_x": list(zip(t_ms.tolist(), gyro_residual_x.astype(float).tolist())),
        }

        try:
            result = spec.evaluate(dataset_pairs)
            return normalize_rtamt_trace(result)
        except Exception as second_error:
            raise RuntimeError(
                "RTAMT offline evaluation failed with both supported dataset formats.\n"
                f"First error using separate 'time' key:\n{first_error}\n\n"
                f"Second error using time-value pairs:\n{second_error}"
            )


def main():
    d = scipy.io.loadmat(DATASET_PATH)

    Yseg = d["Yseg"][0]
    EXTRAseg = d["EXTRAseg"][0]

    Y = Yseg[SEG_IDX]
    EXTR = EXTRAseg[SEG_IDX]

    Ts = float(d["Ts"].flat[0]) if "Ts" in d else TS_EXPECTED
    fs = float(d["fs"].flat[0]) if "fs" in d else FS_EXPECTED

    N = Y.shape[0]

    alt = Y[:, 2].astype(float)
    BARO_Alt = EXTR[:, 1].astype(float)
    GyrX_clean = Y[:, 9].astype(float)

    # Paper-aligned residuals (Choi et al., Algorithm 1): |m_sensor - ms_sensor|
    #   m_baro  = physical barometer measurement (dataset: BARO_Alt)
    #   ms_baro = software-sensor / model prediction (dataset: alt)
    #   m_gyr_x = physical roll-rate measurement (dataset: Y[:,9])
    #   ms_gyr_x = software-sensor prediction = model roll-rate state (paper §6.2)
    # STL below uses INSTANTANEOUS error |m - ms| (guide-based simplification of
    # the paper's accumulated residual r <- r + |m - ms|).
    ms_baro  = alt
    ms_gyr_x = GyrX_clean.copy()          # model angular-rate state = ms_gyr_x
    GyrX_predicted = ms_gyr_x             # alias kept for downstream references

    m_baro_clean  = BARO_Alt
    m_gyr_x_clean = GyrX_clean

    # ── Offline attack simulation ONLY: corrupt the physical measurements ──────
    m_baro = m_baro_clean.copy()
    m_baro[ATTACK_START:ATTACK_END] += BARO_ATTACK_OFFSET_M
    BARO_Alt_attacked = m_baro           # alias kept for downstream references

    m_gyr_x = m_gyr_x_clean.copy()
    m_gyr_x[ATTACK_START:ATTACK_END] = GYRX_ATTACK_VALUE_RAD_S
    GyrX_attacked = m_gyr_x              # alias kept for downstream references

    # Residuals: |m_sensor - ms_sensor|
    baro_residual_clean = np.abs(m_baro_clean - ms_baro)
    baro_residual_attacked = np.abs(m_baro - ms_baro)

    gyro_residual_x_clean = np.abs(m_gyr_x_clean - ms_gyr_x)
    gyro_residual_x_attacked = np.abs(m_gyr_x - ms_gyr_x)

    t_sec = np.arange(N) * Ts
    t_ms = (np.arange(N) * int(round(Ts * 1000))).astype(int)

    attack_start_time = t_sec[ATTACK_START]
    attack_end_time = t_sec[ATTACK_END]

    print(f"Loaded dataset: {DATASET_PATH}")
    print(f"Using segment: {SEG_IDX}")
    print(f"Ts = {Ts:.2f} s, fs = {int(round(fs))} Hz")
    print("Using Y[:, 2] as alt / model altitude AGL")
    print("Using EXTRA[:, 1] as BARO_Alt / barometer altitude AGL")
    print("Using Y[:, 9] as GyrX / roll rate")
    print("Barometer attack: BARO_Alt_attacked = BARO_Alt + 3.0 m")
    print("Gyroscope attack: GyrX_attacked = 0.8 rad/s")
    print(f"Attack samples: {ATTACK_START}:{ATTACK_END}")
    print(f"Attack window: {attack_start_time:.2f} s to {attack_end_time:.2f} s")
    print(f"Outer T: {OUTER_T_MS} ms")
    print(f"Recovery window: {RECOVERY_WINDOW_MS} ms")
    print(f"STL formula: {STL_FORMULA}")

    rho_time_clean_ms, rho_clean = evaluate_offline(
        baro_residual_clean,
        gyro_residual_x_clean,
        t_ms,
    )

    rho_time_attacked_ms, rho_attacked = evaluate_offline(
        baro_residual_attacked,
        gyro_residual_x_attacked,
        t_ms,
    )

    rho_time_clean_sec = rho_time_clean_ms / 1000.0
    rho_time_attacked_sec = rho_time_attacked_ms / 1000.0

    violations = np.where(rho_attacked < 0)[0]

    if len(violations) > 0:
        violation_idx = int(violations[0])
        violation_time = rho_time_attacked_sec[violation_idx]
        print(
            f"Recovery STL violation detected at t = {violation_time:.2f} s "
            f"(trace index {violation_idx})"
        )
        print(f"Attack started at t = {attack_start_time:.2f} s")
        print(f"Violation latency = {violation_time - attack_start_time:.2f} s")
    else:
        violation_idx = None
        print("No recovery STL violation detected.")

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    axes[0].plot(t_sec, BARO_Alt, label="BARO_Alt clean")
    axes[0].plot(t_sec, BARO_Alt_attacked, "--", label="BARO_Alt attacked")
    axes[0].plot(t_sec, alt, label="alt model")
    axes[0].axvspan(attack_start_time, attack_end_time, alpha=0.15, label="attack window")
    axes[0].set_ylabel("Altitude (m AGL)")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_sec, baro_residual_clean, label="baro_residual clean")
    axes[1].plot(t_sec, baro_residual_attacked, "--", label="baro_residual attacked")
    axes[1].axhline(EPS_BARO, linestyle=":", label="ε_baro = 0.30 m")
    axes[1].axvspan(attack_start_time, attack_end_time, alpha=0.15)
    axes[1].set_ylabel("Baro residual (m)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    axes[2].plot(t_sec, gyro_residual_x_clean, label="gyro_residual_x clean")
    axes[2].plot(t_sec, gyro_residual_x_attacked, "--", label="gyro_residual_x attacked")
    axes[2].axhline(EPS_GYR, linestyle=":", label="ε_gyr = 0.15 rad/s")
    axes[2].axvspan(attack_start_time, attack_end_time, alpha=0.15)
    axes[2].set_ylabel("Gyro X residual (rad/s)")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)

    axes[3].plot(rho_time_clean_sec, rho_clean, label="ρ clean")
    axes[3].plot(rho_time_attacked_sec, rho_attacked, "--", label="ρ attacked")
    axes[3].axhline(0.0, linewidth=1.5, label="ρ = 0 boundary")
    axes[3].axvspan(attack_start_time, attack_end_time, alpha=0.15)
    if violation_idx is not None:
        axes[3].axvline(
            rho_time_attacked_sec[violation_idx],
            linestyle=":",
            label=f"violation t={rho_time_attacked_sec[violation_idx]:.2f}s",
        )
    axes[3].set_ylabel("Robustness ρ")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(fontsize=8)
    axes[3].grid(alpha=0.3)

    plt.suptitle(
        "OFFLINE STL — Multi-Sensor Any-Attack Recovery (recorded 50 Hz dataset)\n"
        f"Spec: {STL_FORMULA}",
        fontsize=10,
    )
    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=130)
    print(f"Saved: {OUTPUT_PLOT}")


if __name__ == "__main__":
    main()
