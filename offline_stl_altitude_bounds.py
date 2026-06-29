#!/usr/bin/env python3
"""
Offline STL monitor: Altitude Bounds / Mission Spec S3-S4 equivalent

Interpreter:
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Dataset:
  /home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat

Formula:
  G[0:580ms] ((alt > 0.97) and (alt < 29.70))
"""

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import rtamt


DATASET_PATH = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_altitude_bounds.png"

SEG_IDX = 0

H_MIN = 0.97
H_MAX = 29.70

OUTER_T_MS = 580

STL_FORMULA = "G[0:580ms] ((alt > 0.97) and (alt < 29.70))"


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

    spec.declare_var("alt", "float")

    try:
        spec.set_sampling_period(20, "ms", 0.1)
    except TypeError:
        spec.set_sampling_period(20, "ms")

    spec.spec = STL_FORMULA
    spec.parse()
    return spec


def normalize_rtamt_trace(result):
    """
    Convert RTAMT offline output to:
      times_ms, rho_values
    """
    if isinstance(result, (float, int, np.floating, np.integer)):
        return np.array([0.0]), np.array([float(result)])

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


def evaluate_offline(alt, t_ms):
    """
    Prefer offline spec.evaluate().

    First try dataset with separate 'time' key.
    If this installed RTAMT version expects time-value pairs instead,
    use the smallest dataset-format fallback only.
    """
    spec = make_stl_spec()

    dataset_with_time_key = {
        "time": t_ms.tolist(),
        "alt": alt.astype(float).tolist(),
    }

    try:
        result = spec.evaluate(dataset_with_time_key)
        return normalize_rtamt_trace(result)
    except Exception as first_error:
        spec = make_stl_spec()

        dataset_pairs = {
            "alt": list(zip(t_ms.tolist(), alt.astype(float).tolist())),
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

    Ts = float(d["Ts"].flat[0])
    fs = float(d["fs"].flat[0])

    N = Y.shape[0]

    alt = Y[:, 2].astype(float)

    t_sec = np.arange(N) * Ts
    t_ms = (np.arange(N) * int(round(Ts * 1000))).astype(int)

    print(f"Loaded dataset: {DATASET_PATH}")
    print(f"Using segment: {SEG_IDX}")
    print(f"Ts = {Ts:.2f} s, fs = {int(round(fs))} Hz")
    print("Using Y[:, 2] as alt / model altitude AGL")
    print(f"Altitude lower bound h_min: {H_MIN:.2f} m")
    print(f"Altitude upper bound h_max: {H_MAX:.2f} m")
    print(f"Outer T: {OUTER_T_MS} ms")
    print(f"STL formula: {STL_FORMULA}")

    rho_time_ms, rho = evaluate_offline(alt, t_ms)
    rho_time_sec = rho_time_ms / 1000.0

    violations = np.where(rho < 0)[0]

    if len(violations) > 0:
        det_idx = int(violations[0])
        det_time = rho_time_sec[det_idx]
        print(
            f"Altitude bounds STL violation detected at t = {det_time:.2f} s "
            f"(trace index {det_idx})"
        )
    else:
        print("No altitude bounds STL violation detected.")

    alt_lower_margin = alt - H_MIN
    alt_upper_margin = H_MAX - alt
    pointwise_margin = np.minimum(alt_lower_margin, alt_upper_margin)

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)

    axes[0].plot(t_sec, alt, label="alt / model altitude AGL")
    axes[0].axhline(H_MIN, linestyle=":", label="h_min = 0.97 m")
    axes[0].axhline(H_MAX, linestyle=":", label="h_max = 29.70 m")
    axes[0].set_ylabel("Altitude (m AGL)")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_sec, alt_lower_margin, label="alt - h_min")
    axes[1].plot(t_sec, alt_upper_margin, label="h_max - alt")
    axes[1].plot(t_sec, pointwise_margin, "--", label="min pointwise bound margin")
    axes[1].axhline(0.0, linewidth=1.5, label="0 boundary")
    axes[1].set_ylabel("Pointwise margin (m)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    axes[2].plot(rho_time_sec, rho, label="ρ altitude bounds")
    axes[2].axhline(0.0, linewidth=1.5, label="ρ = 0 boundary")
    if len(violations) > 0:
        axes[2].axvline(
            rho_time_sec[det_idx],
            linestyle=":",
            label=f"violation t={rho_time_sec[det_idx]:.2f}s",
        )
    axes[2].set_ylabel("Robustness ρ")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)

    plt.suptitle(
        "STL Robustness — Altitude Bounds / Mission Spec S3-S4 equivalent\n"
        f"Spec: {STL_FORMULA}",
        fontsize=10,
    )
    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=130)

    print(f"Saved: {OUTPUT_PLOT}")


if __name__ == "__main__":
    main()
