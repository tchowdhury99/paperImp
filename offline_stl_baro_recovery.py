#!/usr/bin/env python3
"""
Offline STL monitor: Barometer Recovery Within 10 s

Interpreter:
  /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3

Dataset:
  /home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat

Formula:
  G[0:580ms] ((baro_residual > 0.30) -> F[0:10000ms] (baro_residual < 0.30))
"""

import numpy as np
import scipy.io
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rtamt


DATASET_PATH = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_baro_recovery.png"

SEG_IDX = 0

EPS_BARO = 0.30
ATTACK_VALUE_M = 3.0
ATTACK_START = 2000
ATTACK_END = 2500

OUTER_T_MS = 580
RECOVERY_WINDOW_MS = 10000


def make_discrete_time_spec():
    """
    Smallest compatibility fallback only:
    first try rtamt.STLDiscreteTimeSpecification,
    then rtamt.StlDiscreteTimeSpecification.
    """
    if hasattr(rtamt, "STLDiscreteTimeSpecification"):
        return rtamt.STLDiscreteTimeSpecification()
    return rtamt.StlDiscreteTimeSpecification()


def normalize_rtamt_output(rho_output):
    """
    RTAMT offline evaluate can return different shapes depending on version:
      - list of (time, robustness)
      - list/array of robustness values
      - single scalar robustness

    This function converts the output to:
      times_ms_out, rho_values
    """
    if isinstance(rho_output, (float, int, np.floating, np.integer)):
        return np.array([0], dtype=int), np.array([float(rho_output)], dtype=float)

    arr = np.array(rho_output, dtype=object)

    if len(arr) == 0:
        return np.array([], dtype=int), np.array([], dtype=float)

    first = arr[0]

    if isinstance(first, (list, tuple, np.ndarray)) and len(first) >= 2:
        times = np.array([int(x[0]) for x in rho_output], dtype=int)
        vals = np.array([float(x[1]) for x in rho_output], dtype=float)
        return times, vals

    vals = np.array(rho_output, dtype=float)
    times = np.arange(len(vals), dtype=int) * 20
    return times, vals


def evaluate_offline_trace(spec, times_ms, baro_residual):
    """
    Prefer offline spec.evaluate().

    Uses a dataset dictionary with a separate 'time' key.
    If the installed RTAMT version rejects that form, use the smallest
    compatibility correction: list of (time, value) pairs for the variable.
    """
    dataset_with_time_key = {
        "time": times_ms.tolist(),
        "baro_residual": baro_residual.astype(float).tolist(),
    }

    try:
        return spec.evaluate(dataset_with_time_key)
    except Exception as e1:
        print("Offline evaluate with separate 'time' key failed.")
        print("Using smallest compatibility correction: variable as (time, value) pairs.")
        print(f"Original evaluate error: {type(e1).__name__}: {e1}")

        dataset_pairs = {
            "baro_residual": list(zip(times_ms.tolist(), baro_residual.astype(float).tolist())),
        }
        return spec.evaluate(dataset_pairs)


def main():
    # ── 1. Load dataset ──────────────────────────────────────────────────────
    d = scipy.io.loadmat(DATASET_PATH)

    Yseg = d["Yseg"][0]
    EXTRAseg = d["EXTRAseg"][0]

    Y = Yseg[SEG_IDX]
    EXTR = EXTRAseg[SEG_IDX]

    Ts = float(d["Ts"].flat[0])
    fs = float(d["fs"].flat[0])
    N = Y.shape[0]

    # ── 2. Extract exact guide channels ─────────────────────────────────────
    alt = Y[:, 2]          # m AGL, model altitude
    BARO_Alt = EXTR[:, 1]  # m AGL, barometer altitude

    # ── 3. Simulate exact barometer attack ──────────────────────────────────
    BARO_Alt_attacked = BARO_Alt.copy()
    BARO_Alt_attacked[ATTACK_START:ATTACK_END] += ATTACK_VALUE_M

    # Exact residual construction:
    # baro_residual(t) = |BARO_Alt_attacked(t) - alt(t)|
    baro_residual_clean = np.abs(BARO_Alt - alt)
    baro_residual_attacked = np.abs(BARO_Alt_attacked - alt)

    # ── 4. Time axis ────────────────────────────────────────────────────────
    t_sec = np.arange(N) * Ts
    t_ms = (np.arange(N) * int(Ts * 1000)).astype(int)

    attack_start_sec = t_sec[ATTACK_START]
    attack_end_sec = t_sec[ATTACK_END]

    # ── 5. Define exact STL formula ─────────────────────────────────────────
    spec = make_discrete_time_spec()
    spec.declare_var("baro_residual", "float")

    # RTAMT compatibility setting used in previous completed STL steps.
    spec.set_sampling_period(20, "ms", 0.1)

    spec.spec = (
        "G[0:580ms] "
        "((baro_residual > 0.30) -> F[0:10000ms] (baro_residual < 0.30))"
    )

    spec.parse()

    # ── 6. Offline evaluation ───────────────────────────────────────────────
    rho_clean_raw = evaluate_offline_trace(spec, t_ms, baro_residual_clean)
    rho_attacked_raw = evaluate_offline_trace(spec, t_ms, baro_residual_attacked)

    rho_clean_t_ms, rho_clean = normalize_rtamt_output(rho_clean_raw)
    rho_attacked_t_ms, rho_attacked = normalize_rtamt_output(rho_attacked_raw)

    rho_clean_t_sec = rho_clean_t_ms / 1000.0
    rho_attacked_t_sec = rho_attacked_t_ms / 1000.0

    # ── 7. Find violation point ─────────────────────────────────────────────
    violations = np.where(rho_attacked < 0)[0]

    print(f"Loaded dataset: {DATASET_PATH}")
    print(f"Using segment: {SEG_IDX}")
    print(f"Ts = {Ts:.2f} s, fs = {fs:.0f} Hz")
    print("Using Y[:, 2] as alt / model altitude AGL")
    print("Using EXTRA[:, 1] as BARO_Alt / barometer altitude AGL")
    print(f"Barometer threshold epsilon_baro = {EPS_BARO:.2f} m")
    print(f"Attack: BARO_Alt_attacked = BARO_Alt + {ATTACK_VALUE_M:.1f} m")
    print(f"Attack samples: {ATTACK_START}:{ATTACK_END}")
    print(f"Attack window: {attack_start_sec:.2f} s to {attack_end_sec:.2f} s")
    print(f"Outer T: {OUTER_T_MS} ms")
    print(f"Recovery window: {RECOVERY_WINDOW_MS} ms")
    print(f"STL formula: {spec.spec}")

    if len(violations):
        det_idx = violations[0]
        det_time = rho_attacked_t_sec[det_idx]
        print(f"Recovery STL violation detected at t = {det_time:.2f} s")
        print(f"  Attack started at t = {attack_start_sec:.2f} s")
        print(f"  Detection latency = {det_time - attack_start_sec:.2f} s")
    else:
        print("No recovery STL violation detected.")
        print("This means the attacked barometer signal satisfied the selected recovery-within-10s formula.")

    # ── 8. Plot ─────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=False)

    axes[0].plot(t_sec, alt, label="alt = Y[:, 2] model altitude")
    axes[0].plot(t_sec, BARO_Alt, label="BARO_Alt clean = EXTRA[:, 1]", alpha=0.7)
    axes[0].plot(t_sec, BARO_Alt_attacked, label="BARO_Alt attacked", linestyle="--", alpha=0.8)
    axes[0].axvspan(attack_start_sec, attack_end_sec, alpha=0.15, label="attack window")
    axes[0].set_ylabel("Altitude (m AGL)")
    axes[0].set_title("Barometer Recovery Within 10 s — Signals")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_sec, baro_residual_clean, label="baro_residual clean")
    axes[1].plot(t_sec, baro_residual_attacked, label="baro_residual attacked", linestyle="--")
    axes[1].axhline(EPS_BARO, linestyle=":", label="epsilon_baro = 0.30 m")
    axes[1].axvspan(attack_start_sec, attack_end_sec, alpha=0.15, label="attack window")
    axes[1].set_ylabel("|BARO_Alt - alt| (m)")
    axes[1].set_title("Residual")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    axes[2].plot(rho_clean_t_sec, rho_clean, label="rho clean")
    axes[2].plot(rho_attacked_t_sec, rho_attacked, label="rho attacked", linestyle="--")
    axes[2].axhline(0.0, linewidth=1.5, label="rho = 0 boundary")
    if len(violations):
        axes[2].axvline(rho_attacked_t_sec[violations[0]], linestyle=":", label="first violation")
    axes[2].axvspan(attack_start_sec, attack_end_sec, alpha=0.15, label="attack window")
    axes[2].set_ylabel("Robustness rho")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title(f"STL Robustness: {spec.spec}")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=130)
    print(f"Saved: {OUTPUT_PLOT}")


if __name__ == "__main__":
    main()
