import os
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import rtamt


DATASET_PATH = "/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat"
OUTPUT_PLOT = "/home/tchowdh4/paperImp/stl_result_baro_persistent.png"

TS = 0.02
FS = 50

SEGMENT_INDEX = 0

ATTACK_START = 2000
ATTACK_END = 2500
BARO_ATTACK_OFFSET = 3.0

EPSILON_BARO = 0.30

OUTER_WINDOW_MS = 580
INNER_PERSISTENCE_WINDOW_MS = 1000


def make_discrete_time_spec():
    try:
        return rtamt.STLDiscreteTimeSpecification()
    except AttributeError:
        return rtamt.StlDiscreteTimeSpecification()


def extract_robustness(result):
    times = []
    values = []

    for item in result:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            times.append(float(item[0]))
            values.append(float(item[1]))
        else:
            values.append(float(item))

    return np.array(times), np.array(values)


def main():
    if not os.path.exists(DATASET_PATH):
        raise FileNotFoundError(f"Dataset not found: {DATASET_PATH}")

    d = sio.loadmat(DATASET_PATH)

    Yseg = d["Yseg"][0]
    EXTRAseg = d["EXTRAseg"][0]

    Y = Yseg[SEGMENT_INDEX]
    EXTR = EXTRAseg[SEGMENT_INDEX]

    alt = Y[:, 2]
    baro_alt = EXTR[:, 1]

    n = len(alt)
    t = np.arange(n) * TS

    # Paper-aligned residual (Choi et al., Algorithm 1): |m_baro - ms_baro|
    #   m_baro  = physical barometer measurement (dataset: BARO_Alt)
    #   ms_baro = software-sensor / model prediction (dataset: alt)
    # STL below uses the INSTANTANEOUS error |m - ms| (guide-based simplification
    # of the paper's accumulated residual r <- r + |m - ms|).
    ms_baro = alt

    # Offline attack simulation ONLY: corrupt the physical measurement m_baro.
    m_baro = np.array(baro_alt, copy=True)
    m_baro[ATTACK_START:ATTACK_END] += BARO_ATTACK_OFFSET
    baro_alt_attacked = m_baro   # alias kept for downstream references

    baro_residual = np.abs(m_baro - ms_baro)

    spec = make_discrete_time_spec()
    spec.name = "Persistent Barometer Attack Pattern"

    spec.declare_var("baro_residual", "float")

    spec.spec = (
        f"always[0:{OUTER_WINDOW_MS}ms] "
        f"(always[0:{INNER_PERSISTENCE_WINDOW_MS}ms] "
        f"(baro_residual < {EPSILON_BARO}))"
    )

    spec.set_sampling_period(20, "ms", 0.1)
    spec.parse()

    dataset = {
        "time": list(t),
        "baro_residual": list(baro_residual),
    }

    result = spec.evaluate(dataset)
    rho_time, rho = extract_robustness(result)

    if len(rho_time) == 0:
        rho_time = t[: len(rho)]

    detected_indices = np.where(rho < 0)[0]

    print(f"Loaded dataset: {DATASET_PATH}")
    print(f"Using segment: {SEGMENT_INDEX}")
    print("Using Y[:, 2] as alt / altitude AGL")
    print("Using EXTR[:, 1] as BARO_Alt")
    print(f"Ts = {TS:.2f} s")
    print(f"fs = {FS} Hz")
    print(f"Attack samples: {ATTACK_START}:{ATTACK_END}")
    print(f"Attack window: {ATTACK_START * TS:.2f} s to {ATTACK_END * TS:.2f} s")
    print(f"Attack value: BARO_Alt + {BARO_ATTACK_OFFSET:.1f} m")
    print(f"epsilon_baro = {EPSILON_BARO:.2f} m")
    print(
        "STL formula: "
        f"G[0:{OUTER_WINDOW_MS}ms] "
        f"(G[0:{INNER_PERSISTENCE_WINDOW_MS}ms] "
        f"(baro_residual < {EPSILON_BARO:.2f}))"
    )

    if len(detected_indices) > 0:
        detection_index = detected_indices[0]
        detection_time = rho_time[detection_index]
        detection_latency = detection_time - (ATTACK_START * TS)

        print(f"Attack detected at t = {detection_time:.2f} s")
        print(f"Detection latency = {detection_latency:.2f} s")
    else:
        print("No attack detected by STL robustness crossing below 0")

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, alt, label="alt = Y[:, 2]")
    plt.plot(t, baro_alt, label="BARO_Alt original = EXTR[:, 1]")
    plt.plot(t, baro_alt_attacked, label="BARO_Alt attacked")
    plt.axvspan(ATTACK_START * TS, ATTACK_END * TS, alpha=0.2, label="attack window")
    plt.ylabel("Altitude (m)")
    plt.title("OFFLINE STL — Persistent Barometer Attack Pattern (recorded 50 Hz dataset)")
    plt.grid(True)
    plt.legend(loc="best")

    plt.subplot(3, 1, 2)
    plt.plot(t, baro_residual, label="baro_residual = |BARO_Alt_attacked - alt|")
    plt.axhline(EPSILON_BARO, linestyle="--", label="epsilon_baro = 0.30 m")
    plt.axvspan(ATTACK_START * TS, ATTACK_END * TS, alpha=0.2, label="attack window")
    plt.ylabel("Residual (m)")
    plt.grid(True)
    plt.legend(loc="best")

    plt.subplot(3, 1, 3)
    plt.plot(rho_time, rho, label="STL robustness")
    plt.axhline(0.0, linestyle="--", label="robustness = 0")
    plt.axvspan(ATTACK_START * TS, ATTACK_END * TS, alpha=0.2, label="attack window")
    plt.xlabel("Time (s)")
    plt.ylabel("Robustness")
    plt.grid(True)
    plt.legend(loc="best")

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    plt.close()

    print(f"Saved plot: {OUTPUT_PLOT}")


if __name__ == "__main__":
    main()
