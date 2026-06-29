#!/usr/bin/env python3
"""
Offline STL monitor on operation_data_50hz.mat
Interpreter: /home/tchowdh4/.pyenv/versions/3.10.14/bin/python3
"""

import scipy.io
import numpy as np
import rtamt
import matplotlib.pyplot as plt

# ── 1. Load dataset ──────────────────────────────────────────────────────────
d = scipy.io.loadmat('/home/tchowdh4/paperImp/rv_recovery/data/operation_data_50hz.mat')

Yseg     = d['Yseg'][0]
Useg     = d['Useg'][0]
EXTRAseg = d['EXTRAseg'][0]

Ts = float(d['Ts'].flat[0])
fs = float(d['fs'].flat[0])

# Pick one segment: segment 0
seg_idx = 0

Y    = Yseg[seg_idx]
U    = Useg[seg_idx]
EXTR = EXTRAseg[seg_idx]

N = Y.shape[0]

# Required channels from guide
alt      = Y[:, 2]      # altitude AGL, model state
baro_alt = EXTR[:, 1]   # BARO_Alt, measured barometer altitude AGL

# Residual: model altitude vs barometer altitude
baro_res = np.abs(alt - baro_alt)

# ── 2. Simulate barometer attack exactly as guide ────────────────────────────
baro_attacked = baro_alt.copy()

attack_start = 2000     # sample index = 40 s at 50 Hz
attack_end   = 2500     # sample index = 50 s at 50 Hz

baro_attacked[attack_start:attack_end] += 3.0

baro_res_attacked = np.abs(alt - baro_attacked)

# ── 3. Time axis ─────────────────────────────────────────────────────────────
t_sec = np.arange(N) * Ts
t_ms  = (np.arange(N) * 1000 * Ts).astype(int)

# ── 4. Define STL spec ───────────────────────────────────────────────────────
spec = rtamt.StlDiscreteTimeSpecification()

spec.declare_var('alt', 'float')
spec.declare_var('baro_res', 'float')

spec.set_sampling_period(int(Ts * 1000), 'ms')

spec.spec = 'G[0:2000ms] (baro_res < 0.30)'

spec.parse()

# ── 5. Evaluate robustness trace ─────────────────────────────────────────────
rho_clean = []
rho_attacked = []

for i in range(N):
    r1 = spec.update(i, [('alt', alt[i]),
                         ('baro_res', baro_res[i])])

    r2 = spec.update(i, [('alt', alt[i]),
                         ('baro_res', baro_res_attacked[i])])

    rho_clean.append(r1)
    rho_attacked.append(r2)

rho_clean = np.array(rho_clean)
rho_attacked = np.array(rho_attacked)

# ── 6. Find detection point ──────────────────────────────────────────────────
violations = np.where(rho_attacked < 0)[0]

print("Dataset loaded successfully")
print(f"Segment index: {seg_idx}")
print(f"Y shape: {Y.shape}")
print(f"U shape: {U.shape}")
print(f"EXTRA shape: {EXTR.shape}")
print(f"Sampling time Ts: {Ts}")
print(f"Sampling frequency fs: {fs}")
print(f"STL formula: {spec.spec}")
print(f"Attack window: sample {attack_start} to {attack_end}")
print(f"Attack start time: {t_sec[attack_start]:.2f} s")
print(f"Attack end time: {t_sec[attack_end]:.2f} s")

if len(violations):
    det_idx = violations[0]
    print(f"Attack detected at t = {t_sec[det_idx]:.2f} s  (sample {det_idx})")
    print(f"Detection latency = {t_sec[det_idx] - t_sec[attack_start]:.2f} s")
else:
    print("No violation detected")

# ── 7. Plot ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axes[0].plot(t_sec, alt, label='alt (model)', color='blue')
axes[0].plot(t_sec, baro_alt, label='BARO_Alt (clean)', color='green', alpha=0.6)
axes[0].plot(t_sec, baro_attacked, label='BARO_Alt (attacked)', color='red', alpha=0.6, ls='--')
axes[0].axvspan(t_sec[attack_start], t_sec[attack_end], color='red', alpha=0.1, label='attack window')
axes[0].set_ylabel('Altitude (m AGL)')
axes[0].legend(fontsize=8)
axes[0].grid(alpha=0.3)

axes[1].plot(t_sec, baro_res, label='residual (clean)', color='green')
axes[1].plot(t_sec, baro_res_attacked, label='residual (attacked)', color='red', ls='--')
axes[1].axhline(0.30, color='orange', ls=':', label='threshold ε=0.30m')
axes[1].set_ylabel('|alt - BARO| (m)')
axes[1].legend(fontsize=8)
axes[1].grid(alpha=0.3)

axes[2].plot(t_sec, rho_clean, label='ρ (clean)', color='green')
axes[2].plot(t_sec, rho_attacked, label='ρ (attacked)', color='red', ls='--')
axes[2].axhline(0, color='black', lw=1.5, ls='-', label='ρ = 0 boundary')

if len(violations):
    axes[2].axvline(t_sec[det_idx], color='purple', ls=':', label=f'detection t={t_sec[det_idx]:.1f}s')

axes[2].set_ylabel('Robustness ρ')
axes[2].set_xlabel('Time (s)')
axes[2].legend(fontsize=8)
axes[2].grid(alpha=0.3)

plt.suptitle('STL Robustness — Barometer Attack Detection\n'
             f'Spec: {spec.spec}', fontsize=10)

plt.tight_layout()
plt.savefig('/home/tchowdh4/paperImp/stl_result.png', dpi=130)

print("Saved: /home/tchowdh4/paperImp/stl_result.png")

plt.show()
