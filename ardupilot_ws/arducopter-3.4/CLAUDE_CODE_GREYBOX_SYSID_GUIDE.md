# Claude Code Guide — Paper-Faithful System Identification (Grey-Box `ssest` / PEM)

**Goal:** Replicate Choi et al. RAID 2020, Section 3.1 **as written**: a *constrained*
discrete-time state-space model identified with **prediction error minimization (PEM)**,
where the template encodes the rigid-body dynamics and PID controller known a priori, and
**only the physical coefficients are free**.

**This replaces the prior black-box approach.** Do NOT use `n4sid` and do NOT fit a free
12-state model. The paper does neither. The paper says (§3.1):

> "we use a discrete-time state-space model template, encoding a PID controller and dynamics
> equations known a priori for the family of the subject RV. Then, for each variable, we
> specify a model order ... we employ SI to instantiate the unknown coefficients of the
> template using an iterative prediction error minimization algorithm."

The operative words are **template**, **known a priori**, **instantiate the unknown
coefficients**. That is a *grey-box* identification (`idgrey` / `greyest` / structured `ssest`
in MATLAB), not subspace ID and not a free black box.

---

## 0. Why this matters (read before coding)

A state-space model `x⁺ = Ax + Bu`, `y = Cx + Du` plays **two roles**:
- the **state equation** is a physics simulator (rolls the vehicle state forward),
- the **output equation** is a sensor model (what a healthy sensor *should* report).

The attacker can corrupt only the measured stream `m`, never the physics. The software
sensor `ms = convert(Cx + Du)` is an independent estimate grounded in commands + dynamics,
which the threat model says the attacker cannot observe. That independence is the entire
defense. A black-box fit destroys the physical meaning of `x`, so you can no longer reason
about *which* sensor maps to *which* state, can't check observability per sensor, and can't
defend the model on physical grounds. The grey box keeps `x` physical. That is the point.

Three properties you must be able to compute and report (these are how the field "proves"
things about such models):
- **Stability:** spectral radius of `A` (discrete: all `|λ| < 1` ⇒ open-loop decays).
- **Controllability:** rank of `[B AB … Aⁿ⁻¹B]` (can `u` reach all states).
- **Observability:** rank of `[C; CA; … ; CAⁿ⁻¹]` (can a sensor be reconstructed from outputs).
  Observability is *why* a software sensor for a given physical sensor is possible at all.

---

## 1. The physical state and what is FIXED vs FREE

Paper Eq. (3), 12 states, continuous-time intuition first:

```
x = [ x  y  z |  φ  θ  ψ |  ẋ  ẏ  ż |  p  q  r ]
      position  attitude    lin.vel    ang.vel
```

The template splits into rows that are **exact kinematics (fixed)** and rows that are
**dynamics + control (free coefficients)**.

### 1.1 FIXED rows — pure kinematics, no unknowns

These are definitions, not physics to be learned:

```
ẋ = ẋ        d/dt(position)  = linear velocity
ẏ = ẏ
ż = ż

φ̇ ≈ p       d/dt(attitude)  ≈ body angular rate
θ̇ ≈ q       (small-angle hover approximation: W_η ≈ I, paper App. A Eq. 9-10
ψ̇ ≈ r        reduces to identity near level flight)
```

So six rows of the continuous `A` are **known constants (1's)**, not free parameters.
This is the "dynamics equations known a priori" the paper refers to.

### 1.2 FREE rows — dynamics + closed-loop PID (the coefficients to identify)

**Translational (near hover, thrust ≈ mg):** horizontal acceleration comes from tilt,
vertical from throttle. Linearized about hover:

```
ẍ ≈  g·θ                    tilt forward → accelerate forward   (g free-ish / identified)
ÿ ≈ -g·φ                    tilt right   → accelerate right
z̈ ≈  kt·u_throttle - kz·ż   throttle minus drag
```

**Rotational (this is where the PID lives):** the attitude controller drives angular rate
to track commanded attitude. Closing the loop, each axis becomes a **second-order system**
— exactly the paper's "dominating system dynamic is a second-order system" (§3.1):

```
ṗ = -a_p1·p - a_p0·φ + b_p·φ_cmd      roll  axis closed-loop 2nd order
q̇ = -a_q1·q - a_q0·θ + b_q·θ_cmd      pitch axis
ṙ = -a_r1·r - a_r0·ψ + b_r·ψ_cmd      yaw   axis
```

The `a_*` and `b_*` are the **unknown coefficients** PEM will instantiate. They bundle the
PID gains, the moments of inertia, and the motor constants — exactly the things the paper
says are "jointly determined by the RV's physical attributes, its control algorithm, and the
laws of physics."

### 1.3 Inputs

```
u = [ φ_cmd  θ_cmd  ψ_cmd  u_throttle ]   (from Dataflash: DesRoll, DesPitch, DesYaw, ThI)
```

These enter only the FREE rows (`ṗ, q̇, ṙ, z̈`). They do NOT enter the kinematic rows.

### 1.4 Output / sensor map `C`

`C` maps physical state to sensor channels. For the channels we log it is mostly a
**selection matrix** (identity rows picking out states), with conversions handled separately
by `software_sensors.h` (baro Eq. 5, etc.). Keeping `C` structured (not free) is what lets
you say "output channel k IS gyro-X" with a straight face. Do not let PEM scramble `C`.

---

## 2. MATLAB grey-box construction (`idgrey` + `greyest`/`ssest`)

Create `~/rv_recovery/matlab/sysid_greybox.m`. This is the **paper-faithful** identifier.

### 2.1 The parameterized template function

The grey-box approach needs a function that builds continuous `A,B,C,D` from the free
parameter vector. Save as `~/rv_recovery/matlab/quad_template.m`:

```matlab
function [A,B,C,D] = quad_template(params, Ts, aux)
% QUAD_TEMPLATE  Grey-box state-space template for a quadrotor (paper Eq.1-3).
% Encodes rigid-body kinematics (FIXED) + closed-loop PID dynamics (FREE params).
%
% State x = [x y z  phi theta psi  xd yd zd  p q r]  (12)
% Input u = [phi_cmd theta_cmd psi_cmd throttle]      (4)
%
% params (free coefficients to be identified by PEM):
%   1: g_x      effective gravity coupling tilt->accel (x via theta)
%   2: g_y      effective gravity coupling tilt->accel (y via phi)
%   3: kt       throttle -> vertical accel gain
%   4: kz       vertical velocity drag
%   5: a_p1     roll-rate damping
%   6: a_p0     roll-angle restoring (PID P-term effect)
%   7: b_p      roll command gain
%   8: a_q1     pitch-rate damping
%   9: a_q0     pitch-angle restoring
%  10: b_q      pitch command gain
%  11: a_r1     yaw-rate damping
%  12: a_r0     yaw-angle restoring
%  13: b_r      yaw command gain
%
% aux unused (required by idgrey signature).

    g   = 9.81;
    p   = params;   % shorthand

    % ---- indices ----
    X=1; Y=2; Z=3; PH=4; TH=5; PS=6; XD=7; YD=8; ZD=9; P=10; Q=11; R=12;

    A = zeros(12,12);
    B = zeros(12,4);

    % ===== FIXED kinematic rows (no free params) =====
    A(X,XD) = 1;            % xdot = xd
    A(Y,YD) = 1;            % ydot = yd
    A(Z,ZD) = 1;            % zdot = zd
    A(PH,P) = 1;            % phidot   ~ p   (small-angle)
    A(TH,Q) = 1;            % thetadot ~ q
    A(PS,R) = 1;            % psidot   ~ r

    % ===== FREE translational rows =====
    A(XD,TH) =  p(1)*g;     % xddot  =  g_x * g * theta
    A(YD,PH) = -p(2)*g;     % yddot  = -g_y * g * phi
    A(ZD,ZD) = -p(4);       % zddot  = -kz*zd + kt*throttle
    B(ZD,4)  =  p(3);

    % ===== FREE rotational rows (closed-loop PID, 2nd order per axis) =====
    A(P,P)  = -p(5);  A(P,PH) = -p(6);   B(P,1) = p(7);   % roll
    A(Q,Q)  = -p(8);  A(Q,TH) = -p(9);   B(Q,2) = p(10);  % pitch
    A(R,R)  = -p(11); A(R,PS) = -p(12);  B(R,3) = p(13);  % yaw

    % ===== Output map C: structured selection (NOT free) =====
    % Order Y columns to match parse_dataflash.py y_labels.
    % Here: pick the directly-sensed states. Conversions (baro etc.) done downstream.
    % Channels: [Roll Pitch Yaw  GyrX GyrY GyrZ  ... ]  -> attitude + body rates first.
    C = zeros(6,12);
    C(1,PH)=1; C(2,TH)=1; C(3,PS)=1;     % Roll,Pitch,Yaw  (= phi,theta,psi)
    C(4,P)=1;  C(5,Q)=1;  C(6,R)=1;      % GyrX,GyrY,GyrZ  (= p,q,r)
    D = zeros(6,4);

    % NOTE: continuous-time model. idgrey will discretize with Ts.
end
```

> If you need more output channels (accel, baro, GPS), extend `C` with the corresponding
> selection/derivative rows — but keep them **structured**, not free. Start with the 6
> attitude+rate channels: they are what the gyro-attack experiment needs and they are the
> ones with the cleanest observability.

### 2.2 The identification script

Save as `~/rv_recovery/matlab/sysid_greybox.m`:

```matlab
%% sysid_greybox.m — PAPER-FAITHFUL grey-box PEM identification (Choi et al. §3.1)
% Replaces sysid_n4sid.m. Uses idgrey + greyest (PEM) on a structured template.
clear; clc;

%% 1. Load operation data (from parse_dataflash.py)
load(fullfile(getenv('HOME'),'rv_recovery','data','operation_data.mat'));
% Provides: U [N x 4], Y [N x 16], Ts (=0.0025 if parsed at 400 Hz)

% Select the 6 outputs the template's C predicts, in C's row order:
% [Roll Pitch Yaw GyrX GyrY GyrZ]
% Adjust these indices to match your y_labels!
yidx = [1 2 3 4 5 6];
Ysel = Y(:, yidx);

% Inputs in template order: [phi_cmd theta_cmd psi_cmd throttle]
% = [DesRoll DesPitch DesYaw ThrottleIn]; adjust to your u_labels.
Usel = U(:, [1 2 3 4]);

%% 2. PAPER FIDELITY: keep the model at the control-loop rate (400 Hz).
% Do NOT downsample to 50 Hz. The paper's context is 400 Hz (2.5 ms loop).
% If memory is a concern, reduce DATA LENGTH (use fewer missions / a time slice),
% NOT the sample rate. Downsampling changes the dynamics the model sees.
Ts_model = Ts;             % e.g. 0.0025

% Optional: detrend each channel (remove DC) so PEM fits dynamics, not offsets.
Ysel = Ysel - mean(Ysel,1,'omitnan');
Usel = Usel - mean(Usel,1,'omitnan');

% Drop NaN rows
ok = all(isfinite(Ysel),2) & all(isfinite(Usel),2);
Ysel = Ysel(ok,:); Usel = Usel(ok,:);

% Train/validate split
n = size(Ysel,1); ne = round(0.7*n);
data_est = iddata(Ysel(1:ne,:),    Usel(1:ne,:),    Ts_model);
data_val = iddata(Ysel(ne+1:end,:),Usel(ne+1:end,:),Ts_model);
data_est.TimeUnit='seconds'; data_val.TimeUnit='seconds';

%% 3. Build the grey-box model object
% Initial guess for the 13 free params (physically reasonable):
p0 = [1.0;   % g_x
      1.0;   % g_y
      9.0;   % kt
      0.5;   % kz
      8.0;   % a_p1
     20.0;   % a_p0
     20.0;   % b_p
      8.0;   % a_q1
     20.0;   % a_q0
     20.0;   % b_q
      4.0;   % a_r1
      6.0;   % a_r0
      6.0];  % b_r

% CONTINUOUS-time grey box; greyest discretizes internally with Ts.
sys0 = idgrey(@quad_template, p0, 'c', {}, Ts_model);

% Bound/condition the search (keeps params physical, aids convergence)
sys0.Structure.Parameters.Minimum = zeros(13,1);   % all coeffs >= 0 by construction
%% 4. PEM identification (paper's "iterative prediction error minimization")
opt = greyestOptions;
opt.InitialState   = 'estimate';
opt.Display        = 'on';
opt.SearchMethod   = 'lm';          % Levenberg-Marquardt (a PEM search)
opt.EnforceStability = false;       % PAPER FIDELITY: do NOT force stability.
                                    % Report whatever PEM finds; resync handles drift.

sysc = greyest(data_est, sys0, opt);   % <-- this is the paper's SI step

%% 5. Discretize for the firmware (zero-order hold at the loop rate)
sysd = c2d(sysc, Ts_model, 'zoh');
A = sysd.A; B = sysd.B; C = sysd.C; D = sysd.D;

%% 6. PROVE properties (this is the part the user asked for — report them)
lambda = eig(A);
rho    = max(abs(lambda));                 % spectral radius (stability)
Co     = ctrb(A,B);  rank_ctrb = rank(Co); % controllability
Ob     = obsv(A,C);  rank_obsv = rank(Ob); % observability
nx     = size(A,1);

fprintf('\n===== MODEL PROPERTY REPORT (discrete, Ts=%.4g s) =====\n', Ts_model);
fprintf('  states nx           = %d\n', nx);
fprintf('  spectral radius |A| = %.5f   (%s)\n', rho, ...
        ternary(rho<1,'open-loop STABLE', 'open-loop UNSTABLE/marginal'));
fprintf('  controllability rank= %d / %d  (%s)\n', rank_ctrb, nx, ...
        ternary(rank_ctrb==nx,'fully controllable','NOT fully controllable'));
fprintf('  observability   rank= %d / %d  (%s)\n', rank_obsv, nx, ...
        ternary(rank_obsv==nx,'fully observable','NOT fully observable'));
fprintf('  est fit (NRMSE %%)   = %s\n', mat2str(round(sysd.Report.Fit.FitPercent,1)));
fprintf('=========================================================\n');

% If unstable: DO NOT switch to n4sid. The paper keeps PEM and relies on the
% windowed synchronization/error-reset (Section 3.3) to bound drift over the
% ~10 s recovery window. Record rho and move on. Per-axis observability is what
% tells you whether a given software sensor is viable.

%% 7. Validate against held-out data
figure; compare(data_val, sysd);
title(sprintf('Grey-box PEM validation  (|A|=%.4f)', rho));

%% 8. Save model + property report
outdir = fullfile(getenv('HOME'),'rv_recovery','matlab','models');
if ~exist(outdir,'dir'); mkdir(outdir); end
params = getpvec(sysc);
save(fullfile(outdir,'quadrotor_greybox.mat'), ...
     'A','B','C','D','Ts_model','params','lambda','rho', ...
     'rank_ctrb','rank_obsv','nx');

%% 9. Export C header for firmware (paper-faithful open-loop form: x = A x + B u)
fid = fopen(fullfile(outdir,'model_matrices.h'),'w');
fprintf(fid,'// Grey-box PEM model (Choi et al. RAID2020 §3.1). Open-loop form.\n');
fprintf(fid,'// x[k+1] = A x[k] + B u[k];  y[k] = C x[k] + D u[k]\n');
fprintf(fid,'// No Kalman/observer term (paper Algorithm 1 line 7 is x = Ax + Bu).\n');
fprintf(fid,'// spectral_radius(A) = %.5f\n\n', rho);
fprintf(fid,'#pragma once\n');
fprintf(fid,'static const int   NX = %d;\n', size(A,1));
fprintf(fid,'static const int   NU = %d;\n', size(B,2));
fprintf(fid,'static const int   NY = %d;\n', size(C,1));
fprintf(fid,'static const float TS = %.8ff;\n\n', Ts_model);
wm(fid,'A_MAT',A); wm(fid,'B_MAT',B); wm(fid,'C_MAT',C); wm(fid,'D_MAT',D);
fclose(fid);
fprintf('Wrote model_matrices.h (open-loop, paper-faithful)\n');

%% helpers
function s = ternary(c,a,b); if c; s=a; else; s=b; end; end
function wm(fid,name,M)
  [r,c]=size(M);
  fprintf(fid,'static const float %s[%d][%d] = {\n',name,r,c);
  for i=1:r; fprintf(fid,'  {'); fprintf(fid,'%.8ff, ',M(i,:)); fprintf(fid,'},\n'); end
  fprintf(fid,'};\n\n');
end
```

---

## 3. Hard rules for this step (paper fidelity)

1. **Use `idgrey` + `greyest` (PEM).** Not `n4sid`. Not free `ssest`. The structure IS the
   paper's "template ... known a priori."
2. **Keep `C` structured.** A channel must map to a known physical state. If you can't say
   "output k is sensor S," the model isn't paper-faithful.
3. **Identify at the control-loop rate (400 Hz).** If memory is tight, cut data length, not
   sample rate. Downsampling to 50 Hz changes the identified dynamics and breaks the timebase
   contract with the firmware.
4. **Do NOT enforce stability and do NOT switch methods if `|A| ≥ 1`.** Report the spectral
   radius. The paper tolerates a marginal open-loop model and bounds drift with the windowed
   synchronization + error reset (§3.3). That resync is the designed remedy — implement it
   rather than papering over instability by changing the identification algorithm.
5. **Firmware uses the open-loop form** `x = A·x + B·u` (paper Algorithm 1, line 7). No
   Kalman innovation term in the firmware path. If you keep an observer form anywhere, label
   it explicitly as an implementation deviation, never as the paper's algorithm.
6. **Always emit the property report** (spectral radius, controllability rank, observability
   rank, fit%). These three numbers are how the model is "proven" — they must appear in the
   replication record for every identified model.

---

## 4. What to do with the observability result (conceptual hook)

After running, look at **per-channel observability**, not just the global rank. If you reduce
`C` to a single sensor type (e.g. only gyro rows) and `obsv` drops rank, that sensor *alone*
cannot reconstruct the full state — which is exactly why the paper adds the
accelerometer+magnetometer compensation (Appendix B) when **all** gyros are attacked. So:

- Compute `rank(obsv(A, C_gyro_only))`. If < nx, document that gyro-only recovery is
  observability-limited and that App. B compensation is the paper's answer.
- This is the principled justification for the supplementary-compensation path — not a hack,
  an observability necessity.

---

## 5. Deliverables for this task

- `~/rv_recovery/matlab/quad_template.m` (grey-box template)
- `~/rv_recovery/matlab/sysid_greybox.m` (PEM identification + property report)
- `~/rv_recovery/matlab/models/quadrotor_greybox.mat`
- `~/rv_recovery/matlab/models/model_matrices.h` (open-loop, paper-faithful)
- A printed/saved **property report**: spectral radius, ctrb rank, obsv rank, fit%, for the
  record. Retire `sysid_n4sid.m` (keep it only as a labeled deviation experiment, not the
  primary model).
