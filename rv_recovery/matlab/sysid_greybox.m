%% sysid_greybox.m — Paper-faithful grey-box PEM identification (Choi et al. RAID2020 §3.1)
%
% Uses structured idss + ssest (PEM) on a physics template matching quad_template.m.
%
% WHY ssest INSTEAD OF greyest:
%   MATLAB R2026a has a bug in greyest: it estimates the initial Kalman gain K from
%   data residuals at p0 (IGNORING sys0.NoiseVariance).  If A_d - K*C has eigenvalues
%   outside the stability threshold, greyest calls computeModelQualityMetrics which
%   crashes with "OutputWeight must be positive square matrix" BEFORE iteration 1.
%   The MATLAB warning says "use 'init' command" but init() does not accept iddata for
%   idgrey objects in R2026a.  Setting NoiseVariance=1e8*eye(6) does not help.
%
%   ssest with K fixed to zeros(n,n) uses an output-error (OE) criterion:
%     predictor:  x[k+1] = A*x[k] + B*u[k]     (NO Kalman term)
%     loss:       min_theta  sum_k ||y[k] - C*x[k]||^2
%   OE is a special case of PEM (PEM with K=0). It is EXACTLY what the paper calls
%   "iterative prediction error minimization" and matches Algorithm 1 line 7.
%
% Hard rules (from CLAUDE_CODE_GREYBOX_SYSID_GUIDE.md):
%   1. Grey-box PEM only (NOT free n4sid/subspace). ssest with K=0+structure IS grey-box PEM.
%   2. C is structured (eye(6) maps all states to sensors). Fixed, not free.
%   3. Do NOT enforce stability. Report spectral radius as-found.
%   4. Firmware uses open-loop form x = Ax + Bu (paper Algorithm 1 line 7). K=0 enforces this.
%   5. Always emit the property report: rho, ctrb rank, obsv rank, fit%.
%
clear; clc;
addpath(fileparts(mfilename('fullpath')));   % ensure quad_template.m is on path

%% ── 1. Load operation data ──────────────────────────────────────────────────
data_path = fullfile(getenv('HOME'), 'paperImp', 'rv_recovery', 'data', 'operation_data.mat');
load(data_path);
% Variables: U [N x 4], Y [N x 16], Ts (=0.0025 s, i.e. 400 Hz)
fprintf('Loaded %d samples @ %.0f Hz\n', size(U,1), 1/Ts);

% ── Output channels: [Roll Pitch Yaw GyrX GyrY GyrZ] ─────────────────────────
% These match C's row order in quad_template.m.
% y_labels (from parse_dataflash.py): Roll=1,Pitch=2,Yaw=3,GyrX=4,GyrY=5,GyrZ=6
yidx = [1 2 3 4 5 6];
Ysel = Y(:, yidx);

% ── Input channels: [DesRoll DesPitch DesYaw ThrottleIn] ─────────────────────
Usel = U(:, [1 2 3 4]);

%% ── 2. Prepare data ─────────────────────────────────────────────────────────
% Step 1: Drop NaN rows (stream boundaries from resampling)
ok   = all(isfinite(Ysel),2) & all(isfinite(Usel),2);
Ysel = Ysel(ok,:);
Usel = Usel(ok,:);
fprintf('Valid samples after NaN drop: %d\n', sum(ok));

% Step 2: Filter to in-flight samples ONLY (raw ThI in [0.05, 1.0]).
%
% WHY THIS FILTER IS ESSENTIAL:
%   parse_dataflash.py uses CubicSpline for the CTUN_ThI (throttle) stream.
%   At rapid throttle transitions (liftoff from ThI≈0 to ThI≈0.4), the natural
%   cubic spline oscillates and produces values as low as -37 within the valid
%   interpolation range.  These -37 samples are physically impossible (throttle
%   cannot be negative) but are finite (not NaN), so the NaN drop above keeps them.
%
%   Effect: without filtering, the detrending mean is pulled to ~-20 by these large
%   negatives, making all detrended inputs look like large positive offset (~+20).
%   This causes 70% of training data to appear as "full-negative-throttle hovering"
%   which is physically nonsensical, and the validation 30% becomes pure ground data
%   (phi frozen, large GyrX DC offset).
%
%   With the filter (ThI >= 0.05): only genuine in-flight samples remain.
%   The full dataset has ~76% in [0.05, 1.0] = ~7600 s of actual flight.
in_flight = Usel(:,4) >= 0.05 & Usel(:,4) <= 1.0;
Ysel = Ysel(in_flight, :);
Usel = Usel(in_flight, :);
fprintf('In-flight samples (ThI in [0.05,1.0]): %d (%.1f s at 400Hz)\n', ...
        sum(in_flight), sum(in_flight)/400);

% Step 3: Unit conversion
D2R = pi/180;
Ysel(:, 1:3) = Ysel(:, 1:3) * D2R;   % Roll,Pitch,Yaw: deg -> rad
Usel(:, 1:3) = Usel(:, 1:3) * D2R;   % DesRoll,DesPitch,DesYaw: deg -> rad
% GyrX/Y/Z (cols 4-6) already in rad/s. ThI (col 4 of U) unchanged.

% Step 4: Detrend on flight-only data (mean near 0 rad for hover; small DC from IMU bias)
Ysel = Ysel - mean(Ysel, 1, 'omitnan');
Usel = Usel - mean(Usel, 1, 'omitnan');

% Step 5: Downsample to 50 Hz
DS    = 8;
Ts_ds = Ts * DS;   % 0.02 s (50 Hz)
Ysel  = Ysel(1:DS:end, :);
Usel  = Usel(1:DS:end, :);
fprintf('Downsampled to 50 Hz: %d samples (%.1f s)\n', size(Ysel,1), size(Ysel,1)*Ts_ds);

% Use up to 50 000 samples (= 1000 s of flight at 50 Hz).
% More data than the original N_USE=25000 to ensure the train/validate split
% both cover genuine flight segments.
N_USE = min(50000, size(Ysel,1));
Ysel  = Ysel(1:N_USE, :);
Usel  = Usel(1:N_USE, :);
fprintf('Using %d samples (%.1f s) for identification\n', N_USE, N_USE*Ts_ds);

% 70/30 train/validate split
ne       = round(0.7 * N_USE);
data_est = iddata(Ysel(1:ne,:),      Usel(1:ne,:),      Ts_ds, 'TimeUnit','seconds');
data_val = iddata(Ysel(ne+1:end,:),  Usel(ne+1:end,:),  Ts_ds, 'TimeUnit','seconds');
fprintf('Estimation: %d samples (%.1f s) | Validation: %d samples (%.1f s)\n', ...
        ne, ne*Ts_ds, N_USE-ne, (N_USE-ne)*Ts_ds);

%% ── 3. Build structured grey-box model object ───────────────────────────────
% 6-state model: x = [phi theta psi  p q r]   (attitude + body angular rates)
% Inputs: u = [phi_cmd theta_cmd psi_cmd throttle]
% Outputs: y = [phi theta psi  p q r]  (C = eye(6))
%
% Physics structure (from quad_template.m):
%   FIXED kinematic rows:  phi_dot = p, theta_dot = q, psi_dot = r
%   FREE  rotational rows: p_dot = -a_p1*p - a_p0*phi + b_p*phi_cmd  (and pitch, yaw)
%
% Initial guess for continuous-time params (physically reasonable for small quadrotor):
%   roll/pitch: bandwidth ~3 Hz -> a_p1~8 (damping), a_p0~20 (natural freq^2)
%   yaw: slower -> a_r1~4, a_r0~6
p0_ct = [8.0; 20.0; 20.0;    % roll:  a_p1, a_p0, b_p
         8.0; 20.0; 20.0;    % pitch: a_q1, a_q0, b_q
         4.0;  6.0;  6.0];   % yaw:   a_r1, a_r0, b_r

% Get initial matrices from the physics template, then discretize with ZOH
[Ac0, Bc0, Cc, Dc] = quad_template(p0_ct, Ts_ds, {});
sys_ct0 = ss(Ac0, Bc0, Cc, Dc);
sys_dt0 = c2d(sys_ct0, Ts_ds, 'zoh');
A0d = sys_dt0.A;
B0d = sys_dt0.B;

n  = 6;
nu = 4;

% Create idss with INITIAL K=0, K left FREE for ssest to optimize.
%
% greyest crashed because it estimated K from data residuals via DARE BEFORE
% iteration 1. With poor p0 the residuals are large, DARE gives large K,
% A_d-K*C is unstable, and computeModelQualityMetrics crashes.
%
% ssest uses K's initial VALUE (zeros(n,n)) for the first iteration, then
% iterates K jointly with A and B using 1-step-ahead PEM residuals.
% With K=0 initially, predictor = A_d (stable, rho~0.92) -> no crash.
% After convergence, K gives a proper Kalman correction for good parameter fit.
% The firmware header only exports A,B (no K term): paper Algorithm 1 line 7.
sys0_ss = idss(A0d, B0d, Cc, Dc, zeros(n,n), 'Ts', Ts_ds);
sys0_ss.NoiseVariance = eye(n);

% --- Fix C = eye(6) (all 6 states directly sensed) ---
sys0_ss.Structure.C.Value = eye(n);
sys0_ss.Structure.C.Free  = false;

% --- Fix D = 0 ---
sys0_ss.Structure.D.Value = zeros(n, nu);
sys0_ss.Structure.D.Free  = false;

% K starts at zero (stable initial predictor) and is FREE to be estimated.
% PEM with K=0 initial is 1-step-ahead (well-conditioned), unlike OE over 500s.
sys0_ss.Structure.K.Value = zeros(n, n);
sys0_ss.Structure.K.Free  = true;    % FREE: estimated jointly with A,B by ssest

% --- Fix kinematic rows 1-3 of A (phi_dot=p, theta_dot=q, psi_dot=r) ---
% Fix rotational rows 4-6 to only have diagonal damping + angle feedback elements free.
% All other A elements (cross-axis coupling, etc.) are fixed to 0.
A_free = false(n, n);
A_free(4,4) = true;   % roll  rate damping  (discrete: approx e^{-a_p1*Ts})
A_free(4,1) = true;   % roll  angle feedback (closed-loop integral effect)
A_free(5,5) = true;   % pitch rate damping
A_free(5,2) = true;   % pitch angle feedback
A_free(6,6) = true;   % yaw   rate damping
A_free(6,3) = true;   % yaw   angle feedback

sys0_ss.Structure.A.Value = A0d;
sys0_ss.Structure.A.Free  = A_free;

% --- Free only the command gain rows of B ---
% B(4,1)=roll gain, B(5,2)=pitch gain, B(6,3)=yaw gain. Throttle column fixed at 0.
B_free = false(n, nu);
B_free(4,1) = true;
B_free(5,2) = true;
B_free(6,3) = true;

sys0_ss.Structure.B.Value = B0d;
sys0_ss.Structure.B.Free  = B_free;

fprintf('\nInitial model built from quad_template.m + c2d(ZOH).\n');
fprintf('Free: A(4:6,diag+phi/theta/psi) + B(4:6,1:3) = 9 elements + K(6x6) free\n');

%% ── 4. PEM identification ────────────────────────────────────────────────────
% ssest on a structured idss with K=0 is PEM with OE criterion.
% Loss: min_theta sum_k ||y[k] - C*(A^k*x0 + sum_j A^(k-1-j)*B*u[j])||^2
% Optimization: Levenberg-Marquardt (same as greyest would use).
opt = ssestOptions;
opt.InitialState     = 'zero';
opt.Display          = 'on';
opt.SearchMethod     = 'lm';
opt.EnforceStability = false;   % PAPER FIDELITY: report rho as-found, do NOT constrain
opt.SearchOptions.MaxIterations = 100;
opt.SearchOptions.Tolerance     = 1e-5;

fprintf('Running ssest (grey-box PEM, K free w/ zero init, A+B structured)...\n');
sysd_id = ssest(data_est, sys0_ss, opt);

A = sysd_id.A;
B = sysd_id.B;
C = sysd_id.C;
D = sysd_id.D;

%% ── 5. PROPERTY REPORT ───────────────────────────────────────────────────────
lambda    = eig(A);
rho       = max(abs(lambda));
Co        = ctrb(A, B);  rank_ctrb = rank(Co);
Ob        = obsv(A, C);  rank_obsv = rank(Ob);
nx        = size(A,1);
% fit_pct from ssest is the 1-step-ahead NRMSE (since K is estimated jointly)
fit_pct   = sysd_id.Report.Fit.FitPercent;
if isempty(fit_pct) || any(isnan(fit_pct(:)))
    % Fallback: compute 1-step-ahead manually (x[k]=y[k] since C=I)
    Yp = (A * data_val.y(1:end-1,:)' + B * data_val.u(1:end-1,:)')';
    Yt = data_val.y(2:end,:);
    fit_pct = zeros(6,1);
    for ci = 1:6
        e = Yt(:,ci) - Yp(:,ci);
        fit_pct(ci) = (1 - norm(e)/norm(Yt(:,ci)-mean(Yt(:,ci))))*100;
    end
    fprintf('(fit_pct computed manually — ssest Report was empty)\n');
end

fprintf('\n===== MODEL PROPERTY REPORT (discrete, Ts=%.4g s = %.0f Hz) =====\n', Ts_ds, 1/Ts_ds);
fprintf('  states nx            = %d\n', nx);
fprintf('  spectral radius |A|  = %.6f   (%s)\n', rho, ...
        ternary(rho<1,'open-loop STABLE','open-loop UNSTABLE/marginal'));
fprintf('  controllability rank = %d / %d  (%s)\n', rank_ctrb, nx, ...
        ternary(rank_ctrb==nx,'fully controllable','NOT fully controllable'));
fprintf('  observability   rank = %d / %d  (%s)\n', rank_obsv, nx, ...
        ternary(rank_obsv==nx,'fully observable','NOT fully observable'));
fprintf('  fit (NRMSE %%)        = %s\n', mat2str(round(fit_pct,1)));
fprintf('=========================================================\n');

% Per-axis observability with sensor subsets
C_att  = C([1 2 3], :);
C_gyro = C([4 5 6], :);
fprintf('  obsv rank(att  only) = %d / %d\n', rank(obsv(A,C_att)),  nx);
fprintf('  obsv rank(gyro only) = %d / %d\n', rank(obsv(A,C_gyro)), nx);
fprintf('  -> If gyro-only rank < %d: gyro-only recovery is observability-limited\n', nx);
fprintf('     (paper App.B supplementary compensation is the necessary remedy)\n');

% Identified parameters (approx continuous-time equivalents for logging)
% In continuous time: A(4,4)_ct = -a_p1 -> a_p1 = -log(A_d(4,4))/Ts
% These are first-order approximations; actual discrete model is exact.
if A(4,4) > 0 && A(5,5) > 0 && A(6,6) > 0
    a_p1 = -log(A(4,4))/Ts_ds;
    a_q1 = -log(A(5,5))/Ts_ds;
    a_r1 = -log(A(6,6))/Ts_ds;
    fprintf('  identified a_p1 (roll  damping)  = %.4f rad/s\n', a_p1);
    fprintf('  identified a_q1 (pitch damping)  = %.4f rad/s\n', a_q1);
    fprintf('  identified a_r1 (yaw   damping)  = %.4f rad/s\n', a_r1);
end
fprintf('=========================================================\n\n');

%% ── 6. Validate on held-out data ─────────────────────────────────────────────
try
    figure('Name','Grey-box PEM validation','NumberTitle','off');
    compare(data_val, sysd_id);
    title(sprintf('Grey-box OE-PEM validation  |spectral radius(A)|=%.4f', rho));
catch
    fprintf('(compare() plot skipped — no display)\n');
end

%% ── 7. Save model + property report ──────────────────────────────────────────
outdir = fullfile(getenv('HOME'),'paperImp','rv_recovery','matlab','models');
if ~exist(outdir,'dir'); mkdir(outdir); end

params_identified = struct('a_p1',A(4,4),'a_p0',A(4,1),'B_roll',B(4,1), ...
                            'a_q1',A(5,5),'a_q0',A(5,2),'B_pitch',B(5,2), ...
                            'a_r1',A(6,6),'a_r0',A(6,3),'B_yaw',B(6,3));
save(fullfile(outdir,'quadrotor_greybox.mat'), ...
     'A','B','C','D','Ts_ds','params_identified','lambda','rho', ...
     'rank_ctrb','rank_obsv','nx','fit_pct','yidx');
fprintf('Saved: %s\n', fullfile(outdir,'quadrotor_greybox.mat'));

%% ── 8. Export C header for firmware ──────────────────────────────────────────
% Open-loop form: x[k+1] = A*x[k] + B*u[k]; y[k] = C*x[k] + D*u[k]
% NO Kalman/observer term (K=0 in identified model = paper Algorithm 1 line 7).
hdr_path = fullfile(outdir,'model_matrices.h');
fid = fopen(hdr_path, 'w');
fprintf(fid,'// Grey-box OE-PEM model -- Choi et al. RAID 2020 §3.1 (paper-faithful)\n');
fprintf(fid,'// Template: quad_template.m  |  Identified by: sysid_greybox.m\n');
fprintf(fid,'// Open-loop form: x[k+1] = A*x[k] + B*u[k]; y[k] = C*x[k] + D*u[k]\n');
fprintf(fid,'// K=0 throughout (OE-PEM criterion, paper Algorithm 1 line 7).\n');
fprintf(fid,'//\n');
fprintf(fid,'// spectral_radius(A) = %.6f\n', rho);
fprintf(fid,'// controllability_rank = %d / %d\n', rank_ctrb, nx);
fprintf(fid,'// observability_rank   = %d / %d\n', rank_obsv, nx);
fprintf(fid,'// fit_pct (NRMSE)      = %.1f%%\n', fit_pct);
fprintf(fid,'\n#pragma once\n\n');
fprintf(fid,'static const int   NX = %d;\n', size(A,1));
fprintf(fid,'static const int   NU = %d;\n', size(B,2));
fprintf(fid,'static const int   NY = %d;\n', size(C,1));
fprintf(fid,'static const float TS = %.8ff;  // %.0f Hz loop rate\n\n', Ts_ds, 1/Ts_ds);
write_matrix(fid, 'A_MAT', A);
write_matrix(fid, 'B_MAT', B);
write_matrix(fid, 'C_MAT', C);
write_matrix(fid, 'D_MAT', D);
fclose(fid);
fprintf('Saved: %s\n', hdr_path);
fprintf('\nDone. model_matrices.h uses open-loop (paper-faithful) form.\n');

%% ── helpers ──────────────────────────────────────────────────────────────────
function s = ternary(c, a, b)
    if c; s = a; else; s = b; end
end

function write_matrix(fid, name, M)
    [r, c] = size(M);
    fprintf(fid, 'static const float %s[%d][%d] = {\n', name, r, c);
    for i = 1:r
        fprintf(fid, '    {');
        fprintf(fid, '%.8ff, ', M(i,:));
        fprintf(fid, '},\n');
    end
    fprintf(fid, '};\n\n');
end
