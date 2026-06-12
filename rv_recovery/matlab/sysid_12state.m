%% sysid_12state.m — Paper-faithful system identification (Choi et al. RAID 2020 §3.1)
%
% Paper §3.1:
%   "For each variable, we first determine the state and output template equations.
%    Specifically, we use a discrete-time state-space model template, encoding a PID
%    controller and dynamics equations known a priori ... Then, for each variable, we
%    specify a model order ... We then employ SI to instantiate the unknown coefficients
%    of the template using an iterative prediction error minimization algorithm."
%   "the linear-model is sufficient ... the dominating system dynamic is a second-order
%    system"
%
% Eq. (3) state vector: x = [x y z phi theta psi xd yd zd p q r]
%   ours (same, named):   [pN pE alt phi theta psi vN vE vUp p q r]
%
% Implementation: six per-axis SECOND-ORDER blocks (each = one "variable pair"
% [value, derivative], both directly measured -> C = I pins the states physically,
% which is required by the §3.3 windowed synchronization that re-seeds model states
% from real readings):
%   roll : [phi,  p ]  <- phi_cmd
%   pitch: [theta,q ]  <- theta_cmd
%   yaw  : [psi,  r ]  <- psi_cmd          (unwrapped yaw)
%   N    : [pN,   vN]  <- tiltN            (frame-canonicalized cmd, Appendix A)
%   E    : [pE,   vE]  <- tiltE
%   U    : [alt, vUp]  <- [thr, const]     (const = gravity/hover equilibrium term)
%
% Each block: discrete-time state-space template of order 2, C=eye fixed, D=0 fixed,
% K=0 FIXED (paper Algorithm 1 line 7 is open-loop: x <- Ax + Bu; no observer term).
% ssest with K fixed at zero == output-error PEM, which IS "iterative prediction
% error minimization". NO detrending (runtime feeds raw signals).
%
% Blocks are assembled block-diagonally into the 12-state A,B,C,D of Eq. (1)-(2)
% and exported to model_matrices.h for the firmware monitor.
%
% Sample rate: 50 Hz (Ts=0.02) — the one sanctioned deviation from the paper's 400 Hz.

clear; clc;

%% ── 1. Load per-segment operation data ──────────────────────────────────────
data_path = fullfile(getenv('HOME'),'paperImp','rv_recovery','data','operation_data_50hz.mat');
S = load(data_path);
Ts = double(S.Ts);
nseg = numel(S.Useg);
fprintf('Loaded %d segment(s), Ts=%.3f s (%.0f Hz)\n', nseg, Ts, 1/Ts);

% Eq.(3) output order produced by parse_dataflash.py:
%   Y: [pN pE alt phi theta psi vN vE vUp p q r],  U: [phic thetac psic thr tiltN tiltE 1]
Us = cell(nseg,1); Ys = cell(nseg,1);
for i = 1:nseg
    Us{i} = double(S.Useg{i});
    Ys{i} = double(S.Yseg{i});
    fprintf('  segment %d: %d samples (%.0f s)\n', i, size(Us{i},1), size(Us{i},1)*Ts);
end

% 70/30 estimation/validation split within each segment (time-ordered)
Ue = cell(nseg,1); Ye = cell(nseg,1); Uv = cell(nseg,1); Yv = cell(nseg,1);
for i = 1:nseg
    n  = size(Us{i},1); ne = round(0.7*n);
    Ue{i} = Us{i}(1:ne,:);     Ye{i} = Ys{i}(1:ne,:);
    Uv{i} = Us{i}(ne+1:end,:); Yv{i} = Ys{i}(ne+1:end,:);
end

%% ── 2. Per-axis block definitions ───────────────────────────────────────────
% yidx: [value, derivative] columns in Y;  uidx: input columns in U
blocks = struct( ...
  'name', {'roll','pitch','yaw','N','E','U'}, ...
  'yidx', {[4 10], [5 11], [6 12], [1 7], [2 8], [3 9]}, ...
  'uidx', {1, 2, 3, 5, 6, [4 7]});

% initial guesses: 2nd-order closed-loop, bandwidth a few rad/s
init_ct = struct( ...
  'roll',  struct('A',[0 1; -20 -8],  'B',[0; 20]), ...
  'pitch', struct('A',[0 1; -20 -8],  'B',[0; 20]), ...
  'yaw',   struct('A',[0 1; -6  -4],  'B',[0; 6]), ...
  'N',     struct('A',[0 1;  0  -1],  'B',[0; 5]), ...
  'E',     struct('A',[0 1;  0  -1],  'B',[0; 5]), ...
  'U',     struct('A',[0 1;  0  -2],  'B',[0 0; 15 -5]));

opt = ssestOptions;
opt.InitialState     = 'estimate';     % x0 per experiment; not a Kalman term
opt.EnforceStability = false;          % report dynamics as found (integrators = 1.0)
opt.SearchMethod     = 'lm';
opt.Display          = 'off';
opt.SearchOptions.MaxIterations = 80;

models = struct();
fitrep = struct();
for b = 1:numel(blocks)
    nm   = blocks(b).name;
    yix  = blocks(b).yidx;
    uix  = blocks(b).uidx;
    nu_b = numel(uix);

    % multi-experiment iddata
    de = iddata(cellfun(@(Y) Y(:,yix), Ye, 'uni',0), ...
                cellfun(@(U) U(:,uix), Ue, 'uni',0), Ts);
    dv = iddata(cellfun(@(Y) Y(:,yix), Yv, 'uni',0), ...
                cellfun(@(U) U(:,uix), Uv, 'uni',0), Ts);

    % discrete initial template via ZOH of the continuous-time guess
    g  = init_ct.(nm);
    sd = c2d(ss(g.A, g.B, eye(2), zeros(2,nu_b)), Ts, 'zoh');

    % structured idss: order 2, C=I and D=0 FIXED (states pinned to outputs),
    % K=0 FIXED (open-loop OE-PEM, Algorithm 1 line 7), A and B free.
    m0 = idss(sd.A, sd.B, eye(2), zeros(2,nu_b), zeros(2,2), 'Ts', Ts);
    m0.Structure.C.Free = false;
    m0.Structure.D.Free = false;
    m0.Structure.K.Value = zeros(2,2);
    m0.Structure.K.Free  = false;

    fprintf('Identifying block %-5s (u: %s) ... ', nm, mat2str(uix));
    m = ssest(de, m0, opt);

    % open-loop simulation fit on held-out data (what the firmware actually runs)
    copt = compareOptions('InitialCondition','e');
    [~, fitv] = compare(dv, m, Inf, copt);
    if iscell(fitv)
        fmat = cell2mat(cellfun(@(c) c(:)', fitv(:), 'uni', 0));  % exp x outputs
    else
        fmat = reshape(fitv, [], 2);
    end
    fprintf('val sim-fit %% (per output, mean over exp) = %s\n', ...
            mat2str(round(mean(fmat, 1), 1)));
    fitv = fmat;

    models.(nm) = m;
    fitrep.(nm) = fitv;
end

%% ── 3. Assemble the 12-state Eq. (1)-(2) model ──────────────────────────────
% state order (Eq. 3): [pN pE alt phi theta psi vN vE vUp p q r]
% input order:         [phic thetac psic thr tiltN tiltE const]
NX = 12; NU = 7; NY = 12;
A = zeros(NX); B = zeros(NX,NU); C = eye(NY,NX); D = zeros(NY,NU);

% block state rows in the big vector: [value_row, deriv_row]
rows = struct('roll',[4 10], 'pitch',[5 11], 'yaw',[6 12], ...
              'N',[1 7], 'E',[2 8], 'U',[3 9]);
ucols = struct('roll',1, 'pitch',2, 'yaw',3, 'N',5, 'E',6, 'U',[4 7]);

fn = fieldnames(rows);
for k = 1:numel(fn)
    nm = fn{k};
    r2 = rows.(nm); uc = ucols.(nm);
    Ab = models.(nm).A;  Bb = models.(nm).B;
    A(r2, r2) = Ab;
    B(r2, uc) = Bb;
end

lambda = eig(A); rho = max(abs(lambda));
fprintf('\n===== 12-STATE MODEL (Ts=%.3f s) =====\n', Ts);
fprintf('  spectral radius = %.6f (integrator states expected ~1.0)\n', rho);
fprintf('  paper Eq.(3) order: [pN pE alt phi theta psi vN vE vUp p q r]\n');

%% ── 4. Save model + per-block fit report ────────────────────────────────────
outdir = fullfile(getenv('HOME'),'paperImp','rv_recovery','matlab','models');
if ~exist(outdir,'dir'); mkdir(outdir); end
save(fullfile(outdir,'quadrotor_12state.mat'), 'A','B','C','D','Ts','models','fitrep');
fprintf('Saved: %s\n', fullfile(outdir,'quadrotor_12state.mat'));

%% ── 5. Export C header for the firmware monitor ─────────────────────────────
hdr = fullfile(outdir,'model_matrices.h');
fid = fopen(hdr,'w');
fprintf(fid,'// 12-state model — Choi et al. RAID 2020 Eq.(1)-(3), identified by sysid_12state.m\n');
fprintf(fid,'// Per-variable 2nd-order templates (PEM, K=0), block-diagonal assembly.\n');
fprintf(fid,'// State order: [pN pE alt phi theta psi vN vE vUp p q r]\n');
fprintf(fid,'// Input order: [phi_cmd theta_cmd psi_cmd thr tiltN tiltE const]\n');
fprintf(fid,'// tiltN = (-theta_cmd)*cos(psi) - phi_cmd*sin(psi)   (Appendix A frame canon.)\n');
fprintf(fid,'// tiltE = (-theta_cmd)*sin(psi) + phi_cmd*cos(psi)\n');
fprintf(fid,'// spectral_radius = %.6f\n', rho);
fprintf(fid,'\n#pragma once\n\n');
fprintf(fid,'static const int   NX = %d;\n', NX);
fprintf(fid,'static const int   NU = %d;\n', NU);
fprintf(fid,'static const int   NY = %d;\n', NY);
fprintf(fid,'static const float TS = %.8ff;  // %.0f Hz\n\n', Ts, 1/Ts);
wm(fid,'A_MAT',A); wm(fid,'B_MAT',B); wm(fid,'C_MAT',C); wm(fid,'D_MAT',D);
fclose(fid);
fprintf('Saved: %s\n', hdr);

function wm(fid, name, M)
    [r,c] = size(M);
    fprintf(fid,'static const float %s[%d][%d] = {\n', name, r, c);
    for i = 1:r
        fprintf(fid,'    {');
        fprintf(fid,'%.8ff, ', M(i,:));
        fprintf(fid,'},\n');
    end
    fprintf(fid,'};\n\n');
end
