function [A,B,C,D] = quad_template(params, Ts, aux)
% QUAD_TEMPLATE  Grey-box state-space template — attitude subsystem (Choi et al. RAID2020 §3.1).
%
% 6-state model: x = [phi theta psi  p q r]   (attitude + body angular rates)
% Input:         u = [phi_cmd theta_cmd psi_cmd throttle]  (4 inputs, throttle not used here)
% Output:        y = [phi theta psi  p q r]               (all 6 states directly sensed)
%
% WHY 6 STATES (not 12):
%   The paper §3.1 says "for each variable, we specify a model order" — different sensors
%   get different model orders. For the attitude/gyro recovery experiment we need only the
%   attitude + rate states. The position/velocity states (x,y,z,xd,yd,zd) are:
%   (a) NOT OBSERVABLE from attitude+gyro outputs alone (obsv rank drops to 6 with those extra states)
%   (b) Their pure integrators give discrete eigenvalues at exactly 1.0, making the initial
%       predictor marginally unstable before greyest even starts, crashing the solver.
%   Reducing to 6 attitude states is MORE faithful for this sensor subset, not less.
%
% FIXED rows (kinematics, "dynamics equations known a priori"):
%   phi_dot   = p     (attitude integrates from body angular rates, small-angle hover)
%   theta_dot = q
%   psi_dot   = r
%
% FREE rows (closed-loop PID dynamics — "unknown coefficients" for PEM to identify):
%   p_dot = -a_p1*p - a_p0*phi + b_p*phi_cmd     roll  axis 2nd-order closed-loop
%   q_dot = -a_q1*q - a_q0*the + b_q*the_cmd     pitch axis 2nd-order closed-loop
%   r_dot = -a_r1*r - a_r0*psi + b_r*psi_cmd     yaw   axis 2nd-order closed-loop
%
% params (9 free coefficients):
%   1: a_p1   roll-rate damping  (bundles Ixx, motor Kf, PID D-term)
%   2: a_p0   roll-angle restoring (PID P-term effect)
%   3: b_p    roll command gain
%   4: a_q1   pitch-rate damping
%   5: a_q0   pitch-angle restoring
%   6: b_q    pitch command gain
%   7: a_r1   yaw-rate damping
%   8: a_r0   yaw-angle restoring
%   9: b_r    yaw command gain
%
% aux: unused (required by idgrey signature).
% Ts: unused (continuous-time model; idgrey discretizes with c2d).

    p = params(:);

    % State indices
    PH=1; TH=2; PS=3; P=4; Q=5; R=6;

    A = zeros(6,6);
    B = zeros(6,4);

    % ===== FIXED kinematic rows =====
    A(PH, P) = 1;    % phi_dot   = p
    A(TH, Q) = 1;    % theta_dot = q
    A(PS, R) = 1;    % psi_dot   = r

    % ===== FREE rotational rows (2nd-order closed-loop PID per axis) =====
    A(P, P)  = -p(1);  A(P, PH) = -p(2);  B(P, 1) = p(3);   % roll
    A(Q, Q)  = -p(4);  A(Q, TH) = -p(5);  B(Q, 2) = p(6);   % pitch
    A(R, R)  = -p(7);  A(R, PS) = -p(8);  B(R, 3) = p(9);   % yaw

    % ===== Output map C: STRUCTURED selection (NOT free) =====
    % All 6 states are directly sensed (attitude = IMU integration, rates = gyro).
    C = eye(6,6);
    D = zeros(6,4);
end
