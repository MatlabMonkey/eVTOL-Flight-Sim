% Design_LQR_Controller_EVTOL_Cruise.m
%
% LQR design for the EVTOL cruise trim.
%
% Expected reduced model from the trim script:
%   sys_ss_13state states =
%   [phi theta psi u v w P Q R dLW dRW dLT dRT]
%
% The LQR itself is built on the rigid-body states, while the four actuator
% position states are folded into effective control channels.

fprintf('\n=== Cruise LQR Controller Design ===\n');

%% Sanity checks
if ~exist('sys_ss_13state', 'var')
    error(['sys_ss_13state not found. Update the trim script first so it builds ', ...
           'the 13-state reduced model before calling this file.']);
end

if size(sys_ss_13state.A,1) ~= 13
    error('sys_ss_13state must be 13x13. Current size is %dx%d.', ...
        size(sys_ss_13state.A,1), size(sys_ss_13state.A,2));
end

%% Pull rigid-body and actuator blocks from the 13-state model
% sys_ss_13state order must be:
% [phi theta psi u v w P Q R dLW dRW dLT dRT]
rb9  = 1:9;
act4 = 10:13;

A_13 = sys_ss_13state.A;
B_13 = sys_ss_13state.B;

% Rigid-body dynamics
A_lqr = A_13(rb9, rb9);

% Effective control matrix:
%   column 1 = direct front collective RPM input
%   columns 2:5 = actuator-state influence on rigid-body dynamics
B_lqr = [B_13(rb9,1), A_13(rb9,act4)];

fprintf('Open-loop rigid-body eigenvalues:\n');
ev = eig(A_lqr);
for i = 1:numel(ev)
    fprintf('  %+10.4f %+10.4fi\n', real(ev(i)), imag(ev(i)));
end

ctrb_rank_9 = rank(ctrb(A_lqr, B_lqr));
fprintf('Controllability rank (9-state) : %d / 9\n', ctrb_rank_9);

%% Decide whether to use full 9-state LQR or 8-state inner-loop LQR
% If psi is not controllable enough in this cruise linearization, treat psi
% as an outer-loop variable and design the inner-loop LQR on:
% [phi theta u v w P Q R]
use_full_9state = (ctrb_rank_9 == 9);

if use_full_9state
    fprintf('Using full 9-state LQR: [phi theta psi u v w P Q R]\n');

    keep_idx = 1:9;

    %            phi theta psi  u  v  w  P  Q  R
    Q_use = diag([  8,   10,   1,  1, 1, 3, 6, 8, 6 ]);

else
    fprintf(['9-state pair is not fully controllable.\n', ...
             'Falling back to 8-state inner-loop LQR with psi excluded:\n', ...
             '  [phi theta u v w P Q R]\n']);

    % keep phi, theta, u, v, w, P, Q, R
    keep_idx = [1 2 4 5 6 7 8 9];

    A_test = A_lqr(keep_idx, keep_idx);
    B_test = B_lqr(keep_idx, :);

    ctrb_rank_8 = rank(ctrb(A_test, B_test));
    fprintf('Controllability rank (8-state) : %d / 8\n', ctrb_rank_8);

    if ctrb_rank_8 < 8
        error(['Even the 8-state inner-loop model is not fully controllable. ', ...
               'The trim script reduction is still inconsistent, or the plant ', ...
               'channels are not mapped the way the controller expects.']);
    end

    %          phi theta  u  v  w  P  Q  R
    Q_use = diag([  8,   10,  1, 1, 3, 6, 8, 6 ]);
end

A_use = A_lqr(keep_idx, keep_idx);
B_use = B_lqr(keep_idx, :);

%% Control penalty
% front collective is in RPM, surfaces are in radians, so keep the first
% channel much cheaper than the surface channels.
%          fRPM   dLW  dRW  dLT  dRT
R_use = diag([1e-4,  3,   3,   3,   3]);

%% Solve LQR
[K_use, S_lqr, cl_eigs] = lqr(A_use, B_use, Q_use, R_use);

% Expand back to 5x9 so the wrapper/controller interface stays unchanged.
K_lqr = zeros(5, 9);
K_lqr(:, keep_idx) = K_use;

K_lqr_cruise = K_lqr;

fprintf('\nK_lqr_cruise (5 x 9):\n');
disp(K_lqr_cruise);

fprintf('Closed-loop eigenvalues of designed subsystem:\n');
for i = 1:numel(cl_eigs)
    fprintf('  %+10.4f %+10.4fi\n', real(cl_eigs(i)), imag(cl_eigs(i)));
end

if any(real(cl_eigs) > 1e-6)
    warning('Designed closed-loop subsystem still has unstable mode(s).');
else
    fprintf('Designed closed-loop subsystem is stable.\n');
end

if ~use_full_9state
    fprintf(['NOTE: psi is excluded from the inner-loop LQR.\n', ...
             'Its gain column is intentionally zero, so heading should be\n', ...
             'handled by an outer loop later.\n']);
end

%% Spot checks
fprintf('\nSaturation spot-check (surface limit = +/-%.0f deg):\n', surface_limit_deg);

x_phi = zeros(9,1);
x_phi(1) = 10*pi/180;
u_phi = -K_lqr * x_phi;

x_theta = zeros(9,1);
x_theta(2) = 5*pi/180;
u_theta = -K_lqr * x_theta;

x_psi = zeros(9,1);
x_psi(3) = 10*pi/180;
u_psi = -K_lqr * x_psi;

fmt = '  %-14s  fRPM=%+8.1f   dLW=%+7.2f  dRW=%+7.2f  dLT=%+7.2f  dRT=%+7.2f  deg\n';
fprintf(fmt, '10 deg roll :',  u_phi(1),   u_phi(2)*180/pi,   u_phi(3)*180/pi,   u_phi(4)*180/pi,   u_phi(5)*180/pi);
fprintf(fmt, ' 5 deg pitch:',  u_theta(1), u_theta(2)*180/pi, u_theta(3)*180/pi, u_theta(4)*180/pi, u_theta(5)*180/pi);
fprintf(fmt, '10 deg yaw  :',  u_psi(1),   u_psi(2)*180/pi,   u_psi(3)*180/pi,   u_psi(4)*180/pi,   u_psi(5)*180/pi);

surface_mags_deg = abs([u_phi(2:5); u_theta(2:5); u_psi(2:5)]) * 180/pi;
max_surface = max(surface_mags_deg);
fprintf('  Max |surface| command across probes: %.2f deg\n', max_surface);

if max_surface > 0.9 * surface_limit_deg
    fprintf('  WARNING: within 10%% of surface saturation.\n');
else
    fprintf('  OK: well within surface limits.\n');
end

rpm_mags = abs([u_phi(1); u_theta(1); u_psi(1)]);
fprintf('  Max |front_coll| delta across probes: %.1f rpm\n', max(rpm_mags));

%% Trim references for the wrapper
% Keep the 9-state interface expected by the wrapper:
% [phi theta psi u v w P Q R]
x_trim_lqr = [Att_Trim; Vel_B_BA_Trim; Rates_Trim];
U_trim_lqr = U_trim(:);

fprintf('\nx_trim_lqr = [phi theta psi u v w P Q R]:\n');
disp(x_trim_lqr.');

fprintf('U_trim_lqr = [front_coll_rpm dLW dRW dLT dRT]:\n');
disp(U_trim_lqr.');

fprintf('=== LQR Design Complete ===\n\n');