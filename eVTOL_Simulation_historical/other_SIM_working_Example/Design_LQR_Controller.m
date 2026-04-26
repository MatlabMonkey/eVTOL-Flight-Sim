% Design_LQR_Controller.m
%
% Designs a 9-state LQR regulator.
%
% State vector order
%   x = [phi, theta, psi, u, v, w, P, Q, R]   (9 states, all SI units: rad, m/s, rad/s)
%
% OUTPUT
%   [Ail_cmd, Flap_cmd, Elev_cmd, Rud_cmd]
%   These feed the ControlMixer_A8 block which applies MixMatrix to produce
%   the actual surface positions [dER, dEL, dE, dR].

fprintf('\n=== LQR Controller Design ===\n');

%% State and actuator indices
% states_order = [1:16] (identity) so indices are direct
rb_idx  = states_order(1:9);    % [phi,theta,psi,u,v,w,P,Q,R]
act_idx = states_order(10:13);  % [dER,dEL,dE,dR]

%% Extract reduced-order matrices
% A_lqr: rigid body dynamics
% B_lqr: actuator-to-rigid-body coupling 
A_lqr = sys_struct.a(rb_idx, rb_idx);
B_lqr = sys_struct.a(rb_idx, act_idx);

fprintf('Open-loop rigid body eigenvalues:\n');
ev = eig(A_lqr);
for i = 1:length(ev)
    fprintf('  %+.4f %+.4fi\n', real(ev(i)), imag(ev(i)));
end
fprintf('Controllability rank: %d / 9\n\n', rank(ctrb(A_lqr, B_lqr)));

%% LQR Weights
%              phi  tht  psi   u     v     w     P    Q    R
Q_lqr = diag([ 5,   5,   5,   0.5,  0.5,  0.5,   3,   3,   3  ]);

%              dER dEL dE  dR
R_lqr = diag([7,   7,   7,   7]);

%% Compute LQR gain
[K_lqr, ~, lqr_eigs_rb] = lqr(A_lqr, B_lqr, Q_lqr, R_lqr);

fprintf('K_lqr (surface-position space, 4x9):\n'); disp(K_lqr);
fprintf('Rigid-body closed-loop eigenvalues:\n');
for i = 1:length(lqr_eigs_rb)
    fprintf('  %+.4f %+.4fi\n', real(lqr_eigs_rb(i)), imag(lqr_eigs_rb(i)));
end

if any(real(lqr_eigs_rb) > 1e-6)
    fprintf('WARNING: unstable closed-loop mode. Increase Q or decrease R.\n');
else
    fprintf('All closed-loop eigenvalues stable.\n\n');
end

%% Convert K to functional-command space (account for ControlMixer_A8)
%
% ControlMixer_A8 applies:   surface = MixMatrix * functional_cmd
%   MixMatrix = [-1 1 0 0;   (dER = -Ail + Flap)
%                 1 1 0 0;   (dEL =  Ail + Flap)
%                 0 0 1 0;   (dE  =  Elev)
%                 0 0 0 1]   (dR  =  Rud)

MixMatrix_val    = [-1 1 0 0; 
                    1 1 0 0; 
                    0 0 1 0; 
                    0 0 0 1];
DeMixMatrix_val  = inv(MixMatrix_val);

K_lqr_cmd = DeMixMatrix_val * K_lqr;   % 4x9: [Ail; Flap; Elev; Rud] per state

fprintf('K_lqr_cmd after DeMixMatrix:\n');
disp(K_lqr_cmd);

%% Verify saturation margins for typical perturbations
fprintf('Saturation check (+-25 deg limit):\n');
delta_phi   = 10 * pi/180;    % 10 deg roll perturbation
delta_theta =  5 * pi/180;    % 5 deg pitch perturbation
x_phi   = [delta_phi; zeros(8,1)];
x_theta = [0; delta_theta; zeros(7,1)];

u_phi   = K_lqr_cmd * x_phi   * 180/pi;  % deg
u_theta = K_lqr_cmd * x_theta * 180/pi;  % deg

fprintf('  10 deg roll:  Ail=%.1f, Flap=%.1f, Elev=%.1f, Rud=%.1f deg\n', u_phi(1), u_phi(2), u_phi(3), u_phi(4));
fprintf('  5 deg pitch:  Ail=%.1f, Flap=%.1f, Elev=%.1f, Rud=%.1f deg\n', u_theta(1), u_theta(2), u_theta(3), u_theta(4));
if max(abs([u_phi; u_theta])) > 22
    fprintf('  WARNING: commands near or above 25 deg saturation limit.\n');
    fprintf('           Increase R_lqr or reduce test perturbation size.\n');
else
    fprintf('  All within saturation limits.\n');
end

%% Trim setpoints for Simulink
% x_trim_lqr: matches Mux order [phi, theta, psi, u, v, w, P, Q, R]
x_trim_lqr  = [Att_Trim; Vel_B_BA_Trim; Rates_Trim];   % 9x1, rad and m/s

% U_trim in functional-command space
U_trim_lqr  = U_trim(:);                                % 4x1 surface positions at trim

fprintf('\nx_trim_lqr = [phi;theta;psi;u;v;w;P;Q;R]:\n');
disp(x_trim_lqr');
fprintf('U_trim_lqr = [dER;dEL;dE;dR] (rad):\n');
disp(U_trim_lqr');

fprintf('=== LQR Design Complete ===\n\n');