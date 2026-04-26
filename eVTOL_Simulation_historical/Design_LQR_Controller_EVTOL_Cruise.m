% Design_LQR_Controller_EVTOL_Cruise.m
%
% Designs a 9-state LQR regulator about the cruise trim point.
%
% State vector order
%   x = [phi, theta, psi, u, v, w, P, Q, R]
%
% Input vector order
%   u = [front_collective_rpm, deltaLW, deltaRW, deltaLT, deltaRT]

fprintf('\n=== Cruise LQR Controller Design ===\n');

%% Extract reduced-order matrices
A_lqr = sys_ss_9state.a;
B_lqr = sys_ss_9state.b;

fprintf('Open-loop rigid-body eigenvalues:\n');
ev = eig(A_lqr);
for i = 1:length(ev)
    fprintf('  %+.4f %+.4fi\n', real(ev(i)), imag(ev(i)));
end
fprintf('Controllability rank: %d / 9\n\n', rank(ctrb(A_lqr, B_lqr)));

%% LQR weights
%              phi  tht  psi   u    v    w    P    Q    R
Q_lqr = diag([ 6,   10,  3,    1,   1,   2,   4,   6,   4 ]);

%              frontRPM dLW dRW dLT dRT
R_lqr = diag([ 1e-4,    3,  3,  3,  3 ]);

%% Compute LQR gain
[K_lqr, ~, lqr_eigs_rb] = lqr(A_lqr, B_lqr, Q_lqr, R_lqr);

fprintf('K_lqr (5x9):\n');
disp(K_lqr);

fprintf('Rigid-body closed-loop eigenvalues:\n');
for i = 1:length(lqr_eigs_rb)
    fprintf('  %+.4f %+.4fi\n', real(lqr_eigs_rb(i)), imag(lqr_eigs_rb(i)));
end

if any(real(lqr_eigs_rb) > 1e-6)
    fprintf('WARNING: unstable closed-loop mode.\n');
else
    fprintf('All closed-loop eigenvalues stable.\n');
end

%% Trim set-points for later controller hookup
x_trim_lqr = [Att_Trim; Vel_B_BA_Trim; Rates_Trim];
U_trim_lqr = U_trim(:);

fprintf('\nx_trim_lqr = [phi;theta;psi;u;v;w;P;Q;R]:\n');
disp(x_trim_lqr');
fprintf('U_trim_lqr = [front_collective_rpm; deltaLW; deltaRW; deltaLT; deltaRT]:\n');
disp(U_trim_lqr');

fprintf('=== Cruise LQR Design Complete ===\n\n');
