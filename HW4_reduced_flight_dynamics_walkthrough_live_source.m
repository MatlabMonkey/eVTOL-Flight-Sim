%% Homework 4 Reduced Flight-Dynamics Walkthrough
% This is the maintainable source for HW4_reduced_flight_dynamics_walkthrough.mlx.

clear; clc; close all;
format short g;
addpath(genpath(fullfile(pwd, 'scripts')));

%% User choices
Kq_pick = 0.4;      % pitch damper
Kp_roll_pick = 0.2; % roll damper
Kv_pick = 0.15;     % proportional airspeed gain on u
Ki_pick = 0.02;     % integral airspeed gain

%% Trim and linearize the Euler plant
data = linearize_reduced_flight_dynamics_eul('CruiseSpeed', 70, 'DisplayReport', 'off');

trimSummary = table( ...
    data.trimPoint.inputs.front_collective_rpm, ...
    data.trimPoint.inputs.rear_collective_rpm, ...
    rad2deg(data.trimPoint.inputs.delta_e), ...
    rad2deg(data.trimPoint.eul(2)), ...
    norm(data.trimPoint.velocity), ...
    'VariableNames', {'front_collective_rpm','rear_collective_rpm','delta_e_deg','theta_deg','vinf_mps'})

%% Full model states and grouped inputs
% I am ignoring position states and psi for the control design model.
fullStateNames = string(data.full.StateName(:));
fullInputNames = string(data.full.InputName([19 20 22 23 24]));

fullStateNames
fullInputNames

%% Reduced state choice
% Longitudinal states: [u w q theta]
% Lateral states:      [v p r phi]
longStateTable = table(data.long.state_names.', data.long.state_idx.', ...
    'VariableNames', {'state','full_state_index'})

latStateTable = table(data.lat.state_names.', data.lat.state_idx.', ...
    'VariableNames', {'state','full_state_index'})

%% Reduced longitudinal and lateral matrices
A_long = data.long.A
B_long = data.long.B

A_lat = data.lat.A
B_lat = data.lat.B

longInputTable = table(string(data.long.input_names).', ...
    'VariableNames', {'longitudinal_inputs'})

latInputTable = table(string(data.lat.input_names).', ...
    'VariableNames', {'lateral_inputs'})

%% Coupling check
% These are the cross-coupling blocks I am neglecting for the simplified design.
A_long_from_lat = data.long.coupling_from_lateral
A_lat_from_long = data.lat.coupling_from_longitudinal

%% Open-loop poles of the reduced models
longPoles = eig(A_long);
latPoles = eig(A_lat);

longPoleTable = table(real(longPoles), imag(longPoles), ...
    'VariableNames', {'real_part','imag_part'})

latPoleTable = table(real(latPoles), imag(latPoles), ...
    'VariableNames', {'real_part','imag_part'})

figure('Color', 'w', 'Position', [100 100 1100 440]);
tiledlayout(1,2);

nexttile;
plot(real(longPoles), imag(longPoles), 'x', 'LineWidth', 1.4, 'MarkerSize', 8);
grid on;
xlabel('Real');
ylabel('Imaginary');
title('Longitudinal Poles: [u w q \theta]');

nexttile;
plot(real(latPoles), imag(latPoles), 'x', 'LineWidth', 1.4, 'MarkerSize', 8);
grid on;
xlabel('Real');
ylabel('Imaginary');
title('Lateral Poles: [v p r \phi]');

%% Part A: pure dampers
% For the pure dampers, rate over control is the right transfer function.
Gq = data.transfer.q_over_delta_e;
Gp = data.transfer.p_over_delta_a;
Gr = data.transfer.r_over_delta_r;

pitch_damper_tf = minreal(zpk(Gq))
roll_damper_tf = minreal(zpk(Gp))
yaw_damper_tf = minreal(zpk(Gr))

figure('Color', 'w', 'Position', [100 100 1200 420]);
tiledlayout(1,3);

nexttile;
rlocus(Gq);
grid on;
title('Pitch Damper Root Locus: q / \delta_e');

nexttile;
rlocus(Gp);
grid on;
title('Roll Damper Root Locus: p / \delta_a');

nexttile;
rlocus(Gr);
grid on;
title('Yaw Damper Root Locus: r / \delta_r');

%% Pick pure damper gains
if ~exist('Kq_pick', 'var')
    Kq_pick = 0.4;
end
if ~exist('Kp_roll_pick', 'var')
    Kp_roll_pick = 0.2;
end

A_long_q = A_long - B_long(:, 3) * Kq_pick * [0 0 1 0];
A_lat_p = A_lat - B_lat(:, 1) * Kp_roll_pick * [0 1 0 0];

longQPoles = eig(A_long_q);
latPPoles = eig(A_lat_p);

damperGainSummary = table(Kq_pick, Kp_roll_pick)

%% Part B: add proportional outer-loop control
% For the longitudinal path, I am using u as the small-disturbance
% airspeed state. For the lateral path, I am using phi for bank angle.
Gu = ss(A_long_q, B_long(:, 1), [1 0 0 0], 0);
Gphi = ss(A_lat_p, B_lat(:, 1), [0 0 0 1], 0);

airspeed_p_tf = minreal(zpk(Gu))
bank_p_tf = minreal(zpk(Gphi))

figure('Color', 'w', 'Position', [100 100 1100 420]);
tiledlayout(1,2);

nexttile;
rlocus(Gu);
grid on;
title('Part B Longitudinal: u / front collective');

nexttile;
rlocus(Gphi);
grid on;
title('Part B Lateral: \phi / \delta_a with roll damper locked');

%% Pick one proportional airspeed gain
if ~exist('Kv_pick', 'var')
    Kv_pick = 0.15;
end

A_long_p = A_long_q - B_long(:, 1) * Kv_pick * [1 0 0 0];
longPPoles = eig(A_long_p);

longPPoleTable = table(real(longPPoles), imag(longPPoles), ...
    'VariableNames', {'real_part','imag_part'})

figure('Color', 'w', 'Position', [100 100 560 420]);
plot(real(longPPoles), imag(longPPoles), 'o', 'LineWidth', 1.2, 'MarkerSize', 6);
grid on;
xlabel('Real');
ylabel('Imaginary');
title(sprintf('Longitudinal Poles After K_q and K_v, K_v = %.3f', Kv_pick));

%% Part C: add integral action
Gu_p = ss(A_long_p, B_long(:, 1), [1 0 0 0], 0);
Gi = minreal(Gu_p / tf('s'));

figure('Color', 'w', 'Position', [100 100 560 420]);
rlocus(Gi);
grid on;
title('Part C Root Locus: u / (s * front collective)');

%% Pick one integral gain
if ~exist('Ki_pick', 'var')
    Ki_pick = 0.02;
end

A_long_pi = [A_long_p zeros(4,1); -[1 0 0 0] 0];
B_long_pi = [B_long(:,1); 0];
piPoles = eig(A_long_pi - B_long_pi * Ki_pick * [zeros(1,4) 1]);

piPoleTable = table(real(piPoles), imag(piPoles), ...
    'VariableNames', {'real_part','imag_part'})

figure('Color', 'w', 'Position', [100 100 560 420]);
plot(real(piPoles), imag(piPoles), 'o', 'LineWidth', 1.2, 'MarkerSize', 6);
grid on;
xlabel('Real');
ylabel('Imaginary');
title(sprintf('Longitudinal Poles After Adding I Gain, K_i = %.3f', Ki_pick));

%% Gain summary
gainSummary = table(Kq_pick, Kp_roll_pick, Kv_pick, Ki_pick)
