%% Homework 4 Submission Root Locus
% This is the maintainable source for HW4_submission_root_locus.mlx.

clear; clc; close all;
format short g;
addpath(genpath(fullfile(pwd, 'scripts')));

%% Trim and reduced models
data = linearize_reduced_flight_dynamics_eul('CruiseSpeed', 70, 'DisplayReport', 'off');

trimSummary = table( ...
    data.trimPoint.inputs.front_collective_rpm, ...
    data.trimPoint.inputs.rear_collective_rpm, ...
    rad2deg(data.trimPoint.inputs.delta_e), ...
    rad2deg(data.trimPoint.eul(2)), ...
    norm(data.trimPoint.velocity), ...
    'VariableNames', {'front_collective_rpm','rear_collective_rpm','delta_e_deg','theta_deg','vinf_mps'})

%% Reduced state definitions
% Longitudinal states: [u w q theta]
% Lateral states:      [v p r phi]
longStateTable = table(string(data.long.state_names).', ...
    'VariableNames', {'longitudinal_states'})

latStateTable = table(string(data.lat.state_names).', ...
    'VariableNames', {'lateral_states'})

%% Reduced A and B matrices
A_long = data.long.A
B_long = data.long.B

A_lat = data.lat.A
B_lat = data.lat.B

%% Part A: pure dampers
% Pure dampers use rate-over-control transfer functions.
Gq = data.transfer.q_over_delta_e;
Gp = data.transfer.p_over_delta_a;
Gr = data.transfer.r_over_delta_r;

figure('Color', 'w', 'Position', [100 100 1200 420]);
tiledlayout(1,3);

nexttile;
rlocus(Gq);
grid on;
title('Pitch Damper: q / \delta_e');

nexttile;
rlocus(Gp);
grid on;
title('Roll Damper: p / \delta_a');

nexttile;
rlocus(Gr);
grid on;
title('Yaw Damper: r / \delta_r');

%% Pick damper gains
% Change these after looking at the root loci.
Kq_pick = 0.4;
Kp_roll_pick = 0.2;

A_long_q = data.long.A - data.long.B(:,3) * Kq_pick * [0 0 1 0];
A_lat_p = data.lat.A - data.lat.B(:,1) * Kp_roll_pick * [0 1 0 0];

damperGainSummary = table(Kq_pick, Kp_roll_pick)

%% Part B: proportional outer loops
% Longitudinal option: airspeed-like u response to front collective
% Lateral option: bank angle response to aileron with roll damper locked in
Gu = ss(A_long_q, data.long.B(:,1), [1 0 0 0], 0);
Gphi = ss(A_lat_p, data.lat.B(:,1), [0 0 0 1], 0);

figure('Color', 'w', 'Position', [100 100 1100 420]);
tiledlayout(1,2);

nexttile;
rlocus(Gu);
grid on;
title('Part B Longitudinal: u / front collective');

nexttile;
rlocus(Gphi);
grid on;
title('Part B Lateral: \phi / \delta_a');

%% Pick one proportional longitudinal gain
Kv_pick = 0.15;

A_long_p = A_long_q - data.long.B(:,1) * Kv_pick * [1 0 0 0];
longPPoles = eig(A_long_p);

longPPoleTable = table(real(longPPoles), imag(longPPoles), ...
    'VariableNames', {'real_part','imag_part'})

%% Part C: add integral action
Gu_p = ss(A_long_p, data.long.B(:,1), [1 0 0 0], 0);
Gi = minreal(Gu_p / tf('s'));

figure('Color', 'w', 'Position', [100 100 560 420]);
rlocus(Gi);
grid on;
title('Part C Longitudinal: u / (s * front collective)');

%% Pick one integral gain
Ki_pick = 0.02;

A_long_pi = [A_long_p zeros(4,1); -[1 0 0 0] 0];
B_long_pi = [data.long.B(:,1); 0];
piPoles = eig(A_long_pi - B_long_pi * Ki_pick * [0 0 0 0 1]);

piPoleTable = table(real(piPoles), imag(piPoles), ...
    'VariableNames', {'real_part','imag_part'})

%% Final chosen gains
gainSummary = table(Kq_pick, Kp_roll_pick, Kv_pick, Ki_pick)
