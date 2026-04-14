%% Homework 3, 5, and 6 Simple Demo
% This is the maintainable source for HW356_simple_demos.mlx.

clear; clc; close all;
format short g;

repoRoot = localResolveRepoRoot();
origDir = pwd;
cleanupDir = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);
addpath(genpath(fullfile(repoRoot, 'scripts')));

%% Setup
run(fullfile(repoRoot, 'scripts', 'control', 'setup_eul_controller_demo.m'));
controller_sample_time = 0.01;
modelName = 'Brown_6DOF_Sim_Wrapper';

%% Tune gains here
% These are the controller gains used by the runs below.
gains.V.Kp = 300;
gains.V.Ki = 5;
gains.theta.Kp = 0.10;
gains.theta.Kd = 0.10;
gains.phi.Kp = 2;
gains.phi.Kd = -2;

% Step commands used below.
step.longitudinal.time = 3.0;
step.longitudinal.d_vinf_cmd = 5.0;
step.lateral.time = 3.0;
step.lateral.d_phi_cmd = deg2rad(5.0);

gainSummary = table( ...
    gains.V.Kp, gains.V.Ki, ...
    gains.theta.Kp, gains.theta.Kd, ...
    gains.phi.Kp, gains.phi.Kd, ...
    'VariableNames', { ...
    'V_Kp', 'V_Ki', ...
    'theta_Kp', 'theta_Kd', ...
    'phi_Kp', 'phi_Kd'})

trimSummary = table( ...
    base_set_point.vinf_cmd, ...
    rad2deg(base_set_point.euler_cmd(2)), ...
    base_set_point.front_collective_rpm, ...
    rad2deg(base_set_point.delta_e_cmd), ...
    'VariableNames', { ...
    'trim_vinf_cmd_mps', ...
    'trim_theta_cmd_deg', ...
    'trim_front_collective_rpm', ...
    'trim_delta_e_deg'})

%% Homework 3 - sensor check
% Start at trim and do not command any step yet.
cmd_profile.vinf.enable = false;
cmd_profile.phi.enable = false;
cmd_profile.theta.enable = false;

simOut3 = localRunSim(modelName, 5);

[tVinfTruth3, vinfTruth3] = localSignal(simOut3.get('vinf_truth'));
[tAlphaTruth3, alphaTruth3] = localSignal(simOut3.get('alpha_truth'));
[tEulTruth3, eulTruth3] = localSignal(simOut3.get('eul_truth'));
[tOmegaTruth3, omegaTruth3] = localSignal(simOut3.get('omega_truth'));

sensor3 = localSensorBus(simOut3.get('sensor_bus'), log_idx.sensor_bus);

figure('Color', 'w', 'Position', [100 100 1000 720]);
tiledlayout(2,2);

nexttile;
plot(tVinfTruth3, vinfTruth3, 'LineWidth', 1.2); hold on;
plot(sensor3.t, sensor3.airspeed, '--', 'LineWidth', 1.2);
grid on;
xlim([0.1 5]);
title('Airspeed');
legend('truth', 'sensor', 'Location', 'best');

nexttile;
plot(tAlphaTruth3, rad2deg(alphaTruth3), 'LineWidth', 1.2); hold on;
plot(sensor3.t, rad2deg(sensor3.alpha), '--', 'LineWidth', 1.2);
grid on;
xlim([0.1 5]);
title('Alpha');
legend('truth', 'sensor', 'Location', 'best');

nexttile;
plot(tOmegaTruth3, omegaTruth3(:,2), 'LineWidth', 1.2); hold on;
plot(sensor3.t, sensor3.gyro(:,2), '--', 'LineWidth', 1.2);
grid on;
xlim([0.1 5]);
title('Pitch Rate q');
legend('truth', 'sensor', 'Location', 'best');

nexttile;
plot(tEulTruth3, rad2deg(eulTruth3(:,2)), 'LineWidth', 1.2); hold on;
plot(sensor3.t, rad2deg(sensor3.attitude(:,2)), '--', 'LineWidth', 1.2);
grid on;
xlim([0.1 5]);
title('Pitch Angle \theta');
legend('truth', 'sensor', 'Location', 'best');

sensorSummary = table( ...
    vinfTruth3(end), sensor3.airspeed(end), ...
    rad2deg(alphaTruth3(end)), rad2deg(sensor3.alpha(end)), ...
    omegaTruth3(end,2), sensor3.gyro(end,2), ...
    rad2deg(eulTruth3(end,2)), rad2deg(sensor3.attitude(end,2)), ...
    'VariableNames', { ...
    'vinf_truth_end', 'vinf_sensor_end', ...
    'alpha_truth_end_deg', 'alpha_sensor_end_deg', ...
    'q_truth_end', 'q_sensor_end', ...
    'theta_truth_end_deg', 'theta_sensor_end_deg'})

%% Homework 5 - longitudinal set-point hold and step
% I am using airspeed feedback here. The command steps up by 5 m/s at 3 s.
cmd_profile.vinf.enable = true;
cmd_profile.vinf.time = step.longitudinal.time;
cmd_profile.vinf.magnitude = step.longitudinal.d_vinf_cmd;
cmd_profile.phi.enable = false;
cmd_profile.theta.enable = false;

simOut5 = localRunSim(modelName, 30);

[tVinf5, vinfTruth5] = localSignal(simOut5.get('vinf_truth'));
[tCmd5, vinfCmd5] = localSignal(simOut5.get('vinf_cmd'));
[tEul5, eulTruth5] = localSignal(simOut5.get('eul_truth'));
plantCmd5 = localPlantCmdBus(simOut5.get('plant_cmd_bus'), log_idx.plant_cmd_bus);

beforeIdx5 = localBeforeStepIndex(tVinf5, cmd_profile.vinf.time);

figure('Color', 'w', 'Position', [100 100 950 700]);
tiledlayout(3,1);

nexttile;
plot(tVinf5, vinfTruth5, 'LineWidth', 1.3); hold on;
plot(tCmd5, vinfCmd5, '--', 'LineWidth', 1.2);
grid on;
ylabel('m/s');
title('Homework 5 Longitudinal Response');
legend('airspeed', 'command', 'Location', 'best');

nexttile;
plot(tEul5, rad2deg(eulTruth5(:,2)), 'LineWidth', 1.3);
grid on;
ylabel('deg');
legend('\theta', 'Location', 'best');

nexttile;
plot(plantCmd5.t, plantCmd5.front_collective_rpm, 'LineWidth', 1.2); hold on;
plot(plantCmd5.t, rad2deg(plantCmd5.delta_e_cmd), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('cmd');
legend('front collective [rpm]', '\delta_e [deg]', 'Location', 'best');

longSummary = table( ...
    vinfTruth5(beforeIdx5), vinfCmd5(beforeIdx5), ...
    vinfTruth5(end), vinfCmd5(end), ...
    plantCmd5.front_collective_rpm(end), rad2deg(plantCmd5.delta_e_cmd(end)), ...
    'VariableNames', { ...
    'vinf_before_step', 'vinf_cmd_before_step', ...
    'vinf_end', 'vinf_cmd_end', ...
    'front_collective_end_rpm', 'delta_e_end_deg'})

%% Homework 6 - lateral set-point hold and step
% I am using bank-angle feedback here. The command steps up by 5 deg at 3 s.
cmd_profile.vinf.enable = false;
cmd_profile.phi.enable = true;
cmd_profile.phi.time = step.lateral.time;
cmd_profile.phi.magnitude = step.lateral.d_phi_cmd;
cmd_profile.theta.enable = false;

simOut6 = localRunSim(modelName, 30);

[tEul6, eulTruth6] = localSignal(simOut6.get('eul_truth'));
[tCmd6, eulerCmd6] = localSignal(simOut6.get('euler_cmd'));
[tOmega6, omegaTruth6] = localSignal(simOut6.get('omega_truth'));
plantCmd6 = localPlantCmdBus(simOut6.get('plant_cmd_bus'), log_idx.plant_cmd_bus);

beforeIdx6 = localBeforeStepIndex(tEul6, cmd_profile.phi.time);

figure('Color', 'w', 'Position', [100 100 950 700]);
tiledlayout(3,1);

nexttile;
plot(tEul6, rad2deg(eulTruth6(:,1)), 'LineWidth', 1.3); hold on;
plot(tCmd6, rad2deg(eulerCmd6(:,1)), '--', 'LineWidth', 1.2);
grid on;
ylabel('deg');
title('Homework 6 Lateral Response');
legend('\phi', 'command', 'Location', 'best');

nexttile;
plot(tOmega6, omegaTruth6(:,1), 'LineWidth', 1.3);
grid on;
ylabel('rad/s');
legend('p', 'Location', 'best');

nexttile;
plot(plantCmd6.t, rad2deg(plantCmd6.delta_a_cmd), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('deg');
legend('\delta_a', 'Location', 'best');

latSummary = table( ...
    rad2deg(eulTruth6(beforeIdx6,1)), rad2deg(eulerCmd6(beforeIdx6,1)), ...
    rad2deg(eulTruth6(end,1)), rad2deg(eulerCmd6(end,1)), ...
    rad2deg(plantCmd6.delta_a_cmd(end)), ...
    'VariableNames', { ...
    'phi_before_step_deg', 'phi_cmd_before_step_deg', ...
    'phi_end_deg', 'phi_cmd_end_deg', ...
    'delta_a_end_deg'})

%% Notes
% I can paste the sensor block screenshot in this live script by the
% Homework 3 section. This file just reruns the wrapper and makes the plots.

function [t, y] = localSignal(sig)
t = sig.time(:);
vals = sig.signals.values;
y = squeeze(vals);

if isvector(y)
    y = y(:);
    return;
end

n = numel(t);
if size(y,1) == n
    return;
end

if size(y, ndims(y)) == n
    y = reshape(y, [], n).';
else
    y = reshape(y, n, []);
end
end

function sensor = localSensorBus(sig, idx)
[t, raw] = localSignal(sig);

sensor = struct();
sensor.t = t;
sensor.airspeed = raw(:, idx.airspeed);
sensor.alpha = raw(:, idx.alpha);
sensor.beta = raw(:, idx.beta);
sensor.gyro = raw(:, idx.gyro);
sensor.accel = raw(:, idx.accel);
sensor.gps_pos = raw(:, idx.gps_pos);
sensor.gps_vel = raw(:, idx.gps_vel);
sensor.attitude = raw(:, idx.attitude);
end

function cmd = localPlantCmdBus(sig, idx)
[t, raw] = localSignal(sig);

cmd = struct();
cmd.t = t;
cmd.motor_rpm_cmd = raw(:, idx.motor_rpm_cmd);
cmd.tilt_angles_cmd = raw(:, idx.tilt_angles_cmd);
cmd.front_collective_rpm = raw(:, idx.front_collective_rpm);
cmd.rear_collective_rpm = raw(:, idx.rear_collective_rpm);
cmd.delta_f_cmd = raw(:, idx.delta_f_cmd);
cmd.delta_a_cmd = raw(:, idx.delta_a_cmd);
cmd.delta_e_cmd = raw(:, idx.delta_e_cmd);
cmd.delta_r_cmd = raw(:, idx.delta_r_cmd);
end

function idx = localBeforeStepIndex(t, stepTime)
idx = find(t < stepTime, 1, 'last');
if isempty(idx)
    idx = 1;
end
end

function simOut = localRunSim(modelName, stopTime)
warnState = warning;
cleanupWarn = onCleanup(@() warning(warnState)); %#ok<NASGU>
warning('off', 'all');
simOut = sim(modelName, 'StopTime', num2str(stopTime), 'ReturnWorkspaceOutputs', 'on');
end

function repoRoot = localResolveRepoRoot()
candidates = {};

try
    thisPath = mfilename('fullpath');
    if ~isempty(thisPath)
        candidates{end+1} = fileparts(thisPath); %#ok<AGROW>
    end
catch
end

try
    activeFile = matlab.desktop.editor.getActiveFilename;
    if ~isempty(activeFile)
        candidates{end+1} = fileparts(activeFile); %#ok<AGROW>
    end
catch
end

candidates{end+1} = pwd;

for i = 1:numel(candidates)
    root = candidates{i};
    if exist(fullfile(root, 'scripts', 'control', 'setup_eul_controller_demo.m'), 'file')
        repoRoot = root;
        return;
    end
end

error('Could not find the repo root that contains scripts/control/setup_eul_controller_demo.m.');
end
