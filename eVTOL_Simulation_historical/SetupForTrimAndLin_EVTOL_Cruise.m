% SetupForTrimAndLin_EVTOL_Cruise.m
% Workspace setup for steady level-flight cruise trim and linearization.

%% Paths and base workspace init
root_dir = pwd;
models_dir = fullfile(root_dir, 'models');

addpath(root_dir);
addpath(models_dir);

render_enable = false;
Full_Sim_Init;

%% Cruise trim assumptions
Vinf_trim_target = 75; % m/s
psi_trim_target = 0.0;
front_collective_guess_rpm = 1200.0;
rear_collective_trim_rpm = 0.0;
front_tilt_trim_deg = 0.0;
trim_validation_stop_time = 5.0;

%% Trim
% Perfect actuators: 1
% Lagged actuators: 0
trim = struct();
trim.usePerfectAct = 1;

trim.dLW  = 0;
trim.dRW  = 0;
trim.dLT  = 0;
trim.dRT  = 0;

trim.tauLW = 0.05;
trim.tauRW = 0.05;
trim.tauLT = 0.05;
trim.tauRT = 0.05;

% Small floor just so 1/tau never divides by zero in the lag path
trim.tauMin = 1e-3;

% Position limits
trim.dLW_max = deg2rad(25);
trim.dRW_max = deg2rad(25);
trim.dLT_max  = deg2rad(25);
trim.dRT_max  = deg2rad(25);

% Rate limits
trim.dLWdot_max = deg2rad(250);
trim.dRWdot_max = deg2rad(250);
trim.dLTdot_max  = deg2rad(250);
trim.dRTdot_max  = deg2rad(250);