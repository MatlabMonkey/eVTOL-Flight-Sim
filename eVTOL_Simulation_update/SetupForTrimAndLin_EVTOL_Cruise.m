% SetupForTrimAndLin_EVTOL_Cruise.m
%
% Loads aircraft parameters, defines the cruise design point, and
% populates the control-surface trim struct read by the Simulink model.

%% Paths
root_dir   = fileparts(mfilename('fullpath'));
models_dir = fullfile(root_dir, 'models');
addpath(root_dir);
addpath(models_dir);

%% Load base workspace
% Full_Sim_Init populates aircraft params (Mass, J, prop, wing*, tail*),
% sensor variables, controller scaffold, cmds struct, and ICs.
render_enable = false;
Full_Sim_Init;

%% Sim timing
startTime = 0;
stopTime  = 30;
stepTime  = 0.01;

%% Constants
deg2rad = pi / 180;

%% Cruise design point
Vinf_trim_target            = 75.0;

% Use the repo's cruise tilt convention first.
front_tilt_trim_deg         = 90.0;

% Use a repo-like cruise seed.
front_collective_guess_rpm  = 1180.0;
rear_collective_guess_rpm   = 0.0;

% Do not hard-fix rear collective until a feasible trim is found.
front_collective_max_rpm    = 7000.0;
rear_collective_max_rpm     = 7000.0;
%% Surface limits
surface_limit_deg = 25.0;
surface_limit_rad = surface_limit_deg * deg2rad;

%% Trim validation
trim_validation_stop_time = 5.0;   % s

%% Trim (control-surface struct consumed by Simulink model)
% Perfect actuators: 1
% Lagged actuators:  0
trim = struct();
trim.usePerfectAct = 0;

trim.dLW = 0;
trim.dRW = 0;
trim.dLT = 0;
trim.dRT = 0;

trim.tauLW = 0.001;
trim.tauRW = 0.001;
trim.tauLT = 0.001;
trim.tauRT = 0.001;

% Small floor so 1/tau never divides by zero in the lag path
trim.tauMin = 1e-3;

% Position limits
trim.dLW_max = deg2rad * 25;
trim.dRW_max = deg2rad * 25;
trim.dLT_max = deg2rad * 25;
trim.dRT_max = deg2rad * 25;

% Rate limits
trim.dLWdot_max = deg2rad * 250;
trim.dRWdot_max = deg2rad * 250;
trim.dLTdot_max = deg2rad * 250;
trim.dRTdot_max = deg2rad * 250;

fprintf('SetupForTrimAndLin_EVTOL_Cruise done.\n');
fprintf('  Vinf = %.0f m/s,  tilt = %.0f deg\n', ...
    Vinf_trim_target, front_tilt_trim_deg);
fprintf('  Front collective: %.0f rpm initial guess (free)\n', ...
    front_collective_guess_rpm);
