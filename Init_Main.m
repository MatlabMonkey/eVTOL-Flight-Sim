% Init_Main.m
% Canonical initialization script for the eVTOL Flight Sim workflow.
%
% This script owns aircraft constants, default workspace variables, and a
% neutral default run configuration. It does not solve trim or build a
% controller. Those jobs now live in the trim and controller scripts.

clearvars -except render_enable initOptions trimCase trimSpec trimResult ...
    controllerData runCase runSpec runSuite suiteResults runResult ...
    transitionPathPrepOptions transitionPathSchedulePrepOptions ...
    babyTransitionPrepOptions;
close all;
clc;

if ~exist('render_enable', 'var')
    render_enable = false;
end

%% Paths
stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end
workspace_root = root_dir;
legacy_parent_root = fileparts(root_dir);
models_dir = fullfile(root_dir, 'models');
cases_dir = fullfile(root_dir, 'cases');
trim_cases_dir = fullfile(cases_dir, 'trim');
run_cases_dir = fullfile(cases_dir, 'run');
controller_cases_dir = fullfile(cases_dir, 'controller');
report_cases_dir = fullfile(cases_dir, 'report');
controllers_dir = fullfile(root_dir, 'controllers');
controller_builders_dir = fullfile(controllers_dir, 'builders');
controller_presets_dir = fullfile(controllers_dir, 'presets');
planning_dir = fullfile(root_dir, 'planning');
plotting_dir = fullfile(root_dir, 'plotting');
trim_db_paths = TrimDB_Paths(root_dir);
database_dir = trim_db_paths.database_dir;
workspace_plots_dir = trim_db_paths.workspace_plots_dir;
repo_root = workspace_root;
external_scripts_dir = localResolveFirstExistingDir({ ...
    fullfile(workspace_root, 'scripts'), ...
    fullfile(legacy_parent_root, 'scripts')});
localAddPathIfExists(root_dir);
localAddPathIfExists(models_dir);
localAddPathIfExists(trim_cases_dir);
localAddPathIfExists(run_cases_dir);
localAddPathIfExists(controller_cases_dir);
localAddPathIfExists(report_cases_dir);
localAddPathIfExists(controllers_dir);
localAddPathIfExists(controller_builders_dir);
localAddPathIfExists(controller_presets_dir);
localAddPathIfExists(planning_dir);
localAddPathIfExists(plotting_dir);

% Keep this copied workspace on its own Simulink cache/codegen folders so
% it does not fight with similarly named models elsewhere in the repo.
cache_dir = fullfile(root_dir, '.simcache');
codegen_dir = fullfile(root_dir, '.simcode');
if exist(cache_dir, 'dir') ~= 7
    mkdir(cache_dir);
end
if exist(codegen_dir, 'dir') ~= 7
    mkdir(codegen_dir);
end
if exist(database_dir, 'dir') ~= 7
    mkdir(database_dir);
end
if exist(workspace_plots_dir, 'dir') ~= 7
    mkdir(workspace_plots_dir);
end
Simulink.fileGenControl('set', ...
    'CacheFolder', cache_dir, ...
    'CodeGenFolder', codegen_dir, ...
    'createDir', true);

%% Simulation timing defaults
startTime = 0;
stopTime = 30;
stepTime = 0.01;

%% Aircraft definition
aircraft = aircraft_def('flight_mode', 0);

% Export the same workspace variables that the current Simulink models
% already expect so we do not break teammates' existing blocks.
structural_tilt_angle = aircraft.tilt_angle;
rho = aircraft.rho;

prop = aircraft.prop;
compData = aircraft.compData;
aeroData = aircraft.aeroData;
controls = aircraft.controls;
MixMatrix = aircraft.MixMatrix;
DeMixMatrix = aircraft.DeMixMatrix;

Mass = aircraft.Mass;
CG = aircraft.CG;
J = aircraft.J;

% Legacy aliases kept for existing Simulink block parameters.
m = Mass;
I = J;

wing = localPrepareSurfaceRuntimeStruct(aircraft.wing);
wingL = localPrepareSurfaceRuntimeStruct(aircraft.wingL);
wingR = localPrepareSurfaceRuntimeStruct(aircraft.wingR);
tailL = localPrepareSurfaceRuntimeStruct(aircraft.tailL);
tailR = localPrepareSurfaceRuntimeStruct(aircraft.tailR);

wingPolar = struct();
tailPolar = struct();
polar_data_file = localResolveFirstExistingFile({ ...
    fullfile(database_dir, 'aero_polars', 'final_airfoil_polar_tables.mat'), ...
    fullfile(workspace_root, 'scripts', 'avl', 'generated', 'final_airfoil_polar_tables.mat'), ...
    fullfile(legacy_parent_root, 'scripts', 'avl', 'generated', 'final_airfoil_polar_tables.mat')});
if strlength(string(polar_data_file)) == 0
    polar_data_file = fullfile(database_dir, 'aero_polars', 'final_airfoil_polar_tables.mat');
end
if exist(polar_data_file, 'file') == 2
    polarData = load(polar_data_file, 'wingPolar', 'tailPolar');
    wingPolar = localPrepareSurfaceRuntimeStruct(polarData.wingPolar);
    tailPolar = localPrepareSurfaceRuntimeStruct(polarData.tailPolar);
else
    warning('Init_Main:MissingPolarTables', ...
        'Did not find %s. LUT aero blocks will need the final polar table MAT file.', ...
        polar_data_file);
end

aircraft.wing = wing;
aircraft.wingL = wingL;
aircraft.wingR = wingR;
aircraft.tailL = tailL;
aircraft.tailR = tailR;
aircraft.render_surfaces = {wingL, wingR, tailL, tailR};

%% Default initial condition and scenario placeholders
scenario_name = "Flight_Test";

% These are defaults only. The trim and run scripts overwrite them when a
% specific operating point or experiment is selected.
%
% NOTE: active trim architecture now uses grouped motor RPM commands in the
% order [FR; FL; RR; RL]. Keep that 4x1 convention here unless/until the
% full run/wrapper path is migrated back to per-prop commands everywhere.
tilt_angle_deg = 90;
motor_rpm_value = 1500;

Motor_RPMs = motor_rpm_value * ones(4, 1);
Tilt_angles = tilt_angle_deg * ones(2, 1);

front_rpm_group = mean(Motor_RPMs(1:min(2, numel(Motor_RPMs))));
rear_rpm_group = mean(Motor_RPMs(min(3, numel(Motor_RPMs)):end));
if isempty(rear_rpm_group) || isnan(rear_rpm_group)
    rear_rpm_group = front_rpm_group;
end
front_tilt_group = mean(Tilt_angles(:));

% First-order actuator parameter masks used by the grouped propeller,
% grouped tilt, and control-surface servo blocks in the active models.
wingL = localAttachSurfaceServoMask(wingL, 0.0, 0.15);
wingR = localAttachSurfaceServoMask(wingR, 0.0, 0.15);
tailL = localAttachSurfaceServoMask(tailL, 0.0, 0.15);
tailR = localAttachSurfaceServoMask(tailR, 0.0, 0.15);

prop.Jrotor = 3.0;
prop.rotor = struct( ...
    'sat_high', 2400.0, ...
    'sat_low', 0.0, ...
    'dot_sat_high', 1000.0, ...
    'dot_sat_low', -1000.0, ...
    'init', localGroupMotorRPMs(Motor_RPMs), ...
    'tau', 0.10);
prop.tilt = struct( ...
    'sat_high', 90.0, ...
    'sat_low', 0.0, ...
    'dot_sat_high', 7.5, ...
    'dot_sat_low', -7.5, ...
    'init', localCollapseFrontTiltAngles(Tilt_angles), ...
    'tau', 0.25);

if numel(prop.rotor.init) >= 4
    prop.rotor.init_FR = prop.rotor.init(1);
    prop.rotor.init_FL = prop.rotor.init(2);
    prop.rotor.init_RR = prop.rotor.init(3);
    prop.rotor.init_RL = prop.rotor.init(4);
end
if numel(prop.tilt.init) >= 2
    prop.tilt.init_FR = prop.tilt.init(1);
    prop.tilt.init_FL = prop.tilt.init(2);
end

aircraft.prop = prop;
aircraft.wingL = wingL;
aircraft.wingR = wingR;
aircraft.tailL = tailL;
aircraft.tailR = tailR;
aircraft.render_surfaces = {wingL, wingR, tailL, tailR};

tilt_angle = mean(Tilt_angles(:));
flight_mode = double(tilt_angle >= 45);
g = 9.81;
Fext_B = [0; 0; 0];
Mext_B = [0; 0; 0];

% Initial earth position [lat lon alt] and NED reference.
lla_init = [34.05105, -118.25439, 1000];
lla_ref = [33.9416, -118.4085, 38];
pos_init = lla2ned(lla_init, lla_ref, 'flat');
pos_init = pos_init(:);

V_init = [75.0; 0.0; 0.0];
eul_init = deg2rad([0; 0; 0]);
omega_init = [0.0; 0.0; 0.0];

%% Controller defaults
% The controller script and run script both consume these as neutral
% starting values. They are not assumed to be tuned.
controller_enable = false;
controller_base_rpm = mean([front_rpm_group; rear_rpm_group]);
controller_mode = 1; % 1=open loop, 2=PID
ctrlMode = controller_mode; % Wrapper variant-control alias.

controller_trim_motor_rpms = zeros(4, 1);
controller_trim_tilt_angles = [front_tilt_group; front_tilt_group];
controller_trim_motor_rpm_cmd = zeros(4, 1);
controller_trim_tilt_angles_cmd = front_tilt_group * ones(numel(Tilt_angles), 1);
controller_trim_delta_f = 0.0;
controller_trim_delta_a = 0.0;
controller_trim_delta_e = 0.0;
controller_trim_delta_r = 0.0;
controller_airspeed_cmd_base = norm(V_init);
controller_bank_cmd_base = eul_init(1);
controller_step_time = 1.0;
controller_airspeed_step = 0.0;
controller_bank_step = 0.0;
controller_front_motor_mask = [1; 1; 0; 0];
controller_surface_limit_rad = deg2rad(25.0);
controller_rpm_delta_limit = 2000.0;
controller_integrator_limit = 200.0;
controller_sample_time = stepTime;
controller_k_p_damp = 0.0;
controller_k_q_damp = 0.0;
controller_k_r_damp = 0.0;
controller_k_phi_p = 0.0;
controller_k_phi_i = 0.0;
controller_k_v_p = 0.0;
controller_k_v_i = 0.0;

% Controller-dispatch defaults for the MATLAB Function block delegator path.
% The block can compile against these immediately, then Run_Main
% overwrites the reference and gain data with trim-specific values.
controller_id = 0; % 0=hold trim, 1-5=LQR variants, 6=scheduled INDI transition
controller_state_ref = zeros(24, 20); % first column/page for trim-point controllers, full schedule for path controllers
controller_trim_cmd = zeros(6, 20); % [front; rear; df; da; de; dr] across the schedule columns
controller_gain_lqr = zeros(6, 9, 20);
front_collective_min_rpm = 0.0;
front_collective_max_rpm = 7000.0;
rear_collective_min_rpm = 0.0;
rear_collective_max_rpm = 7000.0;

controllerDefaults = struct( ...
    'enable', controller_enable, ...
    'mode', controller_mode, ...
    'ctrlMode', ctrlMode, ...
    'base_rpm', controller_base_rpm, ...
    'trim_motor_rpms', controller_trim_motor_rpms, ...
    'trim_tilt_angles', controller_trim_tilt_angles, ...
    'trim_motor_rpm_cmd', controller_trim_motor_rpm_cmd, ...
    'trim_tilt_angles_cmd', controller_trim_tilt_angles_cmd, ...
    'trim_delta_f', controller_trim_delta_f, ...
    'trim_delta_a', controller_trim_delta_a, ...
    'trim_delta_e', controller_trim_delta_e, ...
    'trim_delta_r', controller_trim_delta_r, ...
    'airspeed_cmd_base', controller_airspeed_cmd_base, ...
    'bank_cmd_base', controller_bank_cmd_base, ...
    'step_time', controller_step_time, ...
    'airspeed_step', controller_airspeed_step, ...
    'bank_step', controller_bank_step, ...
    'front_motor_mask', controller_front_motor_mask, ...
    'surface_limit_rad', controller_surface_limit_rad, ...
    'rpm_delta_limit', controller_rpm_delta_limit, ...
    'integrator_limit', controller_integrator_limit, ...
    'sample_time', controller_sample_time);

%% Sensor defaults
sensor_sample_time = stepTime;
sensor_minAirspeed_mps = 0.25;
sensor_airdata_bias = [0; 0; 0];
sensor_airdata_sigma = [0; 0; 0];
sensor_gyro_bias = [0; 0; 0];
sensor_gyro_sigma = [0; 0; 0];
sensor_accel_bias = [0; 0; 0];
sensor_accel_sigma = [0; 0; 0];
angular_accel_bias = [0; 0; 0];
angular_accel_sigma = [0; 0; 0];

% Actuator measurement sensor placeholders. These should be tuned later in
% actuator-native units: rpm entries in rpm, tilt entries in tilt-angle units,
% and surface entries in surface-deflection units. Bias order is
% [4 motor RPM states; 2 tilt states; 4 surface states].
sensor_rpm_sigma = zeros(4, 1);
sensor_tilt_sigma = zeros(2, 1);
sensor_surface_sigma = zeros(4, 1);
actuator_sensor_bias = zeros(10, 1);

sensor_gps_posBias = [0; 0; 0];
sensor_gps_posSigma = [0; 0; 0];
sensor_gps_velBias = [0; 0; 0];
sensor_gps_velSigma = [0; 0; 0];
sensor_att_bias = [0; 0; 0];
sensor_att_sigma = [0; 0; 0];
sensor_magField_NED = [1; 0; 0];
sensor_mag_bias = [0; 0; 0];
sensor_mag_sigma = [0; 0; 0];
sensor_seed_airdata = 11;
sensor_seed_gyro = 21;
sensor_seed_accel = 31;
sensor_seed_gpsPos = 41;
sensor_seed_gpsVel = 51;
sensor_seed_att = 61;
sensor_seed_mag = 71;

sensorDefaults = struct( ...
    'sample_time', sensor_sample_time, ...
    'minAirspeed_mps', sensor_minAirspeed_mps, ...
    'airdata_bias', sensor_airdata_bias, ...
    'airdata_sigma', sensor_airdata_sigma, ...
    'gyro_bias', sensor_gyro_bias, ...
    'gyro_sigma', sensor_gyro_sigma, ...
    'accel_bias', sensor_accel_bias, ...
    'accel_sigma', sensor_accel_sigma, ...
    'angular_accel_bias', angular_accel_bias, ...
    'angular_accel_sigma', angular_accel_sigma, ...
    'sensor_rpm_sigma', sensor_rpm_sigma, ...
    'sensor_tilt_sigma', sensor_tilt_sigma, ...
    'sensor_surface_sigma', sensor_surface_sigma, ...
    'actuator_sensor_bias', actuator_sensor_bias, ...
    'gps_pos_bias', sensor_gps_posBias, ...
    'gps_pos_sigma', sensor_gps_posSigma, ...
    'gps_vel_bias', sensor_gps_velBias, ...
    'gps_vel_sigma', sensor_gps_velSigma, ...
    'att_bias', sensor_att_bias, ...
    'att_sigma', sensor_att_sigma, ...
    'magField_NED', sensor_magField_NED, ...
    'mag_bias', sensor_mag_bias, ...
    'mag_sigma', sensor_mag_sigma);

%% Default run specification and command placeholders
% This is a neutral "hold what we initialized" run case. The run script
% takes ownership of experiment/scenario definitions and can overwrite this.
defaultRunSpec = struct();
defaultRunSpec.name = char(scenario_name);
defaultRunSpec.modelName = 'Wrapper';
defaultRunSpec.startTime = startTime;
defaultRunSpec.stopTime = stopTime;
defaultRunSpec.stepTime = stepTime;
defaultRunSpec.pos_init = pos_init;
defaultRunSpec.V_init = V_init;
defaultRunSpec.eul_init = eul_init;
defaultRunSpec.omega_init = omega_init;
defaultRunSpec.Motor_RPMs = Motor_RPMs;
defaultRunSpec.Tilt_angles = Tilt_angles;
defaultRunSpec.front_collective = front_rpm_group;
defaultRunSpec.rear_collective = rear_rpm_group(1);
defaultRunSpec.eulerStepTime = stopTime + stepTime;
defaultRunSpec.eulerStepDeg = [0; 0; 0];
defaultRunSpec.useController = false;
defaultRunSpec.attemptSimulation = false;

cmds = localMakeEvtolCmds(defaultRunSpec);

% Neutral memory-block initial conditions for updating/opening Wrapper after
% Init_Main alone. Run_Main overwrites these from the selected run case.
V_mem_init = V_init;
accel_mem_init = [0; 0; 0];
actuator_mem_init = [prop.rotor.init(:); prop.tilt.init(:); zeros(4, 1)];
angular_accel_mem_init = [0; 0; 0];

%% Structured summary for downstream scripts
initData = struct();
initData.root_dir = root_dir;
initData.workspace_root = workspace_root;
initData.legacy_parent_root = legacy_parent_root;
initData.external_scripts_dir = external_scripts_dir;
initData.models_dir = models_dir;
initData.trim_cases_dir = trim_cases_dir;
initData.run_cases_dir = run_cases_dir;
initData.database_dir = database_dir;
initData.workspace_plots_dir = workspace_plots_dir;
initData.trimDatabasePaths = trim_db_paths;
initData.cache_dir = cache_dir;
initData.codegen_dir = codegen_dir;
initData.modelNames = struct( ...
    'trim', 'Trim_Plant', ...
    'run', 'Wrapper', ...
    'wrapper', 'Wrapper', ...
    'controller', 'Wrapper');
initData.timing = struct('startTime', startTime, 'stopTime', stopTime, 'stepTime', stepTime);
initData.aircraft = aircraft;
initData.initialState = struct( ...
    'pos_init', pos_init, ...
    'V_init', V_init, ...
    'eul_init', eul_init, ...
    'omega_init', omega_init);
initData.defaults = struct( ...
    'Motor_RPMs', Motor_RPMs, ...
    'Tilt_angles', Tilt_angles, ...
    'front_rpm_group', front_rpm_group, ...
    'rear_rpm_group', rear_rpm_group, ...
    'front_tilt_group', front_tilt_group, ...
    'Fext_B', Fext_B, ...
    'Mext_B', Mext_B, ...
    'V_mem_init', V_mem_init, ...
    'accel_mem_init', accel_mem_init, ...
    'actuator_mem_init', actuator_mem_init, ...
    'angular_accel_mem_init', angular_accel_mem_init, ...
    'wingPolar', wingPolar, ...
    'tailPolar', tailPolar);
initData.controllerDefaults = controllerDefaults;
initData.sensorDefaults = sensorDefaults;
initData.defaultRunSpec = defaultRunSpec;
initData.polar_data_file = polar_data_file;

fprintf('Initialized eVTOL workspace in %s\n', root_dir);
fprintf('Trim model: %s | Run model: %s\n', ...
    initData.modelNames.trim, initData.modelNames.run);

%% 3D visualization
if render_enable
    disp('Rendering 3D Aircraft Model...');
    render_aircraft(compData, CG, tilt_angle, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', prop, ...
        'controls', controls, ...
        'thrust_tilt_deg', Tilt_angles, ...
        'title_text', sprintf('%s (Scenario: %s)', 'Brown eVTOL Aircraft', scenario_name));
end

function cmds = localMakeEvtolCmds(runSpec)
startTime = runSpec.startTime;
stopTime = runSpec.stopTime;
stepTime = runSpec.stepTime;

t = (startTime:stepTime:stopTime).';
N = numel(t);

if isfield(runSpec, 'eulerStepTime')
    eulerStepTime = runSpec.eulerStepTime;
else
    eulerStepTime = stopTime + stepTime;
end

if isfield(runSpec, 'eulerStepDeg')
    eulerStepDeg = runSpec.eulerStepDeg(:);
else
    eulerStepDeg = [0; 0; 0];
end

cmds = struct();

Vinf_cmd = runSpec.V_init(1) * ones(N, 1);
alpha_cmd = runSpec.V_init(2) * ones(N, 1);
beta_cmd = runSpec.V_init(3) * ones(N, 1);
cmds.airData_cmd = [t, Vinf_cmd, alpha_cmd, beta_cmd];

phi_cmd = runSpec.eul_init(1) * ones(N, 1);
theta_cmd = runSpec.eul_init(2) * ones(N, 1);
psi_cmd = runSpec.eul_init(3) * ones(N, 1);
phi_cmd(t >= eulerStepTime) = phi_cmd(t >= eulerStepTime) + deg2rad(eulerStepDeg(1));
theta_cmd(t >= eulerStepTime) = theta_cmd(t >= eulerStepTime) + deg2rad(eulerStepDeg(2));
psi_cmd(t >= eulerStepTime) = psi_cmd(t >= eulerStepTime) + deg2rad(eulerStepDeg(3));
cmds.eul_cmd = [t, phi_cmd, theta_cmd, psi_cmd];

north_cmd = runSpec.pos_init(1) * ones(N, 1);
east_cmd = runSpec.pos_init(2) * ones(N, 1);
down_cmd = runSpec.pos_init(3) * ones(N, 1);
cmds.gps_Pos_cmd = [t, north_cmd, east_cmd, down_cmd];

P_cmd = runSpec.omega_init(1) * ones(N, 1);
Q_cmd = runSpec.omega_init(2) * ones(N, 1);
R_cmd = runSpec.omega_init(3) * ones(N, 1);
cmds.omega_cmd = [t, P_cmd, Q_cmd, R_cmd];

cmds.accel_cmd = [t, zeros(N, 1), zeros(N, 1), zeros(N, 1)];

motor_hist = zeros(N, numel(runSpec.Motor_RPMs));
for i = 1:numel(runSpec.Motor_RPMs)
    motor_hist(:, i) = runSpec.Motor_RPMs(i) * ones(N, 1);
end
cmds.motor_cmd = [t, motor_hist];

tilt_hist = zeros(N, numel(runSpec.Tilt_angles));
for i = 1:numel(runSpec.Tilt_angles)
    tilt_hist(:, i) = runSpec.Tilt_angles(i) * ones(N, 1);
end
cmds.tilt_cmd = [t, tilt_hist];

cmds.front_cmd = [t, runSpec.front_collective * ones(N, 1)];
cmds.rear_cmd = [t, runSpec.rear_collective * ones(N, 1)];
end

function surface = localPrepareSurfaceRuntimeStruct(surface)
% Runtime-parameter surface structs must stay numeric/logical only.
fields = fieldnames(surface);
for idx = numel(fields):-1:1
    field_name = fields{idx};
    field_value = surface.(field_name);

    if isstruct(field_value)
        surface.(field_name) = localPrepareSurfaceRuntimeStruct(field_value);
    elseif ~(isnumeric(field_value) || islogical(field_value))
        surface = rmfield(surface, field_name);
    end
end
end

function surface = localAttachSurfaceServoMask(surface, init_value, tau_value)
surface.sat_high = surface.delta_max;
surface.sat_low = -surface.delta_max;
surface.dot_sat_high = inf;
surface.dot_sat_low = -inf;
surface.init = init_value;
surface.tau = tau_value;
end

function grouped = localGroupMotorRPMs(rpm_in)
rpm_in = rpm_in(:);

if isempty(rpm_in)
    grouped = zeros(4, 1);
elseif numel(rpm_in) >= 12
    % Legacy order:
    %   1:3 FL, 4:6 FR, 7:9 RL, 10:12 RR
    grouped = [ ...
        mean(rpm_in(4:6)); ...
        mean(rpm_in(1:3)); ...
        mean(rpm_in(10:12)); ...
        mean(rpm_in(7:9))];
elseif numel(rpm_in) == 4
    grouped = rpm_in;
elseif numel(rpm_in) == 2
    grouped = [rpm_in(1); rpm_in(2); rpm_in(1); rpm_in(2)];
else
    grouped = rpm_in(1) * ones(4, 1);
end
end

function grouped = localCollapseFrontTiltAngles(tilt_in)
tilt_in = tilt_in(:);

if isempty(tilt_in)
    grouped = zeros(2, 1);
elseif numel(tilt_in) >= 6
    % Legacy front-tilt order:
    %   1:3 FR, 4:6 FL
    grouped = [mean(tilt_in(1:3)); mean(tilt_in(4:6))];
elseif numel(tilt_in) == 2
    grouped = tilt_in;
else
    grouped = tilt_in(1) * ones(2, 1);
end
end

function localAddPathIfExists(pathStr)
if exist(pathStr, 'dir') == 7
    addpath(pathStr);
end
end

function resolvedDir = localResolveFirstExistingDir(candidateDirs)
resolvedDir = "";
for i = 1:numel(candidateDirs)
    candidate = string(candidateDirs{i});
    if strlength(candidate) > 0 && exist(candidate, 'dir') == 7
        resolvedDir = char(candidate);
        return;
    end
end
end

function resolvedFile = localResolveFirstExistingFile(candidateFiles)
resolvedFile = "";
for i = 1:numel(candidateFiles)
    candidate = string(candidateFiles{i});
    if strlength(candidate) > 0 && exist(candidate, 'file') == 2
        resolvedFile = char(candidate);
        return;
    end
end
end
