%% =========================================================================
%% 1. INITIALIZATION & AIRCRAFT PARAMETERS
%% =========================================================================
clearvars -except render_enable; close all; clc;

% Keep the setup portable when these files are shared outside the repo.
% Full_Sim_Init.m, aircraft_def.m, scenario_def.m, and render_aircraft.m
% should live in the same folder.
stack = dbstack('-completenames');
if ~isempty(stack)
    script_dir = fileparts(stack(1).file);
else
    script_dir = pwd;
end
addpath(script_dir);
if exist(fullfile(script_dir, 'scripts'), 'dir') == 7
    addpath(genpath(fullfile(script_dir, 'scripts')));
end

required_files = { ...
    'aircraft_def.m', ...
    'scenario_def.m', ...
    'render_aircraft.m', ...
    'load_avl_aero_model.m', ...
    'avl_aircraft_aero_eval.m', ...
    'sensor_suite_eval.m', ...
    'attitude_complementary_step.m', ...
    'outer_controller_step.m'};
for idx = 1:numel(required_files)
    required_path = fullfile(script_dir, required_files{idx});
    if exist(required_path, 'file') ~= 2
        error('Full_Sim_Init:MissingDependency', ...
            ['Missing %s in %s.\n' ...
             'Keep Full_Sim_Init.m, aircraft_def.m, scenario_def.m, and render_aircraft.m together when sharing.'], ...
            required_files{idx}, script_dir);
    end
end

% Change this to run different cases.
test_case = 'Stable_6DOF';
if ~exist('render_enable', 'var')
    render_enable = true;
end

% The aircraft definition keeps the legacy reference geometry at hover.
% Visualization uses the scenario tilt commands instead.
aircraft = aircraft_def('flight_mode', 0);
scenario = scenario_def(test_case);

% Export the same workspace variables that the Simulink models expect.
flight_mode = scenario.flight_mode;
structural_tilt_angle = aircraft.tilt_angle;
tilt_angle = scenario.visual_tilt_deg;

rho = aircraft.rho;
g = scenario.g;

prop = aircraft.prop;
compData = aircraft.compData;
aeroData = aircraft.aeroData;
controls = aircraft.controls; % Metadata for ruddervators / future flaperons

Mass = aircraft.Mass;
CG = aircraft.CG;
J = aircraft.J;

% Legacy aliases kept for existing Simulink block parameters.
m = Mass;
I = J;

wing = aircraft.wing;
wingL = aircraft.wingL;
wingR = aircraft.wingR;
tailL = aircraft.tailL;
tailR = aircraft.tailR;

if isfield(wing, 'name')
    wing = rmfield(wing, 'name');
end
if isfield(wingL, 'name')
    wingL = rmfield(wingL, 'name');
end
if isfield(wingR, 'name')
    wingR = rmfield(wingR, 'name');
end
if isfield(tailL, 'name')
    tailL = rmfield(tailL, 'name');
end
if isfield(tailR, 'name')
    tailR = rmfield(tailR, 'name');
end

pos_init = scenario.pos_init;
V_init = scenario.V_init;
eul_init = scenario.eul_init;
omega_init = scenario.omega_init;
Motor_RPMs = scenario.Motor_RPMs;
Tilt_angles = scenario.Tilt_angles;
Fext_B = scenario.Fext_B;
Mext_B = scenario.Mext_B;

% Controller scaffold variables used by the optional Simulink PD block.
controller_enable = false;
front_rpm_group = mean(Motor_RPMs(1:min(6, numel(Motor_RPMs))));
rear_rpm_group = mean(Motor_RPMs(min(7, numel(Motor_RPMs)):end));
if isempty(rear_rpm_group) || isnan(rear_rpm_group)
    rear_rpm_group = front_rpm_group;
end
front_tilt_group = mean(Tilt_angles(:));

% The current Propellers subsystem is grouped as:
%   motor_rpms = [FR; FL; RR; RL]
%   tilt_angles = [FR; FL]
controller_base_rpm = mean([front_rpm_group; rear_rpm_group]);
controller_mode = 0; % 0=open loop, 1=dampers, 2=PD, 3=PID

% Controller trim commands and nominal set-points. These are intentionally
% kept separate from the scenario vectors so control-design scripts can
% replace them with cruise-trim values without rewriting the whole scenario.
% The grouped front/rear collectives provide the trim baseline in the
% wrapper path, so keep the motor-group trim vector at zero and reserve it
% for differential control offsets.
controller_trim_motor_rpms = zeros(4, 1);
controller_trim_tilt_angles = [front_tilt_group; front_tilt_group];
controller_trim_motor_rpm_cmd = zeros(12, 1);
controller_trim_tilt_angles_cmd = front_tilt_group * ones(6, 1);
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
controller_sample_time = 0.01;
controller_k_p_damp = 0.0;
controller_k_q_damp = 0.0;
controller_k_r_damp = 0.0;
controller_k_phi_p = 0.0;
controller_k_phi_i = 0.0;
controller_k_v_p = 0.0;
controller_k_v_i = 0.0;

% Sensor / estimator defaults. The first pass uses perfect sensors by
% default, but the additive bias/noise paths are exposed through startup
% flags. Bias defaults to zero; repeatable pseudo-noise is on by default.
sensor_enable_bias = true;
sensor_enable_noise = true;
sensor_airdata_bias = zeros(3, 1); % [airspeed; alpha; beta]
sensor_gyro_bias = zeros(3, 1);
sensor_accel_bias = zeros(3, 1);
sensor_gps_pos_bias = zeros(3, 1);
sensor_gps_vel_bias = zeros(3, 1);
sensor_mag_bias = zeros(3, 1);
sensor_attitude_bias = zeros(3, 1);
sensor_airdata_sigma = [0.20; deg2rad(0.10); deg2rad(0.10)];
sensor_gyro_sigma = deg2rad([0.05; 0.05; 0.05]);
sensor_accel_sigma = [0.05; 0.05; 0.05];
sensor_gps_pos_sigma = [0.50; 0.50; 0.50];
sensor_gps_vel_sigma = [0.05; 0.05; 0.05];
sensor_mag_sigma = [0.005; 0.005; 0.005];
sensor_attitude_sigma = deg2rad([0.10; 0.10; 0.15]);
mag_ned = [0.23; 0.00; 0.43];
sensor_sample_time = controller_sample_time;

estimator_enable = true;
estimator_init_eul = eul_init;
estimator_k_acc = 3.0;
estimator_k_mag = 1.5;
estimator_k_att = 0.0;
estimator_sample_time = controller_sample_time;

% Aerodynamic model selection for the top-level Simulink switch:
% false => existing analytical surface-based branch
% true  => compact aircraft-level AVL fit branch
use_avl_aero = false;
avlAero = load_avl_aero_model(script_dir);

% The AVL Simulink branch uses avlAero as a typed bus signal so the model
% can compile without flattening the entire fit structure into scalar
% constants.
if evalin('base', 'exist(''avlAeroBus'', ''var'')')
    evalin('base', 'clear avlAeroBus');
end
busInfo = Simulink.Bus.createObject(avlAero);
if evalin('base', sprintf('exist(''%s'', ''var'')', busInfo.busName))
    avlAeroBus = evalin('base', busInfo.busName);
    if ~strcmp(busInfo.busName, 'avlAeroBus')
        evalin('base', sprintf('clear %s', busInfo.busName));
    end
else
    error('Full_Sim_Init:MissingBusObject', ...
        'Simulink.Bus.createObject returned %s, but that variable was not found.', ...
        busInfo.busName);
end
assignin('base', 'avlAeroBus', avlAeroBus);

%% =========================================================================
%% 2. 3D VISUALIZATION
%% =========================================================================
if render_enable
    disp('Rendering 3D Aircraft Model...');

    render_aircraft(compData, CG, tilt_angle, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', prop, ...
        'controls', controls, ...
        'thrust_tilt_deg', Tilt_angles, ...
        'title_text', sprintf('%s (Scenario: %s)', 'Brown eVTOL Aircraft', test_case));
end
