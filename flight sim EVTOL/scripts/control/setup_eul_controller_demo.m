%% EUL Controller Demo Setup
% Run this script to populate the workspace for the Euler-angle wrapper and
% controller demos. It computes a fresh trim point, applies the trim to the
% legacy plant/workspace variables, and defines the grouped structs used by
% the controller and sensor blocks.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupDir = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run('Full_Sim_Init.m');

%% Trim operating point
trim = struct();
trim.model = 'Brown_6DOF_Plant_EUL';
trim.cruise_speed_mps = 70;
trim.tilt_deg = 90;
trim.altitude_ned_m = -1000;

trimPoint = find_trim_point_eul_simple( ...
    'Model', trim.model, ...
    'CruiseSpeed', trim.cruise_speed_mps, ...
    'TiltDeg', trim.tilt_deg, ...
    'AltitudeNED', trim.altitude_ned_m, ...
    'DisplayReport', 'off');

apply_trim_point_to_workspace(trimPoint);

%% Controller gains
gains = struct();

gains.V = struct();
gains.V.Kp = 100;
gains.V.Ki = 5;

gains.phi = struct();
gains.phi.Kp = 2;     % Assumed bank-angle proportional gain; confirm/tune in Simulink.
gains.phi.Kd = -2;    % Roll-rate damping term on p. Negative sign is intentional.

gains.theta = struct();
gains.theta.Kp = 0.10; % Pitch-angle hold term on (theta_cmd - theta).
gains.theta.Kd = 0.1;  % Pitch-rate damping term on q. Positive sign matches delta_e = trim + Kd*q - Kp*(theta_cmd-theta).

%% Saturation limits
sat = struct();

sat.frontcollective = struct();
sat.frontcollective.low = 0;
sat.frontcollective.high = 3000;

sat.delta_a = struct();
sat.delta_a.low = deg2rad(-25);
sat.delta_a.high = deg2rad(25);

sat.delta_e = struct();
sat.delta_e.low = deg2rad(-25);
sat.delta_e.high = deg2rad(25);

%% Sensor settings
sensors = struct();
NoiseOn = 1;
sensor_sample_time = 0.001;

sensors.airspeed = makeScalarSensor(0.5, sensor_sample_time, 0.0);
sensors.alpha = makeScalarSensor(deg2rad(0.2), sensor_sample_time, 0.0);
sensors.beta = makeScalarSensor(deg2rad(0.2), sensor_sample_time, 0.0);

sensors.omega = makeVectorSensor(deg2rad([0.05; 0.05; 0.05]), sensor_sample_time, [0; 0; 0]);
sensors.accel = makeVectorSensor([0.05; 0.05; 0.05], sensor_sample_time, [0; 0; 0]);
sensors.pos = makeVectorSensor([1.0; 1.0; 1.5], sensor_sample_time, [0; 0; 0]);
sensors.vel = makeVectorSensor([0.1; 0.1; 0.1], sensor_sample_time, [0; 0; 0]);
sensors.eul = makeVectorSensor(deg2rad([0.2; 0.2; 0.5]), sensor_sample_time, [0; 0; 0]);

%% Base set point
base_set_point = makeBaseSetPoint(trimPoint);

%% Truth-memory initial conditions
truth_ic = struct();
truth_ic.pos_NED = trimPoint.position(:);
truth_ic.V_B_truth = trimPoint.velocity(:);
truth_ic.V_E_truth = [trimPoint.vinf; 0; 0];
truth_ic.eul_truth = trimPoint.eul(:);
truth_ic.omega_truth = trimPoint.omega(:);
truth_ic.C_NB_truth = localEulToDCM(trimPoint.eul(:));
truth_ic.V_BA_truth = trimPoint.velocity(:);
truth_ic.vinf_truth = trimPoint.vinf;
truth_ic.alpha_truth = atan2(trimPoint.velocity(3), trimPoint.velocity(1));
truth_ic.beta_truth = 0.0;
truth_ic.specific_force_truth = [0; 0; 0];

%% Command-delta defaults
command_delta = makeCommandDelta();

%% Command profile
cmd_profile = makeCommandProfile(stepDefaults());

%% Step command defaults
step = struct();

step.longitudinal = struct();
step.longitudinal.time = 3.0;
step.longitudinal.d_vinf_cmd = 5.0;

step.lateral = struct();
step.lateral.time = 3.0;
step.lateral.d_phi_cmd = deg2rad(5.0);

step.pitch = struct();
step.pitch.time = 3.0;
step.pitch.d_theta_cmd = 0.0;

cmd_profile.vinf.enable = true;
cmd_profile.vinf.time = step.longitudinal.time;
cmd_profile.vinf.magnitude = step.longitudinal.d_vinf_cmd;

cmd_profile.phi.enable = false;
cmd_profile.phi.time = step.lateral.time;
cmd_profile.phi.magnitude = step.lateral.d_phi_cmd;

cmd_profile.theta.enable = false;
cmd_profile.theta.time = step.pitch.time;
cmd_profile.theta.magnitude = step.pitch.d_theta_cmd;

%% Controller mode switches
controller = struct();
controller.enable = true;
controller.mode = 3;

%% Logged bus indices
log_idx = struct();

log_idx.sensor_bus = struct( ...
    'airspeed', 1, ...
    'alpha', 2, ...
    'beta', 3, ...
    'gyro', 4:6, ...
    'accel', 7:9, ...
    'gps_pos', 10:12, ...
    'gps_vel', 13:15, ...
    'attitude', 16:18);

log_idx.plant_cmd_bus = struct( ...
    'motor_rpm_cmd', 1:12, ...
    'tilt_angles_cmd', 13:18, ...
    'front_collective_rpm', 19, ...
    'rear_collective_rpm', 20, ...
    'delta_f_cmd', 21, ...
    'delta_a_cmd', 22, ...
    'delta_e_cmd', 23, ...
    'delta_r_cmd', 24);

%% Bus objects for Constant blocks
baseSetPointBusName = createBusObjectFromStruct(base_set_point, 'BaseSetPointBus');
cmdProfileBusName = createBusObjectFromStruct(cmd_profile, 'CmdProfileBus');
commandDeltaBusName = createBusObjectFromStruct(command_delta, 'CommandDeltaBus');
truth_bus_template = makeTruthBusTemplate();
truthBusName = createBusObjectFromStruct(truth_bus_template, 'TruthBus');

%% Display summary
trimSummary = table( ...
    trimPoint.inputs.front_collective_rpm, ...
    trimPoint.inputs.rear_collective_rpm, ...
    rad2deg(trimPoint.inputs.delta_e), ...
    rad2deg(trimPoint.eul(2)), ...
    trimPoint.vinf, ...
    'VariableNames', { ...
    'front_collective_rpm', ...
    'rear_collective_rpm', ...
    'delta_e_deg', ...
    'theta_deg', ...
    'vinf_mps'});

disp('Trim point loaded into workspace:')
disp(trimSummary)

disp('Controller gains:')
disp(gains)

disp('Sensor settings:')
disp(sensors)

disp('Truth-memory initial conditions:')
disp(truth_ic)

disp('Logged bus indices:')
disp(log_idx)

disp('Bus objects:')
disp(struct( ...
    'base_set_point', baseSetPointBusName, ...
    'cmd_profile', cmdProfileBusName, ...
    'command_delta', commandDeltaBusName, ...
    'truth_bus', truthBusName))

function sensor = makeScalarSensor(sigma, Ts, bias)
sensor = struct();
sensor.sigma = sigma;
sensor.Ts = Ts;
sensor.bias = bias;
end

function sensor = makeVectorSensor(sigma, Ts, bias)
sensor = struct();
sensor.sigma = sigma(:);
sensor.Ts = Ts;
sensor.bias = bias(:);
end

function set_point = makeBaseSetPoint(trimPoint)
set_point = struct();

set_point.vinf_cmd = trimPoint.vinf;
set_point.euler_cmd = trimPoint.eul(:);
set_point.omega_cmd = trimPoint.omega(:);
set_point.v_b_cmd = trimPoint.velocity(:);

set_point.motor_rpm_cmd = trimPoint.inputs.motor_rpm_cmd(:);
set_point.tilt_angles_cmd = trimPoint.inputs.tilt_angles_cmd(:);

set_point.front_collective_rpm = trimPoint.inputs.front_collective_rpm;
set_point.rear_collective_rpm = trimPoint.inputs.rear_collective_rpm;

set_point.delta_f_cmd = trimPoint.inputs.delta_f;
set_point.delta_a_cmd = trimPoint.inputs.delta_a;
set_point.delta_e_cmd = trimPoint.inputs.delta_e;
set_point.delta_r_cmd = trimPoint.inputs.delta_r;
end

function cmd = makeCommandDelta()
cmd = struct();
cmd.d_vinf_cmd = 0.0;
cmd.d_phi_cmd = 0.0;
cmd.d_theta_cmd = 0.0;
cmd.d_p_cmd = 0.0;
cmd.d_q_cmd = 0.0;
cmd.d_r_cmd = 0.0;
end

function profile = makeCommandProfile(defaults)
profile = struct();
profile.vinf = defaults.vinf;
profile.phi = defaults.phi;
profile.theta = defaults.theta;
end

function defaults = stepDefaults()
defaults = struct();

defaults.vinf = struct();
defaults.vinf.enable = false;
defaults.vinf.type = 1;
defaults.vinf.time = 3.0;
defaults.vinf.magnitude = 0.0;

defaults.phi = struct();
defaults.phi.enable = false;
defaults.phi.type = 1;
defaults.phi.time = 3.0;
defaults.phi.magnitude = 0.0;

defaults.theta = struct();
defaults.theta.enable = false;
defaults.theta.type = 1;
defaults.theta.time = 3.0;
defaults.theta.magnitude = 0.0;
end

function C_NB = localEulToDCM(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);

cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

C_NB = [ ...
    cth*cpsi,                          cth*spsi,                         -sth; ...
    sphi*sth*cpsi - cphi*spsi,        sphi*sth*spsi + cphi*cpsi,       sphi*cth; ...
    cphi*sth*cpsi + sphi*spsi,        cphi*sth*spsi - sphi*cpsi,       cphi*cth];
end

function busName = createBusObjectFromStruct(value, desiredName)
busObj = buildBusRecursive(value, desiredName);
assignin('base', desiredName, busObj);
busName = char(desiredName);
end

function busObj = buildBusRecursive(value, busName)
fields = fieldnames(value);
elements = repmat(Simulink.BusElement, numel(fields), 1);

for i = 1:numel(fields)
    name = fields{i};
    fieldValue = value.(name);

    elem = Simulink.BusElement;
    elem.Name = name;
    elem.SampleTime = -1;

    if isstruct(fieldValue)
        childBusName = [busName '_' name];
        childBusObj = buildBusRecursive(fieldValue, childBusName);
        assignin('base', childBusName, childBusObj);
        elem.DataType = ['Bus: ' childBusName];
        elem.Dimensions = 1;
    else
        if islogical(fieldValue)
            elem.DataType = 'boolean';
        else
            elem.DataType = class(fieldValue);
        end
        dims = size(fieldValue);
        if isequal(dims, [1 1])
            elem.Dimensions = 1;
        else
            elem.Dimensions = dims;
        end
    end

    elements(i) = elem;
end

busObj = Simulink.Bus;
busObj.Elements = elements;
end

function truth = makeTruthBusTemplate()
truth = struct();
truth.pos_NED = zeros(3,1);
truth.V_B_truth = zeros(3,1);
truth.V_E_truth = zeros(3,1);
truth.eul_truth = zeros(3,1);
truth.omega_truth = zeros(3,1);
truth.C_NB_truth = zeros(3,3);
truth.V_BA_truth = zeros(3,1);
truth.vinf_truth = 0.0;
truth.alpha_truth = 0.0;
truth.beta_truth = 0.0;
truth.specific_force_truth = zeros(3,1);
end
