% Full_Sim_Init.m
% 
%% Simulation Timing
startTime = 0;
stopTime = 30;
stepTime = 0.01;

%% Aircraft definition
aircraft = aircraft_def('flight_mode', 0);

% Export the same workspace variables that the Simulink models expect.
structural_tilt_angle = aircraft.tilt_angle;

rho = aircraft.rho;

prop = aircraft.prop;
compData = aircraft.compData;
aeroData = aircraft.aeroData;
controls = aircraft.controls; % Metadata for ruddervators / future flaperons

% For control mixing
MixMatrix = aircraft.MixMatrix;
DeMixMatrix = aircraft.DeMixMatrix;

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

%% Scenario Init
% Test scenario name
name = "Flight_Test";

% Init propellor stuff
tilt_angle_deg = 90;        % Degrees
motor_rpm_value = 1500;     % rpm

% Propellor vector
Motor_RPMs = motor_rpm_value * ones(12, 1);
Tilt_angles = tilt_angle_deg * ones(6, 1);

% Controller scaffold variables used by the optional Simulink PD block.
controller_enable = false;
front_rpm_group = mean(Motor_RPMs(1:min(6, numel(Motor_RPMs))));
rear_rpm_group = mean(Motor_RPMs(min(7, numel(Motor_RPMs)):end));
if isempty(rear_rpm_group) || isnan(rear_rpm_group)
    rear_rpm_group = front_rpm_group;
end
front_tilt_group = mean(Tilt_angles(:));

% Flight mode and tilt angle
tilt_angle = mean(Tilt_angles(:));
flight_mode = double(tilt_angle >= 45);

% Gravity on or off
g = 9.81;

% Forces and Moments
Fext_B = [0; 0; 0];
Mext_B = [0; 0; 0];

% Initial p_E_E_BE=[lat; lon; alt] 
% Your desired starting location (Downtown LA)
lla_init = [ 34.05105, -118.25439, 1000 ]; 

% The local [0,0,0] origin of your simulation (e.g., LAX runway)
lla_ref = [ 33.9416, -118.4085, 38 ]; 

% Convert to NED using the Navigation Toolbox
pos_init = lla2ned(lla_init, lla_ref, 'flat'); 

% Ensure it's a column vector if your simulation requires it
pos_init = pos_init(:);

% Velocity u v w
V_init = [75.0; 0.0; 0.0];

% Attitude phi theta psi 
% Degrees
phi = 0;
theta = 0;
psi = 0;

eul_init = [phi; theta; psi] * (pi / 180);

% Rates P Q R
P = 0.0;
Q = 0.0;
R = 0.0;
omega_init = [P; Q; R];

% For memory
% Total velocity
V_mem_init = [norm(V_init); ...
              atan2(V_init(3), V_init(1)); ...
              asin(V_init(2) / max(norm(V_init), 0.1))];


accel_mem_init = [0;0;0];

%% Controller init
% The current Propellers subsystem is grouped as:
%   motor_rpms = [FR; FL; RR; RL]
%   tilt_angles = [FR; FL]
controller_base_rpm = mean([front_rpm_group; rear_rpm_group]);
controller_mode = 3; % 1=open loop, 2=PID

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
controller_surface_limit_rad = deg2rad * 25.0;
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

%% Sensors init
sensor_sample_time = stepTime;

% This is just a numerical safety floor so alpha and beta do not blow up.
sensor_minAirspeed_mps = 0.25;

% Air-data sensors: [Vinf; alpha; beta]
sensor_airdata_bias = [0; 0; 0];
sensor_airdata_sigma = [0; 0; 0];

% Body-rate gyros: [P; Q; R]
sensor_gyro_bias = [0; 0; 0];
sensor_gyro_sigma = [0; 0; 0];

% Accelerometers: [ax; ay; az]
sensor_accel_bias = [0; 0; 0];
sensor_accel_sigma = [0; 0; 0];

% GPS position and velocity_
sensor_gps_posBias = [0; 0; 0];
sensor_gps_posSigma = [0; 0; 0];
sensor_gps_velBias = [0; 0; 0];
sensor_gps_velSigma = [0; 0; 0];

% Attitude sensor: [phi; theta; psi]
sensor_att_bias = [0; 0; 0];
sensor_att_sigma = [0; 0; 0];

% Magnetometer_
sensor_magField_NED = [1; 0; 0];
sensor_mag_bias = [0; 0; 0];
sensor_mag_sigma = [0; 0; 0];

% Randomness seeds_
sensor_seed_airdata = 11;
sensor_seed_gyro = 21;
sensor_seed_accel = 31;
sensor_seed_gpsPos = 41;
sensor_seed_gpsVel = 51;
sensor_seed_att = 61;
sensor_seed_mag = 71;

%% Command histories for From Workspace blocks
cmds = struct();

t = (startTime:stepTime:stopTime).';
N = numel(t);

% AirData 

Vinf_cmd   = V_init(1) * ones(N, 1);
alpha_cmd  = V_init(2) * ones(N, 1);
beta_cmd   = V_init(3) * ones(N, 1);

cmds.airData_cmd = [t, Vinf_cmd, alpha_cmd, beta_cmd];

% Euler angles
phi_cmd   = eul_init(1) * ones(N, 1);
theta_cmd = eul_init(2) * ones(N, 1);
psi_cmd   = eul_init(3) * ones(N, 1);

% Perturbs time or function this is an example make higher than stopTime to
% not engage. This is an example of a step.
tStep = 100.0;

phiStepDeg = 5.0;
thetaStepDeg = 10.0;
psiStepDeg = 5.0;

phi_cmd(t >= tStep) = phi_cmd(t >= tStep) + deg2rad * phiStepDeg;
theta_cmd(t >= tStep) = theta_cmd(t >= tStep) + deg2rad * thetaStepDeg;
psi_cmd(t >= tStep) = psi_cmd(t >= tStep) + deg2rad * psiStepDeg;

cmds.eul_cmd = [t, phi_cmd, theta_cmd, psi_cmd];


% GPS Position This is NED for now
north_cmd = pos_init(1) * ones(N, 1);
east_cmd = pos_init(2) * ones(N, 1);
beta_cmd = pos_init(3) * ones(N, 1);

cmds.gps_Pos_cmd = [t, north_cmd, east_cmd, beta_cmd];

% Omega commands
P_cmd = omega_init(1) * ones(N, 1);
Q_cmd = omega_init(2) * ones(N, 1);
R_cmd = omega_init(3) * ones(N, 1);

cmds.omega_cmd = [t, P_cmd, Q_cmd, R_cmd];


% Accel commands
ax_cmd = zeros(N, 1);
ay_cmd = zeros(N, 1);
az_cmd = zeros(N, 1);

cmds.accel_cmd = [t, ax_cmd, ay_cmd, az_cmd];

% Motor RPM commands
motor_1_cmd = Motor_RPMs(1) * ones(N, 1);
motor_2_cmd = Motor_RPMs(2) * ones(N, 1);
motor_3_cmd = Motor_RPMs(3) * ones(N, 1);
motor_4_cmd = Motor_RPMs(4) * ones(N, 1);
motor_5_cmd = Motor_RPMs(5) * ones(N, 1);
motor_6_cmd = Motor_RPMs(6) * ones(N, 1);
motor_7_cmd = Motor_RPMs(7) * ones(N, 1);
motor_8_cmd = Motor_RPMs(8) * ones(N, 1);
motor_9_cmd = Motor_RPMs(9) * ones(N, 1);
motor_10_cmd = Motor_RPMs(10) * ones(N, 1);
motor_11_cmd = Motor_RPMs(11) * ones(N, 1);
motor_12_cmd = Motor_RPMs(12) * ones(N, 1);

cmds.motor_cmd = [t, motor_1_cmd, motor_2_cmd, motor_3_cmd, motor_4_cmd,... 
                     motor_5_cmd, motor_6_cmd, motor_7_cmd, motor_8_cmd,...
                     motor_9_cmd, motor_10_cmd, motor_11_cmd, motor_12_cmd];

% Tilt angles
tilt_1_cmd = Tilt_angles(1) * ones(N, 1);
tilt_2_cmd = Tilt_angles(2) * ones(N, 1);
tilt_3_cmd = Tilt_angles(3) * ones(N, 1);
tilt_4_cmd = Tilt_angles(4) * ones(N, 1);
tilt_5_cmd = Tilt_angles(5) * ones(N, 1);
tilt_6_cmd = Tilt_angles(6) * ones(N, 1);


cmds.tilt_cmd = [t, tilt_1_cmd, tilt_2_cmd, tilt_3_cmd, tilt_4_cmd, tilt_5_cmd, tilt_6_cmd];

% Front Command
front_rpm_cmd = front_rpm_group * ones(N, 1);

cmds.front_cmd = [t, front_rpm_cmd];

% Rear Command
rear_rpm_cmd = rear_rpm_group(1) * ones(N, 1);

cmds.rear_cmd = [t, rear_rpm_cmd];
assignin('base','cmds', cmds);
%% Aerodynamic model selection for the top-level Simulink switch:
% false => existing analytical surface-based branch
% true  => compact aircraft-level AVL fit branch
use_avl_aero = false;
avlAero = load_avl_aero_model();

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

%% 3D VISUALIZATION
if render_enable
    disp('Rendering 3D Aircraft Model...');

    render_aircraft(compData, CG, tilt_angle, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', prop, ...
        'controls', controls, ...
        'thrust_tilt_deg', Tilt_angles, ...
        'title_text', sprintf('%s (Scenario: %s)', 'Brown eVTOL Aircraft', test_case));
end
