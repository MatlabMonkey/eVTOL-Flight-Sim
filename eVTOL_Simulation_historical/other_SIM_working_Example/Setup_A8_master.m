clearvars;
clc;

TrimAndLinearize_ATT_wInputs_Aircraft6dof;

A8 = struct();

%% Sim time
A8.startTime = 0;
A8.stopTime  = 20;
A8.stepTime  = 0.01;

%% Initial Conditions
% Define Wind Disturbance and Earth Model
A8.init.v_N_AE=[0; 0; 0];
A8.init.g_E_N=[0; 0; 9.81]*1;
A8.init.Rearth = 6371000;

% Initial p_E_E_BE=[lat; lon; alt] 
InitPos_deg = [37.6286; -122.393; 10]; % San Fransisco Airport
GoogleProvideLatLonAltOfUsBankInLa = [ 34.051122; -118.254399; 310 ];
GuessAndCheckToGetPlaneOnUsBankInLa = [34.05105; -118.25439; 415];
FlightGearOffset = GuessAndCheckToGetPlaneOnUsBankInLa - GoogleProvideLatLonAltOfUsBankInLa;

InitPos_deg = GoogleProvideLatLonAltOfUsBankInLa + FlightGearOffset;

A8.init.InitPos = InitPos_deg .* [deg2rad; deg2rad; 1];
InitPos = A8.init.InitPos;

% Initial x_E_E_BE=[x; y; z] derives from p_E_E_BE
x_E_E_BE0 = (A8.init.Rearth + InitPos(3)) * [ ...
    cos(InitPos(1))*cos(InitPos(2)); ...
    cos(InitPos(1))*sin(InitPos(2)); ...
    sin(InitPos(1))];


% Initial att_E_E_BE=[roll; pitch; yaw]=[phi; theta; psi]
InitAtt_deg = Att_Trim_deg + [10;10;0];
InitAtt = InitAtt_deg * deg2rad;

% Initial q_E_E_BE=[q0; q1; q2; a3] derives from att_E_E_BE
InitQuat(1) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(2) = sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(3) = cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(4) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2);

A8.init.InitQuat = InitQuat;

% Initial v_B_B_BE=[u; v; w] 
A8.init.InitVel = Vel_B_BA_Trim + [0;0;0]; 

% Initial om_B_B_BE=[P; Q; R] 
A8.init.InitRates = Rates_Trim + [0;0;0];

% Problem 2 - equivalent internal rotating assembly for 6 props
Nprops_eq  = 6;
Jprop_each = 1.5e-4;     % kg*m^2
axis_body  = [1; 0; 0];  % body x-axis
Jeq        = Nprops_eq * Jprop_each;

om_motor_init = 0;
omDot_motor   = 0;

% Variables used directly by the plant model
A8.init.motor.J_int(1)      = Jeq;
n_int(:,1)    = axis_body;
n_p1 = axis_body; % For now
A8.init.motor.om_motor_init = om_motor_init;
A8.init.motor.omDot_motor   = omDot_motor;

% Define Motor Parameters
A8.init.motor.Rmotor = 0.4;         %Ohms
A8.init.motor.Io_motor = 1.6;       %Amps
A8.init.motor.Ki_motor = 0.009549;  %N.m/Amp

%{
% The first rotational internal component is the motor
A8.init.motor.J_int(1) = 0.01;
A8.init.motor.omDot_motor = 0;
A8.init.motor.om_motor_init = 0;
%}

% Define Battery Parameters
A8.init.battery.nCellsSeries = 6; % cells
A8.init.battery.VcellMax=4.2; %Volts
%Rcell=0.3; %Ohms
%nPacksParallel = inf;
%SocVec=[-0.5;0;0.1;0.75;1.0];
%dVoltageVec=[VcellMax;1.2;0.6;0.4;0.0];

%% Trim
% Perfect actuators: 1
% Lagged actuators: 0
A8.trim.usePerfectAct = 1;

A8.trim.att_rad      = Att_Trim;
A8.trim.velBody_mps  = Vel_B_BA_Trim;
A8.trim.rates_radps  = Rates_Trim;
A8.trim.surfCmd_rad  = U_trim(:);

A8.trim.dER0 = X_trim(states_order(10));
A8.trim.dEL0 = X_trim(states_order(11));
A8.trim.dE0  = X_trim(states_order(12));
A8.trim.dR0  = X_trim(states_order(13));

A8.trim.tauER = 0.05;
A8.trim.tauEL = 0.05;
A8.trim.tauE = 0.05;
A8.trim.tauR = 0.05;

% Small floor just so 1/tau never divides by zero in the lag path
A8.trim.tauMin = 1e-3;

% Position limits
A8.trim.dER_min = deg2rad * -25;
A8.trim.dER_max = deg2rad *  25;

A8.trim.dEL_min = deg2rad * -25;
A8.trim.dEL_max = deg2rad *  25;

A8.trim.dE_min  = deg2rad * -25;
A8.trim.dE_max  = deg2rad *  25;

A8.trim.dR_min  = deg2rad * -25;
A8.trim.dR_max  = deg2rad *  25;

% Rate limits
A8.trim.dERdot_max = deg2rad * 250;
A8.trim.dELdot_max = deg2rad * 250;
A8.trim.dEdot_max  = deg2rad * 250;
A8.trim.dRdot_max  = deg2rad * 250;

A8.trim.MixMatrix = [-1 1 0 0;
                    1 1 0 0;
                    0 0 1 0;
                    0 0 0 1] ;

A8.trim.DeMixMatrix = inv(A8.trim.MixMatrix);

% Throttle
A8.trim.throttle_cmd = 0.0;
A8.trim.dT1_min = 0.0;
A8.trim.dT1_max = 1.0;

%% Sensor
A8.sensor.sampleTime = A8.stepTime;

% This is just a numerical safety floor so alpha and beta do not blow up.
A8.sensor.minAirspeed_mps = 0.25;

% Air-data A8.sensors: [Vinf; alpha; beta]
A8.sensor.airdata.bias = [0; 0; 0];
A8.sensor.airdata.sigma = [0; 0; 0];

% Body-rate gyros: [P; Q; R]
A8.sensor.gyro.bias = [0; 0; 0];
A8.sensor.gyro.sigma = [0; 0; 0];

% Accelerometers: [ax; ay; az]
A8.sensor.accel.bias = [0; 0; 0];
A8.sensor.accel.sigma = [0; 0; 0];

% GPS position and velocity.
A8.sensor.gps.posBias = [0; 0; 0];
A8.sensor.gps.posSigma = [0; 0; 0];
A8.sensor.gps.velBias = [0; 0; 0];
A8.sensor.gps.velSigma = [0; 0; 0];

% Attitude A8.sensor: [phi; theta; psi]
A8.sensor.att.bias = [0; 0; 0];
A8.sensor.att.sigma = [0; 0; 0];

% Magnetometer.
A8.sensor.magField_NED = [1; 0; 0];
A8.sensor.mag.bias = [0; 0; 0];
A8.sensor.mag.sigma = [0; 0; 0];

% Randomness seeds.
A8.sensor.seed.airdata = 11;
A8.sensor.seed.gyro = 21;
A8.sensor.seed.accel = 31;
A8.sensor.seed.gpsPos = 41;
A8.sensor.seed.gpsVel = 51;
A8.sensor.seed.att = 61;
A8.sensor.seed.mag = 71;

%% Controller
% Switch selector
A8.gains.on = 3;  % 1=OpenLoop, 2=PID, 3=LQR

A8.gains.roll.Kp_phi  = 0.8;
A8.gains.roll.Ki_phi  = 0.05;
A8.gains.roll.Kp_damp = 1.0;

A8.gains.pitch.Kp_theta = 0.8;
A8.gains.pitch.Ki_theta = 0.05;
A8.gains.pitch.Kq_damp  = 1.0;

A8.gains.yaw.Kp_psi  = 0.0;
A8.gains.yaw.Ki_psi  = 0.0;
A8.gains.yaw.Kr_damp = 0.2;

% Throttle / speed
A8.gains.throttle.Kp_V = 0.0;
A8.gains.throttle.Ki_V = 0.0;


%% Steps / command histories for From Workspace blocks
% Go to trim from ICs: 1
% Use these commands: 2
A8.step.useCMDs = 2; % Only for LQR
t = (A8.startTime:A8.stepTime:A8.stopTime).';
N = numel(t);

phi_cmd   = Att_Trim(1) * ones(N, 1);
theta_cmd = Att_Trim(2) * ones(N, 1);
psi_cmd   = Att_Trim(3) * ones(N, 1);


% Perturbs
tStep = 5.0;

phiStepDeg = 5.0;
thetaStepDeg = 10.0;
psiStepDeg = 5.0;

phi_cmd(t >= tStep) = phi_cmd(t >= tStep) + deg2rad * phiStepDeg;
theta_cmd(t >= tStep) = theta_cmd(t >= tStep) + deg2rad * thetaStepDeg;
psi_cmd(t >= tStep) = psi_cmd(t >= tStep) + deg2rad * psiStepDeg;

% Trim commands repeated for all time
rates_trim_row   = Rates_Trim(:).';
airdata_trim_row = Vel_W_Trim(:).';

A8.step.attDes     = [t, phi_cmd, theta_cmd, psi_cmd];
A8.step.ratesDes   = [t, repmat(rates_trim_row,   N, 1)];
A8.step.airDataDes = [t, repmat(airdata_trim_row, N, 1)];
A8.step.accelDes   = [t, zeros(N,3)];

A8.step.omDot_motor = [t, zeros(N,1)];

% small pulse in internal rotor acceleration
A8.step.omDot_motor(t >= 5.0 & t < 5.5, 2) = 40;
A8.step.omDot_motor(t >= 5.5 & t < 6.0, 2) = -40;


run_model_here = true;

if run_model_here
    assignin('base', 'A8', A8);
    open_system('Aircraft6dof_V2');
    Level_flight_out = sim('Aircraft6dof_V2');
    save('Level_flight_out.mat', 'Level_flight_out');
end