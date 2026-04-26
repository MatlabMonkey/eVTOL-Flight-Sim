% run_evtol_cruise_scenario.m
%
% Top-level runner for the EVTOL cruise design/simulation workflow.
%   (1) Trim EVTOL_6DOF_Plant_EUL at steady level cruise
%   (2) Linearize about the trim point
%   (3) Design the 9-state LQR regulator
%   (4) Build an initial condition + step-command scenario
%   (5) Run EVTOL_6DOF_Sim_Wrapper closed loop

clearvars; clc;

%% Trim, linearize
TrimAndLinearize_EVTOL_Cruise;

%% DESIGN LQR
Design_LQR_Controller_EVTOL_Cruise;

%% Scenario setup

% Sim timing — keep consistent with the Setup script
EVTOL.startTime = 0;
EVTOL.stopTime  = 300;
EVTOL.stepTime  = 0.01;

startTime = EVTOL.startTime;
stopTime  = EVTOL.stopTime;
stepTime  = EVTOL.stepTime;

%% Initial conditions: trim + small attitude perturbation
% This verifies the LQR pulls the vehicle back to trim from a disturbed
% initial attitude, on top of any commanded steps.
InitAtt_offset_deg = [0.0; 0.0; 0.0];                  % [phi, theta, psi]
InitAtt_deg        = Att_Trim_deg + InitAtt_offset_deg;
InitAtt            = InitAtt_deg * deg2rad;            % rad

% Euler -> quaternion (ZYX, matches the reference example's convention)
cphi = cos(InitAtt(1)/2);  sphi = sin(InitAtt(1)/2);
cthe = cos(InitAtt(2)/2);  sthe = sin(InitAtt(2)/2);
cpsi = cos(InitAtt(3)/2);  spsi = sin(InitAtt(3)/2);
InitQuat = zeros(4,1);
InitQuat(1) = cphi*cthe*cpsi - sphi*sthe*spsi;
InitQuat(2) = sphi*cthe*cpsi - cphi*sthe*spsi;
InitQuat(3) = cphi*sthe*cpsi - sphi*cthe*spsi;
InitQuat(4) = cphi*cthe*spsi - sphi*sthe*cpsi;

% Push the ICs into variables the Simulink model reads on start-up.
eul_init   = InitAtt;
V_init     = Vel_B_BA_Trim;
omega_init = Rates_Trim;

% For memory
% Total velocity
Vinf_memory = sqrt(V_init(1)^2 + V_init(2)^2 + V_init(3)^2);

% Angle of attack (using atan2 for 4-quadrant stability)
alpha_memory = atan2(V_init(3)^2, V_init(1));

% Sideslip angle
if Vinf_memory == 0
    beta_memory = 0;
else
    beta_memory = asin(V_init(2) / Vinf_memory);
end
V_mem_init = [Vinf_memory; alpha_memory; beta_memory];

accel_mem_init = [0;0;0];

EVTOL.init.eul   = eul_init;
EVTOL.init.vel   = V_init;
EVTOL.init.rates = omega_init;
EVTOL.init.quat  = InitQuat;
EVTOL.init.pos   = pos_init;

%% Controller mode and gains
controller_mode = 3;   % 1 = open loop, 2 = PID, 3 = LQR
useCMDs = 2;           % 1 = hold trim, 2 = use command inputs

EVTOL.gains.mode       = controller_mode;
EVTOL.gains.K_lqr      = K_lqr_cruise;
EVTOL.gains.x_trim_lqr = x_trim_lqr;
EVTOL.gains.U_trim_lqr = U_trim_lqr;

%% Command histories
t = (startTime : stepTime : stopTime).';
N = numel(t);

% Attitude step injected at tStep
tStep        = 150.0;   % s
phiStepDeg   = 5.0;
thetaStepDeg = 5.0;
psiStepDeg   = 0.0;

phi_cmd   = Att_Trim(1) * ones(N, 1);
theta_cmd = Att_Trim(2) * ones(N, 1);
psi_cmd   = Att_Trim(3) * ones(N, 1);

step_mask = (t >= tStep);
phi_cmd(step_mask)   = phi_cmd(step_mask)   + phiStepDeg   * deg2rad;
theta_cmd(step_mask) = theta_cmd(step_mask) + thetaStepDeg * deg2rad;
psi_cmd(step_mask)   = psi_cmd(step_mask)   + psiStepDeg   * deg2rad;

% cmds struct — timeseries with leading time column for From-Workspace blocks
cmds = struct();
cmds.eul_cmd     = [t, phi_cmd, theta_cmd, psi_cmd];
cmds.omega_cmd   = [t, repmat(Rates_Trim.', N, 1)];
cmds.airData_cmd = [t, repmat(Vel_W_Trim.', N, 1)];
cmds.accel_cmd   = [t, zeros(N, 3)];

cmds.front_cmd   = [t, U_trim_lqr(1) * ones(N, 1)];    % trim RPM
cmds.rear_cmd    = [t, zeros(N, 1)];                   % rear rotors OFF

cmds.tilt_cmd    = [t, front_tilt_trim_deg * ones(N, 6)];
cmds.motor_cmd   = [t, zeros(N, 12)];

cmds.gps_Pos_cmd = [t, pos_init(1)*ones(N,1), ...
                       pos_init(2)*ones(N,1), ...
                       pos_init(3)*ones(N,1)];

% Push structs to base workspace so Simulink can find them
assignin('base', 'cmds',            cmds);
assignin('base', 'EVTOL',           EVTOL);
assignin('base', 'controller_mode', controller_mode);
assignin('base', 'K_lqr_cruise',    K_lqr_cruise);
assignin('base', 'x_trim_lqr',      x_trim_lqr);
assignin('base', 'U_trim_lqr',      U_trim_lqr);

fprintf('\n--- Cruise Scenario Ready ---\n');
fprintf('  IC perturbation (deg):  phi %+0.1f   theta %+0.1f   psi %+0.1f\n', ...
    InitAtt_offset_deg(1), InitAtt_offset_deg(2), InitAtt_offset_deg(3));
fprintf('  Step at t = %.1f s (deg): phi %+0.1f   theta %+0.1f   psi %+0.1f\n', ...
    tStep, phiStepDeg, thetaStepDeg, psiStepDeg);
fprintf('  Sim duration: %.0f s   controller_mode = %d (LQR)\n\n', ...
    stopTime, controller_mode);

%% Run the wrapper
run_model_here = true;

if run_model_here
    open_system('EVTOL_6DOF_Sim_Wrapper');
    Cruise_sim_out = sim('EVTOL_6DOF_Sim_Wrapper', ...
        'StartTime', num2str(startTime), ...
        'StopTime',  num2str(stopTime));
    save('Cruise_sim_out.mat', 'Cruise_sim_out');
    fprintf('Simulation complete. Output saved to Cruise_sim_out.mat\n');
else
    fprintf('run_model_here=false — set to true to execute the wrapper.\n');
end