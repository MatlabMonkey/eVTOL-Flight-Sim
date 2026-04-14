function linData = linearize_cruise_plant(varargin)
%LINEARIZE_CRUISE_PLANT Linearize Brown_6DOF_Plant at a trimmed cruise point.

p = inputParser;
p.addParameter('CruiseSpeed', 70, @(x) isnumeric(x) && isscalar(x));
p.addParameter('TrimBankDeg', 0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('UseAVLAero', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
ensure_control_analysis_helper();
setup_cruise_trim_case( ...
    'CruiseSpeed', p.Results.CruiseSpeed, ...
    'TrimBankDeg', p.Results.TrimBankDeg, ...
    'BankCmdDeg', p.Results.TrimBankDeg, ...
    'AirspeedStep', 0, ...
    'BankStepDeg', 0);
assignin('base', 'use_avl_aero', logical(p.Results.UseAVLAero));

load_system('Brown_6DOF_Plant');
load_system('Brown_Control_Analysis_Helper');
[x0, u0] = localBuildPlantOperatingPoint();
[A, B, C, D] = linmod('Brown_Control_Analysis_Helper', x0, u0);

io = struct();
io.inputs.motor = 1:12;
io.inputs.tilt = 13:18;
io.inputs.front_collective = 19;
io.inputs.rear_collective = 20;
io.inputs.delta_f = 21;
io.inputs.delta_a = 22;
io.inputs.delta_e = 23;
io.inputs.delta_r = 24;

io.outputs.pos = 1:3;
io.outputs.v_b = 4:6;
io.outputs.v_e = 7:9;
io.outputs.eul = 10:12;
io.outputs.omega = 13:15;
io.outputs.C_NB = 16:24;
io.outputs.v_ba = 25:27;
io.outputs.vinf = 28;
io.outputs.alpha = 29;
io.outputs.beta = 30;
io.outputs.specific_force = 31:33;

frontCollective = zeros(size(B, 2), 1);
frontCollective(io.inputs.front_collective) = 1;
rearCollective = zeros(size(B, 2), 1);
rearCollective(io.inputs.rear_collective) = 1;

linData = struct();
linData.A = A;
linData.B = B;
linData.C = C;
linData.D = D;
linData.io = io;
linData.x0 = x0;
linData.u0 = u0;
linData.sys = ss(A, B, C, D);
linData.frontCollectiveVec = frontCollective;
linData.rearCollectiveVec = rearCollective;
linData.frontCollectiveChannel = ss(A, B * frontCollective, C, D * frontCollective);
linData.rearCollectiveChannel = ss(A, B * rearCollective, C, D * rearCollective);
linData.openLoopEig = eig(A);
linData.trimCase = load_homework_trim_case( ...
    'CruiseSpeed', p.Results.CruiseSpeed, ...
    'BankDeg', p.Results.TrimBankDeg);
end

function [x0, u0] = localBuildPlantOperatingPoint()
pos = evalin('base', 'pos_init(:)');
omega = evalin('base', 'omega_init(:)');
vBody = evalin('base', 'V_init(:)');
eul = evalin('base', 'eul_init(:)');
quat = localEulerToQuat(eul(1), eul(2), eul(3));

x0 = [pos; omega; vBody; quat];

u0 = zeros(24, 1);
motorGroup = evalin('base', 'controller_trim_motor_rpms(:)');
tiltGroup = evalin('base', 'controller_trim_tilt_angles(:)');
u0(1:12) = [motorGroup(1); motorGroup(1); motorGroup(1); ...
    motorGroup(2); motorGroup(2); motorGroup(2); ...
    motorGroup(3); motorGroup(3); motorGroup(3); ...
    motorGroup(4); motorGroup(4); motorGroup(4)];
u0(13:18) = [tiltGroup(1); tiltGroup(1); tiltGroup(1); ...
    tiltGroup(2); tiltGroup(2); tiltGroup(2)];
u0(19) = evalin('base', 'trim_front_collective_rpm');
u0(20) = evalin('base', 'trim_rear_collective_rpm');
u0(21) = evalin('base', 'controller_trim_delta_f');
u0(22) = evalin('base', 'controller_trim_delta_a');
u0(23) = evalin('base', 'controller_trim_delta_e');
u0(24) = evalin('base', 'controller_trim_delta_r');
end

function q = localEulerToQuat(phi, theta, psi)
cphi = cos(phi / 2); sphi = sin(phi / 2);
cth = cos(theta / 2); sth = sin(theta / 2);
cpsi = cos(psi / 2); spsi = sin(psi / 2);

q = [ ...
    cphi * cth * cpsi + sphi * sth * spsi; ...
    sphi * cth * cpsi - cphi * sth * spsi; ...
    cphi * sth * cpsi + sphi * cth * spsi; ...
    cphi * cth * spsi - sphi * sth * cpsi];
q = q / norm(q);
end
