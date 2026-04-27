function setup_cruise_trim_case(varargin)
%SETUP_CRUISE_TRIM_CASE Populate cruise-trim-like initial conditions.
%   This helper is intended for controller demos and linear analysis. It
%   uses the converged Homework 7 trim results as the source of truth for
%   the cruise operating point and controller trim commands.

p = inputParser;
p.addParameter('CruiseSpeed', 70, @(x) isnumeric(x) && isscalar(x));
p.addParameter('TrimBankDeg', 0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('TiltDeg', 90, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BankCmdDeg', 0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('AirspeedStep', 5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BankStepDeg', 10, @(x) isnumeric(x) && isscalar(x));
p.parse(varargin{:});

trimCase = load_homework_trim_case( ...
    'CruiseSpeed', p.Results.CruiseSpeed, ...
    'BankDeg', p.Results.TrimBankDeg);

vinfTrim = trimCase.vinf_trim;
alphaTrim = deg2rad(trimCase.alpha_trim_deg);
betaTrim = deg2rad(trimCase.beta_trim_deg);
phiTrim = deg2rad(trimCase.roll_trim_deg);
thetaTrim = deg2rad(trimCase.pitch_trim_deg);
psiTrim = deg2rad(trimCase.yaw_trim_deg);

pTrim = localFieldOrDefault(trimCase, 'p_trim', 0);
qTrim = localFieldOrDefault(trimCase, 'q_trim', 0);
rTrim = localFieldOrDefault(trimCase, 'r_trim', 0);

frontRPM = trimCase.front_collective_rpm;
rearRPM = trimCase.rear_collective_rpm;
tiltDeg = localFieldOrDefault(trimCase, 'front_tilt_deg_mean', p.Results.TiltDeg);

uBody = vinfTrim * cos(alphaTrim) * cos(betaTrim);
vBody = vinfTrim * sin(betaTrim);
wBody = vinfTrim * sin(alphaTrim) * cos(betaTrim);

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'V_init', [uBody; vBody; wBody]);
assignin('base', 'eul_init', [phiTrim; thetaTrim; psiTrim]);
assignin('base', 'omega_init', [pTrim; qTrim; rTrim]);
assignin('base', 'Motor_RPMs', [frontRPM * ones(6, 1); rearRPM * ones(6, 1)]);
assignin('base', 'Tilt_angles', tiltDeg * ones(6, 1));
assignin('base', 'trim_front_collective_rpm', frontRPM);
assignin('base', 'trim_rear_collective_rpm', rearRPM);

% The current Propellers subsystem is grouped as [FR; FL; RR; RL] motors
% and [FR; FL] tilt commands. The trim collectives are applied through the
% dedicated grouped collective inputs, so keep the motor-group trim vector
% at zero and use it only for differential controller offsets.
assignin('base', 'controller_trim_motor_rpms', zeros(4, 1));
assignin('base', 'controller_trim_tilt_angles', [tiltDeg; tiltDeg]);
assignin('base', 'controller_trim_motor_rpm_cmd', zeros(12, 1));
assignin('base', 'controller_trim_tilt_angles_cmd', tiltDeg * ones(6, 1));
assignin('base', 'controller_trim_delta_f', 0.0);
assignin('base', 'controller_trim_delta_a', deg2rad(trimCase.delta_a_deg));
assignin('base', 'controller_trim_delta_e', deg2rad(trimCase.delta_e_deg));
assignin('base', 'controller_trim_delta_r', deg2rad(trimCase.delta_r_deg));
assignin('base', 'controller_airspeed_cmd_base', vinfTrim);
assignin('base', 'controller_bank_cmd_base', deg2rad(p.Results.BankCmdDeg));
assignin('base', 'controller_step_time', 1.0);
assignin('base', 'controller_airspeed_step', p.Results.AirspeedStep);
assignin('base', 'controller_bank_step', deg2rad(p.Results.BankStepDeg));
assignin('base', 'controller_front_motor_mask', [1; 1; 0; 0]);
assignin('base', 'estimator_init_eul', [phiTrim; thetaTrim; psiTrim]);
end

function value = localFieldOrDefault(s, fieldName, defaultValue)
if isfield(s, fieldName) && ~isempty(s.(fieldName)) && isfinite(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end
