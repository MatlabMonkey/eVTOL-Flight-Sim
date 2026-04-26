function out = make_transition_demo_out(varargin)
%MAKE_TRANSITION_DEMO_OUT Build a fake video-ready out struct.
%
% This helper fabricates a minimal `out` struct that matches the fields
% consumed by make_evtol_video, so you can render a believable transition
% demo without running the nonlinear plant first.
%
% Preferred usage:
%   out = make_transition_demo_out()
%   out = make_transition_demo_out('direction', 'cruise_to_trim')
%   out = make_transition_demo_out('assign_to_base', true)
%
% The returned struct includes:
%   pos_NED, eul_truth, V_B_truth, omega_truth,
%   tilt_angles_cmd, front_collective_rpm_out, rear_collective_rpm_out,
%   deltaLW_cmd, deltaRW_cmd, deltaLT_cmd, deltaRT_cmd, tout

parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'direction', 'trim_to_cruise');
addParameter(parser, 'target_vinf_mps', []);
addParameter(parser, 'stop_time_s', 30.0);
addParameter(parser, 'hold_start_s', 2.0);
addParameter(parser, 'hold_end_s', 3.0);
addParameter(parser, 'step_time_s', []);
addParameter(parser, 'assign_to_base', false);
parse(parser, varargin{:});

opts = parser.Results;

demo = make_transition_demo_cmds( ...
    'direction', opts.direction, ...
    'target_vinf_mps', opts.target_vinf_mps, ...
    'stop_time_s', opts.stop_time_s, ...
    'hold_start_s', opts.hold_start_s, ...
    'hold_end_s', opts.hold_end_s, ...
    'step_time_s', opts.step_time_s, ...
    'assign_to_base', false);

t = demo.cmds.airData_cmd(:, 1);
dt = median(diff(t));

vinf_cmd = demo.cmds.airData_cmd(:, 2);
alpha_cmd = demo.cmds.airData_cmd(:, 3);
beta_cmd = demo.cmds.airData_cmd(:, 4);
theta_cmd = demo.cmds.eul_cmd(:, 3);

tilt_cmd = demo.cmds.tilt_cmd(:, 2:3);
surface_cmd = demo.cmds.surface_local_cmd(:, 2:5);
rpm_cmd = [demo.cmds.front_cmd(:, 2), demo.cmds.front_cmd(:, 2), ...
           demo.cmds.rear_cmd(:, 2), demo.cmds.rear_cmd(:, 2)];

% Simple first-order actuator/airframe lag so the fake histories look like
% outputs rather than perfect command tracking.
tilt_truth = localFirstOrderTrack(tilt_cmd, dt, 0.35);
surface_truth = localFirstOrderTrack(surface_cmd, dt, 0.12);
rpm_truth = localFirstOrderTrack(rpm_cmd, dt, 0.25);
vinf_truth = localFirstOrderTrack(vinf_cmd, dt, 1.8);
alpha_truth = localFirstOrderTrack(alpha_cmd, dt, 0.9);
beta_truth = localFirstOrderTrack(beta_cmd, dt, 0.9);
theta_truth = localFirstOrderTrack(theta_cmd, dt, 1.0);

eul_truth = [zeros(numel(t), 1), theta_truth, zeros(numel(t), 1)];
omega_truth = [zeros(numel(t), 1), gradient(theta_truth, dt), zeros(numel(t), 1)];

u_truth = vinf_truth .* cos(alpha_truth) .* cos(beta_truth);
v_truth = vinf_truth .* sin(beta_truth);
w_truth = vinf_truth .* sin(alpha_truth) .* cos(beta_truth);
V_B_truth = [u_truth, v_truth, w_truth];

pos_NED = zeros(numel(t), 3);
V_E_truth = zeros(numel(t), 3);

for idx = 1:numel(t)
    C_BN = localCbnFromEuler(eul_truth(idx, :).');
    V_E_truth(idx, :) = (C_BN * V_B_truth(idx, :).').';
end

pos_NED(:, 1) = cumtrapz(t, V_E_truth(:, 1));
pos_NED(:, 2) = cumtrapz(t, V_E_truth(:, 2));
pos_NED(:, 3) = cumtrapz(t, V_E_truth(:, 3));

out = struct();
out.pos_NED = localMakeStructWithTime(t, pos_NED);
out.eul_truth = localMakeStructWithTime(t, eul_truth);
out.V_B_truth = localMakeStructWithTime(t, V_B_truth);
out.omega_truth = localMakeStructWithTime(t, omega_truth);
out.V_E_truth = localMakeStructWithTime(t, V_E_truth);
out.alpha_truth = localMakeStructWithTime(t, alpha_truth);
out.beta_truth = localMakeStructWithTime(t, beta_truth);
out.vinf_truth = localMakeStructWithTime(t, vinf_truth);
out.tilt_angles_cmd = localMakeStructWithTime(t, tilt_truth);
out.front_collective_rpm_out = localMakeStructWithTime(t, rpm_truth(:, 1));
out.rear_collective_rpm_out = localMakeStructWithTime(t, rpm_truth(:, 3));
out.deltaLW_cmd = localMakeStructWithTime(t, surface_truth(:, 1));
out.deltaRW_cmd = localMakeStructWithTime(t, surface_truth(:, 2));
out.deltaLT_cmd = localMakeStructWithTime(t, surface_truth(:, 3));
out.deltaRT_cmd = localMakeStructWithTime(t, surface_truth(:, 4));
out.viz_tilt_states = localMakeStructWithTime(t, tilt_truth);
out.viz_surface_states = localMakeStructWithTime(t, surface_truth);
out.viz_prop_rpm = localMakeStructWithTime(t, rpm_truth);
out.tout = t;
out.demo = demo;

if opts.assign_to_base
    assignin('base', 'out', out);
end
end

function tracked = localFirstOrderTrack(command, dt, tau)
if isvector(command)
    command = command(:);
end

tracked = zeros(size(command));
tracked(1, :) = command(1, :);

if tau <= 0
    tracked = command;
    return;
end

alpha = min(max(dt / tau, 0.0), 1.0);
for idx = 2:size(command, 1)
    tracked(idx, :) = tracked(idx - 1, :) + alpha * (command(idx, :) - tracked(idx - 1, :));
end
end

function s = localMakeStructWithTime(t, values)
s = struct();
s.time = t(:);
s.signals = struct('values', values);
end

function C_BN = localCbnFromEuler(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);

cphi = cos(phi);
sphi = sin(phi);
cth = cos(theta);
sth = sin(theta);
cps = cos(psi);
sps = sin(psi);

C_BN = [ ...
    cth * cps,                                cth * sps,                               -sth; ...
    sphi * sth * cps - cphi * sps,           sphi * sth * sps + cphi * cps,          sphi * cth; ...
    cphi * sth * cps + sphi * sps,           cphi * sth * sps - sphi * cps,          cphi * cth];
end
