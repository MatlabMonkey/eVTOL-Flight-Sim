function controllerData = build_trim_point_lqr_controller(trimResult, opts)
%BUILD_TRIM_POINT_LQR_CONTROLLER Build a rigid-body LQR around a trim point.
%
% Required input:
%   trimResult = struct from Trim_EVTOL_Main / trim_evtol_case
%
% Optional opts fields:
%   variant_ctrl_mode = active Variant Subsystem mode (default 2)
%   controller_id     = dispatch helper id (default 2)
%   require_exact_trim = true to reject inexact trims (default true)
%   Q_full            = 9x9 state penalty for the full rigid-body design
%   Q_inner           = 8x8 state penalty when psi is excluded
%   R                 = 6x6 input penalty for
%                       [front_coll rear_coll delta_f delta_a delta_e delta_r]
%
% Controller convention:
%   states = [phi theta psi u v w P Q R]
%   inputs = [front_coll_rpm rear_coll_rpm delta_f delta_a delta_e delta_r]
%
% The design intentionally ignores actuator dynamics and builds directly on
% the reduced rigid-body trim linearization.

if nargin < 2 || isempty(opts)
    opts = struct();
end

opts = local_apply_defaults(opts);

fprintf('\n=== Trim-Point LQR Controller Build ===\n');

if ~isstruct(trimResult)
    error(['trimResult must be the struct returned by Trim_EVTOL_Main so the ', ...
           'controller is designed around an explicit operating point.']);
end

if opts.require_exact_trim
    if ~isfield(trimResult, 'success') || ~trimResult.success
        error(['trimResult does not report an exact trim solution. ', ...
               'Refine the trim first, or explicitly opt out of the exact-trim ', ...
               'requirement before building a linear controller.']);
    end
elseif ~isfield(trimResult, 'success') || ~trimResult.success
    warning(['trimResult did not report an exact trim solution. ', ...
             'Continuing because a reduced linear model is present, but the ', ...
             'nonlinear validation should be treated as provisional.']);
end

if ~isfield(trimResult, 'linear') || ~isfield(trimResult.linear, 'sys_ss_9state') || ...
        isempty(trimResult.linear.sys_ss_9state)
    error(['trimResult.linear.sys_ss_9state is missing. The selected trim did ', ...
           'not produce the rigid-body reduced-order model needed by this LQR path.']);
end

sys_ss_9state = trimResult.linear.sys_ss_9state;
local_validate_reduced_model(sys_ss_9state);

A_lqr = sys_ss_9state.A;
B_lqr = sys_ss_9state.B;
Att_Trim = trimResult.trim.Att_Trim;
Vel_B_BA_Trim = trimResult.trim.Vel_B_BA_Trim;
Rates_Trim = trimResult.trim.Rates_Trim;
surface_limit_deg = local_get_surface_limit_deg(trimResult);

fprintf('Trim point: %s (%s)\n', local_get_field(trimResult, 'name', 'unnamed_trim'), ...
    local_get_field(trimResult, 'mode', 'unknown_mode'));
fprintf('Open-loop rigid-body eigenvalues:\n');
ev = eig(A_lqr);
for i = 1:numel(ev)
    fprintf('  %+10.4f %+10.4fi\n', real(ev(i)), imag(ev(i)));
end

fprintf('Reduced-model input channel norms:\n');
input_names = cellstr(sys_ss_9state.InputName);
input_norms = vecnorm(B_lqr, 2, 1);
for i = 1:numel(input_names)
    fprintf('  %-14s %0.6g\n', input_names{i}, input_norms(i));
end

weak_inputs = find(input_norms < 1e-10);
if ~isempty(weak_inputs)
    weak_labels = strjoin(input_names(weak_inputs), ', ');
    warning(['Some reduced-model input channels have effectively zero rigid-body ', ...
             'authority at this trim point: %s'], weak_labels);
end

ctrb_rank_9 = rank(ctrb(A_lqr, B_lqr));
fprintf('Controllability rank (9-state) : %d / 9\n', ctrb_rank_9);

use_full_9state = (ctrb_rank_9 == 9);
if use_full_9state
    fprintf('Using full 9-state LQR: [phi theta psi u v w P Q R]\n');
    keep_idx = 1:9;
    Q_use = opts.Q_full;
else
    fprintf(['9-state pair is not fully controllable.\n', ...
             'Falling back to 8-state inner-loop LQR with psi excluded:\n', ...
             '  [phi theta u v w P Q R]\n']);
    keep_idx = [1 2 4 5 6 7 8 9];
    A_test = A_lqr(keep_idx, keep_idx);
    B_test = B_lqr(keep_idx, :);
    ctrb_rank_8 = rank(ctrb(A_test, B_test));
    fprintf('Controllability rank (8-state) : %d / 8\n', ctrb_rank_8);
    if ctrb_rank_8 < 8
        error(['Even the 8-state inner-loop model is not fully controllable. ', ...
               'The reduced trim linearization is too aggressive, or the chosen ', ...
               'actuator channels do not span the states you are trying to regulate.']);
    end
    Q_use = opts.Q_inner;
end

A_use = A_lqr(keep_idx, keep_idx);
B_use = B_lqr(keep_idx, :);
R_use = opts.R;

[K_use, S_lqr, cl_eigs] = lqr(A_use, B_use, Q_use, R_use); %#ok<ASGLU>

K_dispatch = zeros(6, 9);
K_dispatch(:, keep_idx) = K_use;

fprintf('\nK_dispatch (6 x 9):\n');
disp(K_dispatch);

fprintf('Closed-loop eigenvalues of designed subsystem:\n');
for i = 1:numel(cl_eigs)
    fprintf('  %+10.4f %+10.4fi\n', real(cl_eigs(i)), imag(cl_eigs(i)));
end

if any(real(cl_eigs) > 1e-6)
    warning('Designed closed-loop subsystem still has unstable mode(s).');
else
    fprintf('Designed closed-loop subsystem is stable.\n');
end

if ~use_full_9state
    fprintf(['NOTE: psi is excluded from the inner-loop LQR.\n', ...
             'Its gain column is intentionally zero, so heading should be\n', ...
             'handled by an outer loop later.\n']);
end

fprintf('\nSaturation spot-check (surface limit = +/-%.0f deg):\n', surface_limit_deg);
probe_roll = zeros(9, 1);
probe_roll(1) = deg2rad(10);
probe_pitch = zeros(9, 1);
probe_pitch(2) = deg2rad(5);
probe_yaw = zeros(9, 1);
probe_yaw(3) = deg2rad(10);

u_roll = -K_dispatch * probe_roll;
u_pitch = -K_dispatch * probe_pitch;
u_yaw = -K_dispatch * probe_yaw;

fmt = '  %-14s  fRPM=%+8.1f   rRPM=%+8.1f   df=%+7.2f  da=%+7.2f  de=%+7.2f  dr=%+7.2f  deg\n';
fprintf(fmt, '10 deg roll :',  u_roll(1),  u_roll(2),  rad2deg(u_roll(3)),  rad2deg(u_roll(4)),  rad2deg(u_roll(5)),  rad2deg(u_roll(6)));
fprintf(fmt, ' 5 deg pitch:',  u_pitch(1), u_pitch(2), rad2deg(u_pitch(3)), rad2deg(u_pitch(4)), rad2deg(u_pitch(5)), rad2deg(u_pitch(6)));
fprintf(fmt, '10 deg yaw  :',  u_yaw(1),   u_yaw(2),   rad2deg(u_yaw(3)),   rad2deg(u_yaw(4)),   rad2deg(u_yaw(5)),   rad2deg(u_yaw(6)));

surface_mags_deg = abs(rad2deg([u_roll(3:6); u_pitch(3:6); u_yaw(3:6)]));
fprintf('  Max |surface| command across probes: %.2f deg\n', max(surface_mags_deg));
fprintf('  Max |front_coll| delta across probes: %.1f rpm\n', max(abs([u_roll(1); u_pitch(1); u_yaw(1)])));
fprintf('  Max |rear_coll| delta across probes: %.1f rpm\n', max(abs([u_roll(2); u_pitch(2); u_yaw(2)])));

x_trim = [Att_Trim; Vel_B_BA_Trim; Rates_Trim];
front_trim = trimResult.trim.U_trim_full(7);
rear_trim = trimResult.trim.U_trim_full(8);
mixed_trim = trimResult.trim.mixed_control_trim(:);
trim_cmd = [front_trim; rear_trim; mixed_trim];

controllerData = struct();
controllerData.name = sprintf('TrimPointLQR_%s', local_get_field(trimResult, 'name', 'unnamed_trim'));
controllerData.type = 'trim_point_lqr';
controllerData.design_source = mfilename;
controllerData.trimName = local_get_field(trimResult, 'name', 'unnamed_trim');
controllerData.trimMode = local_get_field(trimResult, 'mode', 'unknown_mode');
controllerData.variant_ctrl_mode = opts.variant_ctrl_mode;
controllerData.controller_id = opts.controller_id;
controllerData.controller_state_ref = x_trim;
controllerData.controller_trim_cmd = trim_cmd;
controllerData.controller_gain_lqr = K_dispatch;
controllerData.K_dispatch = K_dispatch;
controllerData.x_trim = x_trim;
controllerData.trim_cmd = trim_cmd;
controllerData.A_lqr = A_lqr;
controllerData.B_lqr = B_lqr;
controllerData.keep_idx = keep_idx;
controllerData.use_full_9state = use_full_9state;
controllerData.closed_loop_eigs = cl_eigs;
controllerData.Q = Q_use;
controllerData.R = R_use;
controllerData.reduced_input_names = input_names(:);
controllerData.reduced_input_norms = input_norms(:);

% Legacy fields kept so Run_EVTOL_Main and older scripts still have a sane
% fallback view of the trim-point controller in mixed-control coordinates.
controllerData.K_lqr_cruise = K_dispatch([1 3 4 5 6], :);
controllerData.x_trim_lqr = x_trim;
controllerData.U_trim_lqr = [front_trim; mixed_trim];

fprintf('\ncontroller_state_ref = [phi theta psi u v w P Q R]:\n');
disp(x_trim.');
fprintf('controller_trim_cmd = [front_coll rear_coll delta_f delta_a delta_e delta_r]:\n');
disp(trim_cmd.');
fprintf('=== Trim-Point LQR Build Complete ===\n\n');
end

function opts = local_apply_defaults(opts)
if ~isfield(opts, 'variant_ctrl_mode')
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id')
    opts.controller_id = 2;
end
if ~isfield(opts, 'require_exact_trim')
    opts.require_exact_trim = true;
end
if ~isfield(opts, 'Q_full')
    opts.Q_full = diag([8, 10, 1, 1, 1, 3, 6, 8, 6]);
end
if ~isfield(opts, 'Q_inner')
    opts.Q_inner = diag([8, 10, 1, 1, 3, 6, 8, 6]);
end
if ~isfield(opts, 'R')
    opts.R = diag([1e-4, 1e-4, 3, 3, 3, 3]);
end
end

function local_validate_reduced_model(sys_ss_9state)
if size(sys_ss_9state.A, 1) ~= 9 || size(sys_ss_9state.A, 2) ~= 9
    error('trimResult.linear.sys_ss_9state must be 9x9. Current size is %dx%d.', ...
        size(sys_ss_9state.A, 1), size(sys_ss_9state.A, 2));
end

expected_inputs = {'front_coll_rpm'; 'rear_coll_rpm'; 'delta_f'; 'delta_a'; 'delta_e'; 'delta_r'};
input_names = cellstr(sys_ss_9state.InputName);
if numel(input_names) ~= numel(expected_inputs) || ~isequal(input_names(:), expected_inputs)
    error(['Unexpected reduced-model input set in sys_ss_9state. ', ...
           'Current inputs are: %s'], strjoin(input_names(:).', ', '));
end
end

function value = local_get_field(s, field_name, fallback)
value = fallback;
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
end
end

function value = local_get_surface_limit_deg(trimResult)
value = 25;
if isfield(trimResult, 'trimSpec') && isfield(trimResult.trimSpec, 'surface_limit_deg_scalar')
    value = trimResult.trimSpec.surface_limit_deg_scalar;
end
end
