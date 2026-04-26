function controllerData = build_trim_lqr_controller(trimResult, opts)
%BUILD_TRIM_POINT_LONGITUDINAL_LQR_CONTROLLER Conservative longitudinal trim-point LQR.
%
% Required input:
%   trimResult = struct from Trim_Main / trim_evtol_case
%
% Optional opts fields:
%   variant_ctrl_mode = active Variant Subsystem mode (default 2)
%   controller_id     = dispatch helper id (default 3)
%   require_exact_trim = true to reject inexact trims (default true)
%   Q_long            = 4x4 state penalty for [theta u w Q]
%   R_long            = 4x4 input penalty for
%                       [front_coll rear_coll delta_f delta_e]
%
% Controller convention:
%   full states = [phi theta psi u v w P Q R]
%   full inputs = [front_coll_rpm rear_coll_rpm delta_f delta_a delta_e delta_r]
%
% This builder intentionally ignores lateral states and lateral controls.
% The returned 6x9 gain matrix has zero columns for lateral states and zero
% rows for delta_a / delta_r, so those channels stay fixed at the trim point.

if nargin < 2 || isempty(opts)
    opts = struct();
end

opts = local_apply_defaults(opts);

fprintf('\n=== Trim-Point Longitudinal LQR Controller Build ===\n');

if ~isstruct(trimResult)
    error(['trimResult must be the struct returned by Trim_Main so the ', ...
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

A_full = sys_ss_9state.A;
B_full = sys_ss_9state.B;
state_idx = opts.longitudinal_state_idx(:).';
input_idx = opts.longitudinal_input_idx(:).';
state_labels = opts.longitudinal_state_labels(:).';
surface_limit_deg = local_get_surface_limit_deg(trimResult);

A_long = A_full(state_idx, state_idx);
B_long = B_full(state_idx, input_idx);

input_names = cellstr(sys_ss_9state.InputName);
input_labels = input_names(input_idx);

fprintf('Trim point: %s (%s)\n', local_get_field(trimResult, 'name', 'unnamed_trim'), ...
    local_get_field(trimResult, 'mode', 'unknown_mode'));
fprintf('Longitudinal states: %s\n', strjoin(state_labels, ', '));
fprintf('Longitudinal inputs: %s\n', strjoin(input_labels, ', '));
fprintf('Holding lateral channels at trim: delta_a = %+6.3f deg, delta_r = %+6.3f deg\n', ...
    rad2deg(trimResult.trim.mixed_control_trim(2)), ...
    rad2deg(trimResult.trim.mixed_control_trim(4)));

ctrb_rank = rank(ctrb(A_long, B_long));
fprintf('Controllability rank (longitudinal) : %d / %d\n', ctrb_rank, size(A_long, 1));
if ctrb_rank < size(A_long, 1)
    error(['The selected longitudinal state/input pair is not fully controllable. ', ...
           'Choose a different reduced state set or actuator subset before using LQR.']);
end

[K_long, ~, cl_eigs] = lqr(A_long, B_long, opts.Q_long, opts.R_long);

K_dispatch = zeros(6, 9);
K_dispatch(input_idx, state_idx) = K_long;

x_trim = [trimResult.trim.Att_Trim; ...
          trimResult.trim.Vel_B_BA_Trim; ...
          trimResult.trim.Rates_Trim];
front_trim = trimResult.trim.U_trim_full(7);
rear_trim = trimResult.trim.U_trim_full(8);
mixed_trim = trimResult.trim.mixed_control_trim(:);
trim_cmd = [front_trim; rear_trim; mixed_trim];

fprintf('\nK_long (%d x %d):\n', size(K_long, 1), size(K_long, 2));
disp(K_long);

fprintf('Closed-loop eigenvalues of longitudinal subsystem:\n');
for i = 1:numel(cl_eigs)
    fprintf('  %+10.4f %+10.4fi\n', real(cl_eigs(i)), imag(cl_eigs(i)));
end

probe = zeros(9, 1);
probe(state_idx) = [deg2rad(5); 2; 0.5; deg2rad(5)];
u_probe = -K_dispatch * probe;
fprintf('\nGentle-gain spot check for [theta=5deg, u=2m/s, w=0.5m/s, Q=5deg/s]:\n');
fprintf(['  dfc=%+8.4f rpm   drc=%+8.4f rpm   df=%+7.3f deg   da=%+7.3f deg   ', ...
         'de=%+7.3f deg   dr=%+7.3f deg\n'], ...
    u_probe(1), u_probe(2), rad2deg(u_probe(3)), rad2deg(u_probe(4)), ...
    rad2deg(u_probe(5)), rad2deg(u_probe(6)));
fprintf('  Surface limit still enforced at +/-%.0f deg in the dispatcher.\n', surface_limit_deg);

controllerData = struct();
controllerData.name = sprintf('TrimPointLongitudinalLQR_%s', local_get_field(trimResult, 'name', 'unnamed_trim'));
controllerData.type = 'trim_point_longitudinal_lqr';
controllerData.design_source = mfilename;
controllerData.trimName = local_get_field(trimResult, 'name', 'unnamed_trim');
controllerData.trimMode = local_get_field(trimResult, 'mode', 'unknown_mode');
controllerData.variant_ctrl_mode = opts.variant_ctrl_mode;
controllerData.controller_id = opts.controller_id;
controllerData.controller_state_ref = x_trim;
controllerData.controller_trim_cmd = trim_cmd;
controllerData.controller_gain_lqr = K_dispatch;
controllerData.K_dispatch = K_dispatch;
controllerData.K_long = K_long;
controllerData.x_trim = x_trim;
controllerData.trim_cmd = trim_cmd;
controllerData.longitudinal_state_idx = state_idx(:);
controllerData.longitudinal_input_idx = input_idx(:);
controllerData.longitudinal_state_labels = state_labels(:);
controllerData.longitudinal_input_labels = input_labels(:);
controllerData.A_long = A_long;
controllerData.B_long = B_long;
controllerData.closed_loop_eigs = cl_eigs;
controllerData.Q_long = opts.Q_long;
controllerData.R_long = opts.R_long;
controllerData.lateral_trim_cmd = [mixed_trim(2); mixed_trim(4)];

% Legacy compatibility view.
controllerData.K_lqr_cruise = K_dispatch([1 3 4 5 6], :);
controllerData.x_trim_lqr = x_trim;
controllerData.U_trim_lqr = [front_trim; mixed_trim];

fprintf('\ncontroller_state_ref = [phi theta psi u v w P Q R]:\n');
disp(x_trim.');
fprintf('controller_trim_cmd = [front_coll rear_coll delta_f delta_a delta_e delta_r]:\n');
disp(trim_cmd.');
fprintf('=== Trim-Point Longitudinal LQR Build Complete ===\n\n');
end

function opts = local_apply_defaults(opts)
if ~isfield(opts, 'variant_ctrl_mode')
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id')
    opts.controller_id = 3;
end
if ~isfield(opts, 'require_exact_trim')
    opts.require_exact_trim = true;
end
if ~isfield(opts, 'longitudinal_state_idx')
    opts.longitudinal_state_idx = [2 4 6 8];
end
if ~isfield(opts, 'longitudinal_input_idx')
    opts.longitudinal_input_idx = [1 2 3 5];
end
if ~isfield(opts, 'longitudinal_state_labels')
    opts.longitudinal_state_labels = {'theta', 'u', 'w', 'Q'};
end
if ~isfield(opts, 'Q_long')
    opts.Q_long = diag([3.4, 0.45, 0.35, 0.95]);
end
if ~isfield(opts, 'R_long')
    opts.R_long = diag([6.5, 20, 100, 100]);
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
