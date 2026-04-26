function controllerData = build_two_point_transition_lqr_controller(trimResultA, trimResultB, opts)
%BUILD_TWO_POINT_TRANSITION_LQR_CONTROLLER Build a 2-point scheduled LQR.
%
% Usage:
%   controllerData = build_two_point_transition_lqr_controller(trimResultA, trimResultB)
%   controllerData = build_two_point_transition_lqr_controller(trimResultA, trimResultB, opts)
%
% The scheduled controller uses the existing controller_id = 4 path-LQR
% dispatch, but only keeps two exact operating points:
%   point A -> progress 0
%   point B -> progress 1
%
% Default design path:
%   - build a longitudinal trim-point LQR at each point
%   - schedule x_ref, trim_cmd, and K across the two points
%
% Optional opts fields:
%   lqr_opts          forwarded into build_trim_point_longitudinal_lqr_controller
%   variant_ctrl_mode default 2
%   controller_id     default 4
%   pointA_name       override display label
%   pointB_name       override display label
%   progress_pair     default [0 1]

if nargin < 3 || isempty(opts)
    opts = struct();
end

if ~isstruct(trimResultA) || ~isstruct(trimResultB)
    error('trimResultA and trimResultB must be valid trimResult structs.');
end

opts = localApplyDefaults(opts);

ctrlA = build_trim_point_longitudinal_lqr_controller(trimResultA, opts.lqr_opts);
ctrlB = build_trim_point_longitudinal_lqr_controller(trimResultB, opts.lqr_opts);

nameA = localGetField(opts, 'pointA_name', localGetField(trimResultA, 'name', 'PointA'));
nameB = localGetField(opts, 'pointB_name', localGetField(trimResultB, 'name', 'PointB'));
prog = opts.progress_pair(:).';
if numel(prog) ~= 2
    error('opts.progress_pair must contain exactly two values.');
end

stateSchedule = zeros(12, 2);
stateSchedule(1:9, 1) = ctrlA.controller_state_ref(1:9, 1);
stateSchedule(1:9, 2) = ctrlB.controller_state_ref(1:9, 1);
stateSchedule(10, 1) = localResolveTiltDeg(trimResultA);
stateSchedule(10, 2) = localResolveTiltDeg(trimResultB);
stateSchedule(11, 1) = localResolveVinf(trimResultA);
stateSchedule(11, 2) = localResolveVinf(trimResultB);
stateSchedule(12, :) = prog;

trimSchedule = zeros(6, 2);
trimSchedule(:, 1) = ctrlA.controller_trim_cmd(:, 1);
trimSchedule(:, 2) = ctrlB.controller_trim_cmd(:, 1);

gainSchedule = zeros(6, 9, 2);
gainSchedule(:, :, 1) = ctrlA.controller_gain_lqr(:, :, 1);
gainSchedule(:, :, 2) = ctrlB.controller_gain_lqr(:, :, 1);

controllerData = struct();
controllerData.name = sprintf('TwoPointTransitionLQR_%s_to_%s', char(string(nameA)), char(string(nameB)));
controllerData.type = 'two_point_transition_lqr';
controllerData.design_source = mfilename;
controllerData.variant_ctrl_mode = opts.variant_ctrl_mode;
controllerData.controller_id = opts.controller_id;
controllerData.controller_state_ref = stateSchedule;
controllerData.controller_trim_cmd = trimSchedule;
controllerData.controller_gain_lqr = gainSchedule;
controllerData.schedule_count = 2;
controllerData.schedule_progress = prog(:);
controllerData.schedule_tilt_deg = stateSchedule(10, :).';
controllerData.schedule_vinf_mps = stateSchedule(11, :).';
controllerData.schedule_names = string({nameA; nameB});
controllerData.schedule_groups = string({localGetField(trimResultA, 'mode', 'unknown'); localGetField(trimResultB, 'mode', 'unknown')});
controllerData.K_dispatch = gainSchedule;
controllerData.x_trim = stateSchedule(1:9, :);
controllerData.trim_cmd = trimSchedule;
controllerData.schedule_table = table( ...
    string({nameA; nameB}), ...
    stateSchedule(10, :).', ...
    stateSchedule(11, :).', ...
    prog(:), ...
    'VariableNames', {'name', 'tilt_deg', 'vinf_mps', 'progress'});

% Legacy view keeps the initial point so the old compatibility variables
% still look sane if someone inspects them after Run_EVTOL_Main.
controllerData.K_lqr_cruise = ctrlA.K_lqr_cruise;
controllerData.x_trim_lqr = ctrlA.x_trim_lqr;
controllerData.U_trim_lqr = ctrlA.U_trim_lqr;
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'variant_ctrl_mode') || isempty(opts.variant_ctrl_mode)
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id') || isempty(opts.controller_id)
    opts.controller_id = 4;
end
if ~isfield(opts, 'progress_pair') || isempty(opts.progress_pair)
    opts.progress_pair = [0.0, 1.0];
end
if ~isfield(opts, 'lqr_opts') || isempty(opts.lqr_opts)
    opts.lqr_opts = struct();
end
opts.lqr_opts.variant_ctrl_mode = opts.variant_ctrl_mode;
opts.lqr_opts.controller_id = opts.controller_id;
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
end
end

function tiltDeg = localResolveTiltDeg(trimResult)
tiltDeg = localGetField(trimResult, 'front_tilt_deg', NaN);
if ~isfinite(tiltDeg) && isfield(trimResult, 'trimSpec') && isfield(trimResult.trimSpec, 'case') %#ok<ISMAT>
    tiltDeg = localGetField(trimResult.trimSpec.case, 'front_tilt_deg', NaN);
end
if ~isfinite(tiltDeg) && isfield(trimResult, 'case')
    tiltDeg = localGetField(trimResult.case, 'front_tilt_deg', NaN);
end
if ~isfinite(tiltDeg) && isfield(trimResult, 'scheduling')
    tiltDeg = localGetField(trimResult.scheduling, 'front_tilt_deg', NaN);
end
if ~isfinite(tiltDeg)
    tiltDeg = 90.0;
end
end

function vinf = localResolveVinf(trimResult)
vinf = NaN;
if isfield(trimResult, 'Vel_W_Trim') && numel(trimResult.Vel_W_Trim) >= 1
    vinf = trimResult.Vel_W_Trim(1);
end
if ~isfinite(vinf) && isfield(trimResult, 'scheduling')
    vinf = localGetField(trimResult.scheduling, 'Vinf_mps', NaN);
end
if ~isfinite(vinf) && isfield(trimResult, 'case')
    vinf = localGetField(trimResult.case, 'Vinf_mps', NaN);
end
if ~isfinite(vinf)
    vinf = 0.0;
end
end
