function controllerData = build_corridor_lqr_controller(opts)
%BUILD_TRANSITION_CORRIDOR_LQR_CONTROLLER
% Build a scheduled LQR along an auto-selected controller-DB corridor path.

if nargin < 1 || isempty(opts)
    opts = struct();
end

opts = localApplyDefaults(opts);

selectorOpts = opts.selector_opts;
selectorOpts.plot_result = localGetField(selectorOpts, 'plot_result', false);
selectorOpts.show_popup = localGetField(selectorOpts, 'show_popup', false);

[pathSpec, selectedPoints, selectedPathTable, selectorDebug] = ...
    select_corridor_path_from_db([], selectorOpts); %#ok<ASGLU>

nPts = numel(selectedPoints);
if nPts < 2
    error('Need at least two selected corridor points to build a scheduled controller.');
end

stateSchedule = zeros(18, nPts);
trimSchedule = zeros(6, nPts);
gainSchedule = zeros(6, 9, nPts);
progress = localPathProgress(selectedPoints);
pathPoints = repmat(localPointTemplate(), nPts, 1);
summaryRows = repmat(struct( ...
    'index', 0, ...
    'name', "", ...
    'key', "", ...
    'family', "", ...
    'classification', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'guide_vinf_mps', NaN, ...
    'guide_tilt_deg', NaN, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'closed_loop_eigs', zeros(4, 1)), nPts, 1);

fprintf('\n=== Transition Corridor LQR Controller Build ===\n');
fprintf('Path: %s\n', pathSpec.name);
fprintf('Points: %d\n', nPts);

for i = 1:nPts
    point = selectedPoints(i);
    trimResult = trim_result_from_controller_db_point(point);
    if opts.verbose
        ctrl = build_trim_lqr_controller(trimResult, opts.lqr_opts);
    else
        ctrl = [];
        evalc('ctrl = build_trim_lqr_controller(trimResult, opts.lqr_opts);');
    end

    pathPoints(i).point = point;
    pathPoints(i).trimResult = trimResult;
    pathPoints(i).path_index = i;
    pathPoints(i).path_progress = progress(i);
    pathPoints(i).guide_vinf_mps = localGetField(point, 'guide_vinf_mps', NaN);
    pathPoints(i).guide_tilt_deg = localGetField(point, 'guide_tilt_deg', NaN);

    stateSchedule(1:9, i) = ctrl.controller_state_ref(1:9, 1);
    stateSchedule(10, i) = point.tilt_deg;
    stateSchedule(11, i) = point.vinf_mps;
    stateSchedule(12, i) = progress(i);
    stateSchedule(13, i) = opts.gating_opts.settle_theta_deg;
    stateSchedule(14, i) = opts.gating_opts.settle_u_mps;
    stateSchedule(15, i) = opts.gating_opts.settle_w_mps;
    stateSchedule(16, i) = opts.gating_opts.settle_q_deg_s;
    stateSchedule(17, i) = opts.gating_opts.settle_time_s;
    stateSchedule(18, i) = opts.gating_opts.segment_ramp_time_s;
    trimSchedule(:, i) = ctrl.controller_trim_cmd(:, 1);
    gainSchedule(:, :, i) = ctrl.controller_gain_lqr(:, :, 1);

    summaryRows(i).index = i;
    summaryRows(i).name = string(localGetField(point, 'name', ""));
    summaryRows(i).key = string(localGetField(point, 'key', ""));
    summaryRows(i).family = string(localGetField(point, 'family', ""));
    summaryRows(i).classification = string(localGetField(point, 'classification', ""));
    summaryRows(i).tilt_deg = point.tilt_deg;
    summaryRows(i).vinf_mps = point.vinf_mps;
    summaryRows(i).guide_vinf_mps = localGetField(point, 'guide_vinf_mps', NaN);
    summaryRows(i).guide_tilt_deg = localGetField(point, 'guide_tilt_deg', NaN);
    summaryRows(i).front_collective_rpm = localGetField(point, 'front_collective_rpm', NaN);
    summaryRows(i).rear_collective_rpm = localGetField(point, 'rear_collective_rpm', NaN);
    eigsLong = localGetField(ctrl, 'closed_loop_eigs', zeros(4, 1));
    summaryRows(i).closed_loop_eigs(1:numel(eigsLong)) = eigsLong(:);

    fprintf('  [%2d/%2d] %-52s tilt=%6.1f V=%6.1f rear=%7.1f\n', ...
        i, nPts, char(string(point.name)), point.tilt_deg, point.vinf_mps, ...
        localGetField(point, 'rear_collective_rpm', NaN));
end

controllerData = struct();
controllerData.name = sprintf('TransitionCorridorLQR_%dpt', nPts);
controllerData.type = 'transition_corridor_lqr';
controllerData.design_source = mfilename;
controllerData.variant_ctrl_mode = opts.variant_ctrl_mode;
controllerData.controller_id = opts.controller_id;
controllerData.controller_state_ref = stateSchedule;
controllerData.controller_trim_cmd = trimSchedule;
controllerData.controller_gain_lqr = gainSchedule;
controllerData.schedule_count = nPts;
controllerData.schedule_progress = progress(:);
controllerData.schedule_tilt_deg = stateSchedule(10, :).';
controllerData.schedule_vinf_mps = stateSchedule(11, :).';
controllerData.gating_opts = opts.gating_opts;
controllerData.schedule_names = string(arrayfun(@(p) string(p.point.name), pathPoints, 'UniformOutput', false)).';
controllerData.schedule_groups = repmat("corridor_path", nPts, 1);
controllerData.schedule_table = struct2table(summaryRows);
controllerData.path_spec = pathSpec;
controllerData.path_points = pathPoints;
controllerData.selected_path_table = selectedPathTable;
controllerData.selector_debug = selectorDebug;
controllerData.K_dispatch = gainSchedule;
controllerData.x_trim = stateSchedule(1:9, :);
controllerData.trim_cmd = trimSchedule;

controllerData.K_lqr_cruise = gainSchedule([1 3 4 5 6], :, end);
controllerData.x_trim_lqr = stateSchedule(1:9, end);
controllerData.U_trim_lqr = [trimSchedule(1, end); trimSchedule(3:6, end)];

fprintf('Controller id = %d (scheduled path LQR)\n', controllerData.controller_id);
fprintf('Variant ctrlMode = %d (MATLAB Function dispatch)\n', controllerData.variant_ctrl_mode);
fprintf('=== Transition Corridor LQR Build Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'variant_ctrl_mode') || isempty(opts.variant_ctrl_mode)
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id') || isempty(opts.controller_id)
    opts.controller_id = 5;
end
if ~isfield(opts, 'lqr_opts') || isempty(opts.lqr_opts)
    opts.lqr_opts = struct();
end
if ~isfield(opts, 'selector_opts') || isempty(opts.selector_opts)
    opts.selector_opts = struct();
end
if ~isfield(opts, 'gating_opts') || isempty(opts.gating_opts)
    opts.gating_opts = struct();
end
if ~isfield(opts, 'verbose') || isempty(opts.verbose)
    opts.verbose = false;
end

opts.lqr_opts.variant_ctrl_mode = opts.variant_ctrl_mode;
opts.lqr_opts.controller_id = opts.controller_id;
opts.lqr_opts.require_exact_trim = false;
opts.gating_opts = localApplyGatingDefaults(opts.gating_opts);
opts.selector_opts = localApplySelectorStepDefaults(opts.selector_opts);
end

function gatingOpts = localApplyGatingDefaults(gatingOpts)
if ~isfield(gatingOpts, 'settle_theta_deg') || isempty(gatingOpts.settle_theta_deg)
    gatingOpts.settle_theta_deg = 4.0;
end
if ~isfield(gatingOpts, 'settle_u_mps') || isempty(gatingOpts.settle_u_mps)
    gatingOpts.settle_u_mps = 2.0;
end
if ~isfield(gatingOpts, 'settle_w_mps') || isempty(gatingOpts.settle_w_mps)
    gatingOpts.settle_w_mps = 1.5;
end
if ~isfield(gatingOpts, 'settle_q_deg_s') || isempty(gatingOpts.settle_q_deg_s)
    gatingOpts.settle_q_deg_s = 4.0;
end
if ~isfield(gatingOpts, 'settle_time_s') || isempty(gatingOpts.settle_time_s)
    gatingOpts.settle_time_s = 0.5;
end
if ~isfield(gatingOpts, 'segment_ramp_time_s') || isempty(gatingOpts.segment_ramp_time_s)
    gatingOpts.segment_ramp_time_s = 4.0;
end
if ~isfield(gatingOpts, 'sample_time_s') || isempty(gatingOpts.sample_time_s)
    gatingOpts.sample_time_s = 0.01;
end
end

function selectorOpts = localApplySelectorStepDefaults(selectorOpts)
if ~isfield(selectorOpts, 'max_delta_vinf_per_step') || isempty(selectorOpts.max_delta_vinf_per_step)
    selectorOpts.max_delta_vinf_per_step = inf;
end
if ~isfield(selectorOpts, 'max_delta_tilt_per_step') || isempty(selectorOpts.max_delta_tilt_per_step)
    selectorOpts.max_delta_tilt_per_step = inf;
end
end

function progress = localPathProgress(selectedPoints)
nPts = numel(selectedPoints);
progress = zeros(1, nPts);
if nPts <= 1
    return;
end

arc = zeros(1, nPts);
for i = 2:nPts
    prev = [selectedPoints(i - 1).tilt_deg, selectedPoints(i - 1).vinf_mps];
    curr = [selectedPoints(i).tilt_deg, selectedPoints(i).vinf_mps];
    arc(i) = arc(i - 1) + norm(curr - prev);
end
if arc(end) <= eps
    progress = linspace(0.0, 1.0, nPts);
else
    progress = arc / arc(end);
end
end

function entry = localPointTemplate()
entry = struct( ...
    'point', struct(), ...
    'trimResult', struct(), ...
    'path_index', 0, ...
    'path_progress', NaN, ...
    'guide_vinf_mps', NaN, ...
    'guide_tilt_deg', NaN);
end

function value = localGetField(s, fieldName, fallback)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = fallback;
end
end
