function controllerData = build_transition_jumpgap_lqr_controller(opts)
%BUILD_TRANSITION_JUMPGAP_LQR_CONTROLLER Build the scheduled LQR for the jump-gap path.

if nargin < 1 || isempty(opts)
    opts = struct();
end

opts = localApplyDefaults(opts);
[pathSpec, pathPoints, ~] = load_transition_jumpgap_path_points();
nPts = numel(pathPoints);
if nPts < 2
    error('Need at least two path points to build the jump-gap scheduled controller.');
end

stateSchedule = zeros(12, nPts);
trimSchedule = zeros(6, nPts);
gainSchedule = zeros(6, 9, nPts);
progress = localPathProgress(pathPoints);
summaryRows = repmat(struct( ...
    'index', 0, ...
    'name', "", ...
    'source_kind', "", ...
    'source_file', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'jump_after', false, ...
    'closed_loop_eigs', zeros(4, 1)), nPts, 1);

fprintf('\n=== Transition Jump-Gap LQR Controller Build ===\n');
fprintf('Path: %s\n', pathSpec.name);
fprintf('Points: %d\n', nPts);

for i = 1:nPts
    if opts.verbose
        ctrl = build_trim_point_longitudinal_lqr_controller(pathPoints(i).trimResult, opts.lqr_opts);
    else
        ctrl = [];
        evalc('ctrl = build_trim_point_longitudinal_lqr_controller(pathPoints(i).trimResult, opts.lqr_opts);');
    end
    stateSchedule(1:9, i) = ctrl.controller_state_ref(1:9, 1);
    stateSchedule(10, i) = pathPoints(i).tilt_deg;
    stateSchedule(11, i) = pathPoints(i).vinf_mps;
    stateSchedule(12, i) = progress(i);
    trimSchedule(:, i) = ctrl.controller_trim_cmd(:, 1);
    gainSchedule(:, :, i) = ctrl.controller_gain_lqr(:, :, 1);

    summaryRows(i).index = i;
    summaryRows(i).name = string(pathPoints(i).name);
    summaryRows(i).source_kind = string(pathPoints(i).source_kind);
    summaryRows(i).source_file = string(pathPoints(i).source_file);
    summaryRows(i).tilt_deg = pathPoints(i).tilt_deg;
    summaryRows(i).vinf_mps = pathPoints(i).vinf_mps;
    summaryRows(i).front_collective_rpm = pathPoints(i).front_collective_rpm;
    summaryRows(i).rear_collective_rpm = pathPoints(i).rear_collective_rpm;
    summaryRows(i).jump_after = pathPoints(i).jump_after;
    eigsLong = localGetField(ctrl, 'closed_loop_eigs', zeros(4, 1));
    summaryRows(i).closed_loop_eigs(1:numel(eigsLong)) = eigsLong(:);

    fprintf('  [%2d/%2d] %-32s tilt=%6.1f V=%6.1f rear=%7.1f %s\n', ...
        i, nPts, char(string(pathPoints(i).name)), ...
        pathPoints(i).tilt_deg, pathPoints(i).vinf_mps, ...
        pathPoints(i).rear_collective_rpm, localJumpLabel(pathPoints(i).jump_after));
end

controllerData = struct();
controllerData.name = sprintf('TransitionJumpGapLQR_%dpt', nPts);
controllerData.type = 'transition_jumpgap_lqr';
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
controllerData.schedule_names = string({pathPoints.name}).';
controllerData.schedule_groups = repmat("jumpgap_path", nPts, 1);
controllerData.schedule_table = struct2table(summaryRows);
controllerData.path_spec = pathSpec;
controllerData.path_points = pathPoints;
controllerData.K_dispatch = gainSchedule;
controllerData.x_trim = stateSchedule(1:9, :);
controllerData.trim_cmd = trimSchedule;

controllerData.K_lqr_cruise = gainSchedule([1 3 4 5 6], :, end);
controllerData.x_trim_lqr = stateSchedule(1:9, end);
controllerData.U_trim_lqr = [trimSchedule(1, end); trimSchedule(3:6, end)];

fprintf('Controller id = %d (scheduled path LQR)\n', controllerData.controller_id);
fprintf('Variant ctrlMode = %d (MATLAB Function dispatch)\n', controllerData.variant_ctrl_mode);
fprintf('=== Transition Jump-Gap LQR Build Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'variant_ctrl_mode') || isempty(opts.variant_ctrl_mode)
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id') || isempty(opts.controller_id)
    opts.controller_id = 4;
end
if ~isfield(opts, 'lqr_opts') || isempty(opts.lqr_opts)
    opts.lqr_opts = struct();
end
if ~isfield(opts, 'verbose') || isempty(opts.verbose)
    opts.verbose = false;
end
opts.lqr_opts.variant_ctrl_mode = opts.variant_ctrl_mode;
opts.lqr_opts.controller_id = opts.controller_id;
opts.lqr_opts.require_exact_trim = true;
end

function progress = localPathProgress(pathPoints)
nPts = numel(pathPoints);
progress = zeros(1, nPts);
if nPts <= 1
    return;
end

arc = zeros(1, nPts);
for i = 2:nPts
    prev = [pathPoints(i - 1).tilt_deg, pathPoints(i - 1).vinf_mps];
    curr = [pathPoints(i).tilt_deg, pathPoints(i).vinf_mps];
    arc(i) = arc(i - 1) + norm(curr - prev);
end
if arc(end) <= eps
    progress = linspace(0.0, 1.0, nPts);
else
    progress = arc / arc(end);
end
end

function label = localJumpLabel(isJump)
if isJump
    label = '<jump next>';
else
    label = '';
end
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
end
end
