function controllerData = build_indi_transition_controller(opts)
%BUILD_INDI_TRANSITION_CONTROLLER Build path + INDI schedules for Wrapper.
%
% This builder runs outside Simulink. It:
%   1. loads the current Vinf/alpha trim attempt database,
%   2. finds a smooth hover-to-cruise path through acceptable candidates,
%   3. samples the aerodynamic surface-effectiveness map at each path point,
%   4. adds analytic propeller effectiveness columns, and
%   5. packs everything into numeric schedules consumed by controller_id = 6.

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
previousDir = pwd;
cleanupDir = onCleanup(@() cd(previousDir));
cd(repoRoot);

evalin('base', 'Init_Main');

trimTable = localLoadTrimCandidateTable(opts.trim_db_file);
indiMaps = localLoadIndiMaps(opts.indi_map_file);
plant = localLoadPlantDataFromBase();

[pathTable, pathDebug] = localSelectPath(trimTable, opts.path, plant);
nPts = height(pathTable);
if nPts < 2
    error('build_indi_transition_controller:PathTooShort', ...
        'Need at least two selected path points.');
end

stateSchedule = zeros(24, nPts);
trimSchedule = zeros(6, nPts);
indiSchedule = zeros(6, 9, nPts);
progress = localPathProgress(pathTable);

for idx = 1:nPts
    row = pathTable(idx, :);

    stateSchedule(1:9, idx) = localStateVectorFromPathRow(row);
    stateSchedule(10, idx) = row.tilt_deg;
    stateSchedule(11, idx) = row.vinf_mps;
    stateSchedule(12, idx) = progress(idx);
    stateSchedule(13, idx) = opts.gating.settle_theta_deg;
    stateSchedule(14, idx) = opts.gating.settle_u_mps;
    stateSchedule(15, idx) = opts.gating.settle_w_mps;
    stateSchedule(16, idx) = opts.gating.settle_q_deg_s;
    stateSchedule(17, idx) = opts.gating.settle_accel_norm;
    stateSchedule(18, idx) = opts.gating.settle_time_s;
    stateSchedule(19, idx) = opts.gating.segment_ramp_time_s;
    outerAtPoint = localOuterLoopAtPoint(row, opts.outer_loop);
    stateSchedule(20, idx) = outerAtPoint.ku;
    stateSchedule(21, idx) = outerAtPoint.kw;
    stateSchedule(22, idx) = outerAtPoint.kq;
    stateSchedule(23, idx) = outerAtPoint.ktheta;
    stateSchedule(24, idx) = outerAtPoint.accel_error_clip;

    trimSchedule(:, idx) = localTrimVectorFromPathRow(row);
    G = localBuildEffectivenessAtPoint(row, indiMaps, plant);
    indiSchedule(1:3, 1:4, idx) = G;
    indiSchedule(4, 1:4, idx) = opts.allocation.control_regularization(:).';
    indiSchedule(5, 1:3, idx) = opts.allocation.virtual_error_weights(:).';
    indiSchedule(6, 1:4, idx) = opts.allocation.delta_eta_limits(:).';
    indiSchedule(6, 5, idx) = opts.allocation.rotor_trim_feedforward_blend;
    indiSchedule(6, 6, idx) = opts.allocation.surface_trim_feedforward_blend;
end

runtimeGMap = zeros(6, 9, 1);
runtimeGInfo = struct('enabled', false);
if opts.runtime_g.enabled
    [runtimeGMap, runtimeGInfo] = localBuildRuntimeSurfaceGMap( ...
        indiMaps, plant, opts.runtime_g);
end

controllerData = struct();
controllerData.name = sprintf('TransitionINDI_%dpt', nPts);
controllerData.type = 'transition_corridor_indi';
controllerData.design_source = mfilename;
controllerData.variant_ctrl_mode = opts.variant_ctrl_mode;
controllerData.controller_id = opts.controller_id;
controllerData.controller_state_ref = stateSchedule;
controllerData.controller_trim_cmd = trimSchedule;
controllerData.controller_gain_lqr = indiSchedule;
controllerData.controller_runtime_g_map = runtimeGMap;
controllerData.schedule_count = nPts;
controllerData.schedule_progress = progress(:);
controllerData.schedule_tilt_deg = pathTable.tilt_deg;
controllerData.schedule_vinf_mps = pathTable.vinf_mps;
controllerData.schedule_alpha_deg = pathTable.alpha_deg;
controllerData.schedule_table = pathTable;
controllerData.path_debug = pathDebug;
controllerData.gating_opts = opts.gating;
controllerData.outer_loop = opts.outer_loop;
controllerData.allocation = opts.allocation;
controllerData.runtime_g = runtimeGInfo;
controllerData.trim_db_file = opts.trim_db_file;
controllerData.indi_map_file = opts.indi_map_file;
controllerData.x_trim = stateSchedule(1:9, :);
controllerData.trim_cmd = trimSchedule;

% Compatibility fields consumed by Run_Main and older plotting helpers.
controllerData.K_lqr_cruise = zeros(5, 9);
controllerData.x_trim_lqr = stateSchedule(1:9, end);
controllerData.U_trim_lqr = [trimSchedule(1, end); trimSchedule(3:6, end)];

assignin('base', 'controllerData', controllerData);
assignin('base', 'indiTransitionPathTable', pathTable);
assignin('base', 'indiTransitionPathDebug', pathDebug);

fprintf('\n=== INDI Transition Controller Build ===\n');
fprintf('Path points    : %d\n', nPts);
fprintf('Controller id  : %d\n', controllerData.controller_id);
fprintf('Variant ctrlMode: %d\n', controllerData.variant_ctrl_mode);
fprintf('Trim DB        : %s\n', opts.trim_db_file);
fprintf('INDI map DB    : %s\n', opts.indi_map_file);
if isfield(runtimeGInfo, 'enabled') && runtimeGInfo.enabled
    fprintf('Runtime G map  : %s (%d x %d x %d grid, %d packed pages)\n', ...
        runtimeGInfo.entry_name, runtimeGInfo.n_vinf, runtimeGInfo.n_alpha, ...
        runtimeGInfo.n_delta, runtimeGInfo.page_count);
end
fprintf('\nSelected path:\n');
localDisplaySelectedPath(pathTable);
fprintf('=== INDI Transition Build Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if ~isfield(opts, 'trim_db_file') || isempty(opts.trim_db_file)
    opts.trim_db_file = fullfile(repoRoot, 'databases', ...
        'trim_vinf_alpha_v1', 'trim_attempts.mat');
end
if ~isfield(opts, 'indi_map_file') || isempty(opts.indi_map_file)
    opts.indi_map_file = fullfile(repoRoot, 'databases', ...
        'indi_surface_effectiveness_maps_polar_fast.mat');
end
if ~isfield(opts, 'variant_ctrl_mode') || isempty(opts.variant_ctrl_mode)
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id') || isempty(opts.controller_id)
    opts.controller_id = 6;
end
if ~isfield(opts, 'path') || isempty(opts.path)
    opts.path = struct();
end
if ~isfield(opts, 'gating') || isempty(opts.gating)
    opts.gating = struct();
end
if ~isfield(opts, 'outer_loop') || isempty(opts.outer_loop)
    opts.outer_loop = struct();
end
if ~isfield(opts, 'allocation') || isempty(opts.allocation)
    opts.allocation = struct();
end
if ~isfield(opts, 'runtime_g') || isempty(opts.runtime_g)
    opts.runtime_g = struct();
end

opts.path = localApplyPathDefaults(opts.path);
opts.gating = localApplyGatingDefaults(opts.gating);
opts.outer_loop = localApplyOuterLoopDefaults(opts.outer_loop);
opts.allocation = localApplyAllocationDefaults(opts.allocation);
opts.runtime_g = localApplyRuntimeGDefaults(opts.runtime_g);
end

function path = localApplyPathDefaults(path)
if ~isfield(path, 'guide_vinf_mps') || isempty(path.guide_vinf_mps)
    path.guide_vinf_mps = [0 5 10 15 20 30 40 50 60 70];
end
if ~isfield(path, 'guide_tilt_deg') || isempty(path.guide_tilt_deg)
    path.guide_tilt_deg = [0 0 10 20 35 55 70 80 87 90];
end
if ~isfield(path, 'vinf_window_mps') || isempty(path.vinf_window_mps)
    path.vinf_window_mps = 25.0;
end
if ~isfield(path, 'tilt_window_deg') || isempty(path.tilt_window_deg)
    path.tilt_window_deg = 35.0;
end
if ~isfield(path, 'max_candidates_per_waypoint') || isempty(path.max_candidates_per_waypoint)
    path.max_candidates_per_waypoint = 120;
end
if ~isfield(path, 'allowed_classifications') || isempty(path.allowed_classifications)
    path.allowed_classifications = ["exact_trim", "quasi_trim_usable"];
end
if ~isfield(path, 'max_abs_alpha_deg') || isempty(path.max_abs_alpha_deg)
    path.max_abs_alpha_deg = 25.0;
end
if ~isfield(path, 'use_effective_alpha_filter') || isempty(path.use_effective_alpha_filter)
    path.use_effective_alpha_filter = true;
end
if ~isfield(path, 'effective_alpha_hard_filter') || isempty(path.effective_alpha_hard_filter)
    path.effective_alpha_hard_filter = true;
end
if ~isfield(path, 'effective_alpha_min_vinf_mps') || isempty(path.effective_alpha_min_vinf_mps)
    path.effective_alpha_min_vinf_mps = 5.0;
end
if ~isfield(path, 'max_abs_effective_alpha_deg') || isempty(path.max_abs_effective_alpha_deg)
    path.max_abs_effective_alpha_deg = 12.0;
end
if ~isfield(path, 'effective_alpha_penalty_start_deg') || isempty(path.effective_alpha_penalty_start_deg)
    path.effective_alpha_penalty_start_deg = 8.0;
end
if ~isfield(path, 'point_weight_effective_alpha') || isempty(path.point_weight_effective_alpha)
    path.point_weight_effective_alpha = 1.25;
end
if ~isfield(path, 'edge_weight_effective_alpha') || isempty(path.edge_weight_effective_alpha)
    path.edge_weight_effective_alpha = 0.80;
end
if ~isfield(path, 'flap_effective_alpha_tau') || isempty(path.flap_effective_alpha_tau)
    path.flap_effective_alpha_tau = 0.55;
end
if ~isfield(path, 'elevator_effective_alpha_tau') || isempty(path.elevator_effective_alpha_tau)
    path.elevator_effective_alpha_tau = 0.55;
end
if ~isfield(path, 'max_abs_surface_deg') || isempty(path.max_abs_surface_deg)
    path.max_abs_surface_deg = 22.0;
end
if ~isfield(path, 'max_abs_theta_deg') || isempty(path.max_abs_theta_deg)
    path.max_abs_theta_deg = 45.0;
end
if ~isfield(path, 'monotonic_vinf') || isempty(path.monotonic_vinf)
    path.monotonic_vinf = true;
end
if ~isfield(path, 'monotonic_tilt') || isempty(path.monotonic_tilt)
    path.monotonic_tilt = true;
end
if ~isfield(path, 'force_guide_endpoints') || isempty(path.force_guide_endpoints)
    path.force_guide_endpoints = true;
end
if ~isfield(path, 'guide_tracking_weight') || isempty(path.guide_tracking_weight)
    path.guide_tracking_weight = 0.05;
end
if ~isfield(path, 'point_weight_alpha') || isempty(path.point_weight_alpha)
    path.point_weight_alpha = 0.20;
end
if ~isfield(path, 'point_weight_surface') || isempty(path.point_weight_surface)
    path.point_weight_surface = 0.15;
end
if ~isfield(path, 'point_weight_score') || isempty(path.point_weight_score)
    path.point_weight_score = 0.10;
end
if ~isfield(path, 'edge_weight_actuator') || isempty(path.edge_weight_actuator)
    path.edge_weight_actuator = 1.0;
end
if ~isfield(path, 'edge_weight_alpha') || isempty(path.edge_weight_alpha)
    path.edge_weight_alpha = 0.60;
end
if ~isfield(path, 'edge_weight_theta') || isempty(path.edge_weight_theta)
    path.edge_weight_theta = 0.40;
end
end

function gating = localApplyGatingDefaults(gating)
gating = localDefaultScalar(gating, 'settle_theta_deg', 5.0);
gating = localDefaultScalar(gating, 'settle_u_mps', 3.0);
gating = localDefaultScalar(gating, 'settle_w_mps', 3.0);
gating = localDefaultScalar(gating, 'settle_q_deg_s', 8.0);
gating = localDefaultScalar(gating, 'settle_accel_norm', 6.0);
gating = localDefaultScalar(gating, 'settle_time_s', 0.35);
gating = localDefaultScalar(gating, 'segment_ramp_time_s', 5.0);
end

function outer = localApplyOuterLoopDefaults(outer)
outer = localDefaultScalar(outer, 'ku', 0.08);
outer = localDefaultScalar(outer, 'ku_high_speed', 0.16);
outer = localDefaultScalar(outer, 'ku_schedule_start_vinf_mps', 50.0);
outer = localDefaultScalar(outer, 'ku_schedule_end_vinf_mps', 70.0);
outer = localDefaultScalar(outer, 'kw', 0.20);
outer = localDefaultScalar(outer, 'kw_high_speed', 0.65);
outer = localDefaultScalar(outer, 'kq', 0.35);
outer = localDefaultScalar(outer, 'ktheta', 0.45);
outer = localDefaultScalar(outer, 'ktheta_high_speed', 1.20);
outer = localDefaultScalar(outer, 'accel_error_clip', 2.0);
end

function outerAtPoint = localOuterLoopAtPoint(row, outer)
outerAtPoint = outer;
lambda = (row.vinf_mps - outer.ku_schedule_start_vinf_mps) / ...
    max(outer.ku_schedule_end_vinf_mps - outer.ku_schedule_start_vinf_mps, 1.0);
lambda = min(max(lambda, 0.0), 1.0);
outerAtPoint.ku = (1.0 - lambda) * outer.ku + lambda * outer.ku_high_speed;
outerAtPoint.kw = (1.0 - lambda) * outer.kw + lambda * outer.kw_high_speed;
outerAtPoint.ktheta = (1.0 - lambda) * outer.ktheta + lambda * outer.ktheta_high_speed;
end

function allocation = localApplyAllocationDefaults(allocation)
if ~isfield(allocation, 'control_regularization') || isempty(allocation.control_regularization)
    allocation.control_regularization = [2.0e-7; 2.0e-7; 6.0; 6.0];
end
if ~isfield(allocation, 'virtual_error_weights') || isempty(allocation.virtual_error_weights)
    allocation.virtual_error_weights = [0.25; 1.0; 0.6];
end
if ~isfield(allocation, 'delta_eta_limits') || isempty(allocation.delta_eta_limits)
    allocation.delta_eta_limits = [5.0e5; 5.0e5; deg2rad(2.0); deg2rad(2.0)];
end
if isfield(allocation, 'trim_feedforward_blend') && ...
        ~isempty(allocation.trim_feedforward_blend)
    legacyBlend = allocation.trim_feedforward_blend;
else
    legacyBlend = [];
end
if ~isfield(allocation, 'rotor_trim_feedforward_blend') || ...
        isempty(allocation.rotor_trim_feedforward_blend)
    if isempty(legacyBlend)
        allocation.rotor_trim_feedforward_blend = 0.0;
    else
        allocation.rotor_trim_feedforward_blend = legacyBlend;
    end
end
if ~isfield(allocation, 'surface_trim_feedforward_blend') || ...
        isempty(allocation.surface_trim_feedforward_blend)
    allocation.surface_trim_feedforward_blend = 0.0;
end
allocation.control_regularization = allocation.control_regularization(:);
allocation.virtual_error_weights = allocation.virtual_error_weights(:);
allocation.delta_eta_limits = allocation.delta_eta_limits(:);
allocation.rotor_trim_feedforward_blend = ...
    min(max(allocation.rotor_trim_feedforward_blend, 0.0), 1.0);
allocation.surface_trim_feedforward_blend = ...
    min(max(allocation.surface_trim_feedforward_blend, 0.0), 1.0);
end

function runtimeG = localApplyRuntimeGDefaults(runtimeG)
runtimeG = localDefaultLogical(runtimeG, 'enabled', false);
if ~isfield(runtimeG, 'entry_name') || isempty(runtimeG.entry_name)
    runtimeG.entry_name = "stall_boundary_delta_alpha_dense";
end
end

function s = localDefaultScalar(s, fieldName, value)
if ~isfield(s, fieldName) || isempty(s.(fieldName))
    s.(fieldName) = value;
end
end

function s = localDefaultLogical(s, fieldName, value)
if ~isfield(s, fieldName) || isempty(s.(fieldName))
    s.(fieldName) = value;
end
s.(fieldName) = logical(s.(fieldName));
end

function localDisplaySelectedPath(pathTable)
summaryVars = {'path_index', 'vinf_mps', 'tilt_deg', 'alpha_deg', ...
    'max_abs_effective_alpha_deg', 'max_abs_wing_effective_alpha_deg', ...
    'max_abs_tail_effective_alpha_deg', 'theta_deg', ...
    'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_e_deg', 'classification'};
keepMask = ismember(summaryVars, pathTable.Properties.VariableNames);
disp(pathTable(:, summaryVars(keepMask)));
end

function trimTable = localLoadTrimCandidateTable(dbFile)
if exist(dbFile, 'file') ~= 2
    error('build_indi_transition_controller:MissingTrimDb', ...
        'Trim attempt DB not found: %s', dbFile);
end

try
    data = load(dbFile);
catch ME
    trimTable = localLoadTrimCandidateTableCsvFallback(dbFile, ME);
    return;
end
if isfield(data, 'transitionTrimMasterAttemptDB')
    db = data.transitionTrimMasterAttemptDB;
    if isfield(db, 'master_attempt_db_best_unique_points')
        trimTable = db.master_attempt_db_best_unique_points;
        return;
    end
end
if isfield(data, 'controllerScheduleDB')
    db = data.controllerScheduleDB;
    if isfield(db, 'summary_table')
        trimTable = db.summary_table;
        return;
    end
end

error('build_indi_transition_controller:UnsupportedTrimDb', ...
    'Could not find a supported trim table in %s.', dbFile);
end

function trimTable = localLoadTrimCandidateTableCsvFallback(dbFile, loadError)
[dbDir, dbBase, ~] = fileparts(dbFile);
csvFile = fullfile(dbDir, [dbBase '.csv']);
if exist(csvFile, 'file') ~= 2
    rethrow(loadError);
end

warning('build_indi_transition_controller:TrimMatLoadFailedCsvFallback', ...
    ['Could not load trim MAT DB "%s" (%s). Falling back to CSV "%s". ', ...
     'This usually means another process is writing the MAT file.'], ...
    dbFile, loadError.message, csvFile);
trimTable = readtable(csvFile, 'TextType', 'string');
end

function indiMaps = localLoadIndiMaps(mapFile)
if exist(mapFile, 'file') ~= 2
    error('build_indi_transition_controller:MissingIndiMap', ...
        'INDI effectiveness map DB not found: %s', mapFile);
end
data = load(mapFile);
if isfield(data, 'indiMaps')
    indiMaps = data.indiMaps;
elseif isfield(data, 'map')
    indiMaps = struct('entries', struct('name', 'single', 'map', data.map));
else
    error('build_indi_transition_controller:UnsupportedIndiMap', ...
        'Could not find indiMaps or map in %s.', mapFile);
end
end

function plant = localLoadPlantDataFromBase()
plant = struct();
plant.Mass = evalin('base', 'Mass');
plant.J = evalin('base', 'J');
plant.CG = evalin('base', 'CG');
plant.prop = evalin('base', 'prop');
plant.wingL = evalin('base', 'wingL');
plant.wingR = evalin('base', 'wingR');
plant.tailL = evalin('base', 'tailL');
plant.tailR = evalin('base', 'tailR');
end

function [pathTable, debug] = localSelectPath(trimTable, pathOpts, plant)
candidateTable = localFilterCandidateTable(trimTable, pathOpts, plant);
[candidateSets, candidateCosts, candidateMeta] = localBuildCandidateSets(candidateTable, pathOpts);
bestRows = localDynamicProgrammingPath(candidateTable, candidateSets, candidateCosts, pathOpts);

if isempty(bestRows)
    error('build_indi_transition_controller:NoPath', ...
        'No feasible INDI path found. Relax path filters/windows.');
end

pathTable = candidateTable(bestRows, :);
[pathTable, keptPathMask] = localDropConsecutiveDuplicatePathRows(pathTable);
if height(pathTable) < 2
    error('build_indi_transition_controller:PathTooShortAfterDedupe', ...
        'Selected INDI path collapsed to fewer than two unique kinematic points.');
end
pathTable.path_index = (1:height(pathTable)).';
pathTable.path_progress = localPathProgress(pathTable).';
debug = struct();
debug.candidate_meta = candidateMeta;
debug.selected_source_rows = bestRows(keptPathMask);
debug.dropped_duplicate_path_points = sum(~keptPathMask);
debug.effective_alpha_opts = localEffectiveAlphaDebugOptions(pathOpts);
end

function [pathTable, keepMask] = localDropConsecutiveDuplicatePathRows(pathTable)
keepMask = true(height(pathTable), 1);
if height(pathTable) <= 1
    return;
end

for i = 2:height(pathTable)
    sameKinematics = abs(pathTable.vinf_mps(i) - pathTable.vinf_mps(i - 1)) < 1e-9 && ...
        abs(pathTable.tilt_deg(i) - pathTable.tilt_deg(i - 1)) < 1e-9 && ...
        abs(pathTable.alpha_deg(i) - pathTable.alpha_deg(i - 1)) < 1e-9 && ...
        abs(pathTable.theta_deg(i) - pathTable.theta_deg(i - 1)) < 1e-9;
    if sameKinematics
        keepMask(i) = false;
    end
end

pathTable = pathTable(keepMask, :);
end

function optsDebug = localEffectiveAlphaDebugOptions(pathOpts)
optsDebug = struct();
optsDebug.enabled = pathOpts.use_effective_alpha_filter;
optsDebug.hard_filter = pathOpts.effective_alpha_hard_filter;
optsDebug.min_vinf_mps = pathOpts.effective_alpha_min_vinf_mps;
optsDebug.max_abs_effective_alpha_deg = pathOpts.max_abs_effective_alpha_deg;
optsDebug.penalty_start_deg = pathOpts.effective_alpha_penalty_start_deg;
optsDebug.point_weight = pathOpts.point_weight_effective_alpha;
optsDebug.edge_weight = pathOpts.edge_weight_effective_alpha;
optsDebug.flap_tau = pathOpts.flap_effective_alpha_tau;
optsDebug.elevator_tau = pathOpts.elevator_effective_alpha_tau;
end

function tbl = localFilterCandidateTable(tbl, pathOpts, plant)
required = {'tilt_deg', 'vinf_mps', 'alpha_deg', 'theta_deg', ...
    'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_e_deg', 'classification'};
for i = 1:numel(required)
    if ~ismember(required{i}, tbl.Properties.VariableNames)
        error('build_indi_transition_controller:MissingColumn', ...
            'Trim table is missing required column "%s".', required{i});
    end
end

classMask = false(height(tbl), 1);
allowed = string(pathOpts.allowed_classifications(:));
for i = 1:numel(allowed)
    classMask = classMask | strcmp(string(tbl.classification), allowed(i));
end

finiteMask = isfinite(tbl.tilt_deg) & isfinite(tbl.vinf_mps) & ...
    isfinite(tbl.alpha_deg) & isfinite(tbl.theta_deg) & ...
    isfinite(tbl.front_collective_rpm) & isfinite(tbl.rear_collective_rpm) & ...
    isfinite(tbl.delta_f_deg) & isfinite(tbl.delta_e_deg);

qualityMask = classMask & finiteMask & ...
    abs(tbl.alpha_deg) <= pathOpts.max_abs_alpha_deg & ...
    abs(tbl.theta_deg) <= pathOpts.max_abs_theta_deg & ...
    abs(tbl.delta_f_deg) <= pathOpts.max_abs_surface_deg & ...
    abs(tbl.delta_e_deg) <= pathOpts.max_abs_surface_deg;

if pathOpts.use_effective_alpha_filter
    tbl = localAddEffectiveAlphaMetrics(tbl, pathOpts, plant);
    if pathOpts.effective_alpha_hard_filter
        active = tbl.effective_alpha_filter_active;
        qualityMask = qualityMask & (~active | ...
            tbl.max_abs_effective_alpha_deg <= pathOpts.max_abs_effective_alpha_deg);
    end
end

if ismember('acceptable', tbl.Properties.VariableNames)
    qualityMask = qualityMask & tbl.acceptable;
end
if ismember('rear_on_ok', tbl.Properties.VariableNames)
    qualityMask = qualityMask & tbl.rear_on_ok;
end

tbl = tbl(qualityMask, :);
if isempty(tbl)
    error('build_indi_transition_controller:NoCandidates', ...
        'No trim candidates survived INDI path filters.');
end
tbl.source_row_index = (1:height(tbl)).';
end

function tbl = localAddEffectiveAlphaMetrics(tbl, pathOpts, plant)
deltaA = localTableColumnOrZeros(tbl, 'delta_a_deg');
deltaR = localTableColumnOrZeros(tbl, 'delta_r_deg');
betaDeg = localTableColumnOrZeros(tbl, 'beta_deg');

wingLDelta = tbl.delta_f_deg + deltaA;
wingRDelta = tbl.delta_f_deg - deltaA;
tailLDelta = tbl.delta_e_deg - deltaR;
tailRDelta = tbl.delta_e_deg + deltaR;

wingLTau = localSurfaceTau(plant.wingL, pathOpts.flap_effective_alpha_tau);
wingRTau = localSurfaceTau(plant.wingR, pathOpts.flap_effective_alpha_tau);
tailLTau = localSurfaceTau(plant.tailL, pathOpts.elevator_effective_alpha_tau);
tailRTau = localSurfaceTau(plant.tailR, pathOpts.elevator_effective_alpha_tau);

wingLAlphaGeom = localSurfaceGeomAlphaDeg(tbl.alpha_deg, betaDeg, plant.wingL);
wingRAlphaGeom = localSurfaceGeomAlphaDeg(tbl.alpha_deg, betaDeg, plant.wingR);
tailLAlphaGeom = localSurfaceGeomAlphaDeg(tbl.alpha_deg, betaDeg, plant.tailL);
tailRAlphaGeom = localSurfaceGeomAlphaDeg(tbl.alpha_deg, betaDeg, plant.tailR);

tbl.wing_left_effective_alpha_deg = wingLAlphaGeom + wingLTau .* wingLDelta;
tbl.wing_right_effective_alpha_deg = wingRAlphaGeom + wingRTau .* wingRDelta;
tbl.tail_left_effective_alpha_deg = tailLAlphaGeom + tailLTau .* tailLDelta;
tbl.tail_right_effective_alpha_deg = tailRAlphaGeom + tailRTau .* tailRDelta;
tbl.max_abs_wing_effective_alpha_deg = max(abs([ ...
    tbl.wing_left_effective_alpha_deg, tbl.wing_right_effective_alpha_deg]), [], 2);
tbl.max_abs_tail_effective_alpha_deg = max(abs([ ...
    tbl.tail_left_effective_alpha_deg, tbl.tail_right_effective_alpha_deg]), [], 2);
tbl.max_abs_effective_alpha_deg = max(abs([ ...
    tbl.wing_left_effective_alpha_deg, tbl.wing_right_effective_alpha_deg, ...
    tbl.tail_left_effective_alpha_deg, tbl.tail_right_effective_alpha_deg]), [], 2);
tbl.effective_alpha_filter_active = tbl.vinf_mps >= pathOpts.effective_alpha_min_vinf_mps;
end

function alphaGeomDeg = localSurfaceGeomAlphaDeg(alphaDeg, betaDeg, surface)
alphaRad = deg2rad(alphaDeg(:));
betaRad = deg2rad(betaDeg(:));
vDir = [ ...
    cos(alphaRad) .* cos(betaRad), ...
    sin(betaRad), ...
    sin(alphaRad) .* cos(betaRad)];

normal = surface.n(:);
normalNorm = norm(normal);
if normalNorm <= 0
    normal = [0; 0; -1];
else
    normal = normal ./ normalNorm;
end

normalProj = vDir * normal;
normalProj = min(max(normalProj, -1.0), 1.0);
alphaGeomDeg = rad2deg(surface.i - asin(normalProj));
end

function tau = localSurfaceTau(surface, fallback)
tau = fallback;
if isstruct(surface) && isfield(surface, 'ctrl_tau') && ~isempty(surface.ctrl_tau) && ...
        isfinite(surface.ctrl_tau)
    tau = surface.ctrl_tau;
end
end

function values = localTableColumnOrZeros(tbl, fieldName)
if ismember(fieldName, tbl.Properties.VariableNames)
    values = tbl.(fieldName);
    values(~isfinite(values)) = 0.0;
else
    values = zeros(height(tbl), 1);
end
end

function [candidateSets, candidateCosts, meta] = localBuildCandidateSets(tbl, pathOpts)
guideV = pathOpts.guide_vinf_mps(:);
guideT = pathOpts.guide_tilt_deg(:);
if numel(guideV) ~= numel(guideT)
    error('Guide V and tilt vectors must have the same length.');
end

nWp = numel(guideV);
candidateSets = cell(nWp, 1);
candidateCosts = cell(nWp, 1);
meta = repmat(struct('waypoint', 0, 'guide_vinf_mps', NaN, ...
    'guide_tilt_deg', NaN, 'candidate_count', 0), nWp, 1);

for i = 1:nWp
    dist = localWaypointDistance(tbl, guideV(i), guideT(i), pathOpts);
    inWindow = abs(tbl.vinf_mps - guideV(i)) <= pathOpts.vinf_window_mps & ...
        abs(tbl.tilt_deg - guideT(i)) <= pathOpts.tilt_window_deg;
    rows = find(inWindow);
    if isempty(rows)
        [~, order] = sort(dist, 'ascend');
        rows = order(1:min(pathOpts.max_candidates_per_waypoint, numel(order)));
    else
        [~, orderLocal] = sort(dist(rows), 'ascend');
        rows = rows(orderLocal(1:min(pathOpts.max_candidates_per_waypoint, numel(orderLocal))));
    end
    if pathOpts.force_guide_endpoints && (i == 1 || i == nWp)
        [~, endpointOrder] = sort(localPureGuideDistance(tbl(rows, :), guideV(i), guideT(i)), ...
            'ascend');
        rows = rows(endpointOrder(1));
    end
    candidateSets{i} = rows(:);
    candidateCosts{i} = dist(rows(:));
    meta(i).waypoint = i;
    meta(i).guide_vinf_mps = guideV(i);
    meta(i).guide_tilt_deg = guideT(i);
    meta(i).candidate_count = numel(rows);
end
end

function dist = localWaypointDistance(tbl, guideV, guideT, pathOpts)
dv = (tbl.vinf_mps - guideV) ./ max(pathOpts.vinf_window_mps, 1.0);
dt = (tbl.tilt_deg - guideT) ./ max(pathOpts.tilt_window_deg, 1.0);
dist = pathOpts.guide_tracking_weight * (dv.^2 + dt.^2) + ...
    pathOpts.point_weight_alpha * (tbl.alpha_deg ./ 20.0).^2 + ...
    pathOpts.point_weight_surface * ...
        ((tbl.delta_f_deg ./ 25.0).^2 + (tbl.delta_e_deg ./ 25.0).^2);
if pathOpts.use_effective_alpha_filter && ...
        ismember('max_abs_effective_alpha_deg', tbl.Properties.VariableNames)
    excess = max(tbl.max_abs_effective_alpha_deg - ...
        pathOpts.effective_alpha_penalty_start_deg, 0.0);
    scale = max(pathOpts.max_abs_effective_alpha_deg - ...
        pathOpts.effective_alpha_penalty_start_deg, 1.0);
    active = localTableLogicalOrTrue(tbl, 'effective_alpha_filter_active');
    dist = dist + pathOpts.point_weight_effective_alpha * active .* (excess ./ scale).^2;
end
if ismember('score', tbl.Properties.VariableNames)
    score = tbl.score;
    score(~isfinite(score)) = 0.0;
    dist = dist + pathOpts.point_weight_score * min(max(score, 0.0), 10.0);
end
end

function dist = localPureGuideDistance(tbl, guideV, guideT)
dist = ((tbl.vinf_mps - guideV) ./ 10.0).^2 + ...
    ((tbl.tilt_deg - guideT) ./ 15.0).^2;
end

function active = localTableLogicalOrTrue(tbl, fieldName)
if ismember(fieldName, tbl.Properties.VariableNames)
    active = double(logical(tbl.(fieldName)));
else
    active = ones(height(tbl), 1);
end
end

function bestRows = localDynamicProgrammingPath(tbl, candidateSets, candidateCosts, pathOpts)
nWp = numel(candidateSets);
if nWp == 0
    bestRows = [];
    return;
end

cost = cell(nWp, 1);
prev = cell(nWp, 1);
for i = 1:nWp
    cost{i} = inf(numel(candidateSets{i}), 1);
    prev{i} = zeros(numel(candidateSets{i}), 1);
end
cost{1}(:) = candidateCosts{1}(:);

for i = 2:nWp
    rowsPrev = candidateSets{i - 1};
    rowsCurr = candidateSets{i};
    for c = 1:numel(rowsCurr)
        bestCost = inf;
        bestPrev = 0;
        for p = 1:numel(rowsPrev)
            edgeCost = localEdgeCost(tbl, rowsPrev(p), rowsCurr(c), pathOpts);
            totalCost = cost{i - 1}(p) + edgeCost + candidateCosts{i}(c);
            if totalCost < bestCost
                bestCost = totalCost;
                bestPrev = p;
            end
        end
        cost{i}(c) = bestCost;
        prev{i}(c) = bestPrev;
    end
end

[finalCost, finalIdx] = min(cost{end});
if ~isfinite(finalCost)
    bestRows = [];
    return;
end

bestRows = zeros(nWp, 1);
bestRows(end) = candidateSets{end}(finalIdx);
backIdx = finalIdx;
for i = nWp:-1:2
    backIdx = prev{i}(backIdx);
    if backIdx <= 0
        bestRows = [];
        return;
    end
    bestRows(i - 1) = candidateSets{i - 1}(backIdx);
end
end

function cost = localEdgeCost(tbl, prevRow, currRow, pathOpts)
prev = tbl(prevRow, :);
curr = tbl(currRow, :);

if pathOpts.monotonic_vinf && curr.vinf_mps < prev.vinf_mps - 1e-9
    cost = inf;
    return;
end
if pathOpts.monotonic_tilt && curr.tilt_deg < prev.tilt_deg - 1e-9
    cost = inf;
    return;
end

dFront = (curr.front_collective_rpm - prev.front_collective_rpm) / 500.0;
dRear = (curr.rear_collective_rpm - prev.rear_collective_rpm) / 500.0;
dDf = (curr.delta_f_deg - prev.delta_f_deg) / 10.0;
dDe = (curr.delta_e_deg - prev.delta_e_deg) / 10.0;
dAlpha = (curr.alpha_deg - prev.alpha_deg) / 10.0;
dTheta = (curr.theta_deg - prev.theta_deg) / 10.0;
dV = (curr.vinf_mps - prev.vinf_mps) / 10.0;
dTilt = (curr.tilt_deg - prev.tilt_deg) / 15.0;
dEffAlpha = 0.0;
if pathOpts.use_effective_alpha_filter && ...
        ismember('max_abs_effective_alpha_deg', tbl.Properties.VariableNames)
    dEffAlpha = (curr.max_abs_effective_alpha_deg - ...
        prev.max_abs_effective_alpha_deg) / 8.0;
end

cost = dV^2 + dTilt^2 + ...
    pathOpts.edge_weight_actuator * (dFront^2 + dRear^2 + dDf^2 + dDe^2) + ...
    pathOpts.edge_weight_alpha * dAlpha^2 + ...
    pathOpts.edge_weight_theta * dTheta^2 + ...
    pathOpts.edge_weight_effective_alpha * dEffAlpha^2;
end

function x = localStateVectorFromPathRow(row)
x = zeros(9, 1);
x(2) = deg2rad(row.theta_deg);
x(4) = row.u_mps;
x(6) = row.w_mps;
end

function trim = localTrimVectorFromPathRow(row)
trim = zeros(6, 1);
trim(1) = row.front_collective_rpm;
trim(2) = row.rear_collective_rpm;
trim(3) = deg2rad(row.delta_f_deg);
trim(4) = deg2rad(localTableValue(row, 'delta_a_deg', 0.0));
trim(5) = deg2rad(row.delta_e_deg);
trim(6) = deg2rad(localTableValue(row, 'delta_r_deg', 0.0));
end

function value = localTableValue(row, fieldName, fallback)
if ismember(fieldName, row.Properties.VariableNames)
    value = row.(fieldName);
else
    value = fallback;
end
end

function progress = localPathProgress(pathTable)
n = height(pathTable);
progress = zeros(1, n);
if n <= 1
    return;
end
for i = 2:n
    dV = pathTable.vinf_mps(i) - pathTable.vinf_mps(i - 1);
    dT = pathTable.tilt_deg(i) - pathTable.tilt_deg(i - 1);
    dA = pathTable.alpha_deg(i) - pathTable.alpha_deg(i - 1);
    progress(i) = progress(i - 1) + sqrt(dV^2 + dT^2 + 0.25 * dA^2);
end
if progress(end) > 0
    progress = progress ./ progress(end);
else
    progress = linspace(0.0, 1.0, n);
end
end

function [runtimeMap, info] = localBuildRuntimeSurfaceGMap(indiMaps, plant, runtimeG)
mapEntry = localSelectRuntimeMapEntry(indiMaps, runtimeG.entry_name);
map = mapEntry.map;
mass = plant.Mass;
iyy = plant.J(2, 2);

vGrid = map.grid.vinf_mps(:);
aGrid = map.grid.alpha_deg(:);
dGrid = map.grid.delta_deg(:);
nV = numel(vGrid);
nA = numel(aGrid);
nD = numel(dGrid);
nDataPages = nV * nA * nD;
nGridValues = nV + nA + nD;
nGridPages = ceil(nGridValues / 54);

headerPage = 1;
gridStartPage = headerPage + 1;
dataStartPage = gridStartPage + nGridPages;
nPagesOut = dataStartPage + nDataPages - 1;
runtimeMap = zeros(6, 9, nPagesOut);

marker = 6606.0;
runtimeMap(1, 1:6, headerPage) = [marker, nV, nA, nD, nGridPages, dataStartPage];

gridFlat = [vGrid; aGrid; dGrid];
for idx = 1:numel(gridFlat)
    pageOffset = floor((idx - 1) / 54);
    offsetInPage = idx - pageOffset * 54;
    row = ceil(offsetInPage / 9);
    col = offsetInPage - (row - 1) * 9;
    runtimeMap(row, col, gridStartPage + pageOffset) = gridFlat(idx);
end

for iD = 1:nD
    for iA = 1:nA
        for iV = 1:nV
            dataPage = dataStartPage + localRuntimeMapLinearIndex(iV, iA, iD, nV, nA) - 1;
            flapForce = squeeze(map.flap.dF_drad_N_per_rad(iV, iA, iD, :));
            flapMoment = squeeze(map.flap.dM_drad_Nm_per_rad(iV, iA, iD, :));
            elevForce = squeeze(map.elevator.dF_drad_N_per_rad(iV, iA, iD, :));
            elevMoment = squeeze(map.elevator.dM_drad_Nm_per_rad(iV, iA, iD, :));

            runtimeMap(1, 1:6, dataPage) = [ ...
                flapForce(1) / mass, flapForce(3) / mass, flapMoment(2) / iyy, ...
                elevForce(1) / mass, elevForce(3) / mass, elevMoment(2) / iyy];
        end
    end
end

info = struct();
info.enabled = true;
info.entry_name = string(mapEntry.name);
info.n_vinf = nV;
info.n_alpha = nA;
info.n_delta = nD;
info.header_page = headerPage;
info.grid_start_page = gridStartPage;
info.data_start_page = dataStartPage;
info.page_count = nPagesOut;
end

function mapEntry = localSelectRuntimeMapEntry(indiMaps, entryName)
entries = indiMaps.entries;
wanted = string(entryName);
for idx = 1:numel(entries)
    if string(entries(idx).name) == wanted
        mapEntry = entries(idx);
        return;
    end
end
available = strings(numel(entries), 1);
for idx = 1:numel(entries)
    available(idx) = string(entries(idx).name);
end
error('build_indi_transition_controller:MissingRuntimeGEntry', ...
    'Runtime G map entry "%s" was not found. Available entries: %s', ...
    wanted, strjoin(available, ', '));
end

function idx = localRuntimeMapLinearIndex(iV, iA, iD, nV, nA)
idx = (iD - 1) * nV * nA + (iA - 1) * nV + iV;
end

function G = localBuildEffectivenessAtPoint(row, indiMaps, plant)
mass = plant.Mass;
iyy = plant.J(2, 2);
G = zeros(3, 4);

tiltDeg = row.tilt_deg;
G(:, 1) = localPropColumn(plant.prop, plant.CG, mass, iyy, ...
    tiltDeg, "front");
G(:, 2) = localPropColumn(plant.prop, plant.CG, mass, iyy, ...
    0.0, "rear");

[flapForce, flapMoment] = localNearestSurfaceDerivative( ...
    indiMaps, "flap", row.vinf_mps, row.alpha_deg, row.delta_f_deg);
[elevForce, elevMoment] = localNearestSurfaceDerivative( ...
    indiMaps, "elevator", row.vinf_mps, row.alpha_deg, row.delta_e_deg);

G(1, 3) = flapForce(1) / mass;
G(2, 3) = flapForce(3) / mass;
G(3, 3) = flapMoment(2) / iyy;
G(1, 4) = elevForce(1) / mass;
G(2, 4) = elevForce(3) / mass;
G(3, 4) = elevMoment(2) / iyy;
end

function col = localPropColumn(prop, cg, mass, iyy, tiltDeg, groupName)
kT = prop.k_Thrust;
if groupName == "front"
    positions = [prop.posFR; prop.posFL];
    forceDir = [sind(tiltDeg); 0.0; -cosd(tiltDeg)];
else
    positions = [prop.posRR; prop.posRL];
    forceDir = [0.0; 0.0; -1.0];
end

dFtotal = zeros(3, 1);
dMtotal = zeros(3, 1);
for i = 1:size(positions, 1)
    r = positions(i, :).'-cg(:);
    dF = kT * forceDir;
    dM = cross(r, dF);
    dFtotal = dFtotal + dF;
    dMtotal = dMtotal + dM;
end

col = [dFtotal(1) / mass; dFtotal(3) / mass; dMtotal(2) / iyy];
end

function [forceDeriv, momentDeriv] = localNearestSurfaceDerivative( ...
    indiMaps, surfaceName, vinf, alphaDeg, deltaDeg)
entries = indiMaps.entries;
bestDist = inf;
forceDeriv = zeros(3, 1);
momentDeriv = zeros(3, 1);

for e = 1:numel(entries)
    map = entries(e).map;
    vGrid = map.grid.vinf_mps(:);
    aGrid = map.grid.alpha_deg(:);
    dGrid = map.grid.delta_deg(:);

    [dv, iV] = min(abs(vGrid - vinf));
    [da, iA] = min(abs(aGrid - alphaDeg));
    [dd, iD] = min(abs(dGrid - deltaDeg));
    dist = (dv / 10.0)^2 + (da / 5.0)^2 + (dd / 5.0)^2;

    if dist < bestDist
        bestDist = dist;
        surface = map.(char(surfaceName));
        forceDeriv = squeeze(surface.dF_drad_N_per_rad(iV, iA, iD, :));
        momentDeriv = squeeze(surface.dM_drad_Nm_per_rad(iV, iA, iD, :));
        forceDeriv = forceDeriv(:);
        momentDeriv = momentDeriv(:);
    end
end
end
