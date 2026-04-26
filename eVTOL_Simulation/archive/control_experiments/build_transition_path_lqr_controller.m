function controllerData = build_transition_path_lqr_controller(dbInput, opts)
%BUILD_TRANSITION_PATH_LQR_CONTROLLER Build a scheduled LQR along a trim path.
%
% Usage:
%   controllerData = build_transition_path_lqr_controller()
%   controllerData = build_transition_path_lqr_controller(dbFile)
%   controllerData = build_transition_path_lqr_controller(dbFile, opts)
%
% The controller is parameterized by a path through (front_tilt_deg, Vinf)
% space. During simulation, the MATLAB-function dispatch branch projects the
% commanded (tilt, airspeed) pair onto this path and interpolates:
%   - x_ref(s)
%   - trim_cmd(s)
%   - K(s)
%
% Defaults:
%   dbInput = newest trim_linearization_db*.mat if no explicit latest file
%             is present.
%
% Optional opts fields:
%   variant_ctrl_mode = active controller variant (default 2)
%   controller_id     = dispatch helper id (default 4)
%   require_exact_trim = require replay_exact_trim on every point (default false)
%   Q_full            = 9x9 state penalty
%   Q_inner           = 8x8 state penalty when psi is excluded
%   R                 = 6x6 input penalty
%   guide_polyline    = optional override for path projection

if nargin < 1 || isempty(dbInput)
    dbInput = localResolveDefaultDb();
end
if nargin < 2 || isempty(opts)
    opts = struct();
end

opts = localApplyDefaults(opts);
[trimLinearizationDB, dbFile] = localLoadDb(dbInput);
guide = localResolveGuide(trimLinearizationDB, opts);
scheduleRows = localCollectScheduleRows(trimLinearizationDB, guide, opts);

if numel(scheduleRows) < 2
    error(['Need at least two replayed path points with reduced linear models ', ...
           'to build a scheduled controller.']);
end

nPts = numel(scheduleRows);
state_schedule = zeros(12, nPts);
trim_schedule = zeros(6, nPts);
gain_schedule = zeros(6, 9, nPts);

scheduleSummary = repmat(struct( ...
    'index', 0, ...
    'name', "", ...
    'group', "", ...
    'source_map_label', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'progress', NaN, ...
    'is_exact_trim', false, ...
    'max_state_residual', NaN, ...
    'design_status', "", ...
    'use_full_9state', false, ...
    'keep_idx', zeros(1, 9), ...
    'closed_loop_eigs', zeros(9, 1)), nPts, 1);

fprintf('\n=== Transition Path LQR Controller Build ===\n');
fprintf('DB source: %s\n', dbFile);
fprintf('Schedule points: %d\n', nPts);

for i = 1:nPts
    row = scheduleRows(i);
    design = localDesignDispatchLqr(row.A_9, row.B_9, opts);

    state_schedule(1:9, i) = row.x_trim;
    state_schedule(10, i) = row.tilt_deg;
    state_schedule(11, i) = row.vinf_mps;
    state_schedule(12, i) = row.progress;

    trim_schedule(:, i) = row.trim_cmd;
    gain_schedule(:, :, i) = design.K_dispatch;

    scheduleSummary(i).index = i;
    scheduleSummary(i).name = string(row.name);
    scheduleSummary(i).group = string(row.group);
    scheduleSummary(i).source_map_label = string(row.source_map_label);
    scheduleSummary(i).tilt_deg = row.tilt_deg;
    scheduleSummary(i).vinf_mps = row.vinf_mps;
    scheduleSummary(i).progress = row.progress;
    scheduleSummary(i).is_exact_trim = row.is_exact_trim;
    scheduleSummary(i).max_state_residual = row.max_state_residual;
    scheduleSummary(i).design_status = string(design.design_status);
    scheduleSummary(i).use_full_9state = design.use_full_9state;
    scheduleSummary(i).keep_idx(1:numel(design.keep_idx)) = design.keep_idx(:).';
    scheduleSummary(i).closed_loop_eigs(1:numel(design.closed_loop_eigs)) = design.closed_loop_eigs(:);

    fprintf('  [%2d/%2d] %-28s  tilt=%6.1f  V=%6.1f  exact=%d  mode=%s\n', ...
        i, nPts, row.name, row.tilt_deg, row.vinf_mps, row.is_exact_trim, design.design_status);
end

legacyIdx = nPts;
legacyX = state_schedule(1:9, legacyIdx);
legacyTrim = trim_schedule(:, legacyIdx);
legacyK = gain_schedule(:, :, legacyIdx);

controllerData = struct();
controllerData.name = sprintf('TransitionPathLQR_%dpt', nPts);
controllerData.type = 'transition_path_lqr';
controllerData.design_source = mfilename;
controllerData.db_source = dbFile;
controllerData.variant_ctrl_mode = opts.variant_ctrl_mode;
controllerData.controller_id = opts.controller_id;
controllerData.controller_state_ref = state_schedule;
controllerData.controller_trim_cmd = trim_schedule;
controllerData.controller_gain_lqr = gain_schedule;
controllerData.schedule_count = nPts;
controllerData.schedule_progress = state_schedule(12, :).';
controllerData.schedule_tilt_deg = state_schedule(10, :).';
controllerData.schedule_vinf_mps = state_schedule(11, :).';
controllerData.schedule_names = string({scheduleRows.name}).';
controllerData.schedule_groups = string({scheduleRows.group}).';
controllerData.schedule_table = struct2table(scheduleSummary);
controllerData.schedule_guide_polyline = guide;
controllerData.x_trim = state_schedule(1:9, :);
controllerData.trim_cmd = trim_schedule;
controllerData.K_dispatch = gain_schedule;

% Legacy fields retained so Run_EVTOL_Main and older code still have a
% sensible cruise-anchor view, even though the active controller is the
% MATLAB-function scheduled branch.
controllerData.K_lqr_cruise = legacyK([1 3 4 5 6], :);
controllerData.x_trim_lqr = legacyX;
controllerData.U_trim_lqr = [legacyTrim(1); legacyTrim(3:6)];

fprintf('Controller id = %d (scheduled path LQR)\n', controllerData.controller_id);
fprintf('Variant ctrlMode = %d (MATLAB Function dispatch)\n', controllerData.variant_ctrl_mode);
fprintf('Cruise anchor used for legacy compatibility: %s\n', scheduleRows(legacyIdx).name);
fprintf('=== Transition Path LQR Build Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'variant_ctrl_mode') || isempty(opts.variant_ctrl_mode)
    opts.variant_ctrl_mode = 2;
end
if ~isfield(opts, 'controller_id') || isempty(opts.controller_id)
    opts.controller_id = 4;
end
if ~isfield(opts, 'require_exact_trim') || isempty(opts.require_exact_trim)
    opts.require_exact_trim = false;
end
if ~isfield(opts, 'Q_full') || isempty(opts.Q_full)
    opts.Q_full = diag([8, 10, 1, 1, 1, 3, 6, 8, 6]);
end
if ~isfield(opts, 'Q_inner') || isempty(opts.Q_inner)
    opts.Q_inner = diag([8, 10, 1, 1, 3, 6, 8, 6]);
end
if ~isfield(opts, 'R') || isempty(opts.R)
    opts.R = diag([1e-4, 1e-4, 3, 3, 3, 3]);
end
if ~isfield(opts, 'guide_polyline')
    opts.guide_polyline = [];
end
end

function dbFile = localResolveDefaultDb()
stack = dbstack('-completenames');
if ~isempty(stack)
    rootDir = fileparts(stack(1).file);
else
    rootDir = pwd;
end

preferred = fullfile(rootDir, 'workspace_plots', 'trim_linearization_db_latest.mat');
if exist(preferred, 'file') == 2
    dbFile = preferred;
    return;
end

fallback = fullfile(rootDir, 'workspace_plots', 'trim_linearization_db.mat');
if exist(fallback, 'file') == 2
    dbFile = fallback;
    return;
end

dirInfo = dir(fullfile(rootDir, 'workspace_plots', 'trim_linearization_db*'));
dirInfo = dirInfo([dirInfo.isdir]);
bestDatenum = -inf;
dbFile = '';
for i = 1:numel(dirInfo)
    candidate = fullfile(dirInfo(i).folder, dirInfo(i).name, 'trim_linearization_db.mat');
    if exist(candidate, 'file') ~= 2
        continue;
    end
    if dirInfo(i).datenum > bestDatenum
        bestDatenum = dirInfo(i).datenum;
        dbFile = candidate;
    end
end

if isempty(dbFile)
    error(['Could not find a trim linearization DB. Run Build_Trim_Linearization_DB ', ...
           'first, or pass the DB file path explicitly.']);
end
end

function [trimLinearizationDB, dbFile] = localLoadDb(dbInput)
if isstruct(dbInput)
    trimLinearizationDB = dbInput;
    dbFile = '<struct input>';
    return;
end

dbFile = char(string(dbInput));
if exist(dbFile, 'file') ~= 2
    error('DB file not found: %s', dbFile);
end

data = load(dbFile);
if isfield(data, 'trimLinearizationDB')
    trimLinearizationDB = data.trimLinearizationDB;
elseif isfield(data, 'trimLinearizationDb')
    trimLinearizationDB = data.trimLinearizationDb;
else
    error('Could not find trimLinearizationDB in %s.', dbFile);
end
end

function guide = localResolveGuide(trimLinearizationDB, opts)
guide = [];
if isfield(opts, 'guide_polyline') && ~isempty(opts.guide_polyline)
    guide = opts.guide_polyline;
elseif isfield(trimLinearizationDB, 'meta') && isfield(trimLinearizationDB.meta, 'options') && ...
        isstruct(trimLinearizationDB.meta.options) && isfield(trimLinearizationDB.meta.options, 'guide_polyline')
    guide = trimLinearizationDB.meta.options.guide_polyline;
end

if isempty(guide)
    guide = [0.0, 0.0; 0.0, 2.5; 35.0, 20.0; 55.0, 25.0; 55.0, 55.0; 90.0, 75.0];
end
end

function scheduleRows = localCollectScheduleRows(trimLinearizationDB, guide, opts)
scheduleRows = repmat(localScheduleRowTemplate(), 0, 1);
groupNames = {'hover', 'cruise'};

for iGroup = 1:numel(groupNames)
    groupName = groupNames{iGroup};
    if ~isfield(trimLinearizationDB, groupName) || ~isfield(trimLinearizationDB.(groupName), 'entries')
        continue;
    end

    entries = trimLinearizationDB.(groupName).entries;
    for i = 1:numel(entries)
        entry = entries(i);
        if ~logical(localGetField(entry, 'replay_success', false))
            continue;
        end
        if ~isfield(entry, 'linear') || ~isstruct(entry.linear) || ...
                ~logical(localGetField(entry.linear, 'reduced_model_available', false))
            continue;
        end

        isExactTrim = logical(localGetField(entry.linear, 'replay_exact_trim', false));
        if opts.require_exact_trim && ~isExactTrim
            continue;
        end

        row = localScheduleRowTemplate();
        row.name = localGetField(entry, 'name', '');
        row.group = groupName;
        row.source_map_label = localGetField(entry, 'source_map_label', '');
        row.tilt_deg = localGetField(entry, 'target_tilt_deg', NaN);
        row.vinf_mps = localGetField(entry, 'target_vinf_mps', NaN);
        if ~isfinite(row.vinf_mps)
            row.vinf_mps = localResolveTrimSummaryField(entry, 'vinf_mps', NaN);
        end

        row.max_state_residual = localGetField(entry, 'max_state_residual', NaN);
        row.is_exact_trim = isExactTrim;
        row.A_9 = entry.linear.A_9;
        row.B_9 = entry.linear.B_9;
        row.x_trim = localBuildXTrim(entry);
        row.trim_cmd = localBuildTrimCmd(entry);
        [row.progress, row.dist_norm] = localProjectPointToGuide(row.tilt_deg, row.vinf_mps, guide);

        scheduleRows(end + 1, 1) = row; %#ok<SAGROW>
    end
end

if isempty(scheduleRows)
    return;
end

[~, order] = sort([scheduleRows.progress], 'ascend');
scheduleRows = scheduleRows(order);
scheduleRows = localEnforceMonotoneRows(scheduleRows);
end

function row = localScheduleRowTemplate()
row = struct( ...
    'name', '', ...
    'group', '', ...
    'source_map_label', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'progress', NaN, ...
    'dist_norm', inf, ...
    'max_state_residual', NaN, ...
    'is_exact_trim', false, ...
    'x_trim', zeros(9, 1), ...
    'trim_cmd', zeros(6, 1), ...
    'A_9', zeros(9, 9), ...
    'B_9', zeros(9, 6));
end

function x_trim = localBuildXTrim(entry)
theta_deg = localResolveTrimSummaryField(entry, 'theta_deg', 0.0);
u_mps = localResolveTrimSummaryField(entry, 'u_mps', 0.0);
w_mps = localResolveTrimSummaryField(entry, 'w_mps', 0.0);

x_trim = zeros(9, 1);
x_trim(2) = deg2rad(theta_deg);
x_trim(4) = u_mps;
x_trim(6) = w_mps;
end

function trim_cmd = localBuildTrimCmd(entry)
trim_cmd = zeros(6, 1);
trim_cmd(1) = localResolveTrimSummaryField(entry, 'front_collective_rpm', 0.0);
trim_cmd(2) = localResolveTrimSummaryField(entry, 'rear_collective_rpm', 0.0);
trim_cmd(3) = deg2rad(localResolveTrimSummaryField(entry, 'delta_f_deg', 0.0));
trim_cmd(4) = deg2rad(localResolveTrimSummaryField(entry, 'delta_a_deg', 0.0));
trim_cmd(5) = deg2rad(localResolveTrimSummaryField(entry, 'delta_e_deg', 0.0));
trim_cmd(6) = deg2rad(localResolveTrimSummaryField(entry, 'delta_r_deg', 0.0));
end

function value = localResolveTrimSummaryField(entry, fieldName, fallback)
value = fallback;
if isfield(entry, 'trim_summary') && isstruct(entry.trim_summary) && isfield(entry.trim_summary, fieldName)
    value = entry.trim_summary.(fieldName);
end
end

function rows = localEnforceMonotoneRows(rows)
if numel(rows) <= 1
    return;
end

keepMask = false(numel(rows), 1);
lastTilt = -inf;
lastVinf = -inf;
for i = 1:numel(rows)
    if rows(i).tilt_deg >= lastTilt - 1e-9 && rows(i).vinf_mps >= lastVinf - 1e-9
        keepMask(i) = true;
        lastTilt = rows(i).tilt_deg;
        lastVinf = rows(i).vinf_mps;
    end
end
rows = rows(keepMask);
end

function [progress, distNorm] = localProjectPointToGuide(tiltDeg, vinfMps, guide)
if size(guide, 1) < 2
    progress = 0.0;
    distNorm = inf;
    return;
end

totalLength = localGuideTotalLength(guide);
point = [tiltDeg, vinfMps];
bestDist = inf;
bestArc = 0.0;
arcSoFar = 0.0;

for i = 1:(size(guide, 1) - 1)
    p0 = guide(i, :);
    p1 = guide(i + 1, :);
    seg = p1 - p0;
    segLen = norm(seg);
    if segLen <= eps
        continue;
    end

    tau = dot(point - p0, seg) / max(segLen^2, eps);
    tau = min(max(tau, 0.0), 1.0);
    proj = p0 + tau * seg;
    d = norm(point - proj);
    if d < bestDist
        bestDist = d;
        bestArc = arcSoFar + tau * segLen;
    end
    arcSoFar = arcSoFar + segLen;
end

progress = bestArc / max(totalLength, eps);
distNorm = bestDist / 20.0;
end

function lengthValue = localGuideTotalLength(guide)
lengthValue = 0.0;
for i = 1:(size(guide, 1) - 1)
    lengthValue = lengthValue + norm(guide(i + 1, :) - guide(i, :));
end
end

function design = localDesignDispatchLqr(A_lqr, B_lqr, opts)
design = struct();
design.keep_idx = 1:9;
design.use_full_9state = false;
design.closed_loop_eigs = zeros(9, 1);
design.K_dispatch = zeros(6, 9);
design.design_status = 'trim_hold_only';

ctrb_rank_9 = rank(ctrb(A_lqr, B_lqr));
if ctrb_rank_9 == 9
    keep_idx = 1:9;
    Q_use = opts.Q_full;
    use_full_9state = true;
    design.design_status = 'full_9state_lqr';
else
    keep_idx = [1 2 4 5 6 7 8 9];
    A_test = A_lqr(keep_idx, keep_idx);
    B_test = B_lqr(keep_idx, :);
    ctrb_rank_8 = rank(ctrb(A_test, B_test));
    if ctrb_rank_8 < 8
        design.keep_idx = [];
        design.use_full_9state = false;
        design.closed_loop_eigs = eig(A_lqr);
        design.K_dispatch = zeros(6, 9);
        design.design_status = 'trim_hold_only';
        return;
    end
    Q_use = opts.Q_inner;
    use_full_9state = false;
    design.design_status = 'inner_8state_lqr';
end

R_use = opts.R;
try
    K_use = lqr(A_lqr(keep_idx, keep_idx), B_lqr(keep_idx, :), Q_use, R_use);
catch
    design.keep_idx = keep_idx;
    design.use_full_9state = use_full_9state;
    design.closed_loop_eigs = eig(A_lqr(keep_idx, keep_idx));
    design.K_dispatch = zeros(6, 9);
    design.design_status = 'trim_hold_only';
    return;
end

K_dispatch = zeros(6, 9);
K_dispatch(:, keep_idx) = K_use;

design.keep_idx = keep_idx;
design.use_full_9state = use_full_9state;
design.closed_loop_eigs = eig(A_lqr(keep_idx, keep_idx) - B_lqr(keep_idx, :) * K_use);
design.K_dispatch = K_dispatch;
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end
