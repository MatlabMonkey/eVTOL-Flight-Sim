function [pathSpec, pathPoints, pathTable, debugInfo] = select_corridor_path_from_db(controllerDbInput, opts)
%SELECT_TRANSITION_CORRIDOR_PATH_FROM_CONTROLLER_DB
% Pick one smooth scheduled transition path from controller_schedule.
%
% Usage:
%   [pathSpec, pathPoints, pathTable] = select_corridor_path_from_db()
%   [pathSpec, pathPoints, pathTable] = select_corridor_path_from_db(dbFile)
%   [pathSpec, pathPoints, pathTable] = select_corridor_path_from_db(dbFile, opts)
%
% This helper is meant for the current "trim corridor" workflow:
%   1) define a hand-picked guide corridor in (Vinf, front tilt)
%   2) find controller-ready DB points near that guide
%   3) choose one smooth ordered branch through those candidates
%   4) use the selected path for scheduled trim/LQR interpolation
%
% Inputs:
%   controllerDbInput  - optional path to controller_schedule.mat, or the
%                        controllerScheduleDB struct itself.
%
% Optional opts fields:
%   guide_vinf_mps              = waypoint x-values
%   guide_tilt_deg              = waypoint y-values
%   guide_polyline_vinf_tilt    = Nx2 [vinf_mps, tilt_deg] override
%   allowed_classifications     = string array (default exact+quasi)
%   require_rear_on             = true by default
%   min_rear_collective_rpm     = 1.0 by default
%   waypoint_search_vinf_mps    = 7.5 by default
%   waypoint_search_tilt_deg    = 7.5 by default
%   max_candidates_per_waypoint = 12 by default
%   monotonic_vinf              = true by default
%   monotonic_tilt              = true by default
%   waypoint_weight_vinf        = 1.0
%   waypoint_weight_tilt        = 1.0
%   smooth_weight_front_rpm     = 0.20
%   smooth_weight_rear_rpm      = 0.20
%   smooth_weight_delta_f_deg   = 1.00
%   smooth_weight_delta_e_deg   = 1.00
%   smooth_weight_theta_deg     = 0.50
%   point_weight_abs_delta_f_deg = 0.35
%   point_weight_abs_delta_e_deg = 0.35
%   point_weight_abs_theta_deg   = 0.75
%   max_abs_delta_f_deg         = inf by default
%   max_abs_delta_e_deg         = inf by default
%   max_abs_theta_deg           = inf by default
%   max_delta_vinf_per_step     = inf by default
%   max_delta_tilt_per_step     = inf by default
%   rear_floor_min_rpm          = -inf by default (disabled)
%   rear_floor_max_vinf_mps     = inf by default
%   rear_floor_max_tilt_deg     = inf by default
%   edge_weight_rear_drop_rpm   = 0 by default
%   max_rear_drop_rpm_per_step  = inf by default
%   plot_result                 = true by default
%   show_popup                  = true by default
%   save_plot                   = false by default
%
% Outputs:
%   pathSpec    - path description and selected names/keys
%   pathPoints  - controller DB point structs for the selected path
%   pathTable   - summary table of the selected path
%   debugInfo   - candidate and DP diagnostics

if nargin < 1 || isempty(controllerDbInput)
    controllerDbInput = localDefaultControllerDbFile();
end
if nargin < 2 || isempty(opts)
    opts = struct();
end

opts = localApplyDefaults(opts);
[controllerDb, controllerDbFile] = localLoadControllerDb(controllerDbInput);
[guideV, guideT] = localResolveGuide(opts);

[summaryTbl, pointStructs] = localPrepareCandidates(controllerDb, opts);
[candidateSets, candidateMeta] = localBuildCandidateSets(summaryTbl, guideV, guideT, opts);
[bestRowIndices, dpInfo] = localSolvePath(candidateSets, guideV, guideT, opts);

if isempty(bestRowIndices)
    error(['Could not build a transition path from the controller DB. ', ...
           'Widen the waypoint search window or relax monotonic constraints.']);
end

pathPoints = pointStructs(bestRowIndices);
pathPoints = localAnnotatePathPoints(pathPoints, guideV, guideT);
pathTable = localBuildPathTable(pathPoints);
pathSpec = localBuildPathSpec(controllerDbFile, guideV, guideT, pathPoints, opts);
debugInfo = struct();
debugInfo.candidate_meta = candidateMeta;
debugInfo.dp = dpInfo;
debugInfo.controller_db_file = string(controllerDbFile);

assignin('base', 'selectedTransitionPathSpec', pathSpec);
assignin('base', 'selectedTransitionPathPoints', pathPoints);
assignin('base', 'selectedTransitionPathTable', pathTable);
assignin('base', 'selectedTransitionPathDebug', debugInfo);

if opts.plot_result
    localPlotSelection(controllerDb, summaryTbl, pathTable, guideV, guideT, opts);
end
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'guide_polyline_vinf_tilt')
    opts.guide_polyline_vinf_tilt = [];
end
if ~isfield(opts, 'guide_vinf_mps') || isempty(opts.guide_vinf_mps)
    opts.guide_vinf_mps = [0 2.5 5 10 15 20 25 30 40 50 60 70];
end
if ~isfield(opts, 'guide_tilt_deg') || isempty(opts.guide_tilt_deg)
    opts.guide_tilt_deg = [0 10 22 45 55 62 68 72 78 83 87 90];
end
if ~isfield(opts, 'allowed_classifications') || isempty(opts.allowed_classifications)
    opts.allowed_classifications = ["exact_trim", "quasi_trim_usable"];
end
if ~isfield(opts, 'require_rear_on') || isempty(opts.require_rear_on)
    opts.require_rear_on = true;
end
if ~isfield(opts, 'min_rear_collective_rpm') || isempty(opts.min_rear_collective_rpm)
    opts.min_rear_collective_rpm = 1.0;
end
if ~isfield(opts, 'waypoint_search_vinf_mps') || isempty(opts.waypoint_search_vinf_mps)
    opts.waypoint_search_vinf_mps = 7.5;
end
if ~isfield(opts, 'waypoint_search_tilt_deg') || isempty(opts.waypoint_search_tilt_deg)
    opts.waypoint_search_tilt_deg = 7.5;
end
if ~isfield(opts, 'max_candidates_per_waypoint') || isempty(opts.max_candidates_per_waypoint)
    opts.max_candidates_per_waypoint = 12;
end
if ~isfield(opts, 'monotonic_vinf') || isempty(opts.monotonic_vinf)
    opts.monotonic_vinf = true;
end
if ~isfield(opts, 'monotonic_tilt') || isempty(opts.monotonic_tilt)
    opts.monotonic_tilt = true;
end
if ~isfield(opts, 'waypoint_weight_vinf') || isempty(opts.waypoint_weight_vinf)
    opts.waypoint_weight_vinf = 1.0;
end
if ~isfield(opts, 'waypoint_weight_tilt') || isempty(opts.waypoint_weight_tilt)
    opts.waypoint_weight_tilt = 1.0;
end
if ~isfield(opts, 'smooth_weight_front_rpm') || isempty(opts.smooth_weight_front_rpm)
    opts.smooth_weight_front_rpm = 0.20;
end
if ~isfield(opts, 'smooth_weight_rear_rpm') || isempty(opts.smooth_weight_rear_rpm)
    opts.smooth_weight_rear_rpm = 0.20;
end
if ~isfield(opts, 'smooth_weight_delta_f_deg') || isempty(opts.smooth_weight_delta_f_deg)
    opts.smooth_weight_delta_f_deg = 1.00;
end
if ~isfield(opts, 'smooth_weight_delta_e_deg') || isempty(opts.smooth_weight_delta_e_deg)
    opts.smooth_weight_delta_e_deg = 1.00;
end
if ~isfield(opts, 'smooth_weight_theta_deg') || isempty(opts.smooth_weight_theta_deg)
    opts.smooth_weight_theta_deg = 0.50;
end
if ~isfield(opts, 'point_weight_abs_delta_f_deg') || isempty(opts.point_weight_abs_delta_f_deg)
    opts.point_weight_abs_delta_f_deg = 0.35;
end
if ~isfield(opts, 'point_weight_abs_delta_e_deg') || isempty(opts.point_weight_abs_delta_e_deg)
    opts.point_weight_abs_delta_e_deg = 0.35;
end
if ~isfield(opts, 'point_weight_abs_theta_deg') || isempty(opts.point_weight_abs_theta_deg)
    opts.point_weight_abs_theta_deg = 0.75;
end
if ~isfield(opts, 'max_abs_delta_f_deg') || isempty(opts.max_abs_delta_f_deg)
    opts.max_abs_delta_f_deg = inf;
end
if ~isfield(opts, 'max_abs_delta_e_deg') || isempty(opts.max_abs_delta_e_deg)
    opts.max_abs_delta_e_deg = inf;
end
if ~isfield(opts, 'max_abs_theta_deg') || isempty(opts.max_abs_theta_deg)
    opts.max_abs_theta_deg = inf;
end
if ~isfield(opts, 'max_delta_vinf_per_step') || isempty(opts.max_delta_vinf_per_step)
    opts.max_delta_vinf_per_step = inf;
end
if ~isfield(opts, 'max_delta_tilt_per_step') || isempty(opts.max_delta_tilt_per_step)
    opts.max_delta_tilt_per_step = inf;
end
if ~isfield(opts, 'rear_floor_min_rpm') || isempty(opts.rear_floor_min_rpm)
    opts.rear_floor_min_rpm = -inf;
end
if ~isfield(opts, 'rear_floor_max_vinf_mps') || isempty(opts.rear_floor_max_vinf_mps)
    opts.rear_floor_max_vinf_mps = inf;
end
if ~isfield(opts, 'rear_floor_max_tilt_deg') || isempty(opts.rear_floor_max_tilt_deg)
    opts.rear_floor_max_tilt_deg = inf;
end
if ~isfield(opts, 'edge_weight_rear_drop_rpm') || isempty(opts.edge_weight_rear_drop_rpm)
    opts.edge_weight_rear_drop_rpm = 0.0;
end
if ~isfield(opts, 'max_rear_drop_rpm_per_step') || isempty(opts.max_rear_drop_rpm_per_step)
    opts.max_rear_drop_rpm_per_step = inf;
end
if ~isfield(opts, 'plot_result') || isempty(opts.plot_result)
    opts.plot_result = true;
end
if ~isfield(opts, 'show_popup') || isempty(opts.show_popup)
    opts.show_popup = true;
end
if ~isfield(opts, 'save_plot') || isempty(opts.save_plot)
    opts.save_plot = false;
end
end

function dbFile = localDefaultControllerDbFile()
stack = dbstack('-completenames');
if ~isempty(stack)
    rootDir = fileparts(fileparts(stack(1).file));
else
    rootDir = pwd;
end
dbPaths = TrimDB_Paths(rootDir);
dbFile = dbPaths.controller_mat_file;
end

function [controllerDb, dbFile] = localLoadControllerDb(controllerDbInput)
if isstruct(controllerDbInput)
    controllerDb = controllerDbInput;
    dbFile = '<struct input>';
    return;
end

dbFile = char(string(controllerDbInput));
if exist(dbFile, 'file') ~= 2
    error('Controller DB file not found: %s', dbFile);
end

data = load(dbFile, 'controllerScheduleDB');
if ~isfield(data, 'controllerScheduleDB') || ~isstruct(data.controllerScheduleDB)
    error('File does not contain controllerScheduleDB: %s', dbFile);
end

controllerDb = data.controllerScheduleDB;
if ~isfield(controllerDb, 'summary_table') || isempty(controllerDb.summary_table) || ...
        ~isfield(controllerDb, 'points') || isempty(controllerDb.points)
    error('controllerScheduleDB is missing summary_table and/or points.');
end
end

function [guideV, guideT] = localResolveGuide(opts)
if ~isempty(opts.guide_polyline_vinf_tilt)
    guide = opts.guide_polyline_vinf_tilt;
    if size(guide, 2) ~= 2
        error('opts.guide_polyline_vinf_tilt must be Nx2 [vinf_mps, tilt_deg].');
    end
    guideV = guide(:, 1);
    guideT = guide(:, 2);
else
    guideV = opts.guide_vinf_mps(:);
    guideT = opts.guide_tilt_deg(:);
end

if numel(guideV) ~= numel(guideT) || numel(guideV) < 2
    error('Guide must have at least two waypoints and matched vinf/tilt lengths.');
end
end

function [summaryTbl, pointStructs] = localPrepareCandidates(controllerDb, opts)
summaryTbl = controllerDb.summary_table;
pointStructs = controllerDb.points(:);

if height(summaryTbl) ~= numel(pointStructs)
    error('controllerScheduleDB.summary_table and .points size mismatch.');
end

summaryTbl.row_index = (1:height(summaryTbl)).';

classMask = false(height(summaryTbl), 1);
allowed = string(opts.allowed_classifications(:));
for i = 1:numel(allowed)
    classMask = classMask | strcmp(string(summaryTbl.classification), allowed(i));
end
summaryTbl = summaryTbl(classMask, :);

if opts.require_rear_on
    summaryTbl = summaryTbl(summaryTbl.rear_collective_rpm > opts.min_rear_collective_rpm, :);
end

absMask = abs(summaryTbl.delta_f_deg) <= opts.max_abs_delta_f_deg & ...
    abs(summaryTbl.delta_e_deg) <= opts.max_abs_delta_e_deg & ...
    abs(summaryTbl.theta_deg) <= opts.max_abs_theta_deg;
summaryTbl = summaryTbl(absMask, :);

if isfinite(opts.rear_floor_min_rpm)
    rearFloorMask = ~(summaryTbl.vinf_mps < opts.rear_floor_max_vinf_mps & ...
        summaryTbl.tilt_deg < opts.rear_floor_max_tilt_deg & ...
        summaryTbl.rear_collective_rpm < opts.rear_floor_min_rpm);
    summaryTbl = summaryTbl(rearFloorMask, :);
end

if isempty(summaryTbl)
    error('No controller DB candidates survived the classification/rear-on filters.');
end
end

function [candidateSets, candidateMeta] = localBuildCandidateSets(summaryTbl, guideV, guideT, opts)
nWaypoints = numel(guideV);
candidateSets = cell(nWaypoints, 1);
candidateMeta = repmat(struct( ...
    'waypoint_index', 0, ...
    'guide_vinf_mps', NaN, ...
    'guide_tilt_deg', NaN, ...
    'candidate_count', 0, ...
    'search_vinf_mps', NaN, ...
    'search_tilt_deg', NaN), nWaypoints, 1);

for i = 1:nWaypoints
    dv = summaryTbl.vinf_mps - guideV(i);
    dt = summaryTbl.tilt_deg - guideT(i);
    inWindow = abs(dv) <= opts.waypoint_search_vinf_mps & ...
        abs(dt) <= opts.waypoint_search_tilt_deg;

    localTbl = summaryTbl(inWindow, :);
    if isempty(localTbl)
        [~, order] = sort(localWaypointDistance(summaryTbl, guideV(i), guideT(i), opts), 'ascend');
        take = min(opts.max_candidates_per_waypoint, numel(order));
        localTbl = summaryTbl(order(1:take), :);
    else
        dist = localWaypointDistance(localTbl, guideV(i), guideT(i), opts);
        localTbl.waypoint_cost = dist;
        localTbl = sortrows(localTbl, {'waypoint_cost', 'tilt_deg', 'vinf_mps'});
        take = min(opts.max_candidates_per_waypoint, height(localTbl));
        localTbl = localTbl(1:take, :);
    end

    candidateSets{i} = localTbl;
    candidateMeta(i).waypoint_index = i;
    candidateMeta(i).guide_vinf_mps = guideV(i);
    candidateMeta(i).guide_tilt_deg = guideT(i);
    candidateMeta(i).candidate_count = height(localTbl);
    candidateMeta(i).search_vinf_mps = opts.waypoint_search_vinf_mps;
    candidateMeta(i).search_tilt_deg = opts.waypoint_search_tilt_deg;
end
end

function waypointCost = localWaypointDistance(tbl, guideV, guideT, opts)
dv = (tbl.vinf_mps - guideV) ./ max(opts.waypoint_search_vinf_mps, eps);
dt = (tbl.tilt_deg - guideT) ./ max(opts.waypoint_search_tilt_deg, eps);
waypointCost = opts.waypoint_weight_vinf * dv.^2 + opts.waypoint_weight_tilt * dt.^2;
waypointCost = waypointCost + localPointPenalty(tbl, opts);
end

function pointCost = localPointPenalty(tbl, opts)
dFabs = abs(tbl.delta_f_deg) / 10.0;
dEabs = abs(tbl.delta_e_deg) / 10.0;
dThetaAbs = abs(tbl.theta_deg) / 10.0;

pointCost = opts.point_weight_abs_delta_f_deg * dFabs.^2 + ...
    opts.point_weight_abs_delta_e_deg * dEabs.^2 + ...
    opts.point_weight_abs_theta_deg * dThetaAbs.^2;
end

function [bestRowIndices, dpInfo] = localSolvePath(candidateSets, guideV, guideT, opts)
nWaypoints = numel(candidateSets);
dpCosts = cell(nWaypoints, 1);
dpPrev = cell(nWaypoints, 1);

firstTbl = candidateSets{1};
dpCosts{1} = localWaypointDistance(firstTbl, guideV(1), guideT(1), opts);
dpPrev{1} = zeros(height(firstTbl), 1);

for i = 2:nWaypoints
    prevTbl = candidateSets{i - 1};
    currTbl = candidateSets{i};
    prevCosts = dpCosts{i - 1};

    currCosts = inf(height(currTbl), 1);
    currPrev = zeros(height(currTbl), 1);
    waypointCost = localWaypointDistance(currTbl, guideV(i), guideT(i), opts);

    for j = 1:height(currTbl)
        for k = 1:height(prevTbl)
            edgeCost = localTransitionCost(prevTbl(k, :), currTbl(j, :), opts);
            totalCost = prevCosts(k) + waypointCost(j) + edgeCost;
            if totalCost < currCosts(j)
                currCosts(j) = totalCost;
                currPrev(j) = k;
            end
        end
    end

    finiteMask = isfinite(currCosts);
    if ~any(finiteMask)
        error(['No feasible path transition between waypoint %d and %d. ', ...
               'Relax monotonic constraints or widen candidate windows.'], i - 1, i);
    end

    dpCosts{i} = currCosts;
    dpPrev{i} = currPrev;
end

[~, bestEndIdx] = min(dpCosts{end});
selectedLocalIdx = zeros(nWaypoints, 1);
selectedLocalIdx(end) = bestEndIdx;

for i = nWaypoints:-1:2
    selectedLocalIdx(i - 1) = dpPrev{i}(selectedLocalIdx(i));
end

bestRowIndices = zeros(nWaypoints, 1);
for i = 1:nWaypoints
    localTbl = candidateSets{i};
    bestRowIndices(i) = localTbl.row_index(selectedLocalIdx(i));
end

dpInfo = struct();
dpInfo.total_cost = dpCosts{end}(bestEndIdx);
dpInfo.selected_local_indices = selectedLocalIdx;
dpInfo.candidate_counts = cellfun(@height, candidateSets);
end

function edgeCost = localTransitionCost(prevRow, currRow, opts)
if ismember('row_index', prevRow.Properties.VariableNames) && ...
        ismember('row_index', currRow.Properties.VariableNames) && ...
        prevRow.row_index == currRow.row_index
    edgeCost = inf;
    return;
end

if opts.monotonic_vinf && currRow.vinf_mps < prevRow.vinf_mps
    edgeCost = inf;
    return;
end
if opts.monotonic_tilt && currRow.tilt_deg < prevRow.tilt_deg
    edgeCost = inf;
    return;
end
if abs(currRow.vinf_mps - prevRow.vinf_mps) > opts.max_delta_vinf_per_step
    edgeCost = inf;
    return;
end
if abs(currRow.tilt_deg - prevRow.tilt_deg) > opts.max_delta_tilt_per_step
    edgeCost = inf;
    return;
end

rearDropRpm = max(prevRow.rear_collective_rpm - currRow.rear_collective_rpm, 0.0);
if rearDropRpm > opts.max_rear_drop_rpm_per_step
    edgeCost = inf;
    return;
end

dFront = (currRow.front_collective_rpm - prevRow.front_collective_rpm) / 400.0;
dRear = (currRow.rear_collective_rpm - prevRow.rear_collective_rpm) / 400.0;
dF = (currRow.delta_f_deg - prevRow.delta_f_deg) / 5.0;
dE = (currRow.delta_e_deg - prevRow.delta_e_deg) / 5.0;
dTheta = (currRow.theta_deg - prevRow.theta_deg) / 5.0;

edgeCost = opts.smooth_weight_front_rpm * dFront.^2 + ...
    opts.smooth_weight_rear_rpm * dRear.^2 + ...
    opts.smooth_weight_delta_f_deg * dF.^2 + ...
    opts.smooth_weight_delta_e_deg * dE.^2 + ...
    opts.smooth_weight_theta_deg * dTheta.^2;

if opts.edge_weight_rear_drop_rpm > 0.0 && rearDropRpm > 0.0
    progressMag = abs(currRow.vinf_mps - prevRow.vinf_mps) + 0.25 * abs(currRow.tilt_deg - prevRow.tilt_deg);
    progressMag = max(progressMag, 5.0);
    rearDropNorm = rearDropRpm / 400.0;
    edgeCost = edgeCost + opts.edge_weight_rear_drop_rpm * rearDropNorm.^2 * (10.0 / progressMag);
end
end

function pathPoints = localAnnotatePathPoints(pathPoints, guideV, guideT)
nPts = numel(pathPoints);
progress = zeros(nPts, 1);
for i = 2:nPts
    dv = pathPoints(i).vinf_mps - pathPoints(i - 1).vinf_mps;
    dt = pathPoints(i).tilt_deg - pathPoints(i - 1).tilt_deg;
    progress(i) = progress(i - 1) + hypot(dv, dt);
end
if progress(end) > 0
    progress = progress / progress(end);
end

for i = 1:nPts
    pathPoints(i).path_index = i;
    pathPoints(i).path_progress = progress(i);
    pathPoints(i).guide_vinf_mps = guideV(i);
    pathPoints(i).guide_tilt_deg = guideT(i);
end
end

function pathTable = localBuildPathTable(pathPoints)
nPts = numel(pathPoints);
rows = repmat(struct( ...
    'path_index', 0, ...
    'path_progress', NaN, ...
    'guide_vinf_mps', NaN, ...
    'guide_tilt_deg', NaN, ...
    'selected_vinf_mps', NaN, ...
    'selected_tilt_deg', NaN, ...
    'name', "", ...
    'key', "", ...
    'family', "", ...
    'classification', "", ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'theta_deg', NaN), nPts, 1);

for i = 1:nPts
    p = pathPoints(i);
    rows(i).path_index = i;
    rows(i).path_progress = localGetField(p, 'path_progress', NaN);
    rows(i).guide_vinf_mps = localGetField(p, 'guide_vinf_mps', NaN);
    rows(i).guide_tilt_deg = localGetField(p, 'guide_tilt_deg', NaN);
    rows(i).selected_vinf_mps = localGetField(p, 'vinf_mps', NaN);
    rows(i).selected_tilt_deg = localGetField(p, 'tilt_deg', NaN);
    rows(i).name = string(localGetField(p, 'name', ""));
    rows(i).key = string(localGetField(p, 'key', ""));
    rows(i).family = string(localGetField(p, 'family', ""));
    rows(i).classification = string(localGetField(p, 'classification', ""));
    rows(i).front_collective_rpm = localGetField(p, 'front_collective_rpm', NaN);
    rows(i).rear_collective_rpm = localGetField(p, 'rear_collective_rpm', NaN);
    rows(i).delta_f_deg = localGetField(p, 'delta_f_deg', NaN);
    rows(i).delta_e_deg = localGetField(p, 'delta_e_deg', NaN);
    rows(i).theta_deg = localGetField(p, 'theta_deg', NaN);
end

pathTable = struct2table(rows);
end

function pathSpec = localBuildPathSpec(controllerDbFile, guideV, guideT, pathPoints, opts)
pathSpec = struct();
pathSpec.name = 'AutoSelectedTransitionCorridorPath';
pathSpec.description = ['Auto-selected ordered corridor path from ', ...
    'controller_schedule using guide proximity plus actuator/state smoothness.'];
pathSpec.controller_db_file = string(controllerDbFile);
pathSpec.guide_vinf_mps = guideV(:);
pathSpec.guide_tilt_deg = guideT(:);
pathSpec.selected_point_names = string({pathPoints.name}).';
pathSpec.selected_point_keys = string({pathPoints.key}).';
pathSpec.selected_vinf_mps = [pathPoints.vinf_mps].';
pathSpec.selected_tilt_deg = [pathPoints.tilt_deg].';
pathSpec.options = opts;
end

function localPlotSelection(controllerDb, summaryTbl, pathTable, guideV, guideT, opts)
figVisible = 'on';
if ~opts.show_popup
    figVisible = 'off';
end

f = figure('Color', 'w', 'Position', [120 120 1180 840], 'Visible', figVisible);
hold on;
grid on;
box on;

class = string(summaryTbl.classification);
isExact = class == "exact_trim";
isQuasi = class == "quasi_trim_usable";

if any(isExact)
    scatter(summaryTbl.vinf_mps(isExact), summaryTbl.tilt_deg(isExact), 28, ...
        [0.10 0.65 0.20], 'filled', 'DisplayName', 'Exact trim');
end
if any(isQuasi)
    scatter(summaryTbl.vinf_mps(isQuasi), summaryTbl.tilt_deg(isQuasi), 32, ...
        [0.90 0.70 0.10], 's', 'filled', 'DisplayName', 'Quasi trim');
end

plot(guideV, guideT, '-', 'Color', [0.12 0.38 0.92], 'LineWidth', 2.5, ...
    'DisplayName', 'Guide corridor');
plot(pathTable.selected_vinf_mps, pathTable.selected_tilt_deg, 'o-', ...
    'Color', [0.90 0.10 0.10], 'LineWidth', 2.0, 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.90 0.10 0.10], 'DisplayName', 'Selected path');

if ~isempty(pathTable)
    scatter(pathTable.selected_vinf_mps(1), pathTable.selected_tilt_deg(1), 80, ...
        [0 0.35 0.9], 'd', 'filled', 'DisplayName', 'Path start');
    scatter(pathTable.selected_vinf_mps(end), pathTable.selected_tilt_deg(end), 90, ...
        [0 0 0], 'p', 'filled', 'DisplayName', 'Path end');
end

xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Auto-Selected Transition Corridor Path From Controller DB');
legend('Location', 'northwest');

if opts.save_plot
    outDir = localResolveOutputDir(controllerDb);
    if exist(outDir, 'dir') ~= 7
        mkdir(outDir);
    end
    outFile = fullfile(outDir, 'selected_transition_corridor_path.png');
    exportgraphics(f, outFile, 'Resolution', 300);
    assignin('base', 'selectedTransitionPathPlotFile', outFile);
end

if ~opts.show_popup
    close(f);
end
end

function outDir = localResolveOutputDir(controllerDb)
outDir = '';
if isfield(controllerDb, 'meta') && isstruct(controllerDb.meta) && ...
        isfield(controllerDb.meta, 'workspace_plots_dir') && ~isempty(controllerDb.meta.workspace_plots_dir)
    outDir = fullfile(char(controllerDb.meta.workspace_plots_dir), 'selected_transition_corridor_path');
end
if isempty(outDir)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        rootDir = fileparts(stack(1).file);
    else
        rootDir = pwd;
    end
    outDir = fullfile(rootDir, 'workspace_plots', 'selected_transition_corridor_path');
end
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end
