% Build_Trim_Linearization_DB.m
% Build saved linearization databases from the current trim maps.
%
% This script replays exact trim-map points through trim_evtol_case and
% stores compact reduced-order linear models. The replayed points are split
% into two groups:
%   - cruise-side points
%   - hover-side points
%
% Default source maps:
%   - workspace_plots/transition_trim_map_latest.mat
%   - workspace_plots/transition_trim_map_low_speed_latest.mat
%   - workspace_plots/transition_trim_path_scored_latest.mat
%
% Expected usage:
%   Build_Trim_Linearization_DB
%
% Optional configuration:
%   trimLinearizationDbOptions = struct( ...
%       'include_full_order', false, ...
%       'selection_mode', 'guided_path', ...
%       'target_point_count', 20, ...
%       'cruise_limit', inf, ...
%       'hover_limit', inf, ...
%       'write_latest', true);
%   Build_Trim_Linearization_DB
%
% Outputs left in the base workspace:
%   - trimLinearizationDB
%   - trimLinearizationDbSummary
%   - trimLinearizationDbOutputDir

if ~exist('trimLinearizationDbOptions', 'var') || ~isstruct(trimLinearizationDbOptions)
    trimLinearizationDbOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
trimLinearizationDbOptions = localApplyDefaults(trimLinearizationDbOptions, root_dir, timestamp);

initOptions = struct();
initOptions.trimLinearizationDbOptions = trimLinearizationDbOptions;
Init_EVTOL_Main
if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'trimLinearizationDbOptions')
    trimLinearizationDbOptions = initOptions.trimLinearizationDbOptions;
end

sourceEntries = localLoadSourceEntries(trimLinearizationDbOptions.map_files, trimLinearizationDbOptions);
[cruiseSelections, hoverSelections, selectionSummary] = localPartitionSelections(sourceEntries, trimLinearizationDbOptions);

trimLinearizationDbOutputDir = trimLinearizationDbOptions.output_dir;
if exist(trimLinearizationDbOutputDir, 'dir') ~= 7
    mkdir(trimLinearizationDbOutputDir);
end

fprintf('=== Build_Trim_Linearization_DB ===\n');
fprintf('Output dir: %s\n', trimLinearizationDbOutputDir);
fprintf('Selected path points: cruise=%d | hover=%d\n', numel(cruiseSelections), numel(hoverSelections));

trimLinearizationDB = struct();
trimLinearizationDB.meta = struct();
trimLinearizationDB.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
trimLinearizationDB.meta.root_dir = root_dir;
trimLinearizationDB.meta.trim_model = initData.modelNames.trim;
trimLinearizationDB.meta.run_model = initData.modelNames.run;
trimLinearizationDB.meta.options = trimLinearizationDbOptions;
trimLinearizationDB.meta.source_maps = trimLinearizationDbOptions.map_files(:);
trimLinearizationDB.selection_summary = selectionSummary;
trimLinearizationDB.cruise = localEmptyGroupDb('cruise');
trimLinearizationDB.hover = localEmptyGroupDb('hover');

groupOrder = {'cruise', 'hover'};
for iGroup = 1:numel(groupOrder)
    groupName = groupOrder{iGroup};
    switch groupName
        case 'cruise'
            selections = cruiseSelections;
            limitValue = trimLinearizationDbOptions.cruise_limit;
        case 'hover'
            selections = hoverSelections;
            limitValue = trimLinearizationDbOptions.hover_limit;
        otherwise
            error('Unknown group "%s".', groupName);
    end

    if isfinite(limitValue)
        limitCount = max(0, round(limitValue));
        selections = selections(1:min(numel(selections), limitCount));
    end

    fprintf('\nGroup "%s": replaying %d point(s).\n', groupName, numel(selections));
    groupDb = localReplaySelections(groupName, selections, initData, trimLinearizationDbOptions);
    trimLinearizationDB.(groupName) = groupDb;

    localSaveGroupDb(groupDb, trimLinearizationDbOutputDir, root_dir, trimLinearizationDbOptions.write_latest);
end

trimLinearizationDbSummary = localBuildCombinedSummary(trimLinearizationDB);
trimLinearizationDB.summary_table = trimLinearizationDbSummary;

localSaveCombinedDb(trimLinearizationDB, trimLinearizationDbSummary, trimLinearizationDbOutputDir, root_dir, trimLinearizationDbOptions.write_latest);

fprintf('\nCompleted trim linearization DB build.\n');
fprintf('  cruise replay successes = %d / %d\n', nnz(trimLinearizationDB.cruise.summary_table.replay_success), height(trimLinearizationDB.cruise.summary_table));
fprintf('  hover replay successes = %d / %d\n', nnz(trimLinearizationDB.hover.summary_table.replay_success), height(trimLinearizationDB.hover.summary_table));

function options = localApplyDefaults(options, root_dir, timestamp)
defaultMaps = { ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_latest.mat'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_latest.mat'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_path_scored_latest.mat')};

if ~isfield(options, 'map_files') || isempty(options.map_files)
    options.map_files = defaultMaps;
end
if ischar(options.map_files) || isstring(options.map_files)
    options.map_files = cellstr(options.map_files);
end
if ~isfield(options, 'success_only') || isempty(options.success_only)
    options.success_only = false;
end
if ~isfield(options, 'include_acceptable') || isempty(options.include_acceptable)
    options.include_acceptable = true;
end
if ~isfield(options, 'selection_mode') || isempty(options.selection_mode)
    options.selection_mode = 'guided_path';
end
if ~isfield(options, 'target_point_count') || isempty(options.target_point_count)
    options.target_point_count = 20;
end
if ~isfield(options, 'guide_polyline') || isempty(options.guide_polyline)
    options.guide_polyline = [ ...
        0.0, 0.0; ...
        0.0, 2.5; ...
        35.0, 20.0; ...
        55.0, 25.0; ...
        55.0, 55.0; ...
        90.0, 75.0];
end
if ~isfield(options, 'include_full_order') || isempty(options.include_full_order)
    options.include_full_order = false;
end
if ~isfield(options, 'disable_nonlinear_hold') || isempty(options.disable_nonlinear_hold)
    options.disable_nonlinear_hold = true;
end
if ~isfield(options, 'checkpoint_every') || isempty(options.checkpoint_every)
    options.checkpoint_every = 10;
end
if ~isfield(options, 'cruise_limit') || isempty(options.cruise_limit)
    options.cruise_limit = inf;
end
if ~isfield(options, 'hover_limit') || isempty(options.hover_limit)
    options.hover_limit = inf;
end
if ~isfield(options, 'write_latest') || isempty(options.write_latest)
    options.write_latest = true;
end
if ~isfield(options, 'output_dir') || isempty(options.output_dir)
    options.output_dir = fullfile(root_dir, 'workspace_plots', ['trim_linearization_db_' timestamp]);
end
end

function sourceEntries = localLoadSourceEntries(mapFiles, options)
sourceEntries = repmat(localSourceSelectionTemplate(), 0, 1);

for iFile = 1:numel(mapFiles)
    mapFile = mapFiles{iFile};
    if exist(mapFile, 'file') ~= 2
        warning('Build_Trim_Linearization_DB:MissingMap', 'Map file not found: %s', mapFile);
        continue;
    end

    [mapStruct, mapVarName] = localLoadMapStruct(mapFile);
    mapLabel = localMapLabelFromPath(mapFile);
    entries = mapStruct.entries;

    for iEntry = 1:numel(entries)
        entry = entries(iEntry);
        isExact = logical(localGetField(entry, 'success', false));
        isAcceptable = logical(localGetField(entry, 'acceptable', false));

        if options.success_only && ~isExact
            continue;
        end
        if ~options.success_only && ~isExact && ~(options.include_acceptable && isAcceptable)
            continue;
        end
        if ~localEntryHasTrimCase(entry)
            continue;
        end

        selection = localSourceSelectionTemplate();
        selection.key = localSourceKey(mapLabel, entry);
        selection.name = localGetField(entry, 'name', '');
        selection.phase = localGetField(entry, 'phase', '');
        selection.family = localGetField(entry, 'family', '');
        selection.source_map_file = mapFile;
        selection.source_map_var = mapVarName;
        selection.source_map_label = mapLabel;
        selection.is_exact = isExact;
        selection.is_acceptable = isAcceptable;
        selection.quality_rank = localQualityRank(isExact, isAcceptable);
        selection.original_entry = entry;
        selection.group = localClassifyGroup(selection);
        sourceEntries(end + 1, 1) = selection; %#ok<SAGROW>
    end
end
end

function [cruiseSelections, hoverSelections, summary] = localPartitionSelections(sourceEntries, options)
sourceEntries = localSelectPathSubset(sourceEntries, options);

cruiseSelections = repmat(localSourceSelectionTemplate(), 0, 1);
hoverSelections = repmat(localSourceSelectionTemplate(), 0, 1);

for i = 1:numel(sourceEntries)
    selection = sourceEntries(i);
    switch selection.group
        case 'cruise'
            cruiseSelections(end + 1, 1) = selection; %#ok<SAGROW>
        case 'hover'
            hoverSelections(end + 1, 1) = selection; %#ok<SAGROW>
        otherwise
            warning('Build_Trim_Linearization_DB:UnknownGroup', ...
                'Skipping selection "%s" with unknown group "%s".', selection.name, selection.group);
    end
end

summary = struct();
summary.nSourceSelections = numel(sourceEntries);
summary.nCruiseSelections = numel(cruiseSelections);
summary.nHoverSelections = numel(hoverSelections);
summary.sourceMapLabels = unique(string({sourceEntries.source_map_label}'), 'stable');
end

function group = localClassifyGroup(selection)
textBlob = lower(strjoin({selection.phase, selection.family, selection.name, selection.source_map_label}, ' '));
if contains(textBlob, 'transition_trim_path_scored') || contains(textBlob, 'hover')
    group = 'hover';
else
    group = 'cruise';
end
end

function selectedEntries = localSelectPathSubset(sourceEntries, options)
switch lower(options.selection_mode)
    case 'all'
        selectedEntries = sourceEntries;
    case 'guided_path'
        selectedEntries = localSelectGuidedPathEntries(sourceEntries, options);
    otherwise
        error('Unknown selection_mode "%s".', options.selection_mode);
end
end

function selectedEntries = localSelectGuidedPathEntries(sourceEntries, options)
selectedEntries = repmat(localSourceSelectionTemplate(), 0, 1);
if isempty(sourceEntries)
    return;
end

guide = options.guide_polyline;
candidateTable = localBuildGuidedCandidateTable(sourceEntries, guide);
if isempty(candidateTable)
    return;
end

targetCount = min(height(candidateTable), max(2, round(options.target_point_count)));
progressTargets = linspace(0.0, 1.0, targetCount);
selectedEntries = localPickGuidedProgressTargets(candidateTable, progressTargets);
selectedEntries = localEnsureGuideAnchors(selectedEntries, candidateTable, guide);
selectedEntries = localEnforceMonotoneSelections(selectedEntries, guide);
selectedEntries = localFillGuidedPathEntries(selectedEntries, candidateTable, guide, targetCount);
selectedEntries = localEnforceMonotoneSelections(selectedEntries, guide);
selectedEntries = localFillGuidedPathEntries(selectedEntries, candidateTable, guide, targetCount);
selectedEntries = localSortSelectionsByGuide(selectedEntries, guide);
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

function pref = localSourcePreference(selection)
label = lower(selection.source_map_label);
if contains(label, 'path_scored')
    pref = 1.0;
elseif contains(label, 'low_speed')
    pref = 0.5;
else
    pref = 0.0;
end
end

function rank = localQualityRank(isExact, isAcceptable)
if isExact
    rank = 2.0;
elseif isAcceptable
    rank = 1.0;
else
    rank = 0.0;
end
end

function selections = localUniqueSelections(selections)
if isempty(selections)
    return;
end

keys = cell(numel(selections), 1);
for i = 1:numel(selections)
    entry = selections(i);
    tilt = localGetField(entry.original_entry, 'target_tilt_deg', localGetField(entry.original_entry, 'tilt_deg', NaN));
    vinf = localGetField(entry.original_entry, 'target_vinf_mps', localGetField(entry.original_entry, 'vinf_mps', NaN));
    keys{i} = sprintf('%.6f__%.6f', tilt, vinf);
end
[~, ia] = unique(keys, 'stable');
selections = selections(sort(ia));
end

function selections = localSortSelectionsByGuide(selections, guide)
if isempty(selections)
    return;
end

progressVals = zeros(numel(selections), 1);
for i = 1:numel(selections)
    [tilt, vinf] = localSelectionPoint(selections(i));
    [progressVals(i), ~] = localProjectPointToGuide(tilt, vinf, guide);
end

[~, order] = sort(progressVals, 'ascend');
selections = selections(order);
end

function selections = localEnforceMonotoneSelections(selections, guide)
if numel(selections) <= 1
    return;
end

selections = localSortSelectionsByGuide(selections, guide);
keepMask = false(numel(selections), 1);
lastTilt = -inf;
lastVinf = -inf;

for i = 1:numel(selections)
    [tilt, vinf] = localSelectionPoint(selections(i));
    if tilt >= lastTilt - 1e-9 && vinf >= lastVinf - 1e-9
        keepMask(i) = true;
        lastTilt = tilt;
        lastVinf = vinf;
    end
end

selections = selections(keepMask);
end

function candidateTable = localBuildGuidedCandidateTable(sourceEntries, guide)
candidateRows = repmat(struct( ...
    'selection', localSourceSelectionTemplate(), ...
    'tilt', NaN, ...
    'vinf', NaN, ...
    'progress', NaN, ...
    'dist_norm', inf, ...
    'quality_rank', 0.0, ...
    'source_pref', 0.0, ...
    'dedupe_cost', inf), 0, 1);

for i = 1:numel(sourceEntries)
    entry = sourceEntries(i);
    if ~entry.is_exact && ~entry.is_acceptable
        continue;
    end

    [tilt, vinf] = localSelectionPoint(entry);
    if ~isfinite(tilt) || ~isfinite(vinf)
        continue;
    end

    [progress, distNorm] = localProjectPointToGuide(tilt, vinf, guide);
    sourcePref = localSourcePreference(entry);

    row = candidateRowsTemplate(entry, tilt, vinf, progress, distNorm, sourcePref);
    candidateRows(end + 1, 1) = row; %#ok<SAGROW>
end

if isempty(candidateRows)
    candidateTable = table();
    return;
end

keys = cell(numel(candidateRows), 1);
for i = 1:numel(candidateRows)
    keys{i} = sprintf('%.6f__%.6f', candidateRows(i).tilt, candidateRows(i).vinf);
end

[uniqueKeys, ~, groupIdx] = unique(keys, 'stable');
keepIdx = zeros(numel(uniqueKeys), 1);
for iGroup = 1:numel(uniqueKeys)
    memberIdx = find(groupIdx == iGroup);
    [~, bestLocal] = min([candidateRows(memberIdx).dedupe_cost]);
    keepIdx(iGroup) = memberIdx(bestLocal);
end

candidateRows = candidateRows(sort(keepIdx));
candidateTable = struct2table(candidateRows);
[~, order] = sort(candidateTable.progress, 'ascend');
candidateTable = candidateTable(order, :);
end

function row = candidateRowsTemplate(entry, tilt, vinf, progress, distNorm, sourcePref)
dedupeCost = 1.50 * distNorm - 0.50 * entry.quality_rank - 0.25 * sourcePref;
row = struct( ...
    'selection', entry, ...
    'tilt', tilt, ...
    'vinf', vinf, ...
    'progress', progress, ...
    'dist_norm', distNorm, ...
    'quality_rank', entry.quality_rank, ...
    'source_pref', sourcePref, ...
    'dedupe_cost', dedupeCost);
end

function selections = localPickGuidedProgressTargets(candidateTable, progressTargets)
selections = repmat(localSourceSelectionTemplate(), 0, 1);
if isempty(candidateTable)
    return;
end

used = false(height(candidateTable), 1);
for iTarget = 1:numel(progressTargets)
    pTarget = progressTargets(iTarget);
    cost = 2.50 * abs(candidateTable.progress - pTarget) + ...
           1.20 * candidateTable.dist_norm - ...
           0.20 * candidateTable.quality_rank - ...
           0.10 * candidateTable.source_pref;
    cost(used) = inf;

    [bestCost, idxBest] = min(cost); %#ok<ASGLU>
    if ~isfinite(bestCost)
        continue;
    end

    used(idxBest) = true;
    selections(end + 1, 1) = candidateTable.selection(idxBest); %#ok<SAGROW>
end

selections = localUniqueSelections(selections);
end

function selections = localEnsureGuideAnchors(selections, candidateTable, guide)
if isempty(candidateTable)
    return;
end

anchorTargets = [guide(1, :); guide(end, :)];
for iAnchor = 1:size(anchorTargets, 1)
    targetTilt = anchorTargets(iAnchor, 1);
    targetVinf = anchorTargets(iAnchor, 2);
    if localSelectionsContainPoint(selections, targetTilt, targetVinf)
        continue;
    end

    cost = sqrt(((candidateTable.tilt - targetTilt) / 10.0).^2 + ((candidateTable.vinf - targetVinf) / 10.0).^2) + ...
           0.40 * candidateTable.dist_norm - ...
           0.20 * candidateTable.quality_rank - ...
           0.10 * candidateTable.source_pref;
    [bestCost, idxBest] = min(cost); %#ok<ASGLU>
    if isfinite(bestCost)
        selections(end + 1, 1) = candidateTable.selection(idxBest); %#ok<SAGROW>
    end
end

selections = localUniqueSelections(selections);
selections = localSortSelectionsByGuide(selections, guide);
end

function selections = localFillGuidedPathEntries(selections, candidateTable, guide, targetCount)
if numel(selections) >= targetCount || isempty(candidateTable)
    return;
end

while numel(selections) < targetCount
    [progressSel, tiltSel, vinfSel] = localSelectionArrays(selections, guide);
    edgeProgress = [0.0; progressSel; 1.0];
    gapVals = diff(edgeProgress);

    usedMask = false(height(candidateTable), 1);
    for i = 1:height(candidateTable)
        usedMask(i) = localSelectionsContainPoint(selections, candidateTable.tilt(i), candidateTable.vinf(i));
    end

    [~, gapOrder] = sort(gapVals, 'descend');
    idxBest = 0;
    bestCost = inf;

    for iGap = 1:numel(gapOrder)
        gapIdx = gapOrder(iGap);
        targetProgress = 0.5 * (edgeProgress(gapIdx) + edgeProgress(gapIdx + 1));

        inGapMask = candidateTable.progress > edgeProgress(gapIdx) + 1e-9 & ...
                    candidateTable.progress < edgeProgress(gapIdx + 1) - 1e-9;
        if ~any(inGapMask & ~usedMask)
            continue;
        end

        monotoneRangeMask = true(height(candidateTable), 1);
        if gapIdx > 1 && gapIdx <= numel(tiltSel)
            leftTilt = tiltSel(gapIdx - 1);
            leftVinf = vinfSel(gapIdx - 1);
        else
            leftTilt = guide(1, 1);
            leftVinf = guide(1, 2);
        end
        if gapIdx <= numel(tiltSel)
            rightTilt = tiltSel(gapIdx);
            rightVinf = vinfSel(gapIdx);
        else
            rightTilt = guide(end, 1);
            rightVinf = guide(end, 2);
        end

        monotoneRangeMask = monotoneRangeMask & candidateTable.tilt >= leftTilt - 1e-9;
        monotoneRangeMask = monotoneRangeMask & candidateTable.vinf >= leftVinf - 1e-9;
        monotoneRangeMask = monotoneRangeMask & candidateTable.tilt <= rightTilt + 1e-9;
        monotoneRangeMask = monotoneRangeMask & candidateTable.vinf <= rightVinf + 1e-9;

        cost = 2.25 * abs(candidateTable.progress - targetProgress) + ...
               1.15 * candidateTable.dist_norm - ...
               0.20 * candidateTable.quality_rank - ...
               0.10 * candidateTable.source_pref;
        cost(~inGapMask | usedMask | ~monotoneRangeMask) = inf;

        [gapBestCost, gapIdxBest] = min(cost);
        if isfinite(gapBestCost)
            bestCost = gapBestCost;
            idxBest = gapIdxBest;
            break;
        end
    end

    if idxBest == 0 || ~isfinite(bestCost)
        break;
    end

    selections(end + 1, 1) = candidateTable.selection(idxBest); %#ok<SAGROW>
    selections = localUniqueSelections(selections);
    selections = localSortSelectionsByGuide(selections, guide);
end
end

function [progressVals, tiltVals, vinfVals] = localSelectionArrays(selections, guide)
n = numel(selections);
progressVals = zeros(n, 1);
tiltVals = zeros(n, 1);
vinfVals = zeros(n, 1);

for i = 1:n
    [tiltVals(i), vinfVals(i)] = localSelectionPoint(selections(i));
    [progressVals(i), ~] = localProjectPointToGuide(tiltVals(i), vinfVals(i), guide);
end
end

function [tilt, vinf] = localSelectionPoint(selection)
tilt = localGetField(selection.original_entry, 'target_tilt_deg', localGetField(selection.original_entry, 'tilt_deg', NaN));
vinf = localGetField(selection.original_entry, 'target_vinf_mps', localGetField(selection.original_entry, 'vinf_mps', NaN));
end

function tf = localSelectionsContainPoint(selections, tilt, vinf)
tf = false;
for i = 1:numel(selections)
    [tilt_i, vinf_i] = localSelectionPoint(selections(i));
    if abs(tilt_i - tilt) < 1e-9 && abs(vinf_i - vinf) < 1e-9
        tf = true;
        return;
    end
end
end

function groupDb = localReplaySelections(groupName, selections, initData, options)
groupDb = localEmptyGroupDb(groupName);
groupDb.meta.source_count = numel(selections);
groupDb.entries = repmat(localResultEntryTemplate(), 0, 1);

for i = 1:numel(selections)
    selection = selections(i);
    trimCase = localExtractTrimCase(selection.original_entry);
    if options.disable_nonlinear_hold
        trimCase.validate_nonlinear_hold = false;
    end

    fprintf('  [%d/%d] %s | source=%s | phase=%s\n', ...
        i, numel(selections), selection.name, selection.source_map_label, selection.phase);

    resultEntry = localResultEntryTemplate();
    resultEntry.index = i;
    resultEntry.key = selection.key;
    resultEntry.group = groupName;
    resultEntry.name = selection.name;
    resultEntry.phase = selection.phase;
    resultEntry.family = selection.family;
    resultEntry.source_map_file = selection.source_map_file;
    resultEntry.source_map_label = selection.source_map_label;
    resultEntry.source_map_var = selection.source_map_var;
    resultEntry.target_tilt_deg = localGetField(selection.original_entry, 'target_tilt_deg', localGetField(selection.original_entry, 'tilt_deg', NaN));
    resultEntry.target_vinf_mps = localGetField(selection.original_entry, 'target_vinf_mps', localGetField(selection.original_entry, 'vinf_mps', NaN));
    resultEntry.target_rear_fixed_rpm = localGetField(selection.original_entry, 'target_rear_fixed_rpm', NaN);
    resultEntry.trim_case = trimCase;

    try
        trimOptions = struct( ...
            'verbose', false, ...
            'debug', false, ...
            'emitSummary', false, ...
            'emitLinearSummary', false);
        [trimResult, ~] = trim_evtol_case(initData, trimCase, trimOptions);

        resultEntry.replay_success = logical(trimResult.success);
        resultEntry.termination_string = trimResult.terminationString;
        resultEntry.max_state_residual = localMaxStateResidual(trimResult.op_report);
        resultEntry.trim_summary = localCompactTrimSummary(trimResult);
        resultEntry.linear = localPackLinearData(trimResult, options.include_full_order);

        fprintf('    replay success = %d | residual = %.6g\n', ...
            resultEntry.replay_success, resultEntry.max_state_residual);
    catch ME
        resultEntry.replay_success = false;
        resultEntry.had_exception = true;
        resultEntry.termination_string = sprintf('EXCEPTION: %s', ME.message);
        resultEntry.error_identifier = ME.identifier;
        resultEntry.error_message = ME.message;
        fprintf('    FAILED: %s\n', ME.message);
    end

    groupDb.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
    groupDb.progress.completed = numel(groupDb.entries);
    groupDb.progress.success_count = nnz([groupDb.entries.replay_success]);
    groupDb.progress.failure_count = groupDb.progress.completed - groupDb.progress.success_count;

    if mod(i, options.checkpoint_every) == 0 || i == numel(selections)
        groupDb.summary_table = localBuildGroupSummary(groupDb.entries);
    end
end

groupDb.summary_table = localBuildGroupSummary(groupDb.entries);
end

function [mapStruct, mapVarName] = localLoadMapStruct(mapFile)
data = load(mapFile);
vars = fieldnames(data);
for i = 1:numel(vars)
    value = data.(vars{i});
    if isstruct(value) && isfield(value, 'entries')
        mapStruct = value;
        mapVarName = vars{i};
        return;
    end
end
error('Could not find a map struct with entries in %s.', mapFile);
end

function tf = localEntryHasTrimCase(entry)
tf = (isfield(entry, 'trim_case') && isstruct(entry.trim_case) && ~isempty(fieldnames(entry.trim_case))) || ...
     (isfield(entry, 'trimCase') && isstruct(entry.trimCase) && ~isempty(fieldnames(entry.trimCase)));
end

function trimCase = localExtractTrimCase(entry)
if isfield(entry, 'trim_case') && isstruct(entry.trim_case) && ~isempty(fieldnames(entry.trim_case))
    trimCase = entry.trim_case;
    return;
end
if isfield(entry, 'trimCase') && isstruct(entry.trimCase) && ~isempty(fieldnames(entry.trimCase))
    trimCase = entry.trimCase;
    return;
end
error('Entry does not contain trim_case / trimCase.');
end

function label = localMapLabelFromPath(mapFile)
[~, nameOnly, ~] = fileparts(mapFile);
label = regexprep(nameOnly, '_latest$', '');
end

function key = localSourceKey(mapLabel, entry)
baseKey = localGetField(entry, 'key', '');
if isempty(baseKey)
    baseKey = sprintf('%s__%s__%s', ...
        localGetField(entry, 'name', ''), ...
        localGetField(entry, 'phase', ''), ...
        localGetField(entry, 'family', ''));
end
key = sprintf('%s__%s', mapLabel, baseKey);
end

function linearData = localPackLinearData(trimResult, includeFullOrder)
linearData = struct();
linearData.replay_exact_trim = logical(localGetField(trimResult, 'success', false));

if ~isfield(trimResult, 'linear') || ~isstruct(trimResult.linear) || ...
        ~isfield(trimResult.linear, 'sys_ss_9state') || isempty(trimResult.linear.sys_ss_9state)
    linearData.reduced_model_available = false;
    return;
end

sys9 = trimResult.linear.sys_ss_9state;
linearData.reduced_model_available = true;
linearData.A_9 = sys9.A;
linearData.B_9 = sys9.B;
linearData.C_9 = sys9.C;
linearData.D_9 = sys9.D;
linearData.state_names_9 = cellstr(sys9.StateName);
linearData.input_names_9 = cellstr(sys9.InputName);
linearData.eigs_9 = eig(sys9.A);
linearData.front_collective_column_9 = localGetField(trimResult.linear, 'B_front_collective', []);
linearData.rear_collective_column_9 = localGetField(trimResult.linear, 'B_rear_collective', []);

if includeFullOrder
    linearData.A_full = localGetField(trimResult.linear, 'A_full', []);
    linearData.B_full = localGetField(trimResult.linear, 'B_full', []);
    linearData.C_full = localGetField(trimResult.linear, 'C_full', []);
    linearData.D_full = localGetField(trimResult.linear, 'D_full', []);
end
end

function summary = localCompactTrimSummary(trimResult)
summary = struct();
summary.name = localGetField(trimResult, 'name', '');
summary.mode = localGetField(trimResult, 'mode', '');
summary.success = logical(localGetField(trimResult, 'success', false));
summary.termination_string = localGetField(trimResult, 'terminationString', '');
summary.front_tilt_deg = localGetNestedField(trimResult, {'scheduling', 'front_tilt_deg'}, NaN);
summary.front_collective_rpm = localGetNestedField(trimResult, {'scheduling', 'front_collective_rpm'}, NaN);
summary.rear_collective_rpm = localGetNestedField(trimResult, {'scheduling', 'rear_collective_rpm'}, NaN);
summary.delta_f_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'delta_f_rad'}, NaN));
summary.delta_a_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'delta_a_rad'}, NaN));
summary.delta_e_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'delta_e_rad'}, NaN));
summary.delta_r_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'delta_r_rad'}, NaN));
summary.theta_deg = localGetIndexedValue(localGetField(trimResult, 'Att_Trim_deg', []), 2, NaN);
summary.u_mps = localGetIndexedValue(localGetField(trimResult, 'Vel_B_BA_Trim', []), 1, NaN);
summary.w_mps = localGetIndexedValue(localGetField(trimResult, 'Vel_B_BA_Trim', []), 3, NaN);
summary.alpha_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'alpha_rad'}, NaN));
summary.max_state_residual = localMaxStateResidual(localGetField(trimResult, 'op_report', struct()));
end

function maxResidual = localMaxStateResidual(op_report)
maxResidual = inf;
try
    states = op_report.States;
catch
    return;
end

residuals = [];
for i = 1:numel(states)
    state_i = states(i);
    try
        dx = state_i.dx;
    catch
        dx = [];
    end
    if isempty(dx)
        continue;
    end

    dx = dx(:);
    steadyMask = true(size(dx));
    try
        rawSteady = state_i.SteadyState;
    catch
        rawSteady = [];
    end
    if ~isempty(rawSteady)
        steadyMask = logical(rawSteady(:));
        if numel(steadyMask) ~= numel(dx)
            steadyMask = true(size(dx));
        end
    end

    residuals = [residuals; abs(dx(steadyMask))]; %#ok<AGROW>
end

if ~isempty(residuals)
    maxResidual = max(residuals);
end
end

function summaryTable = localBuildGroupSummary(entries)
if isempty(entries)
    summaryTable = table();
    return;
end

n = numel(entries);
summaryTable = table( ...
    strings(n, 1), strings(n, 1), strings(n, 1), strings(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
    false(n, 1), strings(n, 1), zeros(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
    'VariableNames', { ...
        'group', 'name', 'phase', 'source_map_label', ...
        'tilt_deg', 'vinf_mps', 'rear_fixed_rpm', ...
        'replay_success', 'termination', 'max_state_residual', ...
        'front_collective_rpm', 'rear_collective_rpm', 'theta_deg'});

for i = 1:n
    entry = entries(i);
    summaryTable.group(i) = string(entry.group);
    summaryTable.name(i) = string(entry.name);
    summaryTable.phase(i) = string(entry.phase);
    summaryTable.source_map_label(i) = string(entry.source_map_label);
    summaryTable.tilt_deg(i) = entry.target_tilt_deg;
    summaryTable.vinf_mps(i) = entry.target_vinf_mps;
    summaryTable.rear_fixed_rpm(i) = entry.target_rear_fixed_rpm;
    summaryTable.replay_success(i) = entry.replay_success;
    summaryTable.termination(i) = string(entry.termination_string);
    summaryTable.max_state_residual(i) = entry.max_state_residual;
    summaryTable.front_collective_rpm(i) = localGetField(entry.trim_summary, 'front_collective_rpm', NaN);
    summaryTable.rear_collective_rpm(i) = localGetField(entry.trim_summary, 'rear_collective_rpm', NaN);
    summaryTable.theta_deg(i) = localGetField(entry.trim_summary, 'theta_deg', NaN);
end

summaryTable = sortrows(summaryTable, {'replay_success', 'phase', 'tilt_deg', 'vinf_mps'}, ...
    {'descend', 'ascend', 'ascend', 'ascend'});
end

function combinedSummary = localBuildCombinedSummary(db)
tables = {};
if isfield(db, 'cruise') && isfield(db.cruise, 'summary_table') && ~isempty(db.cruise.summary_table)
    tables{end + 1} = db.cruise.summary_table; %#ok<AGROW>
end
if isfield(db, 'hover') && isfield(db.hover, 'summary_table') && ~isempty(db.hover.summary_table)
    tables{end + 1} = db.hover.summary_table; %#ok<AGROW>
end

if isempty(tables)
    combinedSummary = table();
    return;
end

combinedSummary = vertcat(tables{:});
combinedSummary = sortrows(combinedSummary, {'group', 'replay_success', 'phase', 'tilt_deg', 'vinf_mps'}, ...
    {'ascend', 'descend', 'ascend', 'ascend', 'ascend'});
end

function localSaveGroupDb(groupDb, outputDir, rootDir, writeLatest)
groupName = groupDb.meta.group_name;
matFile = fullfile(outputDir, sprintf('trim_linearization_db_%s.mat', groupName));
csvFile = fullfile(outputDir, sprintf('trim_linearization_db_%s_summary.csv', groupName));
mdFile = fullfile(outputDir, sprintf('trim_linearization_db_%s_summary.md', groupName));

groupSummary = groupDb.summary_table; %#ok<NASGU>
trimLinearizationDBGroup = groupDb; %#ok<NASGU>
save(matFile, 'trimLinearizationDBGroup', 'groupSummary', '-v7.3');
writetable(groupDb.summary_table, csvFile);
localWriteGroupMarkdown(mdFile, groupDb);

if writeLatest
    latestMat = fullfile(rootDir, 'workspace_plots', sprintf('trim_linearization_db_%s_latest.mat', groupName));
    latestCsv = fullfile(rootDir, 'workspace_plots', sprintf('trim_linearization_db_%s_latest.csv', groupName));
    latestMd = fullfile(rootDir, 'workspace_plots', sprintf('trim_linearization_db_%s_latest.md', groupName));
    save(latestMat, 'trimLinearizationDBGroup', 'groupSummary', '-v7.3');
    writetable(groupDb.summary_table, latestCsv);
    localWriteGroupMarkdown(latestMd, groupDb);
end
end

function localSaveCombinedDb(db, summaryTable, outputDir, rootDir, writeLatest)
matFile = fullfile(outputDir, 'trim_linearization_db.mat');
csvFile = fullfile(outputDir, 'trim_linearization_db_summary.csv');
mdFile = fullfile(outputDir, 'trim_linearization_db_summary.md');

trimLinearizationDB = db; %#ok<NASGU>
trimLinearizationDbSummary = summaryTable; %#ok<NASGU>
save(matFile, 'trimLinearizationDB', 'trimLinearizationDbSummary', '-v7.3');
writetable(summaryTable, csvFile);
localWriteCombinedMarkdown(mdFile, db, summaryTable);

if writeLatest
    latestMat = fullfile(rootDir, 'workspace_plots', 'trim_linearization_db_latest.mat');
    latestCsv = fullfile(rootDir, 'workspace_plots', 'trim_linearization_db_latest.csv');
    latestMd = fullfile(rootDir, 'workspace_plots', 'trim_linearization_db_latest.md');
    save(latestMat, 'trimLinearizationDB', 'trimLinearizationDbSummary', '-v7.3');
    writetable(summaryTable, latestCsv);
    localWriteCombinedMarkdown(latestMd, db, summaryTable);
end
end

function localWriteGroupMarkdown(filename, groupDb)
fid = fopen(filename, 'w');
if fid < 0
    warning('Build_Trim_Linearization_DB:SummaryWriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

summaryTable = groupDb.summary_table;
fprintf(fid, '# Trim Linearization DB: %s\n\n', groupDb.meta.group_name);
fprintf(fid, '- replayed points: %d\n', height(summaryTable));
fprintf(fid, '- replay successes: %d\n', nnz(summaryTable.replay_success));
fprintf(fid, '- replay failures: %d\n\n', height(summaryTable) - nnz(summaryTable.replay_success));

if isempty(summaryTable)
    fprintf(fid, 'No entries.\n');
    return;
end

fprintf(fid, '| name | phase | source | tilt (deg) | V (m/s) | replay success | residual |\n');
fprintf(fid, '| --- | --- | --- | ---: | ---: | --- | ---: |\n');
limit = min(40, height(summaryTable));
for i = 1:limit
    row = summaryTable(i, :);
    fprintf(fid, '| %s | %s | %s | %.1f | %.1f | %d | %.6g |\n', ...
        row.name, row.phase, row.source_map_label, row.tilt_deg, row.vinf_mps, ...
        row.replay_success, row.max_state_residual);
end
end

function localWriteCombinedMarkdown(filename, db, summaryTable)
fid = fopen(filename, 'w');
if fid < 0
    warning('Build_Trim_Linearization_DB:SummaryWriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '# Trim Linearization DB Summary\n\n');
fprintf(fid, 'Generated: %s\n\n', db.meta.created_on);
fprintf(fid, '- trim model: `%s`\n', db.meta.trim_model);
fprintf(fid, '- run model: `%s`\n', db.meta.run_model);
fprintf(fid, '- cruise-side selections: %d\n', db.selection_summary.nCruiseSelections);
fprintf(fid, '- hover-side selections: %d\n\n', db.selection_summary.nHoverSelections);

fprintf(fid, '## Replay Counts\n\n');
fprintf(fid, '| group | replayed | replay successes |\n');
fprintf(fid, '| --- | ---: | ---: |\n');
fprintf(fid, '| cruise | %d | %d |\n', ...
    height(db.cruise.summary_table), nnz(db.cruise.summary_table.replay_success));
fprintf(fid, '| hover | %d | %d |\n\n', ...
    height(db.hover.summary_table), nnz(db.hover.summary_table.replay_success));

if isempty(summaryTable)
    fprintf(fid, 'No replayed entries.\n');
    return;
end

fprintf(fid, '## First 40 Replay Results\n\n');
fprintf(fid, '| group | name | phase | source | tilt (deg) | V (m/s) | replay success | residual |\n');
fprintf(fid, '| --- | --- | --- | --- | ---: | ---: | --- | ---: |\n');
limit = min(40, height(summaryTable));
for i = 1:limit
    row = summaryTable(i, :);
    fprintf(fid, '| %s | %s | %s | %s | %.1f | %.1f | %d | %.6g |\n', ...
        row.group, row.name, row.phase, row.source_map_label, ...
        row.tilt_deg, row.vinf_mps, row.replay_success, row.max_state_residual);
end
end

function groupDb = localEmptyGroupDb(groupName)
groupDb = struct();
groupDb.meta = struct();
groupDb.meta.group_name = groupName;
groupDb.meta.source_count = 0;
groupDb.progress = struct('completed', 0, 'success_count', 0, 'failure_count', 0);
groupDb.entries = repmat(localResultEntryTemplate(), 0, 1);
groupDb.summary_table = table();
end

function selection = localSourceSelectionTemplate()
selection = struct( ...
    'key', '', ...
    'name', '', ...
    'phase', '', ...
    'family', '', ...
    'source_map_file', '', ...
    'source_map_var', '', ...
    'source_map_label', '', ...
    'is_exact', false, ...
    'is_acceptable', false, ...
    'quality_rank', 0.0, ...
    'group', '', ...
    'original_entry', struct());
end

function entry = localResultEntryTemplate()
entry = struct( ...
    'index', 0, ...
    'key', '', ...
    'group', '', ...
    'name', '', ...
    'phase', '', ...
    'family', '', ...
    'source_map_file', '', ...
    'source_map_label', '', ...
    'source_map_var', '', ...
    'target_tilt_deg', NaN, ...
    'target_vinf_mps', NaN, ...
    'target_rear_fixed_rpm', NaN, ...
    'trim_case', struct(), ...
    'trim_summary', struct(), ...
    'replay_success', false, ...
    'termination_string', '', ...
    'max_state_residual', inf, ...
    'had_exception', false, ...
    'error_identifier', '', ...
    'error_message', '', ...
    'linear', struct());
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function value = localGetNestedField(s, fieldPath, defaultValue)
value = s;
for i = 1:numel(fieldPath)
    key = fieldPath{i};
    if ~isstruct(value) || ~isfield(value, key) || isempty(value.(key))
        value = defaultValue;
        return;
    end
    value = value.(key);
end
end

function value = localGetIndexedValue(vec, idx, defaultValue)
if isempty(vec) || numel(vec) < idx
    value = defaultValue;
else
    value = vec(idx);
end
end
