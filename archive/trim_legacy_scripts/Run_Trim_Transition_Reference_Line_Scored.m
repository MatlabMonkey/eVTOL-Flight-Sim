% Run_Trim_Transition_Reference_Line_Scored.m
% Reference-line scored transition sweep. This script samples a narrow band
% around a fixed hover-to-cruise reference curve (matching the hand-drawn
% guide on the success map) and searches only around those points.
%
% Outputs left in the base workspace:
%   - transitionTrimPathScoredMap
%   - transitionTrimPathScoredSummary
%   - transitionTrimPathScoredOutputDir
%
% On-disk outputs:
%   workspace_plots/transition_trim_reference_line_scored_<timestamp>/
%     transition_trim_reference_line_scored.mat
%     transition_trim_reference_line_scored_summary.csv
%     transition_trim_reference_line_scored_summary.md
%     transition_trim_reference_line_scored.log
%
% Convenience copies:
%   workspace_plots/transition_trim_reference_line_scored_latest.mat
%   workspace_plots/transition_trim_reference_line_scored_latest.csv
%   workspace_plots/transition_trim_reference_line_scored_latest.md

if ~exist('transitionPathScoredOptions', 'var') || ~isstruct(transitionPathScoredOptions)
    transitionPathScoredOptions = struct();
end

initOptions = struct();
initOptions.transitionPathScoredOptions = transitionPathScoredOptions;

Init_EVTOL_Main

if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'transitionPathScoredOptions')
    transitionPathScoredOptions = initOptions.transitionPathScoredOptions;
else
    transitionPathScoredOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
config = localBuildConfig(transitionPathScoredOptions, root_dir);
runDirName = sprintf('%s_%s', config.output_prefix, timestamp);
transitionTrimPathScoredOutputDir = fullfile(root_dir, 'workspace_plots', runDirName);
if exist(transitionTrimPathScoredOutputDir, 'dir') ~= 7
    mkdir(transitionTrimPathScoredOutputDir);
end

checkpointFile = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_reference_line_scored.mat');
summaryCsv = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_reference_line_scored_summary.csv');
summaryMd = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_reference_line_scored_summary.md');
logFile = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_reference_line_scored.log');

latestMat = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_latest.mat']);
latestCsv = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_latest.csv']);
latestMd = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_latest.md']);
linearizationOutputDir = fullfile(transitionTrimPathScoredOutputDir, 'linearizations');
latestLinearizationDir = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_linearizations']);
linearizationIndexMat = fullfile(transitionTrimPathScoredOutputDir, ...
    'transition_trim_reference_line_scored_linearization_index.mat');
linearizationIndexCsv = fullfile(transitionTrimPathScoredOutputDir, ...
    'transition_trim_reference_line_scored_linearization_index.csv');
latestLinearizationIndexMat = fullfile(root_dir, 'workspace_plots', ...
    [config.latest_prefix '_linearization_index.mat']);
latestLinearizationIndexCsv = fullfile(root_dir, 'workspace_plots', ...
    [config.latest_prefix '_linearization_index.csv']);
if exist(linearizationOutputDir, 'dir') ~= 7
    mkdir(linearizationOutputDir);
end
if exist(latestLinearizationDir, 'dir') ~= 7
    mkdir(latestLinearizationDir);
end

diary off;
diary(logFile);
cleanupDiary = onCleanup(@() diary('off'));

fprintf('=== Run_Trim_Transition_Reference_Line_Scored ===\n');
fprintf('Started: %s\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf('Output dir: %s\n', transitionTrimPathScoredOutputDir);

historyBundle = localLoadHistorySeedBundle(config);
targetQueue = localBuildReferenceTargets(historyBundle, config);
attemptedKeys = containers.Map('KeyType', 'char', 'ValueType', 'logical');
knownGoodPoints = historyBundle.knownGoodPoints;
knownGoodKeys = localKnownGoodKeyMap(knownGoodPoints);
[linearizationIndex, linearizationKeyToIndex] = localLoadExistingLinearizationIndex(latestLinearizationIndexMat);

fprintf('Loaded %d exact-history seeds and %d scored-history seeds.\n', ...
    numel(historyBundle.exactSeeds), numel(historyBundle.scoredSeeds));
fprintf('Built %d reference-band targets around the drawn curve.\n', numel(targetQueue));

transitionTrimPathScoredMap = struct();
transitionTrimPathScoredMap.meta = struct();
transitionTrimPathScoredMap.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
transitionTrimPathScoredMap.meta.root_dir = root_dir;
transitionTrimPathScoredMap.meta.output_dir = transitionTrimPathScoredOutputDir;
transitionTrimPathScoredMap.meta.trim_model = initData.modelNames.trim;
transitionTrimPathScoredMap.meta.run_model = initData.modelNames.run;
transitionTrimPathScoredMap.meta.config = config;
transitionTrimPathScoredMap.meta.exact_history_seed_count = numel(historyBundle.exactSeeds);
transitionTrimPathScoredMap.meta.scored_history_seed_count = numel(historyBundle.scoredSeeds);
transitionTrimPathScoredMap.meta.known_good_point_count = size(historyBundle.knownGoodPoints, 1);
transitionTrimPathScoredMap.meta.blocked_point_count = size(historyBundle.blockedPoints, 1);
transitionTrimPathScoredMap.meta.reference_target_count = numel(targetQueue);
transitionTrimPathScoredMap.meta.linearization_output_dir = linearizationOutputDir;
transitionTrimPathScoredMap.meta.latest_linearization_dir = latestLinearizationDir;
transitionTrimPathScoredMap.meta.latest_linearization_index_mat = latestLinearizationIndexMat;
transitionTrimPathScoredMap.meta.latest_linearization_index_csv = latestLinearizationIndexCsv;
transitionTrimPathScoredMap.targets = targetQueue;
transitionTrimPathScoredMap.entries = repmat(localEntryTemplate(), 0, 1);
transitionTrimPathScoredMap.linearization_index = localBuildLinearizationIndexTable(linearizationIndex);
transitionTrimPathScoredMap.progress = struct('completed', 0, 'success_count', 0, ...
    'acceptable_count', 0, 'last_key', '', 'queue_remaining', numel(targetQueue), ...
    'linearization_count', numel(linearizationIndex));

transitionTrimPathScoredSummary = table();

try
    iAttempt = 0;
    while ~isempty(targetQueue) && iAttempt < config.max_attempts
        target = targetQueue(1);
        targetQueue(1) = [];
        queueKey = localTargetKey(target);
        if isKey(knownGoodKeys, queueKey) || isKey(attemptedKeys, queueKey)
            continue;
        end

        iAttempt = iAttempt + 1;
        fprintf('\n[%d/%d queued] %s | tilt=%.1f | V=%.1f | parent=%s | prio=%.3f\n', ...
            iAttempt, iAttempt + numel(targetQueue), target.name, target.tilt_deg, target.vinf_mps, ...
            target.parent_name, target.sort_key);

        entry = localSolveTarget(target, transitionTrimPathScoredMap.entries, historyBundle.allSeeds, initData, config);
        attemptedKeys(queueKey) = true;
        transitionTrimPathScoredMap.entries(end + 1, 1) = entry; %#ok<SAGROW>
        transitionTrimPathScoredMap.progress.completed = numel(transitionTrimPathScoredMap.entries);
        transitionTrimPathScoredMap.progress.success_count = nnz([transitionTrimPathScoredMap.entries.success]);
        transitionTrimPathScoredMap.progress.acceptable_count = nnz([transitionTrimPathScoredMap.entries.acceptable]);
        transitionTrimPathScoredMap.progress.last_key = entry.key;
        transitionTrimPathScoredMap.progress.queue_remaining = numel(targetQueue);

        fprintf('  best family=%s | seed=%s | success=%d | acceptable=%d | score=%.4f | class=%s\n', ...
            entry.family, entry.seed_name, entry.success, entry.acceptable, ...
            entry.score, entry.classification);
        fprintf('  term: %s\n', entry.termination_string);

        if entry.success || entry.acceptable
            knownGoodPoints(end + 1, :) = [entry.tilt_deg, entry.vinf_mps]; %#ok<AGROW>
            knownGoodKeys(queueKey) = true;
        end

        [linearizationIndex, linearizationKeyToIndex] = localSaveUsableLinearization( ...
            entry, linearizationIndex, linearizationKeyToIndex, ...
            linearizationOutputDir, latestLinearizationDir, ...
            linearizationIndexMat, latestLinearizationIndexMat, ...
            linearizationIndexCsv, latestLinearizationIndexCsv);
        transitionTrimPathScoredMap.linearization_index = localBuildLinearizationIndexTable(linearizationIndex);
        transitionTrimPathScoredMap.progress.linearization_count = numel(linearizationIndex);

        if mod(iAttempt, config.checkpoint_every) == 0 || entry.success || entry.acceptable || isempty(targetQueue)
            transitionTrimPathScoredMap.targets = targetQueue;
            [transitionTrimPathScoredSummary, transitionTrimPathScoredMap] = ...
                localCheckpoint(transitionTrimPathScoredMap, checkpointFile, latestMat, ...
                summaryCsv, latestCsv, summaryMd, latestMd);
            fprintf('  checkpoint saved.\n');
        end
    end

    transitionTrimPathScoredMap.targets = targetQueue;
    [transitionTrimPathScoredSummary, transitionTrimPathScoredMap] = ...
        localCheckpoint(transitionTrimPathScoredMap, checkpointFile, latestMat, ...
        summaryCsv, latestCsv, summaryMd, latestMd);

    fprintf('\nCompleted reference-line scored sweep.\n');
    fprintf('  total entries = %d\n', height(transitionTrimPathScoredSummary));
    if ~isempty(transitionTrimPathScoredSummary)
        fprintf('  exact successes = %d\n', nnz(transitionTrimPathScoredSummary.success));
        fprintf('  acceptable near-trims = %d\n', nnz(transitionTrimPathScoredSummary.acceptable));
    else
        fprintf('  exact successes = 0\n');
        fprintf('  acceptable near-trims = 0\n');
    end
    fprintf('  queue remaining = %d\n', numel(targetQueue));
    fprintf('  checkpoint = %s\n', checkpointFile);
catch ME
    fprintf('\nRun_Trim_Transition_Reference_Line_Scored failed: %s\n', ME.message);
    transitionTrimPathScoredMap.targets = targetQueue;
    [transitionTrimPathScoredSummary, transitionTrimPathScoredMap] = ...
        localCheckpoint(transitionTrimPathScoredMap, checkpointFile, latestMat, ...
        summaryCsv, latestCsv, summaryMd, latestMd);
    rethrow(ME);
end

function config = localBuildConfig(userOptions, root_dir)
config = struct();
config.root_dir = root_dir;
config.output_prefix = localGetField(userOptions, 'output_prefix', 'transition_trim_reference_line_scored');
config.latest_prefix = localGetField(userOptions, 'latest_prefix', 'transition_trim_reference_line_scored');
config.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 5);
config.max_attempts = localGetField(userOptions, 'max_attempts', 250);
config.min_tilt_deg = localGetField(userOptions, 'min_tilt_deg', 0.0);
config.max_tilt_deg = localGetField(userOptions, 'max_tilt_deg', 90.0);
config.min_vinf_mps = localGetField(userOptions, 'min_vinf_mps', 0.0);
config.max_vinf_mps = localGetField(userOptions, 'max_vinf_mps', 70.0);
config.neighbor_seed_count = localGetField(userOptions, 'neighbor_seed_count', 1);
config.history_exact_seed_count = localGetField(userOptions, 'history_exact_seed_count', 1);
config.history_scored_seed_count = localGetField(userOptions, 'history_scored_seed_count', 1);
config.max_prop_nudge_fraction = localGetField(userOptions, 'max_prop_nudge_fraction', 0.15);
config.front_collective_min_rpm = localGetField(userOptions, 'front_collective_min_rpm', 0.0);
config.rear_collective_min_rpm = localGetField(userOptions, 'rear_collective_min_rpm', 0.0);
config.history_min_rear_collective_rpm = localGetField(userOptions, 'history_min_rear_collective_rpm', ...
    config.rear_collective_min_rpm);
config.hover_anchor_case_name = localGetField(userOptions, 'hover_anchor_case_name', 'TrimCase_Hover');
config.cruise_anchor_case_name = localGetField(userOptions, 'cruise_anchor_case_name', 'TrimCase_Cruise75_FlapElevator');
config.enabled_families = localGetField(userOptions, 'enabled_families', { ...
    'front_rear_free_flap_elevator', ...
    'prop_fixed_flap_elevator', ...
    'rear_fixed_flap_elevator', ...
    'hover_zero_surface'});
config.use_analytic_fallback = localGetField(userOptions, 'use_analytic_fallback', false);
config.reference_vinf_samples = localGetField(userOptions, 'reference_vinf_samples', 0:2.5:70);
config.reference_knot_vinf_mps = localGetField(userOptions, 'reference_knot_vinf_mps', [0 2.5 5 10 20 30 40 50 60 70]);
config.reference_knot_tilt_deg = localGetField(userOptions, 'reference_knot_tilt_deg', [0 18 30 44 62 72 80 85 88 90]);
config.reference_tilt_offsets_deg = localGetField(userOptions, 'reference_tilt_offsets_deg', [-5 0 5]);
config.reference_grid_step_deg = localGetField(userOptions, 'reference_grid_step_deg', 5.0);
config.reference_parent_seed_count = localGetField(userOptions, 'reference_parent_seed_count', 1);
config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);
config.score_options = localGetField(userOptions, 'score_options', struct( ...
    'profile', 'transition', ...
    'hold_horizon_s', 2.0));
defaultExactCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv');
defaultScoredCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv');
config.history_exact_csv = localGetField(userOptions, 'history_exact_csv', defaultExactCsv);
config.history_scored_csv = localGetField(userOptions, 'history_scored_csv', defaultScoredCsv);
config.history_exact_csvs = localNormalizeCsvList(localGetField(userOptions, 'history_exact_csvs', {config.history_exact_csv}));
config.history_scored_csvs = localNormalizeCsvList(localGetField(userOptions, 'history_scored_csvs', {config.history_scored_csv}));
config.seed_distance_weights = struct('tilt', 1 / 5, 'vinf', 1 / 5);
config.cruise_anchor_seed = localNormalizeCruiseAnchorSeed(localGetField(userOptions, 'cruise_anchor_seed', struct()));
if config.cruise_anchor_seed.is_valid
    config.cruise_anchor = struct( ...
        'tilt_deg', config.cruise_anchor_seed.tilt_deg, ...
        'vinf_mps', config.cruise_anchor_seed.vinf_mps);
else
    config.cruise_anchor = struct('tilt_deg', 90.0, 'vinf_mps', 70.0);
end
config.midband_vinf_range_mps = localGetField(userOptions, 'midband_vinf_range_mps', [20.0 50.0]);
config.midband_interp_tilt_window_deg = localGetField(userOptions, 'midband_interp_tilt_window_deg', 20.0);
config.midband_interp_fracs = localGetField(userOptions, 'midband_interp_fracs', [0.35 0.5 0.65]);
end

function targetQueue = localBuildReferenceTargets(historyBundle, config)
targetQueue = repmat(localTargetTemplate(), 0, 1);
knownGoodKeys = localKnownGoodKeyMap(historyBundle.knownGoodPoints);
seedPool = localUniqueSeeds([historyBundle.scoredSeeds; historyBundle.exactSeeds]);

for iV = 1:numel(config.reference_vinf_samples)
    vinf_mps = config.reference_vinf_samples(iV);
    centerTilt = localReferenceTilt(vinf_mps, config);
    for iOffset = 1:numel(config.reference_tilt_offsets_deg)
        rawTilt = centerTilt + config.reference_tilt_offsets_deg(iOffset);
        tilt_deg = config.reference_grid_step_deg * round(rawTilt / config.reference_grid_step_deg);
        tilt_deg = min(max(tilt_deg, config.min_tilt_deg), config.max_tilt_deg);
        key = localKnownPointKey(tilt_deg, vinf_mps);
        if isKey(knownGoodKeys, key)
            continue;
        end

        parentSeed = localSelectReferenceParentSeed(seedPool, tilt_deg, vinf_mps, config);
        target = localTargetTemplate();
        target.tilt_deg = tilt_deg;
        target.vinf_mps = vinf_mps;
        target.name = sprintf('ReferenceLineScored_Tilt%s_V%s', ...
            localValueLabel(tilt_deg), localValueLabel(vinf_mps));
        target.parent_seed = parentSeed;
        target.parent_name = parentSeed.name;
        target.sort_key = vinf_mps + 0.01 * abs(config.reference_tilt_offsets_deg(iOffset)) + ...
            0.001 * abs(tilt_deg - centerTilt);
        targetQueue(end + 1, 1) = target; %#ok<SAGROW>
    end
end

if isempty(targetQueue)
    return;
end

[~, order] = sort([targetQueue.sort_key], 'ascend');
targetQueue = targetQueue(order);
end

function seed = localSelectReferenceParentSeed(seedPool, tilt_deg, vinf_mps, config)
if isempty(seedPool)
    seed = localSeedTemplate();
    seed.name = 'reference_default';
    seed.source = 'reference_default';
    seed.tilt_deg = tilt_deg;
    seed.vinf_mps = vinf_mps;
    return;
end

distances = arrayfun(@(s) localSeedDistance(s, struct('tilt_deg', tilt_deg, 'vinf_mps', vinf_mps), config.seed_distance_weights), seedPool);
merits = arrayfun(@localFrontierSeedMerit, seedPool);
combined = distances - 0.01 * merits;
[~, idx] = min(combined);
seed = localCanonicalizeSeed(seedPool(idx));
seed.tilt_deg = tilt_deg;
seed.vinf_mps = vinf_mps;
end

function tilt_deg = localReferenceTilt(vinf_mps, config)
tilt_deg = interp1(config.reference_knot_vinf_mps, config.reference_knot_tilt_deg, ...
    vinf_mps, 'pchip', 'extrap');
tilt_deg = min(max(tilt_deg, config.min_tilt_deg), config.max_tilt_deg);
end

function historyBundle = localLoadHistorySeedBundle(config)
historyBundle = struct();
historyBundle.exactSeeds = localLoadHistorySeedsFromCsvList(config.history_exact_csvs, 'success_only', 'history_exact', ...
    config.history_min_rear_collective_rpm);
historyBundle.scoredSeeds = localLoadHistorySeedsFromCsvList(config.history_scored_csvs, 'acceptable_or_better', 'history_scored', ...
    config.history_min_rear_collective_rpm);
historyBundle.allSeeds = localUniqueSeeds([historyBundle.scoredSeeds; historyBundle.exactSeeds]);
historyBundle.knownGoodPoints = localLoadKnownGoodPoints(config.history_exact_csvs, config.history_scored_csvs);
historyBundle.blockedPoints = localLoadRejectedPointsFromCsvList(config.history_scored_csvs);
end

function frontierSeeds = localBuildInitialFrontierSeeds(historyBundle, config)
frontierSeeds = repmat(localSeedTemplate(), 0, 1);
seedPool = historyBundle.scoredSeeds;
if ~config.use_only_scored_frontier
    seedPool = [seedPool; historyBundle.exactSeeds];
end

for i = 1:numel(seedPool)
    seed = localCanonicalizeSeed(seedPool(i));
    if seed.tilt_deg > config.initial_frontier_tilt_max_deg || ...
            seed.vinf_mps > config.initial_frontier_vinf_max_mps
        continue;
    end
    frontierSeeds(end + 1, 1) = seed; %#ok<SAGROW>
end

frontierSeeds = localUniqueSeeds(frontierSeeds);
frontierSeeds = localSelectFrontierEnvelope(frontierSeeds);

if ~isempty(config.start_points)
    frontierSeeds = localSelectCustomStartSeeds(frontierSeeds, historyBundle, config);
end
end

function frontierSeeds = localSelectCustomStartSeeds(defaultFrontierSeeds, historyBundle, config)
candidateSeeds = [historyBundle.scoredSeeds; historyBundle.exactSeeds; defaultFrontierSeeds];
candidateSeeds = localUniqueSeeds(candidateSeeds);

selected = repmat(localSeedTemplate(), 0, 1);
for i = 1:numel(config.start_points)
    startPoint = config.start_points(i);
    distances = arrayfun(@(s) localSeedDistance(s, startPoint, config.seed_distance_weights), candidateSeeds);
    [~, idx] = min(distances);
    selected(end + 1, 1) = candidateSeeds(idx); %#ok<SAGROW>
end

frontierSeeds = localSortSeedsByProgress(localUniqueSeeds(selected));
end

function frontierSeeds = localSelectFrontierEnvelope(seedPool)
if isempty(seedPool)
    return;
end

tiltValues = unique([seedPool.tilt_deg]);
selected = repmat(localSeedTemplate(), 0, 1);
for iTilt = 1:numel(tiltValues)
    tilt_deg = tiltValues(iTilt);
    sameTilt = seedPool(abs([seedPool.tilt_deg] - tilt_deg) < 1e-9);
    merits = arrayfun(@localFrontierSeedMerit, sameTilt);
    [~, idx] = max(merits);
    selected(end + 1, 1) = sameTilt(idx); %#ok<SAGROW>
end
frontierSeeds = localSortSeedsByProgress(selected);
end

function seeds = localSortSeedsByProgress(seeds)
if isempty(seeds)
    return;
end
[~, order] = sortrows([[seeds.tilt_deg]' [seeds.vinf_mps]'], [1 2]);
seeds = seeds(order);
end

function [targetQueue, queuedKeys] = localBuildInitialQueue(frontierSeeds, knownGoodPoints, blockedPoints, config)
targetQueue = repmat(localTargetTemplate(), 0, 1);
queuedKeys = containers.Map('KeyType', 'char', 'ValueType', 'double');
knownGoodKeys = localKnownGoodKeyMap(knownGoodPoints);
blockedKeys = localKnownGoodKeyMap(blockedPoints);
attemptedKeys = containers.Map('KeyType', 'char', 'ValueType', 'logical');

for i = 1:numel(frontierSeeds)
    [targetQueue, queuedKeys] = localEnqueueTargetsFromSeed(targetQueue, queuedKeys, ...
        frontierSeeds(i), knownGoodKeys, blockedKeys, attemptedKeys, config);
end
targetQueue = localSortQueue(targetQueue);
end

function [targetQueue, queuedKeys] = localEnqueueTargetsFromSeed(targetQueue, queuedKeys, parentSeed, knownGoodKeys, blockedKeys, attemptedKeys, config)
childMoves = localExpansionMoves(parentSeed, config);
for iMove = 1:size(childMoves, 1)
    dt = childMoves(iMove, 1);
    dv = childMoves(iMove, 2);
    tilt_deg = parentSeed.tilt_deg + dt;
    vinf_mps = parentSeed.vinf_mps + dv;

    if tilt_deg < config.min_tilt_deg || tilt_deg > config.max_tilt_deg || ...
            vinf_mps < config.min_vinf_mps || vinf_mps > config.max_vinf_mps
        continue;
    end

    target = localTargetTemplate();
    target.tilt_deg = tilt_deg;
    target.vinf_mps = vinf_mps;
    target.name = sprintf('PathScored_Tilt%s_V%s', ...
        localValueLabel(tilt_deg), localValueLabel(vinf_mps));
    target.parent_seed = parentSeed;
    target.parent_name = parentSeed.name;
    target.sort_key = localPathPriority(parentSeed, target, config);
    guideV = localGuideVinfForTilt(target.tilt_deg, config);

    if target.vinf_mps > guideV + config.guide_speed_slack_mps
        continue;
    end

    key = localTargetKey(target);
    if isKey(knownGoodKeys, key) || isKey(blockedKeys, key) || isKey(attemptedKeys, key)
        continue;
    end
    if isKey(queuedKeys, key) && queuedKeys(key) <= target.sort_key
        continue;
    end

    queuedKeys(key) = target.sort_key;
    targetQueue(end + 1, 1) = target; %#ok<SAGROW>
end

targetQueue = localSortQueue(targetQueue);
end

function moves = localExpansionMoves(parentSeed, config)
if parentSeed.tilt_deg < 25
    moves = [5 0; 5 2.5; 0 2.5];
elseif parentSeed.tilt_deg < 45
    moves = [5 0; 5 2.5];
elseif parentSeed.tilt_deg < 70
    moves = [0 5; 5 0; 5 2.5; 0 10; 5 5];
else
    moves = [0 5; 0 10; 5 5; 5 0];
end

% Respect the remaining cruise distance.
cruiseRemaining = [config.cruise_anchor.tilt_deg - parentSeed.tilt_deg, ...
    config.cruise_anchor.vinf_mps - parentSeed.vinf_mps];
keepMask = moves(:, 1) <= max(cruiseRemaining(1), 0) + 1e-9 & ...
    moves(:, 2) <= max(cruiseRemaining(2), 0) + 1e-9;
if any(keepMask)
    moves = moves(keepMask, :);
end
end

function priority = localPathPriority(parentSeed, target, config)
dt = target.tilt_deg - parentSeed.tilt_deg;
dv = target.vinf_mps - parentSeed.vinf_mps;
cruiseDistance = hypot((config.cruise_anchor.tilt_deg - target.tilt_deg) * config.seed_distance_weights.tilt, ...
    (config.cruise_anchor.vinf_mps - target.vinf_mps) * config.seed_distance_weights.vinf);
guideV = localGuideVinfForTilt(target.tilt_deg, config);
guideDeviation = abs(target.vinf_mps - guideV);

priority = cruiseDistance;
if parentSeed.tilt_deg < 45
    priority = priority - 5.0 * dt + 2.0 * dv + 0.9 * guideDeviation;
    if dt == 0
        priority = priority + 5.0;
    end
    if target.vinf_mps > guideV + config.guide_speed_slack_mps
        priority = priority + 8.0 + 0.75 * (target.vinf_mps - guideV);
    end
else
    priority = priority - 1.0 * dt - 3.0 * dv + 0.4 * guideDeviation;
end

if dt > 0 && dv <= 2.5
    priority = priority - 1.0;
end
end

function queue = localSortQueue(queue)
if isempty(queue)
    return;
end
[~, order] = sort([queue.sort_key], 'ascend');
queue = queue(order);
end

function merit = localFrontierSeedMerit(seed)
controlEffort = abs(seed.delta_f_guess_deg) + abs(seed.delta_a_guess_deg) + ...
    abs(seed.delta_e_guess_deg) + abs(seed.delta_r_guess_deg);

classBonus = 0.0;
if localGetField(seed, 'success', false)
    classBonus = 100.0;
elseif localGetField(seed, 'acceptable', false)
    classBonus = 60.0;
elseif strcmp(localGetField(seed, 'classification', ''), 'near_trim_borderline')
    classBonus = 20.0;
end

scorePenalty = min(localGetField(seed, 'score', 10.0), 10.0);
merit = classBonus + seed.vinf_mps - 0.35 * scorePenalty - 0.05 * controlEffort;
end

function guideV = localGuideVinfForTilt(tilt_deg, config)
if tilt_deg <= 30.0
    guideV = min(config.guide_row_speed_mps, 0.8 * tilt_deg + 2.5);
elseif tilt_deg <= config.guide_row_tilt_end_deg
    guideV = config.guide_row_speed_mps;
else
    tiltSpan = max(config.cruise_anchor.tilt_deg - config.guide_row_tilt_end_deg, 1.0);
    frac = (tilt_deg - config.guide_row_tilt_end_deg) / tiltSpan;
    guideV = config.guide_row_speed_mps + frac * (config.cruise_anchor.vinf_mps - config.guide_row_speed_mps);
end
guideV = min(max(guideV, config.min_vinf_mps), config.max_vinf_mps);
end

function historySeeds = localLoadHistorySeedsFromCsv(filename, mode, sourceName, minRearCollectiveRpm)
historySeeds = repmat(localSeedTemplate(), 0, 1);
if exist(filename, 'file') ~= 2
    return;
end
if nargin < 4
    minRearCollectiveRpm = 0.0;
end

summary = readtable(filename);
requiredVars = {'success', 'tilt_deg', 'vinf_mps', 'front_collective_rpm', ...
    'rear_collective_rpm', 'theta_deg', 'delta_f_deg', 'delta_e_deg'};
if ~all(ismember(requiredVars, summary.Properties.VariableNames))
    return;
end

switch mode
    case 'success_only'
        keepMask = localTableLogical(summary.success);
    case 'acceptable_or_better'
        if ismember('acceptable', summary.Properties.VariableNames)
            keepMask = localTableLogical(summary.acceptable) | localTableLogical(summary.success);
        else
            keepMask = localTableLogical(summary.success);
        end
    otherwise
        keepMask = true(height(summary), 1);
end
summary = summary(keepMask, :);
if ismember('rear_collective_rpm', summary.Properties.VariableNames)
    rearValues = summary.rear_collective_rpm;
    rearMask = isfinite(rearValues) & rearValues >= minRearCollectiveRpm;
    summary = summary(rearMask, :);
end
for i = 1:height(summary)
    seed = localSeedTemplate();
    seed.name = sprintf('history_%s', localValueLabel(i));
    seed.source = sourceName;
    seed.tilt_deg = summary.tilt_deg(i);
    seed.vinf_mps = summary.vinf_mps(i);
    seed.front_collective_guess_rpm = summary.front_collective_rpm(i);
    seed.rear_collective_guess_rpm = summary.rear_collective_rpm(i);
    seed.theta_guess_deg = summary.theta_deg(i);
    seed.delta_f_guess_deg = localTableValue(summary, 'delta_f_deg', i, 0.0);
    seed.delta_a_guess_deg = localTableValue(summary, 'delta_a_deg', i, 0.0);
    seed.delta_e_guess_deg = localTableValue(summary, 'delta_e_deg', i, 0.0);
    seed.delta_r_guess_deg = localTableValue(summary, 'delta_r_deg', i, 0.0);
    seed.success = localTableValue(summary, 'success', i, false);
    seed.acceptable = localTableValue(summary, 'acceptable', i, false);
    seed.classification = char(string(localTableValue(summary, 'classification', i, "")));
    seed.score = localTableValue(summary, 'score', i, inf);
    historySeeds(end + 1, 1) = seed; %#ok<SAGROW>
end
end

function historySeeds = localLoadHistorySeedsFromCsvList(filenames, mode, sourceName, minRearCollectiveRpm)
historySeeds = repmat(localSeedTemplate(), 0, 1);
filenames = localNormalizeCsvList(filenames);
for i = 1:numel(filenames)
    fileSeeds = localLoadHistorySeedsFromCsv(filenames{i}, mode, sourceName, minRearCollectiveRpm);
    historySeeds = [historySeeds; fileSeeds]; %#ok<AGROW>
end
historySeeds = localUniqueSeeds(historySeeds);
end

function entry = localSolveTarget(target, existingEntries, historySeeds, initData, config)
seedCandidates = localBuildSeedCandidates(target, existingEntries, historySeeds, initData, config);
attempts = repmat(localAttemptTemplate(), 0, 1);
bestAttempt = localAttemptTemplate();
shouldStop = false;

for iSeed = 1:numel(seedCandidates)
    seed = seedCandidates(iSeed);
    families = localBuildFamilies(target, seed, config);
    for iFam = 1:numel(families)
        family = families(iFam);
        try
            [trimResult, ~] = trim_evtol_case(initData, family.trimCase, config.trim_options);
        catch ME
            trimResult = struct('name', family.trimCase.name, ...
                'mode', family.trimCase.mode, ...
                'success', false, ...
                'terminationString', ME.message);
        end

        try
            scoreData = score_trim_point(trimResult, config.score_options);
        catch
            scoreData = localFallbackScore(trimResult);
        end

        attempt = localMakeAttempt(target, family, trimResult, scoreData, config);
        attempts(end + 1, 1) = attempt; %#ok<SAGROW>
        if localIsBetterAttempt(attempt, bestAttempt)
            bestAttempt = attempt;
        end
        if attempt.success
            shouldStop = true;
            break;
        end
    end
    if shouldStop
        break;
    end
end

entry = localEntryTemplate();
entry.key = localTargetKey(target);
entry.name = target.name;
entry.tilt_deg = target.tilt_deg;
entry.vinf_mps = target.vinf_mps;
entry.family = bestAttempt.family;
entry.seed_name = bestAttempt.seed_name;
entry.success = bestAttempt.success;
entry.acceptable = bestAttempt.acceptable;
entry.classification = bestAttempt.classification;
entry.score = bestAttempt.score;
entry.max_normalized = bestAttempt.max_normalized;
entry.worst_component = bestAttempt.worst_component;
entry.worst_component_normalized = bestAttempt.worst_component_normalized;
entry.termination_string = bestAttempt.termination_string;
entry.front_collective_rpm = bestAttempt.front_collective_rpm;
entry.rear_collective_rpm = bestAttempt.rear_collective_rpm;
entry.delta_f_deg = bestAttempt.delta_f_deg;
entry.delta_a_deg = bestAttempt.delta_a_deg;
entry.delta_e_deg = bestAttempt.delta_e_deg;
entry.delta_r_deg = bestAttempt.delta_r_deg;
entry.theta_deg = bestAttempt.theta_deg;
entry.u_mps = bestAttempt.u_mps;
entry.w_mps = bestAttempt.w_mps;
entry.alpha_deg = bestAttempt.alpha_deg;
entry.trimCase = bestAttempt.trimCase;
entry.trimResult = bestAttempt.trimResult;
entry.scoreData = bestAttempt.scoreData;
entry.attempts = attempts;
end

function seedCandidates = localBuildSeedCandidates(target, existingEntries, historySeeds, initData, config)
seedCandidates = repmat(localSeedTemplate(), 0, 1);

baseCase = localBaseSeedCase(target, config);

parentSeed = localCanonicalizeSeed(target.parent_seed);
parentSeed.name = ['parent_' target.parent_name];
seedCandidates(end + 1, 1) = parentSeed; %#ok<SAGROW>

nudgedSeed = localMakeCruiseNudgedSeed(parentSeed, target, config);
seedCandidates(end + 1, 1) = nudgedSeed; %#ok<SAGROW>

neighborEntries = existingEntries([existingEntries.success] | [existingEntries.acceptable]);
if ~isempty(neighborEntries)
    distances = arrayfun(@(e) localEntryDistance(e, target, config.seed_distance_weights), neighborEntries);
    [~, order] = sort(distances, 'ascend');
    order = order(1:min(numel(order), config.neighbor_seed_count));
    for i = 1:numel(order)
        seedCandidates(end + 1, 1) = localSeedFromEntry(neighborEntries(order(i))); %#ok<SAGROW>
    end
end

if ~isempty(historySeeds)
    exactMask = arrayfun(@(s) strcmp(localGetField(s, 'source', ''), 'history_exact'), historySeeds);
    scoredMask = arrayfun(@(s) strcmp(localGetField(s, 'source', ''), 'history_scored'), historySeeds);
    seedCandidates = localAppendClosestHistorySeeds(seedCandidates, historySeeds(exactMask), ...
        target, config.seed_distance_weights, config.history_exact_seed_count);
    seedCandidates = localAppendClosestHistorySeeds(seedCandidates, historySeeds(scoredMask), ...
        target, config.seed_distance_weights, config.history_scored_seed_count);
end

currentGoodSeeds = repmat(localSeedTemplate(), 0, 1);
if ~isempty(neighborEntries)
    currentGoodSeeds = arrayfun(@localSeedFromEntry, neighborEntries);
end
seedCandidates = localAppendInterpolatedMidbandSeeds(seedCandidates, target, ...
    currentGoodSeeds, historySeeds, config);

if config.use_analytic_fallback
    try
        [propSeed, ~] = make_low_speed_prop_first_pass_seed(initData, baseCase);
        propSeed.source = 'prop_first_pass';
        propSeed.tilt_deg = target.tilt_deg;
        propSeed.vinf_mps = target.vinf_mps;
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(propSeed); %#ok<SAGROW>
    catch
    end

    try
        [twoPassSeed, ~] = make_low_speed_two_pass_seed(initData, baseCase);
        twoPassSeed.source = 'two_pass';
        twoPassSeed.tilt_deg = target.tilt_deg;
        twoPassSeed.vinf_mps = target.vinf_mps;
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(twoPassSeed); %#ok<SAGROW>
    catch
    end
end

if target.tilt_deg <= 20 || target.vinf_mps <= 10
    hoverSeed = localSeedTemplate();
    hoverCase = localEvalTrimCaseFunc(config.hover_anchor_case_name);
    hoverSeed.name = 'hover_default';
    hoverSeed.source = 'hover_default';
    hoverSeed.tilt_deg = target.tilt_deg;
    hoverSeed.vinf_mps = target.vinf_mps;
    hoverSeed.front_collective_guess_rpm = hoverCase.front_collective_guess_rpm;
    hoverSeed.rear_collective_guess_rpm = hoverCase.rear_collective_guess_rpm;
    hoverSeed.theta_guess_deg = 0.0;
    hoverSeed.delta_f_guess_deg = 0.0;
    hoverSeed.delta_a_guess_deg = 0.0;
    hoverSeed.delta_e_guess_deg = 0.0;
    hoverSeed.delta_r_guess_deg = 0.0;
    seedCandidates(end + 1, 1) = hoverSeed; %#ok<SAGROW>
end

if target.tilt_deg >= 45 || target.vinf_mps >= 35
    cruiseSeed = localSeedTemplate();
    cruiseCase = localEvalTrimCaseFunc(config.cruise_anchor_case_name);
    cruiseSeed.name = 'cruise_default';
    cruiseSeed.source = 'cruise_default';
    cruiseSeed.tilt_deg = target.tilt_deg;
    cruiseSeed.vinf_mps = target.vinf_mps;
    cruiseSeed.front_collective_guess_rpm = cruiseCase.front_collective_guess_rpm;
    cruiseSeed.rear_collective_guess_rpm = localGetField(cruiseCase, ...
        'rear_collective_fixed_rpm', localGetField(cruiseCase, 'rear_collective_guess_rpm', 0.0));
    cruiseSeed.theta_guess_deg = localGetField(cruiseCase, 'theta_guess_deg', 0.0);
    cruiseSeed.delta_f_guess_deg = localGetField(cruiseCase, 'delta_f_guess_deg', 0.0);
    cruiseSeed.delta_a_guess_deg = 0.0;
    cruiseSeed.delta_e_guess_deg = localGetField(cruiseCase, 'delta_e_guess_deg', 0.0);
    cruiseSeed.delta_r_guess_deg = 0.0;
    seedCandidates(end + 1, 1) = cruiseSeed; %#ok<SAGROW>
end

seedCandidates = arrayfun(@(s) localApplySeedFloors(s, config), seedCandidates);
seedCandidates = localUniqueSeeds(seedCandidates);
end

function seedCandidates = localAppendInterpolatedMidbandSeeds(seedCandidates, target, currentGoodSeeds, historySeeds, config)
if target.vinf_mps < config.midband_vinf_range_mps(1) || target.vinf_mps > config.midband_vinf_range_mps(2)
    return;
end

seedPool = [currentGoodSeeds(:); historySeeds(:)];
if isempty(seedPool)
    return;
end

validMask = arrayfun(@(s) ...
    (localGetField(s, 'success', false) || localGetField(s, 'acceptable', false) || ...
     strcmp(localGetField(s, 'classification', ''), 'near_trim_borderline')) && ...
    abs(s.tilt_deg - target.tilt_deg) <= config.midband_interp_tilt_window_deg, ...
    seedPool);
seedPool = seedPool(validMask);
if isempty(seedPool)
    return;
end

lowerPool = seedPool([seedPool.vinf_mps] <= target.vinf_mps + 1e-9);
upperPool = seedPool([seedPool.vinf_mps] >= target.vinf_mps - 1e-9);
if isempty(lowerPool) || isempty(upperPool)
    return;
end

lowerDistances = arrayfun(@(s) localSeedDistance(s, target, config.seed_distance_weights), lowerPool);
upperDistances = arrayfun(@(s) localSeedDistance(s, target, config.seed_distance_weights), upperPool);
[~, lowerIdx] = min(lowerDistances);
[~, upperIdx] = min(upperDistances);
lowerSeed = localCanonicalizeSeed(lowerPool(lowerIdx));
upperSeed = localCanonicalizeSeed(upperPool(upperIdx));

if abs(upperSeed.vinf_mps - lowerSeed.vinf_mps) < 1e-9
    return;
end

baseFrac = (target.vinf_mps - lowerSeed.vinf_mps) / (upperSeed.vinf_mps - lowerSeed.vinf_mps);
blendFracs = unique(min(max([config.midband_interp_fracs(:)' baseFrac], 0.0), 1.0));

for iFrac = 1:numel(blendFracs)
    frac = blendFracs(iFrac);
    interpSeed = localBlendSeeds(lowerSeed, upperSeed, frac, target);
    interpSeed.name = sprintf('interp_%s_to_%s_f%s', ...
        localValueLabel(lowerSeed.vinf_mps), localValueLabel(upperSeed.vinf_mps), ...
        localValueLabel(100 * frac));
    interpSeed.source = 'midband_interpolated';
    seedCandidates(end + 1, 1) = interpSeed; %#ok<SAGROW>
end
end

function seed = localBlendSeeds(seedA, seedB, frac, target)
seed = localCanonicalizeSeed(seedA);
frac = min(max(frac, 0.0), 1.0);
blend = @(a, b) a + frac * (b - a);
seed.name = 'interpolated_seed';
seed.source = 'interpolated_seed';
seed.tilt_deg = target.tilt_deg;
seed.vinf_mps = target.vinf_mps;
seed.front_collective_guess_rpm = blend(seedA.front_collective_guess_rpm, seedB.front_collective_guess_rpm);
seed.rear_collective_guess_rpm = blend(seedA.rear_collective_guess_rpm, seedB.rear_collective_guess_rpm);
seed.theta_guess_deg = blend(seedA.theta_guess_deg, seedB.theta_guess_deg);
seed.delta_f_guess_deg = blend(seedA.delta_f_guess_deg, seedB.delta_f_guess_deg);
seed.delta_a_guess_deg = blend(seedA.delta_a_guess_deg, seedB.delta_a_guess_deg);
seed.delta_e_guess_deg = blend(seedA.delta_e_guess_deg, seedB.delta_e_guess_deg);
seed.delta_r_guess_deg = blend(seedA.delta_r_guess_deg, seedB.delta_r_guess_deg);
seed.success = false;
seed.acceptable = false;
seed.classification = '';
seed.score = inf;
end

function seed = localMakeCruiseNudgedSeed(parentSeed, target, config)
seed = localCanonicalizeSeed(parentSeed);
cruiseSeed = localGetCruiseAnchorSeed(config);

dt = max(target.tilt_deg - parentSeed.tilt_deg, 0.0);
dv = max(target.vinf_mps - parentSeed.vinf_mps, 0.0);
stepFraction = 0.05 + 0.05 * (dt / 5.0) + 0.03 * (dv / 2.5);
progressFraction = max(target.tilt_deg / max(config.cruise_anchor.tilt_deg, 1.0), ...
    target.vinf_mps / max(config.cruise_anchor.vinf_mps, 1.0));
gamma = min(config.max_prop_nudge_fraction, stepFraction * max(progressFraction, 0.25));

seed.name = ['nudged_' parentSeed.name];
seed.source = 'parent_nudged_to_cruise';
seed.front_collective_guess_rpm = parentSeed.front_collective_guess_rpm + ...
    gamma * (cruiseSeed.front_collective_guess_rpm - parentSeed.front_collective_guess_rpm);
seed.rear_collective_guess_rpm = parentSeed.rear_collective_guess_rpm + ...
    gamma * (cruiseSeed.rear_collective_guess_rpm - parentSeed.rear_collective_guess_rpm);
seed.theta_guess_deg = parentSeed.theta_guess_deg + ...
    gamma * (cruiseSeed.theta_guess_deg - parentSeed.theta_guess_deg);
seed.delta_f_guess_deg = parentSeed.delta_f_guess_deg + ...
    gamma * (cruiseSeed.delta_f_guess_deg - parentSeed.delta_f_guess_deg);
seed.delta_e_guess_deg = parentSeed.delta_e_guess_deg + ...
    gamma * (cruiseSeed.delta_e_guess_deg - parentSeed.delta_e_guess_deg);
seed.tilt_deg = target.tilt_deg;
seed.vinf_mps = target.vinf_mps;
end

function baseCase = localBaseSeedCase(target, config)
baseCase = localEvalTrimCaseFunc(config.cruise_anchor_case_name);
baseCase.name = target.name;
baseCase.mode = 'transition_path_scored';
baseCase.Vinf_mps = target.vinf_mps;
baseCase.u_body_mps = target.vinf_mps;
baseCase.v_body_mps = 0.0;
baseCase.w_body_mps = 0.0;
baseCase.front_tilt_deg = target.tilt_deg;
baseCase.front_tilt_cmd_deg = target.tilt_deg;
baseCase.use_vinf_output_constraint = true;
baseCase.use_vertical_speed_output_constraint = true;
baseCase.position_steady = [false; false; false];
baseCase.validate_nonlinear_hold = false;
baseCase.rear_collective_known = false;
baseCase.front_collective_min_rpm = config.front_collective_min_rpm;
baseCase.rear_collective_min_rpm = config.rear_collective_min_rpm;
cruiseSeed = localGetCruiseAnchorSeed(config);
baseCase.front_collective_guess_rpm = cruiseSeed.front_collective_guess_rpm;
baseCase.rear_collective_guess_rpm = cruiseSeed.rear_collective_guess_rpm;
baseCase.theta_guess_deg = cruiseSeed.theta_guess_deg;
baseCase.delta_f_guess_deg = cruiseSeed.delta_f_guess_deg;
baseCase.delta_e_guess_deg = cruiseSeed.delta_e_guess_deg;
end

function families = localBuildFamilies(target, seed, config)
families = repmat(localFamilyTemplate(), 0, 1);

if any(strcmp(config.enabled_families, 'front_rear_free_flap_elevator'))
    families(end + 1, 1) = localMakeFrontRearFreeFlapElevatorCase(target, seed, config); %#ok<SAGROW>
end
if any(strcmp(config.enabled_families, 'prop_fixed_flap_elevator'))
    families(end + 1, 1) = localMakePropFixedFlapElevatorCase(target, seed, config); %#ok<SAGROW>
end
if target.tilt_deg >= 30 && any(strcmp(config.enabled_families, 'rear_fixed_flap_elevator'))
    families(end + 1, 1) = localMakeRearFixedFlapElevatorCase(target, seed, config); %#ok<SAGROW>
end
if target.tilt_deg <= 20 && target.vinf_mps <= 15 && any(strcmp(config.enabled_families, 'hover_zero_surface'))
    families(end + 1, 1) = localMakeHoverZeroSurfaceCase(target, seed, config); %#ok<SAGROW>
end
end

function family = localMakeHoverZeroSurfaceCase(target, seed, config)
trimCase = localEvalTrimCaseFunc(config.hover_anchor_case_name);
trimCase.name = sprintf('%s_hoverZero_%s', target.name, seed.name);
trimCase.mode = 'transition_path_scored';
trimCase.Vinf_mps = target.vinf_mps;
trimCase.u_body_mps = target.vinf_mps;
trimCase.v_body_mps = 0.0;
trimCase.w_body_mps = 0.0;
trimCase.front_tilt_deg = target.tilt_deg;
trimCase.front_tilt_cmd_deg = target.tilt_deg;
trimCase.euler_known = [true; false; true];
trimCase.euler_steady = [true; true; true];
trimCase.body_velocity_known = [false; true; false];
trimCase.body_velocity_steady = [true; true; true];
trimCase.position_steady = [false; false; false];
trimCase.use_vinf_output_constraint = true;
trimCase.use_vertical_speed_output_constraint = true;
trimCase.front_collective_known = false;
trimCase.rear_collective_known = false;
trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
trimCase.front_collective_min_rpm = config.front_collective_min_rpm;
trimCase.rear_collective_min_rpm = config.rear_collective_min_rpm;
trimCase.theta_guess_deg = seed.theta_guess_deg;
trimCase.mixed_control_known = [true; true; true; true];
trimCase.delta_f_fixed_deg = 0.0;
trimCase.delta_a_fixed_deg = 0.0;
trimCase.delta_e_fixed_deg = 0.0;
trimCase.delta_r_fixed_deg = 0.0;
trimCase.validate_nonlinear_hold = false;

family = localFamilyTemplate();
family.name = 'hover_zero_surface';
family.seed_name = seed.name;
family.trimCase = trimCase;
end

function family = localMakeFrontRearFreeFlapElevatorCase(target, seed, config)
trimCase = localBaseSeedCase(target, config);
trimCase.name = sprintf('%s_frontRearFree_%s', target.name, seed.name);
trimCase.front_collective_known = false;
trimCase.rear_collective_known = false;
trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
trimCase.theta_guess_deg = seed.theta_guess_deg;
trimCase.delta_f_guess_deg = seed.delta_f_guess_deg;
trimCase.delta_e_guess_deg = seed.delta_e_guess_deg;
trimCase.delta_a_fixed_deg = 0.0;
trimCase.delta_r_fixed_deg = 0.0;
trimCase.mixed_control_known = [false; true; false; true];

family = localFamilyTemplate();
family.name = 'front_rear_free_flap_elevator';
family.seed_name = seed.name;
family.trimCase = trimCase;
end

function family = localMakeRearFixedFlapElevatorCase(target, seed, config)
trimCase = localBaseSeedCase(target, config);
trimCase.name = sprintf('%s_rearFixed_%s', target.name, seed.name);
trimCase.front_collective_known = false;
trimCase.rear_collective_known = true;
trimCase.rear_collective_fixed_rpm = seed.rear_collective_guess_rpm;
trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
trimCase.theta_guess_deg = seed.theta_guess_deg;
trimCase.delta_f_guess_deg = seed.delta_f_guess_deg;
trimCase.delta_e_guess_deg = seed.delta_e_guess_deg;
trimCase.delta_a_fixed_deg = 0.0;
trimCase.delta_r_fixed_deg = 0.0;
trimCase.mixed_control_known = [false; true; false; true];

family = localFamilyTemplate();
family.name = 'rear_fixed_flap_elevator';
family.seed_name = seed.name;
family.trimCase = trimCase;
end

function family = localMakePropFixedFlapElevatorCase(target, seed, config)
trimCase = localBaseSeedCase(target, config);
trimCase.name = sprintf('%s_propFixed_%s', target.name, seed.name);
trimCase.front_collective_known = true;
trimCase.rear_collective_known = true;
trimCase.front_collective_fixed_rpm = seed.front_collective_guess_rpm;
trimCase.rear_collective_fixed_rpm = seed.rear_collective_guess_rpm;
trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
trimCase.theta_guess_deg = seed.theta_guess_deg;
trimCase.delta_f_guess_deg = seed.delta_f_guess_deg;
trimCase.delta_e_guess_deg = seed.delta_e_guess_deg;
trimCase.delta_a_fixed_deg = 0.0;
trimCase.delta_r_fixed_deg = 0.0;
trimCase.mixed_control_known = [false; true; false; true];

family = localFamilyTemplate();
family.name = 'prop_fixed_flap_elevator';
family.seed_name = seed.name;
family.trimCase = trimCase;
end

function attempt = localMakeAttempt(target, family, trimResult, scoreData, config)
attempt = localAttemptTemplate();
attempt.target_key = localTargetKey(target);
attempt.family = family.name;
attempt.seed_name = family.seed_name;
attempt.trimCase = family.trimCase;
attempt.trimResult = trimResult;
attempt.scoreData = scoreData;
attempt.success = localGetField(trimResult, 'success', false);
attempt.acceptable = localGetField(scoreData, 'acceptable', false);
attempt.classification = localGetField(scoreData, 'classification', 'not_scored');
attempt.score = localGetField(scoreData, 'score', inf);
attempt.max_normalized = localGetField(scoreData, 'max_normalized', inf);
attempt.worst_component = localGetField(localGetField(scoreData, 'worst_component', struct()), 'name', '');
attempt.worst_component_normalized = localGetField(localGetField(scoreData, 'worst_component', struct()), 'normalized', inf);
attempt.termination_string = localGetField(trimResult, 'terminationString', '');

    if isstruct(trimResult) && isfield(trimResult, 'scheduling')
        attempt.front_collective_rpm = localGetField(trimResult.scheduling, 'front_collective_rpm', NaN);
        attempt.rear_collective_rpm = localGetField(trimResult.scheduling, 'rear_collective_rpm', NaN);
        attempt.delta_f_deg = rad2deg(localGetField(trimResult.scheduling, 'delta_f_rad', NaN));
        attempt.delta_a_deg = rad2deg(localGetField(trimResult.scheduling, 'delta_a_rad', NaN));
        attempt.delta_e_deg = rad2deg(localGetField(trimResult.scheduling, 'delta_e_rad', NaN));
        attempt.delta_r_deg = rad2deg(localGetField(trimResult.scheduling, 'delta_r_rad', NaN));
        attempt.alpha_deg = rad2deg(localGetField(trimResult.scheduling, 'alpha_rad', NaN));
    end

    if isstruct(trimResult) && isfield(trimResult, 'Att_Trim_deg') && numel(trimResult.Att_Trim_deg) >= 2
        attempt.theta_deg = trimResult.Att_Trim_deg(2);
    end
    if isstruct(trimResult) && isfield(trimResult, 'Vel_B_BA_Trim') && numel(trimResult.Vel_B_BA_Trim) >= 3
        attempt.u_mps = trimResult.Vel_B_BA_Trim(1);
        attempt.w_mps = trimResult.Vel_B_BA_Trim(3);
    end

if isfinite(attempt.rear_collective_rpm) && ...
        attempt.rear_collective_rpm < (config.rear_collective_min_rpm - 1e-6)
    attempt.success = false;
    attempt.acceptable = false;
    attempt.classification = 'rear_below_floor';
    attempt.score = inf;
    attempt.max_normalized = inf;
    attempt.worst_component = 'rear_collective_floor';
    attempt.worst_component_normalized = inf;
end
end

function tf = localIsBetterAttempt(candidate, incumbent)
if isempty(incumbent.family)
    tf = true;
    return;
end

candidateRank = localAttemptRank(candidate);
incumbentRank = localAttemptRank(incumbent);
if candidateRank ~= incumbentRank
    tf = candidateRank < incumbentRank;
    return;
end

if candidate.score ~= incumbent.score
    tf = candidate.score < incumbent.score;
    return;
end

tf = candidate.max_normalized < incumbent.max_normalized;
end

function rank = localAttemptRank(attempt)
if attempt.success
    rank = 0;
elseif attempt.acceptable
    rank = 1;
elseif strcmp(attempt.classification, 'near_trim_borderline')
    rank = 2;
else
    rank = 3;
end
end

function [summary, mapStruct] = localCheckpoint(mapStruct, checkpointFile, latestMat, summaryCsv, latestCsv, summaryMd, latestMd)
summary = localBuildSummary(mapStruct.entries);
mapStruct.summary_table = summary;

save(checkpointFile, 'mapStruct', 'summary', '-v7.3');
copyfile(checkpointFile, latestMat, 'f');
if ~isempty(summary)
    writetable(summary, summaryCsv);
    writetable(summary, latestCsv);
end
localWriteSummaryMarkdown(summary, summaryMd);
localWriteSummaryMarkdown(summary, latestMd);
end

function [indexRows, keyToIndex] = localLoadExistingLinearizationIndex(indexMatFile)
indexRows = repmat(localLinearizationIndexRowTemplate(), 0, 1);
keyToIndex = containers.Map('KeyType', 'char', 'ValueType', 'double');
if exist(indexMatFile, 'file') ~= 2
    return;
end
try
    S = load(indexMatFile, 'linearizationIndex');
    if ~isfield(S, 'linearizationIndex') || isempty(S.linearizationIndex)
        return;
    end
    loadedIndex = S.linearizationIndex;
    if istable(loadedIndex)
        loadedIndex = localLinearizationIndexStructFromTable(loadedIndex);
    end
    if ~isstruct(loadedIndex)
        return;
    end
    indexRows = loadedIndex(:);
    for i = 1:numel(indexRows)
        key = localGetField(indexRows(i), 'key', '');
        if strlength(string(key)) == 0
            continue;
        end
        keyToIndex(char(key)) = i;
    end
catch
    indexRows = repmat(localLinearizationIndexRowTemplate(), 0, 1);
    keyToIndex = containers.Map('KeyType', 'char', 'ValueType', 'double');
end
end

function [indexRows, keyToIndex] = localSaveUsableLinearization(entry, indexRows, keyToIndex, ...
    outputDir, latestDir, outputIndexMat, latestIndexMat, outputIndexCsv, latestIndexCsv)
if ~localShouldSaveLinearization(entry)
    return;
end

linearizationPoint = localBuildLinearizationPoint(entry);
if isempty(fieldnames(linearizationPoint))
    return;
end

fileStem = ['linearization_' localSanitizeFileLabel(entry.key)];
outputFile = fullfile(outputDir, [fileStem '.mat']);
latestFile = fullfile(latestDir, [fileStem '.mat']);
save(outputFile, 'linearizationPoint', '-v7.3');
save(latestFile, 'linearizationPoint', '-v7.3');

row = localLinearizationIndexRowTemplate();
row.key = entry.key;
row.name = entry.name;
row.tilt_deg = entry.tilt_deg;
row.vinf_mps = entry.vinf_mps;
row.family = entry.family;
row.seed_name = entry.seed_name;
row.success = entry.success;
row.acceptable = entry.acceptable;
row.classification = entry.classification;
row.score = entry.score;
row.max_normalized = entry.max_normalized;
row.front_collective_rpm = entry.front_collective_rpm;
row.rear_collective_rpm = entry.rear_collective_rpm;
row.delta_f_deg = entry.delta_f_deg;
row.delta_a_deg = entry.delta_a_deg;
row.delta_e_deg = entry.delta_e_deg;
row.delta_r_deg = entry.delta_r_deg;
row.theta_deg = entry.theta_deg;
row.alpha_deg = entry.alpha_deg;
row.output_file = outputFile;
row.latest_file = latestFile;

entryKey = char(string(entry.key));
if isKey(keyToIndex, entryKey)
    existingIdx = keyToIndex(entryKey);
    if localIsBetterLinearizationRow(row, indexRows(existingIdx))
        indexRows(existingIdx) = row;
    end
else
    indexRows(end + 1, 1) = row; %#ok<SAGROW>
    keyToIndex(entryKey) = numel(indexRows);
end

indexTable = localBuildLinearizationIndexTable(indexRows);
linearizationIndex = indexRows; %#ok<NASGU>
save(outputIndexMat, 'linearizationIndex', 'indexTable', '-v7.3');
save(latestIndexMat, 'linearizationIndex', 'indexTable', '-v7.3');
writetable(indexTable, outputIndexCsv);
writetable(indexTable, latestIndexCsv);
end

function tf = localShouldSaveLinearization(entry)
tf = entry.success || entry.acceptable || strcmp(entry.classification, 'near_trim_borderline');
end

function tf = localIsBetterLinearizationRow(candidate, incumbent)
candidateRank = localLinearizationRank(candidate);
incumbentRank = localLinearizationRank(incumbent);
if candidateRank ~= incumbentRank
    tf = candidateRank < incumbentRank;
    return;
end
if candidate.score ~= incumbent.score
    tf = candidate.score < incumbent.score;
    return;
end
tf = candidate.max_normalized < incumbent.max_normalized;
end

function rank = localLinearizationRank(row)
if row.success
    rank = 0;
elseif row.acceptable
    rank = 1;
elseif strcmp(row.classification, 'near_trim_borderline')
    rank = 2;
else
    rank = 3;
end
end

function linearizationPoint = localBuildLinearizationPoint(entry)
linearizationPoint = struct();
trimResult = localGetField(entry, 'trimResult', struct());
linearData = localGetField(trimResult, 'linear', struct());
if isempty(fieldnames(linearData))
    return;
end

linearizationPoint.name = entry.name;
linearizationPoint.key = entry.key;
linearizationPoint.family = entry.family;
linearizationPoint.seed_name = entry.seed_name;
linearizationPoint.success = entry.success;
linearizationPoint.acceptable = entry.acceptable;
linearizationPoint.classification = entry.classification;
linearizationPoint.score = entry.score;
linearizationPoint.max_normalized = entry.max_normalized;
linearizationPoint.worst_component = entry.worst_component;
linearizationPoint.worst_component_normalized = entry.worst_component_normalized;
linearizationPoint.termination_string = entry.termination_string;
linearizationPoint.tilt_deg = entry.tilt_deg;
linearizationPoint.vinf_mps = entry.vinf_mps;
linearizationPoint.theta_deg = entry.theta_deg;
linearizationPoint.alpha_deg = entry.alpha_deg;
linearizationPoint.u_mps = entry.u_mps;
linearizationPoint.w_mps = entry.w_mps;
linearizationPoint.front_collective_rpm = entry.front_collective_rpm;
linearizationPoint.rear_collective_rpm = entry.rear_collective_rpm;
linearizationPoint.delta_f_deg = entry.delta_f_deg;
linearizationPoint.delta_a_deg = entry.delta_a_deg;
linearizationPoint.delta_e_deg = entry.delta_e_deg;
linearizationPoint.delta_r_deg = entry.delta_r_deg;
linearizationPoint.trimCase = localGetField(entry, 'trimCase', struct());
linearizationPoint.scheduling = localGetField(trimResult, 'scheduling', struct());
linearizationPoint.Att_Trim_deg = localGetField(trimResult, 'Att_Trim_deg', []);
linearizationPoint.Vel_B_BA_Trim = localGetField(trimResult, 'Vel_B_BA_Trim', []);
linearizationPoint.scoreData = localGetField(entry, 'scoreData', struct());
linearizationPoint.linear = localExtractLinearData(linearData);
end

function linear = localExtractLinearData(linearData)
linear = struct();
linear.reduced_model_available = localGetField(linearData, 'reduced_model_available', false);
linear.A_full = localGetField(linearData, 'A_full', []);
linear.B_full = localGetField(linearData, 'B_full', []);
linear.C_full = localGetField(linearData, 'C_full', []);
linear.D_full = localGetField(linearData, 'D_full', []);
linear.B_front_collective = localGetField(linearData, 'B_front_collective', []);
linear.B_rear_collective = localGetField(linearData, 'B_rear_collective', []);

sys_full = localGetField(linearData, 'sys_full', []);
if ~isempty(sys_full)
    linear.full_state_names = string(sys_full.StateName(:));
    linear.full_input_names = string(sys_full.InputName(:));
    linear.full_output_names = string(sys_full.OutputName(:));
else
    linear.full_state_names = strings(0, 1);
    linear.full_input_names = strings(0, 1);
    linear.full_output_names = strings(0, 1);
end

sys9 = localGetField(linearData, 'sys_ss_9state', []);
if ~isempty(sys9)
    linear.A_9 = sys9.A;
    linear.B_9 = sys9.B;
    linear.C_9 = sys9.C;
    linear.D_9 = sys9.D;
    linear.state_names_9 = string(sys9.StateName(:));
    linear.input_names_9 = string(sys9.InputName(:));
    linear.output_names_9 = string(sys9.OutputName(:));
    linear.eigenvalues_9 = eig(sys9.A);
else
    linear.A_9 = [];
    linear.B_9 = [];
    linear.C_9 = [];
    linear.D_9 = [];
    linear.state_names_9 = strings(0, 1);
    linear.input_names_9 = strings(0, 1);
    linear.output_names_9 = strings(0, 1);
    linear.eigenvalues_9 = [];
end
end

function indexTable = localBuildLinearizationIndexTable(indexRows)
if isempty(indexRows)
    indexTable = table();
    return;
end
indexTable = table();
indexTable.key = string({indexRows.key}');
indexTable.name = string({indexRows.name}');
indexTable.tilt_deg = [indexRows.tilt_deg]';
indexTable.vinf_mps = [indexRows.vinf_mps]';
indexTable.family = string({indexRows.family}');
indexTable.seed_name = string({indexRows.seed_name}');
indexTable.success = logical([indexRows.success]');
indexTable.acceptable = logical([indexRows.acceptable]');
indexTable.classification = string({indexRows.classification}');
indexTable.score = [indexRows.score]';
indexTable.max_normalized = [indexRows.max_normalized]';
indexTable.front_collective_rpm = [indexRows.front_collective_rpm]';
indexTable.rear_collective_rpm = [indexRows.rear_collective_rpm]';
indexTable.delta_f_deg = [indexRows.delta_f_deg]';
indexTable.delta_a_deg = [indexRows.delta_a_deg]';
indexTable.delta_e_deg = [indexRows.delta_e_deg]';
indexTable.delta_r_deg = [indexRows.delta_r_deg]';
indexTable.theta_deg = [indexRows.theta_deg]';
indexTable.alpha_deg = [indexRows.alpha_deg]';
indexTable.output_file = string({indexRows.output_file}');
indexTable.latest_file = string({indexRows.latest_file}');
end

function indexRows = localLinearizationIndexStructFromTable(indexTable)
indexRows = repmat(localLinearizationIndexRowTemplate(), 0, 1);
for i = 1:height(indexTable)
    row = localLinearizationIndexRowTemplate();
    row.key = char(indexTable.key(i));
    row.name = char(indexTable.name(i));
    row.tilt_deg = indexTable.tilt_deg(i);
    row.vinf_mps = indexTable.vinf_mps(i);
    row.family = char(indexTable.family(i));
    row.seed_name = char(indexTable.seed_name(i));
    row.success = indexTable.success(i);
    row.acceptable = indexTable.acceptable(i);
    row.classification = char(indexTable.classification(i));
    row.score = indexTable.score(i);
    row.max_normalized = indexTable.max_normalized(i);
    row.front_collective_rpm = indexTable.front_collective_rpm(i);
    row.rear_collective_rpm = indexTable.rear_collective_rpm(i);
    row.delta_f_deg = indexTable.delta_f_deg(i);
    row.delta_a_deg = indexTable.delta_a_deg(i);
    row.delta_e_deg = indexTable.delta_e_deg(i);
    row.delta_r_deg = indexTable.delta_r_deg(i);
    row.theta_deg = indexTable.theta_deg(i);
    row.alpha_deg = indexTable.alpha_deg(i);
    row.output_file = char(indexTable.output_file(i));
    row.latest_file = char(indexTable.latest_file(i));
    indexRows(end + 1, 1) = row; %#ok<SAGROW>
end
end

function row = localLinearizationIndexRowTemplate()
row = struct( ...
    'key', '', ...
    'name', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'family', '', ...
    'seed_name', '', ...
    'success', false, ...
    'acceptable', false, ...
    'classification', '', ...
    'score', inf, ...
    'max_normalized', inf, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'alpha_deg', NaN, ...
    'output_file', '', ...
    'latest_file', '');
end

function label = localSanitizeFileLabel(label)
label = regexprep(char(string(label)), '[^A-Za-z0-9_\-]', '_');
end

function summary = localBuildSummary(entries)
if isempty(entries)
    summary = table();
    return;
end

summary = table();
summary.name = string({entries.name}');
summary.tilt_deg = [entries.tilt_deg]';
summary.vinf_mps = [entries.vinf_mps]';
summary.family = string({entries.family}');
summary.seed_name = string({entries.seed_name}');
summary.success = logical([entries.success]');
summary.acceptable = logical([entries.acceptable]');
summary.classification = string({entries.classification}');
summary.score = [entries.score]';
summary.max_normalized = [entries.max_normalized]';
summary.worst_component = string({entries.worst_component}');
summary.worst_component_normalized = [entries.worst_component_normalized]';
summary.front_collective_rpm = [entries.front_collective_rpm]';
summary.rear_collective_rpm = [entries.rear_collective_rpm]';
summary.delta_f_deg = [entries.delta_f_deg]';
summary.delta_a_deg = [entries.delta_a_deg]';
summary.delta_e_deg = [entries.delta_e_deg]';
summary.delta_r_deg = [entries.delta_r_deg]';
summary.theta_deg = [entries.theta_deg]';
summary.u_mps = [entries.u_mps]';
summary.w_mps = [entries.w_mps]';
summary.alpha_deg = [entries.alpha_deg]';
summary.termination_string = string({entries.termination_string}');
end

function localWriteSummaryMarkdown(summary, filename)
fid = fopen(filename, 'w');
if fid < 0
    warning('Run_Trim_Transition_Reference_Line_Scored:WriteMarkdownFailed', ...
        'Could not write %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Reference Line Scored Trim Sweep\n\n');
fprintf(fid, '- Total points: %d\n', height(summary));
if ~isempty(summary)
    fprintf(fid, '- Exact trims: %d\n', nnz(summary.success));
    fprintf(fid, '- Acceptable scored near-trims: %d\n', nnz(summary.acceptable));
    fprintf(fid, '- Best score: %.4f\n\n', min(summary.score));

    fprintf(fid, '## Top 15 Candidates\n\n');
    fprintf(fid, '| Name | Tilt (deg) | V (m/s) | Family | Success | Acceptable | Score | Class |\n');
    fprintf(fid, '| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |\n');
    [~, order] = sortrows([~summary.success, ~summary.acceptable, summary.score], [1 2 3]);
    topIdx = order(1:min(15, numel(order)));
    for i = 1:numel(topIdx)
        row = summary(topIdx(i), :);
        fprintf(fid, '| %s | %.1f | %.1f | %s | %d | %d | %.4f | %s |\n', ...
            row.name{1}, row.tilt_deg, row.vinf_mps, row.family{1}, ...
            row.success, row.acceptable, row.score, row.classification{1});
    end
end
end

function seed = localSeedFromEntry(entry)
seed = localSeedTemplate();
seed.name = ['entry_' entry.name];
seed.source = 'current_sweep';
seed.tilt_deg = entry.tilt_deg;
seed.vinf_mps = entry.vinf_mps;
seed.front_collective_guess_rpm = entry.front_collective_rpm;
seed.rear_collective_guess_rpm = entry.rear_collective_rpm;
seed.theta_guess_deg = entry.theta_deg;
seed.delta_f_guess_deg = entry.delta_f_deg;
seed.delta_a_guess_deg = entry.delta_a_deg;
seed.delta_e_guess_deg = entry.delta_e_deg;
seed.delta_r_guess_deg = entry.delta_r_deg;
seed.success = entry.success;
seed.acceptable = entry.acceptable;
seed.classification = entry.classification;
seed.score = entry.score;
end

function distance = localEntryDistance(entry, target, weights)
distance = weights.tilt * abs(entry.tilt_deg - target.tilt_deg) + ...
           weights.vinf * abs(entry.vinf_mps - target.vinf_mps);
end

function distance = localSeedDistance(seed, target, weights)
distance = weights.tilt * abs(seed.tilt_deg - target.tilt_deg) + ...
           weights.vinf * abs(seed.vinf_mps - target.vinf_mps);
end

function seedCandidates = localAppendClosestHistorySeeds(seedCandidates, historySeeds, target, weights, count)
if isempty(historySeeds) || count <= 0
    return;
end

distances = arrayfun(@(s) localSeedDistance(s, target, weights), historySeeds);
[~, order] = sort(distances, 'ascend');
order = order(1:min(numel(order), count));
for i = 1:numel(order)
    seedCandidates(end + 1, 1) = localCanonicalizeSeed(historySeeds(order(i))); %#ok<SAGROW>
end
end

function knownGoodPoints = localLoadKnownGoodPoints(exactCsv, scoredCsv)
knownGoodPoints = localLoadKnownGoodPointsFromLists(exactCsv, scoredCsv);
end

function knownGoodPoints = localLoadKnownGoodPointsFromLists(exactCsvs, scoredCsvs)
knownGoodPoints = zeros(0, 2);
exactCsvs = localNormalizeCsvList(exactCsvs);
scoredCsvs = localNormalizeCsvList(scoredCsvs);
for i = 1:numel(exactCsvs)
    knownGoodPoints = [knownGoodPoints; localLoadKnownGoodPointsFromCsv(exactCsvs{i}, 'success_only')]; %#ok<AGROW>
end
for i = 1:numel(scoredCsvs)
    knownGoodPoints = [knownGoodPoints; localLoadKnownGoodPointsFromCsv(scoredCsvs{i}, 'acceptable_or_better')]; %#ok<AGROW>
end
if isempty(knownGoodPoints)
    return;
end
knownGoodPoints = unique(round(knownGoodPoints, 6), 'rows');
end

function points = localLoadKnownGoodPointsFromCsv(filename, mode)
points = zeros(0, 2);
if exist(filename, 'file') ~= 2
    return;
end

summary = readtable(filename);
requiredVars = {'tilt_deg', 'vinf_mps'};
if ~all(ismember(requiredVars, summary.Properties.VariableNames))
    return;
end

switch mode
    case 'success_only'
        if ~ismember('success', summary.Properties.VariableNames)
            return;
        end
        keepMask = localTableLogical(summary.success);
    case 'acceptable_or_better'
        if ismember('acceptable', summary.Properties.VariableNames)
            keepMask = localTableLogical(summary.acceptable);
        elseif ismember('success', summary.Properties.VariableNames)
            keepMask = localTableLogical(summary.success);
        else
            return;
        end
    otherwise
        keepMask = true(height(summary), 1);
end

summary = summary(keepMask, :);
if isempty(summary)
    return;
end
points = [summary.tilt_deg, summary.vinf_mps];
end

function points = localLoadRejectedPointsFromCsv(filename)
points = zeros(0, 2);
if exist(filename, 'file') ~= 2
    return;
end

summary = readtable(filename);
requiredVars = {'tilt_deg', 'vinf_mps'};
if ~all(ismember(requiredVars, summary.Properties.VariableNames))
    return;
end

if ismember('success', summary.Properties.VariableNames)
    successMask = localTableLogical(summary.success);
else
    successMask = false(height(summary), 1);
end
if ismember('acceptable', summary.Properties.VariableNames)
    acceptableMask = localTableLogical(summary.acceptable);
else
    acceptableMask = false(height(summary), 1);
end

rejectMask = ~(successMask | acceptableMask);
if ismember('classification', summary.Properties.VariableNames)
    rejectMask = rejectMask & strcmpi(string(summary.classification), "not_usable");
end

summary = summary(rejectMask, :);
if isempty(summary)
    return;
end
points = [summary.tilt_deg, summary.vinf_mps];
points = unique(round(points, 6), 'rows');
end

function points = localLoadRejectedPointsFromCsvList(filenames)
points = zeros(0, 2);
filenames = localNormalizeCsvList(filenames);
for i = 1:numel(filenames)
    points = [points; localLoadRejectedPointsFromCsv(filenames{i})]; %#ok<AGROW>
end
if isempty(points)
    return;
end
points = unique(round(points, 6), 'rows');
end

function startPoints = localNormalizeStartPoints(rawStartPoints)
startPoints = repmat(struct('tilt_deg', NaN, 'vinf_mps', NaN), 0, 1);
if isempty(rawStartPoints)
    return;
end

if isstruct(rawStartPoints)
    for i = 1:numel(rawStartPoints)
        if isfield(rawStartPoints, 'tilt_deg') && isfield(rawStartPoints, 'vinf_mps')
            startPoints(end + 1, 1) = struct( ... %#ok<SAGROW>
                'tilt_deg', rawStartPoints(i).tilt_deg, ...
                'vinf_mps', rawStartPoints(i).vinf_mps);
        end
    end
elseif isnumeric(rawStartPoints) && size(rawStartPoints, 2) == 2
    for i = 1:size(rawStartPoints, 1)
        startPoints(end + 1, 1) = struct( ... %#ok<SAGROW>
            'tilt_deg', rawStartPoints(i, 1), ...
            'vinf_mps', rawStartPoints(i, 2));
    end
end
end

function keyMap = localKnownGoodKeyMap(points)
keyMap = containers.Map('KeyType', 'char', 'ValueType', 'logical');
for i = 1:size(points, 1)
    keyMap(localKnownPointKey(points(i, 1), points(i, 2))) = true;
end
end

function key = localKnownPointKey(tilt_deg, vinf_mps)
key = sprintf('tilt_%s_v_%s', localValueLabel(tilt_deg), localValueLabel(vinf_mps));
end

function distance = localNearestKnownPointDistance(tilt_deg, vinf_mps, knownGoodPoints, weights)
if isempty(knownGoodPoints)
    distance = 0.0;
    return;
end

distances = weights.tilt * abs(knownGoodPoints(:, 1) - tilt_deg) + ...
    weights.vinf * abs(knownGoodPoints(:, 2) - vinf_mps);
distance = min(distances);
end

function seed = localCanonicalizeSeed(seed)
seed = localSeedTemplate(seed);
seed.delta_f_guess_deg = localGetField(seed, 'delta_f_guess_deg', 0.0);
seed.delta_a_guess_deg = localGetField(seed, 'delta_a_guess_deg', 0.0);
seed.delta_e_guess_deg = localGetField(seed, 'delta_e_guess_deg', 0.0);
seed.delta_r_guess_deg = localGetField(seed, 'delta_r_guess_deg', 0.0);
end

function csvList = localNormalizeCsvList(rawValue)
csvList = {};
if isempty(rawValue)
    return;
end
if ischar(rawValue) || isstring(rawValue)
    csvList = {char(rawValue)};
elseif iscell(rawValue)
    for i = 1:numel(rawValue)
        item = rawValue{i};
        if ischar(item) || isstring(item)
            csvList{end + 1} = char(item); %#ok<AGROW>
        end
    end
end
if isempty(csvList)
    return;
end
seen = strings(0, 1);
normalized = {};
for i = 1:numel(csvList)
    item = string(csvList{i});
    if strlength(item) == 0 || any(seen == item)
        continue;
    end
    seen(end + 1, 1) = item; %#ok<AGROW>
    normalized{end + 1} = char(item); %#ok<AGROW>
end
csvList = normalized;
end

function seed = localNormalizeCruiseAnchorSeed(rawSeed)
seed = localSeedTemplate();
seed.is_valid = false;
if ~isstruct(rawSeed) || isempty(fieldnames(rawSeed))
    return;
end
seed = localSeedTemplate(rawSeed);
seed = localCanonicalizeSeed(seed);
required = [ ...
    isfinite(seed.tilt_deg), ...
    isfinite(seed.vinf_mps), ...
    isfinite(seed.front_collective_guess_rpm), ...
    isfinite(seed.rear_collective_guess_rpm)];
seed.is_valid = all(required);
end

function seed = localGetCruiseAnchorSeed(config)
if isfield(config, 'cruise_anchor_seed') && isstruct(config.cruise_anchor_seed) && ...
        localGetField(config.cruise_anchor_seed, 'is_valid', false)
    seed = localCanonicalizeSeed(config.cruise_anchor_seed);
    return;
end

trimCase = localEvalTrimCaseFunc(config.cruise_anchor_case_name);
seed = localSeedTemplate();
seed.name = 'cruise_anchor_case';
seed.source = 'cruise_anchor_case';
seed.tilt_deg = localGetField(trimCase, 'front_tilt_deg', 90.0);
seed.vinf_mps = localGetField(trimCase, 'Vinf_mps', 70.0);
seed.front_collective_guess_rpm = localGetField(trimCase, 'front_collective_guess_rpm', NaN);
seed.rear_collective_guess_rpm = localGetField(trimCase, 'rear_collective_fixed_rpm', ...
    localGetField(trimCase, 'rear_collective_guess_rpm', NaN));
seed.theta_guess_deg = localGetField(trimCase, 'theta_guess_deg', 0.0);
seed.delta_f_guess_deg = localGetField(trimCase, 'delta_f_guess_deg', 0.0);
seed.delta_a_guess_deg = localGetField(trimCase, 'delta_a_guess_deg', 0.0);
seed.delta_e_guess_deg = localGetField(trimCase, 'delta_e_guess_deg', 0.0);
seed.delta_r_guess_deg = localGetField(trimCase, 'delta_r_guess_deg', 0.0);
seed.is_valid = true;
seed = localCanonicalizeSeed(seed);
end

function seed = localApplySeedFloors(seed, config)
seed = localCanonicalizeSeed(seed);
if isfinite(seed.front_collective_guess_rpm)
    seed.front_collective_guess_rpm = max(seed.front_collective_guess_rpm, config.front_collective_min_rpm);
end
if isfinite(seed.rear_collective_guess_rpm)
    seed.rear_collective_guess_rpm = max(seed.rear_collective_guess_rpm, config.rear_collective_min_rpm);
end
end

function trimCase = localEvalTrimCaseFunc(caseFuncName)
trimCase = feval(caseFuncName);
end

function uniqueSeeds = localUniqueSeeds(seedCandidates)
uniqueSeeds = repmat(localSeedTemplate(), 0, 1);
keys = strings(0, 1);
for i = 1:numel(seedCandidates)
    seed = localCanonicalizeSeed(seedCandidates(i));
    key = sprintf('%.1f|%.1f|%.3f|%.3f|%.3f|%.3f|%.3f', ...
        seed.front_collective_guess_rpm, seed.rear_collective_guess_rpm, ...
        seed.theta_guess_deg, seed.delta_f_guess_deg, seed.delta_a_guess_deg, ...
        seed.delta_e_guess_deg, seed.delta_r_guess_deg);
    if any(keys == key)
        continue;
    end
    keys(end + 1, 1) = string(key); %#ok<SAGROW>
    uniqueSeeds(end + 1, 1) = seed; %#ok<SAGROW>
end
end

function scoreData = localFallbackScore(trimResult)
scoreData = struct();
scoreData.score = inf;
scoreData.max_normalized = inf;
scoreData.acceptable = false;
scoreData.classification = 'not_scored';
scoreData.worst_component = struct('name', '', 'normalized', inf);
scoreData.termination_string = localGetField(trimResult, 'terminationString', '');
end

function target = localTargetTemplate()
target = struct( ...
    'name', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'parent_name', '', ...
    'parent_seed', struct(), ...
    'sort_key', inf);
end

function family = localFamilyTemplate()
family = struct( ...
    'name', '', ...
    'seed_name', '', ...
    'trimCase', struct());
end

function attempt = localAttemptTemplate()
attempt = struct( ...
    'target_key', '', ...
    'family', '', ...
    'seed_name', '', ...
    'trimCase', struct(), ...
    'trimResult', struct(), ...
    'scoreData', struct(), ...
    'success', false, ...
    'acceptable', false, ...
    'classification', '', ...
    'score', inf, ...
    'max_normalized', inf, ...
    'worst_component', '', ...
    'worst_component_normalized', inf, ...
    'termination_string', '', ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'u_mps', NaN, ...
    'w_mps', NaN, ...
    'alpha_deg', NaN);
end

function entry = localEntryTemplate()
entry = struct( ...
    'key', '', ...
    'name', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'family', '', ...
    'seed_name', '', ...
    'success', false, ...
    'acceptable', false, ...
    'classification', '', ...
    'score', inf, ...
    'max_normalized', inf, ...
    'worst_component', '', ...
    'worst_component_normalized', inf, ...
    'termination_string', '', ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'u_mps', NaN, ...
    'w_mps', NaN, ...
    'alpha_deg', NaN, ...
    'trimCase', struct(), ...
    'trimResult', struct(), ...
    'scoreData', struct(), ...
    'attempts', struct([]));
end

function seed = localSeedTemplate(varargin)
seed = struct( ...
    'name', '', ...
    'source', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'front_collective_guess_rpm', NaN, ...
    'rear_collective_guess_rpm', NaN, ...
    'theta_guess_deg', NaN, ...
    'delta_f_guess_deg', 0.0, ...
    'delta_a_guess_deg', 0.0, ...
    'delta_e_guess_deg', 0.0, ...
    'delta_r_guess_deg', 0.0, ...
    'success', false, ...
    'acceptable', false, ...
    'classification', '', ...
    'score', inf);
if nargin == 1
    inputSeed = varargin{1};
    fields = fieldnames(seed);
    for i = 1:numel(fields)
        fieldName = fields{i};
        if isfield(inputSeed, fieldName) && ~isempty(inputSeed.(fieldName))
            seed.(fieldName) = inputSeed.(fieldName);
        end
    end
end
end

function key = localTargetKey(target)
key = sprintf('tilt_%s_v_%s', localValueLabel(target.tilt_deg), localValueLabel(target.vinf_mps));
end

function values = localTableLogical(column)
if islogical(column)
    values = column;
elseif isnumeric(column)
    values = column ~= 0;
elseif iscellstr(column) || isstring(column)
    values = ismember(lower(string(column)), ["1", "true", "yes"]);
else
    values = false(size(column));
end
end

function value = localTableValue(tbl, varName, idx, defaultValue)
if ismember(varName, tbl.Properties.VariableNames)
    value = tbl.(varName)(idx);
else
    value = defaultValue;
end
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end
