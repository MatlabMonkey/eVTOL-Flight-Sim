% Run_Trim_Transition_Path_Scored.m
% Directed path-following scored transition sweep. This script starts from
% the good hover-side acceptable points, prefers "tilt up first, then
% airspeed", and only branches through nearby targets with smooth prop
% changes toward the cruise anchor.
%
% Outputs left in the base workspace:
%   - transitionTrimPathScoredMap
%   - transitionTrimPathScoredSummary
%   - transitionTrimPathScoredOutputDir
%
% On-disk outputs:
%   workspace_plots/transition_trim_path_scored_<timestamp>/
%     transition_trim_path_scored.mat
%     transition_trim_path_scored_summary.csv
%     transition_trim_path_scored_summary.md
%     transition_trim_path_scored.log
%
% Convenience copies:
%   workspace_plots/transition_trim_path_scored_latest.mat
%   workspace_plots/transition_trim_path_scored_latest.csv
%   workspace_plots/transition_trim_path_scored_latest.md

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

checkpointFile = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_path_scored.mat');
summaryCsv = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_path_scored_summary.csv');
summaryMd = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_path_scored_summary.md');
logFile = fullfile(transitionTrimPathScoredOutputDir, 'transition_trim_path_scored.log');

latestMat = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_latest.mat']);
latestCsv = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_latest.csv']);
latestMd = fullfile(root_dir, 'workspace_plots', [config.latest_prefix '_latest.md']);

diary off;
diary(logFile);
cleanupDiary = onCleanup(@() diary('off'));

fprintf('=== Run_Trim_Transition_Path_Scored ===\n');
fprintf('Started: %s\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf('Output dir: %s\n', transitionTrimPathScoredOutputDir);

historyBundle = localLoadHistorySeedBundle(config);
frontierSeeds = localBuildInitialFrontierSeeds(historyBundle, config);
[targetQueue, queuedKeys] = localBuildInitialQueue(frontierSeeds, historyBundle.knownGoodPoints, historyBundle.blockedPoints, config);
attemptedKeys = containers.Map('KeyType', 'char', 'ValueType', 'logical');
knownGoodPoints = historyBundle.knownGoodPoints;
knownGoodKeys = localKnownGoodKeyMap(knownGoodPoints);
blockedKeys = localKnownGoodKeyMap(historyBundle.blockedPoints);

fprintf('Loaded %d exact-history seeds and %d scored-history seeds.\n', ...
    numel(historyBundle.exactSeeds), numel(historyBundle.scoredSeeds));
fprintf('Initialized %d frontier seeds and %d queued path targets.\n', ...
    numel(frontierSeeds), numel(targetQueue));

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
transitionTrimPathScoredMap.meta.initial_frontier_count = numel(frontierSeeds);
transitionTrimPathScoredMap.meta.initial_queue_count = numel(targetQueue);
transitionTrimPathScoredMap.meta.known_good_point_count = size(historyBundle.knownGoodPoints, 1);
transitionTrimPathScoredMap.meta.blocked_point_count = size(historyBundle.blockedPoints, 1);
transitionTrimPathScoredMap.targets = targetQueue;
transitionTrimPathScoredMap.entries = repmat(localEntryTemplate(), 0, 1);
transitionTrimPathScoredMap.progress = struct('completed', 0, 'success_count', 0, ...
    'acceptable_count', 0, 'last_key', '', 'queue_remaining', numel(targetQueue));

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
            newFrontierSeed = localSeedFromEntry(entry);
            frontierSeeds(end + 1, 1) = newFrontierSeed; %#ok<SAGROW>
            knownGoodPoints(end + 1, :) = [entry.tilt_deg, entry.vinf_mps]; %#ok<AGROW>
            knownGoodKeys(queueKey) = true;
            [targetQueue, queuedKeys] = localEnqueueTargetsFromSeed(targetQueue, queuedKeys, ...
                newFrontierSeed, knownGoodKeys, blockedKeys, attemptedKeys, config);
            transitionTrimPathScoredMap.progress.queue_remaining = numel(targetQueue);
        end

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

    fprintf('\nCompleted path scored sweep.\n');
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
    fprintf('\nRun_Trim_Transition_Path_Scored failed: %s\n', ME.message);
    transitionTrimPathScoredMap.targets = targetQueue;
    [transitionTrimPathScoredSummary, transitionTrimPathScoredMap] = ...
        localCheckpoint(transitionTrimPathScoredMap, checkpointFile, latestMat, ...
        summaryCsv, latestCsv, summaryMd, latestMd);
    rethrow(ME);
end

function config = localBuildConfig(userOptions, root_dir)
config = struct();
config.root_dir = root_dir;
config.output_prefix = localGetField(userOptions, 'output_prefix', 'transition_trim_path_scored');
config.latest_prefix = localGetField(userOptions, 'latest_prefix', 'transition_trim_path_scored');
config.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 5);
config.max_attempts = localGetField(userOptions, 'max_attempts', 120);
config.min_tilt_deg = localGetField(userOptions, 'min_tilt_deg', 0.0);
config.max_tilt_deg = localGetField(userOptions, 'max_tilt_deg', 90.0);
config.min_vinf_mps = localGetField(userOptions, 'min_vinf_mps', 0.0);
config.max_vinf_mps = localGetField(userOptions, 'max_vinf_mps', 70.0);
config.initial_frontier_tilt_max_deg = localGetField(userOptions, 'initial_frontier_tilt_max_deg', 30.0);
config.initial_frontier_vinf_max_mps = localGetField(userOptions, 'initial_frontier_vinf_max_mps', 25.0);
config.use_only_scored_frontier = localGetField(userOptions, 'use_only_scored_frontier', true);
config.neighbor_seed_count = localGetField(userOptions, 'neighbor_seed_count', 3);
config.history_exact_seed_count = localGetField(userOptions, 'history_exact_seed_count', 2);
config.history_scored_seed_count = localGetField(userOptions, 'history_scored_seed_count', 2);
config.max_prop_nudge_fraction = localGetField(userOptions, 'max_prop_nudge_fraction', 0.15);
config.use_analytic_fallback = localGetField(userOptions, 'use_analytic_fallback', false);
config.guide_row_speed_mps = localGetField(userOptions, 'guide_row_speed_mps', 25.0);
config.guide_row_tilt_end_deg = localGetField(userOptions, 'guide_row_tilt_end_deg', 50.0);
config.guide_speed_slack_mps = localGetField(userOptions, 'guide_speed_slack_mps', 2.5);
config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);
config.score_options = localGetField(userOptions, 'score_options', struct( ...
    'profile', 'transition', ...
    'hold_horizon_s', 2.0));
config.history_exact_csv = localGetField(userOptions, 'history_exact_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv'));
config.history_scored_csv = localGetField(userOptions, 'history_scored_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'));
config.seed_distance_weights = struct('tilt', 1 / 5, 'vinf', 1 / 5);
config.cruise_anchor = struct('tilt_deg', 90.0, 'vinf_mps', 70.0);
config.start_points = localNormalizeStartPoints(localGetField(userOptions, 'start_points', []));
end

function historyBundle = localLoadHistorySeedBundle(config)
historyBundle = struct();
historyBundle.exactSeeds = localLoadHistorySeedsFromCsv(config.history_exact_csv, 'success_only', 'history_exact');
historyBundle.scoredSeeds = localLoadHistorySeedsFromCsv(config.history_scored_csv, 'acceptable_or_better', 'history_scored');
historyBundle.allSeeds = [historyBundle.scoredSeeds; historyBundle.exactSeeds];
historyBundle.knownGoodPoints = localLoadKnownGoodPoints(config.history_exact_csv, config.history_scored_csv);
historyBundle.blockedPoints = localLoadRejectedPointsFromCsv(config.history_scored_csv);
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

function historySeeds = localLoadHistorySeedsFromCsv(filename, mode, sourceName)
historySeeds = repmat(localSeedTemplate(), 0, 1);
if exist(filename, 'file') ~= 2
    return;
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

function entry = localSolveTarget(target, existingEntries, historySeeds, initData, config)
seedCandidates = localBuildSeedCandidates(target, existingEntries, historySeeds, initData, config);
attempts = repmat(localAttemptTemplate(), 0, 1);
bestAttempt = localAttemptTemplate();

for iSeed = 1:numel(seedCandidates)
    seed = seedCandidates(iSeed);
    families = localBuildFamilies(target, seed);
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

        attempt = localMakeAttempt(target, family, trimResult, scoreData);
        attempts(end + 1, 1) = attempt; %#ok<SAGROW>
        if localIsBetterAttempt(attempt, bestAttempt)
            bestAttempt = attempt;
        end
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

baseCase = localBaseSeedCase(target);

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
    hoverCase = TrimCase_Hover();
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
    cruiseCase = TrimCase_Cruise75_FlapElevator();
    cruiseSeed.name = 'cruise_default';
    cruiseSeed.source = 'cruise_default';
    cruiseSeed.tilt_deg = target.tilt_deg;
    cruiseSeed.vinf_mps = target.vinf_mps;
    cruiseSeed.front_collective_guess_rpm = cruiseCase.front_collective_guess_rpm;
    cruiseSeed.rear_collective_guess_rpm = localGetField(cruiseCase, 'rear_collective_fixed_rpm', 0.0);
    cruiseSeed.theta_guess_deg = localGetField(cruiseCase, 'theta_guess_deg', 0.0);
    cruiseSeed.delta_f_guess_deg = localGetField(cruiseCase, 'delta_f_guess_deg', 0.0);
    cruiseSeed.delta_a_guess_deg = 0.0;
    cruiseSeed.delta_e_guess_deg = localGetField(cruiseCase, 'delta_e_guess_deg', 0.0);
    cruiseSeed.delta_r_guess_deg = 0.0;
    seedCandidates(end + 1, 1) = cruiseSeed; %#ok<SAGROW>
end

seedCandidates = localUniqueSeeds(seedCandidates);
end

function seed = localMakeCruiseNudgedSeed(parentSeed, target, config)
seed = localCanonicalizeSeed(parentSeed);
cruiseCase = TrimCase_Cruise75_FlapElevator();

dt = max(target.tilt_deg - parentSeed.tilt_deg, 0.0);
dv = max(target.vinf_mps - parentSeed.vinf_mps, 0.0);
stepFraction = 0.05 + 0.05 * (dt / 5.0) + 0.03 * (dv / 2.5);
progressFraction = max(target.tilt_deg / max(config.cruise_anchor.tilt_deg, 1.0), ...
    target.vinf_mps / max(config.cruise_anchor.vinf_mps, 1.0));
gamma = min(config.max_prop_nudge_fraction, stepFraction * max(progressFraction, 0.25));

seed.name = ['nudged_' parentSeed.name];
seed.source = 'parent_nudged_to_cruise';
seed.front_collective_guess_rpm = parentSeed.front_collective_guess_rpm + ...
    gamma * (cruiseCase.front_collective_guess_rpm - parentSeed.front_collective_guess_rpm);
seed.rear_collective_guess_rpm = parentSeed.rear_collective_guess_rpm + ...
    gamma * (localGetField(cruiseCase, 'rear_collective_fixed_rpm', 0.0) - parentSeed.rear_collective_guess_rpm);
seed.theta_guess_deg = parentSeed.theta_guess_deg + ...
    gamma * (localGetField(cruiseCase, 'theta_guess_deg', 0.0) - parentSeed.theta_guess_deg);
seed.delta_f_guess_deg = parentSeed.delta_f_guess_deg + ...
    gamma * (localGetField(cruiseCase, 'delta_f_guess_deg', 0.0) - parentSeed.delta_f_guess_deg);
seed.delta_e_guess_deg = parentSeed.delta_e_guess_deg + ...
    gamma * (localGetField(cruiseCase, 'delta_e_guess_deg', 0.0) - parentSeed.delta_e_guess_deg);
seed.tilt_deg = target.tilt_deg;
seed.vinf_mps = target.vinf_mps;
end

function baseCase = localBaseSeedCase(target)
baseCase = TrimCase_Cruise75_FlapElevator();
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
end

function families = localBuildFamilies(target, seed)
families = repmat(localFamilyTemplate(), 0, 1);

families(end + 1, 1) = localMakePropFixedFlapElevatorCase(target, seed); %#ok<SAGROW>
families(end + 1, 1) = localMakeFrontRearFreeFlapElevatorCase(target, seed); %#ok<SAGROW>
if target.tilt_deg >= 30
    families(end + 1, 1) = localMakeRearFixedFlapElevatorCase(target, seed); %#ok<SAGROW>
end
if target.tilt_deg <= 20 && target.vinf_mps <= 15
    families(end + 1, 1) = localMakeHoverZeroSurfaceCase(target, seed); %#ok<SAGROW>
end
end

function family = localMakeHoverZeroSurfaceCase(target, seed)
trimCase = TrimCase_Hover();
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

function family = localMakeFrontRearFreeFlapElevatorCase(target, seed)
trimCase = localBaseSeedCase(target);
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

function family = localMakeRearFixedFlapElevatorCase(target, seed)
trimCase = localBaseSeedCase(target);
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

function family = localMakePropFixedFlapElevatorCase(target, seed)
trimCase = localBaseSeedCase(target);
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

function attempt = localMakeAttempt(target, family, trimResult, scoreData)
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
    warning('Run_Trim_Transition_Path_Scored:WriteMarkdownFailed', ...
        'Could not write %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Path Scored Trim Sweep\n\n');
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
knownGoodPoints = zeros(0, 2);
knownGoodPoints = [knownGoodPoints; localLoadKnownGoodPointsFromCsv(exactCsv, 'success_only')]; %#ok<AGROW>
knownGoodPoints = [knownGoodPoints; localLoadKnownGoodPointsFromCsv(scoredCsv, 'acceptable_or_better')]; %#ok<AGROW>
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
