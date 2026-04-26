% Run_Trim_Transition_Map_LowSpeed_Scored.m
% Focused low-speed transition sweep that keeps near-trim points when they
% score well enough for short-hold controller use.
%
% Outputs left in the base workspace:
%   - transitionTrimLowSpeedScoredMap
%   - transitionTrimLowSpeedScoredSummary
%   - transitionTrimLowSpeedScoredOutputDir
%
% On-disk outputs:
%   workspace_plots/transition_trim_map_low_speed_scored_<timestamp>/
%     transition_trim_map_low_speed_scored.mat
%     transition_trim_map_low_speed_scored_summary.csv
%     transition_trim_map_low_speed_scored_summary.md
%     transition_trim_map_low_speed_scored.log
%
% Convenience copies:
%   workspace_plots/transition_trim_map_low_speed_scored_latest.mat
%   workspace_plots/transition_trim_map_low_speed_scored_latest.csv
%   workspace_plots/transition_trim_map_low_speed_scored_latest.md

if ~exist('transitionLowSpeedScoredOptions', 'var') || ~isstruct(transitionLowSpeedScoredOptions)
    transitionLowSpeedScoredOptions = struct();
end

initOptions = struct();
initOptions.transitionLowSpeedScoredOptions = transitionLowSpeedScoredOptions;

Init_EVTOL_Main

if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'transitionLowSpeedScoredOptions')
    transitionLowSpeedScoredOptions = initOptions.transitionLowSpeedScoredOptions;
else
    transitionLowSpeedScoredOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionTrimLowSpeedScoredOutputDir = fullfile(root_dir, 'workspace_plots', ...
    ['transition_trim_map_low_speed_scored_' timestamp]);
if exist(transitionTrimLowSpeedScoredOutputDir, 'dir') ~= 7
    mkdir(transitionTrimLowSpeedScoredOutputDir);
end

checkpointFile = fullfile(transitionTrimLowSpeedScoredOutputDir, 'transition_trim_map_low_speed_scored.mat');
summaryCsv = fullfile(transitionTrimLowSpeedScoredOutputDir, 'transition_trim_map_low_speed_scored_summary.csv');
summaryMd = fullfile(transitionTrimLowSpeedScoredOutputDir, 'transition_trim_map_low_speed_scored_summary.md');
logFile = fullfile(transitionTrimLowSpeedScoredOutputDir, 'transition_trim_map_low_speed_scored.log');

latestMat = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.mat');
latestCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv');
latestMd = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.md');

diary off;
diary(logFile);
cleanupDiary = onCleanup(@() diary('off'));

fprintf('=== Run_Trim_Transition_Map_LowSpeed_Scored ===\n');
fprintf('Started: %s\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf('Output dir: %s\n', transitionTrimLowSpeedScoredOutputDir);

config = localBuildConfig(transitionLowSpeedScoredOptions, root_dir);
historySeeds = localLoadHistorySeeds(config.history_csv);
targets = localBuildTargets(config);

fprintf('Loaded %d historical exact seeds.\n', numel(historySeeds));
fprintf('Configured %d low-speed scored targets.\n', numel(targets));

transitionTrimLowSpeedScoredMap = struct();
transitionTrimLowSpeedScoredMap.meta = struct();
transitionTrimLowSpeedScoredMap.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
transitionTrimLowSpeedScoredMap.meta.root_dir = root_dir;
transitionTrimLowSpeedScoredMap.meta.output_dir = transitionTrimLowSpeedScoredOutputDir;
transitionTrimLowSpeedScoredMap.meta.trim_model = initData.modelNames.trim;
transitionTrimLowSpeedScoredMap.meta.run_model = initData.modelNames.run;
transitionTrimLowSpeedScoredMap.meta.config = config;
transitionTrimLowSpeedScoredMap.meta.history_seed_count = numel(historySeeds);
transitionTrimLowSpeedScoredMap.targets = targets;
transitionTrimLowSpeedScoredMap.entries = repmat(localEntryTemplate(), 0, 1);
transitionTrimLowSpeedScoredMap.progress = struct('completed', 0, 'success_count', 0, ...
    'acceptable_count', 0, 'last_key', '');

transitionTrimLowSpeedScoredSummary = table();

try
    for iTarget = 1:numel(targets)
        target = targets(iTarget);
        fprintf('\n[%d/%d] %s | tilt=%.1f | V=%.1f\n', ...
            iTarget, numel(targets), target.name, target.tilt_deg, target.vinf_mps);

        entry = localSolveTarget(target, transitionTrimLowSpeedScoredMap.entries, historySeeds, initData, config);
        transitionTrimLowSpeedScoredMap.entries(end + 1, 1) = entry; %#ok<SAGROW>
        transitionTrimLowSpeedScoredMap.progress.completed = numel(transitionTrimLowSpeedScoredMap.entries);
        transitionTrimLowSpeedScoredMap.progress.success_count = nnz([transitionTrimLowSpeedScoredMap.entries.success]);
        transitionTrimLowSpeedScoredMap.progress.acceptable_count = nnz([transitionTrimLowSpeedScoredMap.entries.acceptable]);
        transitionTrimLowSpeedScoredMap.progress.last_key = entry.key;

        fprintf('  best family=%s | seed=%s | success=%d | acceptable=%d | score=%.4f | class=%s\n', ...
            entry.family, entry.seed_name, entry.success, entry.acceptable, ...
            entry.score, entry.classification);
        fprintf('  term: %s\n', entry.termination_string);

        if mod(iTarget, config.checkpoint_every) == 0 || entry.success || entry.acceptable || iTarget == numel(targets)
            [transitionTrimLowSpeedScoredSummary, transitionTrimLowSpeedScoredMap] = ...
                localCheckpoint(transitionTrimLowSpeedScoredMap, checkpointFile, latestMat, ...
                summaryCsv, latestCsv, summaryMd, latestMd);
            fprintf('  checkpoint saved.\n');
        end
    end

    [transitionTrimLowSpeedScoredSummary, transitionTrimLowSpeedScoredMap] = ...
        localCheckpoint(transitionTrimLowSpeedScoredMap, checkpointFile, latestMat, ...
        summaryCsv, latestCsv, summaryMd, latestMd);

    fprintf('\nCompleted low-speed scored sweep.\n');
    fprintf('  total entries = %d\n', height(transitionTrimLowSpeedScoredSummary));
    fprintf('  exact successes = %d\n', nnz(transitionTrimLowSpeedScoredSummary.success));
    fprintf('  acceptable near-trims = %d\n', nnz(transitionTrimLowSpeedScoredSummary.acceptable));
    fprintf('  checkpoint = %s\n', checkpointFile);
catch ME
    fprintf('\nRun_Trim_Transition_Map_LowSpeed_Scored failed: %s\n', ME.message);
    [transitionTrimLowSpeedScoredSummary, transitionTrimLowSpeedScoredMap] = ...
        localCheckpoint(transitionTrimLowSpeedScoredMap, checkpointFile, latestMat, ...
        summaryCsv, latestCsv, summaryMd, latestMd);
    rethrow(ME);
end

function config = localBuildConfig(userOptions, root_dir)
config = struct();
config.root_dir = root_dir;
config.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 5);
config.target_limit = localGetField(userOptions, 'target_limit', inf);
config.tilt_grid_deg = localGetField(userOptions, 'tilt_grid_deg', 0:5:30);
config.vinf_grid_mps = localGetField(userOptions, 'vinf_grid_mps', 0:2.5:25);
config.max_vinf_mps = localGetField(userOptions, 'max_vinf_mps', 25.0);
config.base_vinf_ceiling_mps = localGetField(userOptions, 'base_vinf_ceiling_mps', 5.0);
config.vinf_per_tilt = localGetField(userOptions, 'vinf_per_tilt', 0.8);
config.neighbor_seed_count = localGetField(userOptions, 'neighbor_seed_count', 3);
config.history_seed_count = localGetField(userOptions, 'history_seed_count', 3);
config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);
config.score_options = localGetField(userOptions, 'score_options', struct( ...
    'profile', 'transition', ...
    'hold_horizon_s', 2.0));
config.history_csv = localGetField(userOptions, 'history_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv'));
config.seed_distance_weights = struct('tilt', 1 / 5, 'vinf', 1 / 5);
end

function targets = localBuildTargets(config)
targets = repmat(localTargetTemplate(), 0, 1);
vinf_grid = config.vinf_grid_mps(:).';

for iTilt = 1:numel(config.tilt_grid_deg)
    tilt_deg = config.tilt_grid_deg(iTilt);
    vinf_ceiling = min(config.max_vinf_mps, config.base_vinf_ceiling_mps + config.vinf_per_tilt * tilt_deg);
    vinf_values = vinf_grid(vinf_grid <= vinf_ceiling + 1e-9);

    if tilt_deg == 0
        vinf_values = unique([0.0, vinf_values]);
    else
        vinf_values = vinf_values(vinf_values > 0.0);
    end

    for vinf_mps = vinf_values
        target = localTargetTemplate();
        target.tilt_deg = tilt_deg;
        target.vinf_mps = vinf_mps;
        target.name = sprintf('LowSpeedScored_Tilt%s_V%s', ...
            localValueLabel(tilt_deg), localValueLabel(vinf_mps));
        target.sort_key = vinf_mps + 0.75 * tilt_deg;
        targets(end + 1, 1) = target; %#ok<SAGROW>
    end
end

if ~isinf(config.target_limit)
    targets = targets(1:min(numel(targets), config.target_limit));
end

[~, order] = sort([targets.sort_key], 'ascend');
targets = targets(order);
end

function historySeeds = localLoadHistorySeeds(filename)
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

successMask = localTableLogical(summary.success);
summary = summary(successMask, :);
for i = 1:height(summary)
    seed = localSeedTemplate();
    seed.name = sprintf('history_%s', localValueLabel(i));
    seed.source = 'history_exact';
    seed.tilt_deg = summary.tilt_deg(i);
    seed.vinf_mps = summary.vinf_mps(i);
    seed.front_collective_guess_rpm = summary.front_collective_rpm(i);
    seed.rear_collective_guess_rpm = summary.rear_collective_rpm(i);
    seed.theta_guess_deg = summary.theta_deg(i);
    seed.delta_f_guess_deg = localTableValue(summary, 'delta_f_deg', i, 0.0);
    seed.delta_a_guess_deg = localTableValue(summary, 'delta_a_deg', i, 0.0);
    seed.delta_e_guess_deg = localTableValue(summary, 'delta_e_deg', i, 0.0);
    seed.delta_r_guess_deg = localTableValue(summary, 'delta_r_deg', i, 0.0);
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
    distances = arrayfun(@(s) localSeedDistance(s, target, config.seed_distance_weights), historySeeds);
    [~, order] = sort(distances, 'ascend');
    order = order(1:min(numel(order), config.history_seed_count));
    for i = 1:numel(order)
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(historySeeds(order(i))); %#ok<SAGROW>
    end
end

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

seedCandidates = localUniqueSeeds(seedCandidates);
end

function baseCase = localBaseSeedCase(target)
baseCase = TrimCase_Cruise75_FlapElevator();
baseCase.name = target.name;
baseCase.mode = 'transition_low_speed_scored';
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

families(end + 1, 1) = localMakeHoverZeroSurfaceCase(target, seed); %#ok<SAGROW>
families(end + 1, 1) = localMakeFrontRearFreeFlapElevatorCase(target, seed); %#ok<SAGROW>
families(end + 1, 1) = localMakeRearFixedFlapElevatorCase(target, seed); %#ok<SAGROW>
families(end + 1, 1) = localMakePropFixedFlapElevatorCase(target, seed); %#ok<SAGROW>
end

function family = localMakeHoverZeroSurfaceCase(target, seed)
trimCase = TrimCase_Hover();
trimCase.name = sprintf('%s_hoverZero_%s', target.name, seed.name);
trimCase.mode = 'transition_low_speed_scored';
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
    warning('Run_Trim_Transition_Map_LowSpeed_Scored:WriteMarkdownFailed', ...
        'Could not write %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Low-Speed Scored Trim Sweep\n\n');
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
end

function distance = localEntryDistance(entry, target, weights)
distance = weights.tilt * abs(entry.tilt_deg - target.tilt_deg) + ...
           weights.vinf * abs(entry.vinf_mps - target.vinf_mps);
end

function distance = localSeedDistance(seed, target, weights)
distance = weights.tilt * abs(seed.tilt_deg - target.tilt_deg) + ...
           weights.vinf * abs(seed.vinf_mps - target.vinf_mps);
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
    'delta_r_guess_deg', 0.0);
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
