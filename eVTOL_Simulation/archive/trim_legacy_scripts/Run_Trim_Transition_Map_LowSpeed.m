% Run_Trim_Transition_Map_LowSpeed.m
% Low-speed transition trim exploration with physics-informed seeds.
%
% This script keeps the previous transition map intact and runs a second,
% low-speed-focused exploration that:
%   - loads the latest exact trims from Run_Trim_Transition_Map
%   - builds a low-speed/low-tilt target set
%   - uses a force-balance-style prop seed plus historical exact neighbors
%   - removes the duplicate NED-down steady-state hold and relies on the
%     vertical-speed output constraint for level-flight trimming
%
% Outputs left in the base workspace:
%   - transitionTrimLowSpeedMap
%   - transitionTrimLowSpeedSummary
%   - transitionTrimLowSpeedOutputDir
%   - transitionTrimLowSpeedCheckpointFile
%   - transitionTrimLowSpeedLogFile
%   - transitionTrimMergedSummary
%
% On-disk outputs:
%   workspace_plots/transition_trim_map_low_speed_<timestamp>/
%     transition_trim_map_low_speed.mat
%     transition_trim_map_low_speed_summary.csv
%     transition_trim_map_low_speed_summary.md
%     transition_trim_map_low_speed.log
%
% Convenience copies:
%   workspace_plots/transition_trim_map_low_speed_latest.mat
%   workspace_plots/transition_trim_map_low_speed_latest.csv
%   workspace_plots/transition_trim_map_low_speed_latest.md
%   workspace_plots/transition_trim_map_merged_latest.csv
%   workspace_plots/transition_trim_map_merged_latest.md

if ~exist('transitionLowSpeedOptions', 'var') || ~isstruct(transitionLowSpeedOptions)
    transitionLowSpeedOptions = struct();
end

initOptions = struct();
initOptions.transitionLowSpeedOptions = transitionLowSpeedOptions;

Init_EVTOL_Main

if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'transitionLowSpeedOptions')
    transitionLowSpeedOptions = initOptions.transitionLowSpeedOptions;
else
    transitionLowSpeedOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionTrimLowSpeedOutputDir = fullfile(root_dir, 'workspace_plots', ...
    ['transition_trim_map_low_speed_' timestamp]);
if exist(transitionTrimLowSpeedOutputDir, 'dir') ~= 7
    mkdir(transitionTrimLowSpeedOutputDir);
end

transitionTrimLowSpeedCheckpointFile = fullfile(transitionTrimLowSpeedOutputDir, 'transition_trim_map_low_speed.mat');
transitionTrimLowSpeedSummaryCsv = fullfile(transitionTrimLowSpeedOutputDir, 'transition_trim_map_low_speed_summary.csv');
transitionTrimLowSpeedSummaryMd = fullfile(transitionTrimLowSpeedOutputDir, 'transition_trim_map_low_speed_summary.md');
transitionTrimLowSpeedLogFile = fullfile(transitionTrimLowSpeedOutputDir, 'transition_trim_map_low_speed.log');

transitionTrimLowSpeedLatestMat = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_latest.mat');
transitionTrimLowSpeedLatestCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_latest.csv');
transitionTrimLowSpeedLatestMd = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_latest.md');
transitionTrimMergedLatestCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv');
transitionTrimMergedLatestMd = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.md');

diary off;
diary(transitionTrimLowSpeedLogFile);
cleanupDiary = onCleanup(@() diary('off'));

fprintf('=== Run_Trim_Transition_Map_LowSpeed ===\n');
fprintf('Started: %s\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf('Output dir: %s\n', transitionTrimLowSpeedOutputDir);

config = localBuildConfig(transitionLowSpeedOptions, root_dir);
[baseEntries, baseSummary] = localLoadBaseMap(config.base_map_file);
config.base_entries = baseEntries;
config.base_summary = baseSummary;
config.aircraft_seed_model = localBuildAircraftSeedModel();

targets = localBuildTargets(config);

fprintf('Loaded %d exact historical seed entries.\n', nnz([baseEntries.success]));
fprintf('Configured %d low-speed target trim points across %d phases.\n', ...
    numel(targets), numel(unique({targets.phase})));

transitionTrimLowSpeedMap = struct();
transitionTrimLowSpeedMap.meta = struct();
transitionTrimLowSpeedMap.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
transitionTrimLowSpeedMap.meta.root_dir = root_dir;
transitionTrimLowSpeedMap.meta.output_dir = transitionTrimLowSpeedOutputDir;
transitionTrimLowSpeedMap.meta.base_map_file = config.base_map_file;
transitionTrimLowSpeedMap.meta.trim_model = initData.modelNames.trim;
transitionTrimLowSpeedMap.meta.run_model = initData.modelNames.run;
transitionTrimLowSpeedMap.meta.config = config;
transitionTrimLowSpeedMap.meta.target_count = numel(targets);
transitionTrimLowSpeedMap.progress = struct('completed', 0, 'success_count', 0, 'failure_count', 0, 'last_key', '');
transitionTrimLowSpeedMap.targets = targets;
transitionTrimLowSpeedMap.entries = repmat(localEntryTemplate(), 0, 1);

transitionTrimLowSpeedSummary = table();
transitionTrimMergedSummary = table();

try
    for iTarget = 1:numel(targets)
        target = targets(iTarget);
        key = localTargetKey(target);

        if localHasKey(transitionTrimLowSpeedMap.entries, key)
            continue;
        end

        fprintf('\n[%d/%d] %s | family=%s | tilt=%.1f | V=%.1f | rear=%s\n', ...
            iTarget, numel(targets), target.name, target.family, ...
            target.tilt_deg, target.vinf_mps, localFormatRearLabel(target.rear_fixed_rpm));

        allSeedEntries = [config.base_entries; transitionTrimLowSpeedMap.entries];
        entry = localSolveTarget(target, allSeedEntries, initData, config);
        transitionTrimLowSpeedMap.entries(end + 1, 1) = entry; %#ok<SAGROW>
        transitionTrimLowSpeedMap.progress.completed = numel(transitionTrimLowSpeedMap.entries);
        transitionTrimLowSpeedMap.progress.success_count = nnz([transitionTrimLowSpeedMap.entries.success]);
        transitionTrimLowSpeedMap.progress.failure_count = transitionTrimLowSpeedMap.progress.completed - ...
            transitionTrimLowSpeedMap.progress.success_count;
        transitionTrimLowSpeedMap.progress.last_key = key;

        fprintf('  result: success=%d | maxResidual=%.6g | attempts=%d\n', ...
            entry.success, entry.max_state_residual, entry.attempt_count);
        fprintf('  termination: %s\n', entry.termination_string);

        if mod(iTarget, config.checkpoint_every) == 0 || entry.success || iTarget == numel(targets)
            [transitionTrimLowSpeedSummary, transitionTrimMergedSummary, transitionTrimLowSpeedMap] = ...
                localCheckpoint(transitionTrimLowSpeedMap, config.base_summary, ...
                transitionTrimLowSpeedCheckpointFile, transitionTrimLowSpeedLatestMat, ...
                transitionTrimLowSpeedSummaryCsv, transitionTrimLowSpeedLatestCsv, ...
                transitionTrimLowSpeedSummaryMd, transitionTrimLowSpeedLatestMd, ...
                transitionTrimMergedLatestCsv, transitionTrimMergedLatestMd);
            fprintf('  checkpoint saved.\n');
        end
    end

    [transitionTrimLowSpeedSummary, transitionTrimMergedSummary, transitionTrimLowSpeedMap] = ...
        localCheckpoint(transitionTrimLowSpeedMap, config.base_summary, ...
        transitionTrimLowSpeedCheckpointFile, transitionTrimLowSpeedLatestMat, ...
        transitionTrimLowSpeedSummaryCsv, transitionTrimLowSpeedLatestCsv, ...
        transitionTrimLowSpeedSummaryMd, transitionTrimLowSpeedLatestMd, ...
        transitionTrimMergedLatestCsv, transitionTrimMergedLatestMd);

    fprintf('\nCompleted low-speed transition trim map.\n');
    fprintf('  total new entries = %d\n', height(transitionTrimLowSpeedSummary));
    fprintf('  exact new successes = %d\n', nnz(transitionTrimLowSpeedSummary.success));
    fprintf('  merged exact successes = %d\n', nnz(transitionTrimMergedSummary.success));
    fprintf('  checkpoint = %s\n', transitionTrimLowSpeedCheckpointFile);
catch ME
    fprintf('\nRun_Trim_Transition_Map_LowSpeed failed: %s\n', ME.message);
    [transitionTrimLowSpeedSummary, transitionTrimMergedSummary, transitionTrimLowSpeedMap] = ...
        localCheckpoint(transitionTrimLowSpeedMap, config.base_summary, ...
        transitionTrimLowSpeedCheckpointFile, transitionTrimLowSpeedLatestMat, ...
        transitionTrimLowSpeedSummaryCsv, transitionTrimLowSpeedLatestCsv, ...
        transitionTrimLowSpeedSummaryMd, transitionTrimLowSpeedLatestMd, ...
        transitionTrimMergedLatestCsv, transitionTrimMergedLatestMd);
    rethrow(ME);
end

function config = localBuildConfig(userOptions, root_dir)
config = struct();
config.root_dir = root_dir;
config.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 10);
config.max_seed_attempts = localGetField(userOptions, 'max_seed_attempts', 8);
config.target_limit = localGetField(userOptions, 'target_limit', inf);
config.base_map_file = localGetField(userOptions, 'base_map_file', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_latest.mat'));
config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);
config.distance_weights = struct( ...
    'tilt_deg', 1 / 5, ...
    'vinf_mps', 1 / 5, ...
    'rear_rpm', 1 / 150, ...
    'family_mismatch', 1.0);
end

function [baseEntries, baseSummary] = localLoadBaseMap(filename)
baseEntries = repmat(localEntryTemplate(), 0, 1);
baseSummary = table();

if exist(filename, 'file') ~= 2
    warning('Run_Trim_Transition_Map_LowSpeed:BaseMapMissing', ...
        'Base map file not found: %s', filename);
    return;
end

data = load(filename);
if isfield(data, 'trimMap') && isstruct(data.trimMap) && isfield(data.trimMap, 'entries')
    baseEntries = data.trimMap.entries;
end
if isfield(data, 'summaryTable') && istable(data.summaryTable)
    baseSummary = data.summaryTable;
elseif isfield(data, 'trimMap') && isstruct(data.trimMap) && isfield(data.trimMap, 'summary_table')
    baseSummary = data.trimMap.summary_table;
end
end

function seedModel = localBuildAircraftSeedModel()
aircraft = aircraft_def();

seedModel = struct();
seedModel.aircraft = aircraft;
seedModel.rho = aircraft.rho;
seedModel.g = aircraft.g;
seedModel.mass = aircraft.Mass;
seedModel.weight_N = aircraft.Mass * aircraft.g;
seedModel.k_thrust = aircraft.prop.k_Thrust;
seedModel.front_rotor_count = 6;
seedModel.rear_rotor_count = 6;
seedModel.wing_S = aircraft.wing.S;
seedModel.wing_c = aircraft.wing.c;
seedModel.wing_CL0 = aircraft.wing.CL0;
seedModel.wing_CLa = aircraft.wing.CLa;
seedModel.wing_CD0 = aircraft.wing.CD0;
seedModel.wing_CDa = aircraft.wing.CDa;
seedModel.wing_a0 = aircraft.wing.a0;
seedModel.CL_df = max(1e-6, -aircraft.controls.flaperon.expected.CZ_delta_f_per_rad);
seedModel.CM_de = aircraft.controls.ruddervator.expected.CM_delta_e_per_rad;

[frontPos, rearPos, wingPos] = localExtractSeedGeometry(aircraft);
seedModel.front_pos = frontPos;
seedModel.rear_pos = rearPos;
seedModel.wing_pos = wingPos;
seedModel.front_vertical_share = abs(rearPos(1)) / (frontPos(1) + abs(rearPos(1)));
end

function [frontPos, rearPos, wingPos] = localExtractSeedGeometry(aircraft)
compData = aircraft.compData;
cg = aircraft.CG(:)';

frontMask = contains(compData(:, 1), 'F-Rotor ');
rearMask = contains(compData(:, 1), 'R-Rotor ');
wingMask = contains(compData(:, 1), 'Main Wing');

frontXYZ = localRowsToMatrix(compData(frontMask, 5));
rearXYZ = localRowsToMatrix(compData(rearMask, 5));
wingXYZ = localRowsToMatrix(compData(wingMask, 5));

frontPos = mean(frontXYZ, 1) - cg;
rearPos = mean(rearXYZ, 1) - cg;
wingPos = mean(wingXYZ, 1) - cg;
end

function xyz = localRowsToMatrix(cellRows)
xyz = zeros(numel(cellRows), 3);
for i = 1:numel(cellRows)
    xyz(i, :) = cellRows{i};
end
end

function targets = localBuildTargets(config)
targets = repmat(localTargetTemplate(), 0, 1);

% Phase A: connect the existing exact corridor down toward 30-50 deg / 40-55 m/s.
for tilt = 30:5:50
    for speed = 40:2.5:55
        rearList = localRearCandidateList(tilt, speed, config);
        for rear = rearList
            targets(end + 1) = localMakeLowSpeedTarget('bridge_upper_band', tilt, speed, rear); %#ok<SAGROW>
        end
    end
end

% Phase B: mid-transition bridge between the upper exact corridor and hover.
for tilt = 15:5:40
    for speed = 20:2.5:45
        rearList = localRearCandidateList(tilt, speed, config);
        for rear = rearList
            targets(end + 1) = localMakeLowSpeedTarget('bridge_mid_band', tilt, speed, rear); %#ok<SAGROW>
        end
    end
end

% Phase C: hover-side bridge where prop-vector balancing should matter most.
for tilt = 0:5:20
    for speed = 2.5:2.5:25
        rearList = localRearCandidateList(tilt, speed, config);
        for rear = rearList
            targets(end + 1) = localMakeLowSpeedTarget('bridge_hover_band', tilt, speed, rear); %#ok<SAGROW>
        end
    end
end

targets = localUniqueTargets(targets);
targets = localSortTargets(targets, config);
if isfinite(config.target_limit)
    targets = targets(1:min(numel(targets), max(1, round(config.target_limit))));
end
end

function rearList = localRearCandidateList(tilt_deg, vinf_mps, config)
rearRef = localRearNominalRpm(tilt_deg, vinf_mps, config.aircraft_seed_model);
rearHist = localHistoryRearReference(tilt_deg, vinf_mps, config.base_entries);

if isfinite(rearHist)
    rearCenter = 0.5 * (rearRef + rearHist);
else
    rearCenter = rearRef;
end

if vinf_mps <= 15
    offsets = [-200, -100, 0, 100, 200];
elseif vinf_mps <= 35
    offsets = [-250, -125, 0, 125, 250];
else
    offsets = [-300, -150, 0, 150, 300];
end

rearList = rearCenter + offsets;
rearList = round(rearList / 50.0) * 50.0;
rearList = max(0.0, min(2200.0, rearList));
rearList = unique(rearList, 'stable');
end

function rearRpm = localRearNominalRpm(tilt_deg, vinf_mps, seedModel)
q = 0.5 * seedModel.rho * vinf_mps^2;
alphaGuessRad = deg2rad(max(0.0, 4.0 * (1.0 - min(vinf_mps / 75.0, 1.0)) * cosd(min(tilt_deg, 90.0))));
clBase = max(0.0, seedModel.wing_CL0 + seedModel.wing_CLa * alphaGuessRad);
liftBase = q * seedModel.wing_S * clBase;
weightResidual = max(0.0, seedModel.weight_N - liftBase);

tiltSin = max(sind(tilt_deg), 0.05);
tiltCos = max(cosd(tilt_deg), 0.15);
cdBase = seedModel.wing_CD0 + seedModel.wing_CDa * (alphaGuessRad - seedModel.wing_a0)^2;
dragBase = q * seedModel.wing_S * cdBase;

frontVerticalShare = seedModel.front_vertical_share * weightResidual;
frontFromHover = frontVerticalShare / tiltCos;
frontFromDrag = dragBase / tiltSin;
frontThrust = max(frontFromHover, frontFromDrag);

rearThrust = max(weightResidual - frontThrust * cosd(tilt_deg), 0.0);
rearRpm = localRpmFromGroupThrust(rearThrust, seedModel.rear_rotor_count, seedModel.k_thrust);
end

function rearHist = localHistoryRearReference(tilt_deg, vinf_mps, baseEntries)
rearHist = NaN;
successEntries = baseEntries([baseEntries.success]);
if isempty(successEntries)
    return;
end

distances = [];
rearVals = [];
for i = 1:numel(successEntries)
    entry = successEntries(i);
    if ~isfinite(entry.target_rear_fixed_rpm)
        continue;
    end
    d = abs(entry.target_tilt_deg - tilt_deg) / 5.0 + abs(entry.target_vinf_mps - vinf_mps) / 5.0;
    distances(end + 1, 1) = d; %#ok<AGROW>
    rearVals(end + 1, 1) = entry.target_rear_fixed_rpm; %#ok<AGROW>
end

if isempty(distances)
    return;
end

[distances, order] = sort(distances, 'ascend');
rearVals = rearVals(order);
count = min(4, numel(distances));
weights = 1 ./ (0.25 + distances(1:count)).^2;
rearHist = sum(weights .* rearVals(1:count)) / sum(weights);
end

function target = localMakeLowSpeedTarget(phase, tilt_deg, vinf_mps, rear_fixed_rpm)
target = localTargetTemplate();
target.phase = phase;
target.family = 'low_speed_force_balance_rear_fixed';
target.tilt_deg = tilt_deg;
target.vinf_mps = vinf_mps;
target.rear_fixed_rpm = rear_fixed_rpm;
target.name = sprintf('LSFB_tilt_%s__V_%s__rear_%s', ...
    localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(rear_fixed_rpm));
end

function targets = localSortTargets(targets, config) %#ok<INUSD>
priority = zeros(numel(targets), 1);
for i = 1:numel(targets)
    target = targets(i);
    switch target.phase
        case 'bridge_upper_band'
            base = 10;
        case 'bridge_mid_band'
            base = 20;
        case 'bridge_hover_band'
            base = 30;
        otherwise
            base = 40;
    end

    anchor_term = abs(target.tilt_deg - 45.0) / 5.0 + abs(target.vinf_mps - 50.0) / 5.0;
    if target.tilt_deg <= 20
        anchor_term = abs(target.tilt_deg - 0.0) / 5.0 + abs(target.vinf_mps - 5.0) / 2.5;
    end
    priority(i) = base + anchor_term;
end

[~, order] = sort(priority, 'ascend');
targets = targets(order);
end

function entry = localSolveTarget(target, allSeedEntries, initData, config)
seedCandidates = localBuildSeedCandidates(target, allSeedEntries, config);

bestAttemptSummary = struct();
bestAttemptSummary.max_state_residual = inf;
bestAttemptSummary.success = false;
bestAttemptSummary.trimResult = struct();
bestAttemptSummary.trimCase = struct();
bestAttemptSummary.seed_name = '';

attempts = repmat(localAttemptTemplate(), 0, 1);

for iAttempt = 1:min(numel(seedCandidates), config.max_seed_attempts)
    seed = seedCandidates(iAttempt);
    case_i = localBuildCaseFromTarget(target, seed);

    fprintf('  attempt %d/%d with seed "%s"\n', ...
        iAttempt, min(numel(seedCandidates), config.max_seed_attempts), seed.name);

    attempt = localAttemptTemplate();
    attempt.index = iAttempt;
    attempt.seed_name = seed.name;
    attempt.case_name = case_i.name;
    attempt.trim_case = case_i;

    try
        [result_i, ~] = trim_evtol_case(initData, case_i, config.trim_options);
        attempt.success = logical(result_i.success);
        attempt.termination_string = result_i.terminationString;
        attempt.max_state_residual = localMaxStateResidual(result_i.op_report);
        attempt.summary = localCompactTrimSummary(result_i);

        if attempt.success
            attempts(end + 1, 1) = attempt; %#ok<SAGROW>
            entry = localMakeEntry(target, case_i, result_i, attempts, seed.name, attempt.max_state_residual);
            return;
        end

        if attempt.max_state_residual < bestAttemptSummary.max_state_residual
            bestAttemptSummary.max_state_residual = attempt.max_state_residual;
            bestAttemptSummary.success = false;
            bestAttemptSummary.trimResult = result_i;
            bestAttemptSummary.trimCase = case_i;
            bestAttemptSummary.seed_name = seed.name;
        end
    catch ME
        attempt.success = false;
        attempt.termination_string = sprintf('EXCEPTION: %s', ME.message);
        attempt.max_state_residual = inf;
        attempt.error_identifier = ME.identifier;
        attempt.error_message = ME.message;
    end

    attempts(end + 1, 1) = attempt; %#ok<SAGROW>
end

if isempty(fieldnames(bestAttemptSummary.trimResult))
    entry = localEntryTemplate();
    entry.key = localTargetKey(target);
    entry.name = target.name;
    entry.phase = target.phase;
    entry.family = target.family;
    entry.target_tilt_deg = target.tilt_deg;
    entry.target_vinf_mps = target.vinf_mps;
    entry.target_rear_fixed_rpm = target.rear_fixed_rpm;
    entry.success = false;
    entry.termination_string = 'No usable trim result returned.';
    entry.attempt_count = numel(attempts);
    entry.chosen_seed_name = '';
    entry.attempts = attempts;
    return;
end

entry = localMakeEntry(target, bestAttemptSummary.trimCase, bestAttemptSummary.trimResult, ...
    attempts, bestAttemptSummary.seed_name, bestAttemptSummary.max_state_residual);
entry.success = false;
end

function seedCandidates = localBuildSeedCandidates(target, allSeedEntries, config)
seedCandidates = repmat(localSeedTemplate(), 0, 1);

historySeed = localInterpolatedHistorySeed(target, allSeedEntries, config.distance_weights);
physicsSeed = localForceBalanceSeed(target, config.aircraft_seed_model, historySeed);

if isstruct(physicsSeed) && ~isempty(physicsSeed.name)
    seedCandidates(end + 1, 1) = physicsSeed; %#ok<SAGROW>
end
if isstruct(historySeed) && ~isempty(historySeed.name)
    seedCandidates(end + 1, 1) = historySeed; %#ok<SAGROW>
end

defaultSeed = localDefaultSeedForTarget(target);
seedCandidates(end + 1, 1) = defaultSeed; %#ok<SAGROW>

successEntries = allSeedEntries([allSeedEntries.success]);
if ~isempty(successEntries)
    distances = inf(numel(successEntries), 1);
    for i = 1:numel(successEntries)
        distances(i) = localEntryDistanceToTarget(successEntries(i), target, config.distance_weights);
    end
    [~, order] = sort(distances, 'ascend');
    for i = 1:numel(order)
        seedCandidates(end + 1, 1) = localSeedFromEntry(successEntries(order(i))); %#ok<SAGROW>
    end
end

seedCandidates = localUniqueSeeds(seedCandidates);
end

function case_i = localBuildCaseFromTarget(target, seed)
case_i = TrimCase_Cruise75_FlapElevator();
case_i.name = target.name;
case_i.mode = 'transition_low_speed';
case_i.Vinf_mps = target.vinf_mps;
case_i.u_body_mps = target.vinf_mps;
case_i.v_body_mps = 0.0;
case_i.w_body_mps = 0.0;
case_i.front_tilt_deg = target.tilt_deg;
case_i.front_tilt_cmd_deg = target.tilt_deg;
case_i.rear_collective_known = true;
case_i.rear_collective_fixed_rpm = target.rear_fixed_rpm;
case_i.rear_collective_guess_rpm = target.rear_fixed_rpm;
case_i.use_vinf_output_constraint = true;
case_i.use_vertical_speed_output_constraint = true;
case_i.position_steady = [false; false; false];
case_i.validate_nonlinear_hold = false;

case_i.front_collective_guess_rpm = seed.front_collective_guess_rpm;
case_i.theta_guess_deg = seed.theta_guess_deg;
case_i.delta_f_guess_deg = seed.delta_f_guess_deg;
case_i.delta_e_guess_deg = seed.delta_e_guess_deg;
end

function seed = localDefaultSeedForTarget(target)
case_i = TrimCase_Cruise75_FlapElevator();
seed = localSeedTemplate();
seed.name = 'default_low_speed_family';
seed.front_collective_guess_rpm = localGetField(case_i, 'front_collective_guess_rpm', 808.9);
seed.rear_collective_guess_rpm = target.rear_fixed_rpm;
seed.theta_guess_deg = 0.0;
seed.delta_f_guess_deg = localGetField(case_i, 'delta_f_guess_deg', 0.803);
seed.delta_e_guess_deg = localGetField(case_i, 'delta_e_guess_deg', -4.343);
end

function seed = localInterpolatedHistorySeed(target, allSeedEntries, weights)
seed = localSeedTemplate();
seed.name = '';

successEntries = allSeedEntries([allSeedEntries.success]);
if isempty(successEntries)
    return;
end

distances = [];
keepIdx = [];
for i = 1:numel(successEntries)
    entry = successEntries(i);
    d = localEntryDistanceToTarget(entry, target, weights);
    if isfinite(d)
        distances(end + 1, 1) = d; %#ok<AGROW>
        keepIdx(end + 1, 1) = i; %#ok<AGROW>
    end
end
if isempty(distances)
    return;
end

[distances, order] = sort(distances, 'ascend');
order = keepIdx(order);
count = min(6, numel(order));
subset = successEntries(order(1:count));
weights_i = 1 ./ (0.25 + distances(1:count)).^2;
weights_i = weights_i / sum(weights_i);

seed.name = 'interpolated_history';
seed.front_collective_guess_rpm = sum(weights_i .* [subset.front_collective_rpm]');
seed.rear_collective_guess_rpm = target.rear_fixed_rpm;
seed.theta_guess_deg = sum(weights_i .* [subset.theta_deg]');
seed.delta_f_guess_deg = sum(weights_i .* [subset.delta_f_deg]');
seed.delta_e_guess_deg = sum(weights_i .* [subset.delta_e_deg]');
end

function seed = localForceBalanceSeed(target, seedModel, historySeed)
seed = localSeedTemplate();
seed.name = 'force_balance';

tilt_deg = target.tilt_deg;
vinf_mps = target.vinf_mps;
rearFixedRpm = target.rear_fixed_rpm;

if isstruct(historySeed) && ~isempty(historySeed.name)
    alphaGuessDeg = max(0.0, historySeed.theta_guess_deg);
    deltaFBaseDeg = historySeed.delta_f_guess_deg;
    deltaEBaseDeg = historySeed.delta_e_guess_deg;
else
    alphaGuessDeg = max(0.0, 4.0 * (1.0 - min(vinf_mps / 75.0, 1.0)) * cosd(min(tilt_deg, 90.0)));
    deltaFBaseDeg = 0.0;
    deltaEBaseDeg = -2.0 * sind(min(tilt_deg, 90.0)) * max(0.0, 1.0 - vinf_mps / 75.0);
end

q = 0.5 * seedModel.rho * vinf_mps^2;
alphaRad = deg2rad(alphaGuessDeg);
clBase = max(0.0, seedModel.wing_CL0 + seedModel.wing_CLa * alphaRad);
liftBase = q * seedModel.wing_S * clBase;
cdBase = seedModel.wing_CD0 + seedModel.wing_CDa * (alphaRad - seedModel.wing_a0)^2;
dragBase = q * seedModel.wing_S * cdBase;

rearThrust = localGroupThrustFromRpm(rearFixedRpm, seedModel.rear_rotor_count, seedModel.k_thrust);
tiltSin = max(sind(tilt_deg), 0.05);
tiltCos = max(cosd(tilt_deg), 0.15);

frontFromDrag = dragBase / tiltSin;
frontFromVertical = max((seedModel.weight_N - liftBase - rearThrust) / tiltCos, 0.0);
frontVerticalShare = seedModel.front_vertical_share * max(seedModel.weight_N - liftBase, 0.0);
frontFromHover = frontVerticalShare / tiltCos;
frontThrust = max([0.0, frontFromDrag, frontFromVertical, frontFromHover]);

liftGap = seedModel.weight_N - rearThrust - frontThrust * cosd(tilt_deg) - liftBase;
deltaFGuessRad = deg2rad(deltaFBaseDeg);
if q > 1.0
    deltaFGuessRad = deltaFGuessRad + liftGap / (q * seedModel.wing_S * seedModel.CL_df);
end
deltaFGuessDeg = max(-5.0, min(20.0, rad2deg(deltaFGuessRad)));

seed.front_collective_guess_rpm = localRpmFromGroupThrust(frontThrust, seedModel.front_rotor_count, seedModel.k_thrust);
seed.rear_collective_guess_rpm = rearFixedRpm;
seed.theta_guess_deg = alphaGuessDeg;
seed.delta_f_guess_deg = deltaFGuessDeg;
seed.delta_e_guess_deg = max(-15.0, min(15.0, deltaEBaseDeg));
end

function rpm = localRpmFromGroupThrust(groupThrustN, rotorCount, kThrust)
perRotor = max(groupThrustN / max(rotorCount, 1), 0.0);
rpm = sqrt(perRotor / kThrust);
end

function thrust = localGroupThrustFromRpm(rpm, rotorCount, kThrust)
thrust = rotorCount * kThrust * rpm^2;
end

function seed = localSeedFromEntry(entry)
seed = localSeedTemplate();
seed.name = ['entry_' entry.name];
seed.front_collective_guess_rpm = entry.front_collective_rpm;
seed.rear_collective_guess_rpm = entry.rear_collective_rpm;
seed.theta_guess_deg = entry.theta_deg;
seed.delta_f_guess_deg = entry.delta_f_deg;
seed.delta_e_guess_deg = entry.delta_e_deg;
end

function entry = localMakeEntry(target, case_i, result_i, attempts, seed_name, maxResidual)
summary = localCompactTrimSummary(result_i);

entry = localEntryTemplate();
entry.key = localTargetKey(target);
entry.name = target.name;
entry.phase = target.phase;
entry.family = target.family;
entry.target_tilt_deg = target.tilt_deg;
entry.target_vinf_mps = target.vinf_mps;
entry.target_rear_fixed_rpm = target.rear_fixed_rpm;
entry.success = logical(result_i.success);
entry.termination_string = result_i.terminationString;
entry.attempt_count = numel(attempts);
entry.chosen_seed_name = seed_name;
entry.max_state_residual = maxResidual;
entry.front_collective_rpm = summary.front_collective_rpm;
entry.rear_collective_rpm = summary.rear_collective_rpm;
entry.delta_f_deg = summary.delta_f_deg;
entry.delta_a_deg = summary.delta_a_deg;
entry.delta_e_deg = summary.delta_e_deg;
entry.delta_r_deg = summary.delta_r_deg;
entry.theta_deg = summary.theta_deg;
entry.u_mps = summary.u_mps;
entry.v_mps = summary.v_mps;
entry.w_mps = summary.w_mps;
entry.alpha_deg = summary.alpha_deg;
entry.beta_deg = summary.beta_deg;
entry.trim_case = case_i;
entry.trim_summary = summary;
entry.attempts = attempts;
end

function summary = localCompactTrimSummary(result_i)
summary = struct();
summary.name = result_i.name;
summary.mode = result_i.mode;
summary.success = logical(result_i.success);
summary.termination_string = result_i.terminationString;
summary.front_tilt_deg = result_i.scheduling.front_tilt_deg;
summary.front_collective_rpm = result_i.scheduling.front_collective_rpm;
summary.rear_collective_rpm = result_i.scheduling.rear_collective_rpm;
summary.delta_f_deg = rad2deg(result_i.scheduling.delta_f_rad);
summary.delta_a_deg = rad2deg(result_i.scheduling.delta_a_rad);
summary.delta_e_deg = rad2deg(result_i.scheduling.delta_e_rad);
summary.delta_r_deg = rad2deg(result_i.scheduling.delta_r_rad);
summary.phi_deg = result_i.Att_Trim_deg(1);
summary.theta_deg = result_i.Att_Trim_deg(2);
summary.psi_deg = result_i.Att_Trim_deg(3);
summary.u_mps = result_i.Vel_B_BA_Trim(1);
summary.v_mps = result_i.Vel_B_BA_Trim(2);
summary.w_mps = result_i.Vel_B_BA_Trim(3);
summary.vinf_mps = result_i.scheduling.Vinf_mps;
summary.alpha_deg = rad2deg(result_i.scheduling.alpha_rad);
summary.beta_deg = rad2deg(result_i.scheduling.beta_rad);
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
    s = states(i);
    try
        dx = s.dx;
    catch
        dx = [];
    end
    try
        steadyMask = logical(s.SteadyState);
    catch
        steadyMask = true(size(dx));
    end

    if ~isempty(dx)
        dx = dx(:);
        if isempty(steadyMask)
            steadyMask = true(size(dx));
        else
            steadyMask = logical(steadyMask(:));
            if numel(steadyMask) ~= numel(dx)
                steadyMask = true(size(dx));
            end
        end
        residuals = [residuals; abs(dx(steadyMask))]; %#ok<AGROW>
    end
end

if ~isempty(residuals)
    maxResidual = max(residuals);
end
end

function [summaryTable, mergedSummary, trimMap] = localCheckpoint(trimMap, baseSummary, checkpointFile, latestMat, csvFile, latestCsv, mdFile, latestMd, mergedCsv, mergedMd)
summaryTable = localBuildSummaryTable(trimMap.entries);
trimMap.summary_table = summaryTable;
if ~isempty(summaryTable) && any(strcmp(summaryTable.Properties.VariableNames, 'success'))
    trimMap.progress.success_count = nnz(summaryTable.success);
    trimMap.progress.failure_count = height(summaryTable) - trimMap.progress.success_count;
else
    trimMap.progress.success_count = 0;
    trimMap.progress.failure_count = 0;
end

save(checkpointFile, 'trimMap', 'summaryTable', '-v7.3');
save(latestMat, 'trimMap', 'summaryTable', '-v7.3');

writetable(summaryTable, csvFile);
writetable(summaryTable, latestCsv);

mergedSummary = localMergeSummaries(baseSummary, summaryTable);
if ~isempty(mergedSummary)
    writetable(mergedSummary, mergedCsv);
end

localWriteSummaryMarkdown(mdFile, trimMap, summaryTable);
localWriteSummaryMarkdown(latestMd, trimMap, summaryTable);
localWriteMergedMarkdown(mergedMd, baseSummary, summaryTable, mergedSummary);
end

function mergedSummary = localMergeSummaries(baseSummary, newSummary)
if isempty(baseSummary)
    mergedSummary = newSummary;
    return;
end
if isempty(newSummary)
    mergedSummary = baseSummary;
    return;
end

baseSummary = localEnsureStringColumns(baseSummary);
newSummary = localEnsureStringColumns(newSummary);
mergedSummary = [baseSummary; newSummary];
key = strcat(mergedSummary.family, "__", string(mergedSummary.tilt_deg), "__", ...
    string(mergedSummary.vinf_mps), "__", string(mergedSummary.rear_fixed_rpm));
[~, ia] = unique(key, 'stable');
mergedSummary = mergedSummary(sort(ia), :);
mergedSummary = sortrows(mergedSummary, {'success', 'tilt_deg', 'vinf_mps', 'rear_fixed_rpm'}, ...
    {'descend', 'ascend', 'ascend', 'ascend'});
end

function tbl = localEnsureStringColumns(tbl)
if isempty(tbl)
    return;
end
vars = {'name', 'phase', 'family', 'termination'};
for i = 1:numel(vars)
    if any(strcmp(tbl.Properties.VariableNames, vars{i})) && ~isstring(tbl.(vars{i}))
        tbl.(vars{i}) = string(tbl.(vars{i}));
    end
end
end

function summaryTable = localBuildSummaryTable(entries)
if isempty(entries)
    summaryTable = table();
    return;
end

n = numel(entries);
summaryTable = table( ...
    strings(n, 1), strings(n, 1), strings(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
    false(n, 1), strings(n, 1), zeros(n, 1), zeros(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
    'VariableNames', { ...
        'name', 'phase', 'family', ...
        'tilt_deg', 'vinf_mps', 'rear_fixed_rpm', ...
        'success', 'termination', 'max_state_residual', 'attempt_count', ...
        'front_collective_rpm', 'rear_collective_rpm', ...
        'delta_f_deg', 'delta_e_deg', ...
        'theta_deg', 'u_mps', 'w_mps', 'alpha_deg'});

for i = 1:n
    entry = entries(i);
    summaryTable.name(i) = string(entry.name);
    summaryTable.phase(i) = string(entry.phase);
    summaryTable.family(i) = string(entry.family);
    summaryTable.tilt_deg(i) = entry.target_tilt_deg;
    summaryTable.vinf_mps(i) = entry.target_vinf_mps;
    summaryTable.rear_fixed_rpm(i) = entry.target_rear_fixed_rpm;
    summaryTable.success(i) = entry.success;
    summaryTable.termination(i) = string(entry.termination_string);
    summaryTable.max_state_residual(i) = entry.max_state_residual;
    summaryTable.attempt_count(i) = entry.attempt_count;
    summaryTable.front_collective_rpm(i) = entry.front_collective_rpm;
    summaryTable.rear_collective_rpm(i) = entry.rear_collective_rpm;
    summaryTable.delta_f_deg(i) = entry.delta_f_deg;
    summaryTable.delta_e_deg(i) = entry.delta_e_deg;
    summaryTable.theta_deg(i) = entry.theta_deg;
    summaryTable.u_mps(i) = entry.u_mps;
    summaryTable.w_mps(i) = entry.w_mps;
    summaryTable.alpha_deg(i) = entry.alpha_deg;
end

summaryTable = sortrows(summaryTable, {'success', 'phase', 'tilt_deg', 'vinf_mps', 'rear_fixed_rpm'}, ...
    {'descend', 'ascend', 'ascend', 'ascend', 'ascend'});
end

function localWriteSummaryMarkdown(filename, trimMap, summaryTable)
fid = fopen(filename, 'w');
if fid < 0
    warning('Run_Trim_Transition_Map_LowSpeed:SummaryWriteFailed', ...
        'Could not open %s for writing.', filename);
    return;
end
cleanupFile = onCleanup(@() fclose(fid));

fprintf(fid, '# Low-Speed Transition Trim Map Summary\n\n');
fprintf(fid, 'Generated: %s\n\n', trimMap.meta.created_on);
fprintf(fid, '- trim model: `%s`\n', trimMap.meta.trim_model);
fprintf(fid, '- run model: `%s`\n', trimMap.meta.run_model);
fprintf(fid, '- total targets attempted: %d\n', height(summaryTable));
fprintf(fid, '- exact successes: %d\n', nnz(summaryTable.success));
fprintf(fid, '- failures / inexact points: %d\n\n', height(summaryTable) - nnz(summaryTable.success));

if isempty(summaryTable)
    fprintf(fid, 'No entries yet.\n');
    return;
end

phases = unique(summaryTable.phase, 'stable');
fprintf(fid, '## Success Counts by Phase\n\n');
fprintf(fid, '| phase | exact successes | total |\n');
fprintf(fid, '| --- | ---: | ---: |\n');
for i = 1:numel(phases)
    ph = phases(i);
    mask = summaryTable.phase == ph;
    fprintf(fid, '| %s | %d | %d |\n', ph, nnz(summaryTable.success(mask)), nnz(mask));
end

fprintf(fid, '\n## First 40 Exact Successes\n\n');
fprintf(fid, '| name | phase | tilt (deg) | V (m/s) | rear fixed (rpm) | front (rpm) | df (deg) | de (deg) | theta (deg) | alpha (deg) |\n');
fprintf(fid, '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n');
successRows = summaryTable(summaryTable.success, :);
limit = min(40, height(successRows));
for i = 1:limit
    row = successRows(i, :);
    fprintf(fid, '| %s | %s | %.1f | %.1f | %.1f | %.3f | %.3f | %.3f | %.3f | %.3f |\n', ...
        row.name, row.phase, row.tilt_deg, row.vinf_mps, row.rear_fixed_rpm, ...
        row.front_collective_rpm, row.delta_f_deg, row.delta_e_deg, row.theta_deg, row.alpha_deg);
end
end

function localWriteMergedMarkdown(filename, baseSummary, newSummary, mergedSummary)
fid = fopen(filename, 'w');
if fid < 0
    warning('Run_Trim_Transition_Map_LowSpeed:MergedWriteFailed', ...
        'Could not open %s for writing.', filename);
    return;
end
cleanupFile = onCleanup(@() fclose(fid));

fprintf(fid, '# Merged Transition Trim Summary\n\n');
fprintf(fid, '- base rows: %d\n', height(baseSummary));
fprintf(fid, '- new low-speed rows: %d\n', height(newSummary));
fprintf(fid, '- merged rows: %d\n', height(mergedSummary));
fprintf(fid, '- merged exact successes: %d\n\n', nnz(mergedSummary.success));

if isempty(mergedSummary)
    fprintf(fid, 'No merged data yet.\n');
    return;
end

fprintf(fid, '## Exact Success Counts by Tilt\n\n');
fprintf(fid, '| tilt (deg) | exact successes |\n');
fprintf(fid, '| ---: | ---: |\n');
tilts = unique(mergedSummary.tilt_deg(mergedSummary.success));
for i = 1:numel(tilts)
    t = tilts(i);
    mask = mergedSummary.success & abs(mergedSummary.tilt_deg - t) < 1e-9;
    fprintf(fid, '| %.1f | %d |\n', t, nnz(mask));
end
end

function tf = localHasKey(entries, key)
tf = false;
for i = 1:numel(entries)
    if strcmp(entries(i).key, key)
        tf = true;
        return;
    end
end
end

function key = localTargetKey(target)
rearLabel = localValueLabel(target.rear_fixed_rpm);
key = sprintf('%s__tilt_%s__V_%s__rear_%s', ...
    target.family, localValueLabel(target.tilt_deg), localValueLabel(target.vinf_mps), rearLabel);
end

function d = localEntryDistanceToTarget(entry, target, weights)
rearEntry = entry.target_rear_fixed_rpm;
rearTarget = target.rear_fixed_rpm;

rearTerm = 0;
if ~(isnan(rearEntry) && isnan(rearTarget))
    if isnan(rearEntry) || isnan(rearTarget)
        rearTerm = 5.0;
    else
        rearTerm = abs(rearEntry - rearTarget) * weights.rear_rpm;
    end
end

d = abs(entry.target_tilt_deg - target.tilt_deg) * weights.tilt_deg + ...
    abs(entry.target_vinf_mps - target.vinf_mps) * weights.vinf_mps + ...
    rearTerm;

if ~strcmp(entry.family, target.family)
    d = d + weights.family_mismatch;
end
end

function label = localValueLabel(value)
label = strtrim(num2str(value, '%.4g'));
label = strrep(label, '.', 'p');
label = strrep(label, '-', 'm');
end

function label = localFormatRearLabel(rear)
if isnan(rear)
    label = 'free';
else
    label = sprintf('%.1f', rear);
end
end

function targets = localUniqueTargets(targets)
keys = cell(numel(targets), 1);
for i = 1:numel(targets)
    keys{i} = localTargetKey(targets(i));
end
[~, ia] = unique(keys, 'stable');
targets = targets(sort(ia));
end

function seeds = localUniqueSeeds(seeds)
if isempty(seeds)
    return;
end

keys = cell(numel(seeds), 1);
for i = 1:numel(seeds)
    keys{i} = sprintf('%s__front_%s__rear_%s__theta_%s__df_%s__de_%s', ...
        seeds(i).name, ...
        localValueLabel(seeds(i).front_collective_guess_rpm), ...
        localValueLabel(seeds(i).rear_collective_guess_rpm), ...
        localValueLabel(seeds(i).theta_guess_deg), ...
        localValueLabel(seeds(i).delta_f_guess_deg), ...
        localValueLabel(seeds(i).delta_e_guess_deg));
end
[~, ia] = unique(keys, 'stable');
seeds = seeds(sort(ia));
end

function target = localTargetTemplate()
target = struct( ...
    'phase', '', ...
    'family', '', ...
    'name', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'rear_fixed_rpm', NaN);
end

function entry = localEntryTemplate()
entry = struct( ...
    'key', '', ...
    'name', '', ...
    'phase', '', ...
    'family', '', ...
    'target_tilt_deg', NaN, ...
    'target_vinf_mps', NaN, ...
    'target_rear_fixed_rpm', NaN, ...
    'success', false, ...
    'termination_string', '', ...
    'attempt_count', 0, ...
    'chosen_seed_name', '', ...
    'max_state_residual', inf, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'u_mps', NaN, ...
    'v_mps', NaN, ...
    'w_mps', NaN, ...
    'alpha_deg', NaN, ...
    'beta_deg', NaN, ...
    'trim_case', struct(), ...
    'trim_summary', struct(), ...
    'attempts', repmat(localAttemptTemplate(), 0, 1));
end

function attempt = localAttemptTemplate()
attempt = struct( ...
    'index', 0, ...
    'seed_name', '', ...
    'case_name', '', ...
    'success', false, ...
    'termination_string', '', ...
    'max_state_residual', inf, ...
    'error_identifier', '', ...
    'error_message', '', ...
    'trim_case', struct(), ...
    'summary', struct());
end

function seed = localSeedTemplate()
seed = struct( ...
    'name', '', ...
    'front_collective_guess_rpm', 0.0, ...
    'rear_collective_guess_rpm', 0.0, ...
    'theta_guess_deg', 0.0, ...
    'delta_f_guess_deg', 0.0, ...
    'delta_e_guess_deg', 0.0);
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end
