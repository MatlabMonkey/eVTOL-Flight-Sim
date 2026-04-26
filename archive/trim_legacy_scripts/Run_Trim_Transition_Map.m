% Run_Trim_Transition_Map.m
% Build a large continuation-based trim map between hover and cruise.
%
% This is intended for unattended / overnight exploration.
%
% Outputs left in the base workspace:
%   - transitionTrimMap
%   - transitionTrimSummary
%   - transitionTrimOutputDir
%   - transitionTrimCheckpointFile
%   - transitionTrimLogFile
%
% Main on-disk outputs:
%   workspace_plots/transition_trim_map_<timestamp>/
%     transition_trim_map.mat
%     transition_trim_map_summary.csv
%     transition_trim_map_summary.md
%     transition_trim_map.log
%
% Latest convenience copies:
%   workspace_plots/transition_trim_map_latest.mat
%   workspace_plots/transition_trim_map_latest.csv
%   workspace_plots/transition_trim_map_latest.md

if ~exist('transitionMapOptions', 'var') || ~isstruct(transitionMapOptions)
    transitionMapOptions = struct();
end

initOptions = struct();
initOptions.transitionMapOptions = transitionMapOptions;

Init_EVTOL_Main

if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'transitionMapOptions')
    transitionMapOptions = initOptions.transitionMapOptions;
else
    transitionMapOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionTrimOutputDir = fullfile(root_dir, 'workspace_plots', ['transition_trim_map_' timestamp]);
if exist(transitionTrimOutputDir, 'dir') ~= 7
    mkdir(transitionTrimOutputDir);
end

transitionTrimCheckpointFile = fullfile(transitionTrimOutputDir, 'transition_trim_map.mat');
transitionTrimSummaryCsv = fullfile(transitionTrimOutputDir, 'transition_trim_map_summary.csv');
transitionTrimSummaryMd = fullfile(transitionTrimOutputDir, 'transition_trim_map_summary.md');
transitionTrimLogFile = fullfile(transitionTrimOutputDir, 'transition_trim_map.log');

transitionTrimLatestMat = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_latest.mat');
transitionTrimLatestCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_latest.csv');
transitionTrimLatestMd = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_latest.md');

diary off;
diary(transitionTrimLogFile);

fprintf('=== Run_Trim_Transition_Map ===\n');
fprintf('Started: %s\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf('Output dir: %s\n', transitionTrimOutputDir);

config = localBuildConfig(transitionMapOptions, root_dir);
transitionTrimTargets = localBuildTargets(config);

fprintf('Configured %d target trim points across %d phases.\n', ...
    numel(transitionTrimTargets), numel(unique({transitionTrimTargets.phase})));

transitionTrimMap = struct();
transitionTrimMap.meta = struct();
transitionTrimMap.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
transitionTrimMap.meta.root_dir = root_dir;
transitionTrimMap.meta.output_dir = transitionTrimOutputDir;
transitionTrimMap.meta.notes_file = fullfile(root_dir, 'TRIM_EXPLORATION_NOTES.md');
transitionTrimMap.meta.trim_model = initData.modelNames.trim;
transitionTrimMap.meta.run_model = initData.modelNames.run;
transitionTrimMap.meta.config = config;
transitionTrimMap.meta.target_count = numel(transitionTrimTargets);
transitionTrimMap.progress = struct('completed', 0, 'success_count', 0, 'failure_count', 0, 'last_key', '');
transitionTrimMap.targets = transitionTrimTargets;
transitionTrimMap.entries = repmat(localEntryTemplate(), 0, 1);

transitionTrimSummary = table();

cleanupDiary = onCleanup(@() diary('off'));

try
    for iTarget = 1:numel(transitionTrimTargets)
        target = transitionTrimTargets(iTarget);
        key = localTargetKey(target);

        if localHasKey(transitionTrimMap.entries, key)
            continue;
        end

        fprintf('\n[%d/%d] %s | family=%s | tilt=%.1f | V=%.1f | rear=%s\n', ...
            iTarget, numel(transitionTrimTargets), target.name, target.family, ...
            target.tilt_deg, target.vinf_mps, localFormatRearLabel(target.rear_fixed_rpm));

        entry = localSolveTarget(target, transitionTrimMap.entries, initData, config);
        transitionTrimMap.entries(end + 1, 1) = entry; %#ok<SAGROW>
        transitionTrimMap.progress.completed = numel(transitionTrimMap.entries);
        transitionTrimMap.progress.success_count = nnz([transitionTrimMap.entries.success]);
        transitionTrimMap.progress.failure_count = transitionTrimMap.progress.completed - transitionTrimMap.progress.success_count;
        transitionTrimMap.progress.last_key = key;

        fprintf('  result: success=%d | maxResidual=%.6g | attempts=%d\n', ...
            entry.success, entry.max_state_residual, entry.attempt_count);
        fprintf('  termination: %s\n', entry.termination_string);

        if mod(iTarget, config.checkpoint_every) == 0 || entry.success || iTarget == numel(transitionTrimTargets)
            [transitionTrimSummary, transitionTrimMap] = localCheckpoint( ...
                transitionTrimMap, transitionTrimCheckpointFile, transitionTrimLatestMat, ...
                transitionTrimSummaryCsv, transitionTrimLatestCsv, ...
                transitionTrimSummaryMd, transitionTrimLatestMd);
            fprintf('  checkpoint saved.\n');
        end
    end

    [transitionTrimSummary, transitionTrimMap] = localCheckpoint( ...
        transitionTrimMap, transitionTrimCheckpointFile, transitionTrimLatestMat, ...
        transitionTrimSummaryCsv, transitionTrimLatestCsv, ...
        transitionTrimSummaryMd, transitionTrimLatestMd);

    fprintf('\nCompleted transition trim map.\n');
    fprintf('  total entries = %d\n', height(transitionTrimSummary));
    fprintf('  exact successes = %d\n', nnz(transitionTrimSummary.success));
    fprintf('  checkpoint = %s\n', transitionTrimCheckpointFile);
catch ME
    fprintf('\nRun_Trim_Transition_Map failed: %s\n', ME.message);
    [transitionTrimSummary, transitionTrimMap] = localCheckpoint( ...
        transitionTrimMap, transitionTrimCheckpointFile, transitionTrimLatestMat, ...
        transitionTrimSummaryCsv, transitionTrimLatestCsv, ...
        transitionTrimSummaryMd, transitionTrimLatestMd);
    rethrow(ME);
end

function config = localBuildConfig(userOptions, root_dir)
config = struct();
config.root_dir = root_dir;
config.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 10);
config.max_seed_attempts = localGetField(userOptions, 'max_seed_attempts', 6);
config.target_limit = localGetField(userOptions, 'target_limit', inf);
config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);

% Use nearby exact trims first. Same-family seeds get a smaller distance.
config.distance_weights = struct( ...
    'tilt_deg', 1 / 5, ...
    'vinf_mps', 1 / 5, ...
    'rear_rpm', 1 / 100, ...
    'family_mismatch', 1.0);
end

function targets = localBuildTargets(config) %#ok<INUSD>
targets = repmat(localTargetTemplate(), 0, 1);

% Phase 1: exact anchors that we already trust.
targets(end + 1) = localMakeNamedCaseTarget('anchor_hover', TrimCase_Hover(), 0.0, 0.0, NaN); %#ok<SAGROW>

rear_anchor_list = [0, 200, 500];
for rear = rear_anchor_list
    target = localMakeFlapElevatorTarget('anchor_cruise90', 90.0, 75.0, rear);
    targets(end + 1) = target; %#ok<SAGROW>
end

target = localMakeFlapElevatorTarget('anchor_mid45', 45.0, 75.0, 500.0);
target.manual_seed = struct( ...
    'name', 'notes_seed_45deg_75ms_rear500', ...
    'front_collective_guess_rpm', 1000.0, ...
    'rear_collective_guess_rpm', 500.0, ...
    'theta_guess_deg', 0.0, ...
    'delta_f_guess_deg', -0.8, ...
    'delta_e_guess_deg', -3.2);
targets(end + 1) = target; %#ok<SAGROW>

% Phase 2: 90-deg exact rear family at cruise.
for rear = 0:100:700
    targets(end + 1) = localMakeFlapElevatorTarget('family_90deg_rear', 90.0, 75.0, rear); %#ok<SAGROW>
end

% Phase 3: 45-deg exact rear family at cruise.
for rear = 0:100:700
    target = localMakeFlapElevatorTarget('family_45deg_rear', 45.0, 75.0, rear);
    if rear == 500
        target.manual_seed = struct( ...
            'name', 'notes_seed_45deg_75ms_rear500', ...
            'front_collective_guess_rpm', 1000.0, ...
            'rear_collective_guess_rpm', 500.0, ...
            'theta_guess_deg', 0.0, ...
            'delta_f_guess_deg', -0.8, ...
            'delta_e_guess_deg', -3.2);
    end
    targets(end + 1) = target; %#ok<SAGROW>
end

% Phase 4: constant-speed tilt continuation families at 75 m/s.
for rear = [0, 200, 500, 700]
    for tilt = 90:-5:0
        targets(end + 1) = localMakeFlapElevatorTarget('tilt_line_speed75', tilt, 75.0, rear); %#ok<SAGROW>
    end
end

% Phase 5: speed refinement around the known 45-deg pocket.
for rear = 0:100:700
    for speed = 45:2.5:80
        target = localMakeFlapElevatorTarget('speed_line_tilt45', 45.0, speed, rear);
        if rear == 500 && abs(speed - 75.0) < 1e-6
            target.manual_seed = struct( ...
                'name', 'notes_seed_45deg_75ms_rear500', ...
                'front_collective_guess_rpm', 1000.0, ...
                'rear_collective_guess_rpm', 500.0, ...
                'theta_guess_deg', 0.0, ...
                'delta_f_guess_deg', -0.8, ...
                'delta_e_guess_deg', -3.2);
        end
        targets(end + 1) = target; %#ok<SAGROW>
    end
end

% Phase 6: broader corridor with the rear-fixed flap/elevator family.
for rear = 0:100:700
    for tilt = 30:5:90
        for speed = 40:5:75
            targets(end + 1) = localMakeFlapElevatorTarget('transition_corridor', tilt, speed, rear); %#ok<SAGROW>
        end
    end
end

% Phase 7: low-speed hover-side exploration with surfaces pinned out.
for tilt = 0:5:30
    for speed = 2.5:2.5:25
        targets(end + 1) = localMakeHoverFamilyTarget('hover_side', tilt, speed); %#ok<SAGROW>
    end
end

targets = localUniqueTargets(targets);
targets = localSortTargets(targets);
if isfinite(config.target_limit)
    targets = targets(1:min(numel(targets), max(1, round(config.target_limit))));
end
end

function target = localMakeNamedCaseTarget(phase, caseStruct, tilt_deg, vinf_mps, rear_fixed_rpm)
target = localTargetTemplate();
target.phase = phase;
target.family = 'named_case';
target.name = char(caseStruct.name);
target.tilt_deg = tilt_deg;
target.vinf_mps = vinf_mps;
target.rear_fixed_rpm = rear_fixed_rpm;
target.case_struct = caseStruct;
end

function target = localMakeFlapElevatorTarget(phase, tilt_deg, vinf_mps, rear_fixed_rpm)
target = localTargetTemplate();
target.phase = phase;
target.family = 'cruise_flap_elevator_rear_fixed';
target.tilt_deg = tilt_deg;
target.vinf_mps = vinf_mps;
target.rear_fixed_rpm = rear_fixed_rpm;
target.name = sprintf('FE_tilt_%s__V_%s__rear_%s', ...
    localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(rear_fixed_rpm));
end

function target = localMakeHoverFamilyTarget(phase, tilt_deg, vinf_mps)
target = localTargetTemplate();
target.phase = phase;
target.family = 'hover_zero_surface';
target.tilt_deg = tilt_deg;
target.vinf_mps = vinf_mps;
target.rear_fixed_rpm = NaN;
target.name = sprintf('HoverLike_tilt_%s__V_%s', ...
    localValueLabel(tilt_deg), localValueLabel(vinf_mps));
end

function targets = localSortTargets(targets)
priority = zeros(numel(targets), 1);
for i = 1:numel(targets)
    target = targets(i);
    switch target.phase
        case 'anchor_hover'
            base = 0;
        case 'anchor_cruise90'
            base = 10;
        case 'anchor_mid45'
            base = 20;
        case {'family_90deg_rear', 'family_45deg_rear'}
            base = 30;
        case 'tilt_line_speed75'
            base = 40;
        case 'speed_line_tilt45'
            base = 50;
        case 'transition_corridor'
            base = 60;
        case 'hover_side'
            base = 70;
        otherwise
            base = 80;
    end

    rear_term = 0;
    if ~isnan(target.rear_fixed_rpm)
        rear_term = abs(target.rear_fixed_rpm - 500.0) / 100.0;
    end

    anchor_term = min(abs(target.tilt_deg - 90.0), abs(target.tilt_deg - 45.0)) / 5.0 + ...
                  abs(target.vinf_mps - 75.0) / 5.0 + rear_term;
    if strcmp(target.family, 'hover_zero_surface')
        anchor_term = abs(target.tilt_deg - 0.0) / 5.0 + abs(target.vinf_mps - 0.0) / 2.5;
    end

    priority(i) = base + anchor_term;
end

[~, order] = sort(priority, 'ascend');
targets = targets(order);
end

function [entry, config] = localSolveTarget(target, existingEntries, initData, config) %#ok<INUSD>
seedCandidates = localBuildSeedCandidates(target, existingEntries, config);

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

function seedCandidates = localBuildSeedCandidates(target, existingEntries, config)
seedCandidates = repmat(localSeedTemplate(), 0, 1);

    if isstruct(target.manual_seed) && ~isempty(fieldnames(target.manual_seed))
        manualSeed = localSeedTemplate();
        manualSeed.name = target.manual_seed.name;
        manualSeed.front_collective_guess_rpm = target.manual_seed.front_collective_guess_rpm;
    manualSeed.rear_collective_guess_rpm = target.manual_seed.rear_collective_guess_rpm;
    manualSeed.theta_guess_deg = target.manual_seed.theta_guess_deg;
    manualSeed.delta_f_guess_deg = localGetField(target.manual_seed, 'delta_f_guess_deg', 0.0);
    manualSeed.delta_e_guess_deg = localGetField(target.manual_seed, 'delta_e_guess_deg', 0.0);
    seedCandidates(end + 1, 1) = manualSeed; %#ok<SAGROW>
end

defaultSeed = localDefaultSeedForTarget(target);
seedCandidates(end + 1, 1) = defaultSeed; %#ok<SAGROW>

successEntries = existingEntries([existingEntries.success]);
if isempty(successEntries)
    seedCandidates = localUniqueSeeds(seedCandidates);
    return;
end

distances = inf(numel(successEntries), 1);
for i = 1:numel(successEntries)
    distances(i) = localEntryDistanceToTarget(successEntries(i), target, config.distance_weights);
end
    [~, order] = sort(distances, 'ascend');

for i = 1:numel(order)
    entry = successEntries(order(i));
    seedCandidates(end + 1, 1) = localSeedFromEntry(entry); %#ok<SAGROW>
end

seedCandidates = localUniqueSeeds(seedCandidates);
end

function case_i = localBuildCaseFromTarget(target, seed)
switch target.family
    case 'named_case'
        case_i = target.case_struct;
    case 'cruise_flap_elevator_rear_fixed'
        case_i = TrimCase_Cruise75_FlapElevator();
        case_i.name = target.name;
        case_i.mode = 'transition';
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
        case_i.validate_nonlinear_hold = false;

    case 'hover_zero_surface'
        case_i = TrimCase_Hover();
        case_i.name = target.name;
        case_i.mode = 'transition';
        case_i.Vinf_mps = target.vinf_mps;
        case_i.u_body_mps = target.vinf_mps;
        case_i.v_body_mps = 0.0;
        case_i.w_body_mps = 0.0;
        case_i.front_tilt_deg = target.tilt_deg;
        case_i.front_tilt_cmd_deg = target.tilt_deg;
        case_i.euler_known = [true; false; true];
        case_i.euler_steady = [true; true; true];
        case_i.body_velocity_known = [false; true; false];
        case_i.body_velocity_steady = [true; true; true];
        case_i.use_vinf_output_constraint = true;
        case_i.use_vertical_speed_output_constraint = true;
        case_i.mixed_control_known = [true; true; true; true];
        case_i.delta_f_fixed_deg = 0.0;
        case_i.delta_a_fixed_deg = 0.0;
        case_i.delta_e_fixed_deg = 0.0;
        case_i.delta_r_fixed_deg = 0.0;
        case_i.validate_nonlinear_hold = false;
    otherwise
        error('Unknown target family "%s".', target.family);
end

case_i.front_collective_guess_rpm = seed.front_collective_guess_rpm;
case_i.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
case_i.theta_guess_deg = seed.theta_guess_deg;

if isfield(case_i, 'delta_f_guess_deg') || strcmp(target.family, 'cruise_flap_elevator_rear_fixed')
    case_i.delta_f_guess_deg = seed.delta_f_guess_deg;
end
if isfield(case_i, 'delta_e_guess_deg') || strcmp(target.family, 'cruise_flap_elevator_rear_fixed')
    case_i.delta_e_guess_deg = seed.delta_e_guess_deg;
end
end

function seed = localDefaultSeedForTarget(target)
seed = localSeedTemplate();

switch target.family
    case 'named_case'
        case_i = target.case_struct;
        seed.name = ['default_' char(case_i.name)];
        seed.front_collective_guess_rpm = localGetField(case_i, 'front_collective_guess_rpm', 0.0);
        seed.rear_collective_guess_rpm = localGetField(case_i, 'rear_collective_guess_rpm', 0.0);
        seed.theta_guess_deg = localGetField(case_i, 'theta_guess_deg', 0.0);
        seed.delta_f_guess_deg = localGetField(case_i, 'delta_f_guess_deg', 0.0);
        seed.delta_e_guess_deg = localGetField(case_i, 'delta_e_guess_deg', 0.0);

    case 'cruise_flap_elevator_rear_fixed'
        case_i = TrimCase_Cruise75_FlapElevator();
        seed.name = 'default_cruise_flap_elevator';
        seed.front_collective_guess_rpm = localGetField(case_i, 'front_collective_guess_rpm', 808.9);
        seed.rear_collective_guess_rpm = target.rear_fixed_rpm;
        seed.theta_guess_deg = 0.0;
        seed.delta_f_guess_deg = localGetField(case_i, 'delta_f_guess_deg', 0.803);
        seed.delta_e_guess_deg = localGetField(case_i, 'delta_e_guess_deg', -4.343);

        if abs(target.tilt_deg - 45.0) < 1e-6
            seed.name = 'default_45deg_notes_seed';
            seed.front_collective_guess_rpm = 1000.0;
            seed.rear_collective_guess_rpm = target.rear_fixed_rpm;
            seed.theta_guess_deg = 0.0;
            seed.delta_f_guess_deg = -0.8;
            seed.delta_e_guess_deg = -3.2;
        end

    case 'hover_zero_surface'
        case_i = TrimCase_Hover();
        seed.name = 'default_hover_like';
        seed.front_collective_guess_rpm = localGetField(case_i, 'front_collective_guess_rpm', 1865.76);
        seed.rear_collective_guess_rpm = localGetField(case_i, 'rear_collective_guess_rpm', 1756.38);
        seed.theta_guess_deg = 0.0;
        seed.delta_f_guess_deg = 0.0;
        seed.delta_e_guess_deg = 0.0;

    otherwise
        error('Unknown target family "%s".', target.family);
end
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

function [summaryTable, trimMap] = localCheckpoint(trimMap, checkpointFile, latestMat, csvFile, latestCsv, mdFile, latestMd)
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

localWriteSummaryMarkdown(mdFile, trimMap, summaryTable);
localWriteSummaryMarkdown(latestMd, trimMap, summaryTable);
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

summaryTable = sortrows(summaryTable, {'success', 'phase', 'family', 'tilt_deg', 'vinf_mps', 'rear_fixed_rpm'}, ...
    {'descend', 'ascend', 'ascend', 'ascend', 'ascend', 'ascend'});
end

function localWriteSummaryMarkdown(filename, trimMap, summaryTable)
fid = fopen(filename, 'w');
if fid < 0
    warning('Run_Trim_Transition_Map:SummaryWriteFailed', ...
        'Could not open %s for writing.', filename);
    return;
end
cleanupFile = onCleanup(@() fclose(fid));

fprintf(fid, '# Transition Trim Map Summary\n\n');
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

families = unique(summaryTable.family, 'stable');
fprintf(fid, '## Success Counts by Family\n\n');
fprintf(fid, '| family | exact successes | total |\n');
fprintf(fid, '| --- | ---: | ---: |\n');
for i = 1:numel(families)
    fam = families(i);
    mask = summaryTable.family == fam;
    fprintf(fid, '| %s | %d | %d |\n', fam, nnz(summaryTable.success(mask)), nnz(mask));
end

fprintf(fid, '\n## First 40 Exact Successes\n\n');
fprintf(fid, '| name | phase | tilt (deg) | V (m/s) | rear fixed (rpm) | front (rpm) | rear (rpm) | df (deg) | de (deg) | theta (deg) | alpha (deg) |\n');
fprintf(fid, '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n');

successRows = summaryTable(summaryTable.success, :);
limit = min(40, height(successRows));
for i = 1:limit
    row = successRows(i, :);
    fprintf(fid, '| %s | %s | %.1f | %.1f | %.1f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |\n', ...
        row.name, row.phase, row.tilt_deg, row.vinf_mps, row.rear_fixed_rpm, ...
        row.front_collective_rpm, row.rear_collective_rpm, row.delta_f_deg, ...
        row.delta_e_deg, row.theta_deg, row.alpha_deg);
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
rearLabel = 'free';
if ~isnan(target.rear_fixed_rpm)
    rearLabel = localValueLabel(target.rear_fixed_rpm);
end
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
    'rear_fixed_rpm', NaN, ...
    'case_struct', struct(), ...
    'manual_seed', struct());
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
