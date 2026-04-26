% TrimSearch_Engine.m
% Internal guide-grid transition-search engine. Prefer calling
% TrimSearch_Run.m for normal use.
%
% This engine searches a tilt/airspeed guide grid, scores each trim, writes
% attempts into the canonical master DB, and saves linked linearization
% artifacts for usable points.
%
% Original default profile is a mid-band closure sweep over the 20..50 m/s
% gap around a fixed tilt guide line and a fixed front/rear collective guide
% corridor, using the canonical trim/controller databases as the anchor bank.

if ~exist('transitionMidbandGuideGridOptions', 'var') || ~isstruct(transitionMidbandGuideGridOptions)
    transitionMidbandGuideGridOptions = struct();
end

initOptions = struct();
initOptions.transitionMidbandGuideGridOptions = transitionMidbandGuideGridOptions;
Init_Main

if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'transitionMidbandGuideGridOptions')
    transitionMidbandGuideGridOptions = initOptions.transitionMidbandGuideGridOptions;
else
    transitionMidbandGuideGridOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

runStartedAt = datetime('now', 'TimeZone', 'local');
runTimer = tic;

plan = TrimSearch_BuildPlan(transitionMidbandGuideGridOptions, root_dir);
config = plan.config;
targets = plan.targets;
historySeeds = plan.anchor_seeds;

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
writeDebugRunOutputs = isfield(config, 'write_debug_run_outputs') && config.write_debug_run_outputs;
if writeDebugRunOutputs
    transitionTrimMidbandGuideGridOutputDir = fullfile(root_dir, 'workspace_plots', ...
        [config.output_prefix '_' timestamp]);
    if exist(transitionTrimMidbandGuideGridOutputDir, 'dir') ~= 7
        mkdir(transitionTrimMidbandGuideGridOutputDir);
    end
else
    transitionTrimMidbandGuideGridOutputDir = "";
end

checkpointFile = fullfile(transitionTrimMidbandGuideGridOutputDir, 'transition_trim_midband_guidegrid_scored.mat');
summaryCsv = fullfile(transitionTrimMidbandGuideGridOutputDir, 'transition_trim_midband_guidegrid_scored_summary.csv');
summaryMd = fullfile(transitionTrimMidbandGuideGridOutputDir, 'transition_trim_midband_guidegrid_scored_summary.md');
logFile = fullfile(transitionTrimMidbandGuideGridOutputDir, 'transition_trim_midband_guidegrid_scored.log');

masterAttemptDbCsv = fullfile(config.database_dir, 'trim_attempts.csv');
masterAttemptDbMat = fullfile(config.database_dir, 'trim_attempts.mat');
controllerScheduleDbCsv = fullfile(config.database_dir, 'controller_schedule.csv');

masterAttemptSummary = localLoadMasterAttemptSummary(masterAttemptDbCsv);
controllerScheduleSummary = localLoadControllerScheduleSummary(controllerScheduleDbCsv);
controllerScheduleSeeds = localBuildHistorySeedsFromControllerSchedule(controllerScheduleSummary, config.rear_collective_min_rpm);
masterHistorySeeds = localBuildHistorySeedsFromSummary(masterAttemptSummary, config.rear_collective_min_rpm);
historySeeds = localMergeSeedLists(historySeeds, controllerScheduleSeeds, masterHistorySeeds);

if writeDebugRunOutputs
    diary off;
    diary(logFile);
    cleanupDiary = onCleanup(@() diary('off')); %#ok<NASGU>
end

fprintf('=== TrimSearch_Engine ===\n');
fprintf('Started: %s\n', char(runStartedAt, 'yyyy-MM-dd HH:mm:ss Z'));
if writeDebugRunOutputs
    fprintf('Debug output dir: %s\n', transitionTrimMidbandGuideGridOutputDir);
else
    fprintf('Debug output dir: disabled; writing canonical DB artifacts only.\n');
end
fprintf('Anchor seeds after DB merge: %d\n', numel(historySeeds));
fprintf('Configured %d mid-band targets.\n', numel(targets));

transitionTrimMidbandGuideGridMap = struct();
transitionTrimMidbandGuideGridMap.meta = struct();
transitionTrimMidbandGuideGridMap.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
transitionTrimMidbandGuideGridMap.meta.run_started_on = char(runStartedAt, 'yyyy-MM-dd HH:mm:ss Z');
transitionTrimMidbandGuideGridMap.meta.root_dir = root_dir;
transitionTrimMidbandGuideGridMap.meta.output_dir = transitionTrimMidbandGuideGridOutputDir;
transitionTrimMidbandGuideGridMap.meta.trim_model = initData.modelNames.trim;
transitionTrimMidbandGuideGridMap.meta.run_model = initData.modelNames.run;
transitionTrimMidbandGuideGridMap.meta.config = config;
transitionTrimMidbandGuideGridMap.meta.anchor_history_csv = config.anchor_history_csv;
transitionTrimMidbandGuideGridMap.meta.anchor_seed_count = numel(historySeeds);
transitionTrimMidbandGuideGridMap.meta.master_attempt_db_csv = masterAttemptDbCsv;
transitionTrimMidbandGuideGridMap.meta.controller_schedule_csv = controllerScheduleDbCsv;
transitionTrimMidbandGuideGridMap.meta.linearization_index_csv = config.linearization_index_csv;
transitionTrimMidbandGuideGridMap.meta.linearization_root = config.linearization_root;
transitionTrimMidbandGuideGridMap.targets = targets;
transitionTrimMidbandGuideGridMap.entries = repmat(localEntryTemplate(), 0, 1);
transitionTrimMidbandGuideGridMap.progress = struct('completed', 0, 'success_count', 0, ...
    'acceptable_count', 0, 'skipped_count', 0, 'last_key', '');

transitionTrimMidbandGuideGridSummary = table();

try
    for iTarget = 1:numel(targets)
        target = targets(iTarget);
        masterAttemptSummary = localLoadMasterAttemptSummary(masterAttemptDbCsv);
        controllerScheduleSummary = localLoadControllerScheduleSummary(controllerScheduleDbCsv);
        controllerScheduleSeeds = localBuildHistorySeedsFromControllerSchedule(controllerScheduleSummary, config.rear_collective_min_rpm);
        masterHistorySeeds = localBuildHistorySeedsFromSummary(masterAttemptSummary, config.rear_collective_min_rpm);
        combinedHistorySeeds = localMergeSeedLists(historySeeds, controllerScheduleSeeds, masterHistorySeeds);
        fprintf('\n[%d/%d] %s | tilt=%.1f | V=%.1f | guide front=%.1f | guide rear=%.1f\n', ...
            iTarget, numel(targets), target.name, target.tilt_deg, target.vinf_mps, ...
            target.front_guide_rpm, target.rear_guide_rpm);

        if localHasSolvedTarget(masterAttemptSummary, target)
            transitionTrimMidbandGuideGridMap.progress.skipped_count = ...
                transitionTrimMidbandGuideGridMap.progress.skipped_count + 1;
            fprintf('  skipped: exact/acceptable point already exists in canonical attempts.\n');
            continue;
        end

        entry = localSolveTarget(target, transitionTrimMidbandGuideGridMap.entries, combinedHistorySeeds, masterAttemptSummary, config, initData);
        transitionTrimMidbandGuideGridMap.entries(end + 1, 1) = entry; %#ok<SAGROW>
        transitionTrimMidbandGuideGridMap.progress.completed = numel(transitionTrimMidbandGuideGridMap.entries);
        transitionTrimMidbandGuideGridMap.progress.success_count = nnz([transitionTrimMidbandGuideGridMap.entries.success]);
        transitionTrimMidbandGuideGridMap.progress.acceptable_count = nnz([transitionTrimMidbandGuideGridMap.entries.acceptable]);
                transitionTrimMidbandGuideGridMap.progress.last_key = entry.key;

        fprintf('  best family=%s | seed=%s | success=%d | acceptable=%d | score=%.4f | class=%s\n', ...
            entry.family, entry.seed_name, entry.success, entry.acceptable, entry.score, entry.classification);
        fprintf('  term: %s\n', entry.termination_string);

        if mod(iTarget, config.checkpoint_every) == 0 || entry.success || entry.acceptable || iTarget == numel(targets)
            [transitionTrimMidbandGuideGridSummary, transitionTrimMidbandGuideGridMap] = ...
                localCheckpoint(transitionTrimMidbandGuideGridMap, checkpointFile, summaryCsv, summaryMd, ...
                masterAttemptDbMat, masterAttemptDbCsv, config.output_prefix, ...
                transitionTrimMidbandGuideGridOutputDir, root_dir, writeDebugRunOutputs);
            fprintf('  canonical DB checkpoint saved.\n');
        end
    end

    [transitionTrimMidbandGuideGridSummary, transitionTrimMidbandGuideGridMap] = ...
        localCheckpoint(transitionTrimMidbandGuideGridMap, checkpointFile, summaryCsv, summaryMd, ...
        masterAttemptDbMat, masterAttemptDbCsv, config.output_prefix, ...
        transitionTrimMidbandGuideGridOutputDir, root_dir, writeDebugRunOutputs);

    fprintf('\nCompleted mid-band guide-grid sweep.\n');
    fprintf('  total entries = %d\n', height(transitionTrimMidbandGuideGridSummary));
    if ismember('success', transitionTrimMidbandGuideGridSummary.Properties.VariableNames)
        exactCount = nnz(transitionTrimMidbandGuideGridSummary.success);
    else
        exactCount = 0;
    end
    if ismember('acceptable', transitionTrimMidbandGuideGridSummary.Properties.VariableNames)
        acceptableCount = nnz(transitionTrimMidbandGuideGridSummary.acceptable);
    else
        acceptableCount = 0;
    end
    fprintf('  exact successes = %d\n', exactCount);
    fprintf('  acceptable near-trims = %d\n', acceptableCount);
    fprintf('  skipped via master db = %d\n', transitionTrimMidbandGuideGridMap.progress.skipped_count);
    if writeDebugRunOutputs
        fprintf('  debug checkpoint = %s\n', checkpointFile);
    end
    fprintf('  elapsed = %.1f s (%.2f min)\n', toc(runTimer), toc(runTimer)/60);
catch ME
    fprintf('\nTrimSearch_Engine failed: %s\n', ME.message);
    [transitionTrimMidbandGuideGridSummary, transitionTrimMidbandGuideGridMap] = ...
        localCheckpoint(transitionTrimMidbandGuideGridMap, checkpointFile, summaryCsv, summaryMd, ...
        masterAttemptDbMat, masterAttemptDbCsv, config.output_prefix, ...
        transitionTrimMidbandGuideGridOutputDir, root_dir, writeDebugRunOutputs);
    fprintf('  elapsed before failure = %.1f s (%.2f min)\n', toc(runTimer), toc(runTimer)/60);
    rethrow(ME);
end

function entry = localSolveTarget(target, existingEntries, historySeeds, masterAttemptSummary, config, initData)
seedCandidates = localBuildSeedCandidates(target, existingEntries, historySeeds, masterAttemptSummary, config, initData);
attempts = repmat(localAttemptTemplate(), 0, 1);
bestAttempt = localAttemptTemplate();

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

        attempt = localMakeAttempt(target, family, trimResult, scoreData);
        attempts(end + 1, 1) = attempt; %#ok<SAGROW>
        if localIsBetterAttempt(attempt, bestAttempt)
            bestAttempt = attempt;
        end
        if config.stop_after_exact && bestAttempt.success
            break;
        end
    end
    if config.stop_after_exact && bestAttempt.success
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

function seedCandidates = localBuildSeedCandidates(target, existingEntries, historySeeds, masterAttemptSummary, config, initData)
seedCandidates = repmat(localSeedTemplate(), 0, 1);

if localGetField(config, 'enable_low_speed_physics_seeds', false)
    baseCase = localBaseTrimCase(target, config);
    try
        [propSeed, ~] = make_low_speed_prop_seed(initData, baseCase);
        propSeed.name = ['physics_' propSeed.name];
        propSeed.source = 'low_speed_prop_first_pass';
        propSeed.tilt_deg = target.tilt_deg;
        propSeed.vinf_mps = target.vinf_mps;
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(propSeed); %#ok<SAGROW>
    catch
    end

    try
        [twoPassSeed, ~] = make_low_speed_surface_seed(initData, baseCase);
        twoPassSeed.name = ['physics_' twoPassSeed.name];
        twoPassSeed.source = 'low_speed_two_pass';
        twoPassSeed.tilt_deg = target.tilt_deg;
        twoPassSeed.vinf_mps = target.vinf_mps;
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(twoPassSeed); %#ok<SAGROW>
    catch
    end
end

anchorSeeds = historySeeds;
if isempty(anchorSeeds)
    anchorSeeds = repmat(localSeedTemplate(), 0, 1);
end

anchorSeeds = localFilterAnchorSeedsByPropWindow(anchorSeeds, target, config);

if ~isempty(anchorSeeds)
    distances = arrayfun(@(s) localSeedDistance(s, target, config.seed_distance_weights), anchorSeeds);
    [~, order] = sort(distances, 'ascend');
    order = order(1:min(numel(order), config.history_seed_count));
    for i = 1:numel(order)
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(anchorSeeds(order(i))); %#ok<SAGROW>
    end
end

neighborEntries = existingEntries([existingEntries.success] | [existingEntries.acceptable]);
neighborEntries = localFilterEntryAnchorsByPropWindow(neighborEntries, target, config);
if ~isempty(neighborEntries)
    distances = arrayfun(@(e) localEntryDistance(e, target, config.seed_distance_weights), neighborEntries);
    [~, order] = sort(distances, 'ascend');
    order = order(1:min(numel(order), config.neighbor_seed_count));
    for i = 1:numel(order)
        seedCandidates(end + 1, 1) = localSeedFromEntry(neighborEntries(order(i))); %#ok<SAGROW>
    end
end

baseSeed = localBestBaseSeed(seedCandidates, target);
guideNameRoot = sprintf('guide_V%s_T%s', localValueLabel(target.vinf_mps), localValueLabel(target.tilt_deg));
for iFront = 1:numel(config.front_seed_offsets_rpm)
    for iRear = 1:numel(config.rear_seed_offsets_rpm)
        seed = baseSeed;
        seed.name = sprintf('%s_F%s_R%s', guideNameRoot, ...
            localValueLabel(config.front_seed_offsets_rpm(iFront)), ...
            localValueLabel(config.rear_seed_offsets_rpm(iRear)));
        seed.source = 'midband_guide_grid';
        seed.tilt_deg = target.tilt_deg;
        seed.vinf_mps = target.vinf_mps;
        seed.front_collective_guess_rpm = max(config.front_collective_min_rpm, ...
            target.front_guide_rpm + config.front_seed_offsets_rpm(iFront));
        seed.rear_collective_guess_rpm = max(config.rear_collective_min_rpm, ...
            target.rear_guide_rpm + config.rear_seed_offsets_rpm(iRear));
        seedCandidates(end + 1, 1) = localCanonicalizeSeed(seed); %#ok<SAGROW>
    end
end

blendSeed = baseSeed;
blendSeed.name = [guideNameRoot '_blend'];
blendSeed.source = 'midband_guide_blend';
blendSeed.tilt_deg = target.tilt_deg;
blendSeed.vinf_mps = target.vinf_mps;
blendSeed.front_collective_guess_rpm = max(config.front_collective_min_rpm, ...
    0.5 * (baseSeed.front_collective_guess_rpm + target.front_guide_rpm));
blendSeed.rear_collective_guess_rpm = max(config.rear_collective_min_rpm, ...
    0.5 * (baseSeed.rear_collective_guess_rpm + target.rear_guide_rpm));
seedCandidates(end + 1, 1) = localCanonicalizeSeed(blendSeed); %#ok<SAGROW>

seedCandidates = localUniqueSeeds(seedCandidates);
seedCandidates = localFilterSeedsNearDenseFailures(seedCandidates, masterAttemptSummary, target, config);
end

function anchorSeeds = localFilterAnchorSeedsByPropWindow(anchorSeeds, target, config)
if isempty(anchorSeeds)
    return;
end

frontWindow = localGetField(config, 'anchor_seed_front_window_rpm', inf);
rearWindow = localGetField(config, 'anchor_seed_rear_window_rpm', inf);
if ~isfinite(frontWindow) && ~isfinite(rearWindow)
    return;
end

keepMask = true(numel(anchorSeeds), 1);
for i = 1:numel(anchorSeeds)
    if isfinite(frontWindow)
        keepMask(i) = keepMask(i) && ...
            abs(anchorSeeds(i).front_collective_guess_rpm - target.front_guide_rpm) <= frontWindow + 1e-9;
    end
    if isfinite(rearWindow)
        keepMask(i) = keepMask(i) && ...
            abs(anchorSeeds(i).rear_collective_guess_rpm - target.rear_guide_rpm) <= rearWindow + 1e-9;
    end
end

if any(keepMask)
    anchorSeeds = anchorSeeds(keepMask);
end
end

function neighborEntries = localFilterEntryAnchorsByPropWindow(neighborEntries, target, config)
if isempty(neighborEntries)
    return;
end

frontWindow = localGetField(config, 'anchor_seed_front_window_rpm', inf);
rearWindow = localGetField(config, 'anchor_seed_rear_window_rpm', inf);
if ~isfinite(frontWindow) && ~isfinite(rearWindow)
    return;
end

keepMask = true(numel(neighborEntries), 1);
for i = 1:numel(neighborEntries)
    if isfinite(frontWindow)
        keepMask(i) = keepMask(i) && ...
            abs(neighborEntries(i).front_collective_rpm - target.front_guide_rpm) <= frontWindow + 1e-9;
    end
    if isfinite(rearWindow)
        keepMask(i) = keepMask(i) && ...
            abs(neighborEntries(i).rear_collective_rpm - target.rear_guide_rpm) <= rearWindow + 1e-9;
    end
end

if any(keepMask)
    neighborEntries = neighborEntries(keepMask);
end
end

function seedCandidates = localFilterSeedsNearDenseFailures(seedCandidates, masterAttemptSummary, target, config)
if ~localGetField(config, 'failed_seed_filter_enabled', false) || isempty(seedCandidates) || isempty(masterAttemptSummary)
    return;
end

required = {'vinf_mps','tilt_deg','front_collective_rpm','rear_collective_rpm','success','acceptable','classification'};
if ~all(ismember(required, masterAttemptSummary.Properties.VariableNames))
    return;
end

successMask = localAsLogical(masterAttemptSummary.success);
acceptableMask = localAsLogical(masterAttemptSummary.acceptable);
classification = localAsString(masterAttemptSummary.classification);
failedMask = ~(successMask | acceptableMask) | classification == "not_usable";
failedMask = failedMask & ...
    abs(masterAttemptSummary.vinf_mps - target.vinf_mps) <= config.failed_seed_vinf_radius_mps + 1e-9 & ...
    abs(masterAttemptSummary.tilt_deg - target.tilt_deg) <= config.failed_seed_tilt_radius_deg + 1e-9;

if ~any(failedMask)
    return;
end

failedTbl = masterAttemptSummary(failedMask, :);
keepMask = true(numel(seedCandidates), 1);
for i = 1:numel(seedCandidates)
    seed = seedCandidates(i);
    if ~(strcmp(seed.source, 'midband_guide_grid') || strcmp(seed.source, 'midband_guide_blend'))
        continue;
    end

    closeMask = abs(failedTbl.front_collective_rpm - seed.front_collective_guess_rpm) <= config.failed_seed_front_radius_rpm + 1e-9 & ...
        abs(failedTbl.rear_collective_rpm - seed.rear_collective_guess_rpm) <= config.failed_seed_rear_radius_rpm + 1e-9;
    if nnz(closeMask) >= config.failed_seed_density_threshold
        keepMask(i) = false;
    end
end

if any(keepMask)
    seedCandidates = seedCandidates(keepMask);
end
end

function baseSeed = localBestBaseSeed(seedCandidates, target)
if isempty(seedCandidates)
    baseSeed = localSeedTemplate();
    baseSeed.name = 'default_midband_base';
    baseSeed.source = 'default_midband_base';
    baseSeed.tilt_deg = target.tilt_deg;
    baseSeed.vinf_mps = target.vinf_mps;
    baseSeed.front_collective_guess_rpm = target.front_guide_rpm;
    baseSeed.rear_collective_guess_rpm = target.rear_guide_rpm;
    baseSeed.theta_guess_deg = 0.0;
    baseSeed.delta_f_guess_deg = 0.0;
    baseSeed.delta_e_guess_deg = 0.0;
    return;
end

baseSeed = localCanonicalizeSeed(seedCandidates(1));
baseSeed.tilt_deg = target.tilt_deg;
baseSeed.vinf_mps = target.vinf_mps;
end

function families = localBuildFamilies(target, seed, config)
families = repmat(localFamilyTemplate(), 0, 1);
for iFamily = 1:numel(config.family_names)
    familyName = string(config.family_names{iFamily});
    switch familyName
        case "front_rear_free_flap_elevator"
            families(end + 1, 1) = localMakeFrontRearFreeFlapElevatorCase(target, seed, config); %#ok<SAGROW>
        case "rear_fixed_flap_elevator"
            families(end + 1, 1) = localMakeRearFixedFlapElevatorCase(target, seed, config); %#ok<SAGROW>
        case "hover_zero_surface"
            families(end + 1, 1) = localMakeHoverZeroSurfaceCase(target, seed, config); %#ok<SAGROW>
        case "prop_fixed_flap_elevator"
            families(end + 1, 1) = localMakePropFixedFlapElevatorCase(target, seed, config); %#ok<SAGROW>
        otherwise
            error('TrimSearch_Engine:UnknownFamily', ...
                'Unknown family name: %s', familyName);
    end
end
end

function trimCase = localBaseTrimCase(target, config)
trimCase = struct();
trimCase.name = target.name;
trimCase.mode = 'transition_midband_guidegrid';
trimCase.Vinf_mps = target.vinf_mps;
trimCase.u_body_mps = target.vinf_mps;
trimCase.v_body_mps = 0.0;
trimCase.w_body_mps = 0.0;
trimCase.front_tilt_deg = target.tilt_deg;
trimCase.front_tilt_cmd_deg = target.tilt_deg;
trimCase.use_vinf_output_constraint = true;
trimCase.use_vertical_speed_output_constraint = true;
trimCase.position_steady = [false; false; false];
trimCase.validate_nonlinear_hold = false;
trimCase.front_collective_min_rpm = config.front_collective_min_rpm;
trimCase.rear_collective_min_rpm = config.rear_collective_min_rpm;
trimCase.front_collective_max_rpm = 2400.0;
trimCase.rear_collective_max_rpm = 2400.0;
trimCase.front_collective_guess_rpm = target.front_guide_rpm;
trimCase.rear_collective_guess_rpm = target.rear_guide_rpm;
trimCase.motor_rpm_cmd = zeros(4, 1);
trimCase.surface_init_deg = zeros(4, 1);
trimCase.surface_limit_deg = [25; 25; 25; 25];
trimCase.surface_rate_limit_deg_s = [250; 250; 250; 250];
trimCase.surface_tau_s = [1e-3; 1e-3; 1e-3; 1e-3];
trimCase.mixed_control_known = [false; false; false; false];
trimCase.body_rates_rad_s = [0; 0; 0];
trimCase.body_rates_known = [true; true; true];
trimCase.body_rates_steady = [true; true; true];
trimCase.body_velocity_known = [true; true; true];
trimCase.body_velocity_steady = [true; true; true];
trimCase.euler_known = [true; false; true];
trimCase.euler_steady = [true; true; true];
trimCase.position_known = [false; false; false];
end

function family = localMakeHoverZeroSurfaceCase(target, seed, config)
trimCase = localBaseTrimCase(target, config);
trimCase.name = sprintf('%s_hoverZero_%s', target.name, seed.name);
trimCase.front_collective_known = false;
trimCase.rear_collective_known = false;
trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
trimCase.theta_guess_deg = seed.theta_guess_deg;
trimCase.body_velocity_known = [false; true; false];
trimCase.mixed_control_known = [true; true; true; true];
trimCase.delta_f_fixed_deg = 0.0;
trimCase.delta_a_fixed_deg = 0.0;
trimCase.delta_e_fixed_deg = 0.0;
trimCase.delta_r_fixed_deg = 0.0;

family = localFamilyTemplate();
family.name = 'hover_zero_surface';
family.seed_name = seed.name;
family.trimCase = trimCase;
end

function family = localMakeFrontRearFreeFlapElevatorCase(target, seed, config)
trimCase = localBaseTrimCase(target, config);
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
trimCase = localBaseTrimCase(target, config);
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
trimCase = localBaseTrimCase(target, config);
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

function [summary, mapStruct] = localCheckpoint(mapStruct, checkpointFile, summaryCsv, summaryMd, masterMat, masterCsv, runPrefix, runOutputDir, rootDir, writeDebugRunOutputs)
summary = localBuildSummary(mapStruct.entries);
if isfield(mapStruct.meta, 'config') && isfield(mapStruct.meta.config, 'write_linearizations_to_db') && ...
        mapStruct.meta.config.write_linearizations_to_db
    [summary, mapStruct] = localWriteLinearizationArtifacts(summary, mapStruct, rootDir);
end
mapStruct.summary_table = summary;

if writeDebugRunOutputs
    save(checkpointFile, 'mapStruct', 'summary', '-v7.3');
    if ~isempty(summary)
        writetable(summary, summaryCsv);
    end
    localWriteSummaryMarkdown(summary, summaryMd);
end

if isfield(mapStruct.meta, 'config') && isfield(mapStruct.meta.config, 'update_master_attempt_db_on_checkpoint') && ...
        mapStruct.meta.config.update_master_attempt_db_on_checkpoint
    localUpdateMasterAttemptDb(summary, masterMat, masterCsv, runPrefix, runOutputDir, rootDir);
end
if isfield(mapStruct.meta, 'config') && isfield(mapStruct.meta.config, 'refresh_canonical_databases_on_checkpoint') && ...
        mapStruct.meta.config.refresh_canonical_databases_on_checkpoint
    localRefreshCanonicalDatabases(rootDir);
end
end

function localUpdateMasterAttemptDb(summary, masterMat, masterCsv, runPrefix, runOutputDir, rootDir)
helperFile = fullfile(rootDir, 'TrimDB_UpdateMaster.m');
if exist(helperFile, 'file') ~= 2
    return;
end

try
    dbPaths = TrimDB_Paths(rootDir);
    updateOptions = struct( ...
        'root_dir', rootDir, ...
        'database_dir', dbPaths.database_dir, ...
        'workspace_plots_dir', dbPaths.workspace_plots_dir, ...
        'run_prefix', runPrefix, ...
        'run_output_dir', runOutputDir, ...
        'source_file', string(runPrefix) + "_direct_db", ...
        'master_mat_file', masterMat, ...
        'master_csv_file', masterCsv);
    TrimDB_UpdateMaster(summary, updateOptions);
catch ME
    warning('TrimSearch_Engine:MasterDbUpdateFailed', ...
        'Canonical master DB update failed after checkpoint: %s', ME.message);
end
end

function localRefreshCanonicalDatabases(rootDir)
builderFile = fullfile(rootDir, 'TrimDB_Build.m');
if exist(builderFile, 'file') ~= 2
    return;
end

try
    transitionTrimDatabaseOptions = struct(); %#ok<NASGU>
    run(builderFile);
catch ME
    warning('TrimSearch_Engine:CanonicalDbRefreshFailed', ...
        'Canonical DB refresh failed after checkpoint: %s', ME.message);
end
end

function [summary, mapStruct] = localWriteLinearizationArtifacts(summary, mapStruct, rootDir)
if isempty(summary) || ~isfield(mapStruct.meta, 'config')
    return;
end

config = mapStruct.meta.config;
dbPaths = TrimDB_Paths(rootDir);
linearizationRoot = localGetField(config, 'linearization_root', ...
    dbPaths.linearization_root);
indexCsv = localGetField(config, 'linearization_index_csv', ...
    dbPaths.linearization_index_csv);
indexMat = localGetField(config, 'linearization_index_mat', ...
    dbPaths.linearization_index_mat);

if exist(linearizationRoot, 'dir') ~= 7
    mkdir(linearizationRoot);
end

newIndexRows = repmat(localLinearizationIndexRowTemplate(), 0, 1);
for i = 1:numel(mapStruct.entries)
    entry = mapStruct.entries(i);
    if i > height(summary) || ~(entry.success || entry.acceptable) || ~localHasReducedLinearization(entry.trimResult)
        continue;
    end

    fileStem = localSafeFileStem(sprintf('%s_%s_%s', entry.key, entry.family, entry.seed_name));
    outputFile = fullfile(linearizationRoot, [fileStem '.mat']);
    linearizationPoint = localBuildLinearizationPoint(entry, summary(i, :), outputFile); %#ok<NASGU>
    save(outputFile, 'linearizationPoint', '-v7.3');

    summary.linearization_available(i) = true;
    summary.linearization_index_source(i) = string(indexCsv);
    summary.linearization_latest_file(i) = string(outputFile);

    newIndexRows(end + 1, 1) = localBuildLinearizationIndexRow(summary(i, :), outputFile); %#ok<SAGROW>
end

if isempty(newIndexRows)
    return;
end

indexTable = localMergeLinearizationIndex(indexCsv, struct2table(newIndexRows));
linearizationIndex = table2struct(indexTable); %#ok<NASGU>
save(indexMat, 'linearizationIndex', 'indexTable', '-v7.3');
writetable(indexTable, indexCsv);
mapStruct.meta.linearization_index_csv = indexCsv;
mapStruct.meta.linearization_index_mat = indexMat;
mapStruct.meta.linearization_root = linearizationRoot;
end

function tf = localHasReducedLinearization(trimResult)
tf = isstruct(trimResult) && isfield(trimResult, 'linear') && isstruct(trimResult.linear) && ...
    isfield(trimResult.linear, 'sys_ss_9state') && ~isempty(trimResult.linear.sys_ss_9state);
end

function linearizationPoint = localBuildLinearizationPoint(entry, row, outputFile)
trimResult = entry.trimResult;
linearizationPoint = struct();
linearizationPoint.name = char(string(row.name));
linearizationPoint.key = char(string(row.key));
linearizationPoint.family = char(string(row.family));
linearizationPoint.seed_name = char(string(row.seed_name));
linearizationPoint.source_file = "direct_trim_search";
linearizationPoint.source_mat_file = "";
linearizationPoint.source_var_name = "";
linearizationPoint.output_file = char(string(outputFile));
linearizationPoint.success = logical(row.success);
linearizationPoint.acceptable = logical(row.acceptable);
linearizationPoint.classification = char(string(row.classification));
linearizationPoint.score = row.score;
linearizationPoint.max_normalized = row.max_normalized;
linearizationPoint.worst_component = char(string(row.worst_component));
linearizationPoint.worst_component_normalized = row.worst_component_normalized;
linearizationPoint.termination_string = char(string(row.termination_string));
linearizationPoint.tilt_deg = row.tilt_deg;
linearizationPoint.vinf_mps = row.vinf_mps;
linearizationPoint.theta_deg = row.theta_deg;
linearizationPoint.alpha_deg = row.alpha_deg;
linearizationPoint.u_mps = row.u_mps;
linearizationPoint.w_mps = row.w_mps;
linearizationPoint.front_collective_rpm = row.front_collective_rpm;
linearizationPoint.rear_collective_rpm = row.rear_collective_rpm;
linearizationPoint.delta_f_deg = row.delta_f_deg;
linearizationPoint.delta_a_deg = row.delta_a_deg;
linearizationPoint.delta_e_deg = row.delta_e_deg;
linearizationPoint.delta_r_deg = row.delta_r_deg;
linearizationPoint.trimCase = entry.trimCase;
linearizationPoint.scheduling = localGetField(trimResult, 'scheduling', struct());
linearizationPoint.Att_Trim_deg = localGetField(trimResult, 'Att_Trim_deg', []);
linearizationPoint.Vel_B_BA_Trim = localGetField(trimResult, 'Vel_B_BA_Trim', []);
linearizationPoint.Rates_Trim = localGetField(trimResult, 'Rates_Trim', []);
linearizationPoint.linear = localExtractLinearData(localGetField(trimResult, 'linear', struct()));
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

sysFull = localGetField(linearData, 'sys_full', []);
if ~isempty(sysFull)
    linear.full_state_names = string(sysFull.StateName(:));
    linear.full_input_names = string(sysFull.InputName(:));
    linear.full_output_names = string(sysFull.OutputName(:));
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

function row = localBuildLinearizationIndexRow(sourceRow, outputFile)
row = localLinearizationIndexRowTemplate();
row.key = string(sourceRow.key);
row.name = string(sourceRow.name);
row.tilt_deg = sourceRow.tilt_deg;
row.vinf_mps = sourceRow.vinf_mps;
row.family = string(sourceRow.family);
row.seed_name = string(sourceRow.seed_name);
row.success = logical(sourceRow.success);
row.acceptable = logical(sourceRow.acceptable);
row.classification = string(sourceRow.classification);
row.score = sourceRow.score;
row.max_normalized = sourceRow.max_normalized;
row.front_collective_rpm = sourceRow.front_collective_rpm;
row.rear_collective_rpm = sourceRow.rear_collective_rpm;
row.delta_f_deg = sourceRow.delta_f_deg;
row.delta_a_deg = sourceRow.delta_a_deg;
row.delta_e_deg = sourceRow.delta_e_deg;
row.delta_r_deg = sourceRow.delta_r_deg;
row.theta_deg = sourceRow.theta_deg;
row.alpha_deg = sourceRow.alpha_deg;
row.output_file = string(outputFile);
row.latest_file = string(outputFile);
end

function row = localLinearizationIndexRowTemplate()
row = struct( ...
    'key', "", ...
    'name', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'family', "", ...
    'seed_name', "", ...
    'success', false, ...
    'acceptable', false, ...
    'classification', "", ...
    'score', NaN, ...
    'max_normalized', NaN, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'alpha_deg', NaN, ...
    'output_file', "", ...
    'latest_file', "");
end

function indexTable = localMergeLinearizationIndex(indexCsv, newRows)
newRows = localEnsureLinearizationIndexSchema(newRows);
if exist(indexCsv, 'file') == 2
    existingRows = readtable(indexCsv, 'TextType', 'string');
    existingRows = localEnsureLinearizationIndexSchema(existingRows);
    indexTable = [existingRows; newRows]; %#ok<AGROW>
else
    indexTable = newRows;
end

ids = string(indexTable.key) + "|" + string(indexTable.family) + "|" + ...
    string(indexTable.front_collective_rpm) + "|" + string(indexTable.rear_collective_rpm);
[~, keepIdx] = unique(flipud(ids), 'stable');
keepIdx = height(indexTable) - keepIdx + 1;
indexTable = indexTable(sort(keepIdx), :);
end

function tbl = localEnsureLinearizationIndexSchema(tbl)
if isempty(tbl)
    tbl = struct2table(localLinearizationIndexRowTemplate());
    tbl(1, :) = [];
    return;
end
required = fieldnames(localLinearizationIndexRowTemplate());
defaultRow = localLinearizationIndexRowTemplate();
for i = 1:numel(required)
    name = required{i};
    if ismember(name, tbl.Properties.VariableNames)
        continue;
    end
    tbl.(name) = repmat(defaultRow.(name), height(tbl), 1);
end
tbl = tbl(:, required);
end

function value = localSafeFileStem(value)
value = char(string(value));
value = regexprep(value, '[^A-Za-z0-9_\\-]+', '_');
value = regexprep(value, '_+', '_');
value = regexprep(value, '^_|_$', '');
if isempty(value)
    value = 'linearization_point';
end
if numel(value) > 120
    value = value(1:120);
end
end

function summary = localBuildSummary(entries)
if isempty(entries)
    summary = table();
    return;
end
summary = table();
summary.key = string({entries.key}');
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
summary.linearization_available = false(height(summary), 1);
summary.linearization_index_source = strings(height(summary), 1);
summary.linearization_latest_file = strings(height(summary), 1);
end

function localWriteSummaryMarkdown(summary, filename)
fid = fopen(filename, 'w');
if fid < 0
    warning('TrimSearch_Engine:WriteMarkdownFailed', 'Could not write %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Mid-Band Guide-Grid Scored Sweep\n\n');
fprintf(fid, '- Total points: %d\n', height(summary));
if isempty(summary)
    return;
end
fprintf(fid, '- Exact trims: %d\n', nnz(summary.success));
fprintf(fid, '- Acceptable scored near-trims: %d\n', nnz(summary.acceptable));
fprintf(fid, '- Best score: %.4f\n\n', min(summary.score));
fprintf(fid, '## Top 15 Candidates\n\n');
fprintf(fid, '| Name | Tilt (deg) | V (m/s) | Family | Success | Acceptable | Score | Class |\n');
fprintf(fid, '| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |\n');
sortMat = [[~summary.success] [~summary.acceptable] summary.score];
[~, order] = sortrows(sortMat, [1 2 3]);
topIdx = order(1:min(15, numel(order)));
for i = 1:numel(topIdx)
    row = summary(topIdx(i), :);
    fprintf(fid, '| %s | %.1f | %.1f | %s | %d | %d | %.4f | %s |\n', ...
        localTextValue(row.name), row.tilt_deg, row.vinf_mps, localTextValue(row.family), ...
        row.success, row.acceptable, row.score, localTextValue(row.classification));
end
end

function summary = localLoadGlobalAttemptSummary(globalCsv)
if exist(globalCsv, 'file') ~= 2
    summary = localEmptyGlobalAttemptSummary();
    return;
end
summary = readtable(globalCsv, 'TextType', 'string');
summary = localEnsureGlobalAttemptSchema(summary);
end

function seeds = localBuildHistorySeedsFromSummary(summary, rearCollectiveMinRpm)
seeds = repmat(localSeedTemplate(), 0, 1);
if isempty(summary)
    return;
end

successMask = localAsLogical(summary.success);
acceptableMask = localAsLogical(summary.acceptable);
borderlineMask = localAsString(summary.classification) == "near_trim_borderline";
rearMask = summary.rear_collective_rpm > rearCollectiveMinRpm;
keepIdx = find((successMask | acceptableMask | borderlineMask) & rearMask);

for i = 1:numel(keepIdx)
    row = summary(keepIdx(i), :);
    seed = localSeedTemplate();
    seed.name = ['master_' char(row.name)];
    seed.source = 'master_attempt_db';
    seed.tilt_deg = row.tilt_deg;
    seed.vinf_mps = row.vinf_mps;
    seed.front_collective_guess_rpm = row.front_collective_rpm;
    seed.rear_collective_guess_rpm = row.rear_collective_rpm;
    seed.theta_guess_deg = row.theta_deg;
    seed.delta_f_guess_deg = row.delta_f_deg;
    seed.delta_a_guess_deg = row.delta_a_deg;
    seed.delta_e_guess_deg = row.delta_e_deg;
    seed.delta_r_guess_deg = row.delta_r_deg;
    seeds(end + 1, 1) = localCanonicalizeSeed(seed); %#ok<SAGROW>
end

seeds = localUniqueSeeds(seeds);
end

function summary = localLoadMasterAttemptSummary(masterCsv)
masterMat = replace(masterCsv, '.csv', '.mat');
if exist(masterMat, 'file') == 2
    try
        raw = load(masterMat);
        if isfield(raw, 'transitionTrimMasterAttemptDB') && isfield(raw.transitionTrimMasterAttemptDB, 'master_attempt_db_all_rows')
            summary = raw.transitionTrimMasterAttemptDB.master_attempt_db_all_rows;
            summary = localEnsureGlobalAttemptSchema(summary);
            return;
        end
    catch
    end
end

if exist(masterCsv, 'file') ~= 2
    summary = localEmptyGlobalAttemptSummary();
    return;
end

summary = readtable(masterCsv, 'TextType', 'string');
summary = localEnsureGlobalAttemptSchema(summary);
end

function summary = localLoadControllerScheduleSummary(controllerCsv)
if exist(controllerCsv, 'file') ~= 2
    summary = table();
    return;
end

summary = readtable(controllerCsv, 'TextType', 'string');
summary = localEnsureControllerScheduleSchema(summary);
end

function seeds = localBuildHistorySeedsFromControllerSchedule(summary, rearCollectiveMinRpm)
seeds = repmat(localSeedTemplate(), 0, 1);
if isempty(summary)
    return;
end

rearMask = summary.rear_collective_rpm > rearCollectiveMinRpm;
keepIdx = find(rearMask);
for i = 1:numel(keepIdx)
    row = summary(keepIdx(i), :);
    seed = localSeedTemplate();
    seed.name = ['controller_' localTextValue(row.name)];
    seed.source = 'controller_schedule';
    seed.tilt_deg = row.tilt_deg;
    seed.vinf_mps = row.vinf_mps;
    seed.front_collective_guess_rpm = row.front_collective_rpm;
    seed.rear_collective_guess_rpm = row.rear_collective_rpm;
    seed.theta_guess_deg = row.theta_deg;
    seed.delta_f_guess_deg = row.delta_f_deg;
    seed.delta_a_guess_deg = row.delta_a_deg;
    seed.delta_e_guess_deg = row.delta_e_deg;
    seed.delta_r_guess_deg = row.delta_r_deg;
    seeds(end + 1, 1) = localCanonicalizeSeed(seed); %#ok<SAGROW>
end

seeds = localUniqueSeeds(seeds);
end

function tf = localHasSolvedTarget(summary, target)
tf = false;
if isempty(summary)
    return;
end

if ismember('key', summary.Properties.VariableNames)
    targetKey = string(localTargetKey(target));
    keyMask = summary.key == targetKey;
else
    keyMask = abs(summary.tilt_deg - target.tilt_deg) <= 1e-9 & ...
        abs(summary.vinf_mps - target.vinf_mps) <= 1e-9;
end

if ~any(keyMask)
    return;
end

successMask = localAsLogical(summary.success);
acceptableMask = localAsLogical(summary.acceptable);
tf = any(keyMask & (successMask | acceptableMask));
end

function globalSummary = localUpdateGlobalAttemptDb(summary, globalMat, globalCsv, globalMd, runPrefix, runOutputDir)
existing = localLoadGlobalAttemptSummary(globalCsv);
current = localBuildGlobalAttemptSummary(summary, runPrefix, runOutputDir);

if isempty(existing)
    globalSummary = current;
else
    globalSummary = [existing; current]; %#ok<AGROW>
end

globalSummary = localEnsureGlobalAttemptSchema(globalSummary);
if ~isempty(globalSummary)
    globalSummary = localDeduplicateGlobalAttemptSummary(globalSummary);
end

save(globalMat, 'globalSummary', '-v7.3');
if ~isempty(globalSummary)
    writetable(globalSummary, globalCsv);
end
localWriteGlobalAttemptMarkdown(globalSummary, globalMd);
end

function summary = localBuildGlobalAttemptSummary(localSummary, runPrefix, runOutputDir)
summary = localEnsureGlobalAttemptSchema(localSummary);
if isempty(summary)
    return;
end
summary.source_run_prefix = repmat(string(runPrefix), height(summary), 1);
summary.source_run_dir = repmat(string(runOutputDir), height(summary), 1);
summary.saved_on = repmat(string(char(datetime('now', 'TimeZone', 'local', ...
    'Format', 'yyyy-MM-dd HH:mm:ss Z'))), height(summary), 1);
end

function summary = localEnsureGlobalAttemptSchema(summary)
if isempty(summary)
    summary = localEmptyGlobalAttemptSummary();
    return;
end

requiredNames = { ...
    'key', 'name', 'tilt_deg', 'vinf_mps', 'family', 'seed_name', ...
    'success', 'acceptable', 'classification', 'score', 'max_normalized', ...
    'worst_component', 'worst_component_normalized', ...
    'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_a_deg', 'delta_e_deg', 'delta_r_deg', ...
    'theta_deg', 'u_mps', 'w_mps', 'alpha_deg', 'termination_string', ...
    'source_run_prefix', 'source_run_dir', 'saved_on'};

for i = 1:numel(requiredNames)
    name = requiredNames{i};
    if ismember(name, summary.Properties.VariableNames)
        continue;
    end
    switch name
        case {'key','name','family','seed_name','classification','worst_component','termination_string','source_run_prefix','source_run_dir','saved_on'}
            summary.(name) = repmat("", height(summary), 1);
        case {'success','acceptable'}
            summary.(name) = false(height(summary), 1);
        otherwise
            summary.(name) = nan(height(summary), 1);
    end
end

summary = summary(:, requiredNames);
end

function summary = localEnsureControllerScheduleSchema(summary)
if isempty(summary)
    summary = table();
    summary.name = strings(0,1);
    summary.tilt_deg = zeros(0,1);
    summary.vinf_mps = zeros(0,1);
    summary.front_collective_rpm = zeros(0,1);
    summary.rear_collective_rpm = zeros(0,1);
    summary.delta_f_deg = zeros(0,1);
    summary.delta_a_deg = zeros(0,1);
    summary.delta_e_deg = zeros(0,1);
    summary.delta_r_deg = zeros(0,1);
    summary.theta_deg = zeros(0,1);
    return;
end

requiredNames = { ...
    'name', 'tilt_deg', 'vinf_mps', ...
    'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_a_deg', 'delta_e_deg', 'delta_r_deg', ...
    'theta_deg'};

for i = 1:numel(requiredNames)
    name = requiredNames{i};
    if ismember(name, summary.Properties.VariableNames)
        continue;
    end
    if strcmp(name, 'name')
        summary.(name) = repmat("", height(summary), 1);
    else
        summary.(name) = nan(height(summary), 1);
    end
end

summary = summary(:, requiredNames);
end

function summary = localDeduplicateGlobalAttemptSummary(summary)
if isempty(summary)
    return;
end

ids = summary.source_run_dir + "|" + summary.name + "|" + summary.family + "|" + summary.seed_name;
[~, keepIdx] = unique(ids, 'stable');
summary = summary(keepIdx, :);
end

function summary = localEmptyGlobalAttemptSummary()
summary = table();
summary.key = strings(0,1);
summary.name = strings(0,1);
summary.tilt_deg = zeros(0,1);
summary.vinf_mps = zeros(0,1);
summary.family = strings(0,1);
summary.seed_name = strings(0,1);
summary.success = false(0,1);
summary.acceptable = false(0,1);
summary.classification = strings(0,1);
summary.score = zeros(0,1);
summary.max_normalized = zeros(0,1);
summary.worst_component = strings(0,1);
summary.worst_component_normalized = zeros(0,1);
summary.front_collective_rpm = zeros(0,1);
summary.rear_collective_rpm = zeros(0,1);
summary.delta_f_deg = zeros(0,1);
summary.delta_a_deg = zeros(0,1);
summary.delta_e_deg = zeros(0,1);
summary.delta_r_deg = zeros(0,1);
summary.theta_deg = zeros(0,1);
summary.u_mps = zeros(0,1);
summary.w_mps = zeros(0,1);
summary.alpha_deg = zeros(0,1);
summary.termination_string = strings(0,1);
summary.source_run_prefix = strings(0,1);
summary.source_run_dir = strings(0,1);
summary.saved_on = strings(0,1);
end

function localWriteGlobalAttemptMarkdown(summary, filename)
fid = fopen(filename, 'w');
if fid < 0
    warning('TrimSearch_Engine:WriteGlobalMarkdownFailed', ...
        'Could not write %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Global Transition Trim Attempt DB\n\n');
fprintf(fid, '- Total saved attempts: %d\n', height(summary));
if isempty(summary)
    return;
end

fprintf(fid, '- Exact trims: %d\n', nnz(localAsLogical(summary.success)));
fprintf(fid, '- Acceptable near-trims: %d\n', nnz(localAsLogical(summary.acceptable)));
fprintf(fid, '- Unique runs: %d\n\n', numel(unique(summary.source_run_dir)));

sortMat = [[~localAsLogical(summary.success)] [~localAsLogical(summary.acceptable)] summary.score];
[~, order] = sortrows(sortMat, [1 2 3]);
topIdx = order(1:min(20, numel(order)));

fprintf(fid, '## Top 20 Rows\n\n');
fprintf(fid, '| Name | Tilt (deg) | V (m/s) | Family | Success | Acceptable | Score | Run |\n');
fprintf(fid, '| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |\n');
for i = 1:numel(topIdx)
    row = summary(topIdx(i), :);
    fprintf(fid, '| %s | %.1f | %.1f | %s | %d | %d | %.4f | %s |\n', ...
        localTextValue(row.name), row.tilt_deg, row.vinf_mps, localTextValue(row.family), ...
        row.success, row.acceptable, row.score, localTextValue(row.source_run_prefix));
end
end

function value = localTextValue(fieldValue)
if iscell(fieldValue)
    fieldValue = fieldValue{1};
end
if isstring(fieldValue)
    fieldValue = fieldValue(1);
    if ismissing(fieldValue)
        value = "";
    else
        value = fieldValue;
    end
elseif ischar(fieldValue)
    value = string(fieldValue);
elseif isempty(fieldValue)
    value = "";
else
    value = string(fieldValue);
end

if ismissing(value)
    value = "";
end

value = char(value);
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
seed.front_collective_guess_rpm = max(seed.front_collective_guess_rpm, 0.0);
seed.rear_collective_guess_rpm = max(seed.rear_collective_guess_rpm, 0.0);
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

function mergedSeeds = localMergeSeedLists(varargin)
mergedSeeds = repmat(localSeedTemplate(), 0, 1);
for iList = 1:nargin
    seedList = varargin{iList};
    if isempty(seedList)
        continue;
    end
    for iSeed = 1:numel(seedList)
        mergedSeeds(end + 1, 1) = localCanonicalizeSeed(seedList(iSeed)); %#ok<SAGROW>
    end
end
mergedSeeds = localUniqueSeeds(mergedSeeds);
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
    'center_tilt_deg', NaN, ...
    'front_guide_rpm', NaN, ...
    'rear_guide_rpm', NaN, ...
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
    'theta_guess_deg', 0.0, ...
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

function out = localAsLogical(col)
if islogical(col)
    out = col;
elseif isnumeric(col)
    out = col ~= 0;
else
    lowered = lower(strtrim(string(col)));
    out = lowered == "1" | lowered == "true" | lowered == "yes";
end
out = logical(out);
end

function out = localAsString(col)
if isstring(col)
    out = col;
elseif iscellstr(col) || iscell(col)
    out = string(col);
elseif isnumeric(col)
    out = string(col);
else
    out = string(col);
end
end
