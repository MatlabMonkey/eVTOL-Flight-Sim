% Run_NewMap_RectangleThenAdaptiveQueue.m
% One-command queue for a coarse rectangular pass followed by adaptive passes.
%
% Usage:
%   run('scripts/trim/Run_NewMap_RectangleThenAdaptiveQueue.m')
%
% Optional dry run:
%   newMapRectangleAdaptiveQueueOptions = struct('dry_run', true);
%   run('scripts/trim/Run_NewMap_RectangleThenAdaptiveQueue.m')

if ~exist('newMapRectangleAdaptiveQueueOptions', 'var') || ...
        ~isstruct(newMapRectangleAdaptiveQueueOptions)
    newMapRectangleAdaptiveQueueOptions = struct();
end

localMain(newMapRectangleAdaptiveQueueOptions);

function localMain(userOptions)
scriptPath = mfilename('fullpath');
scriptDir = fileparts(scriptPath);
rootDir = fileparts(fileparts(scriptDir));
addpath(scriptDir);
cd(rootDir);

config = localOverlayStruct(localDefaultQueueConfig(rootDir), userOptions);
if config.dry_run && ~isfinite(config.adaptive_batch_count)
    config.adaptive_batch_count = 1;
end
timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
queueOutputDir = fullfile(rootDir, 'workspace_plots', ...
    sprintf('newmap_rectangle_then_adaptive_%s', timestamp));
if exist(queueOutputDir, 'dir') ~= 7
    mkdir(queueOutputDir);
end

logFile = fullfile(queueOutputDir, 'rectangle_then_adaptive_queue.log');
diary(logFile);
diaryCleanup = onCleanup(@() diary('off'));

queueStart = datetime('now', 'TimeZone', 'local');
fprintf('=== Run_NewMap_RectangleThenAdaptiveQueue ===\n');
fprintf('Started: %s\n', char(queueStart));
fprintf('Root: %s\n', rootDir);
fprintf('Queue output: %s\n', queueOutputDir);
fprintf('Dry run: %d\n\n', logical(config.dry_run));

rectangleOptions = config.rectangle_options;
rectangleOptions.preview_csv = fullfile(queueOutputDir, 'rectangle_targets.csv');
[rectangleTargets, rectangleSummary] = Build_NewMap_RectangleTargets(rectangleOptions);
fprintf('Rectangle target builder:\n');
fprintf('  proposed targets = %d\n', height(rectangleTargets));
fprintf('  preview CSV = %s\n\n', char(rectangleSummary.preview_csv));

if config.dry_run
    localDryValidatePlan(rootDir, rectangleTargets, config);
else
    rectangleTrimOptions = localTrimOptions(config, rectangleTargets, 'newmap_rectangle_coarse');
    localRunTrimSearchPassInBase('rectangle_coarse', rectangleTrimOptions, config.stop_on_pass_error);
end

iBatch = 1;
while iBatch <= config.adaptive_batch_count
    if isfinite(config.max_wall_clock_hours) && ...
            localElapsedHours(queueStart) >= config.max_wall_clock_hours
        fprintf('Stopping before adaptive batch %d: wall-clock limit reached.\n', iBatch);
        break;
    end

    adaptiveOptions = config.adaptive_options;
    adaptiveOptions.max_targets = config.adaptive_targets_per_batch;
    adaptiveOptions.preview_csv = fullfile(queueOutputDir, ...
        sprintf('adaptive_targets_batch_%02d.csv', iBatch));
    [adaptiveTargets, adaptiveSummary] = Build_NewMap_AdaptiveFrontierTargets(adaptiveOptions);
    fprintf('\nAdaptive batch %d/%s target builder:\n', ...
        iBatch, localCountLabel(config.adaptive_batch_count));
    fprintf('  attempts in DB = %d\n', adaptiveSummary.attempt_rows);
    fprintf('  usable anchors = %d\n', adaptiveSummary.anchor_rows);
    fprintf('  proposed targets = %d\n', height(adaptiveTargets));
    fprintf('  preview CSV = %s\n', char(adaptiveSummary.preview_csv));

    if isempty(adaptiveTargets)
        fprintf('No adaptive targets left; stopping queue.\n');
        break;
    end

    if config.dry_run
        localDryValidatePlan(rootDir, adaptiveTargets, config);
    else
        outputPrefix = sprintf('newmap_adaptive_batch_%02d', iBatch);
        adaptiveTrimOptions = localTrimOptions(config, adaptiveTargets, outputPrefix);
        passOk = localRunTrimSearchPassInBase(outputPrefix, adaptiveTrimOptions, config.stop_on_pass_error);
        if ~passOk && config.stop_on_pass_error
            break;
        end
    end

    iBatch = iBatch + 1;
end

if ~config.dry_run
    localRefreshPlot(queueOutputDir);
end

queueEnd = datetime('now', 'TimeZone', 'local');
elapsedSeconds = seconds(queueEnd - queueStart);
fprintf('\n=== Rectangle then adaptive queue complete ===\n');
fprintf('Finished: %s\n', char(queueEnd));
fprintf('Elapsed: %.1f s (%.2f hr)\n', elapsedSeconds, elapsedSeconds / 3600);
fprintf('Log file: %s\n', logFile);
end

function config = localDefaultQueueConfig(rootDir)
config = struct();
config.root_dir = rootDir;
config.dry_run = false;
config.max_wall_clock_hours = inf;
config.stop_on_pass_error = true;
config.adaptive_batch_count = inf;
config.adaptive_targets_per_batch = 120;

rectangleOptions = struct();
rectangleOptions.max_targets = inf;
rectangleOptions.vinf_grid_mps = 0:5:75;
rectangleOptions.tilt_grid_deg = 0:10:90;
rectangleOptions.alpha_schedule_vinf_mps = [0 10 20 30 40 50 60 70 75];
rectangleOptions.alpha_schedule_deg = [0 5 10 10 7.5 5 2.5 0 0];
rectangleOptions.extra_alpha_vinf_bounds_mps = [20 55];
rectangleOptions.extra_alpha_tilt_bounds_deg = [30 80];
rectangleOptions.extra_alpha_offsets_deg = 5.0;
config.rectangle_options = rectangleOptions;

adaptiveOptions = struct();
adaptiveOptions.vinf_bounds_mps = [0 75];
adaptiveOptions.tilt_bounds_deg = [0 95];
adaptiveOptions.alpha_bounds_deg = [-5 17.5];
adaptiveOptions.vinf_offsets_mps = [-5 -2.5 0 2.5 5];
adaptiveOptions.tilt_offsets_deg = [-5 -2.5 0 2.5 5];
adaptiveOptions.alpha_offsets_deg = [-2.5 0 2.5];
adaptiveOptions.max_local_failed_attempts = 4;
adaptiveOptions.skip_existing_good_targets = true;
adaptiveOptions.skip_dense_failures = true;
adaptiveOptions.enforce_guide_corridor = true;
adaptiveOptions.max_guide_tilt_error_deg = 22.5;
config.adaptive_options = adaptiveOptions;
end

function opts = localTrimOptions(config, targetTable, outputPrefix)
opts = struct();
opts.profile = 'force_balance_newmap';
opts.database_name = 'trim_vinf_alpha_v1';
opts.output_prefix = outputPrefix;
opts.target_strategy = 'explicit_targets';
opts.explicit_targets = targetTable;
opts.target_limit = height(targetTable);
opts.max_vinf_mps = 75;
opts.target_tilt_min_deg = 0;
opts.target_tilt_max_deg = 95;
opts.reference_grid_step_deg = 2.5;

opts.front_guide_knot_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.front_guide_knot_rpm = [1865 1850 1750 1550 1250 1000 875 820 780];
opts.rear_guide_knot_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.rear_guide_knot_rpm = [1755 1650 1400 1100 800 650 550 450 350];

opts.front_seed_offsets_rpm = [-75 0 75];
opts.rear_seed_offsets_rpm = [-250 0 250];
opts.front_collective_min_rpm = 600;
opts.rear_collective_min_rpm = 100;

opts.enable_transition_force_balance_seeds = false;
opts.enable_low_speed_physics_seeds = false;
opts.enable_guide_grid_seeds = true;
opts.neighbor_seed_count = 4;
opts.history_seed_count = 8;
opts.anchor_max_vinf_mps = inf;
opts.anchor_max_tilt_error_deg = inf;
opts.anchor_seed_front_window_rpm = 500;
opts.anchor_seed_rear_window_rpm = 600;
opts.family_names = {'front_rear_free_flap_elevator', ...
    'rear_fixed_flap_elevator', ...
    'prop_fixed_flap_elevator'};
opts.stop_after_exact = true;

opts.failed_seed_filter_enabled = true;
opts.failed_seed_density_threshold = 3;
opts.failed_seed_vinf_radius_mps = 2.5;
opts.failed_seed_tilt_radius_deg = 5.0;
opts.failed_seed_front_radius_rpm = 125.0;
opts.failed_seed_rear_radius_rpm = 125.0;

opts.write_debug_run_outputs = false;
opts.write_linearizations_to_db = false;
opts.update_master_attempt_db_on_checkpoint = true;
opts.refresh_canonical_databases_on_checkpoint = false;
opts.checkpoint_every = 1;

if isfield(config, 'trim_options_overrides') && isstruct(config.trim_options_overrides)
    opts = localOverlayStruct(opts, config.trim_options_overrides);
end
end

function passOk = localRunTrimSearchPassInBase(passName, options, stopOnError)
fprintf('\n=== Trim pass: %s ===\n', passName);
passStart = datetime('now', 'TimeZone', 'local');
fprintf('Pass started: %s\n', char(passStart));
fprintf('Targets configured: %d\n', height(options.explicit_targets));

passOk = false;
assignin('base', 'transitionTrimSearchOptions', options);
try
    evalin('base', 'TrimSearch_Run;');
    passOk = true;
    fprintf('Pass complete: %s\n', char(datetime('now', 'TimeZone', 'local')));
catch ME
    fprintf('Pass failed: %s\n', ME.message);
    for iStack = 1:numel(ME.stack)
        fprintf('  at %s:%d\n', ME.stack(iStack).file, ME.stack(iStack).line);
    end
    if stopOnError
        fprintf('Stopping queue because stop_on_pass_error is true.\n');
    end
end

passElapsedSeconds = seconds(datetime('now', 'TimeZone', 'local') - passStart);
fprintf('Pass elapsed: %.1f s (%.2f min)\n', passElapsedSeconds, passElapsedSeconds / 60);
end

function localDryValidatePlan(rootDir, targetTable, config)
n = min(5, height(targetTable));
if n == 0
    fprintf('Dry validation skipped: empty target table.\n');
    return;
end
trimOptions = localTrimOptions(config, targetTable(1:n, :), 'dry_validation_only');
plan = TrimSearch_BuildPlan(trimOptions, rootDir);
assert(numel(plan.targets) == n, 'Dry validation target count mismatch.');
fprintf('Dry validation: explicit target plan accepted %d targets.\n', n);
end

function localRefreshPlot(queueOutputDir)
try
    trimNewMapPlotOptions = struct();
    trimNewMapPlotOptions.show_popup = false;
    trimNewMapPlotOptions.show_guide_curve = true;
    trimNewMapPlotOptions.output_dir = queueOutputDir;
    trimNewMapPlotOptions.guide_curve_vinf_mps = [0 10 20 30 40 50 60 70];
    trimNewMapPlotOptions.guide_curve_tilt_deg = [0 15 35 55 70 80 85 90];
    assignin('base', 'trimNewMapPlotOptions', trimNewMapPlotOptions);
    evalin('base', 'Plot_Trim_NewMap;');
catch ME
    fprintf('Plot refresh failed: %s\n', ME.message);
end
end

function elapsedHours = localElapsedHours(queueStart)
elapsedHours = seconds(datetime('now', 'TimeZone', 'local') - queueStart) / 3600;
end

function label = localCountLabel(value)
if isfinite(value)
    label = sprintf('%d', value);
else
    label = 'unbounded';
end
end

function out = localOverlayStruct(defaults, overrides)
out = defaults;
if ~isstruct(overrides)
    return;
end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    out.(fields{i}) = overrides.(fields{i});
end
end
