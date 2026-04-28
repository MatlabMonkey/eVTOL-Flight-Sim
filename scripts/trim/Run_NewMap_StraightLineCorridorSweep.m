% Run_NewMap_StraightLineCorridorSweep.m
% One-command straight-line corridor sweep for the Vinf/tilt/alpha trim map.
%
% Default target set is intended to be about a one-hour refinement pass:
%   Vinf         = 0:2.5:70 m/s
%   center tilt  = 90/70 * Vinf
%   tilt offsets = [-10 0 10] deg
%   alpha        = [2.5 5 7.5 10] deg
%
% Usage:
%   run('scripts/trim/Run_NewMap_StraightLineCorridorSweep.m')
%
% Optional dry run:
%   newMapStraightLineCorridorOptions = struct('dry_run', true);
%   run('scripts/trim/Run_NewMap_StraightLineCorridorSweep.m')

if ~exist('newMapStraightLineCorridorOptions', 'var') || ...
        ~isstruct(newMapStraightLineCorridorOptions)
    newMapStraightLineCorridorOptions = struct();
end

localMain(newMapStraightLineCorridorOptions);

function localMain(userOptions)
scriptPath = mfilename('fullpath');
scriptDir = fileparts(scriptPath);
rootDir = fileparts(fileparts(scriptDir));
addpath(scriptDir);
cd(rootDir);

config = localOverlayStruct(localDefaultConfig(rootDir), userOptions);
timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
outputDir = fullfile(rootDir, 'workspace_plots', ...
    sprintf('newmap_straight_corridor_%s', timestamp));
if exist(outputDir, 'dir') ~= 7
    mkdir(outputDir);
end

logFile = fullfile(outputDir, 'straight_line_corridor_sweep.log');
diary(logFile);
diaryCleanup = onCleanup(@() diary('off'));

runStart = datetime('now', 'TimeZone', 'local');
fprintf('=== Run_NewMap_StraightLineCorridorSweep ===\n');
fprintf('Started: %s\n', char(runStart));
fprintf('Root: %s\n', rootDir);
fprintf('Output: %s\n', outputDir);
fprintf('Dry run: %d\n\n', logical(config.dry_run));

targetOptions = config.target_options;
targetOptions.database_name = config.database_name;
targetOptions.preview_csv = fullfile(outputDir, 'straight_line_corridor_targets.csv');
[targets, targetSummary] = Build_NewMap_StraightLineCorridorTargets(targetOptions);

fprintf('Target builder:\n');
fprintf('  raw corridor targets = %d\n', targetSummary.raw_target_count);
fprintf('  skipped existing targets = %d\n', targetSummary.skipped_existing_count);
fprintf('  queued targets = %d\n', height(targets));
fprintf('  Vinf grid = %s\n', mat2str(targetSummary.vinf_grid_mps));
fprintf('  tilt offsets = %s\n', mat2str(targetSummary.tilt_offsets_deg));
fprintf('  alpha grid = %s\n', mat2str(targetSummary.alpha_grid_deg));
fprintf('  preview CSV = %s\n\n', char(targetSummary.preview_csv));

if isempty(targets)
    fprintf('No new targets to run.\n');
    localFinish(runStart, logFile);
    return;
end

trimOptions = localTrimOptions(config, targets);
if config.dry_run
    localDryValidatePlan(rootDir, trimOptions);
else
    localRunTrimSearchPassInBase(trimOptions, config.stop_on_error);
    localRefreshPlot(outputDir);
end

localFinish(runStart, logFile);
end

function config = localDefaultConfig(rootDir)
config = struct();
config.root_dir = rootDir;
config.database_name = 'trim_vinf_alpha_v1';
config.output_prefix = 'newmap_straight_corridor';
config.dry_run = false;
config.stop_on_error = true;

targetOptions = struct();
targetOptions.vinf_grid_mps = 0:2.5:70;
targetOptions.vinf_bounds_mps = [0 70];
targetOptions.tilt_bounds_deg = [0 95];
targetOptions.alpha_bounds_deg = [0 10];
targetOptions.alpha_grid_deg = [2.5 5 7.5 10];
targetOptions.tilt_offsets_deg = [-10 0 10];
targetOptions.skip_existing_targets = true;
targetOptions.max_targets = inf;
config.target_options = targetOptions;
end

function opts = localTrimOptions(config, targetTable)
opts = struct();
opts.profile = 'force_balance_newmap';
opts.database_name = config.database_name;
opts.output_prefix = config.output_prefix;
opts.target_strategy = 'explicit_targets';
opts.explicit_targets = targetTable;
opts.target_limit = height(targetTable);
opts.vinf_grid_mps = unique(targetTable.vinf_mps).';
opts.tilt_grid_deg = unique(targetTable.tilt_deg).';
opts.tilt_offsets_deg = 0;
opts.alpha_grid_deg = unique(targetTable.alpha_target_deg).';
opts.max_vinf_mps = 70;
opts.target_tilt_min_deg = 0;
opts.target_tilt_max_deg = 95;
opts.reference_grid_step_deg = 2.5;

opts.reference_knot_vinf_mps = [0 10 20 30 40 50 60 70];
opts.reference_knot_tilt_deg = [0 15 35 55 70 80 85 90];
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

function localDryValidatePlan(rootDir, trimOptions)
n = min(5, height(trimOptions.explicit_targets));
dryOptions = trimOptions;
dryOptions.explicit_targets = dryOptions.explicit_targets(1:n, :);
dryOptions.target_limit = n;
plan = TrimSearch_BuildPlan(dryOptions, rootDir);
assert(numel(plan.targets) == n, 'Dry validation target count mismatch.');
fprintf('Dry validation: explicit target plan accepted %d targets.\n', n);
end

function passOk = localRunTrimSearchPassInBase(options, stopOnError)
fprintf('\n=== Trim pass: %s ===\n', options.output_prefix);
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
        rethrow(ME);
    end
end

passElapsedSeconds = seconds(datetime('now', 'TimeZone', 'local') - passStart);
fprintf('Pass elapsed: %.1f s (%.2f min)\n', passElapsedSeconds, passElapsedSeconds / 60);
end

function localRefreshPlot(outputDir)
try
    trimNewMapPlotOptions = struct();
    trimNewMapPlotOptions.show_popup = false;
    trimNewMapPlotOptions.show_guide_curve = true;
    trimNewMapPlotOptions.output_dir = outputDir;
    trimNewMapPlotOptions.guide_curve_vinf_mps = [0 70];
    trimNewMapPlotOptions.guide_curve_tilt_deg = [0 90];
    trimNewMapPlotOptions.guide_curve_label = "Straight-line corridor center";
    assignin('base', 'trimNewMapPlotOptions', trimNewMapPlotOptions);
    evalin('base', 'Plot_Trim_NewMap;');
catch ME
    fprintf('Plot refresh failed: %s\n', ME.message);
end
end

function localFinish(runStart, logFile)
runEnd = datetime('now', 'TimeZone', 'local');
elapsedSeconds = seconds(runEnd - runStart);
fprintf('\n=== Straight-line corridor sweep complete ===\n');
fprintf('Finished: %s\n', char(runEnd));
fprintf('Elapsed: %.1f s (%.2f hr)\n', elapsedSeconds, elapsedSeconds / 3600);
fprintf('Log file: %s\n', logFile);
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
