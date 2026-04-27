% Run_NewMap_AdaptiveFrontierQueue.m
% DB-driven trim search for the new Vinf/tilt/alpha map.
%
% This is intentionally narrower than the failed broad overnight grids:
%   1. read databases/trim_vinf_alpha_v1/trim_attempts.csv
%   2. propose targets adjacent to exact/acceptable points
%   3. skip already-solved and densely-failed local neighborhoods
%   4. run those explicit targets through the canonical TrimSearch_Run path

scriptPath = mfilename('fullpath');
rootDir = fileparts(fileparts(fileparts(scriptPath)));
cd(rootDir);

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
queueOutputDir = fullfile(rootDir, 'workspace_plots', ...
    sprintf('newmap_adaptive_frontier_%s', timestamp));
if exist(queueOutputDir, 'dir') ~= 7
    mkdir(queueOutputDir);
end

logFile = fullfile(queueOutputDir, 'adaptive_frontier_queue.log');
diary(logFile);
diaryCleanup = onCleanup(@() diary('off'));

queueStart = datetime('now', 'TimeZone', 'local');
fprintf('=== Run_NewMap_AdaptiveFrontierQueue ===\n');
fprintf('Started: %s\n', char(queueStart));
fprintf('Root: %s\n', rootDir);
fprintf('Queue output: %s\n\n', queueOutputDir);

builderOptions = localBuilderOptions(queueOutputDir);
[targetTable, targetSummary] = Build_NewMap_AdaptiveFrontierTargets(builderOptions);
fprintf('Adaptive target builder:\n');
fprintf('  attempts in DB = %d\n', targetSummary.attempt_rows);
fprintf('  usable anchors = %d\n', targetSummary.anchor_rows);
fprintf('  proposed targets = %d\n', height(targetTable));
fprintf('  preview CSV = %s\n\n', char(targetSummary.preview_csv));

if isempty(targetTable)
    fprintf('No adaptive frontier targets were generated. Nothing to run.\n');
else
    trimOptions = localTrimOptions(targetTable);
    transitionTrimSearchOptions = trimOptions;
    TrimSearch_Run;

    try
        trimNewMapPlotOptions = struct();
        trimNewMapPlotOptions.show_popup = false;
        trimNewMapPlotOptions.show_guide_curve = true;
        trimNewMapPlotOptions.output_dir = queueOutputDir;
        trimNewMapPlotOptions.guide_curve_vinf_mps = [0 10 20 30 40 50 60 70];
        trimNewMapPlotOptions.guide_curve_tilt_deg = [0 15 35 55 70 80 85 90];
        Plot_Trim_NewMap;
    catch ME
        fprintf('Plot refresh failed: %s\n', ME.message);
    end
end

queueEnd = datetime('now', 'TimeZone', 'local');
elapsedSeconds = seconds(queueEnd - queueStart);
fprintf('\n=== Adaptive frontier queue complete ===\n');
fprintf('Finished: %s\n', char(queueEnd));
fprintf('Elapsed: %.1f s (%.2f min)\n', elapsedSeconds, elapsedSeconds / 60);
fprintf('Log file: %s\n', logFile);

function opts = localBuilderOptions(queueOutputDir)
opts = struct();
opts.preview_csv = fullfile(queueOutputDir, 'adaptive_frontier_targets.csv');
opts.max_targets = 140;
opts.vinf_bounds_mps = [0 75];
opts.tilt_bounds_deg = [0 95];
opts.alpha_bounds_deg = [-5 17.5];
opts.vinf_step_mps = 2.5;
opts.tilt_step_deg = 2.5;
opts.alpha_step_deg = 2.5;
opts.vinf_offsets_mps = [-5 -2.5 0 2.5 5];
opts.tilt_offsets_deg = [-5 -2.5 0 2.5 5];
opts.alpha_offsets_deg = [-2.5 0 2.5];
opts.max_local_failed_attempts = 4;
opts.skip_existing_good_targets = true;
opts.skip_dense_failures = true;
opts.enforce_guide_corridor = true;
opts.max_guide_tilt_error_deg = 22.5;
end

function opts = localTrimOptions(targetTable)
opts = struct();
opts.profile = 'force_balance_newmap';
opts.database_name = 'trim_vinf_alpha_v1';
opts.output_prefix = 'newmap_adaptive_frontier';
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
end
