% Run_NewMap_OvernightQueue.m
% Sequential overnight trim-search queue for the new Vinf/tilt/alpha map.
%
% This intentionally uses the canonical TrimSearch_Run entrypoint. Each pass
% checkpoints directly into databases/trim_vinf_alpha_v1/trim_attempts.*.

scriptPath = mfilename('fullpath');
rootDir = fileparts(fileparts(fileparts(scriptPath)));
cd(rootDir);

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
queueOutputDir = fullfile(rootDir, 'workspace_plots', ...
    sprintf('newmap_overnight_queue_%s', timestamp));
if exist(queueOutputDir, 'dir') ~= 7
    mkdir(queueOutputDir);
end

logFile = fullfile(queueOutputDir, 'overnight_queue.log');
diary(logFile);
diaryCleanup = onCleanup(@() diary('off'));

fprintf('=== Run_NewMap_OvernightQueue ===\n');
fprintf('Started: %s\n', char(datetime('now', 'TimeZone', 'local')));
fprintf('Root: %s\n', rootDir);
fprintf('Queue output: %s\n\n', queueOutputDir);

common = localCommonOptions();

passes = {};

highGrid = common;
highGrid.output_prefix = 'newmap_overnight_highspeed_fixed_grid';
highGrid.target_strategy = 'bridge_frontier';
highGrid.vinf_grid_mps = 40:5:75;
highGrid.tilt_grid_deg = 60:5:95;
highGrid.alpha_grid_deg = [0 2.5 5 7.5 10 12.5];
passes{end + 1} = highGrid;

midGrid = common;
midGrid.output_prefix = 'newmap_overnight_mid_transition_fixed_grid';
midGrid.target_strategy = 'bridge_frontier';
midGrid.vinf_grid_mps = 20:5:45;
midGrid.tilt_grid_deg = 25:5:80;
midGrid.alpha_grid_deg = [0 2.5 5 7.5 10 12.5 15];
passes{end + 1} = midGrid;

cruiseFine = common;
cruiseFine.output_prefix = 'newmap_overnight_cruise_fine_grid';
cruiseFine.target_strategy = 'bridge_frontier';
cruiseFine.vinf_grid_mps = 55:2.5:75;
cruiseFine.tilt_grid_deg = 75:2.5:95;
cruiseFine.alpha_grid_deg = [0 2.5 5 7.5];
passes{end + 1} = cruiseFine;

midFine = common;
midFine.output_prefix = 'newmap_overnight_mid_fine_grid';
midFine.target_strategy = 'bridge_frontier';
midFine.vinf_grid_mps = 30:2.5:50;
midFine.tilt_grid_deg = 55:2.5:85;
midFine.alpha_grid_deg = [2.5 5 7.5 10 12.5];
passes{end + 1} = midFine;

queueTimer = tic;
for iPass = 1:numel(passes)
    localRunPass(iPass, numel(passes), passes{iPass});
end

fprintf('\n=== Overnight queue complete ===\n');
fprintf('Elapsed: %.1f s (%.2f hr)\n', toc(queueTimer), toc(queueTimer) / 3600);

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

fprintf('Finished: %s\n', char(datetime('now', 'TimeZone', 'local')));
fprintf('Log file: %s\n', logFile);

function opts = localCommonOptions()
opts = struct();
opts.profile = 'force_balance_newmap';
opts.database_name = 'trim_vinf_alpha_v1';
opts.target_strategy = 'guide_grid';
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
opts.neighbor_seed_count = 0;
opts.history_seed_count = 0;
opts.family_names = {'prop_fixed_flap_elevator', ...
    'rear_fixed_flap_elevator', ...
    'front_rear_free_flap_elevator'};
opts.stop_after_exact = true;

opts.write_debug_run_outputs = false;
opts.write_linearizations_to_db = false;
opts.update_master_attempt_db_on_checkpoint = true;
opts.refresh_canonical_databases_on_checkpoint = false;
opts.checkpoint_every = 1;
opts.failed_seed_filter_enabled = false;
end

function localRunPass(iPass, nPasses, options)
fprintf('\n=== Overnight pass %d/%d: %s ===\n', ...
    iPass, nPasses, char(string(options.output_prefix)));
passStart = datetime('now', 'TimeZone', 'local');
fprintf('Pass started: %s\n', char(passStart));
try
    transitionTrimSearchOptions = options; %#ok<NASGU>
    TrimSearch_Run;
    fprintf('Pass complete: %s\n', char(datetime('now', 'TimeZone', 'local')));
catch ME
    fprintf('Pass failed: %s\n', ME.message);
    for iStack = 1:numel(ME.stack)
        fprintf('  at %s:%d\n', ME.stack(iStack).file, ME.stack(iStack).line);
    end
end
passElapsedSeconds = seconds(datetime('now', 'TimeZone', 'local') - passStart);
fprintf('Pass elapsed: %.1f s (%.2f min)\n', passElapsedSeconds, passElapsedSeconds / 60);
end
