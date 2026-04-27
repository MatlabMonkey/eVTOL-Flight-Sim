function map = Run_INDI_Surface_Effectiveness_Map(mode)
%RUN_INDI_SURFACE_EFFECTIVENESS_MAP Entry point for INDI aero surface maps.
%
% Usage:
%   Run_INDI_Surface_Effectiveness_Map('smoke')         % quick sanity check
%   Run_INDI_Surface_Effectiveness_Map('coarse')        % durable coarse map
%   Run_INDI_Surface_Effectiveness_Map('fillin_offset2')% second fill-in map
%   Run_INDI_Surface_Effectiveness_Map('migrate_database')
%   Run_INDI_Surface_Effectiveness_Map('delta_dense_transition')
%   Run_INDI_Surface_Effectiveness_Map('transition_alpha_band_delta_dense')
%   Run_INDI_Surface_Effectiveness_Map('alpha_refined_global')
%   Run_INDI_Surface_Effectiveness_Map('stall_boundary_delta_alpha_dense')
%   Run_INDI_Surface_Effectiveness_Map('low_speed_high_alpha_fillin')
%   Run_INDI_Surface_Effectiveness_Map('poststall_high_alpha_fillin')
%   Run_INDI_Surface_Effectiveness_Map('low_speed_stall_boundary_dense')

if nargin < 1 || isempty(mode)
    mode = 'smoke';
end

repoRoot = fileparts(mfilename('fullpath'));
buildersDir = fullfile(repoRoot, 'controllers', 'builders');
if exist(buildersDir, 'dir') == 7
    addpath(buildersDir);
end

entryName = "";
mergeToDatabase = false;

switch lower(string(mode))
    case {"smoke", "quick"}
        outputDir = fullfile(repoRoot, 'workspace_plots');
        if exist(outputDir, 'dir') ~= 7
            mkdir(outputDir);
        end

        opts = struct();
        opts.vinf_mps = [10, 30];
        opts.alpha_deg = [0, 20];
        opts.delta_deg = [-10, 0, 10];
        opts.outputMatPath = fullfile(outputDir, 'indi_surface_effectiveness_map_fast_smoke.mat');
        opts.outputSummaryPath = fullfile(outputDir, 'indi_surface_effectiveness_map_fast_smoke.md');
        opts.checkpointEveryCalls = 0;
        opts.progressEveryCalls = 4;

    case {"migrate_database", "migrate"}
        map = [];
        localMigrateExistingMaps(repoRoot);
        return;

    case {"refresh_database_summary", "refresh_database"}
        map = [];
        indiMaps = localLoadCombinedDatabase(localCombinedDatabasePath(repoRoot));
        localWriteCombinedSummary(repoRoot, indiMaps);
        return;

    case "coarse"
        outputDir = fullfile(repoRoot, 'databases');
        if exist(outputDir, 'dir') ~= 7
            mkdir(outputDir);
        end

        opts = struct();
        opts.outputMatPath = fullfile(outputDir, 'indi_surface_effectiveness_map_coarse_polar_fast.mat');
        opts.outputSummaryPath = fullfile(outputDir, 'indi_surface_effectiveness_map_coarse_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "coarse";
        mergeToDatabase = true;

    case {"fillin_offset2", "fillin2"}
        outputDir = fullfile(repoRoot, 'databases');
        if exist(outputDir, 'dir') ~= 7
            mkdir(outputDir);
        end

        % First fill-in grid, kept here for traceability:
        %   Vinf_mps  = [7.5 12.5 17.5 25 35 45 55 65 75]
        %   alpha_deg = -7.5:5:42.5
        %   delta_deg = -22.5:5:22.5
        %
        % This second fill-in uses quarter offsets to avoid reusing coarse
        % grid points or the first half-step fill-in grid points.
        opts = struct();
        opts.vinf_mps = [3.75 6.25 8.75 11.25 13.75 16.25 18.75 ...
            22.5 27.5 32.5 37.5 42.5 47.5 52.5 57.5 62.5 67.5 72.5];
        opts.alpha_deg = -8.75:5:41.25;
        opts.delta_deg = -23.75:5:21.25;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(outputDir, 'indi_surface_effectiveness_map_fillin_offset2_polar_fast.mat');
        opts.outputSummaryPath = fullfile(outputDir, 'indi_surface_effectiveness_map_fillin_offset2_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "fillin_offset2";
        mergeToDatabase = true;

    case {"delta_dense_transition", "delta_dense"}
        opts = struct();
        opts.vinf_mps = 10:5:60;
        opts.alpha_deg = 0:5:35;
        opts.delta_deg = -25:2.5:25;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_delta_dense_transition_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_delta_dense_transition_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "delta_dense_transition";
        mergeToDatabase = true;

    case {"transition_alpha_band_delta_dense", "alpha_band_delta_dense"}
        opts = struct();
        opts.vinf_mps = 30:5:60;
        opts.alpha_deg = [0, 2.5, 5, 7.5, 10, 12.5, 15];
        opts.delta_deg = -25:1.25:25;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_transition_alpha_band_delta_dense_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_transition_alpha_band_delta_dense_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "transition_alpha_band_delta_dense";
        mergeToDatabase = true;

    case {"alpha_refined_global", "global_alpha_refined"}
        opts = struct();
        opts.vinf_mps = [2.5:2.5:20, 25:5:80];
        opts.alpha_deg = -10:2.5:45;
        opts.delta_deg = -25:5:25;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_alpha_refined_global_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_alpha_refined_global_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "alpha_refined_global";
        mergeToDatabase = true;

    case {"stall_boundary_delta_alpha_dense", "stall_boundary_dense"}
        opts = struct();
        opts.vinf_mps = [15:2.5:30, 35:5:80];
        opts.alpha_deg = -5:1.25:25;
        opts.delta_deg = -25:1.25:25;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_stall_boundary_delta_alpha_dense_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_stall_boundary_delta_alpha_dense_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "stall_boundary_delta_alpha_dense";
        mergeToDatabase = true;

    case {"low_speed_high_alpha_fillin", "low_speed_alpha_fillin"}
        opts = struct();
        opts.vinf_mps = [3.75, 6.25, 8.75, 11.25, 13.75];
        opts.alpha_deg = -9.375:1.25:44.375;
        opts.delta_deg = -23.75:2.5:23.75;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_low_speed_high_alpha_fillin_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_low_speed_high_alpha_fillin_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "low_speed_high_alpha_fillin";
        mergeToDatabase = true;

    case {"poststall_high_alpha_fillin", "high_alpha_fillin"}
        opts = struct();
        opts.vinf_mps = 17.5:5:77.5;
        opts.alpha_deg = 26.25:1.25:44.375;
        opts.delta_deg = -23.75:2.5:23.75;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_poststall_high_alpha_fillin_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_poststall_high_alpha_fillin_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "poststall_high_alpha_fillin";
        mergeToDatabase = true;

    case {"low_speed_stall_boundary_dense", "low_speed_stall_dense"}
        opts = struct();
        opts.vinf_mps = 2.5:2.5:15;
        opts.alpha_deg = -5:1.25:25;
        opts.delta_deg = -25:1.25:25;
        opts.perturbation_deg = 0.5;
        opts.outputMatPath = fullfile(tempdir, 'indi_surface_effectiveness_map_low_speed_stall_boundary_dense_polar_fast.mat');
        opts.outputSummaryPath = fullfile(tempdir, 'indi_surface_effectiveness_map_low_speed_stall_boundary_dense_polar_fast.md');
        opts.checkpointEveryCalls = 500;
        opts.progressEveryCalls = 100;
        entryName = "low_speed_stall_boundary_dense";
        mergeToDatabase = true;

    otherwise
        error('Run_INDI_Surface_Effectiveness_Map:UnknownMode', ...
            ['Unknown mode "%s". Use "smoke", "coarse", "fillin_offset2", ', ...
            '"migrate_database", "delta_dense_transition", or ', ...
            '"transition_alpha_band_delta_dense", "alpha_refined_global", ', ...
            '"stall_boundary_delta_alpha_dense", "low_speed_high_alpha_fillin", ', ...
            '"poststall_high_alpha_fillin", or "low_speed_stall_boundary_dense".'], mode);
end

map = build_indi_surface_effectiveness_map(opts);
if mergeToDatabase
    localMergeMapIntoDatabase(repoRoot, entryName, map, opts.outputMatPath);
end
end

function localMigrateExistingMaps(repoRoot)
specs = { ...
    'coarse', fullfile(repoRoot, 'databases', 'indi_surface_effectiveness_map_coarse_polar_fast.mat'); ...
    'fillin', fullfile(repoRoot, 'databases', 'indi_surface_effectiveness_map_fillin_polar_fast.mat'); ...
    'fillin_offset2', fullfile(repoRoot, 'databases', 'indi_surface_effectiveness_map_fillin_offset2_polar_fast.mat')};

for k = 1:size(specs, 1)
    entryName = string(specs{k, 1});
    mapPath = specs{k, 2};
    if exist(mapPath, 'file') ~= 2
        fprintf('Skipping missing map: %s\n', mapPath);
        continue;
    end

    loaded = load(mapPath, 'map');
    localMergeMapIntoDatabase(repoRoot, entryName, loaded.map, mapPath);
end
end

function localMergeMapIntoDatabase(repoRoot, entryName, map, sourceMatPath)
databasePath = localCombinedDatabasePath(repoRoot);
[databaseDir, ~, ~] = fileparts(databasePath);
if exist(databaseDir, 'dir') ~= 7
    mkdir(databaseDir);
end

indiMaps = localLoadCombinedDatabase(databasePath);
map.metadata.database_entry_name = char(entryName);
map.metadata.database_source_mat_path = sourceMatPath;

entry = struct();
entry.name = char(entryName);
entry.source_mat_path = sourceMatPath;
entry.added_at = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
entry.map = map;

names = string({indiMaps.entries.name});
idx = find(names == entryName, 1);
if isempty(idx)
    indiMaps.entries(end + 1) = entry;
else
    indiMaps.entries(idx) = entry;
end

indiMaps.updated_at = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
save(databasePath, 'indiMaps', '-v7.3');
localWriteCombinedSummary(repoRoot, indiMaps);

fprintf('Merged map into combined INDI database:\n');
fprintf('  entry    : %s\n', entryName);
fprintf('  database : %s\n', databasePath);
end

function indiMaps = localLoadCombinedDatabase(databasePath)
if exist(databasePath, 'file') == 2
    loaded = load(databasePath, 'indiMaps');
    indiMaps = loaded.indiMaps;
    return;
end

indiMaps = struct();
indiMaps.version = 1;
indiMaps.description = 'Combined INDI surface effectiveness maps from Trim_Plant';
indiMaps.created_at = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
indiMaps.updated_at = indiMaps.created_at;
indiMaps.entries = struct('name', {}, 'source_mat_path', {}, 'added_at', {}, 'map', {});
end

function localWriteCombinedSummary(repoRoot, indiMaps)
summaryPath = localCombinedSummaryPath(repoRoot);
fid = fopen(summaryPath, 'w');
if fid < 0
    error('Run_INDI_Surface_Effectiveness_Map:SummaryOpenFailed', ...
        'Unable to write %s.', summaryPath);
end
cleanupFile = onCleanup(@() fclose(fid));

fprintf(fid, '# INDI Surface Effectiveness Map Database\n\n');
fprintf(fid, 'Last reviewed: %s\n\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf(fid, 'Combined database: `databases/indi_surface_effectiveness_maps_polar_fast.mat`\n\n');
fprintf(fid, '## Entries\n\n');
fprintf(fid, '| name | V count | alpha count | delta count | calls | elapsed min |\n');
fprintf(fid, '| --- | ---: | ---: | ---: | ---: | ---: |\n');

for k = 1:numel(indiMaps.entries)
    entry = indiMaps.entries(k);
    map = entry.map;
    fprintf(fid, '| `%s` | %d | %d | %d | %d | %.1f |\n', ...
        entry.name, ...
        numel(map.grid.vinf_mps), ...
        numel(map.grid.alpha_deg), ...
        numel(map.grid.delta_deg), ...
        map.metadata.calls_completed, ...
        map.metadata.elapsed_s / 60.0);
end

fprintf(fid, '\n## Notes\n\n');
fprintf(fid, 'This file indexes multiple non-identical sampling grids. Keep individual entries separate until an explicit merge/interpolation product is built.\n');
end

function databasePath = localCombinedDatabasePath(repoRoot)
databasePath = fullfile(repoRoot, 'databases', 'indi_surface_effectiveness_maps_polar_fast.mat');
end

function summaryPath = localCombinedSummaryPath(repoRoot)
summaryPath = fullfile(repoRoot, 'databases', 'indi_surface_effectiveness_maps_polar_fast.md');
end
