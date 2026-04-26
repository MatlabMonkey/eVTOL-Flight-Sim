% TrimDB_Build.m
% Build the canonical trim databases:
%   1. trim_attempts.*
%   2. controller_schedule.*
%
% Preferred workflow:
%   - new trim searches write directly into trim_attempts
%   - new trim searches save linearizationPoint files and link them from the master DB
%   - this builder rebuilds controller_schedule from the canonical master DB
%   - explicit CSV source import remains as a legacy recovery/rebuild mode
%
% Usage:
%   TrimDB_Build
%
% Optional configuration:
%   transitionTrimDatabaseOptions = struct( ...
%       'rebuild_master_from_sources', false, ...
%       'include_smoke_sources', false, ...
%       'controller_include_quasi_trims', false, ...
%       'controller_allow_trim_only_candidates', false, ...
%       'controller_exclude_zero_rear', true, ...
%       'preserve_existing_controller_points', true, ...
%       'write_controller_diagnostics', false, ...
%       'extra_trim_sources', {'legacy_search_summary.csv'}, ...
%       'extra_linearization_index_sources', {'my_linearization_index.csv'}, ...
%       'extra_linearization_roots', {'/abs/path/to/my_linearizations'}, ...
%       'include_full_order_linearization', true);
%   TrimDB_Build

if ~exist('transitionTrimDatabaseOptions', 'var') || ~isstruct(transitionTrimDatabaseOptions)
    transitionTrimDatabaseOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    rootDir = fileparts(stack(1).file);
else
    rootDir = pwd;
end
dbPaths = TrimDB_Paths(rootDir);
workspacePlotsDir = dbPaths.workspace_plots_dir;

transitionTrimDatabaseOptions = localApplyDefaults(transitionTrimDatabaseOptions, rootDir, dbPaths);
rootDir = transitionTrimDatabaseOptions.root_dir;
workspacePlotsDir = transitionTrimDatabaseOptions.workspace_plots_dir;
databaseDir = transitionTrimDatabaseOptions.database_dir;
masterMatFile = transitionTrimDatabaseOptions.master_mat_file;
masterCsvFile = transitionTrimDatabaseOptions.master_csv_file;
masterMdFile = transitionTrimDatabaseOptions.master_md_file;
controllerMatFile = transitionTrimDatabaseOptions.controller_mat_file;
controllerCsvFile = transitionTrimDatabaseOptions.controller_csv_file;
controllerMdFile = transitionTrimDatabaseOptions.controller_md_file;
missingLinearizationsCsvFile = transitionTrimDatabaseOptions.controller_missing_linearizations_csv_file;
buildLogFile = transitionTrimDatabaseOptions.controller_build_log_file;
manifest = localBuildSourceManifest(transitionTrimDatabaseOptions, workspacePlotsDir, databaseDir);

fprintf('=== TrimDB_Build ===\n');
fprintf('databases: %s\n', databaseDir);
fprintf('workspace_plots: %s\n', workspacePlotsDir);

[masterAttemptAllRows, trimSourceStatus, masterSourceMode] = localResolveMasterAttemptRows( ...
    transitionTrimDatabaseOptions, manifest, masterMatFile, masterCsvFile);
[linearizationIndex, linearizationSourceStatus] = localReadLinearizationIndices( ...
    manifest.linearization_index_sources, manifest.allowed_linearization_roots);
[masterAttemptAllRows, linearizationMatchSummary] = localAttachLinearizationMatches( ...
    masterAttemptAllRows, linearizationIndex, transitionTrimDatabaseOptions);
masterAttemptBestUnique = localBuildBestUniquePoints(masterAttemptAllRows);

[controllerSummaryTable, controllerPoints, missingLinearizations, controllerBuildSummary] = ...
    localBuildControllerSchedule(masterAttemptBestUnique, transitionTrimDatabaseOptions);
rebuiltControllerCandidateCount = height(controllerSummaryTable);
rebuiltMissingLinearizationCount = height(missingLinearizations);
preservedExistingControllerDb = false;
mergeSummary = localEmptyControllerMergeSummary();

if transitionTrimDatabaseOptions.preserve_existing_controller_points
    [existingControllerDb, existingControllerRows] = localLoadExistingControllerScheduleDb(controllerMatFile);
    if existingControllerRows > 0
        [controllerSummaryTable, controllerPoints, mergeSummary] = localMergeControllerSchedule( ...
            controllerSummaryTable, controllerPoints, existingControllerDb);
        preservedExistingControllerDb = mergeSummary.preserved_existing_rows > 0 || ...
            mergeSummary.replaced_existing_rows > 0;
    end
end
controllerBuildSummary.rebuilt_controller_rows_before_merge = rebuiltControllerCandidateCount;
controllerBuildSummary.existing_controller_rows_before_merge = mergeSummary.existing_rows_loaded;
controllerBuildSummary.preserved_existing_controller_rows = mergeSummary.preserved_existing_rows;
controllerBuildSummary.replaced_existing_controller_rows = mergeSummary.replaced_existing_rows;
controllerBuildSummary.added_rebuilt_controller_rows = mergeSummary.added_rebuilt_rows;
controllerBuildSummary.controller_rows_after_merge = height(controllerSummaryTable);

transitionTrimMasterAttemptDB = struct();
transitionTrimMasterAttemptDB.meta = localBuildMetaStruct(rootDir, databaseDir, workspacePlotsDir, transitionTrimDatabaseOptions);
transitionTrimMasterAttemptDB.manifest = manifest;
transitionTrimMasterAttemptDB.manifest.mode = masterSourceMode;
transitionTrimMasterAttemptDB.source_status = trimSourceStatus;
transitionTrimMasterAttemptDB.linearization_index_status = linearizationSourceStatus;
transitionTrimMasterAttemptDB.linearization_match_summary = linearizationMatchSummary;
transitionTrimMasterAttemptDB.master_attempt_db_all_rows = masterAttemptAllRows;
transitionTrimMasterAttemptDB.master_attempt_db_best_unique_points = masterAttemptBestUnique;

controllerScheduleDB = struct();
controllerScheduleDB.meta = localBuildMetaStruct(rootDir, databaseDir, workspacePlotsDir, transitionTrimDatabaseOptions);
controllerScheduleDB.meta.preserved_existing_controller_db = preservedExistingControllerDb;
controllerScheduleDB.meta.preserved_existing_controller_db_file = string(controllerMatFile);
controllerScheduleDB.manifest = transitionTrimMasterAttemptDB.manifest;
controllerScheduleDB.source_status = trimSourceStatus;
controllerScheduleDB.linearization_index_status = linearizationSourceStatus;
controllerScheduleDB.linearization_match_summary = linearizationMatchSummary;
controllerScheduleDB.summary_table = controllerSummaryTable;
controllerScheduleDB.points = controllerPoints;
controllerScheduleDB.missing_linearizations = missingLinearizations;
controllerScheduleDB.build_summary = controllerBuildSummary;

validationSummary = localRunValidationChecks( ...
    masterAttemptAllRows, controllerSummaryTable, controllerPoints, ...
    missingLinearizations, manifest, transitionTrimDatabaseOptions);
transitionTrimMasterAttemptDB.validation = validationSummary;
controllerScheduleDB.validation = validationSummary;

save(masterMatFile, 'transitionTrimMasterAttemptDB', '-v7.3');
writetable(masterAttemptAllRows, masterCsvFile);
save(controllerMatFile, 'controllerScheduleDB', '-v7.3');
writetable(controllerSummaryTable, controllerCsvFile);
if transitionTrimDatabaseOptions.write_controller_diagnostics
    writetable(missingLinearizations, missingLinearizationsCsvFile);
end

localWriteMasterMarkdown(masterMdFile, transitionTrimMasterAttemptDB);
localWriteControllerMarkdown(controllerMdFile, controllerScheduleDB);
if transitionTrimDatabaseOptions.write_controller_diagnostics
    localWriteBuildLog(buildLogFile, transitionTrimMasterAttemptDB, controllerScheduleDB);
end

assignin('base', 'transitionTrimMasterAttemptDB', transitionTrimMasterAttemptDB);
assignin('base', 'controllerScheduleDB', controllerScheduleDB);
assignin('base', 'transitionTrimDatabaseOptions', transitionTrimDatabaseOptions);

fprintf('Master rows: %d\n', height(masterAttemptAllRows));
fprintf('Best unique points: %d\n', height(masterAttemptBestUnique));
fprintf('Controller candidates rebuilt before merge: %d\n', rebuiltControllerCandidateCount);
fprintf('Controller candidates missing linearization during rebuild: %d\n', rebuiltMissingLinearizationCount);
fprintf('Controller candidates kept after merge: %d\n', height(controllerSummaryTable));
if preservedExistingControllerDb
    fprintf('Preserved existing controller rows: %d\n', mergeSummary.preserved_existing_rows);
end
fprintf('Wrote:\n');
fprintf('  %s\n', masterMatFile);
fprintf('  %s\n', masterCsvFile);
fprintf('  %s\n', masterMdFile);
fprintf('  %s\n', controllerMatFile);
fprintf('  %s\n', controllerCsvFile);
fprintf('  %s\n', controllerMdFile);
if transitionTrimDatabaseOptions.write_controller_diagnostics
    fprintf('  %s\n', missingLinearizationsCsvFile);
    fprintf('  %s\n', buildLogFile);
end

function [controllerDb, rowCount] = localLoadExistingControllerScheduleDb(controllerMatFile)
controllerDb = struct();
rowCount = 0;
if exist(controllerMatFile, 'file') ~= 2
    return;
end
raw = load(controllerMatFile, 'controllerScheduleDB');
if ~isfield(raw, 'controllerScheduleDB') || ~isstruct(raw.controllerScheduleDB)
    return;
end
candidate = raw.controllerScheduleDB;
if ~isfield(candidate, 'summary_table') || ~istable(candidate.summary_table)
    return;
end
if ~isfield(candidate, 'points')
    return;
end
rowCount = height(candidate.summary_table);
controllerDb = candidate;
end

function summary = localEmptyControllerMergeSummary()
summary = struct( ...
    'existing_rows_loaded', 0, ...
    'rebuilt_rows_loaded', 0, ...
    'preserved_existing_rows', 0, ...
    'replaced_existing_rows', 0, ...
    'added_rebuilt_rows', 0);
end

function [summaryTable, mergedPoints, mergeSummary] = localMergeControllerSchedule(summaryTable, rebuiltPoints, existingControllerDb)
mergeSummary = localEmptyControllerMergeSummary();
if ~isfield(existingControllerDb, 'points') || isempty(existingControllerDb.points)
    [summaryTable, mergedPoints] = localSortControllerSchedulePoints(rebuiltPoints);
    return;
end

existingPoints = existingControllerDb.points(:);
rebuiltPoints = rebuiltPoints(:);
mergeSummary.existing_rows_loaded = numel(existingPoints);
mergeSummary.rebuilt_rows_loaded = numel(rebuiltPoints);

if isempty(existingPoints)
    [summaryTable, mergedPoints] = localSortControllerSchedulePoints(rebuiltPoints);
    return;
end

mergedPoints = repmat(localEmptyControllerPoint(), numel(existingPoints), 1);
mergedKeys = strings(numel(existingPoints), 1);
for i = 1:numel(existingPoints)
    mergedPoints(i) = localNormalizeControllerPoint(existingPoints(i));
    mergedKeys(i) = string(mergedPoints(i).key);
end

for i = 1:numel(rebuiltPoints)
    rebuiltPoint = localNormalizeControllerPoint(rebuiltPoints(i));
    rebuiltKey = string(rebuiltPoint.key);
    existingIdx = find(mergedKeys == rebuiltKey, 1, 'first');
    if isempty(existingIdx)
        mergedPoints(end + 1, 1) = rebuiltPoint; %#ok<AGROW>
        mergedKeys(end + 1, 1) = rebuiltKey; %#ok<AGROW>
        mergeSummary.added_rebuilt_rows = mergeSummary.added_rebuilt_rows + 1;
    else
        mergedPoints(existingIdx) = rebuiltPoint;
        mergeSummary.replaced_existing_rows = mergeSummary.replaced_existing_rows + 1;
    end
end

mergeSummary.preserved_existing_rows = mergeSummary.existing_rows_loaded - mergeSummary.replaced_existing_rows;
[summaryTable, mergedPoints] = localSortControllerSchedulePoints(mergedPoints);
end

function point = localNormalizeControllerPoint(pointIn)
point = localEmptyControllerPoint();
fields = fieldnames(point);
for i = 1:numel(fields)
    name = fields{i};
    if isfield(pointIn, name)
        point.(name) = pointIn.(name);
    end
end
latestFile = string(point.linearization_latest_file);
if strlength(latestFile) > 0 && exist(latestFile, 'file') ~= 2
    point.linearization_latest_file = "";
end
end

function opts = localApplyDefaults(opts, rootDir, dbPaths)
if ~isfield(opts, 'root_dir') || isempty(opts.root_dir)
    opts.root_dir = rootDir;
end
if ~isfield(opts, 'workspace_plots_dir') || isempty(opts.workspace_plots_dir)
    opts.workspace_plots_dir = dbPaths.workspace_plots_dir;
end
if ~isfield(opts, 'database_dir') || isempty(opts.database_dir)
    opts.database_dir = dbPaths.database_dir;
end
if exist(opts.database_dir, 'dir') ~= 7
    mkdir(opts.database_dir);
end
if exist(opts.workspace_plots_dir, 'dir') ~= 7
    mkdir(opts.workspace_plots_dir);
end
if ~isfield(opts, 'master_mat_file') || isempty(opts.master_mat_file)
    opts.master_mat_file = fullfile(opts.database_dir, 'trim_attempts.mat');
end
if ~isfield(opts, 'master_csv_file') || isempty(opts.master_csv_file)
    opts.master_csv_file = fullfile(opts.database_dir, 'trim_attempts.csv');
end
if ~isfield(opts, 'master_md_file') || isempty(opts.master_md_file)
    opts.master_md_file = fullfile(opts.database_dir, 'trim_attempts.md');
end
if ~isfield(opts, 'controller_mat_file') || isempty(opts.controller_mat_file)
    opts.controller_mat_file = fullfile(opts.database_dir, 'controller_schedule.mat');
end
if ~isfield(opts, 'controller_csv_file') || isempty(opts.controller_csv_file)
    opts.controller_csv_file = fullfile(opts.database_dir, 'controller_schedule.csv');
end
if ~isfield(opts, 'controller_md_file') || isempty(opts.controller_md_file)
    opts.controller_md_file = fullfile(opts.database_dir, 'controller_schedule.md');
end
if ~isfield(opts, 'controller_missing_linearizations_csv_file') || isempty(opts.controller_missing_linearizations_csv_file)
    opts.controller_missing_linearizations_csv_file = fullfile(opts.database_dir, ...
        'controller_schedule_missing_linearizations.csv');
end
if ~isfield(opts, 'controller_build_log_file') || isempty(opts.controller_build_log_file)
    opts.controller_build_log_file = fullfile(opts.database_dir, ...
        'controller_schedule_build_log.md');
end
if ~isfield(opts, 'allow_empty_controller_overwrite') || isempty(opts.allow_empty_controller_overwrite)
    opts.allow_empty_controller_overwrite = false;
end
if ~isfield(opts, 'preserve_existing_controller_points') || isempty(opts.preserve_existing_controller_points)
    opts.preserve_existing_controller_points = true;
end
if ~isfield(opts, 'write_controller_diagnostics') || isempty(opts.write_controller_diagnostics)
    opts.write_controller_diagnostics = false;
end
if ~isfield(opts, 'include_smoke_sources') || isempty(opts.include_smoke_sources)
    opts.include_smoke_sources = false;
end
if ~isfield(opts, 'rebuild_master_from_sources') || isempty(opts.rebuild_master_from_sources)
    opts.rebuild_master_from_sources = false;
end
if ~isfield(opts, 'controller_include_quasi_trims') || isempty(opts.controller_include_quasi_trims)
    opts.controller_include_quasi_trims = false;
end
if ~isfield(opts, 'controller_allow_trim_only_candidates') || isempty(opts.controller_allow_trim_only_candidates)
    opts.controller_allow_trim_only_candidates = false;
end
if ~isfield(opts, 'controller_exclude_zero_rear') || isempty(opts.controller_exclude_zero_rear)
    opts.controller_exclude_zero_rear = true;
end
if ~isfield(opts, 'extra_trim_sources') || isempty(opts.extra_trim_sources)
    opts.extra_trim_sources = {};
end
if ischar(opts.extra_trim_sources) || isstring(opts.extra_trim_sources)
    opts.extra_trim_sources = cellstr(opts.extra_trim_sources);
end
if ~isfield(opts, 'extra_linearization_index_sources') || isempty(opts.extra_linearization_index_sources)
    opts.extra_linearization_index_sources = {};
end
if ischar(opts.extra_linearization_index_sources) || isstring(opts.extra_linearization_index_sources)
    opts.extra_linearization_index_sources = cellstr(opts.extra_linearization_index_sources);
end
if ~isfield(opts, 'extra_linearization_roots') || isempty(opts.extra_linearization_roots)
    opts.extra_linearization_roots = {};
end
if ischar(opts.extra_linearization_roots) || isstring(opts.extra_linearization_roots)
    opts.extra_linearization_roots = cellstr(opts.extra_linearization_roots);
end
if ~isfield(opts, 'include_full_order_linearization') || isempty(opts.include_full_order_linearization)
    opts.include_full_order_linearization = true;
end
if ~isfield(opts, 'tilt_match_tolerance') || isempty(opts.tilt_match_tolerance)
    opts.tilt_match_tolerance = 1e-6;
end
if ~isfield(opts, 'vinf_match_tolerance') || isempty(opts.vinf_match_tolerance)
    opts.vinf_match_tolerance = 1e-6;
end
if ~isfield(opts, 'rpm_match_tolerance') || isempty(opts.rpm_match_tolerance)
    opts.rpm_match_tolerance = 1e-3;
end
end

function [allRows, sourceStatus, sourceMode] = localResolveMasterAttemptRows(opts, manifest, masterMatFile, masterCsvFile)
sourceMode = "existing_master_db";
sourceStatus = repmat(localSourceManifestTemplate(), 0, 1);

if ~opts.rebuild_master_from_sources
    [allRows, sourceStatus] = localLoadExistingMasterAttemptRows(masterMatFile, masterCsvFile);
    if ~isempty(allRows)
        return;
    end
end

sourceMode = "explicit_trim_sources";
if opts.rebuild_master_from_sources
    fprintf('Master source mode: explicit trim-source rebuild.\n');
else
    fprintf('Master source mode: explicit trim-source rebuild (no canonical master DB found).\n');
end
[allRows, sourceStatus] = localReadAndNormalizeTrimSources(manifest.trim_sources);
end

function [allRows, sourceStatus] = localLoadExistingMasterAttemptRows(masterMatFile, masterCsvFile)
allRows = localEmptyMasterTable();
sourceStatus = repmat(localSourceManifestTemplate(), 0, 1);

if exist(masterMatFile, 'file') == 2
    raw = load(masterMatFile);
    if isfield(raw, 'transitionTrimMasterAttemptDB') && isfield(raw.transitionTrimMasterAttemptDB, 'master_attempt_db_all_rows')
        allRows = raw.transitionTrimMasterAttemptDB.master_attempt_db_all_rows;
        allRows = localEnsureMasterSchema(allRows);
        sourceStatus = localCanonicalMasterSourceStatus(masterMatFile, height(allRows), "loaded from canonical master MAT");
        fprintf('Master source mode: existing canonical master DB (MAT).\n');
        return;
    end
end

if exist(masterCsvFile, 'file') == 2
    allRows = readtable(masterCsvFile, 'TextType', 'string');
    allRows = localEnsureMasterSchema(allRows);
    sourceStatus = localCanonicalMasterSourceStatus(masterCsvFile, height(allRows), "loaded from canonical master CSV");
    fprintf('Master source mode: existing canonical master DB (CSV).\n');
end
end

function status = localCanonicalMasterSourceStatus(pathValue, rowCount, reason)
status = localSourceManifestTemplate();
[~, name, ext] = fileparts(pathValue);
status.filename = string([name ext]);
status.path = string(pathValue);
status.enabled = true;
status.is_smoke = false;
status.reason = string(reason);
status.run_prefix = "canonical_master_db";
status.source_kind = "canonical_master_db";
status.status = "loaded";
status.row_count = rowCount;
end

function manifest = localBuildSourceManifest(opts, workspacePlotsDir, databaseDir)
trimFiles = { ...
    'transition_trim_map_latest.csv', ...
    'transition_trim_map_low_speed_latest.csv', ...
    'transition_trim_map_low_speed_scored_latest.csv', ...
    'transition_trim_map_bridge_scored_latest.csv', ...
    'transition_trim_map_merged_latest.csv', ...
    'transition_trim_reference_line_scored_latest.csv', ...
    'transition_trim_reference_line_midband_scored_latest.csv', ...
    'transition_trim_lowmid_guidegrid_scored_latest.csv', ...
    'transition_trim_lowmid_guidegrid_fast_latest.csv', ...
    'transition_trim_leftbridge_fast_latest.csv', ...
    'transition_trim_uppermid_bridge_fast_latest.csv', ...
    'transition_trim_mainbridge_fast_latest.csv', ...
    'transition_trim_mainbridge_rescue_fast_latest.csv', ...
    'transition_trim_mainbridge_handoff_fast_latest.csv', ...
    'transition_trim_mainbridge_bluecircle_fast_latest.csv', ...
    'transition_trim_mainbridge_midbasin_fast_latest.csv', ...
    'transition_trim_mainbridge_bluecircle_dense_latest.csv', ...
    'transition_trim_mainbridge_downstream_fast_latest.csv', ...
    'transition_trim_mainbridge_middlegap_fast_latest.csv', ...
    'transition_trim_mainbridge_v40_basinscan_fast_latest.csv', ...
    'transition_trim_mainbridge_v42p5_basinscan_fast_latest.csv', ...
    'transition_trim_mainbridge_v42p5_tilt70_fast_latest.csv', ...
    'transition_trim_mainbridge_v45_basinscan_fast_latest.csv', ...
    'transition_trim_mainbridge_v45_tight_fast_latest.csv', ...
    'transition_trim_mainbridge_v47p5_basinscan_fast_latest.csv', ...
    'transition_trim_mainbridge_highbridge_fast_latest.csv', ...
    'transition_trim_path_scored_latest.csv', ...
    'transition_trim_path_scored_branch30v20_latest.csv', ...
    'transition_trim_path_scored_branch30v25_latest.csv', ...
    'transition_trim_rearon_overnight_scored_latest.csv', ...
    'transition_trim_rearon_forever_scored_latest.csv', ...
    'transition_trim_rearon_connector_forever_latest.csv', ...
    'rear_on_cruise_anchor_latest.csv'};

indexFiles = { ...
    fullfile(databaseDir, 'transition_trim_linearization_index.csv'), ...
    fullfile(databaseDir, 'transition_trim_linearization_backfill_index.csv'), ...
    fullfile(databaseDir, 'transition_trim_rearon_connector_forever_linearization_index.csv'), ...
    'transition_trim_rearon_connector_forever_smoke_linearization_index.csv', ...
    'transition_trim_rearon_connector_forever_smoke2_linearization_index.csv', ...
    'transition_trim_rearon_connector_forever_smoke3_linearization_index.csv'};

manifest = struct();
manifest.trim_sources = repmat(localSourceManifestTemplate(), 0, 1);
manifest.linearization_index_sources = repmat(localSourceManifestTemplate(), 0, 1);
manifest.excluded_by_default = [ ...
    "rear_on_cruise_anchor_smoke_latest.*"; ...
    "rear_on_cruise_anchor_smoke2_latest.*"; ...
    "rear_on_cruise_anchor_smoke3_latest.*"; ...
    "transition_trim_rearon_connector_forever_smoke*_latest.*"; ...
    "transition_trim_reference_line_scored_smoketest*_latest.*"; ...
    "*_visuals_*"; ...
    "*.png"; ...
    "*_launcher.log"; ...
    "*_live.log"; ...
    "*_nohup.out"; ...
    "transition_trim_global_attempt_db_latest.*"; ...
    "trim_map_linearizations_latest.*"];

for i = 1:numel(trimFiles)
    item = localSourceManifestTemplate();
    [item.filename, item.path] = localResolveSourcePath(trimFiles{i}, workspacePlotsDir);
    item.enabled = true;
    item.is_smoke = contains(lower(item.filename), "smoke");
    item.reason = "explicit trim-result source";
    item.run_prefix = localRunPrefixFromFilename(item.filename);
    manifest.trim_sources(end + 1, 1) = item; %#ok<SAGROW>
end

for i = 1:numel(opts.extra_trim_sources)
    item = localSourceManifestTemplate();
    [item.filename, item.path] = localResolveSourcePath(opts.extra_trim_sources{i}, workspacePlotsDir);
    item.enabled = true;
    item.is_smoke = contains(lower(item.filename), "smoke");
    item.reason = "explicit extra trim-result source";
    item.run_prefix = localRunPrefixFromFilename(item.filename);
    manifest.trim_sources(end + 1, 1) = item; %#ok<SAGROW>
end

for i = 1:numel(indexFiles)
    item = localSourceManifestTemplate();
    [item.filename, item.path] = localResolveSourcePath(indexFiles{i}, workspacePlotsDir);
    item.is_smoke = contains(lower(item.filename), "smoke");
    item.enabled = ~item.is_smoke || opts.include_smoke_sources;
    if item.enabled
        item.reason = "explicit linearization index source";
    else
        item.reason = "smoke linearization index excluded by default";
    end
    item.run_prefix = localRunPrefixFromFilename(item.filename);
    manifest.linearization_index_sources(end + 1, 1) = item; %#ok<SAGROW>
end

for i = 1:numel(opts.extra_linearization_index_sources)
    item = localSourceManifestTemplate();
    [item.filename, item.path] = localResolveSourcePath(opts.extra_linearization_index_sources{i}, workspacePlotsDir);
    item.is_smoke = contains(lower(item.filename), "smoke");
    item.enabled = true;
    item.reason = "explicit extra linearization index source";
    item.run_prefix = localRunPrefixFromFilename(item.filename);
    manifest.linearization_index_sources(end + 1, 1) = item; %#ok<SAGROW>
end

rootNames = { ...
    'transition_trim_linearizations', ...
    'transition_trim_linearization_backfill_linearizations', ...
    'transition_trim_rearon_connector_forever_linearizations', ...
    'transition_trim_rearon_connector_forever_smoke_linearizations', ...
    'transition_trim_rearon_connector_forever_smoke2_linearizations', ...
    'transition_trim_rearon_connector_forever_smoke3_linearizations'};
allowedRoots = strings(0, 1);
for i = 1:numel(rootNames)
    rootName = string(rootNames{i});
    isSmoke = contains(lower(rootName), "smoke");
    if isSmoke && ~opts.include_smoke_sources
        continue;
    end
    allowedRoots(end + 1, 1) = string(fullfile(workspacePlotsDir, rootName)); %#ok<SAGROW>
    allowedRoots(end + 1, 1) = string(fullfile(databaseDir, rootName)); %#ok<SAGROW>
end
for i = 1:numel(opts.extra_linearization_roots)
    allowedRoots(end + 1, 1) = string(opts.extra_linearization_roots{i}); %#ok<SAGROW>
end
manifest.trim_sources = localUniqueManifestEntries(manifest.trim_sources);
manifest.linearization_index_sources = localUniqueManifestEntries(manifest.linearization_index_sources);
allowedRoots = unique(allowedRoots, 'stable');
manifest.allowed_linearization_roots = allowedRoots;
end

function item = localSourceManifestTemplate()
item = struct( ...
    'filename', "", ...
    'path', "", ...
    'enabled', false, ...
    'is_smoke', false, ...
    'reason', "", ...
    'run_prefix', "");
end

function runPrefix = localRunPrefixFromFilename(filename)
stem = erase(string(filename), ".csv");
runPrefix = regexprep(stem, '_latest$', '');
end

function [filename, fullPath] = localResolveSourcePath(pathOrName, workspacePlotsDir)
pathOrName = string(pathOrName);
if isfolder(pathOrName) || isfile(pathOrName) || contains(pathOrName, filesep)
    fullPath = pathOrName;
    [~, filename, ext] = fileparts(pathOrName);
    filename = filename + string(ext);
else
    filename = pathOrName;
    fullPath = string(fullfile(workspacePlotsDir, pathOrName));
end
end

function uniqueEntries = localUniqueManifestEntries(entries)
if isempty(entries)
    uniqueEntries = entries;
    return;
end
paths = string({entries.path}).';
[~, keepIdx] = unique(paths, 'stable');
uniqueEntries = entries(sort(keepIdx));
end

function [allRows, sourceStatus] = localReadAndNormalizeTrimSources(trimSources)
rows = repmat(localEmptyMasterRow(), 0, 1);
sourceStatus = repmat(localSourceStatusTemplate(), 0, 1);

for iSource = 1:numel(trimSources)
    src = trimSources(iSource);
    status = localSourceStatusTemplate();
    status.filename = src.filename;
    status.path = src.path;
    status.enabled = src.enabled;
    status.reason = src.reason;

    if ~src.enabled
        status.status = "excluded";
        sourceStatus(end + 1, 1) = status; %#ok<SAGROW>
        continue;
    end
    if exist(src.path, 'file') ~= 2
        status.status = "missing";
        sourceStatus(end + 1, 1) = status; %#ok<SAGROW>
        continue;
    end

    rawTable = readtable(src.path, 'TextType', 'string', 'VariableNamingRule', 'preserve');
    sourceKind = localDetectSourceKind(rawTable, src.filename);
    rawRows = table2struct(rawTable);

    for iRow = 1:numel(rawRows)
        rows(end + 1, 1) = localNormalizeTrimRow(rawRows(iRow), src, sourceKind, iRow); %#ok<SAGROW>
    end

    status.status = "loaded";
    status.row_count = height(rawTable);
    status.source_kind = string(sourceKind);
    sourceStatus(end + 1, 1) = status; %#ok<SAGROW>
end

allRows = struct2table(rows);
if isempty(allRows)
    allRows = localEmptyMasterTable();
else
    allRows = sortrows(allRows, {'source_file', 'tilt_deg', 'vinf_mps', 'family', 'name'});
end
end

function status = localSourceStatusTemplate()
status = struct( ...
    'filename', "", ...
    'path', "", ...
    'enabled', false, ...
    'status', "", ...
    'row_count', 0, ...
    'source_kind', "", ...
    'reason', "");
end

function sourceKind = localDetectSourceKind(rawTable, filename)
filename = string(filename);
varNames = string(rawTable.Properties.VariableNames);
hasKey = any(strcmp(varNames, 'key'));

if filename == "rear_on_cruise_anchor_latest.csv"
    sourceKind = "rear_on_cruise_anchor";
elseif any(filename == ["transition_trim_map_latest.csv", "transition_trim_map_low_speed_latest.csv", "transition_trim_map_merged_latest.csv"])
    sourceKind = "legacy_map_summary";
elseif hasKey
    sourceKind = "scored_fast_keyed";
else
    sourceKind = "scored_fast";
end
end

function row = localNormalizeTrimRow(raw, src, sourceKind, rowIndex)
row = localEmptyMasterRow();
row.source_file = src.filename;
row.source_kind = string(sourceKind);
row.source_run_prefix = src.run_prefix;
row.name = localStringField(raw, {'name'});
row.family = localStringField(raw, {'family'});
row.seed_name = localStringField(raw, {'seed_name'});
row.phase = localStringField(raw, {'phase'});
row.tilt_deg = localNumericField(raw, {'tilt_deg', 'target_tilt_deg'});
row.vinf_mps = localNumericField(raw, {'vinf_mps', 'target_vinf_mps'});
row.rear_fixed_rpm = localNumericField(raw, {'rear_fixed_rpm'});
row.success = localLogicalFieldOrDefault(raw, {'success'}, false);

if localHasAnyField(raw, {'acceptable'})
    row.acceptable = localLogicalFieldOrDefault(raw, {'acceptable'}, row.success);
else
    row.acceptable = row.success;
end

classification = localStringField(raw, {'classification'});
if strlength(classification) == 0
    if row.success
        classification = "exact_trim";
    elseif row.acceptable
        classification = "quasi_trim_usable";
    else
        classification = "not_usable";
    end
end
row.classification = classification;

row.score = localNumericField(raw, {'score'});
row.max_normalized = localNumericField(raw, {'max_normalized'});
row.worst_component = localStringField(raw, {'worst_component'});
row.worst_component_normalized = localNumericField(raw, {'worst_component_normalized'});
row.front_collective_rpm = localNumericField(raw, {'front_collective_rpm'});
row.rear_collective_rpm = localNumericField(raw, {'rear_collective_rpm'});
row.delta_f_deg = localNumericField(raw, {'delta_f_deg'});
row.delta_a_deg = localNumericField(raw, {'delta_a_deg'});
row.delta_e_deg = localNumericField(raw, {'delta_e_deg'});
row.delta_r_deg = localNumericField(raw, {'delta_r_deg'});
row.theta_deg = localNumericField(raw, {'theta_deg'});
row.u_mps = localNumericField(raw, {'u_mps'});
row.w_mps = localNumericField(raw, {'w_mps'});
row.alpha_deg = localNumericField(raw, {'alpha_deg'});
row.termination_string = localStringField(raw, {'termination_string', 'termination'});
row.max_state_residual = localNumericField(raw, {'max_state_residual'});
row.attempt_count = localNumericField(raw, {'attempt_count'});
row.rear_on_ok = isfinite(row.rear_collective_rpm) && row.rear_collective_rpm > 1;
row.key = localResolveRowKey(raw, src, rowIndex, row);
end

function row = localEmptyMasterRow()
row = struct( ...
    'source_file', "", ...
    'source_kind', "", ...
    'source_run_prefix', "", ...
    'name', "", ...
    'key', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'family', "", ...
    'seed_name', "", ...
    'phase', "", ...
    'rear_fixed_rpm', NaN, ...
    'success', false, ...
    'acceptable', false, ...
    'classification', "", ...
    'score', NaN, ...
    'max_normalized', NaN, ...
    'worst_component', "", ...
    'worst_component_normalized', NaN, ...
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
    'termination_string', "", ...
    'max_state_residual', NaN, ...
    'attempt_count', NaN, ...
    'rear_on_ok', false, ...
    'linearization_available', false, ...
    'linearization_index_source', "", ...
    'linearization_latest_file', "");
end

function tbl = localEmptyMasterTable()
row = localEmptyMasterRow();
tbl = struct2table(row);
tbl(1, :) = [];
end

function tbl = localEnsureMasterSchema(tbl)
if isempty(tbl)
    tbl = localEmptyMasterTable();
    return;
end

required = fieldnames(localEmptyMasterRow());
for i = 1:numel(required)
    name = required{i};
    if ismember(name, tbl.Properties.VariableNames)
        continue;
    end
    defaultRow = localEmptyMasterRow();
    tbl.(name) = repmat(defaultRow.(name), height(tbl), 1);
end
tbl = tbl(:, required);
end

function key = localResolveRowKey(raw, src, rowIndex, normalizedRow)
nativeKey = localStringField(raw, {'key'});
if strlength(nativeKey) > 0
    key = nativeKey;
    return;
end

familyLabel = localSanitizeToken(normalizedRow.family);
sourceLabel = localSanitizeToken(src.run_prefix);
seedLabel = localSanitizeToken(normalizedRow.seed_name);

if isfinite(normalizedRow.tilt_deg) && isfinite(normalizedRow.vinf_mps)
    key = "tilt_" + localNumberToken(normalizedRow.tilt_deg) + ...
        "__vinf_" + localNumberToken(normalizedRow.vinf_mps) + ...
        "__family_" + familyLabel + ...
        "__source_" + sourceLabel;
elseif strlength(normalizedRow.name) > 0
    key = sourceLabel + "__name_" + localSanitizeToken(normalizedRow.name);
else
    key = sourceLabel + "__row_" + string(rowIndex);
end

if strlength(seedLabel) > 0
    key = key + "__seed_" + seedLabel;
end
end

function [indexTable, sourceStatus] = localReadLinearizationIndices(indexSources, allowedRoots)
rows = repmat(localEmptyLinearizationIndexRow(), 0, 1);
sourceStatus = repmat(localSourceStatusTemplate(), 0, 1);

for iSource = 1:numel(indexSources)
    src = indexSources(iSource);
    status = localSourceStatusTemplate();
    status.filename = src.filename;
    status.path = src.path;
    status.enabled = src.enabled;
    status.reason = src.reason;

    if ~src.enabled
        status.status = "excluded";
        sourceStatus(end + 1, 1) = status; %#ok<SAGROW>
        continue;
    end
    if exist(src.path, 'file') ~= 2
        status.status = "missing";
        sourceStatus(end + 1, 1) = status; %#ok<SAGROW>
        continue;
    end

    rawTable = readtable(src.path, 'TextType', 'string', 'VariableNamingRule', 'preserve');
    rawRows = table2struct(rawTable);
    for iRow = 1:numel(rawRows)
        rows(end + 1, 1) = localNormalizeLinearizationIndexRow(rawRows(iRow), src, allowedRoots); %#ok<SAGROW>
    end

    status.status = "loaded";
    status.row_count = height(rawTable);
    status.source_kind = "linearization_index";
    sourceStatus(end + 1, 1) = status; %#ok<SAGROW>
end

indexTable = struct2table(rows);
if isempty(indexTable)
    indexTable = localEmptyLinearizationIndexTable();
else
    indexTable = sortrows(indexTable, {'resolved_exists', 'index_priority', 'tilt_deg', 'vinf_mps'}, {'descend', 'ascend', 'ascend', 'ascend'});
end
end

function row = localEmptyLinearizationIndexRow()
row = struct( ...
    'index_source_file', "", ...
    'index_source_kind', "", ...
    'index_source_is_smoke', false, ...
    'index_priority', inf, ...
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
    'latest_file', "", ...
    'resolved_file', "", ...
    'resolved_exists', false);
end

function tbl = localEmptyLinearizationIndexTable()
row = localEmptyLinearizationIndexRow();
tbl = struct2table(row);
tbl(1, :) = [];
end

function row = localNormalizeLinearizationIndexRow(raw, src, allowedRoots)
row = localEmptyLinearizationIndexRow();
row.index_source_file = src.filename;
row.index_source_kind = "linearization_index";
row.index_source_is_smoke = src.is_smoke;
row.index_priority = double(1 + src.is_smoke);
row.key = localStringField(raw, {'key'});
row.name = localStringField(raw, {'name'});
row.tilt_deg = localNumericField(raw, {'tilt_deg'});
row.vinf_mps = localNumericField(raw, {'vinf_mps'});
row.family = localStringField(raw, {'family'});
row.seed_name = localStringField(raw, {'seed_name'});
row.success = localLogicalFieldOrDefault(raw, {'success'}, false);
row.acceptable = localLogicalFieldOrDefault(raw, {'acceptable'}, row.success);
row.classification = localStringField(raw, {'classification'});
row.score = localNumericField(raw, {'score'});
row.max_normalized = localNumericField(raw, {'max_normalized'});
row.front_collective_rpm = localNumericField(raw, {'front_collective_rpm'});
row.rear_collective_rpm = localNumericField(raw, {'rear_collective_rpm'});
row.delta_f_deg = localNumericField(raw, {'delta_f_deg'});
row.delta_a_deg = localNumericField(raw, {'delta_a_deg'});
row.delta_e_deg = localNumericField(raw, {'delta_e_deg'});
row.delta_r_deg = localNumericField(raw, {'delta_r_deg'});
row.theta_deg = localNumericField(raw, {'theta_deg'});
row.alpha_deg = localNumericField(raw, {'alpha_deg'});
row.output_file = localStringField(raw, {'output_file'});
row.latest_file = localStringField(raw, {'latest_file'});

chosenPath = localResolveAllowedLinearizationPath(row.latest_file, allowedRoots);
if strlength(chosenPath) == 0
    chosenPath = localResolveAllowedLinearizationPath(row.output_file, allowedRoots);
end

row.resolved_file = chosenPath;
row.resolved_exists = strlength(chosenPath) > 0 && exist(chosenPath, 'file') == 2;
end

function resolvedPath = localResolveAllowedLinearizationPath(pathValue, allowedRoots)
resolvedPath = "";
pathValue = string(pathValue);
if strlength(pathValue) == 0
    return;
end

if localPathIsAllowed(pathValue, allowedRoots) && exist(pathValue, 'file') == 2
    resolvedPath = pathValue;
    return;
end

[~, name, ext] = fileparts(pathValue);
if strlength(string(name)) == 0
    return;
end
filename = string(name) + string(ext);

for i = 1:numel(allowedRoots)
    rootPath = string(allowedRoots(i));
    if strlength(rootPath) == 0
        continue;
    end
    candidatePath = string(fullfile(rootPath, filename));
    if exist(candidatePath, 'file') == 2
        resolvedPath = candidatePath;
        return;
    end
end

if localPathIsAllowed(pathValue, allowedRoots)
    resolvedPath = pathValue;
end
end

function tf = localPathIsAllowed(pathValue, allowedRoots)
tf = false;
pathValue = string(pathValue);
if strlength(pathValue) == 0
    return;
end
for i = 1:numel(allowedRoots)
    rootPath = string(allowedRoots(i));
    if strlength(rootPath) == 0
        continue;
    end
    if startsWith(pathValue, rootPath)
        tf = true;
        return;
    end
end
end

function [masterRows, summary] = localAttachLinearizationMatches(masterRows, indexTable, opts)
summary = struct();
summary.total_rows = height(masterRows);
summary.rows_with_match = 0;
summary.rows_with_existing_file = 0;
summary.rows_without_match = 0;

if isempty(masterRows)
    return;
end

for i = 1:height(masterRows)
    row = masterRows(i, :);
    existingFile = string(row.linearization_latest_file);
    existingHasFile = strlength(existingFile) > 0 && exist(existingFile, 'file') == 2;
    masterRows.linearization_available(i) = false;
    masterRows.linearization_index_source(i) = "";
    masterRows.linearization_latest_file(i) = "";
    if existingHasFile
        masterRows.linearization_available(i) = true;
        masterRows.linearization_latest_file(i) = existingFile;
        summary.rows_with_match = summary.rows_with_match + 1;
        summary.rows_with_existing_file = summary.rows_with_existing_file + 1;
        continue;
    end

    if isempty(indexTable)
        summary.rows_without_match = summary.rows_without_match + 1;
        continue;
    end

    match = localFindLinearizationMatch(row, indexTable, opts);
    if ~match.found
        summary.rows_without_match = summary.rows_without_match + 1;
        continue;
    end

    masterRows.linearization_index_source(i) = match.index_source_file;
    masterRows.linearization_latest_file(i) = match.resolved_file;
    masterRows.linearization_available(i) = match.resolved_exists;
    summary.rows_with_match = summary.rows_with_match + 1;
    if match.resolved_exists
        summary.rows_with_existing_file = summary.rows_with_existing_file + 1;
    end
end
end

function match = localFindLinearizationMatch(row, indexTable, opts)
match = struct('found', false, 'index_source_file', "", 'resolved_file', "", 'resolved_exists', false);
candidateTable = indexTable(1:0, :);

usePrimaryKey = strcmp(string(row.source_kind), "scored_fast_keyed") && strlength(string(row.key)) > 0;
if usePrimaryKey
    candidateMask = indexTable.key == string(row.key);
    candidateTable = indexTable(candidateMask, :);
end

if isempty(candidateTable)
    if ~(isfinite(row.tilt_deg) && isfinite(row.vinf_mps) && strlength(string(row.family)) > 0)
        return;
    end

    candidateMask = abs(indexTable.tilt_deg - row.tilt_deg) <= opts.tilt_match_tolerance & ...
        abs(indexTable.vinf_mps - row.vinf_mps) <= opts.vinf_match_tolerance & ...
        strcmpi(indexTable.family, string(row.family));

    if isfinite(row.front_collective_rpm)
        candidateMask = candidateMask & ...
            abs(indexTable.front_collective_rpm - row.front_collective_rpm) <= opts.rpm_match_tolerance;
    end
    if isfinite(row.rear_collective_rpm)
        candidateMask = candidateMask & ...
            abs(indexTable.rear_collective_rpm - row.rear_collective_rpm) <= opts.rpm_match_tolerance;
    end
    candidateTable = indexTable(candidateMask, :);
end

if isempty(candidateTable)
    return;
end

bestIdx = localSelectBestLinearizationCandidate(candidateTable, row);
best = candidateTable(bestIdx, :);

match.found = true;
match.index_source_file = string(best.index_source_file);
match.resolved_file = string(best.resolved_file);
match.resolved_exists = logical(best.resolved_exists);
end

function bestIdx = localSelectBestLinearizationCandidate(candidateTable, row)
candidateCount = height(candidateTable);
rank = zeros(candidateCount, 5);
for i = 1:candidateCount
    candidate = candidateTable(i, :);
    rank(i, 1) = ~logical(candidate.resolved_exists);
    rank(i, 2) = double(candidate.index_priority);
    rank(i, 3) = localClassificationRank(candidate.classification, candidate.acceptable, candidate.success);
    rank(i, 4) = localFiniteOrInf(candidate.max_normalized);
    rank(i, 5) = localFiniteDifference(candidate.front_collective_rpm, row.front_collective_rpm) + ...
        localFiniteDifference(candidate.rear_collective_rpm, row.rear_collective_rpm);
end
rankTable = array2table(rank, 'VariableNames', {'missing_file', 'index_priority', 'class_rank', 'max_norm_rank', 'rpm_distance'});
rankTable.original_index = (1:candidateCount).';
rankTable = sortrows(rankTable, {'missing_file', 'index_priority', 'class_rank', 'max_norm_rank', 'rpm_distance', 'original_index'});
bestIdx = rankTable.original_index(1);
end

function bestUnique = localBuildBestUniquePoints(masterRows)
if isempty(masterRows)
    bestUnique = masterRows;
    return;
end

groupKeys = strings(height(masterRows), 1);
classRank = zeros(height(masterRows), 1);
scoreRank = zeros(height(masterRows), 1);
maxNormRank = zeros(height(masterRows), 1);

for i = 1:height(masterRows)
    row = masterRows(i, :);
    groupKeys(i) = localUniquePointKey(row);
    classRank(i) = localClassificationRank(row.classification, row.acceptable, row.success);
    scoreRank(i) = localFiniteOrInf(row.score);
    maxNormRank(i) = localFiniteOrInf(row.max_normalized);
end

work = masterRows;
work.group_key = groupKeys;
work.class_rank = classRank;
work.score_rank = scoreRank;
work.max_norm_rank = maxNormRank;
work.original_index = (1:height(masterRows)).';
work = sortrows(work, {'group_key', 'class_rank', 'score_rank', 'max_norm_rank', 'original_index'});

[~, keepIdx] = unique(work.group_key, 'stable');
bestUnique = work(sort(keepIdx), :);
bestUnique.group_key = [];
bestUnique.class_rank = [];
bestUnique.score_rank = [];
bestUnique.max_norm_rank = [];
bestUnique.original_index = [];
bestUnique = sortrows(bestUnique, {'tilt_deg', 'vinf_mps', 'family', 'rear_fixed_rpm'});
end

function key = localUniquePointKey(row)
rearFixedToken = "free";
if isfinite(row.rear_fixed_rpm)
    rearFixedToken = "rearfixed_" + localNumberToken(row.rear_fixed_rpm);
end
key = "tilt_" + localNumberToken(row.tilt_deg) + ...
    "__vinf_" + localNumberToken(row.vinf_mps) + ...
    "__family_" + localSanitizeToken(row.family) + ...
    "__" + rearFixedToken;
end

function [summaryTable, points, missingTable, buildSummary] = localBuildControllerSchedule(masterBestUnique, opts)
candidateMask = masterBestUnique.rear_on_ok;
if opts.controller_exclude_zero_rear
    candidateMask = candidateMask & masterBestUnique.rear_collective_rpm > 1;
end

if opts.controller_include_quasi_trims
    candidateMask = candidateMask & (strcmp(masterBestUnique.classification, "exact_trim") | masterBestUnique.acceptable);
else
    candidateMask = candidateMask & strcmp(masterBestUnique.classification, "exact_trim");
end

candidateTable = masterBestUnique(candidateMask, :);
candidateTable = sortrows(candidateTable, {'tilt_deg', 'vinf_mps', 'family'});

points = repmat(localEmptyControllerPoint(), 0, 1);
missingRows = repmat(localEmptyMissingLinearizationRow(), 0, 1);

for i = 1:height(candidateTable)
    row = candidateTable(i, :);
    if ~row.linearization_available || strlength(string(row.linearization_latest_file)) == 0
        missingRows(end + 1, 1) = localBuildMissingRow(row, "no matched linearization file"); %#ok<SAGROW>
        if opts.controller_allow_trim_only_candidates
            points(end + 1, 1) = localBuildTrimOnlyControllerPoint(row); %#ok<SAGROW>
        end
        continue;
    end

    [point, loadReason] = localLoadControllerPointFromLinearization(row, opts);
    if strlength(loadReason) > 0
        missingRows(end + 1, 1) = localBuildMissingRow(row, loadReason); %#ok<SAGROW>
        if opts.controller_allow_trim_only_candidates
            points(end + 1, 1) = localBuildTrimOnlyControllerPoint(row); %#ok<SAGROW>
        end
        continue;
    end

    points(end + 1, 1) = point; %#ok<SAGROW>
end

if isempty(points)
    summaryTable = localEmptyControllerSummaryTable();
else
    [summaryTable, points] = localSortControllerSchedulePoints(points);
end
missingTable = struct2table(missingRows);
if isempty(missingTable)
    missingTable = localEmptyMissingLinearizationTable();
else
    missingTable = sortrows(missingTable, {'tilt_deg', 'vinf_mps', 'family'});
end

buildSummary = struct();
buildSummary.candidate_rows = height(candidateTable);
buildSummary.kept_rows = height(summaryTable);
buildSummary.missing_linearization_rows = height(missingTable);
buildSummary.allow_trim_only_candidates = opts.controller_allow_trim_only_candidates;
buildSummary.include_quasi_trims = opts.controller_include_quasi_trims;
end

function point = localEmptyControllerPoint()
point = struct( ...
    'key', "", ...
    'name', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'family', "", ...
    'score', NaN, ...
    'classification', "", ...
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
    'linearization_available', false, ...
    'linearization_latest_file', "", ...
    'Att_Trim_deg', zeros(3, 1), ...
    'Vel_B_BA_Trim', zeros(3, 1), ...
    'trimCase', struct(), ...
    'x0', zeros(9, 1), ...
    'u0', zeros(6, 1), ...
    'trim_cmd_rad', zeros(6, 1), ...
    'A_9', zeros(9, 9), ...
    'B_9', zeros(9, 6), ...
    'C_9', eye(9), ...
    'D_9', zeros(9, 6), ...
    'state_names_9', strings(9, 1), ...
    'input_names_9', strings(6, 1), ...
    'output_names_9', strings(9, 1), ...
    'A_full', [], ...
    'B_full', [], ...
    'C_full', [], ...
    'D_full', [], ...
    'full_state_names', strings(0, 1), ...
    'full_input_names', strings(0, 1), ...
    'full_output_names', strings(0, 1));
end

function point = localBuildTrimOnlyControllerPoint(row)
point = localEmptyControllerPoint();
point.key = row.key;
point.name = row.name;
point.tilt_deg = row.tilt_deg;
point.vinf_mps = row.vinf_mps;
point.family = row.family;
point.score = row.score;
point.classification = row.classification;
point.front_collective_rpm = row.front_collective_rpm;
point.rear_collective_rpm = row.rear_collective_rpm;
point.delta_f_deg = row.delta_f_deg;
point.delta_a_deg = row.delta_a_deg;
point.delta_e_deg = row.delta_e_deg;
point.delta_r_deg = row.delta_r_deg;
point.theta_deg = row.theta_deg;
point.u_mps = row.u_mps;
point.w_mps = row.w_mps;
point.alpha_deg = row.alpha_deg;
point.linearization_available = false;
point.linearization_latest_file = row.linearization_latest_file;
point.Att_Trim_deg = [0; localValueOrZero(row.theta_deg); 0];
point.Vel_B_BA_Trim = [localValueOrZero(row.u_mps); 0; localValueOrZero(row.w_mps)];
point.x0 = [0; deg2rad(localValueOrZero(row.theta_deg)); 0; localValueOrZero(row.u_mps); 0; localValueOrZero(row.w_mps); 0; 0; 0];
point.u0 = [localValueOrZero(row.front_collective_rpm); localValueOrZero(row.rear_collective_rpm); ...
    localValueOrZero(row.delta_f_deg); localValueOrZero(row.delta_a_deg); localValueOrZero(row.delta_e_deg); localValueOrZero(row.delta_r_deg)];
point.trim_cmd_rad = [localValueOrZero(row.front_collective_rpm); localValueOrZero(row.rear_collective_rpm); ...
    deg2rad(localValueOrZero(row.delta_f_deg)); deg2rad(localValueOrZero(row.delta_a_deg)); ...
    deg2rad(localValueOrZero(row.delta_e_deg)); deg2rad(localValueOrZero(row.delta_r_deg))];
point.state_names_9 = ["phi"; "theta"; "psi"; "u"; "v"; "w"; "P"; "Q"; "R"];
point.input_names_9 = ["front_coll_rpm"; "rear_coll_rpm"; "delta_f"; "delta_a"; "delta_e"; "delta_r"];
point.output_names_9 = point.state_names_9;
end

function [point, reason] = localLoadControllerPointFromLinearization(row, opts)
point = localEmptyControllerPoint();
reason = "";

try
    data = load(row.linearization_latest_file, 'linearizationPoint');
catch err
    reason = "load failed: " + string(err.message);
    return;
end

if ~isfield(data, 'linearizationPoint') || ~isstruct(data.linearizationPoint)
    reason = "linearizationPoint struct missing";
    return;
end

lp = data.linearizationPoint;
if ~isfield(lp, 'linear') || ~isstruct(lp.linear)
    reason = "linear field missing";
    return;
end

requiredReducedFields = {'A_9', 'B_9', 'C_9', 'D_9', 'state_names_9', 'input_names_9', 'output_names_9'};
for i = 1:numel(requiredReducedFields)
    if ~isfield(lp.linear, requiredReducedFields{i})
        reason = "missing reduced linearization field: " + string(requiredReducedFields{i});
        return;
    end
end

attDeg = localResolveAttTrimDeg(lp, row);
velBody = localResolveVelBodyTrim(lp, row);
rates = localResolveRatesTrim(lp);

point.key = row.key;
point.name = row.name;
point.tilt_deg = row.tilt_deg;
point.vinf_mps = row.vinf_mps;
point.family = row.family;
point.score = row.score;
point.classification = row.classification;
point.front_collective_rpm = row.front_collective_rpm;
point.rear_collective_rpm = row.rear_collective_rpm;
point.delta_f_deg = row.delta_f_deg;
point.delta_a_deg = row.delta_a_deg;
point.delta_e_deg = row.delta_e_deg;
point.delta_r_deg = row.delta_r_deg;
point.theta_deg = row.theta_deg;
point.u_mps = row.u_mps;
point.w_mps = row.w_mps;
point.alpha_deg = row.alpha_deg;
point.linearization_available = true;
point.linearization_latest_file = row.linearization_latest_file;
point.Att_Trim_deg = attDeg;
point.Vel_B_BA_Trim = velBody;
point.trimCase = localGetStructField(lp, 'trimCase', struct());
point.x0 = [deg2rad(attDeg(:)); velBody(:); rates(:)];
point.u0 = [row.front_collective_rpm; row.rear_collective_rpm; ...
    row.delta_f_deg; row.delta_a_deg; row.delta_e_deg; row.delta_r_deg];
point.trim_cmd_rad = [row.front_collective_rpm; row.rear_collective_rpm; ...
    deg2rad(row.delta_f_deg); deg2rad(row.delta_a_deg); deg2rad(row.delta_e_deg); deg2rad(row.delta_r_deg)];
point.A_9 = lp.linear.A_9;
point.B_9 = lp.linear.B_9;
point.C_9 = lp.linear.C_9;
point.D_9 = lp.linear.D_9;
point.state_names_9 = string(lp.linear.state_names_9(:));
point.input_names_9 = string(lp.linear.input_names_9(:));
point.output_names_9 = string(lp.linear.output_names_9(:));

if opts.include_full_order_linearization
    point.A_full = localGetStructField(lp.linear, 'A_full', []);
    point.B_full = localGetStructField(lp.linear, 'B_full', []);
    point.C_full = localGetStructField(lp.linear, 'C_full', []);
    point.D_full = localGetStructField(lp.linear, 'D_full', []);
    point.full_state_names = string(localGetStructField(lp.linear, 'full_state_names', strings(0, 1)));
    point.full_input_names = string(localGetStructField(lp.linear, 'full_input_names', strings(0, 1)));
    point.full_output_names = string(localGetStructField(lp.linear, 'full_output_names', strings(0, 1)));
end
end

function attDeg = localResolveAttTrimDeg(lp, row)
attDeg = [0; localValueOrZero(row.theta_deg); 0];
if isfield(lp, 'Att_Trim_deg') && ~isempty(lp.Att_Trim_deg)
    value = double(lp.Att_Trim_deg(:));
    if numel(value) >= 3
        attDeg = value(1:3);
    end
end
end

function velBody = localResolveVelBodyTrim(lp, row)
velBody = [localValueOrZero(row.u_mps); 0; localValueOrZero(row.w_mps)];
if isfield(lp, 'Vel_B_BA_Trim') && ~isempty(lp.Vel_B_BA_Trim)
    value = double(lp.Vel_B_BA_Trim(:));
    if numel(value) >= 3
        velBody = value(1:3);
    end
end
end

function rates = localResolveRatesTrim(lp)
rates = zeros(3, 1);
if isfield(lp, 'Rates_Trim') && ~isempty(lp.Rates_Trim)
    value = double(lp.Rates_Trim(:));
    if numel(value) >= 3
        rates = value(1:3);
    end
end
end

function row = localEmptyMissingLinearizationRow()
row = struct( ...
    'key', "", ...
    'name', "", ...
    'source_file', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'family', "", ...
    'classification', "", ...
    'rear_collective_rpm', NaN, ...
    'linearization_index_source', "", ...
    'linearization_latest_file', "", ...
    'reason', "");
end

function tbl = localEmptyMissingLinearizationTable()
row = localEmptyMissingLinearizationRow();
tbl = struct2table(row);
tbl(1, :) = [];
end

function row = localBuildMissingRow(controllerRow, reason)
row = localEmptyMissingLinearizationRow();
row.key = controllerRow.key;
row.name = controllerRow.name;
row.source_file = controllerRow.source_file;
row.tilt_deg = controllerRow.tilt_deg;
row.vinf_mps = controllerRow.vinf_mps;
row.family = controllerRow.family;
row.classification = controllerRow.classification;
row.rear_collective_rpm = controllerRow.rear_collective_rpm;
row.linearization_index_source = controllerRow.linearization_index_source;
row.linearization_latest_file = controllerRow.linearization_latest_file;
row.reason = string(reason);
end

function summaryTable = localControllerSummaryTable(pointTable)
keepVars = { ...
    'key', 'name', 'tilt_deg', 'vinf_mps', 'family', 'score', 'classification', ...
    'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_a_deg', 'delta_e_deg', 'delta_r_deg', ...
    'theta_deg', 'u_mps', 'w_mps', 'alpha_deg', ...
    'linearization_available', 'linearization_latest_file'};
summaryTable = pointTable(:, keepVars);
end

function summaryTable = localControllerSummaryFromPointArray(points)
if isempty(points)
    summaryTable = localEmptyControllerSummaryTable();
    return;
end

points = points(:);
n = numel(points);
key = strings(n, 1);
name = strings(n, 1);
tilt_deg = nan(n, 1);
vinf_mps = nan(n, 1);
family = strings(n, 1);
score = nan(n, 1);
classification = strings(n, 1);
front_collective_rpm = nan(n, 1);
rear_collective_rpm = nan(n, 1);
delta_f_deg = nan(n, 1);
delta_a_deg = nan(n, 1);
delta_e_deg = nan(n, 1);
delta_r_deg = nan(n, 1);
theta_deg = nan(n, 1);
u_mps = nan(n, 1);
w_mps = nan(n, 1);
alpha_deg = nan(n, 1);
linearization_available = false(n, 1);
linearization_latest_file = strings(n, 1);

for i = 1:n
    point = points(i);
    key(i) = string(point.key);
    name(i) = string(point.name);
    tilt_deg(i) = point.tilt_deg;
    vinf_mps(i) = point.vinf_mps;
    family(i) = string(point.family);
    score(i) = point.score;
    classification(i) = string(point.classification);
    front_collective_rpm(i) = point.front_collective_rpm;
    rear_collective_rpm(i) = point.rear_collective_rpm;
    delta_f_deg(i) = point.delta_f_deg;
    delta_a_deg(i) = point.delta_a_deg;
    delta_e_deg(i) = point.delta_e_deg;
    delta_r_deg(i) = point.delta_r_deg;
    theta_deg(i) = point.theta_deg;
    u_mps(i) = point.u_mps;
    w_mps(i) = point.w_mps;
    alpha_deg(i) = point.alpha_deg;
    linearization_available(i) = logical(point.linearization_available);
    linearization_latest_file(i) = string(point.linearization_latest_file);
end

summaryTable = table( ...
    key, name, tilt_deg, vinf_mps, family, score, classification, ...
    front_collective_rpm, rear_collective_rpm, delta_f_deg, delta_a_deg, ...
    delta_e_deg, delta_r_deg, theta_deg, u_mps, w_mps, alpha_deg, ...
    linearization_available, linearization_latest_file);
end

function [summaryTable, points] = localSortControllerSchedulePoints(points)
if isempty(points)
    points = repmat(localEmptyControllerPoint(), 0, 1);
    summaryTable = localEmptyControllerSummaryTable();
    return;
end

points = points(:);
summaryTable = localControllerSummaryFromPointArray(points);
summaryTable.original_index = (1:height(summaryTable)).';
summaryTable = sortrows(summaryTable, {'tilt_deg', 'vinf_mps', 'family', 'key'});
sortIdx = summaryTable.original_index;
summaryTable.original_index = [];
points = points(sortIdx);
end

function tbl = localEmptyControllerSummaryTable()
tbl = table();
tbl.key = strings(0, 1);
tbl.name = strings(0, 1);
tbl.tilt_deg = zeros(0, 1);
tbl.vinf_mps = zeros(0, 1);
tbl.family = strings(0, 1);
tbl.score = zeros(0, 1);
tbl.classification = strings(0, 1);
tbl.front_collective_rpm = zeros(0, 1);
tbl.rear_collective_rpm = zeros(0, 1);
tbl.delta_f_deg = zeros(0, 1);
tbl.delta_a_deg = zeros(0, 1);
tbl.delta_e_deg = zeros(0, 1);
tbl.delta_r_deg = zeros(0, 1);
tbl.theta_deg = zeros(0, 1);
tbl.u_mps = zeros(0, 1);
tbl.w_mps = zeros(0, 1);
tbl.alpha_deg = zeros(0, 1);
tbl.linearization_available = false(0, 1);
tbl.linearization_latest_file = strings(0, 1);
end

function meta = localBuildMetaStruct(rootDir, databaseDir, workspacePlotsDir, opts)
meta = struct();
meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
meta.root_dir = rootDir;
meta.database_dir = databaseDir;
meta.workspace_plots_dir = workspacePlotsDir;
meta.options = opts;
end

function validation = localRunValidationChecks(masterRows, controllerSummaryTable, controllerPoints, missingTable, manifest, opts)
validation = struct();
validation.source_row_count = height(masterRows);
validation.controller_row_count = height(controllerSummaryTable);
validation.controller_missing_linearization_count = height(missingTable);
validation.smoke_sources_imported = false;
if ~opts.include_smoke_sources && ~isempty(masterRows)
    validation.smoke_sources_imported = any(contains(lower(masterRows.source_file), "smoke"));
end
validation.controller_has_zero_rear = false;
if ~isempty(controllerSummaryTable)
    validation.controller_has_zero_rear = any(controllerSummaryTable.rear_collective_rpm <= 1);
end
validation.controller_has_reduced_linearization = false;
if ~isempty(controllerPoints)
    for i = 1:numel(controllerPoints)
        point = controllerPoints(i);
        if ~isempty(point.A_9) && ~isempty(point.B_9)
            validation.controller_has_reduced_linearization = true;
            break;
        end
    end
end
validation.included_trim_sources = nnz([manifest.trim_sources.enabled]);
validation.included_linearization_index_sources = nnz([manifest.linearization_index_sources.enabled]);
validation.pass = ~validation.smoke_sources_imported && ...
    ~validation.controller_has_zero_rear && ...
    (isempty(controllerSummaryTable) || validation.controller_has_reduced_linearization);
end

function localWriteMasterMarkdown(filename, db)
fid = fopen(filename, 'w');
if fid < 0
    warning('TrimDB_Build:WriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

allRows = db.master_attempt_db_all_rows;
bestRows = db.master_attempt_db_best_unique_points;
fprintf(fid, '# Transition Trim Master Attempt DB\n\n');
fprintf(fid, '- Created: `%s`\n', db.meta.created_on);
fprintf(fid, '- All rows: `%d`\n', height(allRows));
fprintf(fid, '- Best unique points: `%d`\n\n', height(bestRows));

fprintf(fid, '## Rows By Source\n\n');
if isempty(allRows)
    fprintf(fid, '_No rows loaded._\n');
else
    counts = groupsummary(allRows(:, {'source_file'}), 'source_file');
    fprintf(fid, '| source_file | GroupCount |\n');
    fprintf(fid, '| --- | ---: |\n');
    for i = 1:height(counts)
        fprintf(fid, '| %s | %d |\n', counts.source_file(i), counts.GroupCount(i));
    end
    fprintf(fid, '\n');
end

fprintf(fid, '## Classification Counts\n\n');
if isempty(allRows)
    fprintf(fid, '_No rows loaded._\n');
else
    counts = groupsummary(allRows(:, {'classification'}), 'classification');
    fprintf(fid, '| classification | GroupCount |\n');
    fprintf(fid, '| --- | ---: |\n');
    for i = 1:height(counts)
        fprintf(fid, '| %s | %d |\n', counts.classification(i), counts.GroupCount(i));
    end
    fprintf(fid, '\n');
end
end

function localWriteControllerMarkdown(filename, db)
fid = fopen(filename, 'w');
if fid < 0
    warning('TrimDB_Build:WriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

summaryTable = db.summary_table;
missingTable = db.missing_linearizations;

fprintf(fid, '# Controller Schedule DB\n\n');
fprintf(fid, '- Created: `%s`\n', db.meta.created_on);
fprintf(fid, '- Controller candidates kept: `%d`\n', height(summaryTable));
fprintf(fid, '- Missing linearizations during raw rebuild: `%d`\n', height(missingTable));
if isfield(db, 'build_summary') && isstruct(db.build_summary)
    fprintf(fid, '- Rebuilt controller rows before merge: `%d`\n', ...
        localGetStructField(db.build_summary, 'rebuilt_controller_rows_before_merge', height(summaryTable)));
    fprintf(fid, '- Preserved existing inlined controller rows: `%d`\n', ...
        localGetStructField(db.build_summary, 'preserved_existing_controller_rows', 0));
end
fprintf(fid, '- Include quasi trims: `%d`\n', db.build_summary.include_quasi_trims);
fprintf(fid, '- Allow trim-only candidates: `%d`\n\n', db.build_summary.allow_trim_only_candidates);
fprintf(fid, 'The controller-facing linear models are inlined in this MAT file. External raw linearization MAT files are not required for normal controller builds.\n\n');

fprintf(fid, '## Controller Schedule Preview\n\n');
if isempty(summaryTable)
    fprintf(fid, '_No controller candidates kept._\n');
else
    fprintf(fid, '| key | tilt_deg | vinf_mps | family | classification | rear_collective_rpm |\n');
    fprintf(fid, '| --- | ---: | ---: | --- | --- | ---: |\n');
    maxRows = min(height(summaryTable), 25);
    for i = 1:maxRows
        fprintf(fid, '| %s | %.6g | %.6g | %s | %s | %.6g |\n', ...
            summaryTable.key(i), summaryTable.tilt_deg(i), summaryTable.vinf_mps(i), ...
            summaryTable.family(i), summaryTable.classification(i), summaryTable.rear_collective_rpm(i));
    end
    fprintf(fid, '\n');
end
end

function localWriteBuildLog(filename, masterDb, controllerDb)
fid = fopen(filename, 'w');
if fid < 0
    warning('TrimDB_Build:WriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '# Controller Schedule DB Build Log\n\n');
fprintf(fid, '- Created: `%s`\n', controllerDb.meta.created_on);
fprintf(fid, '- include_smoke_sources: `%d`\n', controllerDb.meta.options.include_smoke_sources);
fprintf(fid, '- controller_include_quasi_trims: `%d`\n', controllerDb.meta.options.controller_include_quasi_trims);
fprintf(fid, '- controller_allow_trim_only_candidates: `%d`\n\n', controllerDb.meta.options.controller_allow_trim_only_candidates);

fprintf(fid, '## Included Trim Sources\n\n');
trimStatus = masterDb.source_status;
localWriteSourceStatusTable(fid, trimStatus([trimStatus.enabled]));
fprintf(fid, '\n## Included Linearization Index Sources\n\n');
linStatus = masterDb.linearization_index_status;
localWriteSourceStatusTable(fid, linStatus([linStatus.enabled]));

fprintf(fid, '\n## Excluded Linearization Index Sources\n\n');
localWriteSourceStatusTable(fid, linStatus(~[linStatus.enabled]));

fprintf(fid, '\n## Default Exclusions\n\n');
for i = 1:numel(masterDb.manifest.excluded_by_default)
    fprintf(fid, '- `%s`\n', masterDb.manifest.excluded_by_default(i));
end

fprintf(fid, '\n## Summary\n\n');
fprintf(fid, '- Master all rows: `%d`\n', height(masterDb.master_attempt_db_all_rows));
fprintf(fid, '- Master best unique points: `%d`\n', height(masterDb.master_attempt_db_best_unique_points));
fprintf(fid, '- Controller candidate rows kept: `%d`\n', height(controllerDb.summary_table));
fprintf(fid, '- Controller candidates missing linearization: `%d`\n', height(controllerDb.missing_linearizations));
fprintf(fid, '- Rows with matched linearization reference: `%d`\n', masterDb.linearization_match_summary.rows_with_match);
fprintf(fid, '- Rows with existing linearization file: `%d`\n', masterDb.linearization_match_summary.rows_with_existing_file);

fprintf(fid, '\n## Validation\n\n');
fprintf(fid, '- Validation pass: `%d`\n', controllerDb.validation.pass);
fprintf(fid, '- Smoke sources imported unexpectedly: `%d`\n', controllerDb.validation.smoke_sources_imported);
fprintf(fid, '- Controller DB contains zero-rear rows: `%d`\n', controllerDb.validation.controller_has_zero_rear);
fprintf(fid, '- Controller DB has reduced linearization payload: `%d`\n', controllerDb.validation.controller_has_reduced_linearization);
end

function localWriteSourceStatusTable(fid, statusStruct)
if isempty(statusStruct)
    fprintf(fid, '_None._\n');
    return;
end
statusTable = struct2table(statusStruct);
fprintf(fid, '| filename | status | row_count | source_kind | reason |\n');
fprintf(fid, '| --- | --- | ---: | --- | --- |\n');
for i = 1:height(statusTable)
    fprintf(fid, '| %s | %s | %d | %s | %s |\n', ...
        statusTable.filename(i), statusTable.status(i), statusTable.row_count(i), ...
        statusTable.source_kind(i), statusTable.reason(i));
end
end

function rank = localClassificationRank(classification, acceptable, success)
classification = lower(string(classification));
if classification == "exact_trim" || logical(success)
    rank = 1;
elseif classification == "quasi_trim_usable" || logical(acceptable)
    rank = 2;
elseif classification == "near_trim_borderline"
    rank = 3;
else
    rank = 4;
end
end

function value = localFiniteOrInf(value)
if ~isfinite(value)
    value = inf;
end
end

function value = localValueOrZero(value)
if ~isfinite(value)
    value = 0.0;
end
end

function value = localFiniteDifference(a, b)
if ~isfinite(a) || ~isfinite(b)
    value = 0.0;
else
    value = abs(a - b);
end
end

function value = localGetStructField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end

function tf = localHasField(s, fieldName)
tf = isstruct(s) && isfield(s, fieldName);
end

function tf = localHasAnyField(s, names)
tf = false;
for i = 1:numel(names)
    if localHasField(s, names{i})
        tf = true;
        return;
    end
end
end

function value = localStringField(raw, names)
value = "";
for i = 1:numel(names)
    name = names{i};
    if localHasField(raw, name)
        rawValue = raw.(name);
        if ismissing(rawValue) || isempty(rawValue)
            continue;
        end
        value = string(rawValue);
        if strlength(value) > 0
            value = strtrim(value);
            return;
        end
    end
end
end

function value = localNumericField(raw, names)
value = NaN;
for i = 1:numel(names)
    name = names{i};
    if ~localHasField(raw, name)
        continue;
    end
    rawValue = raw.(name);
    value = localRawToDouble(rawValue);
    if isfinite(value) || (isnumeric(rawValue) && isnan(value))
        return;
    end
end
end

function value = localLogicalField(raw, names)
value = NaN;
for i = 1:numel(names)
    name = names{i};
    if ~localHasField(raw, name)
        continue;
    end
    rawValue = raw.(name);
    if islogical(rawValue)
        value = logical(rawValue);
        return;
    end
    if isnumeric(rawValue)
        if isscalar(rawValue) && ~isnan(rawValue)
            value = logical(rawValue ~= 0);
            return;
        end
    end
    rawText = lower(strtrim(string(rawValue)));
    if rawText == "1" || rawText == "true" || rawText == "yes"
        value = true;
        return;
    elseif rawText == "0" || rawText == "false" || rawText == "no"
        value = false;
        return;
    end
end
end

function value = localLogicalFieldOrDefault(raw, names, fallback)
value = localLogicalField(raw, names);
if isnan(double(value))
    value = logical(fallback);
else
    value = logical(value);
end
end

function value = localRawToDouble(rawValue)
if isempty(rawValue)
    value = NaN;
    return;
end
if isnumeric(rawValue)
    value = double(rawValue(1));
    return;
end
if islogical(rawValue)
    value = double(rawValue(1));
    return;
end
textValue = strtrim(string(rawValue));
if strlength(textValue) == 0 || textValue == "<missing>"
    value = NaN;
    return;
end
value = str2double(textValue);
end

function token = localSanitizeToken(value)
token = lower(strtrim(string(value)));
if strlength(token) == 0
    token = "none";
    return;
end
token = regexprep(token, '[^a-zA-Z0-9]+', '_');
token = regexprep(token, '^_+|_+$', '');
if strlength(token) == 0
    token = "none";
end
end

function token = localNumberToken(value)
if ~isfinite(value)
    token = "nan";
    return;
end
token = string(sprintf('%.6f', value));
token = regexprep(token, '0+$', '');
token = regexprep(token, '\.$', '');
token = replace(token, '-', 'm');
token = replace(token, '.', 'p');
if strlength(token) == 0
    token = "0";
end
end
