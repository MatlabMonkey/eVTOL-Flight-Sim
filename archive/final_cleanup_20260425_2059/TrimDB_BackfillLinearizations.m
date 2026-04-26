% TrimDB_BackfillLinearizations.m
% Backfill missing linearization files for transition trim points by replaying
% the stored trimCase data preserved in the source MAT files.
%
% Default behavior:
%   - target best-unique exact trim points that are missing linearization files
%   - keep only rear-on points (controller-relevant set)
%   - prefer stored trimResult.linear when a source MAT already has it
%   - otherwise replay trim_evtol_case(trimCase) to regenerate the linearization
%   - write a new linearization index + linearizationPoint MAT files
%   - rebuild the transition trim databases against the new index
%
% Usage:
%   TrimDB_BackfillLinearizations
%
% Optional configuration:
%   transitionTrimLinearizationBackfillOptions = struct( ...
%       'rear_on_only', true, ...
%       'classification', "exact_trim", ...
%       'source_files', {'transition_trim_map_latest.csv'}, ...
%       'limit', inf, ...
%       'prefer_stored_trimresult_linear', true, ...
%       'disable_nonlinear_hold', true, ...
%       'rebuild_trim_databases', true);
%   TrimDB_BackfillLinearizations

if ~exist('transitionTrimLinearizationBackfillOptions', 'var') || ...
        ~isstruct(transitionTrimLinearizationBackfillOptions)
    transitionTrimLinearizationBackfillOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    rootDir = fileparts(stack(1).file);
else
    rootDir = pwd;
end
dbPaths = TrimDB_Paths(rootDir);
workspacePlotsDir = dbPaths.workspace_plots_dir;
timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));

transitionTrimLinearizationBackfillOptions = localApplyDefaults( ...
    transitionTrimLinearizationBackfillOptions, rootDir, workspacePlotsDir, timestamp);

initOptions = struct();
initOptions.transitionTrimLinearizationBackfillOptions = transitionTrimLinearizationBackfillOptions;
Init_Main
if exist('initOptions', 'var') && isstruct(initOptions) && ...
        isfield(initOptions, 'transitionTrimLinearizationBackfillOptions')
    transitionTrimLinearizationBackfillOptions = initOptions.transitionTrimLinearizationBackfillOptions;
end

[masterDb, masterDbVarName] = localLoadMasterDb(transitionTrimLinearizationBackfillOptions.master_db_file);
candidateTable = localSelectCandidateRows(masterDb, transitionTrimLinearizationBackfillOptions);
sourceRepository = localBuildSourceRepository(masterDb, candidateTable, transitionTrimLinearizationBackfillOptions);

if exist(transitionTrimLinearizationBackfillOptions.output_dir, 'dir') ~= 7
    mkdir(transitionTrimLinearizationBackfillOptions.output_dir);
end
if exist(transitionTrimLinearizationBackfillOptions.linearization_output_dir, 'dir') ~= 7
    mkdir(transitionTrimLinearizationBackfillOptions.linearization_output_dir);
end
if exist(transitionTrimLinearizationBackfillOptions.latest_linearization_dir, 'dir') ~= 7
    mkdir(transitionTrimLinearizationBackfillOptions.latest_linearization_dir);
end

[indexRows, rowIdToIndex] = localLoadExistingIndex(transitionTrimLinearizationBackfillOptions.latest_index_mat);

summaryCsv = fullfile(transitionTrimLinearizationBackfillOptions.output_dir, ...
    'transition_trim_linearization_backfill_summary.csv');
summaryMd = fullfile(transitionTrimLinearizationBackfillOptions.output_dir, ...
    'transition_trim_linearization_backfill_summary.md');
checkpointMat = fullfile(transitionTrimLinearizationBackfillOptions.output_dir, ...
    'transition_trim_linearization_backfill.mat');
latestSummaryCsv = fullfile(transitionTrimLinearizationBackfillOptions.workspace_plots_dir, ...
    'transition_trim_linearization_backfill_summary_latest.csv');
latestSummaryMd = fullfile(transitionTrimLinearizationBackfillOptions.workspace_plots_dir, ...
    'transition_trim_linearization_backfill_summary_latest.md');
latestCheckpointMat = fullfile(transitionTrimLinearizationBackfillOptions.workspace_plots_dir, ...
    'transition_trim_linearization_backfill_latest.mat');

fprintf('=== TrimDB_BackfillLinearizations ===\n');
fprintf('master DB: %s (%s)\n', transitionTrimLinearizationBackfillOptions.master_db_file, masterDbVarName);
fprintf('candidate rows: %d\n', height(candidateTable));
fprintf('source repository items: %d\n', numel(sourceRepository.items));
fprintf('output dir: %s\n', transitionTrimLinearizationBackfillOptions.output_dir);
fprintf('latest linearization dir: %s\n', transitionTrimLinearizationBackfillOptions.latest_linearization_dir);

backfillResult = struct();
backfillResult.meta = struct( ...
    'created_on', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')), ...
    'root_dir', transitionTrimLinearizationBackfillOptions.root_dir, ...
    'database_dir', transitionTrimLinearizationBackfillOptions.database_dir, ...
    'workspace_plots_dir', transitionTrimLinearizationBackfillOptions.workspace_plots_dir, ...
    'master_db_file', transitionTrimLinearizationBackfillOptions.master_db_file, ...
    'master_db_var', masterDbVarName, ...
    'options', transitionTrimLinearizationBackfillOptions, ...
    'source_repository_summary', sourceRepository.summary);
backfillResult.progress = struct('completed', 0, 'saved_count', 0, 'failed_count', 0, 'skipped_count', 0);
backfillResult.entries = repmat(localResultEntryTemplate(), 0, 1);

for iRow = 1:height(candidateTable)
    row = candidateTable(iRow, :);
    resultEntry = localResultEntryTemplate();
    resultEntry.index = iRow;
    resultEntry.key = string(row.key);
    resultEntry.name = string(row.name);
    resultEntry.source_file = string(row.source_file);
    resultEntry.tilt_deg = row.tilt_deg;
    resultEntry.vinf_mps = row.vinf_mps;
    resultEntry.family = string(row.family);
    resultEntry.front_collective_rpm = row.front_collective_rpm;
    resultEntry.rear_collective_rpm = row.rear_collective_rpm;
    resultEntry.classification = string(row.classification);
    resultEntry.original_linearization_available = logical(row.linearization_available);

    fprintf('\n[%d/%d] %s | source=%s | tilt=%.1f | V=%.1f | family=%s\n', ...
        iRow, height(candidateTable), row.name, row.source_file, row.tilt_deg, row.vinf_mps, row.family);

    rowId = localBackfillRowIdFromTableRow(row);
    resultEntry.row_id = rowId;

    if transitionTrimLinearizationBackfillOptions.skip_existing && isKey(rowIdToIndex, rowId)
        existingIdx = rowIdToIndex(rowId);
        existingRow = indexRows(existingIdx);
        if strlength(existingRow.latest_file) > 0 && exist(existingRow.latest_file, 'file') == 2
            resultEntry.status = "skipped_existing";
            resultEntry.match_mode = "existing_index";
            resultEntry.linearization_latest_file = existingRow.latest_file;
            fprintf('  skipped: existing linearization file already present.\n');
            backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
            backfillResult.progress.completed = numel(backfillResult.entries);
            backfillResult.progress.skipped_count = backfillResult.progress.skipped_count + 1;
            continue;
        end
    end

    [sourceItem, matchInfo] = localFindBestSourceItem(row, sourceRepository.items, transitionTrimLinearizationBackfillOptions);
    resultEntry.match_mode = matchInfo.mode;
    resultEntry.match_detail = matchInfo.detail;
    resultEntry.source_mat_file = matchInfo.source_mat_file;
    resultEntry.source_var_name = matchInfo.source_var_name;

    if ~matchInfo.found
        resultEntry.status = "no_source_match";
        resultEntry.error_message = matchInfo.detail;
        fprintf('  no source match.\n');
        backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
        backfillResult.progress.completed = numel(backfillResult.entries);
        backfillResult.progress.failed_count = backfillResult.progress.failed_count + 1;
        continue;
    end

    trimCase = sourceItem.trim_case;
    if ~localStructHasFields(trimCase)
        resultEntry.status = "missing_trim_case";
        resultEntry.error_message = "matched source item has no trimCase";
        fprintf('  matched item has no trimCase.\n');
        backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
        backfillResult.progress.completed = numel(backfillResult.entries);
        backfillResult.progress.failed_count = backfillResult.progress.failed_count + 1;
        continue;
    end

    if transitionTrimLinearizationBackfillOptions.disable_nonlinear_hold
        trimCase.validate_nonlinear_hold = false;
    end

    resultEntry.trim_case_name = string(localGetStructString(trimCase, 'name', ""));

    trimResult = struct();
    trimSpec = struct();
    consoleText = "";
    if transitionTrimLinearizationBackfillOptions.prefer_stored_trimresult_linear && ...
            localTrimResultHasUsableLinear(sourceItem.trim_result)
        trimResult = sourceItem.trim_result;
        resultEntry.replay_mode = "stored_trimResult";
        fprintf('  using stored trimResult.linear from source MAT.\n');
    else
        try
            options_i = struct( ...
                'verbose', false, ...
                'debug', false, ...
                'emitSummary', false, ...
                'emitLinearSummary', false);
            consoleText = string(evalc('[trimResult, trimSpec] = trim_evtol_case(initData, trimCase, options_i);')); %#ok<NASGU>
            resultEntry.replay_mode = "replayed_trimCase";
            fprintf('  replayed trimCase.\n');
        catch ME
            resultEntry.status = "replay_exception";
            resultEntry.error_identifier = string(ME.identifier);
            resultEntry.error_message = string(ME.message);
            resultEntry.console_text = consoleText;
            fprintf('  replay failed: %s\n', ME.message);
            backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
            backfillResult.progress.completed = numel(backfillResult.entries);
            backfillResult.progress.failed_count = backfillResult.progress.failed_count + 1;
            localMaybeCheckpoint( ...
                backfillResult, indexRows, transitionTrimLinearizationBackfillOptions, ...
                checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd);
            continue;
        end
    end

    resultEntry.console_text = consoleText;
    resultEntry.replay_success = logical(localGetStructField(trimResult, 'success', false));
    resultEntry.termination_string = string(localGetStructField(trimResult, 'terminationString', ""));
    resultEntry.max_state_residual = localMaxStateResidual(localGetStructField(trimResult, 'op_report', struct()));

    if ~localTrimResultHasUsableLinear(trimResult)
        resultEntry.status = "missing_linear_after_replay";
        resultEntry.error_message = "trimResult.linear.sys_ss_9state not available";
        fprintf('  no usable linear model on result.\n');
        backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
        backfillResult.progress.completed = numel(backfillResult.entries);
        backfillResult.progress.failed_count = backfillResult.progress.failed_count + 1;
        localMaybeCheckpoint( ...
            backfillResult, indexRows, transitionTrimLinearizationBackfillOptions, ...
            checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd);
        continue;
    end

    if strcmp(resultEntry.replay_mode, "replayed_trimCase") && ~resultEntry.replay_success
        resultEntry.status = "replay_not_exact";
        resultEntry.error_message = "trim replay did not return success";
        fprintf('  replay did not return an exact trim.\n');
        backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
        backfillResult.progress.completed = numel(backfillResult.entries);
        backfillResult.progress.failed_count = backfillResult.progress.failed_count + 1;
        localMaybeCheckpoint( ...
            backfillResult, indexRows, transitionTrimLinearizationBackfillOptions, ...
            checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd);
        continue;
    end

    linearizationPoint = localBuildLinearizationPoint(row, sourceItem, trimCase, trimResult);
    fileStem = char("linearization_" + localSanitizeFileLabel(localPreferredFileLabel(row)));
    outputFile = char(fullfile(transitionTrimLinearizationBackfillOptions.linearization_output_dir, [fileStem '.mat']));
    latestFile = char(fullfile(transitionTrimLinearizationBackfillOptions.latest_linearization_dir, [fileStem '.mat']));
    save(outputFile, 'linearizationPoint', '-v7.3');
    save(latestFile, 'linearizationPoint', '-v7.3');

    indexRow = localBuildIndexRow(row, outputFile, latestFile);
    if isKey(rowIdToIndex, rowId)
        indexRows(rowIdToIndex(rowId)) = indexRow;
    else
        indexRows(end + 1, 1) = indexRow; %#ok<SAGROW>
        rowIdToIndex(rowId) = numel(indexRows);
    end
    localWriteIndexFiles(indexRows, transitionTrimLinearizationBackfillOptions);

    resultEntry.status = "saved";
    resultEntry.linearization_output_file = string(outputFile);
    resultEntry.linearization_latest_file = string(latestFile);
    fprintf('  saved: %s\n', latestFile);

    backfillResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
    backfillResult.progress.completed = numel(backfillResult.entries);
    backfillResult.progress.saved_count = backfillResult.progress.saved_count + 1;

    localMaybeCheckpoint( ...
        backfillResult, indexRows, transitionTrimLinearizationBackfillOptions, ...
        checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd);
end

[backfillSummaryTable, backfillResult] = localFinalizeCheckpoint( ...
    backfillResult, indexRows, transitionTrimLinearizationBackfillOptions, ...
    checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd);

if transitionTrimLinearizationBackfillOptions.rebuild_trim_databases
    fprintf('\nRebuilding transition trim databases against backfill index...\n');
    transitionTrimDatabaseOptions = struct( ...
        'include_smoke_sources', false, ...
        'controller_include_quasi_trims', false, ...
        'controller_allow_trim_only_candidates', false, ...
        'controller_exclude_zero_rear', true, ...
        'include_full_order_linearization', true, ...
        'extra_linearization_index_sources', {{char(transitionTrimLinearizationBackfillOptions.latest_index_csv)}}, ...
        'extra_linearization_roots', {{char(transitionTrimLinearizationBackfillOptions.latest_linearization_dir)}});
    rebuildText = evalc('TrimDB_Build;');
    backfillResult.rebuild_trim_databases_console = rebuildText;
end

assignin('base', 'transitionTrimLinearizationBackfillOptions', transitionTrimLinearizationBackfillOptions);
assignin('base', 'transitionTrimLinearizationBackfillResult', backfillResult);
assignin('base', 'transitionTrimLinearizationBackfillSummary', backfillSummaryTable);

fprintf('\nCompleted backfill.\n');
fprintf('  saved = %d\n', backfillResult.progress.saved_count);
fprintf('  failed = %d\n', backfillResult.progress.failed_count);
fprintf('  skipped = %d\n', backfillResult.progress.skipped_count);
fprintf('  latest index csv = %s\n', transitionTrimLinearizationBackfillOptions.latest_index_csv);

function opts = localApplyDefaults(opts, rootDir, workspacePlotsDir, timestamp)
if ~isfield(opts, 'root_dir') || isempty(opts.root_dir)
    opts.root_dir = rootDir;
end
dbPaths = TrimDB_Paths(opts.root_dir);
if ~isfield(opts, 'database_dir') || isempty(opts.database_dir)
    opts.database_dir = dbPaths.database_dir;
end
if ~isfield(opts, 'workspace_plots_dir') || isempty(opts.workspace_plots_dir)
    opts.workspace_plots_dir = workspacePlotsDir;
end
if ~isfield(opts, 'master_db_file') || isempty(opts.master_db_file)
    opts.master_db_file = fullfile(opts.database_dir, 'trim_attempts.mat');
end
if ~isfield(opts, 'rear_on_only') || isempty(opts.rear_on_only)
    opts.rear_on_only = true;
end
if ~isfield(opts, 'classification') || isempty(opts.classification)
    opts.classification = "exact_trim";
end
if ~isfield(opts, 'include_quasi_trims') || isempty(opts.include_quasi_trims)
    opts.include_quasi_trims = false;
end
if ~isfield(opts, 'selection_table') || isempty(opts.selection_table)
    opts.selection_table = "best_unique";
end
if ~isfield(opts, 'source_files') || isempty(opts.source_files)
    opts.source_files = {};
end
if ischar(opts.source_files) || isstring(opts.source_files)
    opts.source_files = cellstr(opts.source_files);
end
if ~isfield(opts, 'limit') || isempty(opts.limit)
    opts.limit = inf;
end
if ~isfield(opts, 'skip_existing') || isempty(opts.skip_existing)
    opts.skip_existing = true;
end
if ~isfield(opts, 'prefer_stored_trimresult_linear') || isempty(opts.prefer_stored_trimresult_linear)
    opts.prefer_stored_trimresult_linear = true;
end
if ~isfield(opts, 'disable_nonlinear_hold') || isempty(opts.disable_nonlinear_hold)
    opts.disable_nonlinear_hold = true;
end
if ~isfield(opts, 'rebuild_trim_databases') || isempty(opts.rebuild_trim_databases)
    opts.rebuild_trim_databases = true;
end
if ~isfield(opts, 'checkpoint_every') || isempty(opts.checkpoint_every)
    opts.checkpoint_every = 10;
end
if ~isfield(opts, 'tilt_match_tolerance') || isempty(opts.tilt_match_tolerance)
    opts.tilt_match_tolerance = 1e-6;
end
if ~isfield(opts, 'vinf_match_tolerance') || isempty(opts.vinf_match_tolerance)
    opts.vinf_match_tolerance = 1e-6;
end
if ~isfield(opts, 'rpm_match_tolerance') || isempty(opts.rpm_match_tolerance)
    opts.rpm_match_tolerance = 1e-2;
end
if ~isfield(opts, 'output_dir') || isempty(opts.output_dir)
    opts.output_dir = fullfile(workspacePlotsDir, ['transition_trim_linearization_backfill_' timestamp]);
end
if ~isfield(opts, 'linearization_output_dir') || isempty(opts.linearization_output_dir)
    opts.linearization_output_dir = fullfile(opts.output_dir, 'linearizations');
end
if ~isfield(opts, 'latest_linearization_dir') || isempty(opts.latest_linearization_dir)
    opts.latest_linearization_dir = fullfile(opts.database_dir, 'transition_trim_linearization_backfill_linearizations');
end
if ~isfield(opts, 'index_mat') || isempty(opts.index_mat)
    opts.index_mat = fullfile(opts.output_dir, 'transition_trim_linearization_backfill_index.mat');
end
if ~isfield(opts, 'index_csv') || isempty(opts.index_csv)
    opts.index_csv = fullfile(opts.output_dir, 'transition_trim_linearization_backfill_index.csv');
end
if ~isfield(opts, 'latest_index_mat') || isempty(opts.latest_index_mat)
    opts.latest_index_mat = fullfile(opts.database_dir, 'transition_trim_linearization_backfill_index.mat');
end
if ~isfield(opts, 'latest_index_csv') || isempty(opts.latest_index_csv)
    opts.latest_index_csv = fullfile(opts.database_dir, 'transition_trim_linearization_backfill_index.csv');
end
end

function [db, varName] = localLoadMasterDb(filename)
if exist(filename, 'file') ~= 2
    error('Master DB file not found: %s', filename);
end
data = load(filename);
vars = fieldnames(data);
for i = 1:numel(vars)
    value = data.(vars{i});
    if isstruct(value) && isfield(value, 'master_attempt_db_best_unique_points')
        db = value;
        varName = vars{i};
        return;
    end
end
error('Could not find a transition trim master DB struct in %s.', filename);
end

function candidateTable = localSelectCandidateRows(masterDb, opts)
if opts.selection_table == "all_rows"
    candidateTable = masterDb.master_attempt_db_all_rows;
else
    candidateTable = masterDb.master_attempt_db_best_unique_points;
end

if isempty(candidateTable)
    return;
end

if opts.include_quasi_trims
    keepMask = strcmp(candidateTable.classification, string(opts.classification)) | candidateTable.acceptable;
else
    keepMask = strcmp(candidateTable.classification, string(opts.classification));
end
keepMask = keepMask & ~candidateTable.linearization_available;
if opts.rear_on_only
    keepMask = keepMask & candidateTable.rear_on_ok;
end
if ~isempty(opts.source_files)
    sourceMask = false(height(candidateTable), 1);
    for i = 1:numel(opts.source_files)
        sourceMask = sourceMask | strcmp(candidateTable.source_file, string(opts.source_files{i}));
    end
    keepMask = keepMask & sourceMask;
end
candidateTable = candidateTable(keepMask, :);
candidateTable = sortrows(candidateTable, {'source_file', 'tilt_deg', 'vinf_mps', 'family'});

if isfinite(opts.limit)
    candidateTable = candidateTable(1:min(height(candidateTable), max(1, round(opts.limit))), :);
end
end

function repository = localBuildSourceRepository(masterDb, candidateTable, opts)
trimSources = localSelectTrimSourcesForCandidates(masterDb.manifest.trim_sources, candidateTable);
items = repmat(localSourceItemTemplate(), 0, 1);
status = repmat(localSourceMatStatusTemplate(), 0, 1);

for iSource = 1:numel(trimSources)
    src = trimSources(iSource);
    matStatus = localSourceMatStatusTemplate();
    matStatus.source_csv_file = string(src.filename);
    matStatus.enabled = logical(src.enabled);
    if ~src.enabled
        matStatus.status = "excluded";
        status(end + 1, 1) = matStatus; %#ok<SAGROW>
        continue;
    end

    matPath = localCsvSourceToMatPath(src.path);
    matStatus.source_mat_file = string(matPath);
    if exist(matPath, 'file') ~= 2
        matStatus.status = "missing_mat";
        status(end + 1, 1) = matStatus; %#ok<SAGROW>
        continue;
    end

    try
        data = load(matPath);
    catch ME
        matStatus.status = "load_failed";
        matStatus.detail = string(ME.message);
        status(end + 1, 1) = matStatus; %#ok<SAGROW>
        continue;
    end

    vars = fieldnames(data);
    loadedCount = 0;
    for iVar = 1:numel(vars)
        value = data.(vars{iVar});
        items_i = localExtractSourceItems(value, string(src.filename), string(matPath), string(vars{iVar}));
        if ~isempty(items_i)
            items = [items; items_i]; %#ok<AGROW>
            loadedCount = loadedCount + numel(items_i);
        end
    end

    matStatus.status = "loaded";
    matStatus.item_count = loadedCount;
    status(end + 1, 1) = matStatus; %#ok<SAGROW>
end

function trimSources = localSelectTrimSourcesForCandidates(allTrimSources, candidateTable)
if isempty(candidateTable)
    trimSources = allTrimSources;
    return;
end

candidateFiles = unique(string(candidateTable.source_file));
neededFiles = candidateFiles;

if any(candidateFiles == "transition_trim_map_merged_latest.csv")
    neededFiles = [neededFiles; ...
        "transition_trim_map_latest.csv"; ...
        "transition_trim_map_low_speed_latest.csv"; ...
        "transition_trim_map_low_speed_scored_latest.csv"; ...
        "transition_trim_map_bridge_scored_latest.csv"]; %#ok<AGROW>
end

trimSources = repmat(allTrimSources(1), 0, 1);
for i = 1:numel(allTrimSources)
    src = allTrimSources(i);
    if any(string(src.filename) == neededFiles)
        trimSources(end + 1, 1) = src; %#ok<SAGROW>
    end
end

if isempty(trimSources)
    trimSources = allTrimSources;
end
end

repository = struct();
repository.items = items;
repository.status = status;
repository.summary = struct( ...
    'source_mat_count', nnz(strcmp({status.status}, "loaded")), ...
    'item_count', numel(items));
end

function path = localCsvSourceToMatPath(csvPath)
[folderPath, baseName, ~] = fileparts(char(csvPath));
path = fullfile(folderPath, [baseName '.mat']);
end

function items = localExtractSourceItems(value, sourceCsvFile, sourceMatFile, varName)
items = repmat(localSourceItemTemplate(), 0, 1);
if ~isstruct(value)
    return;
end

if isfield(value, 'entries') && isstruct(value.entries) && ~isempty(value.entries)
    for i = 1:numel(value.entries)
        item = localNormalizeEntryItem(value.entries(i), sourceCsvFile, sourceMatFile, varName, i);
        items(end + 1, 1) = item; %#ok<SAGROW>
    end
end

if isfield(value, 'results') && isstruct(value.results) && ~isempty(value.results)
    summaryTable = localResolveSummaryTable(value);
    for i = 1:numel(value.results)
        summaryRow = struct();
        if ~isempty(summaryTable) && height(summaryTable) >= i
            summaryRow = table2struct(summaryTable(i, :));
        end
        item = localNormalizeResultItem(value.results(i), summaryRow, sourceCsvFile, sourceMatFile, varName, i);
        items(end + 1, 1) = item; %#ok<SAGROW>
    end
end

if isempty(items) && localStructHasAnyField(value, {'trimCase', 'trim_case'})
    item = localSourceItemTemplate();
    item.source_csv_file = string(sourceCsvFile);
    item.source_mat_file = string(sourceMatFile);
    item.source_var_name = string(varName);
    item.name = localStringField(value, {'name'});
    item.family = localStringField(value, {'family'});
    item.tilt_deg = localNumericField(value, {'target_tilt_deg', 'tilt_deg'});
    item.vinf_mps = localNumericField(value, {'target_vinf_mps', 'vinf_mps'});
    item.trim_case = localGetStructField(value, 'trimCase', localGetStructField(value, 'trim_case', struct()));
    item.trim_result = localGetStructField(value, 'trimResult', struct());
    item.key = localBuildCanonicalKey(sourceCsvFile, item.name, item.tilt_deg, item.vinf_mps, item.family, "", 1);
    items(end + 1, 1) = item; %#ok<SAGROW>
end
end

function summaryTable = localResolveSummaryTable(value)
summaryTable = table();
if isfield(value, 'summary_table') && istable(value.summary_table)
    summaryTable = value.summary_table;
    return;
end
if isfield(value, 'summaryTable') && istable(value.summaryTable)
    summaryTable = value.summaryTable;
    return;
end
end

function item = localNormalizeEntryItem(entry, sourceCsvFile, sourceMatFile, varName, rowIndex)
item = localSourceItemTemplate();
item.source_csv_file = string(sourceCsvFile);
item.source_mat_file = string(sourceMatFile);
item.source_var_name = string(varName);
item.key = localStringField(entry, {'key'});
item.name = localStringField(entry, {'name'});
item.family = localStringField(entry, {'family'});
item.seed_name = localStringField(entry, {'seed_name', 'chosen_seed_name'});
item.tilt_deg = localNumericField(entry, {'target_tilt_deg', 'tilt_deg'});
item.vinf_mps = localNumericField(entry, {'target_vinf_mps', 'vinf_mps'});
item.rear_fixed_rpm = localNumericField(entry, {'target_rear_fixed_rpm', 'rear_fixed_rpm'});
item.success = localLogicalFieldOrDefault(entry, {'success'}, false);
item.acceptable = localLogicalFieldOrDefault(entry, {'acceptable'}, item.success);
item.classification = localStringField(entry, {'classification'});
item.score = localNumericField(entry, {'score'});
item.max_normalized = localNumericField(entry, {'max_normalized'});
item.front_collective_rpm = localNumericField(entry, {'front_collective_rpm'});
item.rear_collective_rpm = localNumericField(entry, {'rear_collective_rpm'});
item.trim_case = localGetStructField(entry, 'trimCase', localGetStructField(entry, 'trim_case', struct()));
item.trim_result = localGetStructField(entry, 'trimResult', struct());
if localStructHasFields(localGetStructField(entry, 'trim_summary', struct()))
    trimSummary = entry.trim_summary;
    if ~isfinite(item.front_collective_rpm)
        item.front_collective_rpm = localNumericField(trimSummary, {'front_collective_rpm'});
    end
    if ~isfinite(item.rear_collective_rpm)
        item.rear_collective_rpm = localNumericField(trimSummary, {'rear_collective_rpm'});
    end
end
if strlength(item.key) == 0
    item.key = localBuildCanonicalKey(sourceCsvFile, item.name, item.tilt_deg, item.vinf_mps, item.family, item.seed_name, rowIndex);
end
end

function item = localNormalizeResultItem(result, summaryRow, sourceCsvFile, sourceMatFile, varName, rowIndex)
item = localSourceItemTemplate();
item.source_csv_file = string(sourceCsvFile);
item.source_mat_file = string(sourceMatFile);
item.source_var_name = string(varName);
item.name = localFirstNonEmptyString( ...
    localStringField(summaryRow, {'name'}), ...
    localStringField(localGetStructField(result, 'spec', struct()), {'name'}), ...
    localStringField(result, {'name'}));
item.family = localFirstNonEmptyString( ...
    localStringField(summaryRow, {'family'}), ...
    localStringField(localGetStructField(result, 'spec', struct()), {'family'}), ...
    localStringField(result, {'family'}));
item.seed_name = localFirstNonEmptyString( ...
    localStringField(summaryRow, {'seed_name'}), ...
    localStringField(result, {'seed_name'}));
item.tilt_deg = localFirstFinite( ...
    localNumericField(summaryRow, {'target_tilt_deg', 'tilt_deg'}), ...
    localNumericField(localGetStructField(result, 'spec', struct()), {'tilt_deg', 'target_tilt_deg'}), ...
    localNumericField(result, {'tilt_deg'}));
item.vinf_mps = localFirstFinite( ...
    localNumericField(summaryRow, {'target_vinf_mps', 'vinf_mps'}), ...
    localNumericField(localGetStructField(result, 'spec', struct()), {'vinf_mps', 'target_vinf_mps'}), ...
    localNumericField(result, {'vinf_mps'}));
item.rear_fixed_rpm = localFirstFinite( ...
    localNumericField(summaryRow, {'rear_fixed_rpm'}), ...
    localNumericField(localGetStructField(result, 'spec', struct()), {'rear_fixed_rpm'}));
item.success = localLogicalFieldOrDefault(summaryRow, {'success'}, localLogicalFieldOrDefault(result, {'success'}, false));
item.acceptable = localLogicalFieldOrDefault(summaryRow, {'acceptable'}, item.success);
item.classification = localFirstNonEmptyString( ...
    localStringField(summaryRow, {'classification'}), ...
    localStringField(result, {'classification'}));
item.score = localFirstFinite( ...
    localNumericField(summaryRow, {'score'}), ...
    localNumericField(result, {'score'}));
item.max_normalized = localFirstFinite( ...
    localNumericField(summaryRow, {'max_normalized'}), ...
    localNumericField(result, {'max_normalized'}));
item.front_collective_rpm = localFirstFinite( ...
    localNumericField(summaryRow, {'front_collective_rpm'}), ...
    localNumericField(result, {'front_collective_rpm'}));
item.rear_collective_rpm = localFirstFinite( ...
    localNumericField(summaryRow, {'rear_collective_rpm'}), ...
    localNumericField(result, {'rear_collective_rpm'}));
item.trim_case = localGetStructField(result, 'trimCase', localGetStructField(result, 'trim_case', struct()));
item.trim_result = localGetStructField(result, 'trimResult', struct());
item.key = localBuildCanonicalKey(sourceCsvFile, item.name, item.tilt_deg, item.vinf_mps, item.family, item.seed_name, rowIndex);
end

function [indexRows, rowIdToIndex] = localLoadExistingIndex(indexMatFile)
indexRows = repmat(localLinearizationIndexRowTemplate(), 0, 1);
rowIdToIndex = containers.Map('KeyType', 'char', 'ValueType', 'double');

if exist(indexMatFile, 'file') ~= 2
    return;
end

data = load(indexMatFile);
if ~isfield(data, 'linearizationIndex') || ~isstruct(data.linearizationIndex)
    return;
end

indexRows = data.linearizationIndex;
for i = 1:numel(indexRows)
    rowId = localBackfillRowIdFromIndexRow(indexRows(i));
    rowIdToIndex(rowId) = i;
end
end

function [item, matchInfo] = localFindBestSourceItem(row, items, opts)
item = localSourceItemTemplate();
matchInfo = struct('found', false, 'mode', "", 'detail', "", 'source_mat_file', "", 'source_var_name', "");

if isempty(items)
    matchInfo.detail = "source repository is empty";
    return;
end

baseMask = false(numel(items), 1);
for i = 1:numel(items)
    baseMask(i) = localStructHasFields(items(i).trim_case);
end
if ~any(baseMask)
    matchInfo.detail = "no source items contain trimCase";
    return;
end

rowKey = string(row.key);
rowName = string(row.name);
rowSource = string(row.source_file);
rowFamily = string(row.family);
rowTilt = row.tilt_deg;
rowVinf = row.vinf_mps;
rowFront = row.front_collective_rpm;
rowRear = row.rear_collective_rpm;

keyMask = false(numel(items), 1);
nameMask = false(numel(items), 1);
geomMask = false(numel(items), 1);
sourceMask = false(numel(items), 1);
for i = 1:numel(items)
    keyMask(i) = strlength(rowKey) > 0 && items(i).key == rowKey;
    nameMask(i) = strlength(rowName) > 0 && items(i).name == rowName;
    geomMask(i) = isfinite(rowTilt) && isfinite(rowVinf) && ...
        abs(items(i).tilt_deg - rowTilt) <= opts.tilt_match_tolerance && ...
        abs(items(i).vinf_mps - rowVinf) <= opts.vinf_match_tolerance && ...
        strcmpi(items(i).family, rowFamily);
    sourceMask(i) = items(i).source_csv_file == rowSource;
end

candidateMask = baseMask;
mode = "global";
if any(keyMask & baseMask)
    candidateMask = candidateMask & keyMask;
    mode = "key";
elseif any(geomMask & baseMask)
    candidateMask = candidateMask & geomMask;
    mode = "geom_family";
elseif any(nameMask & baseMask)
    candidateMask = candidateMask & nameMask;
    mode = "name";
else
    matchInfo.detail = "no matching key/name/geometry source item";
    return;
end

if any(candidateMask & sourceMask)
    candidateMask = candidateMask & sourceMask;
    mode = mode + "_source";
end
if any(candidateMask & nameMask)
    candidateMask = candidateMask & nameMask;
    mode = mode + "_name";
end

candidateIdx = find(candidateMask);
rank = zeros(numel(candidateIdx), 6);
for i = 1:numel(candidateIdx)
    idx = candidateIdx(i);
    rank(i, 1) = ~(items(idx).source_csv_file == rowSource);
    rank(i, 2) = ~(strlength(rowKey) > 0 && items(idx).key == rowKey);
    rank(i, 3) = ~(strlength(rowName) > 0 && items(idx).name == rowName);
    rank(i, 4) = localFiniteDifference(items(idx).rear_collective_rpm, rowRear);
    rank(i, 5) = localFiniteDifference(items(idx).front_collective_rpm, rowFront);
    rank(i, 6) = idx;
end
rankTable = array2table(rank, 'VariableNames', ...
    {'source_miss', 'key_miss', 'name_miss', 'rear_diff', 'front_diff', 'original_idx'});
rankTable = sortrows(rankTable, {'source_miss', 'key_miss', 'name_miss', 'rear_diff', 'front_diff', 'original_idx'});
bestIdx = rankTable.original_idx(1);

item = items(bestIdx);
matchInfo.found = true;
matchInfo.mode = mode;
matchInfo.detail = sprintf('matched source item: %s | %s', item.source_csv_file, item.name);
matchInfo.source_mat_file = item.source_mat_file;
matchInfo.source_var_name = item.source_var_name;
end

function linearizationPoint = localBuildLinearizationPoint(row, sourceItem, trimCase, trimResult)
linearizationPoint = struct();
linearizationPoint.name = char(string(row.name));
linearizationPoint.key = char(string(row.key));
linearizationPoint.family = char(string(row.family));
linearizationPoint.seed_name = char(localFirstNonEmptyString(string(row.seed_name), sourceItem.seed_name));
linearizationPoint.source_file = char(string(row.source_file));
linearizationPoint.source_mat_file = char(string(sourceItem.source_mat_file));
linearizationPoint.source_var_name = char(string(sourceItem.source_var_name));
linearizationPoint.success = logical(localGetStructField(trimResult, 'success', true));
linearizationPoint.acceptable = logical(localGetTableLogical(row, 'acceptable', true));
linearizationPoint.classification = char(string(row.classification));
linearizationPoint.score = localGetTableNumeric(row, 'score', NaN);
linearizationPoint.max_normalized = localGetTableNumeric(row, 'max_normalized', NaN);
linearizationPoint.worst_component = char(localGetTableString(row, 'worst_component', ""));
linearizationPoint.worst_component_normalized = localGetTableNumeric(row, 'worst_component_normalized', NaN);
linearizationPoint.termination_string = char(localGetStructString(trimResult, 'terminationString', localGetTableString(row, 'termination_string', "")));
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
linearizationPoint.trimCase = trimCase;
linearizationPoint.scheduling = localGetStructField(trimResult, 'scheduling', struct());
linearizationPoint.Att_Trim_deg = localGetStructField(trimResult, 'Att_Trim_deg', []);
linearizationPoint.Vel_B_BA_Trim = localGetStructField(trimResult, 'Vel_B_BA_Trim', []);
linearizationPoint.Rates_Trim = localGetStructField(trimResult, 'Rates_Trim', []);
linearizationPoint.linear = localExtractLinearData(localGetStructField(trimResult, 'linear', struct()));
end

function linear = localExtractLinearData(linearData)
linear = struct();
linear.reduced_model_available = localGetStructField(linearData, 'reduced_model_available', false);
linear.A_full = localGetStructField(linearData, 'A_full', []);
linear.B_full = localGetStructField(linearData, 'B_full', []);
linear.C_full = localGetStructField(linearData, 'C_full', []);
linear.D_full = localGetStructField(linearData, 'D_full', []);
linear.B_front_collective = localGetStructField(linearData, 'B_front_collective', []);
linear.B_rear_collective = localGetStructField(linearData, 'B_rear_collective', []);

sys_full = localGetStructField(linearData, 'sys_full', []);
if ~isempty(sys_full)
    linear.full_state_names = string(sys_full.StateName(:));
    linear.full_input_names = string(sys_full.InputName(:));
    linear.full_output_names = string(sys_full.OutputName(:));
else
    linear.full_state_names = strings(0, 1);
    linear.full_input_names = strings(0, 1);
    linear.full_output_names = strings(0, 1);
end

sys9 = localGetStructField(linearData, 'sys_ss_9state', []);
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

function row = localBuildIndexRow(sourceRow, outputFile, latestFile)
row = localLinearizationIndexRowTemplate();
row.key = string(sourceRow.key);
row.name = string(sourceRow.name);
row.tilt_deg = sourceRow.tilt_deg;
row.vinf_mps = sourceRow.vinf_mps;
row.family = string(sourceRow.family);
row.seed_name = string(sourceRow.seed_name);
row.success = localGetTableLogical(sourceRow, 'success', strcmp(string(sourceRow.classification), "exact_trim"));
row.acceptable = localGetTableLogical(sourceRow, 'acceptable', row.success);
row.classification = string(sourceRow.classification);
row.score = localGetTableNumeric(sourceRow, 'score', NaN);
row.max_normalized = localGetTableNumeric(sourceRow, 'max_normalized', NaN);
row.front_collective_rpm = sourceRow.front_collective_rpm;
row.rear_collective_rpm = sourceRow.rear_collective_rpm;
row.delta_f_deg = sourceRow.delta_f_deg;
row.delta_a_deg = sourceRow.delta_a_deg;
row.delta_e_deg = sourceRow.delta_e_deg;
row.delta_r_deg = sourceRow.delta_r_deg;
row.theta_deg = sourceRow.theta_deg;
row.alpha_deg = sourceRow.alpha_deg;
row.output_file = string(outputFile);
row.latest_file = string(latestFile);
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

function localWriteIndexFiles(indexRows, opts)
indexTable = struct2table(indexRows);
linearizationIndex = indexRows; %#ok<NASGU>
save(char(opts.index_mat), 'linearizationIndex', 'indexTable', '-v7.3');
save(char(opts.latest_index_mat), 'linearizationIndex', 'indexTable', '-v7.3');
writetable(indexTable, char(opts.index_csv));
writetable(indexTable, char(opts.latest_index_csv));
end

function localMaybeCheckpoint(backfillResult, indexRows, opts, checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd)
completed = backfillResult.progress.completed;
if completed == 0
    return;
end
if mod(completed, opts.checkpoint_every) == 0
    localFinalizeCheckpoint(backfillResult, indexRows, opts, checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd);
end
end

function [summaryTable, backfillResult] = localFinalizeCheckpoint(backfillResult, indexRows, opts, checkpointMat, latestCheckpointMat, summaryCsv, latestSummaryCsv, summaryMd, latestSummaryMd)
summaryTable = localBuildSummaryTable(backfillResult.entries);
backfillResult.summary_table = summaryTable;
backfillResult.index_csv = string(opts.latest_index_csv);
backfillResult.index_mat = string(opts.latest_index_mat);
transitionTrimLinearizationBackfillResult = backfillResult; %#ok<NASGU>
transitionTrimLinearizationBackfillSummary = summaryTable; %#ok<NASGU>
linearizationIndex = indexRows; %#ok<NASGU>
save(char(checkpointMat), 'transitionTrimLinearizationBackfillResult', 'transitionTrimLinearizationBackfillSummary', 'linearizationIndex', '-v7.3');
save(char(latestCheckpointMat), 'transitionTrimLinearizationBackfillResult', 'transitionTrimLinearizationBackfillSummary', 'linearizationIndex', '-v7.3');
writetable(summaryTable, char(summaryCsv));
writetable(summaryTable, char(latestSummaryCsv));
localWriteSummaryMarkdown(summaryMd, backfillResult, summaryTable);
localWriteSummaryMarkdown(latestSummaryMd, backfillResult, summaryTable);
localWriteIndexFiles(indexRows, opts);
end

function summaryTable = localBuildSummaryTable(entries)
if isempty(entries)
    summaryTable = table();
    return;
end

n = numel(entries);
summaryTable = table( ...
    strings(n, 1), strings(n, 1), strings(n, 1), strings(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
    strings(n, 1), strings(n, 1), strings(n, 1), false(n, 1), ...
    strings(n, 1), strings(n, 1), zeros(n, 1), strings(n, 1), ...
    'VariableNames', { ...
        'name', 'source_file', 'family', 'status', ...
        'tilt_deg', 'vinf_mps', 'front_collective_rpm', 'rear_collective_rpm', ...
        'replay_mode', 'match_mode', 'linearization_latest_file', 'replay_success', ...
        'termination_string', 'error_message', 'max_state_residual', 'row_id'});

for i = 1:n
    entry = entries(i);
    summaryTable.name(i) = string(entry.name);
    summaryTable.source_file(i) = string(entry.source_file);
    summaryTable.family(i) = string(entry.family);
    summaryTable.status(i) = string(entry.status);
    summaryTable.tilt_deg(i) = entry.tilt_deg;
    summaryTable.vinf_mps(i) = entry.vinf_mps;
    summaryTable.front_collective_rpm(i) = entry.front_collective_rpm;
    summaryTable.rear_collective_rpm(i) = entry.rear_collective_rpm;
    summaryTable.replay_mode(i) = string(entry.replay_mode);
    summaryTable.match_mode(i) = string(entry.match_mode);
    summaryTable.linearization_latest_file(i) = string(entry.linearization_latest_file);
    summaryTable.replay_success(i) = entry.replay_success;
    summaryTable.termination_string(i) = string(entry.termination_string);
    summaryTable.error_message(i) = string(entry.error_message);
    summaryTable.max_state_residual(i) = entry.max_state_residual;
    summaryTable.row_id(i) = string(entry.row_id);
end

summaryTable = sortrows(summaryTable, {'status', 'source_file', 'tilt_deg', 'vinf_mps', 'family'});
end

function localWriteSummaryMarkdown(filename, backfillResult, summaryTable)
fid = fopen(char(filename), 'w');
if fid < 0
    warning('TrimDB_BackfillLinearizations:WriteFailed', ...
        'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# Transition Trim Linearization Backfill\n\n');
fprintf(fid, 'Generated: %s\n\n', backfillResult.meta.created_on);
fprintf(fid, '- Candidate rows: `%d`\n', backfillResult.progress.completed);
fprintf(fid, '- Saved: `%d`\n', backfillResult.progress.saved_count);
fprintf(fid, '- Failed: `%d`\n', backfillResult.progress.failed_count);
fprintf(fid, '- Skipped existing: `%d`\n', backfillResult.progress.skipped_count);
fprintf(fid, '- Latest index CSV: `%s`\n\n', backfillResult.index_csv);

if isempty(summaryTable)
    fprintf(fid, 'No rows processed.\n');
    return;
end

fprintf(fid, '## First 40 Rows\n\n');
fprintf(fid, '| name | source | family | tilt | V | status | replay | match | rear rpm |\n');
fprintf(fid, '| --- | --- | --- | ---: | ---: | --- | --- | --- | ---: |\n');
for i = 1:min(40, height(summaryTable))
    row = summaryTable(i, :);
    fprintf(fid, '| %s | %s | %s | %.1f | %.1f | %s | %s | %s | %.3f |\n', ...
        row.name, row.source_file, row.family, row.tilt_deg, row.vinf_mps, ...
        row.status, row.replay_mode, row.match_mode, row.rear_collective_rpm);
end
end

function entry = localResultEntryTemplate()
entry = struct( ...
    'index', 0, ...
    'row_id', "", ...
    'key', "", ...
    'name', "", ...
    'source_file', "", ...
    'source_mat_file', "", ...
    'source_var_name', "", ...
    'family', "", ...
    'classification', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'trim_case_name', "", ...
    'original_linearization_available', false, ...
    'status', "", ...
    'match_mode', "", ...
    'match_detail', "", ...
    'replay_mode', "", ...
    'replay_success', false, ...
    'termination_string', "", ...
    'max_state_residual', NaN, ...
    'linearization_output_file', "", ...
    'linearization_latest_file', "", ...
    'error_identifier', "", ...
    'error_message', "", ...
    'console_text', "");
end

function item = localSourceItemTemplate()
item = struct( ...
    'source_csv_file', "", ...
    'source_mat_file', "", ...
    'source_var_name', "", ...
    'key', "", ...
    'name', "", ...
    'family', "", ...
    'seed_name', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'rear_fixed_rpm', NaN, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'success', false, ...
    'acceptable', false, ...
    'classification', "", ...
    'score', NaN, ...
    'max_normalized', NaN, ...
    'trim_case', struct(), ...
    'trim_result', struct());
end

function status = localSourceMatStatusTemplate()
status = struct( ...
    'source_csv_file', "", ...
    'source_mat_file', "", ...
    'enabled', false, ...
    'status', "", ...
    'item_count', 0, ...
    'detail', "");
end

function tf = localTrimResultHasUsableLinear(trimResult)
tf = false;
if ~isstruct(trimResult) || ~isfield(trimResult, 'linear') || ~isstruct(trimResult.linear)
    return;
end
if ~isfield(trimResult.linear, 'sys_ss_9state') || isempty(trimResult.linear.sys_ss_9state)
    return;
end
tf = true;
end

function tf = localStructHasFields(s)
tf = isstruct(s) && ~isempty(fieldnames(s));
end

function tf = localStructHasAnyField(s, fieldNames)
tf = false;
if ~isstruct(s)
    return;
end
for i = 1:numel(fieldNames)
    if isfield(s, fieldNames{i}) && ~isempty(s.(fieldNames{i}))
        tf = true;
        return;
    end
end
end

function value = localGetStructField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function value = localStringField(s, fieldNames)
value = "";
for i = 1:numel(fieldNames)
    fieldName = fieldNames{i};
    if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
        value = string(s.(fieldName));
        if strlength(value) > 0
            return;
        end
    end
end
end

function value = localNumericField(s, fieldNames)
value = NaN;
for i = 1:numel(fieldNames)
    fieldName = fieldNames{i};
    if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
        try
            value = double(s.(fieldName));
        catch
            value = NaN;
        end
        if ~isempty(value)
            value = value(1);
        end
        if isfinite(value)
            return;
        end
    end
end
value = NaN;
end

function value = localLogicalFieldOrDefault(s, fieldNames, defaultValue)
value = defaultValue;
for i = 1:numel(fieldNames)
    fieldName = fieldNames{i};
    if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
        raw = s.(fieldName);
        if islogical(raw)
            value = raw(1);
            return;
        end
        if isnumeric(raw)
            value = logical(raw(1));
            return;
        end
        str = lower(strtrim(char(string(raw))));
        if any(strcmp(str, {'true', '1', 'yes'}))
            value = true;
            return;
        end
        if any(strcmp(str, {'false', '0', 'no'}))
            value = false;
            return;
        end
    end
end
end

function value = localGetTableNumeric(row, fieldName, defaultValue)
if istable(row) && any(strcmp(row.Properties.VariableNames, fieldName))
    value = row.(fieldName)(1);
else
    value = defaultValue;
end
end

function value = localGetTableString(row, fieldName, defaultValue)
if istable(row) && any(strcmp(row.Properties.VariableNames, fieldName))
    value = string(row.(fieldName)(1));
else
    value = string(defaultValue);
end
end

function value = localGetTableLogical(row, fieldName, defaultValue)
if istable(row) && any(strcmp(row.Properties.VariableNames, fieldName))
    raw = row.(fieldName)(1);
    if islogical(raw)
        value = logical(raw);
    elseif isnumeric(raw)
        value = logical(raw);
    else
        value = defaultValue;
    end
else
    value = defaultValue;
end
end

function value = localGetStructString(s, fieldName, defaultValue)
value = string(defaultValue);
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = string(s.(fieldName));
end
end

function value = localFirstNonEmptyString(varargin)
value = "";
for i = 1:nargin
    candidate = string(varargin{i});
    if strlength(candidate) > 0
        value = candidate;
        return;
    end
end
end

function value = localFirstFinite(varargin)
value = NaN;
for i = 1:nargin
    candidate = varargin{i};
    if ~isempty(candidate) && isfinite(candidate(1))
        value = double(candidate(1));
        return;
    end
end
end

function key = localBuildCanonicalKey(sourceCsvFile, name, tiltDeg, vinfMps, family, seedName, rowIndex)
runPrefix = localRunPrefixFromFilename(sourceCsvFile);
familyLabel = localSanitizeToken(family);
seedLabel = localSanitizeToken(seedName);

if isfinite(tiltDeg) && isfinite(vinfMps)
    key = "tilt_" + localNumberToken(tiltDeg) + ...
        "__vinf_" + localNumberToken(vinfMps) + ...
        "__family_" + familyLabel + ...
        "__source_" + localSanitizeToken(runPrefix);
elseif strlength(string(name)) > 0
    key = localSanitizeToken(runPrefix) + "__name_" + localSanitizeToken(name);
else
    key = localSanitizeToken(runPrefix) + "__row_" + string(rowIndex);
end

if strlength(seedLabel) > 0
    key = key + "__seed_" + seedLabel;
end
end

function prefix = localRunPrefixFromFilename(filename)
filename = string(filename);
[~, stem, ~] = fileparts(char(filename));
prefix = regexprep(stem, '_latest$', '');
end

function token = localSanitizeToken(value)
token = lower(string(value));
token = regexprep(token, '[^a-zA-Z0-9]+', '_');
token = regexprep(token, '^_+|_+$', '');
if strlength(token) == 0
    token = "none";
end
end

function token = localNumberToken(value)
token = string(strrep(num2str(value, '%.12g'), '.', 'p'));
token = strrep(token, '-', 'm');
end

function label = localPreferredFileLabel(row)
label = string(row.name);
if strlength(label) == 0
    label = string(row.key);
end
if strlength(label) == 0
    label = "row";
end
end

function label = localSanitizeFileLabel(label)
label = regexprep(string(label), '[^a-zA-Z0-9_]+', '_');
label = regexprep(label, '_+', '_');
label = regexprep(label, '^_+|_+$', '');
if strlength(label) == 0
    label = "item";
end
end

function value = localFiniteDifference(a, b)
if ~isfinite(a) || ~isfinite(b)
    value = inf;
else
    value = abs(a - b);
end
end

function rowId = localBackfillRowIdFromTableRow(row)
rowId = sprintf('%s|%.8f|%.8f|%s|%.6f|%.6f', ...
    char(string(row.name)), ...
    row.tilt_deg, row.vinf_mps, char(string(row.family)), ...
    localGetTableNumeric(row, 'front_collective_rpm', NaN), ...
    localGetTableNumeric(row, 'rear_collective_rpm', NaN));
end

function rowId = localBackfillRowIdFromIndexRow(row)
rowId = sprintf('%s|%.8f|%.8f|%s|%.6f|%.6f', ...
    char(string(row.name)), row.tilt_deg, row.vinf_mps, char(string(row.family)), ...
    row.front_collective_rpm, row.rear_collective_rpm);
end

function maxResidual = localMaxStateResidual(opReport)
maxResidual = inf;
try
    states = opReport.States;
catch
    return;
end

residuals = [];
for i = 1:numel(states)
    state = states(i);
    try
        dx = state.dx;
    catch
        dx = [];
    end
    if isempty(dx)
        continue;
    end
    dx = dx(:);
    steadyMask = true(size(dx));
    try
        rawSteady = state.SteadyState;
    catch
        rawSteady = [];
    end
    if ~isempty(rawSteady)
        steadyMask = logical(rawSteady(:));
        if numel(steadyMask) ~= numel(dx)
            steadyMask = true(size(dx));
        end
    end
    residuals = [residuals; abs(dx(steadyMask))]; %#ok<AGROW>
end

if ~isempty(residuals)
    maxResidual = max(residuals);
end
end
