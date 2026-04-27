function transitionTrimMasterAttemptDB = TrimDB_UpdateMaster(runSummary, updateOptions)
% TrimDB_UpdateMaster
% Append or replace run-level trim summary rows directly into the canonical
% master attempt DB. This makes the master DB the source of truth for new
% searches; per-run CSVs become debug/review artifacts rather than inputs
% to the canonical workflow.
%
% Usage:
%   TrimDB_UpdateMaster(runSummary)
%
% Optional configuration:
%   updateOptions = struct( ...
%       'root_dir', pwd, ...
%       'database_dir', fullfile(pwd, 'databases'), ...
%       'workspace_plots_dir', fullfile(pwd, 'workspace_plots'), ...
%       'run_prefix', 'transition_trim_midband_guidegrid_scored', ...
%       'run_output_dir', '/abs/path/to/run_dir', ...
%       'source_file', 'transition_trim_midband_guidegrid_scored_direct_db', ...
%       'source_kind', 'direct_run_summary', ...
%       'write_mat_export', true, ...
%       'write_csv_export', true, ...
%       'write_markdown_export', true);

if nargin < 1 || isempty(runSummary)
    runSummary = table();
end
if nargin < 2 || ~isstruct(updateOptions)
    updateOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    defaultRootDir = fileparts(stack(1).file);
else
    defaultRootDir = pwd;
end

opts = localApplyDefaults(updateOptions, defaultRootDir);
existingDb = localLoadExistingDb(opts.master_mat_file, opts.master_csv_file);
existingRows = existingDb.master_attempt_db_all_rows;

incomingRows = localNormalizeRunSummary(runSummary, opts);
if height(incomingRows) == 0
    transitionTrimMasterAttemptDB = existingDb;
    assignin('base', 'transitionTrimMasterAttemptDB', transitionTrimMasterAttemptDB);
    return;
end
allRows = [existingRows; incomingRows]; %#ok<AGROW>
allRows = localEnsureMasterSchema(allRows);
allRows = localDeduplicateMasterRows(allRows);
bestUnique = localBuildBestUniquePoints(allRows);

transitionTrimMasterAttemptDB = struct();
transitionTrimMasterAttemptDB.meta = struct( ...
    'created_on', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')), ...
    'root_dir', opts.root_dir, ...
    'database_dir', opts.database_dir, ...
    'workspace_plots_dir', opts.workspace_plots_dir, ...
    'write_mode', "direct_append", ...
    'options', opts);
transitionTrimMasterAttemptDB.manifest = struct( ...
    'mode', "direct_append", ...
    'trim_sources', repmat(localSourceStatusTemplate(), 0, 1), ...
    'linearization_index_sources', repmat(localSourceStatusTemplate(), 0, 1), ...
    'excluded_by_default', strings(0, 1));
transitionTrimMasterAttemptDB.source_status = localBuildSourceStatus(opts, height(incomingRows));
transitionTrimMasterAttemptDB.linearization_index_status = repmat(localSourceStatusTemplate(), 0, 1);
transitionTrimMasterAttemptDB.linearization_match_summary = struct( ...
    'rows_with_match', nnz(localAsLogicalColumn(allRows, 'linearization_available')), ...
    'rows_with_existing_file', nnz(strlength(string(localTextColumn(allRows, 'linearization_latest_file'))) > 0));
transitionTrimMasterAttemptDB.master_attempt_db_all_rows = allRows;
transitionTrimMasterAttemptDB.master_attempt_db_best_unique_points = bestUnique;
transitionTrimMasterAttemptDB.validation = struct( ...
    'source_row_count', height(allRows), ...
    'best_unique_row_count', height(bestUnique), ...
    'pass', true);

if opts.write_mat_export
    save(opts.master_mat_file, 'transitionTrimMasterAttemptDB', '-v7.3');
end
if opts.write_csv_export
    writetable(allRows, opts.master_csv_file);
end
if opts.write_markdown_export
    localWriteMasterMarkdown(opts.master_md_file, transitionTrimMasterAttemptDB);
end

assignin('base', 'transitionTrimMasterAttemptDB', transitionTrimMasterAttemptDB);
end

function opts = localApplyDefaults(updateOptions, defaultRootDir)
opts = updateOptions;
opts.root_dir = localGetField(opts, 'root_dir', defaultRootDir);
dbPaths = TrimDB_Paths(opts.root_dir);
opts.database_dir = localGetField(opts, 'database_dir', dbPaths.database_dir);
opts.workspace_plots_dir = localGetField(opts, 'workspace_plots_dir', dbPaths.workspace_plots_dir);
opts.run_prefix = string(localGetField(opts, 'run_prefix', "trim_search_run"));
opts.run_output_dir = string(localGetField(opts, 'run_output_dir', opts.workspace_plots_dir));
opts.source_file = string(localGetField(opts, 'source_file', opts.run_prefix + "_direct_db"));
opts.source_kind = string(localGetField(opts, 'source_kind', "direct_run_summary"));
opts.trim_formulation = string(localGetField(opts, 'trim_formulation', ""));
opts.write_mat_export = logical(localGetField(opts, 'write_mat_export', true));
opts.write_csv_export = logical(localGetField(opts, 'write_csv_export', true));
opts.write_markdown_export = logical(localGetField(opts, 'write_markdown_export', true));
opts.master_mat_file = string(localGetField(opts, 'master_mat_file', fullfile(opts.database_dir, 'trim_attempts.mat')));
opts.master_csv_file = string(localGetField(opts, 'master_csv_file', fullfile(opts.database_dir, 'trim_attempts.csv')));
opts.master_md_file = string(localGetField(opts, 'master_md_file', fullfile(opts.database_dir, 'trim_attempts.md')));

if exist(opts.database_dir, 'dir') ~= 7
    mkdir(opts.database_dir);
end
if exist(opts.workspace_plots_dir, 'dir') ~= 7
    mkdir(opts.workspace_plots_dir);
end
end

function db = localLoadExistingDb(masterMatFile, masterCsvFile)
db = struct();
db.master_attempt_db_all_rows = localEmptyMasterTable();
db.master_attempt_db_best_unique_points = localEmptyMasterTable();

if exist(masterMatFile, 'file') == 2
    raw = load(masterMatFile);
    if isfield(raw, 'transitionTrimMasterAttemptDB')
        src = raw.transitionTrimMasterAttemptDB;
        if isstruct(src) && isfield(src, 'master_attempt_db_all_rows')
            db = src;
            db.master_attempt_db_all_rows = localEnsureMasterSchema(src.master_attempt_db_all_rows);
            if isfield(src, 'master_attempt_db_best_unique_points')
                db.master_attempt_db_best_unique_points = localEnsureMasterSchema(src.master_attempt_db_best_unique_points);
            else
                db.master_attempt_db_best_unique_points = localBuildBestUniquePoints(db.master_attempt_db_all_rows);
            end
            return;
        end
    end
end

if exist(masterCsvFile, 'file') == 2
    tbl = readtable(masterCsvFile, 'TextType', 'string');
    db.master_attempt_db_all_rows = localEnsureMasterSchema(tbl);
    db.master_attempt_db_best_unique_points = localBuildBestUniquePoints(db.master_attempt_db_all_rows);
end
end

function rows = localNormalizeRunSummary(runSummary, opts)
if isempty(runSummary)
    rows = localEmptyMasterTable();
    return;
end

rows = repmat(localEmptyMasterRow(), height(runSummary), 1);
for i = 1:height(runSummary)
    row = localEmptyMasterRow();
    row.source_file = opts.source_file;
    row.source_kind = opts.source_kind;
    row.source_run_prefix = opts.run_prefix;
    row.name = localTableValue(runSummary, 'name', i, "");
    row.key = localTableValue(runSummary, 'key', i, "");
    row.tilt_deg = localTableValue(runSummary, 'tilt_deg', i, NaN);
    row.vinf_mps = localTableValue(runSummary, 'vinf_mps', i, NaN);
    row.alpha_target_deg = localTableValue(runSummary, 'alpha_target_deg', i, NaN);
    row.trim_formulation = localTableValue(runSummary, 'trim_formulation', i, opts.trim_formulation);
    if strlength(string(row.trim_formulation)) == 0
        row.trim_formulation = opts.trim_formulation;
    end
    row.family = localTableValue(runSummary, 'family', i, "");
    row.seed_name = localTableValue(runSummary, 'seed_name', i, "");
    row.success = logical(localTableValue(runSummary, 'success', i, false));
    row.acceptable = logical(localTableValue(runSummary, 'acceptable', i, row.success));
    row.classification = localTableValue(runSummary, 'classification', i, localDefaultClassification(row.success, row.acceptable));
    row.score = localTableValue(runSummary, 'score', i, NaN);
    row.max_normalized = localTableValue(runSummary, 'max_normalized', i, NaN);
    row.worst_component = localTableValue(runSummary, 'worst_component', i, "");
    row.worst_component_normalized = localTableValue(runSummary, 'worst_component_normalized', i, NaN);
    row.front_collective_rpm = localTableValue(runSummary, 'front_collective_rpm', i, NaN);
    row.rear_collective_rpm = localTableValue(runSummary, 'rear_collective_rpm', i, NaN);
    row.delta_f_deg = localTableValue(runSummary, 'delta_f_deg', i, NaN);
    row.delta_a_deg = localTableValue(runSummary, 'delta_a_deg', i, NaN);
    row.delta_e_deg = localTableValue(runSummary, 'delta_e_deg', i, NaN);
    row.delta_r_deg = localTableValue(runSummary, 'delta_r_deg', i, NaN);
    row.theta_deg = localTableValue(runSummary, 'theta_deg', i, NaN);
    row.u_mps = localTableValue(runSummary, 'u_mps', i, NaN);
    row.w_mps = localTableValue(runSummary, 'w_mps', i, NaN);
    row.alpha_deg = localTableValue(runSummary, 'alpha_deg', i, NaN);
    row.gamma_deg = localTableValue(runSummary, 'gamma_deg', i, NaN);
    row.termination_string = localTableValue(runSummary, 'termination_string', i, "");
    row.max_state_residual = localTableValue(runSummary, 'max_state_residual', i, NaN);
    row.attempt_count = localTableValue(runSummary, 'attempt_count', i, NaN);
    row.rear_on_ok = isfinite(row.rear_collective_rpm) && row.rear_collective_rpm > 1;
    row.linearization_available = logical(localTableValue(runSummary, 'linearization_available', i, false));
    row.linearization_index_source = localTableValue(runSummary, 'linearization_index_source', i, "");
    row.linearization_latest_file = localTableValue(runSummary, 'linearization_latest_file', i, "");
    rows(i, 1) = row;
end
rows = struct2table(rows);
rows = localEnsureMasterSchema(rows);
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
    'alpha_target_deg', NaN, ...
    'trim_formulation', "", ...
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
    'gamma_deg', NaN, ...
    'termination_string', "", ...
    'max_state_residual', NaN, ...
    'attempt_count', NaN, ...
    'rear_on_ok', false, ...
    'linearization_available', false, ...
    'linearization_index_source', "", ...
    'linearization_latest_file', "");
end

function tbl = localEmptyMasterTable()
tbl = struct2table(localEmptyMasterRow());
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

function tbl = localDeduplicateMasterRows(tbl)
if isempty(tbl)
    return;
end
ids = string(tbl.source_run_prefix) + "|" + string(localTextColumn(tbl, 'source_file')) + "|" + ...
    string(localTextColumn(tbl, 'key'));
[~, keepIdx] = unique(flipud(ids), 'stable');
keepIdx = height(tbl) - keepIdx + 1;
tbl = tbl(sort(keepIdx), :);
end

function bestUnique = localBuildBestUniquePoints(masterRows)
if isempty(masterRows)
    bestUnique = localEmptyMasterTable();
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
if ismember('rear_fixed_rpm', row.Properties.VariableNames) && isfinite(row.rear_fixed_rpm)
    rearFixedToken = "rearfixed_" + localNumberToken(row.rear_fixed_rpm);
end
key = "tilt_" + localNumberToken(row.tilt_deg) + ...
    "__vinf_" + localNumberToken(row.vinf_mps) + ...
    "__alpha_" + localNumberToken(localTableValue(row, 'alpha_target_deg', 1, NaN)) + ...
    "__form_" + localSanitizeToken(localTableValue(row, 'trim_formulation', 1, "")) + ...
    "__family_" + localSanitizeToken(row.family) + ...
    "__" + rearFixedToken;
end

function status = localBuildSourceStatus(opts, rowCount)
status = localSourceStatusTemplate();
status.filename = opts.source_file;
status.path = string(fullfile(opts.workspace_plots_dir, opts.source_file));
status.enabled = true;
status.is_smoke = contains(lower(opts.source_file), "smoke");
status.reason = "direct canonical append from generic trim runner";
status.run_prefix = opts.run_prefix;
status.source_kind = opts.source_kind;
status.status = "updated";
status.row_count = rowCount;
end

function status = localSourceStatusTemplate()
status = struct( ...
    'filename', "", ...
    'path', "", ...
    'enabled', false, ...
    'is_smoke', false, ...
    'reason', "", ...
    'run_prefix', "", ...
    'source_kind', "", ...
    'status', "", ...
    'row_count', 0);
end

function localWriteMasterMarkdown(filename, db)
fid = fopen(filename, 'w');
if fid < 0
    warning('TrimDB_UpdateMaster:WriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

allRows = db.master_attempt_db_all_rows;
bestRows = db.master_attempt_db_best_unique_points;
fprintf(fid, '# Transition Trim Master Attempt DB\n\n');
fprintf(fid, '- Created: `%s`\n', db.meta.created_on);
fprintf(fid, '- Write mode: `%s`\n', db.meta.write_mode);
fprintf(fid, '- All rows: `%d`\n', height(allRows));
fprintf(fid, '- Best unique points: `%d`\n\n', height(bestRows));

fprintf(fid, '## Latest Direct Update\n\n');
fprintf(fid, '- Run prefix: `%s`\n', db.meta.options.run_prefix);
fprintf(fid, '- Run output dir: `%s`\n', db.meta.options.run_output_dir);
fprintf(fid, '- Source file label: `%s`\n\n', db.meta.options.source_file);

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
end
end

function value = localTableValue(tbl, name, idx, fallback)
value = fallback;
if ~ismember(name, tbl.Properties.VariableNames)
    return;
end
raw = tbl.(name)(idx, :);
if isstring(raw)
    if strlength(raw) == 0 || ismissing(raw)
        return;
    end
    value = raw;
elseif iscell(raw)
    if isempty(raw)
        return;
    end
    value = raw{1};
elseif isnumeric(raw) || islogical(raw)
    if any(isnan(double(raw)), 'all')
        return;
    end
    value = raw;
else
    value = raw;
end
end

function out = localAsLogicalColumn(tbl, name)
if isempty(tbl) || ~ismember(name, tbl.Properties.VariableNames)
    out = false(height(tbl), 1);
    return;
end
col = tbl.(name);
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

function out = localTextColumn(tbl, name)
if isempty(tbl) || ~ismember(name, tbl.Properties.VariableNames)
    out = strings(height(tbl), 1);
    return;
end
out = string(tbl.(name));
end

function classification = localDefaultClassification(success, acceptable)
if success
    classification = "exact_trim";
elseif acceptable
    classification = "quasi_trim_usable";
else
    classification = "not_usable";
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

function label = localNumberToken(value)
label = strrep(num2str(value, '%.10g'), '.', 'p');
label = strrep(label, '-', 'm');
end

function token = localSanitizeToken(value)
token = regexprep(lower(string(value)), '[^a-z0-9]+', '_');
token = regexprep(token, '^_+|_+$', '');
if strlength(token) == 0
    token = "unknown";
end
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
end
end
