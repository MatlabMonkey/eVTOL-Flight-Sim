% Run_Linearize_Trim_Map.m
% Replay saved trim-map points and capture their linearized models.
%
% Expected usage:
%   Run_Linearize_Trim_Map
%
% Optional configuration:
%   trimMapLinearizeOptions = struct( ...
%       'map_file', fullfile(pwd, 'workspace_plots', 'transition_trim_map_latest.mat'), ...
%       'success_only', true, ...
%       'acceptable_only', false, ...
%       'include_full_order', false, ...
%       'limit', 25);
%   Run_Linearize_Trim_Map
%
% Outputs left in the base workspace:
%   - trimMapLinearizationResult
%   - trimMapLinearizationSummary
%   - trimMapLinearizationOutputDir

if ~exist('trimMapLinearizeOptions', 'var') || ~isstruct(trimMapLinearizeOptions)
    trimMapLinearizeOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
trimMapLinearizeOptions = localApplyDefaultOptions(trimMapLinearizeOptions, root_dir, timestamp);

initOptions = struct();
initOptions.trimMapLinearizeOptions = trimMapLinearizeOptions;
Init_EVTOL_Main
if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'trimMapLinearizeOptions')
    trimMapLinearizeOptions = initOptions.trimMapLinearizeOptions;
end

[mapStruct, mapVarName] = localLoadMapStruct(trimMapLinearizeOptions.map_file);
selectedEntries = localSelectEntries(mapStruct.entries, trimMapLinearizeOptions);
nSelected = numel(selectedEntries);

trimMapLinearizationOutputDir = trimMapLinearizeOptions.output_dir;
if exist(trimMapLinearizationOutputDir, 'dir') ~= 7
    mkdir(trimMapLinearizationOutputDir);
end

checkpointFile = fullfile(trimMapLinearizationOutputDir, 'trim_map_linearizations.mat');
summaryCsv = fullfile(trimMapLinearizationOutputDir, 'trim_map_linearizations_summary.csv');
summaryMd = fullfile(trimMapLinearizationOutputDir, 'trim_map_linearizations_summary.md');
latestMat = fullfile(root_dir, 'workspace_plots', 'trim_map_linearizations_latest.mat');
latestCsv = fullfile(root_dir, 'workspace_plots', 'trim_map_linearizations_latest.csv');
latestMd = fullfile(root_dir, 'workspace_plots', 'trim_map_linearizations_latest.md');

fprintf('=== Run_Linearize_Trim_Map ===\n');
fprintf('Source map: %s\n', trimMapLinearizeOptions.map_file);
fprintf('Source map var: %s\n', mapVarName);
fprintf('Selected entries: %d\n', nSelected);
fprintf('Output dir: %s\n', trimMapLinearizationOutputDir);

trimMapLinearizationResult = struct();
trimMapLinearizationResult.meta = struct();
trimMapLinearizationResult.meta.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
trimMapLinearizationResult.meta.root_dir = root_dir;
trimMapLinearizationResult.meta.source_map_file = trimMapLinearizeOptions.map_file;
trimMapLinearizationResult.meta.source_map_var = mapVarName;
trimMapLinearizationResult.meta.trim_model = initData.modelNames.trim;
trimMapLinearizationResult.meta.run_model = initData.modelNames.run;
trimMapLinearizationResult.meta.options = trimMapLinearizeOptions;
trimMapLinearizationResult.meta.selected_count = nSelected;
trimMapLinearizationResult.progress = struct('completed', 0, 'success_count', 0, 'failure_count', 0);
trimMapLinearizationResult.entries = repmat(localResultEntryTemplate(), 0, 1);

trimMapLinearizationSummary = table();

for iEntry = 1:nSelected
    sourceEntry = selectedEntries(iEntry);
    trimCase = localExtractTrimCase(sourceEntry);

    if trimMapLinearizeOptions.disable_nonlinear_hold
        trimCase.validate_nonlinear_hold = false;
    end

    fprintf('\n[%d/%d] %s | family=%s | tilt=%.1f | V=%.1f\n', ...
        iEntry, nSelected, localGetEntryField(sourceEntry, 'name', ''), ...
        localGetEntryField(sourceEntry, 'family', ''), ...
        localGetEntryField(sourceEntry, 'target_tilt_deg', localGetEntryField(sourceEntry, 'tilt_deg', NaN)), ...
        localGetEntryField(sourceEntry, 'target_vinf_mps', localGetEntryField(sourceEntry, 'vinf_mps', NaN)));

    resultEntry = localResultEntryTemplate();
    resultEntry.index = iEntry;
    resultEntry.key = localGetEntryField(sourceEntry, 'key', '');
    resultEntry.name = localGetEntryField(sourceEntry, 'name', '');
    resultEntry.phase = localGetEntryField(sourceEntry, 'phase', '');
    resultEntry.family = localGetEntryField(sourceEntry, 'family', '');
    resultEntry.source_success = logical(localGetEntryField(sourceEntry, 'success', false));
    resultEntry.source_map_file = trimMapLinearizeOptions.map_file;
    resultEntry.trim_case = trimCase;
    resultEntry.target_tilt_deg = localGetEntryField(sourceEntry, 'target_tilt_deg', localGetEntryField(sourceEntry, 'tilt_deg', NaN));
    resultEntry.target_vinf_mps = localGetEntryField(sourceEntry, 'target_vinf_mps', localGetEntryField(sourceEntry, 'vinf_mps', NaN));
    resultEntry.target_rear_fixed_rpm = localGetEntryField(sourceEntry, 'target_rear_fixed_rpm', NaN);

    try
        options_i = struct( ...
            'verbose', false, ...
            'debug', false, ...
            'emitSummary', false, ...
            'emitLinearSummary', false);
        console_text = evalc('[trimResult, trimSpec] = trim_evtol_case(initData, trimCase, options_i);'); %#ok<NASGU>

        resultEntry.console_text = console_text;
        resultEntry.replay_success = logical(trimResult.success);
        resultEntry.termination_string = trimResult.terminationString;
        resultEntry.max_state_residual = localMaxStateResidual(trimResult.op_report);
        resultEntry.trim_spec = trimSpec;
        resultEntry.trim_summary = localCompactTrimSummary(trimResult);
        resultEntry.linear = localPackLinearData(trimResult, trimMapLinearizeOptions.include_full_order);

        fprintf('  replay success = %d | maxResidual = %.6g\n', ...
            resultEntry.replay_success, resultEntry.max_state_residual);
    catch ME
        resultEntry.replay_success = false;
        resultEntry.had_exception = true;
        resultEntry.termination_string = sprintf('EXCEPTION: %s', ME.message);
        resultEntry.error_identifier = ME.identifier;
        resultEntry.error_message = ME.message;
        fprintf('  FAILED: %s\n', ME.message);
    end

    trimMapLinearizationResult.entries(end + 1, 1) = resultEntry; %#ok<SAGROW>
    trimMapLinearizationResult.progress.completed = numel(trimMapLinearizationResult.entries);
    trimMapLinearizationResult.progress.success_count = nnz([trimMapLinearizationResult.entries.replay_success]);
    trimMapLinearizationResult.progress.failure_count = ...
        trimMapLinearizationResult.progress.completed - trimMapLinearizationResult.progress.success_count;

    if mod(iEntry, trimMapLinearizeOptions.checkpoint_every) == 0 || iEntry == nSelected
        [trimMapLinearizationSummary, trimMapLinearizationResult] = localCheckpoint( ...
            trimMapLinearizationResult, checkpointFile, latestMat, ...
            summaryCsv, latestCsv, summaryMd, latestMd);
        fprintf('  checkpoint saved.\n');
    end
end

[trimMapLinearizationSummary, trimMapLinearizationResult] = localCheckpoint( ...
    trimMapLinearizationResult, checkpointFile, latestMat, ...
    summaryCsv, latestCsv, summaryMd, latestMd);

fprintf('\nCompleted trim-map linearization replay.\n');
fprintf('  replay successes = %d / %d\n', nnz(trimMapLinearizationSummary.replay_success), height(trimMapLinearizationSummary));
fprintf('  checkpoint = %s\n', checkpointFile);

function options = localApplyDefaultOptions(options, root_dir, timestamp)
default_map = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_latest.mat');
if ~isfield(options, 'map_file') || isempty(options.map_file)
    options.map_file = default_map;
end
if ~isfield(options, 'success_only') || isempty(options.success_only)
    options.success_only = true;
end
if ~isfield(options, 'acceptable_only') || isempty(options.acceptable_only)
    options.acceptable_only = false;
end
if ~isfield(options, 'include_full_order') || isempty(options.include_full_order)
    options.include_full_order = false;
end
if ~isfield(options, 'disable_nonlinear_hold') || isempty(options.disable_nonlinear_hold)
    options.disable_nonlinear_hold = true;
end
if ~isfield(options, 'checkpoint_every') || isempty(options.checkpoint_every)
    options.checkpoint_every = 10;
end
if ~isfield(options, 'limit') || isempty(options.limit)
    options.limit = inf;
end
if ~isfield(options, 'output_dir') || isempty(options.output_dir)
    options.output_dir = fullfile(root_dir, 'workspace_plots', ['trim_map_linearizations_' timestamp]);
end
end

function [mapStruct, mapVarName] = localLoadMapStruct(map_file)
if exist(map_file, 'file') ~= 2
    error('Map file not found: %s', map_file);
end

data = load(map_file);
vars = fieldnames(data);

for i = 1:numel(vars)
    value = data.(vars{i});
    if isstruct(value) && isfield(value, 'entries')
        mapStruct = value;
        mapVarName = vars{i};
        return;
    end
end

error('Could not find a map struct with an entries field in %s.', map_file);
end

function entriesOut = localSelectEntries(entriesIn, options)
if isempty(entriesIn)
    entriesOut = repmat(localEmptyEntryTemplate(), 0, 1);
    return;
end

entriesOut = repmat(entriesIn(1), 0, 1);

for i = 1:numel(entriesIn)
    entry = entriesIn(i);
    if options.success_only && ~logical(localGetEntryField(entry, 'success', false))
        continue;
    end
    if options.acceptable_only
        if ~isfield(entry, 'acceptable') || ~logical(entry.acceptable)
            continue;
        end
    end
    if ~localEntryHasTrimCase(entry)
        continue;
    end
    entriesOut(end + 1, 1) = entry; %#ok<SAGROW>
end

if isfinite(options.limit)
    entriesOut = entriesOut(1:min(numel(entriesOut), max(1, round(options.limit))));
end
end

function tf = localEntryHasTrimCase(entry)
tf = (isfield(entry, 'trim_case') && isstruct(entry.trim_case) && ~isempty(fieldnames(entry.trim_case))) || ...
     (isfield(entry, 'trimCase') && isstruct(entry.trimCase) && ~isempty(fieldnames(entry.trimCase)));
end

function trimCase = localExtractTrimCase(entry)
if isfield(entry, 'trim_case') && isstruct(entry.trim_case) && ~isempty(fieldnames(entry.trim_case))
    trimCase = entry.trim_case;
    return;
end
if isfield(entry, 'trimCase') && isstruct(entry.trimCase) && ~isempty(fieldnames(entry.trimCase))
    trimCase = entry.trimCase;
    return;
end
error('Entry does not contain a usable trim_case / trimCase struct.');
end

function linearData = localPackLinearData(trimResult, include_full_order)
linearData = struct();
linearData.replay_exact_trim = logical(localGetField(trimResult, 'success', false));

if ~isfield(trimResult, 'linear') || ~isstruct(trimResult.linear) || ...
        ~isfield(trimResult.linear, 'sys_ss_9state') || isempty(trimResult.linear.sys_ss_9state)
    linearData.reduced_model_available = false;
    return;
end

sys9 = trimResult.linear.sys_ss_9state;
linearData.reduced_model_available = true;
linearData.A_9 = sys9.A;
linearData.B_9 = sys9.B;
linearData.C_9 = sys9.C;
linearData.D_9 = sys9.D;
linearData.state_names_9 = cellstr(sys9.StateName);
linearData.input_names_9 = cellstr(sys9.InputName);
linearData.eigs_9 = eig(sys9.A);
linearData.front_collective_column_9 = localGetField(trimResult.linear, 'B_front_collective', []);
linearData.rear_collective_column_9 = localGetField(trimResult.linear, 'B_rear_collective', []);

if include_full_order
    linearData.A_full = localGetField(trimResult.linear, 'A_full', []);
    linearData.B_full = localGetField(trimResult.linear, 'B_full', []);
    linearData.C_full = localGetField(trimResult.linear, 'C_full', []);
    linearData.D_full = localGetField(trimResult.linear, 'D_full', []);
end
end

function summary = localCompactTrimSummary(trimResult)
summary = struct();
summary.name = localGetField(trimResult, 'name', '');
summary.mode = localGetField(trimResult, 'mode', '');
summary.success = logical(localGetField(trimResult, 'success', false));
summary.termination_string = localGetField(trimResult, 'terminationString', '');
summary.front_tilt_deg = localGetNestedField(trimResult, {'scheduling', 'front_tilt_deg'}, NaN);
summary.front_collective_rpm = localGetNestedField(trimResult, {'scheduling', 'front_collective_rpm'}, NaN);
summary.rear_collective_rpm = localGetNestedField(trimResult, {'scheduling', 'rear_collective_rpm'}, NaN);
summary.delta_f_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'delta_f_rad'}, NaN));
summary.delta_e_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'delta_e_rad'}, NaN));
summary.theta_deg = localGetField(localGetField(trimResult, 'Att_Trim_deg', [NaN; NaN; NaN]), 2, NaN);
summary.u_mps = localGetField(localGetField(trimResult, 'Vel_B_BA_Trim', [NaN; NaN; NaN]), 1, NaN);
summary.w_mps = localGetField(localGetField(trimResult, 'Vel_B_BA_Trim', [NaN; NaN; NaN]), 3, NaN);
summary.alpha_deg = rad2deg(localGetNestedField(trimResult, {'scheduling', 'alpha_rad'}, NaN));
summary.max_state_residual = localMaxStateResidual(localGetField(trimResult, 'op_report', struct()));
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
    if isempty(dx)
        continue;
    end

    dx = dx(:);
    steadyMask = true(size(dx));
    try
        rawSteady = s.SteadyState;
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

function [summaryTable, resultStruct] = localCheckpoint(resultStruct, checkpointFile, latestMat, csvFile, latestCsv, mdFile, latestMd)
summaryTable = localBuildSummaryTable(resultStruct.entries);
resultStruct.summary_table = summaryTable;
trimMapLinearizationResult = resultStruct; %#ok<NASGU>
trimMapLinearizationSummary = summaryTable; %#ok<NASGU>
save(checkpointFile, 'trimMapLinearizationResult', 'trimMapLinearizationSummary', '-v7.3');
save(latestMat, 'trimMapLinearizationResult', 'trimMapLinearizationSummary', '-v7.3');
writetable(summaryTable, csvFile);
writetable(summaryTable, latestCsv);
localWriteSummaryMarkdown(mdFile, resultStruct, summaryTable);
localWriteSummaryMarkdown(latestMd, resultStruct, summaryTable);
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
    false(n, 1), false(n, 1), strings(n, 1), zeros(n, 1), ...
    zeros(n, 1), zeros(n, 1), zeros(n, 1), strings(n, 1), ...
    'VariableNames', { ...
        'name', 'phase', 'family', ...
        'tilt_deg', 'vinf_mps', 'rear_fixed_rpm', ...
        'source_success', 'replay_success', 'termination', 'max_state_residual', ...
        'front_collective_rpm', 'rear_collective_rpm', 'theta_deg', 'map_file'});

for i = 1:n
    entry = entries(i);
    summaryTable.name(i) = string(entry.name);
    summaryTable.phase(i) = string(entry.phase);
    summaryTable.family(i) = string(entry.family);
    summaryTable.tilt_deg(i) = entry.target_tilt_deg;
    summaryTable.vinf_mps(i) = entry.target_vinf_mps;
    summaryTable.rear_fixed_rpm(i) = entry.target_rear_fixed_rpm;
    summaryTable.source_success(i) = entry.source_success;
    summaryTable.replay_success(i) = entry.replay_success;
    summaryTable.termination(i) = string(entry.termination_string);
    summaryTable.max_state_residual(i) = entry.max_state_residual;
    summaryTable.front_collective_rpm(i) = localGetField(entry.trim_summary, 'front_collective_rpm', NaN);
    summaryTable.rear_collective_rpm(i) = localGetField(entry.trim_summary, 'rear_collective_rpm', NaN);
    summaryTable.theta_deg(i) = localGetField(entry.trim_summary, 'theta_deg', NaN);
    summaryTable.map_file(i) = string(entry.source_map_file);
end

summaryTable = sortrows(summaryTable, {'replay_success', 'phase', 'family', 'tilt_deg', 'vinf_mps'}, ...
    {'descend', 'ascend', 'ascend', 'ascend', 'ascend'});
end

function localWriteSummaryMarkdown(filename, resultStruct, summaryTable)
fid = fopen(filename, 'w');
if fid < 0
    warning('Run_Linearize_Trim_Map:SummaryWriteFailed', 'Could not open %s for writing.', filename);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '# Trim Map Linearization Replay Summary\n\n');
fprintf(fid, 'Generated: %s\n\n', resultStruct.meta.created_on);
fprintf(fid, '- source map: `%s`\n', resultStruct.meta.source_map_file);
fprintf(fid, '- trim model: `%s`\n', resultStruct.meta.trim_model);
fprintf(fid, '- selected entries: %d\n', height(summaryTable));
fprintf(fid, '- replay successes: %d\n', nnz(summaryTable.replay_success));
fprintf(fid, '- replay failures: %d\n\n', height(summaryTable) - nnz(summaryTable.replay_success));

if isempty(summaryTable)
    fprintf(fid, 'No entries selected.\n');
    return;
end

fprintf(fid, '## First 40 Replay Results\n\n');
fprintf(fid, '| name | phase | family | tilt (deg) | V (m/s) | replay success | front (rpm) | rear (rpm) | theta (deg) | residual |\n');
fprintf(fid, '| --- | --- | --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |\n');

limit = min(40, height(summaryTable));
for i = 1:limit
    row = summaryTable(i, :);
    fprintf(fid, '| %s | %s | %s | %.1f | %.1f | %d | %.3f | %.3f | %.3f | %.6g |\n', ...
        row.name, row.phase, row.family, row.tilt_deg, row.vinf_mps, ...
        row.replay_success, row.front_collective_rpm, row.rear_collective_rpm, ...
        row.theta_deg, row.max_state_residual);
end
end

function entry = localResultEntryTemplate()
entry = struct( ...
    'index', 0, ...
    'key', '', ...
    'name', '', ...
    'phase', '', ...
    'family', '', ...
    'target_tilt_deg', NaN, ...
    'target_vinf_mps', NaN, ...
    'target_rear_fixed_rpm', NaN, ...
    'source_success', false, ...
    'source_map_file', '', ...
    'trim_case', struct(), ...
    'trim_spec', struct(), ...
    'trim_summary', struct(), ...
    'replay_success', false, ...
    'termination_string', '', ...
    'max_state_residual', inf, ...
    'had_exception', false, ...
    'error_identifier', '', ...
    'error_message', '', ...
    'console_text', '', ...
    'linear', struct());
end

function entry = localEmptyEntryTemplate()
entry = struct();
end

function value = localGetEntryField(entry, field_name, default_value)
if isstruct(entry) && isfield(entry, field_name) && ~isempty(entry.(field_name))
    value = entry.(field_name);
else
    value = default_value;
end
end

function value = localGetField(s, field_name, default_value)
if isnumeric(field_name)
    if numel(s) >= field_name
        value = s(field_name);
    else
        value = default_value;
    end
    return;
end

if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function value = localGetNestedField(s, field_path, default_value)
value = s;
for i = 1:numel(field_path)
    key = field_path{i};
    if ~isstruct(value) || ~isfield(value, key) || isempty(value.(key))
        value = default_value;
        return;
    end
    value = value.(key);
end
end
