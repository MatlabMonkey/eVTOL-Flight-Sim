% Check_Bridge_Scored_Run_Status.m
% Quick status readout for the latest bridge scored sweep outputs.
%
% Optional workspace inputs:
%   bridgeScoredStatusCsvFile - absolute path to a scored summary CSV
%   bridgeScoredStatusMatFile - absolute path to a scored sweep .mat file
%
% Outputs left in base workspace:
%   bridgeScoredRunStatus

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('bridgeScoredStatusCsvFile', 'var') && ...
        (ischar(bridgeScoredStatusCsvFile) || isstring(bridgeScoredStatusCsvFile))
    csvFile = char(bridgeScoredStatusCsvFile);
else
    csvFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_bridge_scored_latest.csv');
end

if exist('bridgeScoredStatusMatFile', 'var') && ...
        (ischar(bridgeScoredStatusMatFile) || isstring(bridgeScoredStatusMatFile))
    matFile = char(bridgeScoredStatusMatFile);
else
    matFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_bridge_scored_latest.mat');
end

if exist(csvFile, 'file') ~= 2
    error('Check_Bridge_Scored_Run_Status:MissingCsv', ...
        'Missing bridge scored summary CSV: %s', csvFile);
end

tbl = readtable(csvFile, 'TextType', 'string');

bridgeScoredRunStatus = struct();
bridgeScoredRunStatus.csv_file = csvFile;
bridgeScoredRunStatus.mat_file = matFile;
bridgeScoredRunStatus.row_count = height(tbl);
bridgeScoredRunStatus.csv_modified = string(datetime(dir(csvFile).datenum, 'ConvertFrom', 'datenum'));

if ~isempty(tbl)
    bridgeScoredRunStatus.last_point_name = tbl.name(end);
    bridgeScoredRunStatus.exact_count = nnz(localAsLogical(tbl.success));
    bridgeScoredRunStatus.acceptable_count = nnz(localAsLogical(tbl.acceptable));
else
    bridgeScoredRunStatus.last_point_name = "";
    bridgeScoredRunStatus.exact_count = 0;
    bridgeScoredRunStatus.acceptable_count = 0;
end

bridgeScoredRunStatus.target_count = NaN;
bridgeScoredRunStatus.completed_count = NaN;
bridgeScoredRunStatus.is_complete = false;
bridgeScoredRunStatus.progress_fraction = NaN;

if exist(matFile, 'file') == 2
    try
        S = load(matFile);
        if isfield(S, 'mapStruct')
            mapStruct = S.mapStruct;
        elseif isfield(S, 'transitionTrimBridgeScoredMap')
            mapStruct = S.transitionTrimBridgeScoredMap;
        else
            mapStruct = struct();
        end

        if isfield(mapStruct, 'targets')
            bridgeScoredRunStatus.target_count = numel(mapStruct.targets);
        end
        if isfield(mapStruct, 'progress') && isstruct(mapStruct.progress)
            bridgeScoredRunStatus.completed_count = localGetField(mapStruct.progress, 'completed', NaN);
        end
        if isfinite(bridgeScoredRunStatus.target_count) && isfinite(bridgeScoredRunStatus.completed_count)
            bridgeScoredRunStatus.is_complete = ...
                bridgeScoredRunStatus.completed_count >= bridgeScoredRunStatus.target_count;
            bridgeScoredRunStatus.progress_fraction = ...
                bridgeScoredRunStatus.completed_count / max(bridgeScoredRunStatus.target_count, 1);
        end
    catch ME
        bridgeScoredRunStatus.mat_read_error = string(ME.message);
    end
end

fprintf('Bridge scored sweep status\n');
fprintf('  csv rows = %d\n', bridgeScoredRunStatus.row_count);
fprintf('  exact trims = %d\n', bridgeScoredRunStatus.exact_count);
fprintf('  acceptable points = %d\n', bridgeScoredRunStatus.acceptable_count);
fprintf('  last point = %s\n', bridgeScoredRunStatus.last_point_name);
if isfinite(bridgeScoredRunStatus.target_count)
    fprintf('  progress = %d / %d\n', ...
        bridgeScoredRunStatus.completed_count, bridgeScoredRunStatus.target_count);
    fprintf('  complete = %d\n', bridgeScoredRunStatus.is_complete);
else
    fprintf('  progress = unknown (mat metadata unavailable)\n');
end

function out = localAsLogical(col)
if islogical(col)
    out = col;
elseif isnumeric(col)
    out = col ~= 0;
else
    lowered = lower(strtrim(string(col)));
    out = lowered == "1" | lowered == "true";
end
out = logical(out);
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end
