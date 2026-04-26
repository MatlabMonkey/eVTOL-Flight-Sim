% Check_Path_Scored_Run_Status.m
% Quick status readout for the latest path-following scored sweep outputs.
%
% Optional workspace inputs:
%   pathScoredStatusCsvFile - absolute path to a scored summary CSV
%   pathScoredStatusMatFile - absolute path to a scored sweep .mat file
%
% Outputs left in base workspace:
%   pathScoredRunStatus

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('pathScoredStatusCsvFile', 'var') && ...
        (ischar(pathScoredStatusCsvFile) || isstring(pathScoredStatusCsvFile))
    csvFile = char(pathScoredStatusCsvFile);
else
    csvFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_path_scored_latest.csv');
end

if exist('pathScoredStatusMatFile', 'var') && ...
        (ischar(pathScoredStatusMatFile) || isstring(pathScoredStatusMatFile))
    matFile = char(pathScoredStatusMatFile);
else
    matFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_path_scored_latest.mat');
end

if exist(csvFile, 'file') ~= 2
    error('Check_Path_Scored_Run_Status:MissingCsv', ...
        'Missing path scored summary CSV: %s', csvFile);
end

tbl = readtable(csvFile, 'TextType', 'string');

pathScoredRunStatus = struct();
pathScoredRunStatus.csv_file = csvFile;
pathScoredRunStatus.mat_file = matFile;
pathScoredRunStatus.row_count = height(tbl);
pathScoredRunStatus.csv_modified = string(datetime(dir(csvFile).datenum, 'ConvertFrom', 'datenum'));

if ~isempty(tbl)
    pathScoredRunStatus.last_point_name = tbl.name(end);
    pathScoredRunStatus.exact_count = nnz(localAsLogical(tbl.success));
    pathScoredRunStatus.acceptable_count = nnz(localAsLogical(tbl.acceptable));
else
    pathScoredRunStatus.last_point_name = "";
    pathScoredRunStatus.exact_count = 0;
    pathScoredRunStatus.acceptable_count = 0;
end

pathScoredRunStatus.target_count = NaN;
pathScoredRunStatus.completed_count = NaN;
pathScoredRunStatus.queue_remaining = NaN;
pathScoredRunStatus.is_complete = false;
pathScoredRunStatus.progress_fraction = NaN;

if exist(matFile, 'file') == 2
    try
        S = load(matFile);
        if isfield(S, 'mapStruct')
            mapStruct = S.mapStruct;
        elseif isfield(S, 'transitionTrimPathScoredMap')
            mapStruct = S.transitionTrimPathScoredMap;
        else
            mapStruct = struct();
        end

        if isfield(mapStruct, 'meta') && isstruct(mapStruct.meta)
            pathScoredRunStatus.initial_queue_count = localGetField(mapStruct.meta, 'initial_queue_count', NaN);
        end
        if isfield(mapStruct, 'targets')
            pathScoredRunStatus.queue_remaining = numel(mapStruct.targets);
        end
        if isfield(mapStruct, 'progress') && isstruct(mapStruct.progress)
            pathScoredRunStatus.completed_count = localGetField(mapStruct.progress, 'completed', NaN);
            if ~isfield(pathScoredRunStatus, 'initial_queue_count') || ~isfinite(pathScoredRunStatus.initial_queue_count)
                pathScoredRunStatus.initial_queue_count = localGetField(mapStruct.progress, 'completed', NaN) + numel(mapStruct.targets);
            end
        end
        if isfinite(localGetField(pathScoredRunStatus, 'initial_queue_count', NaN)) && isfinite(pathScoredRunStatus.completed_count)
            pathScoredRunStatus.target_count = pathScoredRunStatus.initial_queue_count;
            pathScoredRunStatus.is_complete = ...
                pathScoredRunStatus.queue_remaining == 0;
            pathScoredRunStatus.progress_fraction = ...
                pathScoredRunStatus.completed_count / max(pathScoredRunStatus.target_count, 1);
        end
    catch ME
        pathScoredRunStatus.mat_read_error = string(ME.message);
    end
end

fprintf('Path scored sweep status\n');
fprintf('  csv rows = %d\n', pathScoredRunStatus.row_count);
fprintf('  exact trims = %d\n', pathScoredRunStatus.exact_count);
fprintf('  acceptable points = %d\n', pathScoredRunStatus.acceptable_count);
fprintf('  last point = %s\n', pathScoredRunStatus.last_point_name);
if isfinite(pathScoredRunStatus.target_count)
    fprintf('  progress = %d / %d\n', ...
        pathScoredRunStatus.completed_count, pathScoredRunStatus.target_count);
    fprintf('  queue remaining = %d\n', pathScoredRunStatus.queue_remaining);
    fprintf('  complete = %d\n', pathScoredRunStatus.is_complete);
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
