% Check_LowSpeed_Scored_Run_Status.m
% Quick status readout for the latest low-speed scored sweep outputs.
%
% Optional workspace inputs:
%   lowSpeedScoredStatusCsvFile - absolute path to a scored summary CSV
%   lowSpeedScoredStatusMatFile - absolute path to a scored sweep .mat file
%
% Outputs left in base workspace:
%   lowSpeedScoredRunStatus

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('lowSpeedScoredStatusCsvFile', 'var') && ...
        (ischar(lowSpeedScoredStatusCsvFile) || isstring(lowSpeedScoredStatusCsvFile))
    csvFile = char(lowSpeedScoredStatusCsvFile);
else
    csvFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv');
end

if exist('lowSpeedScoredStatusMatFile', 'var') && ...
        (ischar(lowSpeedScoredStatusMatFile) || isstring(lowSpeedScoredStatusMatFile))
    matFile = char(lowSpeedScoredStatusMatFile);
else
    matFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.mat');
end

if exist(csvFile, 'file') ~= 2
    error('Check_LowSpeed_Scored_Run_Status:MissingCsv', ...
        'Missing scored summary CSV: %s', csvFile);
end

tbl = readtable(csvFile, 'TextType', 'string');

lowSpeedScoredRunStatus = struct();
lowSpeedScoredRunStatus.csv_file = csvFile;
lowSpeedScoredRunStatus.mat_file = matFile;
lowSpeedScoredRunStatus.row_count = height(tbl);
lowSpeedScoredRunStatus.csv_modified = string(datetime(dir(csvFile).datenum, 'ConvertFrom', 'datenum'));

if ~isempty(tbl)
    lowSpeedScoredRunStatus.last_point_name = tbl.name(end);
    lowSpeedScoredRunStatus.exact_count = nnz(localAsLogical(tbl.success));
    lowSpeedScoredRunStatus.acceptable_count = nnz(localAsLogical(tbl.acceptable));
else
    lowSpeedScoredRunStatus.last_point_name = "";
    lowSpeedScoredRunStatus.exact_count = 0;
    lowSpeedScoredRunStatus.acceptable_count = 0;
end

lowSpeedScoredRunStatus.target_count = NaN;
lowSpeedScoredRunStatus.completed_count = NaN;
lowSpeedScoredRunStatus.is_complete = false;
lowSpeedScoredRunStatus.progress_fraction = NaN;

if exist(matFile, 'file') == 2
    try
        S = load(matFile);
        if isfield(S, 'mapStruct')
            mapStruct = S.mapStruct;
        elseif isfield(S, 'transitionTrimLowSpeedScoredMap')
            mapStruct = S.transitionTrimLowSpeedScoredMap;
        else
            mapStruct = struct();
        end

        if isfield(mapStruct, 'targets')
            lowSpeedScoredRunStatus.target_count = numel(mapStruct.targets);
        end
        if isfield(mapStruct, 'progress') && isstruct(mapStruct.progress)
            lowSpeedScoredRunStatus.completed_count = localGetField(mapStruct.progress, 'completed', NaN);
        end
        if isfinite(lowSpeedScoredRunStatus.target_count) && isfinite(lowSpeedScoredRunStatus.completed_count)
            lowSpeedScoredRunStatus.is_complete = ...
                lowSpeedScoredRunStatus.completed_count >= lowSpeedScoredRunStatus.target_count;
            lowSpeedScoredRunStatus.progress_fraction = ...
                lowSpeedScoredRunStatus.completed_count / max(lowSpeedScoredRunStatus.target_count, 1);
        end
    catch ME
        lowSpeedScoredRunStatus.mat_read_error = string(ME.message);
    end
end

fprintf('Low-speed scored sweep status\n');
fprintf('  csv rows = %d\n', lowSpeedScoredRunStatus.row_count);
fprintf('  exact trims = %d\n', lowSpeedScoredRunStatus.exact_count);
fprintf('  acceptable points = %d\n', lowSpeedScoredRunStatus.acceptable_count);
fprintf('  last point = %s\n', lowSpeedScoredRunStatus.last_point_name);
if isfinite(lowSpeedScoredRunStatus.target_count)
    fprintf('  progress = %d / %d\n', ...
        lowSpeedScoredRunStatus.completed_count, lowSpeedScoredRunStatus.target_count);
    fprintf('  complete = %d\n', lowSpeedScoredRunStatus.is_complete);
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
