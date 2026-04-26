% Replay_Scored_Trim_Point.m
% Replay one scored trim point using the exact saved trimCase from the
% scored sweep .mat checkpoint, instead of reconstructing from the CSV row.
%
% Optional workspace inputs:
%   replayTrimPointMatFile  - absolute path to a scored sweep .mat file
%   replayTrimPointName     - point name, e.g. 'LowSpeedScored_Tilt10_V10'
%   replayTrimPointIndex    - numeric fallback if name is omitted
%
% Outputs left in base workspace:
%   replayedTrimEntry
%   replayedTrimSavedResult
%   replayedTrimResult
%   replayedTrimSavedScore
%   replayedTrimScoreData
%   replayedTrimComparison

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('replayTrimPointMatFile', 'var') && ...
        (ischar(replayTrimPointMatFile) || isstring(replayTrimPointMatFile))
    matFile = char(replayTrimPointMatFile);
else
    matFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.mat');
end

if exist(matFile, 'file') ~= 2
    error('Replay_Scored_Trim_Point:MissingMatFile', ...
        'Missing scored sweep mat file: %s', matFile);
end

S = load(matFile);
if isfield(S, 'mapStruct')
    mapStruct = S.mapStruct;
elseif isfield(S, 'transitionTrimLowSpeedScoredMap')
    mapStruct = S.transitionTrimLowSpeedScoredMap;
else
    error('Replay_Scored_Trim_Point:MissingMapStruct', ...
        'Could not find scored map struct in %s', matFile);
end

if ~isfield(mapStruct, 'entries') || isempty(mapStruct.entries)
    error('Replay_Scored_Trim_Point:EmptyEntries', 'No scored entries found in %s', matFile);
end

if exist('replayTrimPointName', 'var') && ...
        (ischar(replayTrimPointName) || isstring(replayTrimPointName)) && ...
        ~isempty(replayTrimPointName)
    pointName = char(replayTrimPointName);
    idx = find(strcmp({mapStruct.entries.name}, pointName), 1, 'first');
    if isempty(idx)
        error('Replay_Scored_Trim_Point:PointNotFound', ...
            'Could not find scored point named "%s".', pointName);
    end
elseif exist('replayTrimPointIndex', 'var') && isnumeric(replayTrimPointIndex) && isscalar(replayTrimPointIndex)
    idx = replayTrimPointIndex;
    if idx < 1 || idx > numel(mapStruct.entries)
        error('Replay_Scored_Trim_Point:BadIndex', ...
            'Point index %d is out of range 1..%d.', idx, numel(mapStruct.entries));
    end
else
    idx = 1;
end

savedEntries = mapStruct.entries;
replayedTrimEntry = savedEntries(idx, 1);
if ~isstruct(replayedTrimEntry)
    error('Replay_Scored_Trim_Point:EntryNotStruct', ...
        'Saved entry %d is not a struct.', idx);
end
if ~isfield(replayedTrimEntry, 'trimCase') || ~isstruct(replayedTrimEntry.trimCase)
    error('Replay_Scored_Trim_Point:MissingTrimCase', ...
        'Saved entry %s does not contain a usable trimCase.', replayedTrimEntry.name);
end

savedTrimCase = replayedTrimEntry.trimCase;
replayedTrimSavedResult = replayedTrimEntry.trimResult;
if isfield(replayedTrimEntry, 'scoreData')
    replayedTrimSavedScore = replayedTrimEntry.scoreData;
else
    replayedTrimSavedScore = struct();
end

trimOptions = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);
if isfield(mapStruct, 'meta') && isfield(mapStruct.meta, 'config') && ...
        isfield(mapStruct.meta.config, 'trim_options') && isstruct(mapStruct.meta.config.trim_options)
    trimOptions = mapStruct.meta.config.trim_options;
end

initOptions = struct();
initOptions.replaySavedTrimCase = savedTrimCase;
Init_EVTOL_Main
savedTrimCase = initOptions.replaySavedTrimCase;
[replayedTrimResult, ~] = trim_evtol_case(initData, savedTrimCase, trimOptions);

scoreOptions = struct('profile', 'transition', 'hold_horizon_s', 2.0);
if isfield(mapStruct, 'meta') && isfield(mapStruct.meta, 'config') && ...
        isfield(mapStruct.meta.config, 'score_options') && isstruct(mapStruct.meta.config.score_options)
    scoreOptions = mapStruct.meta.config.score_options;
end
replayedTrimScoreData = score_trim_point(replayedTrimResult, scoreOptions);

replayedTrimComparison = struct();
replayedTrimComparison.name = replayedTrimEntry.name;
replayedTrimComparison.family = replayedTrimEntry.family;
replayedTrimComparison.seed_name = replayedTrimEntry.seed_name;
replayedTrimComparison.saved_success = localGetField(replayedTrimSavedResult, 'success', false);
replayedTrimComparison.replay_success = localGetField(replayedTrimResult, 'success', false);
replayedTrimComparison.saved_termination = localGetField(replayedTrimSavedResult, 'terminationString', '');
replayedTrimComparison.replay_termination = localGetField(replayedTrimResult, 'terminationString', '');
replayedTrimComparison.saved_classification = localGetField(replayedTrimSavedScore, 'classification', '');
replayedTrimComparison.replay_classification = localGetField(replayedTrimScoreData, 'classification', '');
replayedTrimComparison.saved_score = localGetField(replayedTrimSavedScore, 'score', NaN);
replayedTrimComparison.replay_score = localGetField(replayedTrimScoreData, 'score', NaN);

fprintf('Replay point: %s\n', replayedTrimComparison.name);
fprintf('  family=%s | seed=%s\n', replayedTrimComparison.family, replayedTrimComparison.seed_name);
fprintf('  saved success=%d | replay success=%d\n', ...
    replayedTrimComparison.saved_success, replayedTrimComparison.replay_success);
fprintf('  saved class=%s | replay class=%s\n', ...
    replayedTrimComparison.saved_classification, replayedTrimComparison.replay_classification);
fprintf('  saved score=%.6g | replay score=%.6g\n', ...
    replayedTrimComparison.saved_score, replayedTrimComparison.replay_score);
fprintf('  saved term=%s\n', replayedTrimComparison.saved_termination);
fprintf('  replay term=%s\n', replayedTrimComparison.replay_termination);

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end
