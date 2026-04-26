% Prep_Wrapper_TransitionPathLQR_BabyTransitionFromDB.m
% Stage Wrapper for a deliberately tiny early transition using the saved
% hover-to-cruise path. This is meant to be much gentler than the full
% 20-point schedule and is a better first sanity check.
%
% Usage:
%   Prep_Wrapper_TransitionPathLQR_BabyTransitionFromDB
%
% Optional workspace overrides:
%   babyTransitionPrepOptions = struct( ...
%       'db_file', '<path to trim_linearization_db.mat>', ...
%       'trim_selector', struct('group','hover','name','Hover'), ...
%       'controller_opts', struct(...), ...
%       'schedule_opts', struct(...), ...
%       'run_case', RunCase_PrepOnly());

Init_EVTOL_Main

prepOptions = struct();
if exist('babyTransitionPrepOptions', 'var') && isstruct(babyTransitionPrepOptions)
    prepOptions = babyTransitionPrepOptions;
end

dbFile = localGetStructField(prepOptions, 'db_file', []);
trimSelector = localGetStructField(prepOptions, 'trim_selector', struct('group', 'hover', 'name', 'Hover'));
controllerOpts = localGetStructField(prepOptions, 'controller_opts', struct());
scheduleOpts = localGetStructField(prepOptions, 'schedule_opts', struct());
runCase = localGetStructField(prepOptions, 'run_case', []);

defaultScheduleOpts = struct( ...
    'stop_time_s', 20.0, ...
    'hold_start_s', 3.0, ...
    'hold_end_s', 3.0, ...
    'path_point_indices', [1 2 3], ...
    'end_progress', 0.20);
scheduleOpts = localMergeStruct(defaultScheduleOpts, scheduleOpts);

transitionPathSchedulePrepOptions = struct();
transitionPathSchedulePrepOptions.db_file = dbFile;
transitionPathSchedulePrepOptions.trim_selector = trimSelector;
transitionPathSchedulePrepOptions.controller_opts = controllerOpts;
transitionPathSchedulePrepOptions.schedule_opts = scheduleOpts;
transitionPathSchedulePrepOptions.run_case = runCase;

Prep_Wrapper_TransitionPathLQR_ScheduledFromDB

fprintf(['Baby-transition schedule uses path points [%s] and ends at %.0f%% of that subset.\n', ...
         'This should be much gentler than the full hover-to-cruise sweep.\n'], ...
    localFormatIndexList(scheduleOpts.path_point_indices), 100.0 * scheduleOpts.end_progress);

function value = localGetStructField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end

function out = localMergeStruct(defaults, overrides)
out = defaults;
if ~isstruct(overrides)
    return;
end
names = fieldnames(overrides);
for i = 1:numel(names)
    out.(names{i}) = overrides.(names{i});
end
end

function txt = localFormatIndexList(values)
values = values(:).';
parts = strings(1, numel(values));
for i = 1:numel(values)
    parts(i) = string(values(i));
end
txt = strjoin(cellstr(parts), ', ');
end
