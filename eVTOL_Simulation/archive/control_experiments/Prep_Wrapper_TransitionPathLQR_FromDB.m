% Prep_Wrapper_TransitionPathLQR_FromDB.m
% Stage Wrapper using a saved trim point and scheduled path controller.
%
% Usage:
%   Prep_Wrapper_TransitionPathLQR_FromDB
%
% Optional workspace overrides:
%   transitionPathPrepOptions = struct( ...
%       'db_file', '<path to trim_linearization_db.mat>', ...
%       'trim_selector', struct('group','hover','name','Hover'), ...
%       'controller_opts', struct(...), ...
%       'run_case', RunCase_PrepOnly());

Init_EVTOL_Main

prepOptions = struct();
if exist('transitionPathPrepOptions', 'var') && isstruct(transitionPathPrepOptions)
    prepOptions = transitionPathPrepOptions;
end

dbFile = localGetStructField(prepOptions, 'db_file', []);
trimSelector = localGetStructField(prepOptions, 'trim_selector', struct('group', 'hover', 'name', 'Hover'));
controllerOpts = localGetStructField(prepOptions, 'controller_opts', struct());
runCase = localGetStructField(prepOptions, 'run_case', []);

[trimResult, trimDbEntry, trimDbMeta] = load_trim_result_from_db(dbFile, trimSelector, initData); %#ok<ASGLU>

transitionPathControllerOptions = struct();
transitionPathControllerOptions.db_file = trimDbMeta.db_file;
transitionPathControllerOptions.controller_opts = controllerOpts;
Build_Controller_From_Transition_Path_LQR

if isempty(runCase)
    runCase = RunCase_PrepOnly();
end
runCase.name = 'TransitionPathPrepFromDB';
runCase.useController = true;
runCase.attemptSimulation = false;

Run_EVTOL_Main
open_system('Wrapper')

fprintf(['Wrapper prepared from trim DB entry %s (%s).\n', ...
         'DB file: %s\n', ...
         'Press Run in Wrapper when ready.\n'], ...
    trimDbMeta.name, trimDbMeta.group, trimDbMeta.db_file);

function value = localGetStructField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end
