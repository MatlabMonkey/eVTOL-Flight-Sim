% Prep_Wrapper_TransitionPathLQR_ScheduledFromDB.m
% Stage Wrapper with DB-backed hover trim, scheduled path LQR, and a smooth
% command schedule that walks from hover toward cruise.
%
% Usage:
%   Prep_Wrapper_TransitionPathLQR_ScheduledFromDB
%
% Optional workspace overrides:
%   transitionPathSchedulePrepOptions = struct( ...
%       'db_file', '<path to trim_linearization_db.mat>', ...
%       'trim_selector', struct('group','hover','name','Hover'), ...
%       'controller_opts', struct(...), ...
%       'schedule_opts', struct(...), ...
%       'run_case', RunCase_PrepOnly());

Init_EVTOL_Main

prepOptions = struct();
if exist('transitionPathSchedulePrepOptions', 'var') && isstruct(transitionPathSchedulePrepOptions)
    prepOptions = transitionPathSchedulePrepOptions;
end

dbFile = localGetStructField(prepOptions, 'db_file', []);
trimSelector = localGetStructField(prepOptions, 'trim_selector', struct('group', 'hover', 'name', 'Hover'));
controllerOpts = localGetStructField(prepOptions, 'controller_opts', struct());
scheduleOpts = localGetStructField(prepOptions, 'schedule_opts', struct());
runCase = localGetStructField(prepOptions, 'run_case', []);

[trimResult, ~, trimDbMeta] = load_trim_result_from_db(dbFile, trimSelector, initData);

transitionPathControllerOptions = struct();
transitionPathControllerOptions.db_file = trimDbMeta.db_file;
transitionPathControllerOptions.controller_opts = controllerOpts;
Build_Controller_From_Transition_Path_LQR

scheduleDemo = make_transition_path_schedule_cmds(controllerData, trimResult, scheduleOpts);

if isempty(runCase)
    runCase = RunCase_PrepOnly();
end
if ~isfield(runCase, 'name') || isempty(runCase.name)
    runCase.name = 'TransitionPathScheduledPrepFromDB';
end
runCase.useController = true;
if ~isfield(runCase, 'attemptSimulation') || isempty(runCase.attemptSimulation)
    runCase.attemptSimulation = false;
end
runCase.stopTime_s = localGetStructField(runCase, 'stopTime_s', scheduleDemo.runSpecHint.stopTime);
runCase.stepTime = scheduleDemo.runSpecHint.stepTime;
runCase.airData_cmd = scheduleDemo.runSpecHint.airData_cmd;
runCase.eul_init = scheduleDemo.runSpecHint.eul_init;
runCase.omega_init = scheduleDemo.runSpecHint.omega_init;
runCase.Motor_RPMs = scheduleDemo.runSpecHint.Motor_RPMs;
runCase.Tilt_angles = scheduleDemo.runSpecHint.Tilt_angles;
runCase.front_collective = scheduleDemo.runSpecHint.front_collective;
runCase.rear_collective = scheduleDemo.runSpecHint.rear_collective;
runCase.surface_init = scheduleDemo.runSpecHint.surface_init;
runCase.cmds_override = scheduleDemo.cmds;

Run_EVTOL_Main
open_system('Wrapper')

fprintf(['Wrapper prepared with scheduled path commands from DB entry %s (%s).\n', ...
         'DB file: %s\n', ...
         'Tilt command: %.1f -> %.1f deg\n', ...
         'Vinf command: %.1f -> %.1f m/s\n', ...
         'Press Run in Wrapper when ready.\n'], ...
    trimDbMeta.name, trimDbMeta.group, trimDbMeta.db_file, ...
    scheduleDemo.cmds.tilt_cmd(1, 2), scheduleDemo.cmds.tilt_cmd(end, 2), ...
    scheduleDemo.cmds.airData_cmd(1, 2), scheduleDemo.cmds.airData_cmd(end, 2));

function value = localGetStructField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end
