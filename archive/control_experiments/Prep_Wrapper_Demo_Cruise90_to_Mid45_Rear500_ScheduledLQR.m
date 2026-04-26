% Prep_Wrapper_Demo_Cruise90_to_Mid45_Rear500_ScheduledLQR.m
% Stage Wrapper for the first transition demo:
%   exact point A: 90 deg tilt, 75 m/s, rear 500 rpm
%   exact point B: 45 deg tilt, 75 m/s, rear 500 rpm
%
% This script uses a cached exact-trim pair plus a two-point scheduled
% longitudinal LQR so the active MATLAB session does not need to rerun
% findop. After running this script, press Run in Wrapper.

Init_EVTOL_Main

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

cache_file = fullfile(root_dir, '.workflow_cache', 'Demo_Cruise90_to_Mid45_Rear500_ScheduledLQR.mat');
if exist(cache_file, 'file') ~= 2
    error(['Cached demo file not found: %s\n', ...
           'Ask me to rebuild the transition demo cache in a clean MATLAB process.'], cache_file);
end

cache_data = load(cache_file, 'trimResult_initial', 'trimResult_target', 'controllerData'); %#ok<NASGU>
trimResult = cache_data.trimResult_initial;
controllerData = cache_data.controllerData;

scheduleOpts = struct();
scheduleOpts.stop_time_s = 18.0;
scheduleOpts.hold_start_s = 2.0;
scheduleOpts.hold_end_s = 2.0;
scheduleOpts.step_time_s = initData.timing.stepTime;

scheduleDemo = make_transition_path_schedule_cmds(controllerData, trimResult, scheduleOpts);
assignin('base', 'transitionDemoSchedule', scheduleDemo);

runCase = RunCase_PrepOnly();
runCase.name = 'Demo_Cruise90_to_Mid45_Rear500_ScheduledLQR';
runCase.useController = true;
runCase.attemptSimulation = false;
runCase.stopTime_s = scheduleDemo.runSpecHint.stopTime;
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
if usejava('desktop')
    open_system('Wrapper')
else
    load_system('Wrapper')
end

fprintf(['Wrapper prepared for the 90-deg to 45-deg transition demo.\n', ...
         '  initial trim: %s\n', ...
         '  target trim : %s\n', ...
         '  tilt command: %.1f -> %.1f deg\n', ...
         '  Vinf command: %.1f -> %.1f m/s\n', ...
         'Press Run in Wrapper, then use:\n', ...
         '  plot_outputs(''important'')\n'], ...
    char(string(cache_data.trimResult_initial.name)), ...
    char(string(cache_data.trimResult_target.name)), ...
    scheduleDemo.cmds.tilt_cmd(1, 2), scheduleDemo.cmds.tilt_cmd(end, 2), ...
    scheduleDemo.cmds.airData_cmd(1, 2), scheduleDemo.cmds.airData_cmd(end, 2));
