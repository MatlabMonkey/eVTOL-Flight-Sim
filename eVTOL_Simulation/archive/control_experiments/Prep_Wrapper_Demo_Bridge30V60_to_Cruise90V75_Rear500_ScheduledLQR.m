% Prep_Wrapper_Demo_Bridge30V60_to_Cruise90V75_Rear500_ScheduledLQR.m
% Stage Wrapper for the second transition demo:
%   exact point A: 30 deg tilt, 60 m/s, rear 500 rpm
%   exact point B: 90 deg tilt, 75 m/s, rear 500 rpm
%
% This starts the nonlinear run from the exact bridge-point trim and uses a
% two-point scheduled longitudinal LQR to move toward cruise.

Init_EVTOL_Main

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

cache_file = fullfile(root_dir, '.workflow_cache', ...
    'Demo_Bridge30V60_to_Cruise90V75_Rear500_ScheduledLQR.mat');
if exist(cache_file, 'file') ~= 2
    error(['Cached demo file not found: %s\n', ...
           'Rebuild the bridge-to-cruise demo cache in a clean MATLAB process first.'], cache_file);
end

cache_data = load(cache_file, 'trimResult_initial', 'trimResult_target', 'controllerData');
trimResult = cache_data.trimResult_target;
controllerData = cache_data.controllerData;

scheduleOpts = struct();
scheduleOpts.stop_time_s = 20.0;
scheduleOpts.hold_start_s = 2.0;
scheduleOpts.hold_end_s = 2.0;
scheduleOpts.step_time_s = initData.timing.stepTime;
scheduleOpts.direction = 'cruise_to_hover';

scheduleDemo = make_transition_path_schedule_cmds(controllerData, trimResult, scheduleOpts);
assignin('base', 'transitionDemoSchedule', scheduleDemo);

runCase = RunCase_PrepOnly();
runCase.name = 'Demo_Bridge30V60_to_Cruise90V75_Rear500_ScheduledLQR';
runCase.useController = true;
runCase.attemptSimulation = false;
runCase.stopTime_s = scheduleDemo.runSpecHint.stopTime;
runCase.stepTime = scheduleDemo.runSpecHint.stepTime;

% Start the nonlinear run at the exact 30-deg / 60-mps bridge trim point.
runCase.pos_init = trimResult.Pos_Trim;
runCase.V_init = trimResult.Vel_B_BA_Trim;
runCase.eul_init = trimResult.Att_Trim;
runCase.omega_init = trimResult.Rates_Trim;
runCase.Motor_RPMs = trimResult.U_trim_full(1:4);
runCase.Tilt_angles = trimResult.U_trim_full(5:6);
runCase.front_collective = trimResult.U_trim_full(7);
runCase.rear_collective = trimResult.U_trim_full(8);
runCase.surface_init = trimResult.U_trim_full(9:12);

runCase.airData_cmd = scheduleDemo.runSpecHint.airData_cmd;
runCase.cmds_override = scheduleDemo.cmds;

Run_EVTOL_Main
if usejava('desktop')
    open_system('Wrapper')
else
    load_system('Wrapper')
end

fprintf(['Wrapper prepared for the bridge-to-cruise transition demo.\n', ...
         '  initial trim: %s\n', ...
         '  target trim : %s\n', ...
         '  tilt command: %.1f -> %.1f deg\n', ...
         '  Vinf command: %.1f -> %.1f m/s\n', ...
         'Press Run in Wrapper, then use:\n', ...
         '  plot_outputs(''important'')\n'], ...
    char(string(cache_data.trimResult_target.name)), ...
    char(string(cache_data.trimResult_initial.name)), ...
    scheduleDemo.cmds.tilt_cmd(1, 2), scheduleDemo.cmds.tilt_cmd(end, 2), ...
    scheduleDemo.cmds.airData_cmd(1, 2), scheduleDemo.cmds.airData_cmd(end, 2));
