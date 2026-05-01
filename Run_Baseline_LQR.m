%% Baseline scheduled LQR run

repoRoot = fileparts(mfilename('fullpath'));
oldRepo = fullfile(fileparts(repoRoot), 'eVTOL_Simulation');

openModels = {'Wrapper', 'Plant', 'Trim_Plant', 'eVTOL_lib'};
for k = 1:numel(openModels)
    if bdIsLoaded(openModels{k})
        close_system(openModels{k}, 0);
    end
end

oldPathEntries = string(strsplit(path, pathsep));
oldPathEntries = oldPathEntries(startsWith(oldPathEntries, oldRepo));
for k = 1:numel(oldPathEntries)
    if strlength(oldPathEntries(k)) > 0
        rmpath(char(oldPathEntries(k)));
    end
end

cd(repoRoot);
addpath(repoRoot, '-begin');
run(fullfile(repoRoot, 'Init_Main.m'));

S = load(fullfile(database_dir, 'controller_schedule.mat'), 'controllerScheduleDB');
points = S.controllerScheduleDB.points;
names = string({points.name});

startName = "FE_tilt_30__V_60__rear_500";
endName = "RearOnAnchor_V75_Tilt90_R550_front_rear_free_flap_elevator";

startIdx = find(names == startName, 1, 'first');
endIdx = find(names == endName, 1, 'first');
selectedPoints = points([startIdx, endIdx]);

lqrOpts = struct();
lqrOpts.variant_ctrl_mode = 2;
lqrOpts.controller_id = 4;
lqrOpts.require_exact_trim = false;

stateSchedule = zeros(12, 2);
trimSchedule = zeros(6, 2);
gainSchedule = zeros(6, 9, 2);

for i = 1:2
    trim_i = trim_result_from_controller_db_point(selectedPoints(i));
    evalc('ctrl_i = build_trim_lqr_controller(trim_i, lqrOpts);');

    stateSchedule(1:9, i) = ctrl_i.controller_state_ref(1:9);
    stateSchedule(10, i) = selectedPoints(i).tilt_deg;
    stateSchedule(11, i) = selectedPoints(i).vinf_mps;
    stateSchedule(12, i) = i - 1;

    trimSchedule(:, i) = ctrl_i.controller_trim_cmd(:, 1);
    gainSchedule(:, :, i) = ctrl_i.controller_gain_lqr(:, :, 1);
end

controllerData = struct();
controllerData.name = 'Baseline_LQR';
controllerData.type = 'two_point_scheduled_lqr';
controllerData.variant_ctrl_mode = 2;
controllerData.controller_id = 4;
controllerData.controller_state_ref = stateSchedule;
controllerData.controller_trim_cmd = trimSchedule;
controllerData.controller_gain_lqr = gainSchedule;
controllerData.schedule_count = 2;
controllerData.K_lqr_cruise = gainSchedule([1 3 4 5 6], :, end);
controllerData.x_trim_lqr = stateSchedule(1:9, end);
controllerData.U_trim_lqr = [trimSchedule(1, end); trimSchedule(3:6, end)];

trimResult = trim_result_from_controller_db_point(selectedPoints(1));

scheduleOpts = struct();
scheduleOpts.stop_time_s = 45;
scheduleOpts.hold_start_s = 2;
scheduleOpts.hold_end_s = 5;
scheduleOpts.direction = 'hover_to_cruise';
scheduleOpts.command_anchor_source = 'selected_path';

pathSchedule = make_path_schedule_cmds(controllerData, trimResult, scheduleOpts);
cmds = pathSchedule.cmds;

runCase = pathSchedule.runSpecHint;
runCase.name = 'Baseline_LQR';
runCase.useController = true;
runCase.attemptSimulation = true;
runCase.cmds_override = cmds;

Run_Main;
if ~exist('runResult', 'var') || ~strcmp(runResult.status, 'simulated')
    error('Wrapper simulation did not complete. Run_Main message: %s', runResult.message);
end

out = simOut;
Plot_Outputs_Simple;
Plot_Actuators;
