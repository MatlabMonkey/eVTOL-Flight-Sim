% Prep_Demo_Transition_JumpGap_Path.m
% Stage Wrapper for the multi-point jump-gap scheduled transition demo.
%
% The demo follows the hand-selected blue-curve path built from the newer
% scored trim datasets. It starts from the first near-hover exact point and
% commands the selected path segment toward the near-cruise endpoint.
%
% Optional base-workspace knob before running this script:
%   jumpGapSegment = 'all'     -> full path
%   jumpGapSegment = 'prejump' -> points 1:jump_after_index
%   jumpGapSegment = 'jump'    -> points jump_after_index:(jump_after_index+1)
%   jumpGapSegment = 'postjump'-> points (jump_after_index+1):end

segmentName = "all";
if exist('jumpGapSegment', 'var') && ~isempty(jumpGapSegment)
    segmentName = string(jumpGapSegment);
end
setappdata(0, 'PrepDemoTransitionJumpGapSegment', char(segmentName));

Init_EVTOL_Main

segmentName = string(getappdata(0, 'PrepDemoTransitionJumpGapSegment'));
rmappdata(0, 'PrepDemoTransitionJumpGapSegment');

controllerData = build_transition_jumpgap_lqr_controller();
pathPoints = controllerData.path_points;
segmentConfig = localResolveJumpGapSegment(pathPoints, controllerData.path_spec, segmentName);
trimResult = pathPoints(segmentConfig.indices(1)).trimResult;

scheduleOpts = struct();
scheduleOpts.stop_time_s = segmentConfig.stop_time_s;
scheduleOpts.hold_start_s = segmentConfig.hold_start_s;
scheduleOpts.hold_end_s = segmentConfig.hold_end_s;
scheduleOpts.step_time_s = initData.timing.stepTime;
scheduleOpts.direction = 'hover_to_cruise';
scheduleOpts.path_point_indices = segmentConfig.indices;

scheduleDemo = make_transition_path_schedule_cmds(controllerData, trimResult, scheduleOpts);
assignin('base', 'transitionDemoSchedule', scheduleDemo);

runCase = RunCase_PrepOnly();
runCase.name = segmentConfig.run_name;
runCase.useController = true;
runCase.attemptSimulation = false;
runCase.stopTime_s = scheduleDemo.runSpecHint.stopTime;
runCase.stepTime = scheduleDemo.runSpecHint.stepTime;
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

fprintf(['Wrapper prepared for the jump-gap transition demo.\n', ...
         '  segment      : %s\n', ...
         '  initial point: %s\n', ...
         '  final point  : %s\n', ...
         '  path points  : %s\n', ...
         '  tilt command : %.1f -> %.1f deg\n', ...
         '  Vinf command : %.1f -> %.1f m/s\n', ...
         'Press Run in Wrapper, then use:\n', ...
         '  plot_outputs(''important'')\n'], ...
    char(segmentConfig.label), ...
    char(string(pathPoints(segmentConfig.indices(1)).name)), ...
    char(string(pathPoints(segmentConfig.indices(end)).name)), ...
    mat2str(segmentConfig.indices), ...
    scheduleDemo.cmds.tilt_cmd(1, 2), scheduleDemo.cmds.tilt_cmd(end, 2), ...
    scheduleDemo.cmds.airData_cmd(1, 2), scheduleDemo.cmds.airData_cmd(end, 2));

function segmentConfig = localResolveJumpGapSegment(pathPoints, pathSpec, segmentName)
nPts = numel(pathPoints);
segmentName = lower(strtrim(char(string(segmentName))));
jumpAfterIdx = localGetField(pathSpec, 'jump_after_index', max(1, min(4, nPts - 1)));
jumpAfterIdx = max(1, min(jumpAfterIdx, nPts - 1));

segmentConfig = struct();
if any(strcmp(segmentName, {'all', 'full', '1:end'}))
        segmentConfig.indices = 1:nPts;
        segmentConfig.label = "full path";
        segmentConfig.run_name = 'Demo_Transition_JumpGap_Path_Full';
        segmentConfig.stop_time_s = 42.0;
        segmentConfig.hold_start_s = 3.0;
        segmentConfig.hold_end_s = 3.0;
elseif any(strcmp(segmentName, {'prejump', 'pre', '1:8', 'bridge'}))
        segmentConfig.indices = 1:jumpAfterIdx;
        segmentConfig.label = "pre-jump segment";
        segmentConfig.run_name = 'Demo_Transition_JumpGap_Path_PreJump';
        segmentConfig.stop_time_s = 18.0;
        segmentConfig.hold_start_s = 2.0;
        segmentConfig.hold_end_s = 2.0;
elseif any(strcmp(segmentName, {'jump', '8:9', 'gap'}))
        if nPts < 2
            error('Jump segment requires at least 2 path points.');
        end
        segmentConfig.indices = [jumpAfterIdx, jumpAfterIdx + 1];
        segmentConfig.label = "jump segment";
        segmentConfig.run_name = 'Demo_Transition_JumpGap_Path_JumpOnly';
        segmentConfig.stop_time_s = 16.0;
        segmentConfig.hold_start_s = 3.0;
        segmentConfig.hold_end_s = 3.0;
elseif any(strcmp(segmentName, {'postjump', 'post', '9:end', 'cruise'}))
        if nPts < 2
            error('Post-jump segment requires at least 2 path points.');
        end
        segmentConfig.indices = (jumpAfterIdx + 1):nPts;
        segmentConfig.label = "post-jump segment";
        segmentConfig.run_name = 'Demo_Transition_JumpGap_Path_PostJump';
        segmentConfig.stop_time_s = 20.0;
        segmentConfig.hold_start_s = 2.0;
        segmentConfig.hold_end_s = 2.0;
else
        error('Unknown jumpGapSegment "%s". Use all, prejump, jump, or postjump.', segmentName);
end
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
end
end
