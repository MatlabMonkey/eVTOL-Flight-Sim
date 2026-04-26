function demo = make_path_schedule_cmds(controllerData, trimResult, opts)
%MAKE_TRANSITION_PATH_SCHEDULE_CMDS Build smooth commands along the active path schedule.
%
% Usage:
%   demo = make_path_schedule_cmds(controllerData, trimResult)
%   demo = make_path_schedule_cmds(controllerData, trimResult, opts)
%
% Required inputs:
%   controllerData = scheduled controller struct from
%                    build_transition_path_lqr_controller
%   trimResult     = run-ready trimResult used for the initial condition
%
% Optional opts fields:
%   stop_time_s    total duration (default 18)
%   hold_start_s   initial dwell before ramp (default 2)
%   hold_end_s     final dwell after ramp (default 2)
%   step_time_s    command sample time (default initData.timing.stepTime or 0.01)
%   direction      'hover_to_cruise' or 'cruise_to_hover' (default hover_to_cruise)
%   path_point_indices indices of scheduled points to keep (default all)
%   end_progress   final normalized progress within the selected subset (default 1)
%   assign_to_base assign demo/cmds into base workspace (default false)
%   command_anchor_source 'selected_path' (default) or 'guide'
%
% Output:
%   demo.cmds        wrapper command traces
%   demo.runSpecHint suggested runSpec overrides for the same schedule
%   demo.meta        summary information about the schedule

if nargin < 3 || isempty(opts)
    opts = struct();
end

if ~isstruct(controllerData)
    error('controllerData must be the struct returned by build_transition_path_lqr_controller.');
end
if ~isstruct(trimResult)
    error('trimResult must be a run-ready trimResult struct.');
end

opts = localApplyDefaults(opts);

stateSchedule = localGetField(controllerData, 'controller_state_ref', []);
trimSchedule = localGetField(controllerData, 'controller_trim_cmd', []);
scheduleCount = localGetField(controllerData, 'schedule_count', size(stateSchedule, 2));
scheduleCount = min(scheduleCount, size(stateSchedule, 2));
scheduleCount = min(scheduleCount, size(trimSchedule, 2));

if scheduleCount < 2
    error('Need at least two scheduled points to build a transition command profile.');
end

stateSchedule = stateSchedule(:, 1:scheduleCount);
trimSchedule = trimSchedule(:, 1:scheduleCount);

selectedIdx = localResolvePointSelection(opts.path_point_indices, scheduleCount);
stateSchedule = stateSchedule(:, selectedIdx);
trimSchedule = trimSchedule(:, selectedIdx);
scheduleCount = numel(selectedIdx);

progressAnchor = localGetScheduleRow(stateSchedule, 12, scheduleCount);
if localAllZero(progressAnchor)
    progressAnchor = linspace(0.0, 1.0, scheduleCount);
end
progressAnchor = localNormalizeProgress(progressAnchor);

phiAnchor = rad2deg(localGetScheduleRow(stateSchedule, 1, scheduleCount));
thetaAnchor = rad2deg(localGetScheduleRow(stateSchedule, 2, scheduleCount));
psiAnchor = rad2deg(localGetScheduleRow(stateSchedule, 3, scheduleCount));
uAnchor = localGetScheduleRow(stateSchedule, 4, scheduleCount);
vAnchor = localGetScheduleRow(stateSchedule, 5, scheduleCount);
wAnchor = localGetScheduleRow(stateSchedule, 6, scheduleCount);
tiltAnchor = localGetScheduleRow(stateSchedule, 10, scheduleCount);
vinfAnchor = localGetScheduleRow(stateSchedule, 11, scheduleCount);
[tiltAnchor, vinfAnchor] = localResolveCommandAnchors(controllerData, selectedIdx, scheduleCount, tiltAnchor, vinfAnchor, opts);

alphaAnchor = zeros(1, scheduleCount);
betaAnchor = zeros(1, scheduleCount);
for i = 1:scheduleCount
    velBody = [uAnchor(i); vAnchor(i); wAnchor(i)];
    vinf = vinfAnchor(i);
    if ~isfinite(vinf) || vinf <= 0
        vinf = norm(velBody);
        vinfAnchor(i) = vinf;
    end
    alphaAnchor(i) = rad2deg(atan2(velBody(3), max(velBody(1), 1e-6)));
    if vinf > 1e-6
        betaArg = max(min(velBody(2) / vinf, 1.0), -1.0);
        betaAnchor(i) = rad2deg(asin(betaArg));
    else
        betaAnchor(i) = 0.0;
    end
end

frontCollectiveAnchor = trimSchedule(1, :);
rearCollectiveAnchor = trimSchedule(2, :);
deltaFAnchor = rad2deg(trimSchedule(3, :));
deltaAAnchor = rad2deg(trimSchedule(4, :));
deltaEAnchor = rad2deg(trimSchedule(5, :));
deltaRAnchor = rad2deg(trimSchedule(6, :));

t = (0:opts.step_time_s:opts.stop_time_s).';
progressCmd = localBuildProgressProfile(t, opts.hold_start_s, opts.hold_end_s, opts.stop_time_s);
progressCmd = opts.end_progress * progressCmd;
if strcmp(opts.direction, 'cruise_to_hover')
    progressCmd = 1.0 - progressCmd;
end

phiCmdDeg = localInterp1(progressAnchor, phiAnchor, progressCmd);
thetaCmdDeg = localInterp1(progressAnchor, thetaAnchor, progressCmd);
psiCmdDeg = localInterp1(progressAnchor, psiAnchor, progressCmd);
vinfCmd = localInterp1(progressAnchor, vinfAnchor, progressCmd);
alphaCmdDeg = localInterp1(progressAnchor, alphaAnchor, progressCmd);
betaCmdDeg = localInterp1(progressAnchor, betaAnchor, progressCmd);
tiltCmdDeg = localInterp1(progressAnchor, tiltAnchor, progressCmd);
frontCollectiveCmd = localInterp1(progressAnchor, frontCollectiveAnchor, progressCmd);
rearCollectiveCmd = localInterp1(progressAnchor, rearCollectiveAnchor, progressCmd);
deltaFCmdDeg = localInterp1(progressAnchor, deltaFAnchor, progressCmd);
deltaACmdDeg = localInterp1(progressAnchor, deltaAAnchor, progressCmd);
deltaECmdDeg = localInterp1(progressAnchor, deltaEAnchor, progressCmd);
deltaRCmdDeg = localInterp1(progressAnchor, deltaRAnchor, progressCmd);

surfaceLocalRad = localMixedToLocalRad( ...
    deg2rad(deltaFCmdDeg), deg2rad(deltaACmdDeg), ...
    deg2rad(deltaECmdDeg), deg2rad(deltaRCmdDeg));

motorTrim = zeros(4, 1);
if isfield(trimResult, 'U_trim_full') && numel(trimResult.U_trim_full) >= 4
    motorTrim = trimResult.U_trim_full(1:4);
end

posInit = zeros(3, 1);
if isfield(trimResult, 'Pos_Trim') && numel(trimResult.Pos_Trim) >= 3
    posInit = trimResult.Pos_Trim(1:3);
end

cmds = struct();
cmds.airData_cmd = [t, vinfCmd, deg2rad(alphaCmdDeg), deg2rad(betaCmdDeg)];
cmds.eul_cmd = [t, deg2rad(phiCmdDeg), deg2rad(thetaCmdDeg), deg2rad(psiCmdDeg)];
cmds.gps_Pos_cmd = [t, ...
    posInit(1) * ones(numel(t), 1), ...
    posInit(2) * ones(numel(t), 1), ...
    posInit(3) * ones(numel(t), 1)];
cmds.omega_cmd = [t, zeros(numel(t), 3)];
cmds.accel_cmd = [t, zeros(numel(t), 3)];
cmds.motor_cmd = [t, repmat(motorTrim(:).', numel(t), 1)];
cmds.tilt_cmd = [t, tiltCmdDeg, tiltCmdDeg];
cmds.front_cmd = [t, frontCollectiveCmd];
cmds.rear_cmd = [t, rearCollectiveCmd];
cmds.surface_mixed_cmd = [t, ...
    deg2rad(deltaFCmdDeg), deg2rad(deltaACmdDeg), ...
    deg2rad(deltaECmdDeg), deg2rad(deltaRCmdDeg)];
cmds.surface_local_cmd = [t, surfaceLocalRad];
cmds.deltaLW_cmd = [t, surfaceLocalRad(:, 1)];
cmds.deltaRW_cmd = [t, surfaceLocalRad(:, 2)];
cmds.deltaLT_cmd = [t, surfaceLocalRad(:, 3)];
cmds.deltaRT_cmd = [t, surfaceLocalRad(:, 4)];
cmds.demo_progress = [t, progressCmd];

runSpecHint = struct();
runSpecHint.startTime = 0.0;
runSpecHint.stopTime = opts.stop_time_s;
runSpecHint.stepTime = opts.step_time_s;
runSpecHint.useController = true;
runSpecHint.attemptSimulation = false;
runSpecHint.pos_init = posInit;
runSpecHint.V_init = [uAnchor(1); vAnchor(1); wAnchor(1)];
runSpecHint.airData_cmd = cmds.airData_cmd(1, 2:4).';
runSpecHint.eul_init = deg2rad([phiCmdDeg(1); thetaCmdDeg(1); psiCmdDeg(1)]);
runSpecHint.omega_init = [0; 0; 0];
runSpecHint.Motor_RPMs = motorTrim;
runSpecHint.Tilt_angles = [tiltCmdDeg(1); tiltCmdDeg(1)];
runSpecHint.front_collective = frontCollectiveCmd(1);
runSpecHint.rear_collective = rearCollectiveCmd(1);
runSpecHint.surface_init = surfaceLocalRad(1, :).';

demo = struct();
demo.name = sprintf('TransitionPathSchedule_%s', opts.direction);
demo.direction = opts.direction;
demo.cmds = cmds;
demo.runSpecHint = runSpecHint;
demo.meta = struct();
demo.meta.schedule_count = scheduleCount;
demo.meta.selected_indices = selectedIdx(:);
demo.meta.schedule_progress = progressAnchor(:);
demo.meta.stop_time_s = opts.stop_time_s;
demo.meta.hold_start_s = opts.hold_start_s;
demo.meta.hold_end_s = opts.hold_end_s;
demo.meta.end_progress = opts.end_progress;

if opts.assign_to_base
    assignin('base', 'transitionPathScheduleDemo', demo);
    assignin('base', 'cmds', cmds);
end
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'stop_time_s') || isempty(opts.stop_time_s)
    opts.stop_time_s = 18.0;
end
if ~isfield(opts, 'hold_start_s') || isempty(opts.hold_start_s)
    opts.hold_start_s = 2.0;
end
if ~isfield(opts, 'hold_end_s') || isempty(opts.hold_end_s)
    opts.hold_end_s = 2.0;
end
if ~isfield(opts, 'step_time_s') || isempty(opts.step_time_s)
    opts.step_time_s = localResolveStepTime();
end
if ~isfield(opts, 'direction') || isempty(opts.direction)
    opts.direction = 'hover_to_cruise';
end
if ~isfield(opts, 'path_point_indices') || isempty(opts.path_point_indices)
    opts.path_point_indices = [];
end
if ~isfield(opts, 'end_progress') || isempty(opts.end_progress)
    opts.end_progress = 1.0;
end
opts.end_progress = max(min(opts.end_progress, 1.0), 0.0);
if ~isfield(opts, 'assign_to_base') || isempty(opts.assign_to_base)
    opts.assign_to_base = false;
end
if ~isfield(opts, 'command_anchor_source') || isempty(opts.command_anchor_source)
    opts.command_anchor_source = 'selected_path';
end
end

function selectedIdx = localResolvePointSelection(pathPointIndices, scheduleCount)
if isempty(pathPointIndices)
    selectedIdx = 1:scheduleCount;
    return;
end

selectedIdx = unique(round(pathPointIndices(:).'), 'stable');
selectedIdx = selectedIdx(selectedIdx >= 1 & selectedIdx <= scheduleCount);
if numel(selectedIdx) < 2
    error('Need at least two valid path_point_indices within the schedule.');
end
end

function progressAnchor = localNormalizeProgress(progressAnchor)
progressAnchor = progressAnchor(:).';
if isempty(progressAnchor)
    return;
end

progressAnchor = progressAnchor - progressAnchor(1);
span = progressAnchor(end);
if span <= 1e-12
    progressAnchor = linspace(0.0, 1.0, numel(progressAnchor));
else
    progressAnchor = progressAnchor / span;
end
end

function stepTime = localResolveStepTime()
stepTime = 0.01;
if evalin('base', 'exist(''initData'', ''var'')') %#ok<EVLC>
    initData = evalin('base', 'initData'); %#ok<EVLC>
    if isstruct(initData) && isfield(initData, 'timing') && isfield(initData.timing, 'stepTime')
        stepTime = initData.timing.stepTime;
    end
end
end

function progress = localBuildProgressProfile(t, holdStart, holdEnd, stopTime)
progress = zeros(size(t));
transitionTime = max(stopTime - holdStart - holdEnd, eps);
activeMask = t >= holdStart & t <= (stopTime - holdEnd);
if any(activeMask)
    tau = (t(activeMask) - holdStart) / transitionTime;
    progress(activeMask) = localSmootherStep(tau);
    progress(t > (stopTime - holdEnd)) = 1.0;
end
end

function yq = localInterp1(x, y, xq)
x = x(:);
y = y(:);
[xUnique, keepIdx] = unique(x, 'stable');
yUnique = y(keepIdx);
if numel(xUnique) == 1
    yq = yUnique(1) * ones(size(xq));
    return;
end
yq = interp1(xUnique, yUnique, xq, 'linear', 'extrap');
end

function row = localGetScheduleRow(stateSchedule, rowIdx, scheduleCount)
row = zeros(1, scheduleCount);
if size(stateSchedule, 1) >= rowIdx
    row(:) = stateSchedule(rowIdx, 1:scheduleCount);
end
end

function [tiltAnchor, vinfAnchor] = localResolveCommandAnchors(controllerData, selectedIdx, scheduleCount, tiltAnchor, vinfAnchor, opts)
if ~strcmpi(string(opts.command_anchor_source), 'guide')
    return;
end
if ~isstruct(controllerData) || ~isfield(controllerData, 'path_spec') || ~isstruct(controllerData.path_spec)
    return;
end

pathSpec = controllerData.path_spec;
if ~isfield(pathSpec, 'guide_vinf_mps') || ~isfield(pathSpec, 'guide_tilt_deg')
    return;
end

guideV = pathSpec.guide_vinf_mps(:);
guideT = pathSpec.guide_tilt_deg(:);
if numel(guideV) < max(selectedIdx) || numel(guideT) < max(selectedIdx)
    return;
end

guideV = guideV(selectedIdx);
guideT = guideT(selectedIdx);
if numel(guideV) ~= scheduleCount || numel(guideT) ~= scheduleCount
    return;
end

vinfAnchor = guideV(:).';
tiltAnchor = guideT(:).';
end

function tf = localAllZero(vec)
tf = all(abs(vec(:)) <= 1e-12);
end

function y = localSmootherStep(x)
x = max(min(x, 1.0), 0.0);
y = x .* x .* x .* (x .* (x * 6.0 - 15.0) + 10.0);
end

function surfaceLocalRad = localMixedToLocalRad(deltaF, deltaA, deltaE, deltaR)
n = numel(deltaF);
surfaceLocalRad = zeros(n, 4);
for i = 1:n
    surfaceLocalRad(i, 1) = deltaF(i) + deltaA(i);
    surfaceLocalRad(i, 2) = deltaF(i) - deltaA(i);
    surfaceLocalRad(i, 3) = deltaE(i) - deltaR(i);
    surfaceLocalRad(i, 4) = deltaE(i) + deltaR(i);
end
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end
