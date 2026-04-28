function result = Run_INDI_FrozenTrimHold_Test(pathIndex, opts)
%RUN_INDI_FROZENTRIMHOLD_TEST Verify INDI can hold one fixed trim point.
%
% Usage:
%   result = Run_INDI_FrozenTrimHold_Test
%   result = Run_INDI_FrozenTrimHold_Test(10)
%   result = Run_INDI_FrozenTrimHold_Test(10, struct('stopTime_s', 60))
%   result = Run_INDI_FrozenTrimHold_Test([], struct( ...
%       'initial_condition_offset', struct('u_mps', 2.0)))
%
% This is the first INDI verification test. It disables path advancement by
% reducing the controller schedule to one point and starts Wrapper exactly
% at that trim state. At trim, the virtual-control target is body specific
% force [g*sin(theta); -g*cos(theta)] and qdot = 0, not zero acceleration.
%
% If opts.initial_condition_offset is provided, the same fixed trim target is
% held but the Wrapper initial condition is offset. Supported fields are:
%   u_mps, w_mps, theta_deg, q_deg_s

if nargin < 1 || isempty(pathIndex)
    pathIndex = [];
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

repoRoot = fileparts(mfilename('fullpath'));
cd(repoRoot);

evalin('base', 'Init_Main');

if isempty(opts.builder_opts)
    fullControllerData = build_indi_transition_controller();
else
    fullControllerData = build_indi_transition_controller(opts.builder_opts);
end

nPts = localScheduleCount(fullControllerData);
if isempty(pathIndex)
    pathIndex = nPts;
end
if pathIndex < 1 || pathIndex > nPts
    error('Run_INDI_FrozenTrimHold_Test:BadPathIndex', ...
        'pathIndex must be in [1, %d]. Received %d.', nPts, pathIndex);
end

controllerData = localSliceSinglePoint(fullControllerData, pathIndex);
trimResult = localBuildTrimResult(controllerData);
runCase = localBuildRunCase(pathIndex, opts, trimResult);

assignin('base', 'fullIndiTransitionControllerData', fullControllerData);
assignin('base', 'controllerData', controllerData);
assignin('base', 'indiFrozenTrimPointTable', controllerData.schedule_table);
assignin('base', 'indiTransitionPathTable', controllerData.schedule_table);
assignin('base', 'trimResult', trimResult);
assignin('base', 'runCase', runCase);

fprintf('\n=== INDI Frozen Trim-Hold Test ===\n');
fprintf('Path point     : %d of %d\n', pathIndex, nPts);
fprintf('Run name       : %s\n', runCase.name);
fprintf('Stop time      : %.3f s\n', runCase.stopTime_s);
localPrintPerturbation(runCase);
disp(controllerData.schedule_table(:, {'path_index', 'vinf_mps', ...
    'tilt_deg', 'alpha_deg', 'theta_deg', 'front_collective_rpm', ...
    'rear_collective_rpm', 'delta_f_deg', 'delta_e_deg', 'classification'}));

evalin('base', 'Run_Main');
runResult = evalin('base', 'runResult');
runContext = evalin('base', 'runContext');

metrics = struct();
debugReplay = struct();
if strcmp(runResult.status, 'simulated')
    simSource = evalin('base', 'out');
    metrics = localBuildMetrics(simSource, controllerData);
    debugReplay = localReplayFinalIndiStep(simSource, controllerData);
    localPrintMetrics(metrics, debugReplay);
    if opts.plot_outputs
        assignin('base', 'out', simSource);
        plot_outputs('important');
    end
end

result = struct();
result.path_index = pathIndex;
result.controllerData = controllerData;
result.trimResult = trimResult;
result.runCase = runCase;
result.runResult = runResult;
result.runContext = runContext;
result.metrics = metrics;
result.debugReplay = debugReplay;
result.initial_condition_offset = opts.initial_condition_offset;

assignin('base', 'indiFrozenTrimHoldResult', result);
fprintf('Status         : %s\n', runResult.status);
fprintf('=== INDI Frozen Trim-Hold Test Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'stopTime_s') || isempty(opts.stopTime_s)
    opts.stopTime_s = 60.0;
end
if ~isfield(opts, 'attemptSimulation') || isempty(opts.attemptSimulation)
    opts.attemptSimulation = true;
end
if ~isfield(opts, 'plot_outputs') || isempty(opts.plot_outputs)
    opts.plot_outputs = true;
end
if ~isfield(opts, 'builder_opts')
    opts.builder_opts = [];
end
if ~isfield(opts, 'initial_condition_offset') || isempty(opts.initial_condition_offset)
    opts.initial_condition_offset = struct();
end
end

function nPts = localScheduleCount(controllerData)
if isfield(controllerData, 'schedule_count') && ~isempty(controllerData.schedule_count)
    nPts = controllerData.schedule_count;
else
    nPts = size(controllerData.controller_trim_cmd, 2);
end
end

function controllerData = localSliceSinglePoint(fullControllerData, pathIndex)
controllerData = fullControllerData;
controllerData.name = sprintf('%s_frozen_point_%02d', ...
    fullControllerData.name, pathIndex);
fullScheduleCount = localScheduleCount(fullControllerData);
controllerData.controller_state_ref = fullControllerData.controller_state_ref(:, pathIndex);
controllerData.controller_trim_cmd = fullControllerData.controller_trim_cmd(:, pathIndex);
controllerData.controller_gain_lqr = fullControllerData.controller_gain_lqr(:, :, pathIndex);
if size(fullControllerData.controller_gain_lqr, 3) > fullScheduleCount
    controllerData.controller_gain_lqr = cat(3, controllerData.controller_gain_lqr, ...
        fullControllerData.controller_gain_lqr(:, :, (fullScheduleCount + 1):end));
end
controllerData.controller_state_ref(12, 1) = 0.0;

pathTable = fullControllerData.schedule_table(pathIndex, :);
pathTable.path_index = 1;
pathTable.path_progress = 0.0;

controllerData.schedule_count = 1;
controllerData.schedule_progress = 0.0;
controllerData.schedule_tilt_deg = pathTable.tilt_deg;
controllerData.schedule_vinf_mps = pathTable.vinf_mps;
controllerData.schedule_alpha_deg = pathTable.alpha_deg;
controllerData.schedule_table = pathTable;
controllerData.x_trim = controllerData.controller_state_ref(1:9, :);
controllerData.trim_cmd = controllerData.controller_trim_cmd;
controllerData.x_trim_lqr = controllerData.controller_state_ref(1:9, 1);
controllerData.U_trim_lqr = [controllerData.controller_trim_cmd(1, 1); ...
    controllerData.controller_trim_cmd(3:6, 1)];
end

function trimResult = localBuildTrimResult(controllerData)
row = controllerData.schedule_table(1, :);
trim = controllerData.controller_trim_cmd(:, 1);
x = controllerData.controller_state_ref(:, 1);
mixMatrix = evalin('base', 'MixMatrix');
surface = mixMatrix * trim(3:6);

trimResult = struct();
trimResult.name = char("INDI_FrozenTrim_" + string(row.path_index));
trimResult.mode = 'transition';
trimResult.modelName = 'Trim_Plant';
trimResult.success = true;
trimResult.isExactTrim = true;
trimResult.Att_Trim = [0; deg2rad(row.theta_deg); 0];
trimResult.Att_Trim_deg = [0; row.theta_deg; 0];
trimResult.Vel_B_BA_Trim = [row.u_mps; 0; row.w_mps];
trimResult.Vel_W_Trim = [row.vinf_mps; deg2rad(row.alpha_deg); 0];
trimResult.Rates_Trim = x(7:9);
trimResult.Pos_Trim = zeros(3, 1);
trimResult.X_trim = [zeros(3, 1); trimResult.Vel_B_BA_Trim; ...
    trimResult.Att_Trim; trimResult.Rates_Trim];
trimResult.Act_Trim = surface;
trimResult.U_trim = [trim(1); trim(3:6)];
trimResult.U_surface_trim = [trim(1); surface];
trimResult.U_trim_full = [zeros(4, 1); row.tilt_deg; row.tilt_deg; ...
    trim(1); trim(2); surface];
trimResult.trim = trimResult;
end

function runCase = localBuildRunCase(pathIndex, opts, trimResult)
runCase = struct();
suffix = localPerturbationNameSuffix(opts.initial_condition_offset);
runCase.name = sprintf('INDI_FrozenTrimHold_Point%02d%s_%gs', ...
    pathIndex, suffix, opts.stopTime_s);
runCase.useController = true;
runCase.attemptSimulation = opts.attemptSimulation;
runCase.stopTime_s = opts.stopTime_s;
runCase.initial_condition_offset = opts.initial_condition_offset;
runCase.cmds_override = localBuildFixedTrimCommands(trimResult, opts.stopTime_s);

[vInit, eulInit, omegaInit] = localPerturbedInitialCondition( ...
    trimResult, opts.initial_condition_offset);
runCase.V_init = vInit;
runCase.eul_init = eulInit;
runCase.omega_init = omegaInit;
end

function suffix = localPerturbationNameSuffix(offset)
suffix = '';
if ~localHasPerturbation(offset)
    return;
end

tokens = {};
if localHasNumericField(offset, 'u_mps')
    tokens{end + 1} = localFormatNameToken('du', offset.u_mps);
end
if localHasNumericField(offset, 'w_mps')
    tokens{end + 1} = localFormatNameToken('dw', offset.w_mps);
end
if localHasNumericField(offset, 'theta_deg')
    tokens{end + 1} = localFormatNameToken('dtheta', offset.theta_deg);
end
if localHasNumericField(offset, 'q_deg_s')
    tokens{end + 1} = localFormatNameToken('dq', offset.q_deg_s);
end
suffix = ['_' strjoin(tokens, '_')];
end

function token = localFormatNameToken(label, value)
if value < 0
    signText = 'm';
else
    signText = 'p';
end
token = sprintf('%s%s%s', label, signText, strrep(sprintf('%.3g', abs(value)), '.', 'p'));
end

function tf = localHasPerturbation(offset)
tf = isstruct(offset) && ( ...
    localHasNumericField(offset, 'u_mps') || ...
    localHasNumericField(offset, 'w_mps') || ...
    localHasNumericField(offset, 'theta_deg') || ...
    localHasNumericField(offset, 'q_deg_s'));
end

function tf = localHasNumericField(s, fieldName)
tf = isstruct(s) && isfield(s, fieldName) && ...
    isnumeric(s.(fieldName)) && isscalar(s.(fieldName)) && isfinite(s.(fieldName));
end

function [vInit, eulInit, omegaInit] = localPerturbedInitialCondition(trimResult, offset)
vInit = trimResult.Vel_B_BA_Trim(:);
eulInit = trimResult.Att_Trim(:);
omegaInit = trimResult.Rates_Trim(:);

if localHasNumericField(offset, 'u_mps')
    vInit(1) = vInit(1) + offset.u_mps;
end
if localHasNumericField(offset, 'w_mps')
    vInit(3) = vInit(3) + offset.w_mps;
end
if localHasNumericField(offset, 'theta_deg')
    eulInit(2) = eulInit(2) + deg2rad(offset.theta_deg);
end
if localHasNumericField(offset, 'q_deg_s')
    omegaInit(2) = omegaInit(2) + deg2rad(offset.q_deg_s);
end
end

function cmds = localBuildFixedTrimCommands(trimResult, stopTime_s)
defaultRunSpec = evalin('base', 'initData.defaultRunSpec');
t = (defaultRunSpec.startTime:defaultRunSpec.stepTime:stopTime_s).';
if isempty(t) || t(end) < stopTime_s
    t = [t; stopTime_s];
end
N = numel(t);

cmds = struct();
cmds.airData_cmd = [t, repmat(trimResult.Vel_W_Trim(:).', N, 1)];
cmds.eul_cmd = [t, repmat(trimResult.Att_Trim(:).', N, 1)];
cmds.gps_Pos_cmd = [t, zeros(N, 3)];
cmds.omega_cmd = [t, repmat(trimResult.Rates_Trim(:).', N, 1)];
cmds.accel_cmd = [t, zeros(N, 3)];
cmds.motor_cmd = [t, repmat(trimResult.U_trim_full(1:4).', N, 1)];
cmds.tilt_cmd = [t, repmat(trimResult.U_trim_full(5:6).', N, 1)];
cmds.front_cmd = [t, trimResult.U_trim_full(7) * ones(N, 1)];
cmds.rear_cmd = [t, trimResult.U_trim_full(8) * ones(N, 1)];
end

function localPrintPerturbation(runCase)
offset = runCase.initial_condition_offset;
if ~localHasPerturbation(offset)
    fprintf('Initial offset : none\n');
    return;
end

fprintf('Initial offset : du=%+.3f m/s, dw=%+.3f m/s, dtheta=%+.3f deg, dq=%+.3f deg/s\n', ...
    localNumericFieldOrDefault(offset, 'u_mps', 0.0), ...
    localNumericFieldOrDefault(offset, 'w_mps', 0.0), ...
    localNumericFieldOrDefault(offset, 'theta_deg', 0.0), ...
    localNumericFieldOrDefault(offset, 'q_deg_s', 0.0));
fprintf('Command target : fixed at the unperturbed trim point\n');
end

function value = localNumericFieldOrDefault(s, fieldName, defaultValue)
if localHasNumericField(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function metrics = localBuildMetrics(simSource, controllerData)
xTrim = controllerData.controller_state_ref(1:9, 1);
trim = controllerData.controller_trim_cmd(:, 1);
row = controllerData.schedule_table(1, :);

[~, vel] = localGetSignal(simSource, 'V_BA_truth');
[~, eul] = localGetSignal(simSource, 'eul_truth');
[~, omega] = localGetSignal(simSource, 'omega_truth');
[~, specForce] = localGetSignal(simSource, 'specific_force_truth');
[~, qdot] = localGetOptionalSignal(simSource, {'angular_accel_meas', 'angular_accel_truth'});
[~, actuators] = localGetOptionalSignal(simSource, {'actuator_state_truth', 'actuator_state_meas'});

thetaTrim = xTrim(2);
nuTarget = [9.81 * sin(thetaTrim); -9.81 * cos(thetaTrim); 0.0];

metrics = struct();
metrics.trim = struct();
metrics.trim.u_mps = xTrim(4);
metrics.trim.w_mps = xTrim(6);
metrics.trim.theta_deg = rad2deg(xTrim(2));
metrics.trim.q_deg_s = rad2deg(xTrim(8));
metrics.trim.nu_target = nuTarget;
metrics.trim.front_rpm = trim(1);
metrics.trim.rear_rpm = trim(2);
metrics.trim.tilt_deg = row.tilt_deg;

metrics.state = struct();
metrics.state.max_abs_u_error_mps = max(abs(vel(:, 1) - xTrim(4)));
metrics.state.max_abs_w_error_mps = max(abs(vel(:, 3) - xTrim(6)));
metrics.state.max_abs_theta_error_deg = max(abs(rad2deg(eul(:, 2) - xTrim(2))));
metrics.state.max_abs_q_error_deg_s = max(abs(rad2deg(omega(:, 2) - xTrim(8))));
metrics.state.initial = [vel(1, 1); vel(1, 3); rad2deg(eul(1, 2)); rad2deg(omega(1, 2))];
metrics.state.final = [vel(end, 1); vel(end, 3); rad2deg(eul(end, 2)); rad2deg(omega(end, 2))];
metrics.state.initial_abs_error = abs(metrics.state.initial - [ ...
    xTrim(4); xTrim(6); rad2deg(xTrim(2)); rad2deg(xTrim(8))]);
metrics.state.final_abs_error = abs(metrics.state.final - [ ...
    xTrim(4); xTrim(6); rad2deg(xTrim(2)); rad2deg(xTrim(8))]);

metrics.virtual = struct();
metrics.virtual.final_specific_force = [specForce(end, 1); specForce(end, 3)];
metrics.virtual.final_specific_force_error = metrics.virtual.final_specific_force - nuTarget(1:2);
metrics.virtual.max_abs_specific_force_error = max(abs([ ...
    specForce(:, 1) - nuTarget(1), specForce(:, 3) - nuTarget(2)]), [], 1).';
if ~isempty(qdot)
    metrics.virtual.final_qdot_deg_s2 = rad2deg(qdot(end, 2));
    metrics.virtual.max_abs_qdot_deg_s2 = max(abs(rad2deg(qdot(:, 2))));
else
    metrics.virtual.final_qdot_deg_s2 = NaN;
    metrics.virtual.max_abs_qdot_deg_s2 = NaN;
end

metrics.actuator = localActuatorMetrics(actuators, controllerData);
end

function actuatorMetrics = localActuatorMetrics(actuators, controllerData)
trim = controllerData.controller_trim_cmd(:, 1);
row = controllerData.schedule_table(1, :);
mixMatrix = evalin('base', 'MixMatrix');
surfaceTrim = mixMatrix * trim(3:6);
target = [trim(1); trim(1); trim(2); trim(2); ...
    row.tilt_deg; row.tilt_deg; surfaceTrim(:)];

actuatorMetrics = struct();
actuatorMetrics.target = target;
if isempty(actuators)
    actuatorMetrics.max_abs_error = NaN(size(target));
    actuatorMetrics.final_error = NaN(size(target));
    return;
end

errors = actuators(:, 1:numel(target)) - target(:).';
actuatorMetrics.max_abs_error = max(abs(errors), [], 1).';
actuatorMetrics.final_error = errors(end, :).';
end

function debugReplay = localReplayFinalIndiStep(simSource, controllerData)
debugReplay = struct();
try
    [~, vel] = localGetSignal(simSource, 'V_BA_truth');
    [~, eul] = localGetSignal(simSource, 'eul_truth');
    [~, omega] = localGetSignal(simSource, 'omega_truth');
    [~, accel] = localGetOptionalSignal(simSource, {'accel_Meas_log', 'specific_force_truth'});
    [~, angularAccel] = localGetOptionalSignal(simSource, {'angular_accel_meas', 'angular_accel_truth'});
    [~, actuators] = localGetOptionalSignal(simSource, {'actuator_state_meas', 'actuator_state_truth'});
    if isempty(accel) || isempty(angularAccel) || isempty(actuators)
        debugReplay.available = false;
        return;
    end

    row = controllerData.schedule_table(1, :);
    airDataCmd = [row.vinf_mps; deg2rad(row.alpha_deg); 0.0];
    tiltCmd = [row.tilt_deg; row.tilt_deg];
    xMeas = [eul(end, :).'; vel(end, :).'; omega(end, :).'];

    clear controller_indi_transition;
    runtimeGMap = localRuntimeGMapOrDefault(controllerData);
    [uMixed, tiltDeg, debug] = controller_indi_transition( ...
        airDataCmd, tiltCmd, xMeas, accel(end, :).', actuators(end, :).', ...
        angularAccel(end, :).', controllerData.controller_state_ref, ...
        controllerData.controller_trim_cmd, controllerData.controller_gain_lqr, runtimeGMap, ...
        evalin('base', 'controller_surface_limit_rad'), ...
        evalin('base', 'front_collective_min_rpm'), ...
        evalin('base', 'front_collective_max_rpm'), ...
        evalin('base', 'rear_collective_min_rpm'), ...
        evalin('base', 'rear_collective_max_rpm'));

    debugReplay.available = true;
    debugReplay.u_mixed = uMixed;
    debugReplay.tilt_cmd_deg = tiltDeg;
    debugReplay.nu_err = debug(6:8);
    debugReplay.delta_eta = debug(9:12);
    debugReplay.nu_meas = [accel(end, 1); accel(end, 3); angularAccel(end, 2)];
    debugReplay.nu_cmd = debugReplay.nu_meas + debugReplay.nu_err;
catch ME
    debugReplay.available = false;
    debugReplay.error = ME.message;
end
end

function runtimeGMap = localRuntimeGMapOrDefault(controllerData)
runtimeGMap = zeros(6, 9, 1);
if isstruct(controllerData) && isfield(controllerData, 'controller_runtime_g_map') && ...
        ~isempty(controllerData.controller_runtime_g_map)
    runtimeGMap = controllerData.controller_runtime_g_map;
end
end

function localPrintMetrics(metrics, debugReplay)
fprintf('\nFrozen trim-hold metrics:\n');
fprintf('  max |u error|     : %.4f m/s\n', metrics.state.max_abs_u_error_mps);
fprintf('  max |w error|     : %.4f m/s\n', metrics.state.max_abs_w_error_mps);
fprintf('  max |theta error| : %.4f deg\n', metrics.state.max_abs_theta_error_deg);
fprintf('  max |q error|     : %.4f deg/s\n', metrics.state.max_abs_q_error_deg_s);
fprintf('  initial [u w theta q]: [%.4f %.4f %.4f %.4f]\n', metrics.state.initial);
fprintf('  final [u w theta q]: [%.4f %.4f %.4f %.4f]\n', metrics.state.final);
fprintf('  final abs error    : [%.4f %.4f %.4f %.4f]\n', metrics.state.final_abs_error);
fprintf('  target [fx fz qdot]: [%.4f %.4f %.4f]\n', metrics.trim.nu_target);
fprintf('  final specific force [fx fz]: [%.4f %.4f]\n', ...
    metrics.virtual.final_specific_force);
fprintf('  final specific force error  : [%.4f %.4f]\n', ...
    metrics.virtual.final_specific_force_error);
fprintf('  max |qdot|       : %.4f deg/s^2\n', metrics.virtual.max_abs_qdot_deg_s2);
if isfield(debugReplay, 'available') && debugReplay.available
    fprintf('  replay nu_err    : [%.4g %.4g %.4g]\n', debugReplay.nu_err);
    fprintf('  replay delta_eta : [%.4g %.4g %.4g %.4g]\n', debugReplay.delta_eta);
end
end

function [t, y] = localGetOptionalSignal(simSource, names)
t = [];
y = [];
for i = 1:numel(names)
    try
        [t, y] = localGetSignal(simSource, names{i});
        return;
    catch
    end
end
end

function [t, y] = localGetSignal(simSource, name)
if isa(simSource, 'Simulink.SimulationOutput')
    sig = simSource.get(name);
elseif isstruct(simSource) && isfield(simSource, name)
    sig = simSource.(name);
else
    error('Run_INDI_FrozenTrimHold_Test:MissingSignal', ...
        'Could not find signal "%s".', name);
end

if isa(sig, 'timeseries')
    t = sig.Time(:);
    y = squeeze(sig.Data);
else
    t = sig.time(:);
    y = squeeze(sig.signals.values);
end

if isempty(y)
    y = zeros(numel(t), 0);
elseif isvector(y)
    y = y(:);
elseif size(y, 1) ~= numel(t) && size(y, 2) == numel(t)
    y = y.';
end
end
