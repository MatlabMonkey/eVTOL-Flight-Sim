% Run_Main.m
% Prepare and optionally run the nonlinear main-plant simulation.
%
% Preferred usage:
%   Init_Main
%   trimCase = struct(...);
%   Trim_Main
%   runCase = struct('attemptSimulation', false);
%   Run_Main
%
% This script consumes:
%   - initData
%   - trimResult
%   - controllerData (optional)
%   - runCase (optional)
%
% It produces:
%   - runSpec     Plain numeric run definition used by the model
%   - runContext  Structured summary of what was pushed into the model
%   - runResult   Outcome summary
%   - simOut      Simulink output (if simulation succeeds)

if ~exist('initData', 'var') || ~isstruct(initData)
    error('initData is required. Run Init_Main before Run_Main.');
end

if ~exist('trimResult', 'var') || ~isstruct(trimResult)
    error(['trimResult is required. Run Trim_Main before Run_Main, ', ...
           'or load a saved point with load_trim_result_from_db.']);
end

if ~exist('runCase', 'var') || ~isstruct(runCase)
    runCase = localDefaultRunCase();
else
    runCase = localMergeRunCaseDefaults(runCase, localDefaultRunCase());
end

trimData = trimResult;

defaultRunSpec = initData.defaultRunSpec;
defaultRunSpec.name = runCase.name;
defaultRunSpec.modelName = initData.modelNames.run;
defaultRunSpec.pos_init = initData.initialState.pos_init;
defaultRunSpec.V_init = trimData.Vel_B_BA_Trim;
defaultRunSpec.airData_cmd = trimData.Vel_W_Trim;
defaultRunSpec.eul_init = trimData.Att_Trim;
defaultRunSpec.omega_init = trimData.Rates_Trim;
defaultRunSpec.Motor_RPMs = trimData.U_trim_full(1:4);
defaultRunSpec.Tilt_angles = trimData.U_trim_full(5:6);
defaultRunSpec.front_collective = trimData.U_trim_full(7);
defaultRunSpec.rear_collective = trimData.U_trim_full(8);
defaultRunSpec.surface_init = trimData.U_trim_full(9:12);
defaultRunSpec.useController = runCase.useController;
defaultRunSpec.attemptSimulation = runCase.attemptSimulation;
defaultRunSpec.stopTime = runCase.stopTime_s;
defaultRunSpec.eulerStepTime = runCase.eulerStepTime_s;
defaultRunSpec.eulerStepDeg = runCase.eulerStepDeg(:);
runSpec = defaultRunSpec;
runSpec = localApplyRunCaseOverrides(runSpec, runCase);

% Rebuild command histories for this run case, unless the run case already
% provides an explicit schedule package.
cmds = localMakeEvtolCmds(runSpec);
if isfield(runCase, 'cmds_override') && isstruct(runCase.cmds_override) && ...
        localHasRequiredCmdFields(runCase.cmds_override)
    cmds = runCase.cmds_override;
end

% Publish trim-aligned values that the current models and teammates'
% scripts may still read directly from the workspace.
pos_init = runSpec.pos_init;
V_init = runSpec.V_init;
eul_init = runSpec.eul_init;
omega_init = runSpec.omega_init;
Motor_RPMs = runSpec.Motor_RPMs;
Tilt_angles = runSpec.Tilt_angles;

front_collective_cmd = trimData.U_trim_full(7);
rear_collective_cmd = trimData.U_trim_full(8);
deltaLW_trim = trimData.U_trim_full(9);
deltaRW_trim = trimData.U_trim_full(10);
deltaLT_trim = trimData.U_trim_full(11);
deltaRT_trim = trimData.U_trim_full(12);

if isfield(runSpec, 'front_collective')
    front_collective_cmd = runSpec.front_collective;
end
if isfield(runSpec, 'rear_collective')
    rear_collective_cmd = runSpec.rear_collective;
end

surface_init = trimData.U_trim_full(9:12);
if isfield(runSpec, 'surface_init') && ~isempty(runSpec.surface_init)
    surface_init = runSpec.surface_init(:);
end

rotor_total_init = runSpec.Motor_RPMs(:) + [ ...
    front_collective_cmd; ...
    front_collective_cmd; ...
    rear_collective_cmd; ...
    rear_collective_cmd];

prop.rotor.init = rotor_total_init;
if numel(rotor_total_init) >= 4
    prop.rotor.init_FR = rotor_total_init(1);
    prop.rotor.init_FL = rotor_total_init(2);
    prop.rotor.init_RR = rotor_total_init(3);
    prop.rotor.init_RL = rotor_total_init(4);
end

prop.tilt.init = runSpec.Tilt_angles(:);
if numel(prop.tilt.init) >= 2
    prop.tilt.init_FR = prop.tilt.init(1);
    prop.tilt.init_FL = prop.tilt.init(2);
end

wingL.init = surface_init(1);
wingR.init = surface_init(2);
tailL.init = surface_init(3);
tailR.init = surface_init(4);

has_controller_data = exist('controllerData', 'var') && isstruct(controllerData);
controllerDataSafe = struct();
if has_controller_data
    controllerDataSafe = controllerData;
end

if runSpec.useController
    if ~has_controller_data
        error(['controllerData is required when runCase.useController is true. ', ...
               'Build controllerData with controllers/builders/build_corridor_lqr_controller, ', ...
               'or set runCase.useController=false.']);
    end
    controller_enable = true;
else
    controller_enable = false;
end

mixed_trim_lqr = localGetStructField(trimData, 'U_trim', []);
if isempty(mixed_trim_lqr)
    mixed_trim_lqr = [front_collective_cmd; zeros(4, 1)];
end

if controller_enable
    controller_mode = localGetStructField(controllerDataSafe, 'variant_ctrl_mode', 3);
else
    controller_mode = 1;
end

EVTOL = struct();
EVTOL.startTime = runSpec.startTime;
EVTOL.stopTime = runSpec.stopTime;
EVTOL.stepTime = runSpec.stepTime;
EVTOL.init = struct();
EVTOL.init.eul = eul_init;
EVTOL.init.vel = V_init;
EVTOL.init.rates = omega_init;
EVTOL.init.pos = pos_init;
EVTOL.init.quat = localEulerToQuat(eul_init);
EVTOL.gains = struct();
EVTOL.gains.mode = controller_mode;
if controller_enable
    EVTOL.gains.K_lqr = localGetStructField(controllerDataSafe, 'K_lqr_cruise', []);
    EVTOL.gains.x_trim_lqr = localGetStructField(controllerDataSafe, 'x_trim_lqr', []);
    EVTOL.gains.U_trim_lqr = localGetStructField(controllerDataSafe, 'U_trim_lqr', []);
else
    EVTOL.gains.K_lqr = [];
    EVTOL.gains.x_trim_lqr = [trimData.Att_Trim; trimData.Vel_B_BA_Trim; trimData.Rates_Trim];
    EVTOL.gains.U_trim_lqr = mixed_trim_lqr;
end

useCMDs = 1;
ctrlMode = controller_mode;
K_lqr_cruise = EVTOL.gains.K_lqr;
x_trim_lqr = EVTOL.gains.x_trim_lqr;
U_trim_lqr = EVTOL.gains.U_trim_lqr;
if isempty(K_lqr_cruise)
    K_lqr_cruise = zeros(5, 9);
end
if isempty(x_trim_lqr)
    x_trim_lqr = [trimData.Att_Trim; trimData.Vel_B_BA_Trim; trimData.Rates_Trim];
end
if isempty(U_trim_lqr)
    U_trim_lqr = mixed_trim_lqr;
end

controller_id = 0;
if controller_enable
    controller_id = localGetStructField(controllerDataSafe, 'controller_id', 1);
end
default_controller_state_ref = x_trim_lqr;
default_controller_trim_cmd = [ ...
    front_collective_cmd; ...
    rear_collective_cmd; ...
    U_trim_lqr(2); ...
    U_trim_lqr(3); ...
    U_trim_lqr(4); ...
    U_trim_lqr(5)];
controller_state_ref = localGetStructField(controllerDataSafe, 'controller_state_ref', default_controller_state_ref);
controller_trim_cmd = localGetStructField(controllerDataSafe, 'controller_trim_cmd', default_controller_trim_cmd);
controller_gain_lqr = localGetStructField(controllerDataSafe, 'controller_gain_lqr', K_lqr_cruise);
controller_state_ref = localPadControllerStateRef(controller_state_ref);
controller_trim_cmd = localPadControllerTrimCmd(controller_trim_cmd);
controller_gain_lqr = localPadControllerGain(controller_gain_lqr);

V_mem_init = runSpec.airData_cmd(:);
accel_mem_init = [0; 0; 0];

% Publish the wrapper-facing aliases explicitly so this works the same way
% whether the script is run from the command window, an editor action, or
% a function-based harness.
assignin('base', 'cmds', cmds);
assignin('base', 'EVTOL', EVTOL);
assignin('base', 'controller_mode', controller_mode);
assignin('base', 'ctrlMode', ctrlMode);
assignin('base', 'useCMDs', useCMDs);
assignin('base', 'K_lqr_cruise', K_lqr_cruise);
assignin('base', 'x_trim_lqr', x_trim_lqr);
assignin('base', 'U_trim_lqr', U_trim_lqr);
assignin('base', 'controller_id', controller_id);
assignin('base', 'controller_state_ref', controller_state_ref);
assignin('base', 'controller_trim_cmd', controller_trim_cmd);
assignin('base', 'controller_gain_lqr', controller_gain_lqr);
assignin('base', 'V_mem_init', V_mem_init);
assignin('base', 'accel_mem_init', accel_mem_init);
assignin('base', 'prop', prop);
assignin('base', 'wingL', wingL);
assignin('base', 'wingR', wingR);
assignin('base', 'tailL', tailL);
assignin('base', 'tailR', tailR);

runContext = struct();
runContext.modelName = runSpec.modelName;
runContext.pos_init = pos_init;
runContext.V_init = V_init;
runContext.eul_init = eul_init;
runContext.omega_init = omega_init;
runContext.front_collective_cmd = front_collective_cmd;
runContext.rear_collective_cmd = rear_collective_cmd;
runContext.surface_trim = [deltaLW_trim; deltaRW_trim; deltaLT_trim; deltaRT_trim];
runContext.surface_init = surface_init;
runContext.rotor_total_init = rotor_total_init;
runContext.airData_cmd = runSpec.airData_cmd(:);
runContext.useController = controller_enable;
runContext.cmds = cmds;
runContext.trimName = trimData.name;
runContext.trimMode = trimData.mode;
runContext.runCaseName = runCase.name;
runContext.EVTOL = EVTOL;

runResult = struct();
runResult.name = runSpec.name;
runResult.modelName = runSpec.modelName;
runResult.status = 'prepared';
runResult.message = 'Run context prepared but simulation not attempted.';
runResult.runSpec = runSpec;
runResult.runContext = runContext;
runResult.trimSummary = struct( ...
    'Att_Trim', trimData.Att_Trim, ...
    'Vel_B_BA_Trim', trimData.Vel_B_BA_Trim, ...
    'Rates_Trim', trimData.Rates_Trim, ...
    'U_trim', trimData.U_trim);

simOut = [];

if runSpec.attemptSimulation
    try
        load_system(runSpec.modelName);
        simOut = sim(runSpec.modelName, 'StopTime', num2str(runSpec.stopTime));
        out = simOut; %#ok<NASGU>
        assignin('base', 'out', out);
        runResult.status = 'simulated';
        runResult.message = 'Simulation completed successfully.';
    catch ME
        runResult.status = 'failed';
        runResult.message = sprintf('Simulation attempt failed: %s', ME.message);
        runResult.error = ME;
        warning('Run_Main:SimulationFailed', '%s', runResult.message);
    end
else
    fprintf('Run context prepared for %s. Simulation attempt skipped by runSpec.\n', runSpec.modelName);
end

function runCase = localDefaultRunCase()
runCase = struct();
runCase.name = 'PrepOnly_CurrentTrim';
runCase.useController = false;
runCase.attemptSimulation = false;
runCase.stopTime_s = 30.0;
runCase.eulerStepTime_s = inf;
runCase.eulerStepDeg = [0; 0; 0];
end

function runCase = localMergeRunCaseDefaults(runCase, defaults)
fields = fieldnames(defaults);
for i = 1:numel(fields)
    fieldName = fields{i};
    if ~isfield(runCase, fieldName) || isempty(runCase.(fieldName))
        runCase.(fieldName) = defaults.(fieldName);
    end
end
end

function cmds = localMakeEvtolCmds(runSpec)
%LOCALMAKEEVTOLCMDS Build simple From Workspace command traces.
%
% These traces are intentionally explicit and minimal. If the model input
% list changes, update this local function and the Simulink From Workspace
% blocks so they continue to plug directly into each other.

t = (runSpec.startTime:runSpec.stepTime:runSpec.stopTime).';
N = numel(t);
cmds = struct();
cmds.airData_cmd = [t, repmat(runSpec.airData_cmd(:).', N, 1)];

phi_cmd = runSpec.eul_init(1) * ones(N, 1);
theta_cmd = runSpec.eul_init(2) * ones(N, 1);
psi_cmd = runSpec.eul_init(3) * ones(N, 1);
phi_cmd(t >= runSpec.eulerStepTime) = phi_cmd(t >= runSpec.eulerStepTime) + deg2rad(runSpec.eulerStepDeg(1));
theta_cmd(t >= runSpec.eulerStepTime) = theta_cmd(t >= runSpec.eulerStepTime) + deg2rad(runSpec.eulerStepDeg(2));
psi_cmd(t >= runSpec.eulerStepTime) = psi_cmd(t >= runSpec.eulerStepTime) + deg2rad(runSpec.eulerStepDeg(3));
cmds.eul_cmd = [t, phi_cmd, theta_cmd, psi_cmd];

cmds.gps_Pos_cmd = [t, ...
    runSpec.pos_init(1) * ones(N, 1), ...
    runSpec.pos_init(2) * ones(N, 1), ...
    runSpec.pos_init(3) * ones(N, 1)];

cmds.omega_cmd = [t, ...
    runSpec.omega_init(1) * ones(N, 1), ...
    runSpec.omega_init(2) * ones(N, 1), ...
    runSpec.omega_init(3) * ones(N, 1)];

cmds.accel_cmd = [t, zeros(N, 3)];
cmds.motor_cmd = [t, repmat(runSpec.Motor_RPMs(:).', N, 1)];
cmds.tilt_cmd = [t, repmat(runSpec.Tilt_angles(:).', N, 1)];
cmds.front_cmd = [t, runSpec.front_collective * ones(N, 1)];
cmds.rear_cmd = [t, runSpec.rear_collective * ones(N, 1)];
end

function value = localGetStructField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function tf = localHasRequiredCmdFields(cmds)
required = {'airData_cmd', 'eul_cmd', 'gps_Pos_cmd', 'omega_cmd', 'accel_cmd', ...
    'motor_cmd', 'tilt_cmd', 'front_cmd', 'rear_cmd'};
tf = isstruct(cmds);
for i = 1:numel(required)
    if ~tf
        return;
    end
    tf = isfield(cmds, required{i});
end
end

function value = localPadControllerStateRef(in)
value = zeros(12, 20);
sz = size(in);
if isempty(in)
    return;
end
rows = min(sz(1), 12);
cols = min(sz(2), 20);
value(1:rows, 1:cols) = in(1:rows, 1:cols);
if cols < 20 && cols >= 1
    value(1:rows, cols+1:20) = repmat(value(1:rows, cols), 1, 20 - cols);
end
end

function value = localPadControllerTrimCmd(in)
value = zeros(6, 20);
sz = size(in);
if isempty(in)
    return;
end
rows = min(sz(1), 6);
cols = min(sz(2), 20);
value(1:rows, 1:cols) = in(1:rows, 1:cols);
if cols < 20 && cols >= 1
    value(1:rows, cols+1:20) = repmat(value(1:rows, cols), 1, 20 - cols);
end
end

function value = localPadControllerGain(in)
value = zeros(6, 9, 20);
if isempty(in)
    return;
end

if ndims(in) < 3
    rows = min(size(in, 1), 6);
    cols = min(size(in, 2), 9);
    value(1:rows, 1:cols, 1:20) = repmat(in(1:rows, 1:cols), 1, 1, 20);
    return;
end

rows = min(size(in, 1), 6);
cols = min(size(in, 2), 9);
pages = min(size(in, 3), 20);
value(1:rows, 1:cols, 1:pages) = in(1:rows, 1:cols, 1:pages);
if pages < 20 && pages >= 1
    value(1:rows, 1:cols, pages+1:20) = repmat(value(1:rows, 1:cols, pages), 1, 1, 20 - pages);
end
end

function runSpec = localApplyRunCaseOverrides(runSpec, runCase)
override_fields = { ...
    'startTime', ...
    'stopTime', ...
    'stepTime', ...
    'pos_init', ...
    'V_init', ...
    'airData_cmd', ...
    'eul_init', ...
    'omega_init', ...
    'Motor_RPMs', ...
    'Tilt_angles', ...
    'front_collective', ...
    'rear_collective', ...
    'surface_init'};

for i = 1:numel(override_fields)
    field_name = override_fields{i};
    if isfield(runCase, field_name) && ~isempty(runCase.(field_name))
        runSpec.(field_name) = runCase.(field_name);
    end
end
end

function quat = localEulerToQuat(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);

cphi = cos(phi / 2);
sphi = sin(phi / 2);
cthe = cos(theta / 2);
sthe = sin(theta / 2);
cpsi = cos(psi / 2);
spsi = sin(psi / 2);

quat = zeros(4, 1);
quat(1) = cphi * cthe * cpsi - sphi * sthe * spsi;
quat(2) = sphi * cthe * cpsi - cphi * sthe * spsi;
quat(3) = cphi * sthe * cpsi - sphi * cthe * spsi;
quat(4) = cphi * cthe * spsi - sphi * sthe * cpsi;
end
