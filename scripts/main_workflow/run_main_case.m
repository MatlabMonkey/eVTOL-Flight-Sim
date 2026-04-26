function result = run_main_case(varargin)
%RUN_MAIN_CASE Run one trimmed main-plant case from the new workflow.
%   result = RUN_MAIN_CASE()
%   result = RUN_MAIN_CASE('TrimSpec', struct('source','saved'))
%
% Purpose
%   This is the one-call entrypoint for single-case studies. It prepares
%   the session, resolves trim, adapts that trim to the main run model,
%   publishes the workspace variables the model expects, and then attempts
%   to simulate the main plant.
%
% Inputs
%   'Session'  : Optional session struct from prepare_main_session.
%   'TrimSpec' : Optional trim-spec struct.
%   'TrimCtx'  : Optional already-resolved trim context.
%   'RunSpec'  : Optional run-spec struct.
%
% Outputs
%   result : Struct containing simOut, trimCtx, runCtx, summary, and meta.
%
% Example
%   result = run_main_case( ...
%       'TrimSpec', struct('source', 'saved', 'cruiseSpeedMps', 70), ...
%       'RunSpec', struct('deltaEStep', deg2rad(1)));

p = inputParser;
p.addParameter('Session', struct(), @isstruct);
p.addParameter('TrimSpec', struct(), @isstruct);
p.addParameter('TrimCtx', struct(), @isstruct);
p.addParameter('RunSpec', struct(), @isstruct);
p.parse(varargin{:});
opts = p.Results;

if isempty(fieldnames(opts.Session))
    session = prepare_main_session();
else
    session = opts.Session;
end

if isempty(fieldnames(opts.TrimCtx))
    trimCtx = resolve_main_trim(session, opts.TrimSpec);
else
    trimCtx = opts.TrimCtx;
end

runSpec = default_run_spec_main(opts.RunSpec);
runCtx = adapt_trim_to_run_model(session, trimCtx, runSpec);
publish_main_workspace(runCtx);

result = struct();
result.simOut = [];
result.trimCtx = trimCtx;
result.runCtx = runCtx;
result.meta = struct( ...
    'runModel', session.runModel, ...
    'sessionKey', session.sessionKey, ...
    'runSimulation', runCtx.runSimulation, ...
    'failureStage', '');

if ~runCtx.runSimulation
    result.summary = summarize_result_main(trimCtx, runCtx, [], []);
    assignin('base', 'main_workflow_last_result', result);
    return;
end

simIn = localBuildSimulationInput(runCtx);

try
    set_param(session.runModel, 'SimulationCommand', 'update');
catch ME
    result.meta.failureStage = 'update';
    result.summary = summarize_result_main(trimCtx, runCtx, [], ME);
    assignin('base', 'main_workflow_last_result', result);
    error('run_main_case:RunModelUpdateFailed', ...
        ['The main run model could not update after the workflow published ' ...
         'the trimmed workspace context.\nModel: %s\nCause: %s\n' ...
         'The published run context is available in base workspace as ' ...
         'main_workflow_last_runCtx for debugging.'], ...
        session.runModel, ME.message);
end

try
    simOut = sim(simIn);
catch ME
    result.meta.failureStage = 'simulate';
    result.summary = summarize_result_main(trimCtx, runCtx, [], ME);
    assignin('base', 'main_workflow_last_result', result);
    error('run_main_case:SimulationFailed', ...
        ['The main run model failed during simulation.\nModel: %s\n' ...
         'Cause: %s\nInspect main_workflow_last_runCtx and ' ...
         'main_workflow_last_trimCtx in the base workspace for the exact ' ...
         'published setup.'], ...
        session.runModel, ME.message);
end

result.simOut = simOut;
result.summary = summarize_result_main(trimCtx, runCtx, simOut, []);
assignin('base', 'main_workflow_last_result', result);
end

function simIn = localBuildSimulationInput(runCtx)
simIn = Simulink.SimulationInput(runCtx.runModel);
simIn = simIn.setModelParameter( ...
    'StopTime', num2str(runCtx.stopTime), ...
    'ReturnWorkspaceOutputs', 'on');

switch lower(runCtx.inputConvention)
    case 'grouped'
        motorCmd = runCtx.commands.grouped.motor_rpm_cmd(:);
        tiltCmd = runCtx.commands.grouped.tilt_angles_cmd(:);
    case 'legacy'
        motorCmd = runCtx.commands.legacy.motor_rpm_cmd(:);
        tiltCmd = runCtx.commands.legacy.tilt_angles_cmd(:);
    otherwise
        error('run_main_case:UnknownInputConvention', ...
            'Unknown input convention ''%s''.', runCtx.inputConvention);
end

dataset = Simulink.SimulationData.Dataset;
dataset = dataset.addElement(localBuildSignal( ...
    'Motor_RPM_cmd', motorCmd, runCtx.stepCommands.motor_rpm_cmd(:), ...
    runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'Tilt_angles_cmd', tiltCmd, runCtx.stepCommands.tilt_angles_cmd(:), ...
    runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'Front_RPM_collective', runCtx.commands.front_collective_rpm, ...
    runCtx.stepCommands.front_collective_rpm, runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'Rear_RPM_collective1', runCtx.commands.rear_collective_rpm, ...
    runCtx.stepCommands.rear_collective_rpm, runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'delta_f', runCtx.commands.delta_f, runCtx.stepCommands.delta_f, ...
    runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'delta_a', runCtx.commands.delta_a, runCtx.stepCommands.delta_a, ...
    runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'delta_e', runCtx.commands.delta_e, runCtx.stepCommands.delta_e, ...
    runCtx.stepTime, runCtx.stopTime));
dataset = dataset.addElement(localBuildSignal( ...
    'delta_r', runCtx.commands.delta_r, runCtx.stepCommands.delta_r, ...
    runCtx.stepTime, runCtx.stopTime));

simIn = simIn.setExternalInput(dataset);
end

function signal = localBuildSignal(name, baseValue, stepValue, stepTime, stopTime)
times = [0; stopTime];
data = [baseValue(:).'; baseValue(:).'];

if any(abs(stepValue(:)) > 0)
    times = [0; stepTime; stopTime];
    steppedValue = baseValue(:) + stepValue(:);
    data = [baseValue(:).'; baseValue(:).'; steppedValue(:).'];
end

ts = timeseries(data, times);
ts.Name = name;
signal = Simulink.SimulationData.Signal;
signal.Name = name;
signal.Values = ts;
end
