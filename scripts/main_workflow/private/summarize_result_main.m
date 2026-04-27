function summary = summarize_result_main(trimCtx, runCtx, simOut, err)
%SUMMARIZE_RESULT_MAIN Build a compact run summary for humans and scripts.

summary = struct();
summary.runModel = runCtx.runModel;
summary.trimSource = trimCtx.meta.source;
summary.vinfTrim = trimCtx.vinf;
summary.eulerDeg = rad2deg(trimCtx.euler(:)).';
summary.motorRPMsGrouped = trimCtx.motor_rpms_grouped(:).';
summary.tiltAnglesGroupedDeg = trimCtx.tilt_angles_grouped(:).';
summary.stopTime = runCtx.stopTime;
summary.stepTime = runCtx.stepTime;
summary.inputConvention = runCtx.inputConvention;
summary.simulationAttempted = logical(runCtx.runSimulation);
summary.simulationSucceeded = false;
summary.failureMessage = '';

if nargin >= 4 && ~isempty(err)
    summary.failureMessage = err.message;
end

if nargin >= 3 && ~isempty(simOut)
    summary.simulationSucceeded = true;
    try
        summary.executionWallTime = simOut.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
    catch
        summary.executionWallTime = NaN;
    end
else
    summary.executionWallTime = NaN;
end
end
