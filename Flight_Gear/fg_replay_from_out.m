function [animation, array6dof] = fg_replay_from_out(outData, cfg)
%FG_REPLAY_FROM_OUT Replay FlightGear from an existing output object.

if nargin < 2 || isempty(cfg)
    cfg = fg_config();
end

if isnumeric(outData)
    array6dof = double(outData);
else
    [positionLLA, eulerAngles] = localExtractSignals(outData, cfg);
    array6dof = fg_build_array6dof(positionLLA, eulerAngles);
end

animation = fg_play_array6dof(array6dof, cfg);
end

function [positionLLA, eulerAngles] = localExtractSignals(outData, cfg)
if isa(outData, "Simulink.SimulationOutput")
    positionLLA = localGetSimulationOutput(outData, cfg.positionLLAVariable);
    eulerAngles = localGetSimulationOutput(outData, cfg.eulerVariable);
    return;
end

if isstruct(outData)
    if isfield(outData, cfg.positionLLAVariable) && isfield(outData, cfg.eulerVariable)
        positionLLA = outData.(cfg.positionLLAVariable);
        eulerAngles = outData.(cfg.eulerVariable);
        return;
    end
end

error("fg_replay_from_out:UnsupportedInput", ...
    ["Expected a Simulink.SimulationOutput, a struct with positionLLA/eul, " ...
     "or an Array6DoF matrix."]);
end

function value = localGetSimulationOutput(simOut, variableName)
try
    value = simOut.get(variableName);
catch
    error("fg_replay_from_out:MissingSignal", ...
        "Simulation output does not contain '%s'.", variableName);
end
end
