function [animation, array6dof, simOut] = fg_replay_from_sim(cfg, runSimulation)
%FG_REPLAY_FROM_SIM Simulate the model and replay the result in FlightGear.

if nargin < 1 || isempty(cfg)
    cfg = fg_config();
end

if nargin < 2
    runSimulation = true;
end

fg_validate_config(cfg);

load_system(cfg.modelName);

if runSimulation
    simOut = sim(cfg.modelName);
else
    simOut = [];
end

positionLLA = evalin("base", cfg.positionLLAVariable);
eulerAngles = evalin("base", cfg.eulerVariable);
array6dof = fg_build_array6dof(positionLLA, eulerAngles);
animation = fg_play_array6dof(array6dof, cfg);
end
