% scripts/smoke_test.m
% Headless-friendly minimal sanity check for eVTOL-Flight-Sim.

fprintf('== eVTOL-Flight-Sim smoke test ==\n');
fprintf('Timestamp: %s\n', datestr(now, 31));

all_ok = true;

requiredFiles = {
    'Full_Sim_Init.m'
    'Brown_Full_Sim.slx'
    'Brown_Flight_Controls_lib.slx'
};

for i = 1:numel(requiredFiles)
    f = requiredFiles{i};
    if isfile(f)
        fprintf('[PASS] Found required file: %s\n', f);
    else
        fprintf('[FAIL] Missing required file: %s\n', f);
        all_ok = false;
    end
end

% Initialization script check
try
    fprintf('[INFO] Running Full_Sim_Init.m ...\n');
    run('Full_Sim_Init.m');
    fprintf('[PASS] Full_Sim_Init.m executed successfully\n');
catch ME
    fprintf('[FAIL] Full_Sim_Init.m failed: %s\n', ME.message);
    all_ok = false;
end

% Headless model load/update check (no GUI rendering required)
modelName = 'Brown_Full_Sim';
try
    fprintf('[INFO] Loading model %s ...\n', modelName);
    load_system(modelName);

    fprintf('[INFO] Updating/compiling model %s ...\n', modelName);
    set_param(modelName, 'SimulationCommand', 'update');

    fprintf('[PASS] Model %s loaded and updated successfully\n', modelName);
catch ME
    fprintf('[FAIL] Model sanity check failed for %s: %s\n', modelName, ME.message);
    all_ok = false;
end

% Clean up open systems from this script context
try
    bdclose('all');
catch
    % no-op
end

if all_ok
    fprintf('SMOKE_TEST: PASS\n');
else
    fprintf('SMOKE_TEST: FAIL\n');
    error('Smoke test failed. See log above for details.');
end
