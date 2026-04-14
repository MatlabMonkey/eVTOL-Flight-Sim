repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_EUL';
load_system(model);
stateBlocks = { ...
    [model '/Rotational Dynamics/Integrator3'], ...
    [model '/Translational Dynamics/Integrator3']};
oldAttrs = cell(size(stateBlocks));
for i = 1:numel(stateBlocks)
    oldAttrs{i} = get_param(stateBlocks{i}, 'ContinuousStateAttributes');
    set_param(stateBlocks{i}, 'ContinuousStateAttributes', '');
end
cleanupAttrs = onCleanup(@() restoreAttrs(stateBlocks, oldAttrs)); %#ok<NASGU>

speed = 70;
alpha0 = deg2rad(2.63250658095872);
theta0 = alpha0;
front0 = 1181.43;
rear0 = 0;
deTrimDeg = -15.0;

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'omega_init', [0; 0; 0]);
assignin('base', 'V_init', [speed * cos(alpha0); 0; speed * sin(alpha0)]);
assignin('base', 'eul_init', [0; theta0; 0]);

ds = Simulink.SimulationData.Dataset;
t = [0; 2.0];
ds{1} = timeseries(repmat(zeros(12,1), 1, 2).', t);
ds{2} = timeseries(repmat(90 * ones(6,1), 1, 2).', t);
ds{3} = timeseries(repmat(front0, 2, 1), t);
ds{4} = timeseries(repmat(rear0, 2, 1), t);
ds{5} = timeseries(repmat(0, 2, 1), t);
ds{6} = timeseries(repmat(0, 2, 1), t);
ds{7} = timeseries(repmat(deg2rad(deTrimDeg), 2, 1), t);
ds{8} = timeseries(repmat(0, 2, 1), t);

simIn = Simulink.SimulationInput(model);
simIn = simIn.setExternalInput(ds);
simIn = simIn.setModelParameter( ...
    'StopTime', '2.0', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'ReturnWorkspaceOutputs', 'on');

out = sim(simIn);
y = out.yout;
eul = y{4}.Values.Data;
omega = y{5}.Values.Data;
vinf = y{8}.Values.Data;
alpha = y{9}.Values.Data;

fprintf('\n2 s check from findop seed on %s\n', model);
fprintf('theta start/end deg : %.6f -> %.6f\n', rad2deg(eul(1,2)), rad2deg(eul(end,2)));
fprintf('q start/end rad/s   : %.6f -> %.6f\n', omega(1,2), omega(end,2));
fprintf('vinf start/end m/s  : %.6f -> %.6f\n', vinf(1), vinf(end));
fprintf('alpha start/end deg : %.6f -> %.6f\n', rad2deg(alpha(1)), rad2deg(alpha(end)));

assignin('base', 'findop_seed_check_eul', struct( ...
    'theta_deg', rad2deg(eul(:,2)), ...
    'q_rad_s', omega(:,2), ...
    'vinf_mps', vinf, ...
    'alpha_deg', rad2deg(alpha)));

close_system(model, 0);

function restoreAttrs(blocks, attrs)
for i = 1:numel(blocks)
    try
        if bdIsLoaded(bdroot(blocks{i}))
            set_param(blocks{i}, 'ContinuousStateAttributes', attrs{i});
        end
    catch
    end
end
end
