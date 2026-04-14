repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));

model = 'Brown_6DOF_Plant_zach';
load_system(model);

speed = 70;
alpha0 = deg2rad(2.63250658095872);
theta0 = alpha0;
front0 = 1181.42797848053;
rear0 = 0;
de_trim_deg = -15.0602653566512;

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'omega_init', [0; 0; 0]);
assignin('base', 'V_init', [speed * cos(alpha0); 0; speed * sin(alpha0)]);
assignin('base', 'eul_init', [0; theta0; 0]);

set_param(model, 'SimulationCommand', 'update');

tests = { ...
    'elev_p', 0, 0,  5,  0; ...
    'elev_m', 0, 0, -5,  0; ...
    'ail_p',  0, 5,  0,  0; ...
    'ail_m',  0,-5,  0,  0; ...
    'rud_p',  0, 0,  0,  5; ...
    'rud_m',  0, 0,  0, -5};

results = struct('name', {}, 'p_rad_s', {}, 'q_rad_s', {}, 'r_rad_s', {}, ...
    'phi_deg', {}, 'theta_deg', {}, 'psi_deg', {});

for k = 1:size(tests, 1)
    name = tests{k, 1};
    deltaF_deg = tests{k, 2};
    deltaA_deg = tests{k, 3};
    deltaE_deg = tests{k, 4};
    deltaR_deg = tests{k, 5};

    ds = Simulink.SimulationData.Dataset;
    t = [0; 0.2];
    ds{1} = timeseries(repmat(zeros(12,1), 1, 2).', t);
    ds{2} = timeseries(repmat(90 * ones(6,1), 1, 2).', t);
    ds{3} = timeseries(repmat(front0, 2, 1), t);
    ds{4} = timeseries(repmat(rear0, 2, 1), t);
    ds{5} = timeseries(repmat(deg2rad(deltaF_deg), 2, 1), t);
    ds{6} = timeseries(repmat(deg2rad(deltaA_deg), 2, 1), t);
    ds{7} = timeseries(repmat(deg2rad(de_trim_deg + deltaE_deg), 2, 1), t);
    ds{8} = timeseries(repmat(deg2rad(deltaR_deg), 2, 1), t);

    simIn = Simulink.SimulationInput(model);
    simIn = simIn.setExternalInput(ds);
    simIn = simIn.setModelParameter( ...
        'StopTime', '0.2', ...
        'SaveOutput', 'on', ...
        'OutputSaveName', 'yout', ...
        'ReturnWorkspaceOutputs', 'on');

    out = sim(simIn);
    y = out.yout;
    eul = y{4}.Values.Data(end, :);
    omg = y{5}.Values.Data(end, :);

    results(k).name = name;
    results(k).p_rad_s = omg(1);
    results(k).q_rad_s = omg(2);
    results(k).r_rad_s = omg(3);
    results(k).phi_deg = rad2deg(eul(1));
    results(k).theta_deg = rad2deg(eul(2));
    results(k).psi_deg = rad2deg(eul(3));
end

resultsTable = struct2table(results);
assignin('base', 'zach_control_sign_results', resultsTable);
disp(resultsTable);

close_system(model, 0);
