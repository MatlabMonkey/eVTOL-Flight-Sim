repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_EUL';

speed = 70;
alpha0 = deg2rad(2.63250658095872);
theta0 = alpha0;
front0 = 1181.42797848053;
rear0 = 0;
deTrimDeg = -15.0602653566512;

pos0 = [0; 0; -1000];
V0 = [speed * cos(alpha0); 0; speed * sin(alpha0)];
eul0 = [0; theta0; 0];
omega0 = [0; 0; 0];

assignin('base', 'pos_init', pos0);
assignin('base', 'V_init', V0);
assignin('base', 'eul_init', eul0);
assignin('base', 'omega_init', omega0);

load_system(model);
x0 = [eul0; pos0; omega0; V0];
u0 = zeros(24, 1);
u0(13:18) = 90;
u0(19) = front0;
u0(20) = rear0;
u0(23) = deg2rad(deTrimDeg);
[A, ~, ~, ~] = linmod(model, x0, u0);
close_system(model, 0);

eigVals = eig(A);
eigTable = array2table(sortrows([real(eigVals), imag(eigVals)], 1), ...
    'VariableNames', {'real_part', 'imag_part'});

tests = [-6; 0; 6];
rows = struct('delta_e_offset_deg', {}, 'q_end_rad_s', {}, ...
    'theta_end_deg', {}, 'alpha_end_deg', {}, 'vinf_end_mps', {});

for k = 1:numel(tests)
    rows(k) = runElevatorCase(model, pos0, V0, eul0, omega0, ...
        front0, rear0, deTrimDeg, tests(k));
end

pitchInvestigationEUL = struct();
pitchInvestigationEUL.seed = struct( ...
    'speed_mps', speed, ...
    'alpha_deg', rad2deg(alpha0), ...
    'theta_deg', rad2deg(theta0), ...
    'front_collective_rpm', front0, ...
    'rear_collective_rpm', rear0, ...
    'delta_e_deg', deTrimDeg);
pitchInvestigationEUL.eigenvalues = eigTable;
pitchInvestigationEUL.positive_real_count = sum(real(eigVals) > 1e-6);
pitchInvestigationEUL.elevator_sweep = struct2table(rows);

assignin('base', 'pitch_investigation_eul', pitchInvestigationEUL);

fprintf('\nPitch stability check for %s\n', model);
fprintf('Positive-real eigenvalues near 70 m/s seed: %d\n', pitchInvestigationEUL.positive_real_count);
disp(pitchInvestigationEUL.eigenvalues);
disp(pitchInvestigationEUL.elevator_sweep);

function row = runElevatorCase(model, pos0, V0, eul0, omega0, front0, rear0, deTrimDeg, deltaEDeg)
assignin('base', 'use_avl_aero', false);
assignin('base', 'pos_init', pos0);
assignin('base', 'V_init', V0);
assignin('base', 'eul_init', eul0);
assignin('base', 'omega_init', omega0);

load_system(model);

ds = Simulink.SimulationData.Dataset;
t = [0; 0.2];
ds{1} = timeseries(repmat(zeros(12, 1), 1, 2).', t);
ds{2} = timeseries(repmat(90 * ones(6, 1), 1, 2).', t);
ds{3} = timeseries(repmat(front0, 2, 1), t);
ds{4} = timeseries(repmat(rear0, 2, 1), t);
ds{5} = timeseries(repmat(0, 2, 1), t);
ds{6} = timeseries(repmat(0, 2, 1), t);
ds{7} = timeseries(repmat(deg2rad(deTrimDeg + deltaEDeg), 2, 1), t);
ds{8} = timeseries(repmat(0, 2, 1), t);

simIn = Simulink.SimulationInput(model);
simIn = simIn.setExternalInput(ds);
simIn = simIn.setModelParameter( ...
    'StopTime', '0.2', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'ReturnWorkspaceOutputs', 'on');

out = sim(simIn);
close_system(model, 0);

y = out.yout;
omega = y{5}.Values.Data;
eul = y{4}.Values.Data;
alpha = y{9}.Values.Data;
vinf = y{8}.Values.Data;

row = struct();
row.delta_e_offset_deg = deltaEDeg;
row.q_end_rad_s = omega(end, 2);
row.theta_end_deg = rad2deg(eul(end, 2));
row.alpha_end_deg = rad2deg(alpha(end));
row.vinf_end_mps = vinf(end);
end
