repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_zach';
load_system(model);

speed = 70;
alpha0 = deg2rad(2.63250658095872);
theta0 = alpha0;
front0 = 1181.42797848053;
rear0 = 0;
deTrimDeg = -15.0602653566512;

pos0 = [0; 0; -1000];
V0 = [speed * cos(alpha0); 0; speed * sin(alpha0)];

cases = { ...
    'baseline', 0, 0, deTrimDeg; ...
    'theta_p1', 1, 0, deTrimDeg; ...
    'theta_m1', -1, 0, deTrimDeg; ...
    'q_p05', 0, 0.05, deTrimDeg; ...
    'q_m05', 0, -0.05, deTrimDeg; ...
    'de_p2', 0, 0, deTrimDeg + 2; ...
    'de_m2', 0, 0, deTrimDeg - 2};

rows = struct('name', {}, 'q0', {}, 'theta0_deg', {}, 'q_0p1', {}, ...
    'q_0p5', {}, 'theta_0p5_deg', {}, 'alpha_0p5_deg', {}, ...
    'vinf_0p5', {}, 'beta_0p5_deg', {});

for k = 1:size(cases, 1)
    name = cases{k, 1};
    dthetaDeg = cases{k, 2};
    qInit = cases{k, 3};
    deDeg = cases{k, 4};

    assignin('base', 'pos_init', pos0);
    assignin('base', 'omega_init', [0; qInit; 0]);
    assignin('base', 'V_init', V0);
    assignin('base', 'eul_init', [0; theta0 + deg2rad(dthetaDeg); 0]);
    set_param(model, 'SimulationCommand', 'update');

    ds = Simulink.SimulationData.Dataset;
    t = [0; 0.5];
    ds{1} = timeseries(repmat(zeros(12,1), 1, 2).', t);
    ds{2} = timeseries(repmat(90 * ones(6,1), 1, 2).', t);
    ds{3} = timeseries(repmat(front0, 2, 1), t);
    ds{4} = timeseries(repmat(rear0, 2, 1), t);
    ds{5} = timeseries(repmat(0, 2, 1), t);
    ds{6} = timeseries(repmat(0, 2, 1), t);
    ds{7} = timeseries(repmat(deg2rad(deDeg), 2, 1), t);
    ds{8} = timeseries(repmat(0, 2, 1), t);

    simIn = Simulink.SimulationInput(model);
    simIn = simIn.setExternalInput(ds);
    simIn = simIn.setModelParameter( ...
        'StopTime', '0.5', ...
        'SaveOutput', 'on', ...
        'OutputSaveName', 'yout', ...
        'ReturnWorkspaceOutputs', 'on');

    out = sim(simIn);
    y = out.yout;
    tOmega = y{5}.Values.Time;
    omega = y{5}.Values.Data;
    eul = y{4}.Values.Data;
    vinf = y{8}.Values.Data;
    alpha = y{9}.Values.Data;
    beta = y{10}.Values.Data;

    idx01 = find(tOmega >= 0.1, 1, 'first');
    idx05 = find(tOmega >= 0.5, 1, 'first');

    rows(k).name = name;
    rows(k).q0 = qInit;
    rows(k).theta0_deg = rad2deg(theta0 + deg2rad(dthetaDeg));
    rows(k).q_0p1 = omega(idx01, 2);
    rows(k).q_0p5 = omega(idx05, 2);
    rows(k).theta_0p5_deg = rad2deg(eul(idx05, 2));
    rows(k).alpha_0p5_deg = rad2deg(alpha(idx05));
    rows(k).vinf_0p5 = vinf(idx05);
    rows(k).beta_0p5_deg = rad2deg(beta(idx05));
end

pitchInvestigation = struct2table(rows);
assignin('base', 'pitch_investigation_zach', pitchInvestigation);
disp(pitchInvestigation);

assignin('base', 'pos_init', pos0);
assignin('base', 'omega_init', [0; 0; 0]);
assignin('base', 'V_init', V0);
assignin('base', 'eul_init', [0; theta0; 0]);
set_param(model, 'SimulationCommand', 'update');

q0 = localEulerToQuat(0, theta0, 0);
x0 = [pos0; 0; 0; 0; V0; q0];
u0 = zeros(24, 1);
u0(13:18) = 90;
u0(19) = front0;
u0(20) = rear0;
u0(23) = deg2rad(deTrimDeg);

[A, B, C, D] = linmod(model, x0, u0); %#ok<ASGLU>

pitchLinear = table( ...
    A(5,5), A(5,7), A(5,9), A(7,5), A(9,5), ...
    'VariableNames', {'A_q_q', 'A_q_u', 'A_q_w', 'A_u_q', 'A_w_q'});
assignin('base', 'pitch_linear_zach', pitchLinear);
disp(pitchLinear);

close_system(model, 0);

function q = localEulerToQuat(phi, theta, psi)
cphi = cos(phi / 2); sphi = sin(phi / 2);
cth = cos(theta / 2); sth = sin(theta / 2);
cpsi = cos(psi / 2); spsi = sin(psi / 2);

q = [ ...
    cphi * cth * cpsi + sphi * sth * spsi; ...
    sphi * cth * cpsi - cphi * sth * spsi; ...
    cphi * sth * cpsi + sphi * cth * spsi; ...
    cphi * cth * spsi - sphi * sth * cpsi];
q = q / norm(q);
end
