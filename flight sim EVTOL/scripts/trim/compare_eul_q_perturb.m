repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_EUL';
load_system(model);
propBlkName = findPropBlockName(model);

ac = aircraft_def();

speed = 70;
alphaDeg = 2.63250658095872;
thetaDeg = alphaDeg;
frontRPM = 1181.42797848053;
rearRPM = 0;
deltaADeg = 0;
deltaEDeg = -15.0602653566512;
deltaRDeg = 0;
bankDeg = 0;
cruiseTiltDeg = 90;

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'V_init', [speed * cosd(alphaDeg); 0; speed * sind(alphaDeg)]);
assignin('base', 'eul_init', [deg2rad(bankDeg); deg2rad(thetaDeg); 0]);

tempBlocks = {};
cleanupModel = onCleanup(@() cleanupTempBlocks(model, tempBlocks)); %#ok<NASGU>
tempBlocks{end+1} = addLogBlock(model, propBlkName, 2, 'tmp_propM');
tempBlocks{end+1} = addLogBlock(model, 'Analytic Aero Moment Sum', 1, 'tmp_aeroM');

fprintf('\nCompare EUL plant vs analytical model under q perturbation\n\n');
fprintf('%8s %14s %14s %14s %14s\n', 'q0', 'sim My_aero', 'ana My_aero', 'sim My_prop', 'ana My_prop');

for q0 = [-0.1 0 0.1]
    assignin('base', 'omega_init', [0; q0; 0]);

    ds = Simulink.SimulationData.Dataset;
    t = [0; 0.001];
    ds{1} = timeseries(repmat(zeros(12, 1), 1, 2).', t);
    ds{2} = timeseries(repmat(cruiseTiltDeg * ones(6, 1), 1, 2).', t);
    ds{3} = timeseries(repmat(frontRPM, 2, 1), t);
    ds{4} = timeseries(repmat(rearRPM, 2, 1), t);
    ds{5} = timeseries(repmat(0, 2, 1), t);
    ds{6} = timeseries(repmat(deg2rad(deltaADeg), 2, 1), t);
    ds{7} = timeseries(repmat(deg2rad(deltaEDeg), 2, 1), t);
    ds{8} = timeseries(repmat(deg2rad(deltaRDeg), 2, 1), t);

    simIn = Simulink.SimulationInput(model);
    simIn = simIn.setExternalInput(ds);
    simIn = simIn.setModelParameter('StopTime', '0.001', 'ReturnWorkspaceOutputs', 'on');
    simOut = sim(simIn);

    simPropM = lastVec(simOut.get('tmp_propM'));
    simAeroM = lastVec(simOut.get('tmp_aeroM'));

    state = struct();
    state.v_body = [speed * cosd(alphaDeg); 0; speed * sind(alphaDeg)];
    state.omega = [0; q0; 0];
    state.phi = deg2rad(bankDeg);
    state.theta = deg2rad(thetaDeg);
    state.psi = 0;

    [~, M_wingL] = evalSurface(ac.wingL, state, ac.CG, deg2rad(deltaADeg));
    [~, M_wingR] = evalSurface(ac.wingR, state, ac.CG, -deg2rad(deltaADeg));
    [~, M_tailL] = evalSurface(ac.tailL, state, ac.CG, deg2rad(deltaEDeg - deltaRDeg));
    [~, M_tailR] = evalSurface(ac.tailR, state, ac.CG, deg2rad(deltaEDeg + deltaRDeg));
    [~, M_frontR] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFR', ac.prop.hub_offset, ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
    [~, M_frontL] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFL', ac.prop.hub_offset, ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
    [~, M_rearR] = evalRearGroup(rearRPM * ones(3,1), ac.prop.posRR', ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
    [~, M_rearL] = evalRearGroup(rearRPM * ones(3,1), ac.prop.posRL', ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);

    anaAeroM = M_wingL + M_wingR + M_tailL + M_tailR;
    anaPropM = M_frontR + M_frontL + M_rearR + M_rearL;

    fprintf('%8.3f %14.6f %14.6f %14.6f %14.6f\n', q0, simAeroM(2), anaAeroM(2), simPropM(2), anaPropM(2));
end

function blockPath = addLogBlock(modelName, srcBlockName, srcPort, varName)
srcPath = [modelName '/' srcBlockName];
srcHandles = get_param(srcPath, 'PortHandles');
srcPortHandle = srcHandles.Outport(srcPort);
pos = get_param(srcPath, 'Position');
blockPath = sprintf('%s/%s_%s', modelName, srcBlockName, varName);
blockPath = strrep(blockPath, ' ', '_');
add_block('simulink/Sinks/To Workspace', blockPath, ...
    'VariableName', varName, ...
    'SaveFormat', 'Array', ...
    'Position', [pos(3)+80 pos(2)+40*srcPort pos(3)+170 pos(2)+40*srcPort+20]);
dstHandles = get_param(blockPath, 'PortHandles');
add_line(modelName, srcPortHandle, dstHandles.Inport(1), 'autorouting', 'on');
end

function cleanupTempBlocks(modelName, blockPaths)
if ~bdIsLoaded(modelName)
    return;
end
for k = 1:numel(blockPaths)
    if exist_block(blockPaths{k})
        delete_block(blockPaths{k});
    end
end
vars = {'tmp_propM','tmp_aeroM'};
for k = 1:numel(vars)
    evalin('base', sprintf('if exist(''%s'',''var''), clear(''%s''); end', vars{k}, vars{k}));
end
end

function tf = exist_block(blockPath)
try
    get_param(blockPath, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function name = findPropBlockName(modelName)
candidates = {'Propellers','Propellers1'};
name = '';
for i = 1:numel(candidates)
    if exist_block([modelName '/' candidates{i}])
        name = candidates{i};
        return;
    end
end
error('Could not locate propeller sum block in %s.', modelName);
end

function vec = lastVec(arr)
sz = size(arr);
if ndims(arr) == 3 && sz(1) == 3
    vec = squeeze(arr(:, 1, end));
elseif ismatrix(arr) && size(arr, 2) >= 3
    vec = arr(end, end-2:end).';
else
    vec = arr(:);
end
end

function [F_cg, M_cg] = evalSurface(surf, state, CG, deltaLocal)
v_body = state.v_body;
omega = state.omega;
if norm(v_body) < 0.1
    F_cg = zeros(3,1);
    M_cg = zeros(3,1);
    return;
end
r_arm = surf.pos - CG;
v_local = v_body + cross(omega, r_arm);
V2 = sum(v_local.^2);
v_mag = sqrt(V2);
v_dir = v_local / v_mag;
normal_proj = dot(v_dir, surf.n);
normal_proj = min(max(normal_proj, -1), 1);
alphaEff = surf.i - asin(normal_proj);
ctrl_tau = 0;
CM_delta = 0;
CD_delta2 = 0;
if isfield(surf, 'ctrl_tau'), ctrl_tau = surf.ctrl_tau; end
if isfield(surf, 'CM_delta'), CM_delta = surf.CM_delta; end
if isfield(surf, 'CD_delta2'), CD_delta2 = surf.CD_delta2; end
alphaEff = alphaEff + ctrl_tau * deltaLocal;
qS = surf.half_rho_S * V2;
CL = surf.CL0 + surf.CLa * alphaEff;
CD = surf.CD0 + surf.CDa * (alphaEff - surf.a0)^2 + CD_delta2 * deltaLocal^2;
CM = surf.CM0 + surf.CMa * alphaEff + CM_delta * deltaLocal;
L = qS * CL;
D = qS * CD;
dirD = -v_dir;
dirL = surf.n - dot(surf.n, v_dir) * v_dir;
if norm(dirL) > 0
    dirL = dirL / norm(dirL);
else
    dirL = surf.n;
end
Fsurf = L * dirL + D * dirD;
mAxis = cross(surf.n, v_dir);
if norm(mAxis) > 0
    mAxis = mAxis / norm(mAxis);
end
Msurf = (qS * surf.c * CM) * mAxis;
F_cg = Fsurf;
M_cg = Msurf + cross(r_arm, Fsurf);
end

function [F_cg, M_cg] = evalFrontGroup(rpms, tilt_deg, pivot_pos, hub_offset, spin_dir, kT, kQ, CG)
F_cg = zeros(3,1);
M_cg = zeros(3,1);
for i = 1:3
    n = [sind(tilt_deg(i)); 0; -cosd(tilt_deg(i))];
    r_hub = pivot_pos(:, i) + hub_offset * n;
    r_arm = r_hub - CG;
    T_mag = kT * rpms(i)^2;
    Q_mag = kQ * rpms(i)^2 * spin_dir(i);
    F_motor = T_mag * n;
    M_motor = -Q_mag * n;
    F_cg = F_cg + F_motor;
    M_cg = M_cg + cross(r_arm, F_motor) + M_motor;
end
end

function [F_cg, M_cg] = evalRearGroup(rpms, prop_pos, spin_dir, kT, kQ, CG)
F_cg = zeros(3,1);
M_cg = zeros(3,1);
n = [0; 0; -1];
for i = 1:3
    r_hub = prop_pos(:, i);
    r_arm = r_hub - CG;
    T_mag = kT * rpms(i)^2;
    Q_mag = kQ * rpms(i)^2 * spin_dir(i);
    F_motor = T_mag * n;
    M_motor = -Q_mag * n;
    F_cg = F_cg + F_motor;
    M_cg = M_cg + cross(r_arm, F_motor) + M_motor;
end
end
