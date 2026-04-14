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
assignin('base', 'omega_init', [0; 0; 0]);

tempBlocks = {};
cleanupModel = onCleanup(@() cleanupTempBlocks(model, tempBlocks)); %#ok<NASGU>
tempBlocks{end+1} = addLogBlock(model, 'Add1', 1, 'tmp_totalForceNoG');
tempBlocks{end+1} = addLogBlock(model, 'Add', 1, 'tmp_totalMoment');

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

simVals.totalForceNoG = lastVec(simOut.get('tmp_totalForceNoG'));
simVals.totalMoment = lastVec(simOut.get('tmp_totalMoment'));
y = simOut.yout;
VB = y{2}.Values;
omegaTS = y{5}.Values;
specForce = y{11}.Values;
simVals.gravityBody = [NaN; NaN; NaN];
simVals.specificForce = lastVec(specForce.Data);
simVals.vdot_from_outputs = [NaN; NaN; NaN];
simVals.omegadot_from_moment = [NaN; NaN; NaN];

state = struct();
state.v_body = [speed * cosd(alphaDeg); 0; speed * sind(alphaDeg)];
state.omega = [0; 0; 0];
state.phi = deg2rad(bankDeg);
state.theta = deg2rad(thetaDeg);
state.psi = 0;

[F_wingL, M_wingL] = evalSurface(ac.wingL, state, ac.CG, deg2rad(deltaADeg));
[F_wingR, M_wingR] = evalSurface(ac.wingR, state, ac.CG, -deg2rad(deltaADeg));
[F_tailL, M_tailL] = evalSurface(ac.tailL, state, ac.CG, deg2rad(deltaEDeg - deltaRDeg));
[F_tailR, M_tailR] = evalSurface(ac.tailR, state, ac.CG, deg2rad(deltaEDeg + deltaRDeg));
[F_frontR, M_frontR] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFR', ac.prop.hub_offset, ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
[F_frontL, M_frontL] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFL', ac.prop.hub_offset, ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
[F_rearR, M_rearR] = evalRearGroup(rearRPM * ones(3,1), ac.prop.posRR', ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
[F_rearL, M_rearL] = evalRearGroup(rearRPM * ones(3,1), ac.prop.posRL', ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);

anaVals.totalForceNoG = F_wingL + F_wingR + F_tailL + F_tailR + F_frontR + F_frontL + F_rearR + F_rearL;
anaVals.totalMoment = M_wingL + M_wingR + M_tailL + M_tailR + M_frontR + M_frontL + M_rearR + M_rearL;

gvecN = [0; 0; ac.g];
CBN = angle2dcm(state.psi, state.theta, state.phi, 'ZYX')';
anaVals.gravityBody = CBN * gvecN;
anaVals.vdot = anaVals.gravityBody + anaVals.totalForceNoG / ac.Mass - cross(state.omega, state.v_body);
anaVals.Jomegadot = anaVals.totalMoment - cross(state.omega, ac.J * state.omega);
anaVals.omegadot = ac.J \ anaVals.Jomegadot;
anaVals.specificForce = anaVals.totalForceNoG / ac.Mass;
simVals.gravityBody = anaVals.gravityBody;
simVals.vdot_from_outputs = simVals.specificForce + simVals.gravityBody - cross(state.omega, state.v_body);
simVals.omegadot_from_moment = ac.J \ simVals.totalMoment;

fprintf('\nCompare EUL plant total loads at analytical 70 m/s trim point\n');
printCompare('Total force (no gravity)', simVals.totalForceNoG, anaVals.totalForceNoG);
printCompare('Specific force', simVals.specificForce, anaVals.specificForce);
printCompare('Gravity in body accel', simVals.gravityBody, anaVals.gravityBody);
printCompare('Vdot from logged loads', simVals.vdot_from_outputs, anaVals.vdot);
printCompare('Total moment', simVals.totalMoment, anaVals.totalMoment);
printCompare('omegadot from logged moment', simVals.omegadot_from_moment, anaVals.omegadot);

assignin('base', 'compare_eul_total_loads_trimpoint', struct('sim', simVals, 'analytical', anaVals));
close_system(model, 0);

function blockPath = addLogBlock(modelName, srcBlockName, srcPort, varName)
srcPath = [modelName '/' srcBlockName];
parentSys = get_param(srcPath, 'Parent');
srcHandles = get_param(srcPath, 'PortHandles');
srcPortHandle = srcHandles.Outport(srcPort);
pos = get_param(srcPath, 'Position');
leafName = strrep(get_param(srcPath, 'Name'), '/', '_');
leafName = strrep(leafName, ' ', '_');
blockPath = sprintf('%s/%s_%s', parentSys, leafName, varName);
add_block('simulink/Sinks/To Workspace', blockPath, ...
    'VariableName', varName, ...
    'SaveFormat', 'Array', ...
    'Position', [pos(3)+80 pos(2)+20*srcPort pos(3)+170 pos(2)+20*srcPort+20]);
dstHandles = get_param(blockPath, 'PortHandles');
add_line(parentSys, srcPortHandle, dstHandles.Inport(1), 'autorouting', 'on');
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
vars = {'tmp_totalForceNoG','tmp_totalMoment'};
for k = 1:numel(vars)
    evalin('base', sprintf('if exist(''%s'',''var''), clear(''%s''); end', vars{k}, vars{k}));
end
set_param(modelName, 'Dirty', 'off');
end

function tf = exist_block(blockPath)
try
    get_param(blockPath, 'Handle');
    tf = true;
catch
    tf = false;
end
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

function printCompare(label, simVec, anaVec)
fprintf('\n%s\n', label);
fprintf('  sim : [%.9f %.9f %.9f]\n', simVec(1), simVec(2), simVec(3));
fprintf('  ana : [%.9f %.9f %.9f]\n', anaVec(1), anaVec(2), anaVec(3));
fprintf('  diff: [%.9f %.9f %.9f]\n', simVec(1)-anaVec(1), simVec(2)-anaVec(2), simVec(3)-anaVec(3));
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
