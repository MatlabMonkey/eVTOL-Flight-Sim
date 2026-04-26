%% Quick validation for LUTaeroTest
% This script initializes the LUT-based surface aero block, runs a few
% simple cases through the Simulink test model, and compares the block
% output against the same lookup-table math evaluated directly in MATLAB.

origDir = pwd;
cleanupDir = onCleanup(@() cd(origDir)); %#ok<NASGU>

stack = dbstack('-completenames');
if ~isempty(stack)
    scriptDir = fileparts(stack(1).file);
else
    scriptDir = pwd;
end
repoRoot = fileparts(fileparts(scriptDir));
cd(repoRoot);

addpath(repoRoot);
addpath(genpath(fullfile(repoRoot, 'scripts')));

model = 'LUTaeroTest';
polarMat = fullfile(repoRoot, 'scripts', 'avl', 'generated', 'final_airfoil_polar_tables.mat');
builderScript = fullfile(repoRoot, 'scripts', 'avl', 'build_final_airfoil_polar_tables.m');

if exist(polarMat, 'file') ~= 2
    run(builderScript);
end

polarData = load(polarMat, 'wingPolar', 'tailPolar');
aircraft = aircraft_def('flight_mode', 0);

surfaceChoice = 'wingR';  % change to wingL / wingR / tailL / tailR if needed
switch surfaceChoice
    case 'wingL'
        surf = aircraft.wingL;
        polar = polarData.wingPolar;
    case 'wingR'
        surf = aircraft.wingR;
        polar = polarData.wingPolar;
    case 'tailL'
        surf = aircraft.tailL;
        polar = polarData.tailPolar;
    case 'tailR'
        surf = aircraft.tailR;
        polar = polarData.tailPolar;
    otherwise
        error('Unknown surfaceChoice "%s".', surfaceChoice);
end

if isfield(surf, 'name')
    surf = rmfield(surf, 'name');
end

cases = [ ...
    struct('name', 'cruise_trim_like', ...
           'v_body', [70; 0; 0], ...
           'omega', [0; 0; 0], ...
           'delta_ctrl', 0.0), ...
    struct('name', 'positive_deflection', ...
           'v_body', [70; 0; 0], ...
           'omega', [0; 0; 0], ...
           'delta_ctrl', deg2rad(10.0)), ...
    struct('name', 'near_zero_flow', ...
           'v_body', [0; 0; 0], ...
           'omega', [0; 0; 0], ...
           'delta_ctrl', 0.0) ...
    ];

load_system('Brown_Flight_Controls_lib');
load_system(model);

lutBlk = [model '/LUT Aero'];
linkStatus = '';
try
    linkStatus = get_param(lutBlk, 'StaticLinkStatus');
catch
end
if strcmpi(linkStatus, 'unresolved')
    error(['%s is still an unresolved library link.\n' ...
        'Fix the test model first by either:\n' ...
        '  1. saving the LUT Aero block into Brown_Flight_Controls_lib, or\n' ...
        '  2. disabling the library link so LUTaeroTest contains a local subsystem.\n' ...
        'Then rerun this script.'], lutBlk);
end

set_param(lutBlk, ...
    'surf', surfaceChoice, ...
    'CG', 'CG', ...
    'polar', 'polar');

fprintf('\nTesting %s using surface "%s"\n', model, surfaceChoice);
fprintf('Polar source: %s\n\n', polar.airfoil);

results = repmat(struct(), numel(cases), 1);

for k = 1:numel(cases)
    thisCase = cases(k);

    [F_ref, M_ref, debugRef] = localReferenceEval( ...
        thisCase.v_body, thisCase.omega, thisCase.delta_ctrl, surf, aircraft.CG, polar);

    ds = Simulink.SimulationData.Dataset;
    t = [0; 0.05];
    ds = ds.addElement(timeseries(repmat(thisCase.v_body.', numel(t), 1), t), 'In1');
    ds = ds.addElement(timeseries(repmat(thisCase.omega.', numel(t), 1), t), 'In2');
    ds = ds.addElement(timeseries(repmat(thisCase.delta_ctrl, numel(t), 1), t), 'In3');

    simIn = Simulink.SimulationInput(model);
    simIn = simIn.setVariable('CG', aircraft.CG);
    simIn = simIn.setVariable('cg', aircraft.CG);
    simIn = simIn.setVariable('polar', polar);
    simIn = simIn.setVariable('surface', surf);
    simIn = simIn.setVariable(surfaceChoice, surf);
    simIn = simIn.setExternalInput(ds);
    simIn = simIn.setModelParameter( ...
        'StopTime', '0.05', ...
        'SaveOutput', 'on', ...
        'OutputSaveName', 'yout', ...
        'SaveFormat', 'Dataset', ...
        'SaveTime', 'on', ...
        'TimeSaveName', 'tout');

    simOut = sim(simIn);
    yout = simOut.yout;

    F_sim = localLastVector(yout{1}.Values.Data);
    M_sim = localLastVector(yout{2}.Values.Data);

    results(k).name = thisCase.name;
    results(k).F_ref = F_ref;
    results(k).F_sim = F_sim;
    results(k).M_ref = M_ref;
    results(k).M_sim = M_sim;
    results(k).F_err = norm(F_sim - F_ref);
    results(k).M_err = norm(M_sim - M_ref);
    results(k).alpha_geom = debugRef.alpha_geom;
    results(k).alpha_eff = debugRef.alpha_eff;
    results(k).CL = debugRef.CL;
    results(k).CD = debugRef.CD;
    results(k).Cm = debugRef.Cm;

    fprintf('Case: %s\n', thisCase.name);
    fprintf('  alpha_geom = % .6f rad\n', debugRef.alpha_geom);
    fprintf('  alpha_eff  = % .6f rad\n', debugRef.alpha_eff);
    fprintf('  CL/CD/Cm   = [% .6f, % .6f, % .6f]\n', debugRef.CL, debugRef.CD, debugRef.Cm);
    fprintf('  ||F_sim - F_ref|| = %.6e\n', results(k).F_err);
    fprintf('  ||M_sim - M_ref|| = %.6e\n\n', results(k).M_err);
end

assignin('base', 'lutAeroTestResults', results);
fprintf('Saved results to workspace variable: lutAeroTestResults\n');

function vec = localLastVector(data)
if isempty(data)
    vec = [];
    return;
end

if size(data, 1) > 1
    slice = squeeze(data(end, :, :));
else
    slice = squeeze(data);
end

vec = slice(:);
end

function [F_cg, M_cg, debugOut] = localReferenceEval(v_body, omega, delta_ctrl, surf, CG, polar)
[alpha_geom, alpha_eff, qS, dirL, dirD, mAxis, r_arm, valid_flow] = ...
    localPrepare(v_body, omega, delta_ctrl, surf, CG);

CL = localLookup(alpha_eff, polar.alpha_rad, polar.CL);
CD = localLookup(alpha_eff, polar.alpha_rad, polar.CD);
Cm = localLookup(alpha_eff, polar.alpha_rad, polar.Cm);

[F_cg, M_cg] = localFinish(CL, CD, Cm, qS, dirL, dirD, mAxis, r_arm, delta_ctrl, valid_flow, surf);

debugOut = struct( ...
    'alpha_geom', alpha_geom, ...
    'alpha_eff', alpha_eff, ...
    'CL', CL, ...
    'CD', CD, ...
    'Cm', Cm);
end

function val = localLookup(alpha_eff, alphaGrid, tableData)
alpha_wrapped = mod(alpha_eff + pi, 2*pi) - pi;
val = interp1(alphaGrid, tableData, alpha_wrapped, 'linear', 'extrap');
end

function [alpha_geom, alpha_eff, qS, dirL, dirD, mAxis, r_arm, valid_flow] = ...
    localPrepare(v_body, omega, delta_ctrl, surf, CG)
alpha_geom = 0.0;
alpha_eff = 0.0;
qS = 0.0;
dirL = zeros(3, 1);
dirD = zeros(3, 1);
mAxis = zeros(3, 1);
r_arm = surf.pos - CG;
valid_flow = false;

v_local = v_body + cross(omega, r_arm);
V2 = sum(v_local.^2);
if V2 < 1.0e-2
    return;
end

v_mag = sqrt(V2);
v_dir = v_local / v_mag;
normal_proj = dot(v_dir, surf.n);
normal_proj = min(max(normal_proj, -1.0), 1.0);
alpha_geom = surf.i - asin(normal_proj);

ctrl_tau = 0.0;
if isfield(surf, 'ctrl_tau') && ~isempty(surf.ctrl_tau)
    ctrl_tau = surf.ctrl_tau;
end
alpha_eff = alpha_geom + ctrl_tau * delta_ctrl;

qS = surf.half_rho_S * V2;

dirD = -v_dir;
dirL = surf.n - dot(surf.n, v_dir) * v_dir;
dirL_norm = norm(dirL);
if dirL_norm > 0.0
    dirL = dirL / dirL_norm;
else
    dirL = surf.n;
end

mAxis = cross(surf.n, v_dir);
mAxis_norm = norm(mAxis);
if mAxis_norm > 0.0
    mAxis = mAxis / mAxis_norm;
else
    mAxis = zeros(3, 1);
end

valid_flow = true;
end

function [F_cg, M_cg] = localFinish(CL, CD, Cm, qS, dirL, dirD, mAxis, r_arm, delta_ctrl, valid_flow, surf)
F_cg = zeros(3, 1);
M_cg = zeros(3, 1);

if ~valid_flow || qS <= 0.0
    return;
end

CD_delta2 = 0.0;
CM_delta = 0.0;
if isfield(surf, 'CD_delta2') && ~isempty(surf.CD_delta2)
    CD_delta2 = surf.CD_delta2;
end
if isfield(surf, 'CM_delta') && ~isempty(surf.CM_delta)
    CM_delta = surf.CM_delta;
end

CD = CD + CD_delta2 * delta_ctrl^2;
Cm = Cm + CM_delta * delta_ctrl;

L = qS * CL;
D = qS * CD;
Fsurf = L * dirL + D * dirD;
Msurf = (qS * surf.c * Cm) * mAxis;

F_cg = Fsurf;
M_cg = Msurf + cross(r_arm, Fsurf);
end
