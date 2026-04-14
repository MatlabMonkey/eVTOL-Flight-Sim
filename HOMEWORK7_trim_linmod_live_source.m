%% Homework 7 Trim and Linearization

clear; clc;
addpath(genpath(fullfile(pwd,'scripts')));

render_enable = false;
run('Full_Sim_Init.m');
use_avl_aero = false;

helperModel = build_homework7_trim_helper();
analysisModel = build_control_analysis_helper();
load_system(helperModel);
load_system('Brown_6DOF_Plant');
load_system(analysisModel);

warning('off','MATLAB:rankDeficientMatrix');
warning('off','MATLAB:nearlySingularMatrix');

cruiseTiltDeg = 90;
trimTol = 1e-4;
displayTol = 1e-8;
trimOptions = zeros(1,18);
trimOptions(14) = 5000;

%% Cases
caseNames = ["70 m/s, bank 0 deg"; "85 m/s, bank 0 deg"; "70 m/s, bank 5 deg"];
speeds = [70; 85; 70];
banks = [0; 0; 5];

u0Cases = [ ...
    deg2rad(2.6), 0, 0,      1180, 0, 0, deg2rad(-15), 0; ...
    deg2rad(0.2), 0, 0,      1300, 0, 0, deg2rad(-3),  0; ...
    deg2rad(2.6), 0, 0.0122, 1182, 0, 0, deg2rad(-15), deg2rad(0.05)];

trimSummary = table('Size',[3 11], ...
    'VariableTypes', {'string','logical','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'case_name','converged','max_output_residual','alpha_trim_deg','beta_trim_deg','r_trim_rad_s','delta_a_deg','delta_e_deg','delta_r_deg','front_collective_rpm','rear_collective_rpm'});

linSummary = table('Size',[3 6], ...
    'VariableTypes', {'string','double','double','double','double','double'}, ...
    'VariableNames', {'case_name','state_count','input_count','output_count','max_real_eig','min_real_eig'});

Afull = cell(3,1);
Aclassic = cell(3,1);
AclassicTables = cell(3,1);

%% Trim and linearize
for k = 1:3
    trim_target_speed_mps = speeds(k); %#ok<NASGU>
    trim_target_bank_deg = banks(k); %#ok<NASGU>
    trim_cruise_tilt_deg = cruiseTiltDeg; %#ok<NASGU>

    assignin('base','trim_target_speed_mps',trim_target_speed_mps);
    assignin('base','trim_target_bank_deg',trim_target_bank_deg);
    assignin('base','trim_cruise_tilt_deg',trim_cruise_tilt_deg);

    y0 = zeros(7,1);
    if banks(k) == 0
        iy = [1; 3; 5];
        iu = [2; 3; 5; 6; 8];
    else
        iy = (1:7).';
        iu = 2;
    end

    [~, uTrim, yTrim] = trim(helperModel, 0, u0Cases(k,:).', y0, 1, iu, iy, 0, 1, trimOptions);
    maxResidual = max(abs(yTrim(iy)));
    converged = maxResidual <= trimTol;

    [xPlant, uPlant] = localBuildPlantSeed(speeds(k), banks(k), uTrim, cruiseTiltDeg);
    [A, B, C, D] = linmod(analysisModel, xPlant, uPlant);
    eigVals = eig(A);

    trimSummary.case_name(k) = caseNames(k);
    trimSummary.converged(k) = converged;
    trimSummary.max_output_residual(k) = localZeroSmall(maxResidual, displayTol);
    trimSummary.alpha_trim_deg(k) = localZeroSmall(rad2deg(uTrim(1)), displayTol);
    trimSummary.beta_trim_deg(k) = localZeroSmall(rad2deg(uTrim(2)), displayTol);
    trimSummary.r_trim_rad_s(k) = localZeroSmall(uTrim(3), displayTol);
    trimSummary.delta_a_deg(k) = localZeroSmall(rad2deg(uTrim(6)), displayTol);
    trimSummary.delta_e_deg(k) = localZeroSmall(rad2deg(uTrim(7)), displayTol);
    trimSummary.delta_r_deg(k) = localZeroSmall(rad2deg(uTrim(8)), displayTol);
    trimSummary.front_collective_rpm(k) = localZeroSmall(uTrim(4), displayTol);
    trimSummary.rear_collective_rpm(k) = localZeroSmall(uTrim(5), displayTol);

    linSummary.case_name(k) = caseNames(k);
    linSummary.state_count(k) = size(A,1);
    linSummary.input_count(k) = size(B,2);
    linSummary.output_count(k) = size(C,1);
    linSummary.max_real_eig(k) = localZeroSmall(max(real(eigVals)), displayTol);
    linSummary.min_real_eig(k) = localZeroSmall(min(real(eigVals)), displayTol);

    Afull{k} = A;
    [Aclassic{k}, AclassicTables{k}] = localClassicAMatrix(A, banks(k), trimSummary.alpha_trim_deg(k));
end

trimSummary
linSummary

%% State meaning
% Full state order in the plant:
% [N E D p q r u v w q0 q1 q2 q3]
%
% For a cleaner aircraft-dynamics view, drop position and replace the
% quaternion part with local Euler-angle perturbations. The reduced state
% order below is:
% [u v w p q r phi theta]

AclassicTables{1}
AclassicTables{2}
AclassicTables{3}

close_system('Brown_6DOF_Plant',0);
close_system(helperModel,0);
close_system(analysisModel,0);

function [x0, u0] = localBuildPlantSeed(speedMps, bankDeg, uTrim, cruiseTiltDeg)
alphaRad = uTrim(1);
betaRad = uTrim(2);
rRadS = uTrim(3);

uBody = speedMps * cos(alphaRad) * cos(betaRad);
vBody = speedMps * sin(betaRad);
wBody = speedMps * sin(alphaRad) * cos(betaRad);

quat = localEulerToQuat(deg2rad(bankDeg), alphaRad, 0);
x0 = [[0;0;-1000]; [0;0;rRadS]; [uBody;vBody;wBody]; quat];

u0 = zeros(24,1);
u0(13:18) = cruiseTiltDeg;
u0(19) = uTrim(4);
u0(20) = uTrim(5);
u0(22) = uTrim(6);
u0(23) = uTrim(7);
u0(24) = uTrim(8);
end

function [Ared, Atbl] = localClassicAMatrix(Afull, bankDeg, alphaDeg)
Adyn = Afull(4:13,4:13);
JqEta = localQuatJacobian(deg2rad(bankDeg), deg2rad(alphaDeg), 0);

M = zeros(10,9);
M(1,4) = 1;
M(2,5) = 1;
M(3,6) = 1;
M(4,1) = 1;
M(5,2) = 1;
M(6,3) = 1;
M(7:10,7:9) = JqEta;

Aeuler = pinv(M) * Adyn * M;
keep = 1:8;
Ared = Aeuler(keep,keep);
Ared(abs(Ared) < 1e-8) = 0;

labels = {'u','v','w','p','q','r','phi','theta'};
Atbl = array2table(round(Ared,4), 'VariableNames', labels, 'RowNames', labels);
end

function J = localQuatJacobian(phi, theta, psi)
epsVal = 1e-6;
eta0 = [phi; theta; psi];
J = zeros(4,3);
for i = 1:3
    d = zeros(3,1);
    d(i) = epsVal;
    qp = localEulerToQuat(eta0(1)+d(1), eta0(2)+d(2), eta0(3)+d(3));
    qm = localEulerToQuat(eta0(1)-d(1), eta0(2)-d(2), eta0(3)-d(3));
    J(:,i) = (qp - qm) / (2*epsVal);
end
end

function q = localEulerToQuat(phi, theta, psi)
cphi = cos(phi/2); sphi = sin(phi/2);
cth = cos(theta/2); sth = sin(theta/2);
cpsi = cos(psi/2); spsi = sin(psi/2);

q = [ ...
    cphi*cth*cpsi + sphi*sth*spsi; ...
    sphi*cth*cpsi - cphi*sth*spsi; ...
    cphi*sth*cpsi + sphi*cth*spsi; ...
    cphi*cth*spsi - sphi*sth*cpsi];
q = q / norm(q);
end

function x = localZeroSmall(x, tol)
if abs(x) < tol
    x = 0;
end
end
