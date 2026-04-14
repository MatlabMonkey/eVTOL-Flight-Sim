function linResults = homework7_linmod_manual_cases(varargin)
%HOMEWORK7_LINMOD_MANUAL_CASES Linearize the plant around manual trim seeds.

p = inputParser;
p.addParameter('ManualTrimFile', '', @(s) ischar(s) || isstring(s));
p.addParameter('RerunManualTrimIfMissing', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'homework7_trim');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
addpath(genpath(fullfile(repoRoot, 'scripts')));

trimFile = char(p.Results.ManualTrimFile);
if isempty(trimFile)
    trimFile = fullfile(outDir, 'homework7_manual_trim_results.mat');
end

if ~exist(trimFile, 'file')
    if p.Results.RerunManualTrimIfMissing
        homework7_manual_trim_search();
    else
        error('homework7_linmod_manual_cases:MissingManualTrimFile', ...
            'Could not find manual trim file: %s', trimFile);
    end
end

trimData = load(trimFile, 'results');
trimResults = trimData.results;

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
assignin('base', 'use_avl_aero', false);
build_control_analysis_helper();
load_system('Brown_6DOF_Plant');
load_system('Brown_Control_Analysis_Helper');

linResults = repmat(emptyLinStruct(), numel(trimResults), 1);
eigRows = [];

for k = 1:numel(trimResults)
    trimCase = trimResults(k);
    [x0, u0] = buildSeed(trimCase);
    [A, B, C, D] = linmod('Brown_Control_Analysis_Helper', x0, u0);
    eigVals = eig(A);

    linResults(k).case_name = char(trimCase.case_name);
    linResults(k).speed_mps = trimCase.speed_mps;
    linResults(k).bank_deg = trimCase.bank_deg;
    linResults(k).A = A;
    linResults(k).B = B;
    linResults(k).C = C;
    linResults(k).D = D;
    linResults(k).eig = eigVals;
    linResults(k).state_count = size(A,1);
    linResults(k).input_count = size(B,2);
    linResults(k).output_count = size(C,1);
    linResults(k).max_real_eig = max(real(eigVals));
    linResults(k).min_real_eig = min(real(eigVals));

    for i = 1:numel(eigVals)
        eigRows = [eigRows; {string(trimCase.case_name), trimCase.speed_mps, trimCase.bank_deg, i, real(eigVals(i)), imag(eigVals(i))}]; %#ok<AGROW>
    end
end

summaryTbl = struct2table(arrayfun(@summarizeLinResult, linResults));
eigTbl = cell2table(eigRows, 'VariableNames', ...
    {'case_name', 'speed_mps', 'bank_deg', 'eig_index', 'eig_real', 'eig_imag'});

writetable(summaryTbl, fullfile(outDir, 'homework7_linmod_manual_summary.csv'));
writetable(eigTbl, fullfile(outDir, 'homework7_linmod_manual_eigs.csv'));
save(fullfile(outDir, 'homework7_linmod_manual_results.mat'), 'linResults', 'summaryTbl', 'eigTbl', 'trimFile');

close_system('Brown_6DOF_Plant', 0);
close_system('Brown_Control_Analysis_Helper', 0);
end

function [x0, u0] = buildSeed(trimCase)
alphaRad = deg2rad(trimCase.alpha_deg);
betaRad = deg2rad(trimCase.beta_deg);
phiRad = deg2rad(trimCase.roll_deg);
thetaRad = deg2rad(trimCase.pitch_deg);
psiRad = deg2rad(trimCase.yaw_deg);
V = trimCase.speed_mps;

uBody = V * cos(alphaRad) * cos(betaRad);
vBody = V * sin(betaRad);
wBody = V * sin(alphaRad) * cos(betaRad);

quat = localEulerToQuat(phiRad, thetaRad, psiRad);
x0 = [[0; 0; -1000]; [trimCase.p_rad_s; trimCase.q_rad_s; trimCase.r_rad_s]; ...
    [uBody; vBody; wBody]; quat];

u0 = zeros(24, 1);
u0(13:18) = trimCase.front_tilt_deg;
u0(19) = trimCase.front_collective_rpm;
u0(20) = trimCase.rear_collective_rpm;
u0(21) = deg2rad(trimCase.delta_f_deg);
u0(22) = deg2rad(trimCase.delta_a_deg);
u0(23) = deg2rad(trimCase.delta_e_deg);
u0(24) = deg2rad(trimCase.delta_r_deg);
end

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

function out = summarizeLinResult(in)
out = struct();
out.case_name = string(in.case_name);
out.speed_mps = in.speed_mps;
out.bank_deg = in.bank_deg;
out.state_count = in.state_count;
out.input_count = in.input_count;
out.output_count = in.output_count;
out.max_real_eig = in.max_real_eig;
out.min_real_eig = in.min_real_eig;
end

function s = emptyLinStruct()
s = struct( ...
    'case_name', '', ...
    'speed_mps', nan, ...
    'bank_deg', nan, ...
    'A', [], ...
    'B', [], ...
    'C', [], ...
    'D', [], ...
    'eig', [], ...
    'state_count', nan, ...
    'input_count', nan, ...
    'output_count', nan, ...
    'max_real_eig', nan, ...
    'min_real_eig', nan);
end
