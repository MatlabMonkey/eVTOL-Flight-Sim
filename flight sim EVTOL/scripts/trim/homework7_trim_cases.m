function results = homework7_trim_cases(varargin)
%HOMEWORK7_TRIM_CASES Run the three Homework 7 trim cases with trim().
%   RESULTS = HOMEWORK7_TRIM_CASES() trims a grouped-input helper model
%   that uses the current aircraft force/moment equations, then maps the
%   resulting operating points back onto Brown_6DOF_Plant for linmod().

p = inputParser;
p.addParameter('LevelSpeeds', [70 85], @(x) isnumeric(x) && numel(x) == 2);
p.addParameter('BankSpeed', 70, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BankDeg', 5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('CruiseTiltDeg', 90, @(x) isnumeric(x) && isscalar(x));
p.addParameter('UseAVLAero', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('AllowFrontRPMTrim', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('MaxFunctionEvals', 5000, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('TrimTol', 1e-4, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.parse(varargin{:});
opts = p.Results;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'homework7_trim');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
addpath(genpath(fullfile(repoRoot, 'scripts')));

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
assignin('base', 'use_avl_aero', logical(opts.UseAVLAero));
helperModel = build_homework7_trim_helper();
load_system(helperModel);
load_system('Brown_6DOF_Plant');

caseDefs = [ ...
    struct('name', sprintf('level_%g', opts.LevelSpeeds(1)), 'speed_mps', opts.LevelSpeeds(1), 'bank_deg', 0), ...
    struct('name', sprintf('level_%g', opts.LevelSpeeds(2)), 'speed_mps', opts.LevelSpeeds(2), 'bank_deg', 0), ...
    struct('name', sprintf('bank_%gdeg_%g', opts.BankDeg, opts.BankSpeed), 'speed_mps', opts.BankSpeed, 'bank_deg', opts.BankDeg)];

seedReport = homework7_manual_trim_search( ...
    'LevelSpeeds', opts.LevelSpeeds, ...
    'BankSpeed', opts.BankSpeed, ...
    'BankDeg', opts.BankDeg, ...
    'CruiseTiltDeg', opts.CruiseTiltDeg, ...
    'WriteFiles', false);

results = repmat(emptyResultStruct(), numel(caseDefs), 1);
for k = 1:numel(caseDefs)
    results(k) = runSingleCase(caseDefs(k), opts, seedReport.results, helperModel);
end

summaryTbl = struct2table(arrayfun(@summarizeResult, results));
summaryCsv = fullfile(outDir, 'homework7_trim_summary.csv');
writetable(summaryTbl, summaryCsv);

meta = struct();
meta.created_utc = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
meta.model = 'Brown_6DOF_Plant';
meta.trim_helper_model = helperModel;
meta.trim_method = 'grouped_helper_trim';
meta.use_avl_aero = logical(opts.UseAVLAero);
meta.allow_front_rpm_trim = logical(opts.AllowFrontRPMTrim);
meta.cruise_tilt_deg = opts.CruiseTiltDeg;
meta.trim_tol = opts.TrimTol;
meta.max_function_evals = opts.MaxFunctionEvals;

resultsMat = fullfile(outDir, 'homework7_trim_results.mat');
save(resultsMat, 'results', 'summaryTbl', 'meta');

fprintf('Homework 7 trim results saved to:\n  %s\n  %s\n', resultsMat, summaryCsv);

close_system('Brown_6DOF_Plant', 0);
close_system(helperModel, 0);
end

function result = runSingleCase(caseDef, opts, manualSeeds, helperModel)
seed = pickManualSeed(caseDef, manualSeeds);

assignin('base', 'trim_target_speed_mps', caseDef.speed_mps);
assignin('base', 'trim_target_bank_deg', caseDef.bank_deg);
assignin('base', 'trim_cruise_tilt_deg', opts.CruiseTiltDeg);

u0 = helperSeedVector(seed);
[y0, iy, iu] = helperTrimSetup(caseDef.bank_deg, logical(opts.AllowFrontRPMTrim));

trimOptions = zeros(1, 18);
trimOptions(14) = opts.MaxFunctionEvals;

trimError = '';
try
    [xTrim, uTrim, yTrim, dxTrim, optOut] = trim(helperModel, ...
        0, u0, y0, 1, iu, iy, 0, 1, trimOptions);
catch ME
    xTrim = 0;
    uTrim = u0;
    yTrim = nan(7, 1);
    dxTrim = 0;
    optOut = trimOptions;
    trimError = ME.message;
end

grouped = unpackHelperInputs(uTrim, caseDef, opts.CruiseTiltDeg);
[fullX, fullU] = buildFullPlantSeed(grouped, opts.CruiseTiltDeg);

result = emptyResultStruct();
result.case_name = caseDef.name;
result.speed_mps = caseDef.speed_mps;
result.bank_deg = caseDef.bank_deg;
result.use_avl_aero = logical(opts.UseAVLAero);
result.allow_front_rpm_trim = logical(opts.AllowFrontRPMTrim);
result.cruise_tilt_deg = opts.CruiseTiltDeg;
result.alpha_seed_deg = seed.alpha_deg;
result.delta_e_seed_deg = seed.delta_e_deg;
result.initial_x = [];
result.initial_u = u0;
result.helper_x_trim = xTrim;
result.helper_u_trim = uTrim;
result.x_trim = fullX;
result.u_trim = fullU;
result.y0 = y0;
result.y_trim = yTrim;
result.iy = iy;
result.iu = iu;
result.dx0 = [];
result.dx_trim = dxTrim;
result.idx = [];
result.input_names = ["alpha_rad","beta_rad","r_rad_s","front_collective_rpm", ...
    "rear_collective_rpm","delta_a_rad","delta_e_rad","delta_r_rad"];
result.trim_options_out = optOut;
result.trim_error = trimError;
result.output_residual = yTrim(iy) - y0(iy);
result.state_derivative_residual = zeros(0, 1);
result.max_output_residual = max(abs(result.output_residual), [], 'omitnan');
result.max_dx_residual = 0;
result.converged = isempty(trimError) ...
    && isfinite(result.max_output_residual) ...
    && result.max_output_residual <= opts.TrimTol;

result = populateGroupedSummary(result, grouped);
end

function seed = pickManualSeed(caseDef, manualSeeds)
match = find(strcmp({manualSeeds.case_name}, caseDef.name), 1, 'first');
if isempty(match)
    error('homework7_trim_cases:MissingManualSeed', ...
        'Could not find manual seed for case %s.', caseDef.name);
end
seed = manualSeeds(match);
end

function u0 = helperSeedVector(seed)
u0 = [ ...
    deg2rad(seed.alpha_deg); ...
    deg2rad(seed.beta_deg); ...
    seed.r_rad_s; ...
    seed.front_collective_rpm; ...
    seed.rear_collective_rpm; ...
    deg2rad(seed.delta_a_deg); ...
    deg2rad(seed.delta_e_deg); ...
    deg2rad(seed.delta_r_deg)];
end

function [y0, iy, iu] = helperTrimSetup(bankDeg, allowFrontRPMTrim)
y0 = zeros(7, 1);
if abs(bankDeg) < 1e-9
    iy = [1; 3; 5];
    iu = [2; 3; 5; 6; 8];
    if ~allowFrontRPMTrim
        iu = sort([iu; 4]);
    end
else
    iy = (1:7).';
    iu = 2;
end
end

function grouped = unpackHelperInputs(uTrim, caseDef, cruiseTiltDeg)
grouped = struct();
grouped.alpha_rad = uTrim(1);
grouped.beta_rad = uTrim(2);
grouped.r_rad_s = uTrim(3);
grouped.front_collective_rpm = uTrim(4);
grouped.rear_collective_rpm = uTrim(5);
grouped.delta_a_rad = uTrim(6);
grouped.delta_e_rad = uTrim(7);
grouped.delta_r_rad = uTrim(8);
grouped.bank_deg = caseDef.bank_deg;
grouped.speed_mps = caseDef.speed_mps;
grouped.cruise_tilt_deg = cruiseTiltDeg;
end

function [x0, u0] = buildFullPlantSeed(grouped, cruiseTiltDeg)
alphaRad = grouped.alpha_rad;
betaRad = grouped.beta_rad;
phiRad = deg2rad(grouped.bank_deg);
thetaRad = alphaRad;
psiRad = 0;
V = grouped.speed_mps;

uBody = V * cos(alphaRad) * cos(betaRad);
vBody = V * sin(betaRad);
wBody = V * sin(alphaRad) * cos(betaRad);

quat = localEulerToQuat(phiRad, thetaRad, psiRad);
x0 = [[0; 0; -1000]; [0; 0; grouped.r_rad_s]; [uBody; vBody; wBody]; quat];

u0 = zeros(24, 1);
u0(13:18) = cruiseTiltDeg;
u0(19) = grouped.front_collective_rpm;
u0(20) = grouped.rear_collective_rpm;
u0(21) = 0;
u0(22) = grouped.delta_a_rad;
u0(23) = grouped.delta_e_rad;
u0(24) = grouped.delta_r_rad;
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

function result = populateGroupedSummary(result, grouped)
result.front_collective_rpm = grouped.front_collective_rpm;
result.rear_collective_rpm = grouped.rear_collective_rpm;
result.front_rpm_mean = grouped.front_collective_rpm;
result.rear_rpm_mean = grouped.rear_collective_rpm;
result.front_tilt_deg_mean = grouped.cruise_tilt_deg;
result.delta_f_deg = 0;
result.delta_a_deg = rad2deg(grouped.delta_a_rad);
result.delta_e_deg = rad2deg(grouped.delta_e_rad);
result.delta_r_deg = rad2deg(grouped.delta_r_rad);
result.vinf_trim = grouped.speed_mps;
result.alpha_trim_deg = rad2deg(grouped.alpha_rad);
result.beta_trim_deg = rad2deg(grouped.beta_rad);
result.roll_trim_deg = grouped.bank_deg;
result.pitch_trim_deg = rad2deg(grouped.alpha_rad);
result.yaw_trim_deg = 0;
result.pqr_trim = [0 0 grouped.r_rad_s];
end

function out = summarizeResult(in)
out = struct();
out.case_name = string(in.case_name);
out.speed_mps = in.speed_mps;
out.bank_deg = in.bank_deg;
out.converged = in.converged;
out.max_output_residual = in.max_output_residual;
out.max_dx_residual = in.max_dx_residual;
out.front_collective_rpm = in.front_collective_rpm;
out.rear_collective_rpm = in.rear_collective_rpm;
out.front_tilt_deg_mean = in.front_tilt_deg_mean;
out.vinf_trim = in.vinf_trim;
out.alpha_trim_deg = in.alpha_trim_deg;
out.beta_trim_deg = in.beta_trim_deg;
out.roll_trim_deg = in.roll_trim_deg;
out.pitch_trim_deg = in.pitch_trim_deg;
out.yaw_trim_deg = in.yaw_trim_deg;
out.delta_f_deg = in.delta_f_deg;
out.delta_a_deg = in.delta_a_deg;
out.delta_e_deg = in.delta_e_deg;
out.delta_r_deg = in.delta_r_deg;
out.p_trim = in.pqr_trim(1);
out.q_trim = in.pqr_trim(2);
out.r_trim = in.pqr_trim(3);
out.use_avl_aero = in.use_avl_aero;
out.allow_front_rpm_trim = in.allow_front_rpm_trim;
out.trim_error = string(in.trim_error);
end

function s = emptyResultStruct()
s = struct( ...
    'case_name', '', ...
    'speed_mps', nan, ...
    'bank_deg', nan, ...
    'use_avl_aero', false, ...
    'allow_front_rpm_trim', false, ...
    'cruise_tilt_deg', nan, ...
    'alpha_seed_deg', nan, ...
    'delta_e_seed_deg', nan, ...
    'initial_x', [], ...
    'initial_u', [], ...
    'helper_x_trim', [], ...
    'helper_u_trim', [], ...
    'x_trim', [], ...
    'u_trim', [], ...
    'y0', [], ...
    'y_trim', [], ...
    'iy', [], ...
    'iu', [], ...
    'dx0', [], ...
    'dx_trim', [], ...
    'idx', [], ...
    'input_names', strings(0, 1), ...
    'trim_options_out', [], ...
    'trim_error', '', ...
    'output_residual', [], ...
    'state_derivative_residual', [], ...
    'max_output_residual', nan, ...
    'max_dx_residual', nan, ...
    'converged', false, ...
    'front_collective_rpm', nan, ...
    'rear_collective_rpm', nan, ...
    'front_rpm_mean', nan, ...
    'rear_rpm_mean', nan, ...
    'front_tilt_deg_mean', nan, ...
    'vinf_trim', nan, ...
    'alpha_trim_deg', nan, ...
    'beta_trim_deg', nan, ...
    'roll_trim_deg', nan, ...
    'pitch_trim_deg', nan, ...
    'yaw_trim_deg', nan, ...
    'delta_f_deg', nan, ...
    'delta_a_deg', nan, ...
    'delta_e_deg', nan, ...
    'delta_r_deg', nan, ...
    'pqr_trim', [nan nan nan]);
end
