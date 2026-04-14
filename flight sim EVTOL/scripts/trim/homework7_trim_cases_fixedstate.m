function results = homework7_trim_cases_fixedstate(varargin)
%HOMEWORK7_TRIM_CASES_FIXEDSTATE Trim grouped inputs while holding the
%flight condition states at a balance-based seed.

p = inputParser;
p.addParameter('LevelSpeeds', [70 85], @(x) isnumeric(x) && numel(x) == 2);
p.addParameter('BankSpeed', 70, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BankDeg', 5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('CruiseTiltDeg', 90, @(x) isnumeric(x) && isscalar(x));
p.addParameter('UseAVLAero', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('AllowFrontRPMTrim', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('MaxFunctionEvals', 5000, @(x) isnumeric(x) && isscalar(x));
p.addParameter('TrimTol', 1e-3, @(x) isnumeric(x) && isscalar(x));
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
load_system('Brown_6DOF_Plant');

diagReport = homework7_trim_diagnostics('Speeds', unique([opts.LevelSpeeds(:); opts.BankSpeed]), 'WriteFiles', false);

caseDefs = [ ...
    struct('name', sprintf('level_%g', opts.LevelSpeeds(1)), 'speed_mps', opts.LevelSpeeds(1), 'bank_deg', 0), ...
    struct('name', sprintf('level_%g', opts.LevelSpeeds(2)), 'speed_mps', opts.LevelSpeeds(2), 'bank_deg', 0), ...
    struct('name', sprintf('bank_%gdeg_%g', opts.BankDeg, opts.BankSpeed), 'speed_mps', opts.BankSpeed, 'bank_deg', opts.BankDeg)];

results = repmat(emptyResultStruct(), numel(caseDefs), 1);
for k = 1:numel(caseDefs)
    results(k) = runSingle(caseDefs(k), diagReport.cases, opts);
end

summaryTbl = struct2table(arrayfun(@summarizeResult, results));
writetable(summaryTbl, fullfile(outDir, 'homework7_trim_fixedstate_summary.csv'));
save(fullfile(outDir, 'homework7_trim_fixedstate_results.mat'), 'results', 'summaryTbl');
close_system('Brown_6DOF_Plant', 0);
end

function result = runSingle(caseDef, diagCases, opts)
seed = seedFromDiag(caseDef, diagCases, opts);
assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'V_init', [caseDef.speed_mps * cos(seed.alpha_rad); 0; caseDef.speed_mps * sin(seed.alpha_rad)]);
assignin('base', 'eul_init', [deg2rad(caseDef.bank_deg); seed.theta_rad; 0]);
assignin('base', 'omega_init', seed.omega_seed);
refreshModelWorkspace('Brown_6DOF_Plant');

inputMeta = getPlantInputMeta('Brown_6DOF_Plant');
u0 = zeros(max([inputMeta.range]), 1);
motorRange = findInputRange(inputMeta, 'Motor_RPM_cmd');
tiltRange = findInputRange(inputMeta, 'Tilt_angles_cmd');
deltaFRange = findInputRange(inputMeta, 'delta_f');
deltaARange = findInputRange(inputMeta, 'delta_a');
deltaERange = findInputRange(inputMeta, 'delta_e');
deltaRRange = findInputRange(inputMeta, 'delta_r');
frontCollectiveRange = findInputRangeStem(inputMeta, 'Front_RPM_collective');
rearCollectiveRange = findInputRangeStem(inputMeta, 'Rear_RPM_collective');

u0(motorRange) = 0;
u0(tiltRange) = opts.CruiseTiltDeg;
u0(deltaFRange) = 0;
u0(deltaARange) = seed.delta_a_rad;
u0(deltaERange) = seed.delta_e_rad;
u0(deltaRRange) = seed.delta_r_rad;
u0(frontCollectiveRange) = seed.front_rpm;
u0(rearCollectiveRange) = seed.rear_rpm;

ix = 1:13;
iu = [motorRange, tiltRange, deltaFRange];
if abs(caseDef.bank_deg) < 1e-9
    iu = [iu, deltaARange, deltaRRange];
end
if ~opts.AllowFrontRPMTrim
    iu = [iu, frontCollectiveRange];
end
dx0 = zeros(13,1);
idx = 4:9;
trimOptions = zeros(1,18);
trimOptions(14) = opts.MaxFunctionEvals;

trimError = '';
try
    [xTrim, uTrim, yTrim, dxTrim, optOut] = trim('Brown_6DOF_Plant', x0FromModel(), u0, [], ix, iu, [], dx0, idx, trimOptions);
catch ME
    xTrim = x0FromModel();
    uTrim = u0;
    yTrim = nan(33,1);
    dxTrim = nan(13,1);
    optOut = trimOptions;
    trimError = ME.message;
end

result = emptyResultStruct();
result.case_name = caseDef.name;
result.speed_mps = caseDef.speed_mps;
result.bank_deg = caseDef.bank_deg;
result.alpha_seed_deg = rad2deg(seed.alpha_rad);
result.theta_seed_deg = rad2deg(seed.theta_rad);
result.front_collective_rpm = uTrim(frontCollectiveRange(1));
result.rear_collective_rpm = uTrim(rearCollectiveRange(1));
result.delta_a_deg = rad2deg(uTrim(deltaARange(1)));
result.delta_e_deg = rad2deg(uTrim(deltaERange(1)));
result.delta_r_deg = rad2deg(uTrim(deltaRRange(1)));
result.vinf_trim = yTrim(28);
result.alpha_trim_deg = rad2deg(yTrim(29));
result.beta_trim_deg = rad2deg(yTrim(30));
result.roll_trim_deg = rad2deg(yTrim(10));
result.pitch_trim_deg = rad2deg(yTrim(11));
result.yaw_trim_deg = rad2deg(yTrim(12));
result.dx_trim = dxTrim;
result.max_dx_residual = max(abs(dxTrim(idx) - dx0(idx)), [], 'omitnan');
result.trim_error = trimError;
result.converged = isempty(trimError) && isfinite(result.max_dx_residual) && result.max_dx_residual <= opts.TrimTol;
result.optOut = optOut;
end

function seed = seedFromDiag(caseDef, diagCases, opts)
speeds = [diagCases.speed_mps];
[~, idx] = min(abs(speeds - caseDef.speed_mps));
row = diagCases(idx);

seed = struct();
seed.alpha_rad = deg2rad(row.best_alpha_deg);
seed.theta_rad = seed.alpha_rad;
seed.front_rpm = row.best_front_rpm;
seed.rear_rpm = row.best_rear_rpm;
seed.delta_e_rad = deg2rad(row.best_delta_e_deg);
seed.delta_a_rad = 0;
seed.delta_r_rad = 0;
seed.omega_seed = [0; 0; 0];

if abs(caseDef.bank_deg) > 1e-9
    gmag = abs(evalin('base', 'g'));
    seed.omega_seed = [0; 0; gmag * tand(caseDef.bank_deg) / caseDef.speed_mps];
end

if logical(opts.UseAVLAero)
    trimTbl = readtable(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'docs', 'avl_homework', 'tables', 'trim_cases.csv'));
    if abs(caseDef.bank_deg) > 1e-9
        bankRows = trimTbl(trimTbl.bank_deg > 0, :);
        [~, ii] = min(abs(bankRows.bank_deg - caseDef.bank_deg) + abs(bankRows.speed_mps - caseDef.speed_mps));
        rowT = bankRows(ii,:);
        seed.delta_a_rad = deg2rad(rowT.aileron(1));
        seed.delta_e_rad = deg2rad(rowT.elevator(1));
        seed.delta_r_rad = deg2rad(rowT.rudder(1));
    end
end
end

function x0 = x0FromModel()
pos = evalin('base', 'pos_init(:)');
omega = evalin('base', 'omega_init(:)');
vBody = evalin('base', 'V_init(:)');
eul = evalin('base', 'eul_init(:)');
quat = localEulerToQuat(eul(1), eul(2), eul(3));

x0 = [pos; omega; vBody; quat];
end

function refreshModelWorkspace(modelName)
% Make sure state ICs backed by base-workspace variables are recompiled
% before trim() reads them.
set_param(modelName, 'SimulationCommand', 'update');
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

function inputMeta = getPlantInputMeta(modelName)
inports = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Inport');
entries = struct('name', {}, 'port', {}, 'width', {}, 'range', {});
for i = 1:numel(inports)
    blk = inports{i};
    e.name = get_param(blk, 'Name');
    e.port = str2double(get_param(blk, 'Port'));
    e.width = parsePortWidth(get_param(blk, 'PortDimensions'));
    e.range = [];
    entries(end+1) = e; %#ok<AGROW>
end
[~, order] = sort([entries.port]);
entries = entries(order);
cursor = 1;
for i = 1:numel(entries)
    entries(i).range = cursor:(cursor + entries(i).width - 1);
    cursor = cursor + entries(i).width;
end
inputMeta = entries;
end

function width = parsePortWidth(rawValue)
if isnumeric(rawValue)
    width = prod(rawValue(rawValue > 0));
    if isempty(width) || width == 0
        width = 1;
    end
    return;
end
rawText = strtrim(char(rawValue));
tokens = regexp(rawText, '[-+]?\d+', 'match');
nums = str2double(tokens);
nums = nums(nums > 0);
if isempty(nums)
    width = 1;
else
    width = prod(nums);
end
end

function idx = findInputRange(inputMeta, name)
idx = [];
for i = 1:numel(inputMeta)
    if strcmp(inputMeta(i).name, name)
        idx = inputMeta(i).range;
        return;
    end
end
end

function idx = findInputRangeStem(inputMeta, stem)
idx = [];
for i = 1:numel(inputMeta)
    if strcmp(inputMeta(i).name, stem) || startsWith(inputMeta(i).name, stem)
        idx = inputMeta(i).range;
        return;
    end
end
end

function out = summarizeResult(in)
out = struct();
out.case_name = string(in.case_name);
out.speed_mps = in.speed_mps;
out.bank_deg = in.bank_deg;
out.converged = in.converged;
out.alpha_seed_deg = in.alpha_seed_deg;
out.theta_seed_deg = in.theta_seed_deg;
out.front_collective_rpm = in.front_collective_rpm;
out.rear_collective_rpm = in.rear_collective_rpm;
out.delta_a_deg = in.delta_a_deg;
out.delta_e_deg = in.delta_e_deg;
out.delta_r_deg = in.delta_r_deg;
out.vinf_trim = in.vinf_trim;
out.alpha_trim_deg = in.alpha_trim_deg;
out.beta_trim_deg = in.beta_trim_deg;
out.roll_trim_deg = in.roll_trim_deg;
out.pitch_trim_deg = in.pitch_trim_deg;
out.yaw_trim_deg = in.yaw_trim_deg;
out.max_dx_residual = in.max_dx_residual;
out.trim_error = string(in.trim_error);
end

function s = emptyResultStruct()
s = struct('case_name','', 'speed_mps',nan, 'bank_deg',nan, 'converged',false, ...
    'alpha_seed_deg',nan, 'theta_seed_deg',nan, 'front_collective_rpm',nan, 'rear_collective_rpm',nan, ...
    'delta_a_deg',nan, 'delta_e_deg',nan, 'delta_r_deg',nan, ...
    'vinf_trim',nan, 'alpha_trim_deg',nan, 'beta_trim_deg',nan, ...
    'roll_trim_deg',nan, 'pitch_trim_deg',nan, 'yaw_trim_deg',nan, ...
    'dx_trim',[], 'max_dx_residual',nan, 'trim_error','', 'optOut',[]);
end
