function report = homework7_manual_trim_search(varargin)
%HOMEWORK7_MANUAL_TRIM_SEARCH Find grouped-input trim conditions manually.
%   This is a fallback trim workflow for Homework 7 when legacy trim() on
%   the quaternion-based plant becomes numerically ill-conditioned.

p = inputParser;
p.addParameter('LevelSpeeds', [70 85], @(x) isnumeric(x) && numel(x) == 2);
p.addParameter('BankSpeed', 70, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BankDeg', 5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('CruiseTiltDeg', 90, @(x) isnumeric(x) && isscalar(x));
p.addParameter('WriteFiles', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opts = p.Results;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'homework7_trim');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
ac = evalin('base', 'aircraft');
gmag = abs(evalin('base', 'g'));

cases = [ ...
    struct('name', sprintf('level_%g', opts.LevelSpeeds(1)), 'speed_mps', opts.LevelSpeeds(1), 'bank_deg', 0), ...
    struct('name', sprintf('level_%g', opts.LevelSpeeds(2)), 'speed_mps', opts.LevelSpeeds(2), 'bank_deg', 0), ...
    struct('name', sprintf('bank_%gdeg_%g', opts.BankDeg, opts.BankSpeed), 'speed_mps', opts.BankSpeed, 'bank_deg', opts.BankDeg)];

results = repmat(emptyManualResult(), numel(cases), 1);
for k = 1:numel(cases)
    if abs(cases(k).bank_deg) < 1e-9
        results(k) = solveLevelCase(ac, gmag, cases(k), opts);
    else
        results(k) = solveBankCase(ac, gmag, cases(k), opts);
    end
end

summaryTbl = struct2table(arrayfun(@summarizeManualResult, results));
report = struct();
report.created_utc = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
report.results = results;
report.summary = summaryTbl;

if opts.WriteFiles
    save(fullfile(outDir, 'homework7_manual_trim_results.mat'), 'report', 'results', 'summaryTbl');
    writetable(summaryTbl, fullfile(outDir, 'homework7_manual_trim_summary.csv'));
end
end

function result = solveLevelCase(ac, gmag, caseDef, opts)
seed = initialLevelSeed(ac, gmag, caseDef.speed_mps, caseDef.bank_deg, opts.CruiseTiltDeg);
z0 = [seed.alpha_rad; seed.delta_e_rad; seed.front_rpm; seed.rear_rpm];
z = fminsearch(@(zz) levelObjective(zz, ac, gmag, caseDef.speed_mps, caseDef.bank_deg, opts.CruiseTiltDeg), ...
    z0, optimset('Display', 'off', 'MaxFunEvals', 10000, 'MaxIter', 10000));
[cost, detail] = levelObjective(z, ac, gmag, caseDef.speed_mps, caseDef.bank_deg, opts.CruiseTiltDeg);

result = emptyManualResult();
result.case_name = caseDef.name;
result.speed_mps = caseDef.speed_mps;
result.bank_deg = caseDef.bank_deg;
result.method = "manual_balance";
result.alpha_deg = rad2deg(detail.alpha_rad);
result.beta_deg = 0;
result.roll_deg = caseDef.bank_deg;
result.pitch_deg = rad2deg(detail.theta_rad);
result.yaw_deg = 0;
result.p_rad_s = 0;
result.q_rad_s = 0;
result.r_rad_s = 0;
result.delta_f_deg = 0;
result.delta_a_deg = 0;
result.delta_e_deg = rad2deg(detail.delta_e_rad);
result.delta_r_deg = 0;
result.front_collective_rpm = detail.front_rpm;
result.rear_collective_rpm = detail.rear_rpm;
result.front_tilt_deg = opts.CruiseTiltDeg;
result.vinf_mps = detail.Vinf;
result.cost = cost;
result.force_residual_norm = norm(detail.accel_residual);
result.moment_residual_norm = norm(detail.moment_residual);
result.force_residual_body = detail.force_residual_body.';
result.moment_residual_body = detail.moment_residual_body.';
result.notes = "Level-flight steady balance from grouped-input analytical search.";
end

function result = solveBankCase(ac, gmag, caseDef, opts)
levelSeed = initialLevelSeed(ac, gmag, caseDef.speed_mps, 0, opts.CruiseTiltDeg);
targetR = gmag * tand(caseDef.bank_deg) / caseDef.speed_mps;
z0 = [levelSeed.alpha_rad; 0; deg2rad(2); levelSeed.delta_e_rad; deg2rad(1); ...
    max(levelSeed.front_rpm, 500); max(levelSeed.rear_rpm, 0); targetR];

z = fminsearch(@(zz) bankObjective(zz, ac, gmag, caseDef.speed_mps, caseDef.bank_deg, opts.CruiseTiltDeg, targetR), ...
    z0, optimset('Display', 'off', 'MaxFunEvals', 20000, 'MaxIter', 20000));
    [cost, detail] = bankObjective(z, ac, gmag, caseDef.speed_mps, caseDef.bank_deg, opts.CruiseTiltDeg, targetR);

result = emptyManualResult();
result.case_name = caseDef.name;
result.speed_mps = caseDef.speed_mps;
result.bank_deg = caseDef.bank_deg;
result.method = "manual_balance";
result.alpha_deg = rad2deg(detail.alpha_rad);
result.beta_deg = rad2deg(detail.beta_rad);
result.roll_deg = caseDef.bank_deg;
result.pitch_deg = rad2deg(detail.theta_rad);
result.yaw_deg = 0;
result.p_rad_s = 0;
result.q_rad_s = 0;
result.r_rad_s = detail.r_rad_s;
result.delta_f_deg = 0;
result.delta_a_deg = rad2deg(detail.delta_a_rad);
result.delta_e_deg = rad2deg(detail.delta_e_rad);
result.delta_r_deg = rad2deg(detail.delta_r_rad);
result.front_collective_rpm = detail.front_rpm;
result.rear_collective_rpm = detail.rear_rpm;
result.front_tilt_deg = opts.CruiseTiltDeg;
result.vinf_mps = detail.Vinf;
result.cost = cost;
result.force_residual_norm = norm(detail.accel_residual);
result.moment_residual_norm = norm(detail.moment_residual);
result.force_residual_body = detail.force_residual_body.';
result.moment_residual_body = detail.moment_residual_body.';
result.notes = "Banked steady-turn balance using grouped controls and body-rate consistency.";
end

function seed = initialLevelSeed(ac, gmag, speedMps, bankDeg, cruiseTiltDeg)
qbar = 0.5 * ac.rho * speedMps^2;
cosBank = max(cosd(bankDeg), 0.5);
CLreq = (ac.Mass * gmag) / (qbar * ac.wing.S * cosBank);
alphaRad = (CLreq - ac.wing.CL0) / ac.wing.CLa;
alphaRad = clamp(alphaRad, deg2rad(-2), deg2rad(12));

cmDeltaE = ac.controls.ruddervator.expected.CM_delta_e_per_rad;
if abs(cmDeltaE) > 1e-6
    deltaERad = clamp(-ac.wing.CM0 / cmDeltaE, deg2rad(-20), deg2rad(5));
else
    deltaERad = deg2rad(-10);
end

dragCoeff = ac.wing.CD0 + ac.wing.CDa * (alphaRad - ac.wing.a0)^2;
dragN = qbar * ac.wing.S * dragCoeff;
frontRPM = sqrt(max(dragN, 0) / (6 * ac.prop.k_Thrust));

seed = struct();
seed.alpha_rad = alphaRad;
seed.delta_e_rad = deltaERad;
seed.front_rpm = frontRPM;
seed.rear_rpm = 0;
seed.cruise_tilt_deg = cruiseTiltDeg;
end

function [cost, detail] = levelObjective(z, ac, gmag, speedMps, bankDeg, cruiseTiltDeg)
alphaRad = clamp(z(1), deg2rad(-5), deg2rad(15));
deltaERad = clamp(z(2), deg2rad(-25), deg2rad(10));
frontRPM = max(z(3), 0);
rearRPM = max(z(4), 0);

detail = evaluatePoint(ac, gmag, speedMps, bankDeg, alphaRad, 0, 0, 0, deltaERad, 0, frontRPM, rearRPM, cruiseTiltDeg);
accelResidual = detail.force_residual_body ./ ac.Mass;
momentResidual = detail.moment_residual_body ./ max(abs(diag(ac.J)), 1);
scaled = [accelResidual(1); accelResidual(3); momentResidual(2)];
penalty = envelopePenalty(alphaRad, 0, 0, deltaERad, 0, frontRPM, rearRPM);
cost = scaled.' * scaled + penalty;
detail.accel_residual = accelResidual;
detail.moment_residual = momentResidual;
detail.delta_e_rad = deltaERad;
detail.front_rpm = frontRPM;
detail.rear_rpm = rearRPM;
end

function [cost, detail] = bankObjective(z, ac, gmag, speedMps, bankDeg, cruiseTiltDeg, targetR)
alphaRad = clamp(z(1), deg2rad(-5), deg2rad(15));
betaRad = clamp(z(2), deg2rad(-8), deg2rad(8));
deltaARad = clamp(z(3), deg2rad(-20), deg2rad(20));
deltaERad = clamp(z(4), deg2rad(-25), deg2rad(10));
deltaRRad = clamp(z(5), deg2rad(-20), deg2rad(20));
frontRPM = max(z(6), 0);
rearRPM = max(z(7), 0);
rRadS = clamp(z(8), 0, 0.25);

detail = evaluatePoint(ac, gmag, speedMps, bankDeg, alphaRad, betaRad, 0, 0, deltaERad, deltaRRad, frontRPM, rearRPM, cruiseTiltDeg, deltaARad, rRadS);
accelResidual = detail.force_residual_body ./ ac.Mass;
momentResidual = detail.moment_residual_body ./ max(abs(diag(ac.J)), 1);
scaled = [accelResidual(:); momentResidual(:)];
penalty = envelopePenalty(alphaRad, betaRad, deltaARad, deltaERad, deltaRRad, frontRPM, rearRPM);
penalty = penalty + 10 * (rRadS - targetR)^2 + 2 * betaRad^2;
cost = scaled.' * scaled + penalty;
detail.accel_residual = accelResidual;
detail.moment_residual = momentResidual;
detail.delta_a_rad = deltaARad;
detail.delta_e_rad = deltaERad;
detail.delta_r_rad = deltaRRad;
detail.front_rpm = frontRPM;
detail.rear_rpm = rearRPM;
detail.r_rad_s = rRadS;
end

function detail = evaluatePoint(ac, gmag, speedMps, bankDeg, alphaRad, betaRad, pRadS, qRadS, deltaERad, deltaRRad, frontRPM, rearRPM, cruiseTiltDeg, deltaARad, rRadS)
if nargin < 14
    deltaARad = 0;
end
if nargin < 15
    rRadS = 0;
end

phi = deg2rad(bankDeg);
theta = alphaRad;
u = speedMps * cos(alphaRad) * cos(betaRad);
v = speedMps * sin(betaRad);
w = speedMps * sin(alphaRad) * cos(betaRad);

state = struct();
state.v_body = [u; v; w];
state.omega = [pRadS; qRadS; rRadS];
state.phi = phi;
state.theta = theta;
state.psi = 0;

[F_wingL, M_wingL] = evalSurface(ac.wingL, state, ac.CG, deltaARad);
[F_wingR, M_wingR] = evalSurface(ac.wingR, state, ac.CG, -deltaARad);
[F_tailL, M_tailL] = evalSurface(ac.tailL, state, ac.CG, deltaERad - deltaRRad);
[F_tailR, M_tailR] = evalSurface(ac.tailR, state, ac.CG, deltaERad + deltaRRad);
[F_frontR, M_frontR] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFR', ac.prop.hub_offset, ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
[F_frontL, M_frontL] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFL', ac.prop.hub_offset, ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
[F_rearR, M_rearR] = evalRearGroup(rearRPM * ones(3,1), ac.prop.posRR', ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
[F_rearL, M_rearL] = evalRearGroup(rearRPM * ones(3,1), ac.prop.posRL', ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);

F_aero = F_wingL + F_wingR + F_tailL + F_tailR;
M_aero = M_wingL + M_wingR + M_tailL + M_tailR;
F_front = F_frontR + F_frontL;
M_front = M_frontR + M_frontL;
F_rear = F_rearR + F_rearL;
M_rear = M_rearR + M_rearL;

Wb = ac.Mass * gmag * [-sin(theta); sin(phi) * cos(theta); cos(phi) * cos(theta)];
F_total = F_aero + F_front + F_rear + Wb;
M_total = M_aero + M_front + M_rear;

omega = state.omega;
Vbody = state.v_body;
J = ac.J;
forceResidual = F_total - ac.Mass * cross(omega, Vbody);
momentResidual = M_total - cross(omega, J * omega);

detail = struct();
detail.Vinf = norm(Vbody);
detail.alpha_rad = atan2(Vbody(3), Vbody(1));
detail.beta_rad = asin(Vbody(2) / max(detail.Vinf, 1e-9));
detail.theta_rad = theta;
detail.force_residual_body = forceResidual;
detail.moment_residual_body = momentResidual;
detail.F_total = F_total;
detail.M_total = M_total;
detail.breakdown = struct();
detail.breakdown.wing = F_wingL + F_wingR;
detail.breakdown.tail = F_tailL + F_tailR;
detail.breakdown.front_prop = F_front;
detail.breakdown.rear_prop = F_rear;
end

function penalty = envelopePenalty(alphaRad, betaRad, deltaARad, deltaERad, deltaRRad, frontRPM, rearRPM)
penalty = 0;
penalty = penalty + softBound(rad2deg(alphaRad), -5, 15, 10);
penalty = penalty + softBound(rad2deg(betaRad), -10, 10, 10);
penalty = penalty + softBound(rad2deg(deltaARad), -20, 20, 50);
penalty = penalty + softBound(rad2deg(deltaERad), -25, 10, 50);
penalty = penalty + softBound(rad2deg(deltaRRad), -20, 20, 50);
penalty = penalty + softBound(frontRPM, 0, 6000, 1e-3);
penalty = penalty + softBound(rearRPM, 0, 6000, 1e-3);
end

function value = softBound(x, lo, hi, scale)
value = 0;
if x < lo
    value = scale * (lo - x)^2;
elseif x > hi
    value = scale * (x - hi)^2;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
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
alphaGeom = surf.i - asin(normal_proj);

ctrl_tau = 0;
CM_delta = 0;
CD_delta2 = 0;
if isfield(surf, 'ctrl_tau'), ctrl_tau = surf.ctrl_tau; end
if isfield(surf, 'CM_delta'), CM_delta = surf.CM_delta; end
if isfield(surf, 'CD_delta2'), CD_delta2 = surf.CD_delta2; end

alphaEff = alphaGeom + ctrl_tau * deltaLocal;
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

function out = summarizeManualResult(in)
out = struct();
out.case_name = string(in.case_name);
out.method = string(in.method);
out.speed_mps = in.speed_mps;
out.bank_deg = in.bank_deg;
out.roll_deg = in.roll_deg;
out.pitch_deg = in.pitch_deg;
out.alpha_deg = in.alpha_deg;
out.beta_deg = in.beta_deg;
out.r_rad_s = in.r_rad_s;
out.delta_a_deg = in.delta_a_deg;
out.delta_e_deg = in.delta_e_deg;
out.delta_r_deg = in.delta_r_deg;
out.front_collective_rpm = in.front_collective_rpm;
out.rear_collective_rpm = in.rear_collective_rpm;
out.front_tilt_deg = in.front_tilt_deg;
out.vinf_mps = in.vinf_mps;
out.cost = in.cost;
out.force_residual_norm = in.force_residual_norm;
out.moment_residual_norm = in.moment_residual_norm;
out.notes = string(in.notes);
end

function s = emptyManualResult()
s = struct( ...
    'case_name', "", ...
    'speed_mps', nan, ...
    'bank_deg', nan, ...
    'method', "", ...
    'alpha_deg', nan, ...
    'beta_deg', nan, ...
    'roll_deg', nan, ...
    'pitch_deg', nan, ...
    'yaw_deg', nan, ...
    'p_rad_s', nan, ...
    'q_rad_s', nan, ...
    'r_rad_s', nan, ...
    'delta_f_deg', nan, ...
    'delta_a_deg', nan, ...
    'delta_e_deg', nan, ...
    'delta_r_deg', nan, ...
    'front_collective_rpm', nan, ...
    'rear_collective_rpm', nan, ...
    'front_tilt_deg', nan, ...
    'vinf_mps', nan, ...
    'cost', nan, ...
    'force_residual_norm', nan, ...
    'moment_residual_norm', nan, ...
    'force_residual_body', nan(1,3), ...
    'moment_residual_body', nan(1,3), ...
    'notes', "");
end
