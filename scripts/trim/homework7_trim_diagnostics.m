function report = homework7_trim_diagnostics(varargin)
%HOMEWORK7_TRIM_DIAGNOSTICS Diagnose level-flight trim feasibility.
%   Uses the current analytical surface/propeller equations directly to
%   identify nearby steady-force/moment operating points before invoking
%   trim().

p = inputParser;
p.addParameter('Speeds', [55 70], @(x) isnumeric(x) && ~isempty(x));
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
g = abs(evalin('base', 'g'));

cases = repmat(struct(), numel(opts.Speeds), 1);
for k = 1:numel(opts.Speeds)
    V = opts.Speeds(k);
    seed = initialGuess(ac, V, 0, opts.CruiseTiltDeg, g);
    z0 = [seed.alpha_rad; seed.delta_e_rad; seed.front_rpm; seed.rear_rpm];
    z = fminsearch(@(zz) objective(zz, ac, V, 0, opts.CruiseTiltDeg, g), z0, ...
        optimset('Display', 'off', 'MaxFunEvals', 5000, 'MaxIter', 5000));
    [cost, detail] = objective(z, ac, V, 0, opts.CruiseTiltDeg, g);

    cases(k).speed_mps = V;
    cases(k).seed_alpha_deg = rad2deg(seed.alpha_rad);
    cases(k).seed_delta_e_deg = rad2deg(seed.delta_e_rad);
    cases(k).seed_front_rpm = seed.front_rpm;
    cases(k).seed_rear_rpm = seed.rear_rpm;
    cases(k).best_alpha_deg = rad2deg(z(1));
    cases(k).best_delta_e_deg = rad2deg(z(2));
    cases(k).best_front_rpm = z(3);
    cases(k).best_rear_rpm = z(4);
    cases(k).cost = cost;
    cases(k).Fx_N = detail.F_total(1);
    cases(k).Fy_N = detail.F_total(2);
    cases(k).Fz_N = detail.F_total(3);
    cases(k).Mx_Nm = detail.M_total(1);
    cases(k).My_Nm = detail.M_total(2);
    cases(k).Mz_Nm = detail.M_total(3);
    cases(k).lift_main_wing_N = detail.breakdown.wing(3);
    cases(k).front_prop_force_x_N = detail.breakdown.front_prop(1);
    cases(k).rear_prop_force_z_N = detail.breakdown.rear_prop(3);
end

tbl = struct2table(cases);
report = struct();
report.created_utc = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
report.cases = cases;
report.table = tbl;

if opts.WriteFiles
    writetable(tbl, fullfile(outDir, 'homework7_diagnostic_balance.csv'));
    save(fullfile(outDir, 'homework7_diagnostic_balance.mat'), 'report');
end
end

function seed = initialGuess(ac, speedMps, bankDeg, cruiseTiltDeg, g)
q = 0.5 * ac.rho * speedMps^2;
cosBank = max(cosd(bankDeg), 0.5);
CLreq = (ac.Mass * g) / (q * ac.wing.S * cosBank);
alphaRad = (CLreq - ac.wing.CL0) / ac.wing.CLa;
alphaRad = min(max(alphaRad, deg2rad(-2)), deg2rad(12));

dragCoeff = ac.wing.CD0 + ac.wing.CDa * (alphaRad - ac.wing.a0)^2;
dragN = q * ac.wing.S * dragCoeff;
frontRPM = sqrt(max(dragN, 0) / (6 * ac.prop.k_Thrust));

cmDeltaE = ac.controls.ruddervator.expected.CM_delta_e_per_rad;
if abs(cmDeltaE) > 1e-6
    deltaERad = -ac.wing.CM0 / cmDeltaE;
else
    deltaERad = deg2rad(-10);
end
deltaERad = min(max(deltaERad, deg2rad(-20)), deg2rad(5));

rearRPM = 0;
seed = struct('alpha_rad', alphaRad, 'delta_e_rad', deltaERad, ...
    'front_rpm', frontRPM, 'rear_rpm', rearRPM, 'tilt_deg', cruiseTiltDeg);
end

function [cost, detail] = objective(z, ac, speedMps, bankDeg, cruiseTiltDeg, g)
alphaRad = z(1);
deltaERad = z(2);
frontRPM = z(3);
rearRPM = z(4);

penalty = 0;
if frontRPM < 0
    penalty = penalty + 1e6 * frontRPM^2;
end
if rearRPM < 0
    penalty = penalty + 1e6 * rearRPM^2;
end
if abs(rad2deg(deltaERad)) > 30
    penalty = penalty + 1e3 * (abs(rad2deg(deltaERad)) - 30)^2;
end
if alphaRad < deg2rad(-5) || alphaRad > deg2rad(15)
    penalty = penalty + 1e3 * (abs(rad2deg(alphaRad - min(max(alphaRad, deg2rad(-5)), deg2rad(15)))))^2;
end

alphaRad = min(max(alphaRad, deg2rad(-5)), deg2rad(15));
frontRPM = max(frontRPM, 0);
rearRPM = max(rearRPM, 0);

state = struct();
state.v_body = [speedMps * cos(alphaRad); 0; speedMps * sin(alphaRad)];
state.omega = [0; 0; 0];
state.phi = deg2rad(bankDeg);
state.theta = alphaRad;
state.psi = 0;

[F_wingL, M_wingL] = evalSurface(ac.wingL, state, ac.CG, 0);
[F_wingR, M_wingR] = evalSurface(ac.wingR, state, ac.CG, 0);
[F_tailL, M_tailL] = evalSurface(ac.tailL, state, ac.CG, deltaERad);
[F_tailR, M_tailR] = evalSurface(ac.tailR, state, ac.CG, deltaERad);
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

Wb = ac.Mass * g * [-sin(state.theta); sin(state.phi) * cos(state.theta); cos(state.phi) * cos(state.theta)];
F_total = F_aero + F_front + F_rear + Wb;
M_total = M_aero + M_front + M_rear;

scaled = [F_total(1) / 1e3; F_total(3) / 1e3; M_total(2) / 1e4];
cost = scaled.' * scaled + penalty;

detail = struct();
detail.F_total = F_total;
detail.M_total = M_total;
detail.breakdown = struct();
detail.breakdown.wing = F_wingL + F_wingR;
detail.breakdown.tail = F_tailL + F_tailR;
detail.breakdown.front_prop = F_front;
detail.breakdown.rear_prop = F_rear;
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
alpha_geom = surf.i - asin(dot(v_dir, surf.n));

ctrl_tau = 0;
CM_delta = 0;
CD_delta2 = 0;
if isfield(surf, 'ctrl_tau'), ctrl_tau = surf.ctrl_tau; end
if isfield(surf, 'CM_delta'), CM_delta = surf.CM_delta; end
if isfield(surf, 'CD_delta2'), CD_delta2 = surf.CD_delta2; end

alpha_eff = alpha_geom + ctrl_tau * deltaLocal;
qS = surf.half_rho_S * V2;

CL = surf.CL0 + surf.CLa * alpha_eff;
CD = surf.CD0 + surf.CDa * (alpha_eff - surf.a0)^2 + CD_delta2 * deltaLocal^2;
CM = surf.CM0 + surf.CMa * alpha_eff + CM_delta * deltaLocal;

L = qS * CL;
D = qS * CD;
dir_D = -v_dir;
dir_L = surf.n - dot(surf.n, v_dir) * v_dir;
if norm(dir_L) > 0
    dir_L = dir_L / norm(dir_L);
else
    dir_L = surf.n;
end

F_surf = L * dir_L + D * dir_D;
m_axis = cross(surf.n, v_dir);
if norm(m_axis) > 0
    m_axis = m_axis / norm(m_axis);
end
M_surf = (qS * surf.c * CM) * m_axis;
F_cg = F_surf;
M_cg = M_surf + cross(r_arm, F_surf);
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
