function [seeds, details] = make_transition_force_balance_seeds(initData, trimCase, options)
%MAKE_TRANSITION_FORCE_BALANCE_SEEDS Build static force/moment trim seeds.
%
% This helper estimates front/rear collective, flap, elevator, and theta
% guesses near a requested (Vinf, alpha, tilt). It does not replace FindOp;
% it only gives FindOp a physically meaningful starting point.

if nargin < 2 || ~isstruct(initData) || ~isstruct(trimCase)
    error('make_transition_force_balance_seeds requires initData and trimCase structs.');
end
if nargin < 3 || ~isstruct(options)
    options = struct();
end

aircraft = initData.aircraft;
vinf_mps = localGetFiniteField(trimCase, 'Vinf_mps', 0.0);
alpha_deg = localResolveAlphaDeg(trimCase);
tilt_deg = localGetFiniteField(trimCase, 'front_tilt_deg', 0.0);

seed_count = localGetFiniteField(options, 'seed_count', 3);
gamma_grid_deg = localGetField(options, 'gamma_grid_deg', [-5 0 5 10 15]);
gamma_bounds_deg = localGetField(options, 'gamma_bounds_deg', [-10 25]);
if numel(gamma_bounds_deg) ~= 2
    gamma_bounds_deg = [-10 25];
end
gamma_bounds_deg = sort(gamma_bounds_deg(:)).';

front_min_rpm = localGetFiniteField(trimCase, 'front_collective_min_rpm', 600.0);
rear_min_rpm = localGetFiniteField(trimCase, 'rear_collective_min_rpm', 100.0);
front_max_rpm = localGetFiniteField(trimCase, 'front_collective_max_rpm', 2400.0);
rear_max_rpm = localGetFiniteField(trimCase, 'rear_collective_max_rpm', 2400.0);
surface_limit_deg = localSurfaceLimitDeg(trimCase, 25.0);

base_front_rpm = localGetFiniteField(trimCase, 'front_collective_guess_rpm', 1200.0);
base_rear_rpm = localGetFiniteField(trimCase, 'rear_collective_guess_rpm', 800.0);
[support_front_rpm, support_rear_rpm] = localWeightSplitRpm(aircraft, vinf_mps, alpha_deg, tilt_deg, ...
    front_min_rpm, front_max_rpm, rear_min_rpm, rear_max_rpm);

lb = [front_min_rpm, rear_min_rpm, -surface_limit_deg, -surface_limit_deg, gamma_bounds_deg(1)];
ub = [front_max_rpm, rear_max_rpm, surface_limit_deg, surface_limit_deg, gamma_bounds_deg(2)];

initial_rows = [];
for iGamma = 1:numel(gamma_grid_deg)
    gamma_deg = min(max(gamma_grid_deg(iGamma), lb(5)), ub(5));
    initial_rows(end + 1, :) = [base_front_rpm, base_rear_rpm, 0.0, 0.0, gamma_deg]; %#ok<AGROW>
    initial_rows(end + 1, :) = [support_front_rpm, support_rear_rpm, 0.0, 0.0, gamma_deg]; %#ok<AGROW>
end
initial_rows = localUniqueRows(localClampRows(initial_rows, lb, ub));

candidates = repmat(localCandidateTemplate(), 0, 1);
use_fmincon = exist('fmincon', 'file') == 2;
if use_fmincon
    opt = optimoptions('fmincon', ...
        'Display', 'off', ...
        'Algorithm', 'sqp', ...
        'MaxIterations', localGetFiniteField(options, 'max_iterations', 60), ...
        'MaxFunctionEvaluations', localGetFiniteField(options, 'max_function_evaluations', 700), ...
        'OptimalityTolerance', 1e-6, ...
        'StepTolerance', 1e-6);
end

for i = 1:size(initial_rows, 1)
    z0 = initial_rows(i, :);
    if use_fmincon
        [z, cost, exitflag] = fmincon(@(z) localObjective(z, aircraft, vinf_mps, alpha_deg, tilt_deg, surface_limit_deg), ...
            z0, [], [], [], [], lb, ub, [], opt);
    else
        [z, cost, exitflag] = fminsearch(@(z) localBoundedObjective(z, lb, ub, aircraft, vinf_mps, alpha_deg, tilt_deg, surface_limit_deg), ...
            z0, optimset('Display', 'off', 'MaxIter', 250, 'MaxFunEvals', 800));
        z = min(max(z, lb), ub);
    end

    candidate = localCandidateTemplate();
    candidate.z = z(:).';
    candidate.cost = cost;
    candidate.exitflag = exitflag;
    candidate.residual = localResidual(z, aircraft, vinf_mps, alpha_deg, tilt_deg);
    candidates(end + 1, 1) = candidate; %#ok<AGROW>
end

candidates = localUniqueCandidates(candidates);
if isempty(candidates)
    seeds = repmat(localSeedTemplate(), 0, 1);
    details = struct('candidates', candidates, 'vinf_mps', vinf_mps, ...
        'alpha_deg', alpha_deg, 'tilt_deg', tilt_deg);
    return;
end

[~, order] = sort([candidates.cost], 'ascend');
order = order(1:min(numel(order), seed_count));
seeds = repmat(localSeedTemplate(), numel(order), 1);
for i = 1:numel(order)
    candidate = candidates(order(i));
    z = candidate.z;
    seed = localSeedTemplate();
    seed.name = sprintf('force_balance_V%s_T%s_A%s_G%s', ...
        localValueLabel(vinf_mps), localValueLabel(tilt_deg), ...
        localValueLabel(alpha_deg), localValueLabel(z(5)));
    seed.source = 'transition_force_balance';
    seed.tilt_deg = tilt_deg;
    seed.vinf_mps = vinf_mps;
    seed.alpha_guess_deg = alpha_deg;
    seed.theta_guess_deg = alpha_deg + z(5);
    seed.front_collective_guess_rpm = z(1);
    seed.rear_collective_guess_rpm = z(2);
    seed.delta_f_guess_deg = z(3);
    seed.delta_a_guess_deg = 0.0;
    seed.delta_e_guess_deg = z(4);
    seed.delta_r_guess_deg = 0.0;
    seeds(i) = seed;
end

details = struct();
details.candidates = candidates(order);
details.vinf_mps = vinf_mps;
details.alpha_deg = alpha_deg;
details.tilt_deg = tilt_deg;
details.front_support_seed_rpm = support_front_rpm;
details.rear_support_seed_rpm = support_rear_rpm;
end

function cost = localBoundedObjective(z, lb, ub, aircraft, vinf_mps, alpha_deg, tilt_deg, surface_limit_deg)
z_clamped = min(max(z, lb), ub);
span = max(ub - lb, 1.0);
violation = (z - z_clamped) ./ span;
cost = localObjective(z_clamped, aircraft, vinf_mps, alpha_deg, tilt_deg, surface_limit_deg) + ...
    100.0 * sum(violation.^2);
end

function cost = localObjective(z, aircraft, vinf_mps, alpha_deg, tilt_deg, surface_limit_deg)
residual = localResidual(z, aircraft, vinf_mps, alpha_deg, tilt_deg);
surface_penalty = 0.03 * hypot(z(3), z(4)) / max(surface_limit_deg, 1e-6);
gamma_penalty = 0.01 * abs(z(5)) / 20.0;
cost = sum(residual.^2) + surface_penalty^2 + gamma_penalty^2;
end

function residual = localResidual(z, aircraft, vinf_mps, alpha_deg, tilt_deg)
front_rpm = z(1);
rear_rpm = z(2);
delta_f_rad = deg2rad(z(3));
delta_e_rad = deg2rad(z(4));
gamma_deg = z(5);

state = struct();
state.v_body = [vinf_mps * cosd(alpha_deg); 0.0; vinf_mps * sind(alpha_deg)];
state.omega = [0.0; 0.0; 0.0];
state.phi = 0.0;
state.theta = deg2rad(alpha_deg + gamma_deg);
state.psi = 0.0;

[F_wingL, M_wingL] = localEvalSurface(aircraft.wingL, state, aircraft.CG, delta_f_rad);
[F_wingR, M_wingR] = localEvalSurface(aircraft.wingR, state, aircraft.CG, delta_f_rad);
[F_tailL, M_tailL] = localEvalSurface(aircraft.tailL, state, aircraft.CG, delta_e_rad);
[F_tailR, M_tailR] = localEvalSurface(aircraft.tailR, state, aircraft.CG, delta_e_rad);

[F_frontR, M_frontR] = localEvalFrontGroup(front_rpm * ones(3, 1), ...
    tilt_deg * ones(3, 1), localPropPos(aircraft.prop, 'posFR'), ...
    aircraft.prop.hub_offset, aircraft.prop.Rspin_dir, ...
    aircraft.prop.k_Thrust, aircraft.prop.k_Torque, aircraft.CG);
[F_frontL, M_frontL] = localEvalFrontGroup(front_rpm * ones(3, 1), ...
    tilt_deg * ones(3, 1), localPropPos(aircraft.prop, 'posFL'), ...
    aircraft.prop.hub_offset, aircraft.prop.Lspin_dir, ...
    aircraft.prop.k_Thrust, aircraft.prop.k_Torque, aircraft.CG);
[F_rearR, M_rearR] = localEvalRearGroup(rear_rpm * ones(3, 1), ...
    localPropPos(aircraft.prop, 'posRR'), aircraft.prop.Rspin_dir, ...
    aircraft.prop.k_Thrust, aircraft.prop.k_Torque, aircraft.CG);
[F_rearL, M_rearL] = localEvalRearGroup(rear_rpm * ones(3, 1), ...
    localPropPos(aircraft.prop, 'posRL'), aircraft.prop.Lspin_dir, ...
    aircraft.prop.k_Thrust, aircraft.prop.k_Torque, aircraft.CG);

gravity_force_B = aircraft.Mass * aircraft.g * ...
    [-sin(state.theta); sin(state.phi) * cos(state.theta); cos(state.phi) * cos(state.theta)];
force_total = F_wingL + F_wingR + F_tailL + F_tailR + F_frontR + F_frontL + F_rearR + F_rearL + gravity_force_B;
moment_total = M_wingL + M_wingR + M_tailL + M_tailR + M_frontR + M_frontL + M_rearR + M_rearL;

force_scale = max(aircraft.Mass * aircraft.g, 1.0);
moment_scale = max(aircraft.Mass * aircraft.g * aircraft.wing.c, 1.0);
residual = [ ...
    force_total(1) / force_scale; ...
    force_total(3) / force_scale; ...
    moment_total(2) / moment_scale];
end

function [front_rpm, rear_rpm] = localWeightSplitRpm(aircraft, vinf_mps, alpha_deg, tilt_deg, front_min, front_max, rear_min, rear_max)
qbar = 0.5 * aircraft.rho * vinf_mps^2;
front_pos = localMeanRotorPos(aircraft.compData, 'F-Rotor ') - aircraft.CG(:);
rear_pos = localMeanRotorPos(aircraft.compData, 'R-Rotor ') - aircraft.CG(:);
front_arm_x = max(front_pos(1), 1e-6);
rear_arm_x = max(abs(rear_pos(1)), 1e-6);
front_fraction = rear_arm_x / (front_arm_x + rear_arm_x);
rear_fraction = front_arm_x / (front_arm_x + rear_arm_x);

wing_lift = qbar * aircraft.wing.S * localLinearLift(aircraft.wingL, alpha_deg);
tail_lift = qbar * (aircraft.tailL.S + aircraft.tailR.S) * localLinearLift(aircraft.tailL, alpha_deg);
prop_vertical = max(aircraft.Mass * aircraft.g - wing_lift - tail_lift, 0.0);
cos_tilt = max(abs(cosd(tilt_deg)), 1e-3);
front_thrust = front_fraction * prop_vertical / cos_tilt;
rear_thrust = rear_fraction * prop_vertical;

front_rpm = sqrt(max(front_thrust / max(6.0 * aircraft.prop.k_Thrust, eps), 0.0));
rear_rpm = sqrt(max(rear_thrust / max(6.0 * aircraft.prop.k_Thrust, eps), 0.0));
front_rpm = min(max(front_rpm, front_min), front_max);
rear_rpm = min(max(rear_rpm, rear_min), rear_max);
end

function cl = localLinearLift(surface, alpha_deg)
alpha_rad = deg2rad(alpha_deg);
cl = surface.CL0 + surface.CLa * (alpha_rad + surface.i);
end

function [F_cg, M_cg] = localEvalSurface(surf, state, CG, deltaLocal)
v_body = state.v_body;
omega = state.omega;
if norm(v_body) < 0.1
    F_cg = zeros(3, 1);
    M_cg = zeros(3, 1);
    return;
end

r_arm = surf.pos(:) - CG(:);
v_local = v_body + cross(omega, r_arm);
v_mag = norm(v_local);
v_dir = v_local / v_mag;
normal_proj = min(max(dot(v_dir, surf.n(:)), -1), 1);
alpha_geom = surf.i - asin(normal_proj);

ctrl_tau = localGetField(surf, 'ctrl_tau', 0.0);
CM_delta = localGetField(surf, 'CM_delta', 0.0);
CD_delta2 = localGetField(surf, 'CD_delta2', 0.0);

alpha_eff = alpha_geom + ctrl_tau * deltaLocal;
qS = surf.half_rho_S * v_mag^2;
CL = surf.CL0 + surf.CLa * alpha_eff;
CD = surf.CD0 + surf.CDa * (alpha_eff - surf.a0)^2 + CD_delta2 * deltaLocal^2;
CM = surf.CM0 + surf.CMa * alpha_eff + CM_delta * deltaLocal;

dirD = -v_dir;
dirL = surf.n(:) - dot(surf.n(:), v_dir) * v_dir;
if norm(dirL) > 0
    dirL = dirL / norm(dirL);
else
    dirL = surf.n(:);
end

force_surface = qS * CL * dirL + qS * CD * dirD;
moment_axis = cross(surf.n(:), v_dir);
if norm(moment_axis) > 0
    moment_axis = moment_axis / norm(moment_axis);
end
moment_surface = qS * surf.c * CM * moment_axis;
F_cg = force_surface;
M_cg = moment_surface + cross(r_arm, force_surface);
end

function [F_cg, M_cg] = localEvalFrontGroup(rpms, tilt_deg, pivot_pos, hub_offset, spin_dir, kT, kQ, CG)
F_cg = zeros(3, 1);
M_cg = zeros(3, 1);
for i = 1:numel(rpms)
    n = [sind(tilt_deg(i)); 0.0; -cosd(tilt_deg(i))];
    r_hub = pivot_pos(:, i) + hub_offset * n;
    r_arm = r_hub - CG(:);
    thrust = kT * rpms(i)^2;
    torque = kQ * rpms(i)^2 * spin_dir(i);
    force_motor = thrust * n;
    moment_motor = -torque * n;
    F_cg = F_cg + force_motor;
    M_cg = M_cg + cross(r_arm, force_motor) + moment_motor;
end
end

function [F_cg, M_cg] = localEvalRearGroup(rpms, prop_pos, spin_dir, kT, kQ, CG)
F_cg = zeros(3, 1);
M_cg = zeros(3, 1);
n = [0.0; 0.0; -1.0];
for i = 1:numel(rpms)
    r_arm = prop_pos(:, i) - CG(:);
    thrust = kT * rpms(i)^2;
    torque = kQ * rpms(i)^2 * spin_dir(i);
    force_motor = thrust * n;
    moment_motor = -torque * n;
    F_cg = F_cg + force_motor;
    M_cg = M_cg + cross(r_arm, force_motor) + moment_motor;
end
end

function pos = localPropPos(prop, fieldName)
pos = prop.(fieldName);
if size(pos, 1) ~= 3 && size(pos, 2) == 3
    pos = pos.';
end
end

function pos = localMeanRotorPos(compData, fragment)
mask = contains(compData(:, 1), fragment);
rows = compData(mask, 5);
xyz = zeros(numel(rows), 3);
for i = 1:numel(rows)
    xyz(i, :) = rows{i};
end
pos = mean(xyz, 1).';
end

function rows = localClampRows(rows, lb, ub)
for i = 1:size(rows, 1)
    rows(i, :) = min(max(rows(i, :), lb), ub);
end
end

function rows = localUniqueRows(rows)
if isempty(rows)
    return;
end
key = round(rows ./ [25 25 1 1 1]);
[~, idx] = unique(key, 'rows', 'stable');
rows = rows(idx, :);
end

function candidates = localUniqueCandidates(candidates)
if isempty(candidates)
    return;
end
[~, order] = sort([candidates.cost], 'ascend');
candidates = candidates(order);
key = zeros(numel(candidates), 5);
for i = 1:numel(candidates)
    key(i, :) = round(candidates(i).z ./ [25 25 1 1 1]);
end
[~, idx] = unique(key, 'rows', 'stable');
candidates = candidates(idx);
end

function alpha_deg = localResolveAlphaDeg(trimCase)
alpha_deg = localGetFiniteField(trimCase, 'alpha_target_deg', NaN);
if isfinite(alpha_deg)
    return;
end
u_body = localGetFiniteField(trimCase, 'u_body_mps', NaN);
w_body = localGetFiniteField(trimCase, 'w_body_mps', NaN);
if isfinite(u_body) && isfinite(w_body)
    alpha_deg = atan2d(w_body, u_body);
else
    alpha_deg = 0.0;
end
end

function limit_deg = localSurfaceLimitDeg(trimCase, defaultValue)
surface_limit_deg = localGetField(trimCase, 'surface_limit_deg', defaultValue);
if isscalar(surface_limit_deg)
    limit_deg = surface_limit_deg;
else
    surface_limit_deg = surface_limit_deg(:);
    limit_deg = surface_limit_deg(1);
end
end

function candidate = localCandidateTemplate()
candidate = struct('z', nan(1, 5), 'cost', inf, 'exitflag', NaN, 'residual', nan(3, 1));
end

function seed = localSeedTemplate()
seed = struct( ...
    'name', '', ...
    'source', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'alpha_guess_deg', NaN, ...
    'theta_guess_deg', NaN, ...
    'front_collective_guess_rpm', NaN, ...
    'rear_collective_guess_rpm', NaN, ...
    'delta_f_guess_deg', NaN, ...
    'delta_a_guess_deg', NaN, ...
    'delta_e_guess_deg', NaN, ...
    'delta_r_guess_deg', NaN);
end

function value = localGetFiniteField(s, fieldName, defaultValue)
value = localGetField(s, fieldName, defaultValue);
if ~(isnumeric(value) && isscalar(value) && isfinite(value))
    value = defaultValue;
end
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end
