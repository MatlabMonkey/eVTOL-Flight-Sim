function residual = trim_residual_eval(u)
%TRIM_RESIDUAL_EVAL Grouped-input residual model for Homework 7 trim().
%   u = [alpha_rad; beta_rad; r_rad_s; front_rpm; rear_rpm; ...
%        delta_a_rad; delta_e_rad; delta_r_rad; ...
%        speed_mps; bank_deg; cruise_tilt_deg]

persistent ac gmag
if isempty(ac)
    ac = evalin('base', 'aircraft');
    gmag = abs(evalin('base', 'g'));
end

alphaRad = u(1);
betaRad = u(2);
rRadS = u(3);
frontRPM = u(4);
rearRPM = u(5);
deltaARad = u(6);
deltaERad = u(7);
deltaRRad = u(8);
speedMps = u(9);
bankDeg = u(10);
cruiseTiltDeg = u(11);

alphaRad = clamp(alphaRad, deg2rad(-5), deg2rad(15));
betaRad = clamp(betaRad, deg2rad(-10), deg2rad(10));
deltaARad = clamp(deltaARad, deg2rad(-20), deg2rad(20));
deltaERad = clamp(deltaERad, deg2rad(-25), deg2rad(10));
deltaRRad = clamp(deltaRRad, deg2rad(-20), deg2rad(20));
frontRPM = max(frontRPM, 0);
rearRPM = max(rearRPM, 0);

phi = deg2rad(bankDeg);
theta = alphaRad;
uBody = speedMps * cos(alphaRad) * cos(betaRad);
vBody = speedMps * sin(betaRad);
wBody = speedMps * sin(alphaRad) * cos(betaRad);

state = struct();
state.v_body = [uBody; vBody; wBody];
state.omega = [0; 0; rRadS];
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

F_total = F_wingL + F_wingR + F_tailL + F_tailR + F_frontR + F_frontL + F_rearR + F_rearL ...
    + ac.Mass * gmag * [-sin(theta); sin(phi) * cos(theta); cos(phi) * cos(theta)];
M_total = M_wingL + M_wingR + M_tailL + M_tailR + M_frontR + M_frontL + M_rearR + M_rearL;

omega = state.omega;
J = ac.J;
forceResidual = F_total - ac.Mass * cross(omega, state.v_body);
momentResidual = M_total - cross(omega, J * omega);
targetR = gmag * tand(bankDeg) / max(speedMps, 1e-6);

residual = [ ...
    forceResidual(1) / ac.Mass; ...
    forceResidual(2) / ac.Mass; ...
    forceResidual(3) / ac.Mass; ...
    momentResidual(1) / max(abs(J(1,1)), 1); ...
    momentResidual(2) / max(abs(J(2,2)), 1); ...
    momentResidual(3) / max(abs(J(3,3)), 1); ...
    rRadS - targetR];
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
