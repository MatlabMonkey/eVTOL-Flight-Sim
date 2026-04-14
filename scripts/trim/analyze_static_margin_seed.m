repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

ac = aircraft_def();
gmag = abs(ac.g);

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

state = struct();
alphaRad = deg2rad(alphaDeg);
betaRad = 0;
state.v_body = [ ...
    speed * cos(alphaRad) * cos(betaRad); ...
    speed * sin(betaRad); ...
    speed * sin(alphaRad) * cos(betaRad)];
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

Wb = ac.Mass * gmag * [-sin(state.theta); sin(state.phi) * cos(state.theta); cos(state.phi) * cos(state.theta)];

F_total = F_wingL + F_wingR + F_tailL + F_tailR + F_frontR + F_frontL + F_rearR + F_rearL + Wb;
M_total = M_wingL + M_wingR + M_tailL + M_tailR + M_frontR + M_frontL + M_rearR + M_rearL;

fprintf('CG                : [%.6f %.6f %.6f] m\n', ac.CG(1), ac.CG(2), ac.CG(3));
fprintf('Wing ref x from CG: %.6f m\n', ac.wing.pos(1) - ac.CG(1));
fprintf('Tail ref x from CG: %.6f m\n', mean([ac.tailL.pos(1), ac.tailR.pos(1)]) - ac.CG(1));
fprintf('Wing quarter-chord x estimate: %.6f m\n', ac.wing.pos(1) + 0.25 * ac.wing.c);
fprintf('Static margin estimate       : %.6f cbar\n', (ac.CG(1) - (ac.wing.pos(1) + 0.25 * ac.wing.c)) / ac.wing.c);
fprintf('\nForce totals body [X Y Z]    : [%.6f %.6f %.6f] N\n', F_total(1), F_total(2), F_total(3));
fprintf('Moment totals body [L M N]   : [%.6f %.6f %.6f] N-m\n', M_total(1), M_total(2), M_total(3));
fprintf('\nPitch moment contributions M_y (N-m)\n');
fprintf('  wingL      %.6f\n', M_wingL(2));
fprintf('  wingR      %.6f\n', M_wingR(2));
fprintf('  tailL      %.6f\n', M_tailL(2));
fprintf('  tailR      %.6f\n', M_tailR(2));
fprintf('  frontR     %.6f\n', M_frontR(2));
fprintf('  frontL     %.6f\n', M_frontL(2));
fprintf('  rearR      %.6f\n', M_rearR(2));
fprintf('  rearL      %.6f\n', M_rearL(2));
fprintf('  total      %.6f\n', M_total(2));

for deltaAlpha = [-1 0 1]
    s2 = state;
    a2 = deg2rad(alphaDeg + deltaAlpha);
    s2.theta = a2;
    s2.v_body = [speed * cos(a2); 0; speed * sin(a2)];
    [~, MwL] = evalSurface(ac.wingL, s2, ac.CG, 0);
    [~, MwR] = evalSurface(ac.wingR, s2, ac.CG, 0);
    [~, MtL] = evalSurface(ac.tailL, s2, ac.CG, deg2rad(deltaEDeg));
    [~, MtR] = evalSurface(ac.tailR, s2, ac.CG, deg2rad(deltaEDeg));
    [~, MfR] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFR', ac.prop.hub_offset, ac.prop.Rspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
    [~, MfL] = evalFrontGroup(frontRPM * ones(3,1), cruiseTiltDeg * ones(3,1), ac.prop.posFL', ac.prop.hub_offset, ac.prop.Lspin_dir, ac.prop.k_Thrust, ac.prop.k_Torque, ac.CG);
    M2 = MwL + MwR + MtL + MtR + MfR + MfL;
    fprintf('alpha %+d deg -> total M_y %.6f N-m\n', deltaAlpha, M2(2));
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
