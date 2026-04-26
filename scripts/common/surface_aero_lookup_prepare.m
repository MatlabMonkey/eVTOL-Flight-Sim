function [alpha_geom, alpha_eff, qS, dirL, dirD, mAxis, r_arm] = ...
    surface_aero_lookup_prepare(v_body, omega, delta_ctrl, surf, CG)
%SURFACE_AERO_LOOKUP_PREPARE Compute local flow geometry for one aero surface.

if nargin < 3 || isempty(delta_ctrl)
    delta_ctrl = 0.0;
end

alpha_geom = 0.0;
alpha_eff = 0.0;
qS = 0.0;
dirL = zeros(3, 1);
dirD = zeros(3, 1);
mAxis = zeros(3, 1);
r_arm = surf.pos - CG;

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
end
