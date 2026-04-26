function [F_cg, M_cg] = surface_aero_lookup_finish( ...
    CL, CD, Cm, qS, dirL, dirD, mAxis, r_arm, delta_ctrl, surf)
%SURFACE_AERO_LOOKUP_FINISH Convert looked-up coefficients into force/moment.

if nargin < 9 || isempty(delta_ctrl)
    delta_ctrl = 0.0;
end
F_cg = zeros(3, 1);
M_cg = zeros(3, 1);

if qS <= 0.0
    return;
end

CD_delta2 = 0.0;
CM_delta = 0.0;
if isfield(surf, 'CD_delta2') && ~isempty(surf.CD_delta2)
    CD_delta2 = surf.CD_delta2;
end
if isfield(surf, 'CM_delta') && ~isempty(surf.CM_delta)
    CM_delta = surf.CM_delta;
end

CD = CD + CD_delta2 * delta_ctrl^2;
Cm = Cm + CM_delta * delta_ctrl;

L = qS * CL;
D = qS * CD;

Fsurf = L * dirL + D * dirD;
Msurf = (qS * surf.c * Cm) * mAxis;

F_cg = Fsurf;
M_cg = Msurf + cross(r_arm, Fsurf);
end
