function [F_cg, M_cg] = fcn(CL, CD, Cm, qS, dirL, dirD, mAxis, r_arm, delta_ctrl, valid_flow, surf)

F_cg = zeros(3, 1);
M_cg = zeros(3, 1);

if ~valid_flow || qS <= 0.0
    return;
end

dirL_vec = reshape(dirL, 3, 1);
dirD_vec = reshape(dirD, 3, 1);
mAxis_vec = reshape(mAxis, 3, 1);
r_arm_vec = reshape(r_arm, 3, 1);

CD_delta2 = 0.0;
CM_delta = 0.0;
if isfield(surf, 'CD_delta2') && ~isempty(surf.CD_delta2)
    CD_delta2 = surf.CD_delta2;
end
if isfield(surf, 'CM_delta') && ~isempty(surf.CM_delta)
    CM_delta = surf.CM_delta;
end

CD_total = CD + CD_delta2 * delta_ctrl^2;
Cm_total = Cm + CM_delta * delta_ctrl;

L = qS * CL;
D = qS * CD_total;

Fsurf = L * dirL_vec + D * dirD_vec;
Msurf = (qS * surf.c * Cm_total) * mAxis_vec;

F_cg = Fsurf;
M_cg = Msurf + cross(r_arm_vec, Fsurf);
end
