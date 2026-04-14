function [F_aero_B, M_aero_B, coeffs, state] = avl_aircraft_aero_eval( ...
    V_BA, deltaLW_deg, deltaRW_deg, deltaLT_deg, deltaRT_deg, rho, avlAero)
%AVL_AIRCRAFT_AERO_EVAL Evaluate compact AVL-based aircraft aero totals.
%   Inputs are body-axis air-relative velocity and the four local control
%   surface deflections already used by the analytical branch. Control
%   deflections and fitted sweep data are in degrees.

if nargin < 7
    error('avl_aircraft_aero_eval:NotEnoughInputs', ...
        'Expected V_BA, local deflections, rho, and avlAero.');
end

V_BA = V_BA(:);
if numel(V_BA) ~= 3
    error('avl_aircraft_aero_eval:BadVelocitySize', ...
        'V_BA must be a 3x1 vector.');
end

Vinf = norm(V_BA);
coeffs = zeros(6, 1);
state = zeros(7, 1);

if Vinf < avlAero.min_airspeed_mps
    F_aero_B = zeros(3, 1);
    M_aero_B = zeros(3, 1);
    return;
end

u = V_BA(1);
v = V_BA(2);
w = V_BA(3);

alpha_deg = atan2d(w, u);
beta_deg = asind(max(-1.0, min(1.0, v / Vinf)));

delta_f_deg = 0.5 * (deltaLW_deg + deltaRW_deg);
delta_a_deg = 0.5 * (deltaLW_deg - deltaRW_deg);
delta_e_deg = 0.5 * (deltaLT_deg + deltaRT_deg);
delta_r_deg = 0.5 * (deltaRT_deg - deltaLT_deg);

alpha_deg = clamp(alpha_deg, avlAero.alpha_limits_deg(1), avlAero.alpha_limits_deg(2));
beta_deg = clamp(beta_deg, avlAero.beta_limits_deg(1), avlAero.beta_limits_deg(2));
delta_f_deg = clamp(delta_f_deg, avlAero.control_limits_deg(1), avlAero.control_limits_deg(2));
delta_a_deg = clamp(delta_a_deg, avlAero.control_limits_deg(1), avlAero.control_limits_deg(2));
delta_e_deg = clamp(delta_e_deg, avlAero.control_limits_deg(1), avlAero.control_limits_deg(2));
delta_r_deg = clamp(delta_r_deg, avlAero.control_limits_deg(1), avlAero.control_limits_deg(2));

CL = polyval(avlAero.fit.alpha.CL, alpha_deg) + ...
    avlAero.fit.flap.dCL * delta_f_deg + ...
    avlAero.fit.elevator.dCL * delta_e_deg;

CD = polyval(avlAero.fit.alpha.CD, alpha_deg) + ...
    avlAero.fit.flap.dCD * delta_f_deg^2 + ...
    avlAero.fit.aileron.dCD * delta_a_deg^2 + ...
    avlAero.fit.elevator.dCD * delta_e_deg^2 + ...
    avlAero.fit.rudder.dCD * delta_r_deg^2;

CY = avlAero.fit.beta.CY * beta_deg + ...
    avlAero.fit.aileron.dCY * delta_a_deg + ...
    avlAero.fit.rudder.dCY * delta_r_deg;

Cl = avlAero.fit.beta.Cl * beta_deg + ...
    avlAero.fit.aileron.dCl * delta_a_deg + ...
    avlAero.fit.rudder.dCl * delta_r_deg;

Cm = polyval(avlAero.fit.alpha.Cm, alpha_deg) + ...
    avlAero.fit.flap.dCm * delta_f_deg + ...
    avlAero.fit.elevator.dCm * delta_e_deg;

Cn = avlAero.fit.beta.Cn * beta_deg + ...
    avlAero.fit.aileron.dCn * delta_a_deg + ...
    avlAero.fit.rudder.dCn * delta_r_deg;

qS = 0.5 * rho * Vinf^2 * avlAero.Sref;

xw = normalizeVec(V_BA);
zBodyDown = [0.0; 0.0; 1.0];
yw = cross(zBodyDown, xw);
if norm(yw) < 1.0e-12
    yw = [0.0; 1.0; 0.0];
else
    yw = yw / norm(yw);
end
zw = cross(xw, yw);
zw = normalizeVec(zw);

F_w = qS * [-CD; CY; -CL];
F_aero_B = xw * F_w(1) + yw * F_w(2) + zw * F_w(3);
M_aero_B = qS * [avlAero.Bref * Cl; avlAero.Cref * Cm; avlAero.Bref * Cn];

coeffs = [CL; CD; CY; Cl; Cm; Cn];
state = [Vinf; alpha_deg; beta_deg; delta_f_deg; delta_a_deg; delta_e_deg; delta_r_deg];

end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function v = normalizeVec(v)
n = norm(v);
if n < 1.0e-12
    v = zeros(size(v));
else
    v = v / n;
end
end
