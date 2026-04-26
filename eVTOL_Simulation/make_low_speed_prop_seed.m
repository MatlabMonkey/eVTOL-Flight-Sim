function [seed, details] = make_low_speed_prop_seed(initData, trimCase)
%MAKE_LOW_SPEED_PROP_FIRST_PASS_SEED Build a physics-based low-speed seed.
%
% This is the reusable part of the old low-speed scored sweep. It estimates
% front/rear prop RPM and theta by balancing weight, drag, and pitch moment
% using the active wing/tail polars. It does not trim the model; it only
% gives the optimizer a better starting point near hover/low-speed flight.

if nargin < 2 || ~isstruct(initData) || ~isstruct(trimCase)
    error('make_low_speed_prop_seed requires initData and trimCase structs.');
end

aircraft = initData.aircraft;
wingPolar = initData.defaults.wingPolar;
tailPolar = initData.defaults.tailPolar;

vinf_mps = localGetField(trimCase, 'Vinf_mps', 0.0);
tilt_deg = localGetField(trimCase, 'front_tilt_deg', 0.0);

qbar = 0.5 * aircraft.rho * vinf_mps^2;
weight_N = aircraft.Mass * aircraft.g;

wing_pos = aircraft.wing.pos(:) - aircraft.CG(:);
tail_pos = ((aircraft.tailL.pos(:) + aircraft.tailR.pos(:)) / 2.0) - aircraft.CG(:);
front_pos = localMeanRotorPos(aircraft.compData, 'F-Rotor ') - aircraft.CG(:);
rear_pos = localMeanRotorPos(aircraft.compData, 'R-Rotor ') - aircraft.CG(:);

wing_area = aircraft.wing.S;
tail_area = aircraft.tailL.S + aircraft.tailR.S;
tail_vertical_proj = abs(aircraft.tailL.n(3)) / norm(aircraft.tailL.n);
k_thrust = aircraft.prop.k_Thrust;

front_arm_x = max(front_pos(1), 1e-6);
rear_arm_x = max(abs(rear_pos(1)), 1e-6);
front_support_fraction = rear_arm_x / (front_arm_x + rear_arm_x);
rear_support_fraction = front_arm_x / (front_arm_x + rear_arm_x);

sin_tilt = sind(tilt_deg);
cos_tilt = cosd(tilt_deg);
cos_tilt_safe = sign(cos_tilt) * max(abs(cos_tilt), 1e-3);

alpha_grid_deg = unique([(-4:0.25:18), localGetField(trimCase, 'theta_guess_deg', 0.0)]);
alpha_grid_deg = alpha_grid_deg(:);
best = struct('cost', inf);

for i = 1:numel(alpha_grid_deg)
    alpha_deg = alpha_grid_deg(i);
    wingCoeff = localEvalPolar(wingPolar, alpha_deg);
    tailCoeff = localEvalPolar(tailPolar, alpha_deg);

    lift_N = qbar * (wing_area * wingCoeff.CL + tail_area * tailCoeff.CL * tail_vertical_proj);
    drag_N = qbar * (wing_area * wingCoeff.CD + tail_area * tailCoeff.CD);

    prop_vertical_support_N = max(weight_N - lift_N, 0.0);
    front_vertical_support_N = front_support_fraction * prop_vertical_support_N;
    rear_thrust_N = rear_support_fraction * prop_vertical_support_N;
    front_thrust_N = max(front_vertical_support_N / cos_tilt_safe, 0.0);

    wing_force_B = [-qbar * wing_area * wingCoeff.CD; 0.0; -qbar * wing_area * wingCoeff.CL];
    tail_force_B = [-qbar * tail_area * tailCoeff.CD; 0.0; -qbar * tail_area * tailCoeff.CL * tail_vertical_proj];
    front_force_B = [front_thrust_N * sin_tilt; 0.0; -front_thrust_N * cos_tilt];
    rear_force_B = [0.0; 0.0; -rear_thrust_N];

    net_pitch_moment_Nm = localPitchMomentY(wing_pos, wing_force_B) + ...
        localPitchMomentY(tail_pos, tail_force_B) + ...
        localPitchMomentY(front_pos, front_force_B) + ...
        localPitchMomentY(rear_pos, rear_force_B);

    forward_residual_N = front_thrust_N * sin_tilt - drag_N;
    vertical_residual_N = weight_N - lift_N - front_thrust_N * cos_tilt - rear_thrust_N;

    cost = abs(net_pitch_moment_Nm) / max(weight_N * aircraft.wing.c, 1.0) + ...
        abs(forward_residual_N) / max(weight_N, 1.0) + ...
        0.5 * abs(vertical_residual_N) / max(weight_N, 1.0) + ...
        0.02 * abs(alpha_deg - localGetField(trimCase, 'theta_guess_deg', 0.0));

    if cost < best.cost
        best.cost = cost;
        best.alpha_deg = alpha_deg;
        best.front_thrust_N = front_thrust_N;
        best.rear_thrust_N = rear_thrust_N;
        best.front_collective_rpm = sqrt(max(front_thrust_N / (6.0 * k_thrust), 0.0));
        best.rear_collective_rpm = sqrt(max(rear_thrust_N / (6.0 * k_thrust), 0.0));
        best.forward_residual_N = forward_residual_N;
        best.vertical_residual_N = vertical_residual_N;
        best.net_pitch_moment_Nm = net_pitch_moment_Nm;
        best.drag_N = drag_N;
        best.lift_N = lift_N;
    end
end

if ~isfinite(best.cost)
    error('No feasible props-only seed found for tilt %.1f deg, Vinf %.1f m/s.', tilt_deg, vinf_mps);
end

seed = struct();
seed.name = sprintf('prop_first_pass_tilt_%s_V_%s', localValueLabel(tilt_deg), localValueLabel(vinf_mps));
seed.front_collective_guess_rpm = best.front_collective_rpm;
seed.rear_collective_guess_rpm = best.rear_collective_rpm;
seed.theta_guess_deg = best.alpha_deg;
seed.delta_f_guess_deg = 0.0;
seed.delta_a_guess_deg = 0.0;
seed.delta_e_guess_deg = 0.0;
seed.delta_r_guess_deg = 0.0;

details = struct('best', best, 'vinf_mps', vinf_mps, 'tilt_deg', tilt_deg);
end

function coeff = localEvalPolar(polar, alpha_deg)
alpha_data = polar.alpha_deg(:);
coeff = struct();
coeff.CL = interp1(alpha_data, polar.CL(:), alpha_deg, 'linear', 'extrap');
coeff.CD = interp1(alpha_data, polar.CD(:), alpha_deg, 'linear', 'extrap');
coeff.Cm = interp1(alpha_data, polar.Cm(:), alpha_deg, 'linear', 'extrap');
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

function moment_y = localPitchMomentY(r_B, f_B)
moment_B = cross(r_B(:), f_B(:));
moment_y = moment_B(2);
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
