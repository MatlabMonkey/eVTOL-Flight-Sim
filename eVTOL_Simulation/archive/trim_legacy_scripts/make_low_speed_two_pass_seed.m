function [seed, details] = make_low_speed_two_pass_seed(initData, trimCase)
%MAKE_LOW_SPEED_TWO_PASS_SEED Build a prop-fixed flap/elevator cleanup seed.
%
% Pass 1:
%   Solve front thrust, rear thrust, and alpha/theta with all aero controls
%   fixed at zero.
%
% Pass 2:
%   Hold the prop-balanced front/rear thrusts fixed and search a small
%   theta/alpha neighborhood. For each candidate alpha, solve the
%   symmetric [delta_f; delta_e] correction that best removes the remaining
%   vertical-force and pitch-moment residuals.

    if nargin < 2
        error('make_low_speed_two_pass_seed requires initData and trimCase.');
    end

    [propSeed, propDetails] = make_low_speed_prop_first_pass_seed(initData, trimCase);

    aircraft = initData.aircraft;
    wingPolar = initData.defaults.wingPolar;
    tailPolar = initData.defaults.tailPolar;

    vinf_mps = localGetField(trimCase, 'Vinf_mps', 0.0);
    tilt_deg = localGetField(trimCase, 'front_tilt_deg', 0.0);
    q = 0.5 * aircraft.rho * vinf_mps^2;
    weight_N = aircraft.Mass * aircraft.g;

    wing_pos = aircraft.wing.pos(:) - aircraft.CG(:);
    tail_pos = ((aircraft.tailL.pos(:) + aircraft.tailR.pos(:)) / 2.0) - aircraft.CG(:);
    front_pos = localMeanRotorPos(aircraft.compData, 'F-Rotor ') - aircraft.CG(:);
    rear_pos = localMeanRotorPos(aircraft.compData, 'R-Rotor ') - aircraft.CG(:);

    tail_vertical_proj = abs(aircraft.tailL.n(3)) / norm(aircraft.tailL.n);
    s_w = aircraft.wing.S;
    s_t = aircraft.tailL.S + aircraft.tailR.S;

    sin_tilt = sind(tilt_deg);
    cos_tilt = cosd(tilt_deg);

    front_thrust_N = 6.0 * aircraft.prop.k_Thrust * propSeed.front_collective_guess_rpm^2;
    rear_thrust_N = 6.0 * aircraft.prop.k_Thrust * propSeed.rear_collective_guess_rpm^2;

    cz_df = aircraft.controls.flaperon.expected.CZ_delta_f_per_rad;
    cz_de = aircraft.controls.ruddervator.expected.CZ_delta_e_per_rad;

    lift_df_per_rad = -q * s_w * cz_df;
    lift_de_per_rad = -q * s_w * cz_de;
    moment_df_per_rad = localPitchMomentY(wing_pos, [0.0; 0.0; -lift_df_per_rad]);
    moment_de_per_rad = localPitchMomentY(tail_pos, [0.0; 0.0; -lift_de_per_rad]);

    flap_limit_deg = localSurfaceLimitDeg(trimCase, 25.0);
    flap_limit_rad = deg2rad(flap_limit_deg);
    elevator_limit_rad = flap_limit_rad;

    alpha_center_deg = propSeed.theta_guess_deg;
    alpha_grid_deg = unique([alpha_center_deg + (-3.0:0.25:3.0), alpha_center_deg]);
    alpha_grid_deg = alpha_grid_deg(:);

    best = struct('cost', inf);

    for i = 1:numel(alpha_grid_deg)
        alpha_deg = alpha_grid_deg(i);
        wingCoeff = localEvalPolar(wingPolar, alpha_deg);
        tailCoeff = localEvalPolar(tailPolar, alpha_deg);

        lift_base_N = q * (s_w * wingCoeff.CL + s_t * tailCoeff.CL * tail_vertical_proj);
        drag_base_N = q * (s_w * wingCoeff.CD + s_t * tailCoeff.CD);

        wing_force_B = [-q * s_w * wingCoeff.CD; 0.0; -q * s_w * wingCoeff.CL];
        tail_force_B = [-q * s_t * tailCoeff.CD; 0.0; -q * s_t * tailCoeff.CL * tail_vertical_proj];
        front_force_B = [front_thrust_N * sin_tilt; 0.0; -front_thrust_N * cos_tilt];
        rear_force_B = [0.0; 0.0; -rear_thrust_N];

        vertical_rhs_N = weight_N - lift_base_N - front_thrust_N * cos_tilt - rear_thrust_N;
        forward_residual_N = front_thrust_N * sin_tilt - drag_base_N;
        pitch_rhs_Nm = -(localPitchMomentY(wing_pos, wing_force_B) + ...
                         localPitchMomentY(tail_pos, tail_force_B) + ...
                         localPitchMomentY(front_pos, front_force_B) + ...
                         localPitchMomentY(rear_pos, rear_force_B));

        A = [lift_df_per_rad, lift_de_per_rad; ...
             moment_df_per_rad, moment_de_per_rad];
        b = [vertical_rhs_N; pitch_rhs_Nm];

        if rcond(A) > 1e-8
            delta_rad = A \ b;
        else
            delta_rad = pinv(A) * b;
        end

        delta_f_rad = max(-flap_limit_rad, min(flap_limit_rad, delta_rad(1)));
        delta_e_rad = max(-elevator_limit_rad, min(elevator_limit_rad, delta_rad(2)));

        vertical_residual_N = vertical_rhs_N - ...
            (lift_df_per_rad * delta_f_rad + lift_de_per_rad * delta_e_rad);
        pitch_residual_Nm = pitch_rhs_Nm - ...
            (moment_df_per_rad * delta_f_rad + moment_de_per_rad * delta_e_rad);

        cost = abs(forward_residual_N) / max(weight_N, 1.0) + ...
               abs(vertical_residual_N) / max(weight_N, 1.0) + ...
               abs(pitch_residual_Nm) / max(weight_N * aircraft.wing.c, 1.0) + ...
               0.04 * abs(alpha_deg - alpha_center_deg) + ...
               0.1 * abs(delta_f_rad) / max(flap_limit_rad, 1e-6) + ...
               0.1 * abs(delta_e_rad) / max(elevator_limit_rad, 1e-6);

        if cost < best.cost
            best.cost = cost;
            best.alpha_deg = alpha_deg;
            best.front_thrust_N = front_thrust_N;
            best.rear_thrust_N = rear_thrust_N;
            best.front_collective_rpm = propSeed.front_collective_guess_rpm;
            best.rear_collective_rpm = propSeed.rear_collective_guess_rpm;
            best.delta_f_rad = delta_f_rad;
            best.delta_e_rad = delta_e_rad;
            best.forward_residual_N = forward_residual_N;
            best.vertical_residual_N = vertical_residual_N;
            best.pitch_residual_Nm = pitch_residual_Nm;
            best.drag_N = drag_base_N;
            best.lift_base_N = lift_base_N;
        end
    end

    if ~isfinite(best.cost)
        error('No feasible two-pass seed found for tilt %.1f deg, Vinf %.1f m/s.', tilt_deg, vinf_mps);
    end

    seed = propSeed;
    seed.name = sprintf('two_pass_tilt_%s_V_%s', ...
        localValueLabel(tilt_deg), localValueLabel(vinf_mps));
    seed.theta_guess_deg = best.alpha_deg;
    seed.delta_f_guess_deg = rad2deg(best.delta_f_rad);
    seed.delta_a_guess_deg = 0.0;
    seed.delta_e_guess_deg = rad2deg(best.delta_e_rad);
    seed.delta_r_guess_deg = 0.0;

    details = struct();
    details.propSeed = propSeed;
    details.propDetails = propDetails;
    details.best = best;
    details.lift_df_per_rad = lift_df_per_rad;
    details.lift_de_per_rad = lift_de_per_rad;
    details.moment_df_per_rad = moment_df_per_rad;
    details.moment_de_per_rad = moment_de_per_rad;
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

function limit_deg = localSurfaceLimitDeg(trimCase, default_value)
    surface_limit_deg = localGetField(trimCase, 'surface_limit_deg', default_value);
    if isscalar(surface_limit_deg)
        limit_deg = surface_limit_deg;
    else
        surface_limit_deg = surface_limit_deg(:);
        limit_deg = surface_limit_deg(1);
    end
end

function value = localGetField(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    else
        value = default_value;
    end
end

function label = localValueLabel(value)
    label = strrep(num2str(value, '%.4g'), '.', 'p');
    label = strrep(label, '-', 'm');
end
