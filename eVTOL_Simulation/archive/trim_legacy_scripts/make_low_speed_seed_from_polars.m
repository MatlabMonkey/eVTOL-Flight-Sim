function [seed, details] = make_low_speed_seed_from_polars(initData, trimCase)
%MAKE_LOW_SPEED_SEED_FROM_POLARS Build a LUT-based low-speed trim seed.
%
% This helper uses the active wing/tail alpha lookup tables plus a simple
% longitudinal force/moment balance to generate a better initial guess for
% low-speed transition trims before calling findop.

    if nargin < 2
        error('make_low_speed_seed_from_polars requires initData and trimCase.');
    end
    if ~isstruct(initData) || ~isstruct(trimCase)
        error('initData and trimCase must both be structs.');
    end

    aircraft = initData.aircraft;
    wingPolar = initData.defaults.wingPolar;
    tailPolar = initData.defaults.tailPolar;

    seed = struct();
    details = struct();

    vinf_mps = localGetField(trimCase, 'Vinf_mps', 0.0);
    tilt_deg = localGetField(trimCase, 'front_tilt_deg', 0.0);
    rear_fixed_rpm = localGetRearRpm(trimCase);
    front_installed_thrust_factor = localGetField(trimCase, ...
        'front_installed_thrust_factor', 1.45);

    rho = aircraft.rho;
    q = 0.5 * rho * vinf_mps^2;
    weight_N = aircraft.Mass * aircraft.g;

    tail_vertical_proj = abs(aircraft.tailL.n(3)) / norm(aircraft.tailL.n);
    wing_pos = aircraft.wing.pos(:) - aircraft.CG(:);
    tail_pos = ((aircraft.tailL.pos(:) + aircraft.tailR.pos(:)) / 2.0) - aircraft.CG(:);
    front_pos = localMeanRotorPos(aircraft.compData, 'F-Rotor ') - aircraft.CG(:);
    rear_pos = localMeanRotorPos(aircraft.compData, 'R-Rotor ') - aircraft.CG(:);

    s_w = aircraft.wing.S;
    c_w = aircraft.wing.c;
    s_t = aircraft.tailL.S + aircraft.tailR.S;
    c_t = 0.5 * (aircraft.tailL.c + aircraft.tailR.c);

    cl_df = max(1e-6, -aircraft.controls.flaperon.expected.CZ_delta_f_per_rad);
    cm_de = aircraft.controls.ruddervator.expected.CM_delta_e_per_rad;

    rear_thrust_N = 6.0 * aircraft.prop.k_Thrust * rear_fixed_rpm^2;
    sin_tilt = sind(tilt_deg);
    cos_tilt = cosd(tilt_deg);
    sin_tilt_safe = sign(sin_tilt) * max(abs(sin_tilt), 1e-3);
    cos_tilt_safe = sign(cos_tilt) * max(abs(cos_tilt), 1e-3);

    alpha_grid_deg = unique([(-2:0.25:18), localGetField(trimCase, 'theta_guess_deg', 0.0)]);
    alpha_grid_deg = alpha_grid_deg(:);

    best = struct('cost', inf);
    flap_limit_deg = localSurfaceLimitDeg(trimCase, 25.0);
    flap_limit_rad = deg2rad(flap_limit_deg);
    elevator_limit_rad = flap_limit_rad;

    for i = 1:numel(alpha_grid_deg)
        alpha_deg = alpha_grid_deg(i);
        [wingCoeff, wingSlope] = localEvalPolar(wingPolar, alpha_deg);
        [tailCoeff, ~] = localEvalPolar(tailPolar, alpha_deg);

        lift_base_N = q * (s_w * wingCoeff.CL + s_t * tailCoeff.CL * tail_vertical_proj);
        drag_base_N = q * (s_w * wingCoeff.CD + s_t * tailCoeff.CD);

        front_from_drag_N = max(drag_base_N / sin_tilt_safe, 0.0);

        lift_gap_if_drag_only_N = weight_N - rear_thrust_N - front_from_drag_N * cos_tilt - lift_base_N;
        if q > 1.0
            delta_f_req_rad = lift_gap_if_drag_only_N / (q * s_w * cl_df);
        else
            delta_f_req_rad = 0.0;
        end
        delta_f_clamped_rad = max(-flap_limit_rad, min(flap_limit_rad, delta_f_req_rad));
        flap_lift_N = q * s_w * cl_df * delta_f_clamped_rad;

        front_vertical_need_N = max((weight_N - rear_thrust_N - lift_base_N - flap_lift_N) / cos_tilt_safe, 0.0);
        front_thrust_N = max(front_vertical_need_N, front_installed_thrust_factor * front_from_drag_N);

        lift_gap_with_front_N = weight_N - rear_thrust_N - front_thrust_N * cos_tilt - lift_base_N;
        if q > 1.0
            delta_f_req_rad = lift_gap_with_front_N / (q * s_w * cl_df);
        else
            delta_f_req_rad = 0.0;
        end
        delta_f_clamped_rad = max(-flap_limit_rad, min(flap_limit_rad, delta_f_req_rad));
        flap_lift_N = q * s_w * cl_df * delta_f_clamped_rad;

        wing_force_B = [-q * s_w * wingCoeff.CD; 0.0; -q * s_w * wingCoeff.CL];
        tail_force_B = [-q * s_t * tailCoeff.CD; 0.0; -q * s_t * tailCoeff.CL * tail_vertical_proj];
        flap_force_B = [0.0; 0.0; -flap_lift_N];
        front_force_B = [front_thrust_N * sin_tilt; 0.0; -front_thrust_N * cos_tilt];
        rear_force_B = [0.0; 0.0; -rear_thrust_N];

        % For the first seed pass, use force-arm moments only. The tabulated
        % airfoil Cm values are still useful diagnostics, but they do not map
        % cleanly to a whole-aircraft pitch moment here without a tighter
        % reference-area/reference-center treatment.
        wing_moment_y_Nm = localPitchMomentY(wing_pos, wing_force_B);
        tail_moment_y_Nm = localPitchMomentY(tail_pos, tail_force_B);
        flap_moment_y_Nm = localPitchMomentY(wing_pos, flap_force_B);
        front_moment_y_Nm = localPitchMomentY(front_pos, front_force_B);
        rear_moment_y_Nm = localPitchMomentY(rear_pos, rear_force_B);

        net_pitch_moment_Nm = wing_moment_y_Nm + tail_moment_y_Nm + flap_moment_y_Nm + ...
                              front_moment_y_Nm + rear_moment_y_Nm;

        if q > 1.0 && abs(cm_de) > 1e-8
            delta_e_req_rad = -net_pitch_moment_Nm / (q * s_w * c_w * cm_de);
        else
            delta_e_req_rad = 0.0;
        end
        delta_e_schedule_deg = -4.0 ...
            - 0.04 * max(tilt_deg - 20.0, 0.0) ...
            - 0.02 * max(50.0 - vinf_mps, 0.0);
        delta_e_schedule_rad = deg2rad(delta_e_schedule_deg);
        if abs(delta_e_req_rad) <= 0.8 * elevator_limit_rad
            delta_e_clamped_rad = max(-elevator_limit_rad, min(elevator_limit_rad, delta_e_req_rad));
        else
            delta_e_clamped_rad = max(-elevator_limit_rad, min(elevator_limit_rad, delta_e_schedule_rad));
        end

        vertical_residual_N = weight_N - rear_thrust_N - front_thrust_N * cos_tilt - lift_base_N - flap_lift_N;
        forward_residual_N = front_thrust_N * sin_tilt - drag_base_N;
        flap_overflow_rad = delta_f_req_rad - delta_f_clamped_rad;
        elevator_overflow_rad = delta_e_req_rad - delta_e_clamped_rad;

        cost = abs(vertical_residual_N) / weight_N + ...
               abs(forward_residual_N) / max(weight_N, 1.0) + ...
               0.5 * abs(flap_overflow_rad) / max(flap_limit_rad, 1e-3) + ...
               0.25 * abs(elevator_overflow_rad) / max(elevator_limit_rad, 1e-3) + ...
               0.05 * abs(alpha_deg - localGetField(trimCase, 'theta_guess_deg', 0.0));

        if cost < best.cost
            best.cost = cost;
            best.alpha_deg = alpha_deg;
            best.front_thrust_N = front_thrust_N;
            best.front_collective_rpm = sqrt(max(front_thrust_N / (6.0 * aircraft.prop.k_Thrust), 0.0));
            best.delta_f_deg = rad2deg(delta_f_clamped_rad);
            best.delta_e_deg = rad2deg(delta_e_clamped_rad);
            best.forward_residual_N = forward_residual_N;
            best.vertical_residual_N = vertical_residual_N;
            best.net_pitch_moment_Nm = net_pitch_moment_Nm;
            best.wingCoeff = wingCoeff;
            best.tailCoeff = tailCoeff;
            best.wingSlope = wingSlope;
        end
    end

    seed.name = sprintf('polar_seed_tilt_%s_V_%s_rear_%s', ...
        localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(rear_fixed_rpm));
    seed.front_collective_guess_rpm = best.front_collective_rpm;
    seed.rear_collective_guess_rpm = rear_fixed_rpm;
    seed.theta_guess_deg = best.alpha_deg;
    seed.delta_f_guess_deg = best.delta_f_deg;
    seed.delta_e_guess_deg = best.delta_e_deg;
    seed.delta_a_guess_deg = 0.0;
    seed.delta_r_guess_deg = 0.0;

    details.best = best;
    details.vinf_mps = vinf_mps;
    details.tilt_deg = tilt_deg;
    details.rear_fixed_rpm = rear_fixed_rpm;
    details.rear_thrust_N = rear_thrust_N;
    details.q_Pa = q;
    details.wingPolarName = localGetField(wingPolar, 'name', 'wingPolar');
    details.tailPolarName = localGetField(tailPolar, 'name', 'tailPolar');
end

function [coeff, slope] = localEvalPolar(polar, alpha_deg)
    alpha_data = polar.alpha_deg(:);
    coeff = struct();
    coeff.CL = interp1(alpha_data, polar.CL(:), alpha_deg, 'linear', 'extrap');
    coeff.CD = interp1(alpha_data, polar.CD(:), alpha_deg, 'linear', 'extrap');
    coeff.Cm = interp1(alpha_data, polar.Cm(:), alpha_deg, 'linear', 'extrap');

    d_alpha = 0.5;
    cl_plus = interp1(alpha_data, polar.CL(:), alpha_deg + d_alpha, 'linear', 'extrap');
    cl_minus = interp1(alpha_data, polar.CL(:), alpha_deg - d_alpha, 'linear', 'extrap');
    cd_plus = interp1(alpha_data, polar.CD(:), alpha_deg + d_alpha, 'linear', 'extrap');
    cd_minus = interp1(alpha_data, polar.CD(:), alpha_deg - d_alpha, 'linear', 'extrap');
    cm_plus = interp1(alpha_data, polar.Cm(:), alpha_deg + d_alpha, 'linear', 'extrap');
    cm_minus = interp1(alpha_data, polar.Cm(:), alpha_deg - d_alpha, 'linear', 'extrap');

    slope = struct();
    slope.dCL_ddeg = (cl_plus - cl_minus) / (2.0 * d_alpha);
    slope.dCD_ddeg = (cd_plus - cd_minus) / (2.0 * d_alpha);
    slope.dCm_ddeg = (cm_plus - cm_minus) / (2.0 * d_alpha);
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

function rear_rpm = localGetRearRpm(trimCase)
    if localGetField(trimCase, 'rear_collective_known', false)
        rear_rpm = localGetField(trimCase, 'rear_collective_fixed_rpm', ...
            localGetField(trimCase, 'rear_collective_guess_rpm', 0.0));
    else
        rear_rpm = localGetField(trimCase, 'rear_collective_guess_rpm', ...
            localGetField(trimCase, 'rear_collective_trim_rpm', 0.0));
    end
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
