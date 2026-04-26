function [opspec, resolved] = make_evtol_cruise_opspec(request, trim_setup_defaults)
% make_evtol_cruise_opspec
%
% Reusable opspec builder for the EVTOL cruise trim workflow.
% Supports both:
%   1) grouped opspec layout
%   2) packed single-state / single-input opspec layout

    if nargin < 1 || isempty(request)
        request = struct();
    end
    if nargin < 2 || isempty(trim_setup_defaults)
        error('trim_setup_defaults is required.');
    end

    %% Resolve request against defaults
    resolved = struct();

    resolved.model_name = pick_field(request, 'model_name', trim_setup_defaults.model_name);

    resolved.Vinf_mps = pick_field(request, 'Vinf_mps', trim_setup_defaults.Vinf_trim_target);

    % Default cruise assumption: body velocity aligned with body x-axis.
    resolved.u_body_mps = pick_field(request, 'u_body_mps', resolved.Vinf_mps);
    resolved.v_body_mps = pick_field(request, 'v_body_mps', 0.0);
    resolved.w_body_mps = pick_field(request, 'w_body_mps', 0.0);

    resolved.phi_deg = pick_field(request, 'phi_deg', 0.0);
    resolved.theta_guess_deg = pick_field(request, 'theta_guess_deg', 0.0);
    resolved.psi_deg = pick_field(request, 'psi_deg', 0.0);

    resolved.pos_init = pick_field(request, 'pos_init', trim_setup_defaults.pos_init);

    resolved.front_tilt_deg = pick_field(request, 'front_tilt_deg', trim_setup_defaults.front_tilt_trim_deg);
    resolved.front_tilt_cmd_deg = pick_field(request, 'front_tilt_cmd_deg', resolved.front_tilt_deg * ones(6,1));

    resolved.motor_rpm_cmd = pick_field(request, 'motor_rpm_cmd', zeros(12,1));

    resolved.front_collective_guess_rpm = pick_field(request, ...
        'front_collective_guess_rpm', trim_setup_defaults.front_collective_guess_rpm);
    resolved.rear_collective_guess_rpm = pick_field(request, ...
        'rear_collective_guess_rpm', trim_setup_defaults.rear_collective_guess_rpm);

    resolved.front_collective_max_rpm = pick_field(request, ...
        'front_collective_max_rpm', trim_setup_defaults.front_collective_max_rpm);
    resolved.rear_collective_max_rpm = pick_field(request, ...
        'rear_collective_max_rpm', trim_setup_defaults.rear_collective_max_rpm);

    resolved.front_collective_known = pick_field(request, 'front_collective_known', false);
    resolved.rear_collective_known = pick_field(request, 'rear_collective_known', false);

    resolved.front_collective_fixed_rpm = pick_field(request, ...
        'front_collective_fixed_rpm', resolved.front_collective_guess_rpm);
    resolved.rear_collective_fixed_rpm = pick_field(request, ...
        'rear_collective_fixed_rpm', resolved.rear_collective_guess_rpm);

    resolved.surface_limit_rad = pick_field(request, ...
        'surface_limit_rad', trim_setup_defaults.surface_limit_rad);

    %% Shape checks
    resolved.pos_init = resolved.pos_init(:);
    if numel(resolved.pos_init) ~= 3
        error('pos_init must be 3x1.');
    end

    resolved.motor_rpm_cmd = resolved.motor_rpm_cmd(:);
    if numel(resolved.motor_rpm_cmd) ~= 12
        error('motor_rpm_cmd must be 12x1.');
    end

    tilt_cmd = resolved.front_tilt_cmd_deg;
    if isscalar(tilt_cmd)
        tilt_cmd = tilt_cmd * ones(6,1);
    else
        tilt_cmd = tilt_cmd(:);
    end
    if numel(tilt_cmd) ~= 6
        error('front_tilt_cmd_deg must be a scalar or 6x1.');
    end
    resolved.front_tilt_cmd_deg = tilt_cmd;

    %% Create opspec and detect layout
    opspec = operspec(resolved.model_name);

    state_layout = detect_state_layout(opspec);
    input_layout = detect_input_layout(opspec);

    resolved.state_layout = state_layout;
    resolved.input_layout = input_layout;

    %% State constraints
    switch state_layout
        case 'grouped'
            idx_dLT   = 1;
            idx_dLW   = 2;
            idx_dRT   = 3;
            idx_dRW   = 4;
            idx_eul   = 5;
            idx_rates = 6;
            idx_vel   = 7;
            idx_pos   = 8;

            for k = [idx_dLT, idx_dLW, idx_dRT, idx_dRW]
                opspec.States(k).x = 0.0;
                opspec.States(k).Known = false;
                opspec.States(k).SteadyState = true;
            end

            opspec.States(idx_eul).x = [deg2rad(resolved.phi_deg); ...
                                        deg2rad(resolved.theta_guess_deg); ...
                                        deg2rad(resolved.psi_deg)];
            opspec.States(idx_eul).Known = [true; false; true];
            opspec.States(idx_eul).SteadyState = [true; true; true];

            opspec.States(idx_rates).x = [0.0; 0.0; 0.0];
            opspec.States(idx_rates).Known = [true; true; true];
            opspec.States(idx_rates).SteadyState = [true; true; true];

            opspec.States(idx_vel).x = [resolved.u_body_mps; ...
                                        resolved.v_body_mps; ...
                                        resolved.w_body_mps];
            opspec.States(idx_vel).Known = [true; true; true];
            opspec.States(idx_vel).SteadyState = [true; true; true];

            opspec.States(idx_pos).x = resolved.pos_init;
            opspec.States(idx_pos).Known = [false; false; false];
            opspec.States(idx_pos).SteadyState = [false; false; true];

        case 'packed'
            if opspec.States(1).Nx < 16
                error('Packed state layout detected, but Nx=%d < 16.', opspec.States(1).Nx);
            end

            x = opspec.States(1).x(:);
            if numel(x) ~= opspec.States(1).Nx
                x = zeros(opspec.States(1).Nx, 1);
            end

            known = false(opspec.States(1).Nx, 1);
            steady = true(opspec.States(1).Nx, 1);

            % Packed state order assumed from your current linearization comments:
            % 1 dLT, 2 dLW, 3 dRT, 4 dRW, 5 phi, 6 theta, 7 psi,
            % 8 P, 9 Q, 10 R, 11 u, 12 v, 13 w, 14 x, 15 y, 16 z

            x(1:4) = 0.0;
            known(1:4) = false;
            steady(1:4) = true;

            x(5:7) = [deg2rad(resolved.phi_deg); ...
                      deg2rad(resolved.theta_guess_deg); ...
                      deg2rad(resolved.psi_deg)];
            known(5:7) = [true; false; true];
            steady(5:7) = [true; true; true];

            x(8:10) = [0.0; 0.0; 0.0];
            known(8:10) = [true; true; true];
            steady(8:10) = [true; true; true];

            x(11:13) = [resolved.u_body_mps; ...
                        resolved.v_body_mps; ...
                        resolved.w_body_mps];
            known(11:13) = [true; true; true];
            steady(11:13) = [true; true; true];

            x(14:16) = resolved.pos_init;
            known(14:16) = [false; false; false];
            steady(14:16) = [false; false; true];

            opspec.States(1).x = x;
            opspec.States(1).Known = known;
            opspec.States(1).SteadyState = steady;

        otherwise
            error('Unsupported state layout: %s', state_layout);
    end

    %% Input constraints
    switch input_layout
        case 'grouped'
            % 1 = Motor_RPM_cmd(12)
            % 2 = Tilt_angles_cmd(6)
            % 3 = Front_RPM_collective
            % 4 = Rear_RPM_collective
            % 5 = deltaLW
            % 6 = deltaRW
            % 7 = deltaLT
            % 8 = deltaRT

            opspec.Inputs(1).u = resolved.motor_rpm_cmd;
            opspec.Inputs(1).Known = true(12,1);

            opspec.Inputs(2).u = resolved.front_tilt_cmd_deg;
            opspec.Inputs(2).Known = true(6,1);

            if resolved.front_collective_known
                opspec.Inputs(3).u = resolved.front_collective_fixed_rpm;
                opspec.Inputs(3).Known = true;
            else
                opspec.Inputs(3).u = resolved.front_collective_guess_rpm;
                opspec.Inputs(3).Known = false;
                opspec.Inputs(3).Min = 0.0;
                opspec.Inputs(3).Max = resolved.front_collective_max_rpm;
            end

            if resolved.rear_collective_known
                opspec.Inputs(4).u = resolved.rear_collective_fixed_rpm;
                opspec.Inputs(4).Known = true;
            else
                opspec.Inputs(4).u = resolved.rear_collective_guess_rpm;
                opspec.Inputs(4).Known = false;
                opspec.Inputs(4).Min = 0.0;
                opspec.Inputs(4).Max = resolved.rear_collective_max_rpm;
            end

            for i_in = 5:8
                opspec.Inputs(i_in).u = 0.0;
                opspec.Inputs(i_in).Known = false;
                opspec.Inputs(i_in).Min = -resolved.surface_limit_rad;
                opspec.Inputs(i_in).Max =  resolved.surface_limit_rad;
            end

        case 'packed'
            nu = numel(opspec.Inputs(1).u);
            if nu < 24
                error('Packed input layout detected, but numel(opspec.Inputs(1).u)=%d < 24.', nu);
            end

            u = opspec.Inputs(1).u(:);
            if numel(u) ~= nu
                u = zeros(nu,1);
            end

            known = false(nu,1);
            umin = -inf(nu,1);
            umax =  inf(nu,1);

            % Packed input order assumed from your current U_trim_full mapping:
            % 1:12  motor rpm commands
            % 13:18 tilt angle commands
            % 19    front collective
            % 20    rear collective
            % 21    deltaLW
            % 22    deltaRW
            % 23    deltaLT
            % 24    deltaRT

            u(1:12) = resolved.motor_rpm_cmd;
            known(1:12) = true;

            u(13:18) = resolved.front_tilt_cmd_deg;
            known(13:18) = true;

            if resolved.front_collective_known
                u(19) = resolved.front_collective_fixed_rpm;
                known(19) = true;
            else
                u(19) = resolved.front_collective_guess_rpm;
                known(19) = false;
                umin(19) = 0.0;
                umax(19) = resolved.front_collective_max_rpm;
            end

            if resolved.rear_collective_known
                u(20) = resolved.rear_collective_fixed_rpm;
                known(20) = true;
            else
                u(20) = resolved.rear_collective_guess_rpm;
                known(20) = false;
                umin(20) = 0.0;
                umax(20) = resolved.rear_collective_max_rpm;
            end

            u(21:24) = 0.0;
            known(21:24) = false;
            umin(21:24) = -resolved.surface_limit_rad;
            umax(21:24) =  resolved.surface_limit_rad;

            opspec.Inputs(1).u = u;
            opspec.Inputs(1).Known = known;
            opspec.Inputs(1).Min = umin;
            opspec.Inputs(1).Max = umax;

        otherwise
            error('Unsupported input layout: %s', input_layout);
    end
end

function value = pick_field(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    else
        value = default_value;
    end
end

function layout = detect_state_layout(opspec)
    n_state_groups = numel(opspec.States);

    if n_state_groups >= 8
        layout = 'grouped';
    elseif n_state_groups == 1
        layout = 'packed';
    else
        error('Unexpected opspec state layout. numel(opspec.States) = %d', n_state_groups);
    end
end

function layout = detect_input_layout(opspec)
    n_input_groups = numel(opspec.Inputs);

    if n_input_groups >= 8
        layout = 'grouped';
    elseif n_input_groups == 1
        layout = 'packed';
    else
        error('Unexpected opspec input layout. numel(opspec.Inputs) = %d', n_input_groups);
    end
end