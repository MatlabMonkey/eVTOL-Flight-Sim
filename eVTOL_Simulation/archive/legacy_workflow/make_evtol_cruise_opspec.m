function [opspec, resolved] = make_evtol_cruise_opspec(request, trim_setup_defaults)
% make_evtol_cruise_opspec
%
% Reusable opspec builder for the active Trim_Plant cruise trim workflow.
% Expected interface:
%   States:
%     euler_attitude_state (3)
%     rotor_actuator_FL/FR/RL/RR (1 each)
%     tilt_actuator_FL/FR (1 each)
%     body_rates_state (3)
%     body_velocity_state (3)
%     ned_position_state (3)
%     tailL/tailR/wingL/wingR servo states (1 each)
%   Inputs:
%     Motor_RPM_cmd(4)      = [FR FL RR RL]
%     Tilt_angles_cmd(2)    = [FR FL]
%     Front_RPM_collective  = scalar
%     Rear_RPM_collective   = scalar
%     delta_LW, delta_RW, delta_LT, delta_RT

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
    resolved.front_tilt_cmd_deg = pick_field(request, 'front_tilt_cmd_deg', resolved.front_tilt_deg);

    resolved.motor_rpm_cmd = pick_field(request, ...
        'motor_rpm_cmd', pick_field(trim_setup_defaults, 'motor_rpm_cmd_default', zeros(4,1)));

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

    %% Create opspec and validate active trim-model layout
    opspec = operspec(resolved.model_name);
    validate_trim_opspec_layout(opspec);

    resolved.state_layout = 'grouped_named';
    resolved.input_layout = 'grouped_named';
    resolved.n_motor_cmd = numel(opspec.Inputs(1).u);
    resolved.n_tilt_cmd = numel(opspec.Inputs(2).u);
    resolved.motor_rpm_cmd = canonicalize_motor_cmd(resolved.motor_rpm_cmd, resolved.n_motor_cmd);
    resolved.front_tilt_cmd_deg = canonicalize_tilt_cmd( ...
        resolved.front_tilt_cmd_deg, resolved.n_tilt_cmd);

    %% State constraints
    idx_eul   = find_state_idx_by_block(opspec, 'euler_attitude_state');
    idx_rates = find_state_idx_by_block(opspec, 'body_rates_state');
    idx_vel   = find_state_idx_by_block(opspec, 'body_velocity_state');
    idx_pos   = find_state_idx_by_block(opspec, 'ned_position_state');

    idx_rotor_FL = find_state_idx_by_block(opspec, 'rotor_actuator_FL');
    idx_rotor_FR = find_state_idx_by_block(opspec, 'rotor_actuator_FR');
    idx_rotor_RL = find_state_idx_by_block(opspec, 'rotor_actuator_RL');
    idx_rotor_RR = find_state_idx_by_block(opspec, 'rotor_actuator_RR');

    idx_tilt_FL = find_state_idx_by_block(opspec, 'tilt_actuator_FL');
    idx_tilt_FR = find_state_idx_by_block(opspec, 'tilt_actuator_FR');

    idx_dLT = find_state_idx_by_block(opspec, 'tailL_servo');
    idx_dRT = find_state_idx_by_block(opspec, 'tailR_servo');
    idx_dLW = find_state_idx_by_block(opspec, 'wingL_servo');
    idx_dRW = find_state_idx_by_block(opspec, 'wingR_servo');

    rotor_guess = resolved.motor_rpm_cmd;
    if numel(rotor_guess) == 4 && all(rotor_guess == 0)
        rotor_guess = [resolved.front_collective_guess_rpm; ...
                       resolved.front_collective_guess_rpm; ...
                       resolved.rear_collective_guess_rpm; ...
                       resolved.rear_collective_guess_rpm];
    end

    opspec.States(idx_rotor_FL).x = rotor_guess(2);
    opspec.States(idx_rotor_FL).Known = false;
    opspec.States(idx_rotor_FL).SteadyState = true;

    opspec.States(idx_rotor_FR).x = rotor_guess(1);
    opspec.States(idx_rotor_FR).Known = false;
    opspec.States(idx_rotor_FR).SteadyState = true;

    opspec.States(idx_rotor_RL).x = rotor_guess(4);
    opspec.States(idx_rotor_RL).Known = false;
    opspec.States(idx_rotor_RL).SteadyState = true;

    opspec.States(idx_rotor_RR).x = rotor_guess(3);
    opspec.States(idx_rotor_RR).Known = false;
    opspec.States(idx_rotor_RR).SteadyState = true;

    opspec.States(idx_tilt_FL).x = resolved.front_tilt_cmd_deg(min(2, resolved.n_tilt_cmd));
    opspec.States(idx_tilt_FL).Known = true;
    opspec.States(idx_tilt_FL).SteadyState = true;

    opspec.States(idx_tilt_FR).x = resolved.front_tilt_cmd_deg(1);
    opspec.States(idx_tilt_FR).Known = true;
    opspec.States(idx_tilt_FR).SteadyState = true;

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

    %% Input constraints
    opspec.Inputs(1).u = resolved.motor_rpm_cmd;
    opspec.Inputs(1).Known = true(resolved.n_motor_cmd, 1);
    opspec.Inputs(1).Min = zeros(resolved.n_motor_cmd, 1);
    opspec.Inputs(1).Max = inf(resolved.n_motor_cmd, 1);

    opspec.Inputs(2).u = resolved.front_tilt_cmd_deg;
    opspec.Inputs(2).Known = true(resolved.n_tilt_cmd, 1);

    opspec.Inputs(3).u = resolved.front_collective_guess_rpm;
    opspec.Inputs(3).Known = resolved.front_collective_known;
    opspec.Inputs(3).Min = 0.0;
    opspec.Inputs(3).Max = resolved.front_collective_max_rpm;
    if resolved.front_collective_known
        opspec.Inputs(3).u = resolved.front_collective_fixed_rpm;
    end

    opspec.Inputs(4).u = resolved.rear_collective_guess_rpm;
    opspec.Inputs(4).Known = resolved.rear_collective_known;
    opspec.Inputs(4).Min = 0.0;
    opspec.Inputs(4).Max = resolved.rear_collective_max_rpm;
    if resolved.rear_collective_known
        opspec.Inputs(4).u = resolved.rear_collective_fixed_rpm;
    end

    for i_in = 5:8
        opspec.Inputs(i_in).u = 0.0;
        opspec.Inputs(i_in).Known = false;
        opspec.Inputs(i_in).Min = -resolved.surface_limit_rad;
        opspec.Inputs(i_in).Max =  resolved.surface_limit_rad;
    end
end

function value = pick_field(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    else
        value = default_value;
    end
end

function motor_cmd = canonicalize_motor_cmd(motor_cmd_in, n_motor)
    motor_cmd = motor_cmd_in;
    if isscalar(motor_cmd)
        motor_cmd = motor_cmd * ones(n_motor, 1);
    else
        motor_cmd = motor_cmd(:);
    end

    if numel(motor_cmd) ~= n_motor
        error('motor_rpm_cmd must be a scalar or %dx1.', n_motor);
    end
end

function tilt_cmd = canonicalize_tilt_cmd(tilt_cmd_in, n_tilt)
    tilt_cmd = tilt_cmd_in;
    if isscalar(tilt_cmd)
        tilt_cmd = tilt_cmd * ones(n_tilt, 1);
    else
        tilt_cmd = tilt_cmd(:);
    end

    if numel(tilt_cmd) ~= n_tilt
        error('front_tilt_cmd_deg must be a scalar or %dx1.', n_tilt);
    end
end

function idx = find_state_idx_by_block(opspec, block_fragment)
    idx = [];
    for i = 1:numel(opspec.States)
        try
            blk = char(opspec.States(i).Block);
        catch
            blk = '';
        end
        if contains(blk, block_fragment)
            idx = i;
            return;
        end
    end
    error('Unable to find opspec state containing block fragment "%s".', block_fragment);
end

function validate_trim_opspec_layout(opspec)
    if numel(opspec.States) ~= 14
        error('Expected Trim_Plant to expose 14 opspec states, found %d.', numel(opspec.States));
    end
    if numel(opspec.Inputs) ~= 8
        error('Expected Trim_Plant to expose 8 opspec inputs, found %d.', numel(opspec.Inputs));
    end

    required_state_blocks = { ...
        'euler_attitude_state', ...
        'body_rates_state', ...
        'body_velocity_state', ...
        'ned_position_state', ...
        'rotor_actuator_FL', ...
        'rotor_actuator_FR', ...
        'rotor_actuator_RL', ...
        'rotor_actuator_RR', ...
        'tilt_actuator_FL', ...
        'tilt_actuator_FR', ...
        'tailL_servo', ...
        'tailR_servo', ...
        'wingL_servo', ...
        'wingR_servo'};

    for i = 1:numel(required_state_blocks)
        find_state_idx_by_block(opspec, required_state_blocks{i});
    end
end
