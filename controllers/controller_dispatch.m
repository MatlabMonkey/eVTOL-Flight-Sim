function [motor_rpm_cmd_out, tilt_angles_cmd_out, front_collective_rpm_out, rear_collective_rpm_out, deltaLW, deltaRW, deltaLT, deltaRT] = ...
    controller_dispatch( ...
    airData_cmd, eul_cmd, gps_Pos_cmd, omega_cmd, accel_cmd, ...
    motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm, ...
    airData_meas, eul_meas, gps_Pos_meas, omega_meas, accel_meas, ...
    actuator_state_meas, angular_accel_meas, ...
    controller_id, x_ref, trim_cmd, K_lqr, runtime_g_map, surface_limit_rad, ...
    front_collective_min_rpm, front_collective_max_rpm, ...
    rear_collective_min_rpm, rear_collective_max_rpm, mix_matrix)
%CONTROLLER_DISPATCH Dispatch to one of the controller helper files.
% Wrapper/Controller interface:
%   Commands     = airData_*, eul_*, gps_Pos_*, omega_*, accel_*
%   Actuator cmd = motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm
%   Measurements = airData_meas, eul_meas, gps_Pos_meas, omega_meas,
%                  accel_meas, actuator_state_meas, angular_accel_meas
%
% Controller core convention:
%   x_*    = [phi; theta; psi; u; v; w; p; q; r]
%   trim   = [front_collective_rpm; rear_collective_rpm; delta_f; delta_a; delta_e; delta_r]
%   K_lqr  = 6x9 and produces
%            [front_collective_rpm; rear_collective_rpm; delta_f; delta_a; delta_e; delta_r]
%            For controller_id = 6, this input is repacked as the INDI
%            schedule described in controller_indi_transition.m.
%   runtime_g_map = optional dense INDI surface-effectiveness table for
%            controller_id = 6.
%
% The dispatcher owns:
%   1. packing command/measurement vectors into the compact controller state,
%   2. selecting the active controller core by `controller_id`,
%   3. converting mixed surface commands into physical surface outputs.

%#codegen
coder.inline('never');

motor_rpm_cmd_out = local_vec(motor_rpm_cmd, 4);
tilt_angles_cmd_out = local_vec(tilt_angles_cmd, 2);
front_collective_rpm_out = local_scalar(front_collective_rpm);
rear_collective_rpm_out = local_scalar(rear_collective_rpm);

trim_cmd_use = local_vec(trim_cmd, 6);
x_ref_use = local_vec(x_ref, 9);
x_meas_use = local_pack_state(eul_meas, airData_meas, omega_meas);
actuator_state_meas_use = local_vec(actuator_state_meas, 10);
angular_accel_meas_use = local_vec(angular_accel_meas, 3);
x_cmd_track_use = local_build_longitudinal_tracking_ref(x_ref_use, airData_cmd, eul_cmd, omega_cmd);
mix_matrix_use = local_mix_matrix(mix_matrix);

K_lqr_use = zeros(6, 9);
if int32(controller_id) ~= 4 && int32(controller_id) ~= 5 && int32(controller_id) ~= 6
    K_lqr_use = local_gain(K_lqr, 6);
end

u_mixed = controller_hold_trim(trim_cmd_use);

switch int32(controller_id)
    case 0
        u_mixed = controller_hold_trim(trim_cmd_use);
    case 1
        u_mixed = controller_lqr_hold(x_meas_use, x_ref_use, trim_cmd_use, K_lqr_use(1:5, :));
    case 2
        u_mixed = controller_lqr_full_hold(x_meas_use, x_ref_use, trim_cmd_use, K_lqr_use);
    case 3
        u_mixed = controller_lqr_full_hold(x_meas_use, x_cmd_track_use, trim_cmd_use, K_lqr_use);
    case 4
        u_mixed = controller_lqr_path_schedule(airData_cmd, tilt_angles_cmd, x_meas_use, x_ref, trim_cmd, K_lqr);
    case 5
        [u_mixed, tilt_sched_deg] = controller_lqr_path_schedule_gated( ...
            airData_cmd, tilt_angles_cmd, x_meas_use, x_ref, trim_cmd, K_lqr);
        tilt_angles_cmd_out = [tilt_sched_deg; tilt_sched_deg];
    case 6
        [u_mixed, tilt_sched_deg] = controller_indi_transition( ...
            airData_cmd, tilt_angles_cmd, x_meas_use, accel_meas, ...
            actuator_state_meas_use, angular_accel_meas_use, ...
            x_ref, trim_cmd, K_lqr, runtime_g_map, surface_limit_rad, ...
            front_collective_min_rpm, front_collective_max_rpm, ...
            rear_collective_min_rpm, rear_collective_max_rpm);
        tilt_angles_cmd_out = [tilt_sched_deg; tilt_sched_deg];
    otherwise
        % Unknown IDs safely fall back to trim hold.
        u_mixed = controller_hold_trim(trim_cmd_use);
end

u_mixed = local_clip_mixed_outputs( ...
    u_mixed, surface_limit_rad, ...
    front_collective_min_rpm, front_collective_max_rpm, ...
    rear_collective_min_rpm, rear_collective_max_rpm);

front_collective_rpm_out = u_mixed(1);
rear_collective_rpm_out = u_mixed(2);

surface_physical = mix_matrix_use * u_mixed(3:6);
surface_physical = local_clip_surface_vector(surface_physical, surface_limit_rad);

deltaLW = surface_physical(1);
deltaRW = surface_physical(2);
deltaLT = surface_physical(3);
deltaRT = surface_physical(4);
end

function vec = local_vec(in, n)
vec = zeros(n, 1);
count = min(numel(in), n);
if count > 0
    vec(1:count) = in(1:count);
end
end

function value = local_scalar(in)
value = 0.0;
if ~isempty(in)
    value = in(1);
end
end

function gain = local_gain(in, nrows)
gain = zeros(nrows, 9);
if ndims(in) >= 3
    in_use = in(:, :, 1);
else
    in_use = in;
end

[nr, nc] = size(in_use);
if nr == 0 || nc == 0
    return;
end
gain(1:min(nr, nrows), 1:min(nc, 9)) = in_use(1:min(nr, nrows), 1:min(nc, 9));
end

function x = local_pack_state(eul, air_data, omega)
x = zeros(9, 1);
x(1:3) = local_vec(eul, 3);
x(4:6) = local_airdata_to_body_vel(air_data);
x(7:9) = local_vec(omega, 3);
end

function vel_body = local_airdata_to_body_vel(air_data)
air_data_use = local_vec(air_data, 3);
vinf = air_data_use(1);
alpha = air_data_use(2);
beta = air_data_use(3);

ca = cos(alpha);
sa = sin(alpha);
cb = cos(beta);
sb = sin(beta);

vel_body = zeros(3, 1);
vel_body(1) = vinf * ca * cb;
vel_body(2) = vinf * sb;
vel_body(3) = vinf * sa * cb;
end

function x_ref_track = local_build_longitudinal_tracking_ref(x_trim_ref, airData_cmd, eul_cmd, omega_cmd)
x_ref_track = x_trim_ref;

cmd_vel = local_airdata_to_body_vel(airData_cmd);
cmd_eul = local_vec(eul_cmd, 3);
cmd_rates = local_vec(omega_cmd, 3);

% Keep lateral-directional references anchored at the trim point and only
% replace the longitudinal channels needed for simple tracking.
x_ref_track(2) = cmd_eul(2);  % theta_cmd
x_ref_track(4) = cmd_vel(1);  % u_cmd from airspeed command
x_ref_track(6) = cmd_vel(3);  % w_cmd from alpha / Vinf command
x_ref_track(8) = cmd_rates(2); % q_cmd
end

function mix_matrix_use = local_mix_matrix(in)
mix_matrix_use = eye(4);
default_mix = [1 1 0 0; ...
               1 -1 0 0; ...
               0 0 1 -1; ...
               0 0 1 1];

if isempty(in)
    mix_matrix_use = default_mix;
    return;
end

[nr, nc] = size(in);
if nr >= 4 && nc >= 4
    mix_matrix_use = in(1:4, 1:4);
else
    mix_matrix_use = default_mix;
end
end

function u_cmd = local_clip_mixed_outputs(u_cmd, surface_limit_rad, ...
    front_collective_min_rpm, front_collective_max_rpm, ...
    rear_collective_min_rpm, rear_collective_max_rpm)
u_cmd(1) = min(max(u_cmd(1), front_collective_min_rpm), front_collective_max_rpm);
u_cmd(2) = min(max(u_cmd(2), rear_collective_min_rpm), rear_collective_max_rpm);
u_cmd(3) = min(max(u_cmd(3), -surface_limit_rad), surface_limit_rad);
u_cmd(4) = min(max(u_cmd(4), -surface_limit_rad), surface_limit_rad);
u_cmd(5) = min(max(u_cmd(5), -surface_limit_rad), surface_limit_rad);
u_cmd(6) = min(max(u_cmd(6), -surface_limit_rad), surface_limit_rad);
end

function vec = local_clip_surface_vector(vec, limit_abs)
for idx = 1:min(numel(vec), 4)
    vec(idx) = min(max(vec(idx), -limit_abs), limit_abs);
end
end
