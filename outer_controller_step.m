function [motor_rpms, delta_f, delta_a, delta_e, delta_r] = ...
    outer_controller_step(t, enable, mode, airspeed_meas, gyro_meas, ...
    eul_hat, airspeed_cmd, bank_cmd, trim_motor_rpms, trim_delta_f, ...
    trim_delta_a, trim_delta_e, trim_delta_r, k_p_damp, k_q_damp, ...
    k_r_damp, k_phi_p, k_phi_i, k_v_p, k_v_i, front_motor_mask, ...
    surface_limit_rad, rpm_delta_limit, integrator_limit)
%OUTER_CONTROLLER_STEP Sequential damper/PD/PID controller.
%   mode = 0 => open-loop trim commands
%   mode = 1 => p/q/r dampers only
%   mode = 2 => dampers + proportional bank/airspeed loops
%   mode = 3 => dampers + proportional + integral bank/airspeed loops

persistent int_phi int_v last_t

if isempty(int_phi) || t <= 0 || enable == 0
    int_phi = 0.0;
    int_v = 0.0;
    last_t = t;
end

dt = max(min(t - last_t, 0.05), 0.0);
last_t = t;

motor_rpms = trim_motor_rpms(:);
delta_f = trim_delta_f;
delta_a = trim_delta_a;
delta_e = trim_delta_e;
delta_r = trim_delta_r;

if enable == 0 || mode == 0
    return;
end

p = gyro_meas(1);
q = gyro_meas(2);
r = gyro_meas(3);
phi_hat = eul_hat(1);

e_phi = bank_cmd - phi_hat;
e_v = airspeed_cmd - airspeed_meas;

if mode >= 3 && dt > 0
    int_phi = clamp(int_phi + e_phi * dt, -integrator_limit, integrator_limit);
    int_v = clamp(int_v + e_v * dt, -integrator_limit, integrator_limit);
else
    int_phi = 0.0;
    int_v = 0.0;
end

roll_damper = -k_p_damp * p;
pitch_damper = -k_q_damp * q;
yaw_damper = -k_r_damp * r;

bank_outer = 0.0;
airspeed_outer = 0.0;
if mode >= 2
    bank_outer = k_phi_p * e_phi;
    airspeed_outer = k_v_p * e_v;
end
if mode >= 3
    bank_outer = bank_outer + k_phi_i * int_phi;
    airspeed_outer = airspeed_outer + k_v_i * int_v;
end

delta_a = clamp(trim_delta_a + roll_damper + bank_outer, ...
    -surface_limit_rad, surface_limit_rad);
delta_e = clamp(trim_delta_e + pitch_damper, ...
    -surface_limit_rad, surface_limit_rad);
delta_r = clamp(trim_delta_r + yaw_damper, ...
    -surface_limit_rad, surface_limit_rad);
delta_f = clamp(trim_delta_f, -surface_limit_rad, surface_limit_rad);

front_delta = clamp(airspeed_outer, -rpm_delta_limit, rpm_delta_limit);
motor_rpms = trim_motor_rpms(:) + front_motor_mask(:) * front_delta;
motor_rpms = max(motor_rpms, 0.0);
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end
