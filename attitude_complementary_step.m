function eul_hat = attitude_complementary_step(t, enable, gyro_meas, ...
    accel_meas, mag_body_meas, attitude_meas, init_eul, k_acc, k_mag, k_att)
%ATTITUDE_COMPLEMENTARY_STEP Lightweight gyro/accel/mag complementary filter.
%   This returns [phi; theta; psi] in radians and uses persistent state so it
%   can be called from a MATLAB Function block.

persistent eul_state last_t

if isempty(eul_state) || t <= 0 || enable == 0
    eul_state = init_eul(:);
    last_t = t;
    eul_hat = eul_state;
    return;
end

dt = max(min(t - last_t, 0.05), 0.0);
last_t = t;

if dt <= 0
    eul_hat = eul_state;
    return;
end

phi = eul_state(1);
theta = eul_state(2);
psi = eul_state(3);

ax = accel_meas(1);
ay = accel_meas(2);
az = accel_meas(3);

% z is positive downward in this model, so steady level flight produces
% approximately [0; 0; -g] specific force.
phi_acc = atan2(-ay, -az);
theta_acc = atan2(ax, sqrt(ay * ay + az * az));

mx = mag_body_meas(1);
my = mag_body_meas(2);
mz = mag_body_meas(3);

mx_h = mx * cos(theta) + ...
    my * sin(phi) * sin(theta) + ...
    mz * cos(phi) * sin(theta);
my_h = my * cos(phi) - mz * sin(phi);
psi_mag = atan2(-my_h, mx_h);

p = gyro_meas(1);
q = gyro_meas(2);
r = gyro_meas(3);

cosTheta = cos(theta);
if abs(cosTheta) < 1.0e-3
    cosTheta = sign(cosTheta) * 1.0e-3;
end

phi_dot = p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
theta_dot = cos(phi) * q - sin(phi) * r;
psi_dot = (sin(phi) / cosTheta) * q + (cos(phi) / cosTheta) * r;

phi = wrap_pi(phi + dt * (phi_dot + ...
    k_acc * wrap_pi(phi_acc - phi) + ...
    k_att * wrap_pi(attitude_meas(1) - phi)));
theta = wrap_pi(theta + dt * (theta_dot + ...
    k_acc * wrap_pi(theta_acc - theta) + ...
    k_att * wrap_pi(attitude_meas(2) - theta)));
psi = wrap_pi(psi + dt * (psi_dot + ...
    k_mag * wrap_pi(psi_mag - psi) + ...
    k_att * wrap_pi(attitude_meas(3) - psi)));

eul_state = [phi; theta; psi];
eul_hat = eul_state;
end

function a = wrap_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end
