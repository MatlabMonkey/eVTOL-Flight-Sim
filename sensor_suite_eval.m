function [airspeed_meas, alpha_meas, beta_meas, gyro_meas, accel_meas, ...
    gps_pos_meas, gps_vel_meas, mag_body_meas, attitude_meas] = ...
    sensor_suite_eval(vinf_truth, alpha_truth, beta_truth, omega_truth, ...
    specific_force_truth, pos_truth, vel_truth, eul_truth, C_NB, ...
    enable_bias, enable_noise, airdata_bias, gyro_bias, accel_bias, ...
    gps_pos_bias, gps_vel_bias, mag_bias, attitude_bias, airdata_sigma, ...
    gyro_sigma, accel_sigma, gps_pos_sigma, gps_vel_sigma, mag_sigma, ...
    attitude_sigma, mag_ned)
%SENSOR_SUITE_EVAL Deterministic first-pass sensor model for the eVTOL.
%   The v1 implementation is intentionally simple: perfect sensors by
%   default, with optional additive bias and repeatable pseudo-noise
%   enabled by startup flags.

if nargin < 26
    error('sensor_suite_eval:NotEnoughInputs', ...
        'Expected truth signals, bias/noise flags, bias vectors, sigma vectors, and mag_ned.');
end

airspeed_meas = vinf_truth;
alpha_meas = alpha_truth;
beta_meas = beta_truth;
gyro_meas = omega_truth;
accel_meas = specific_force_truth;
gps_pos_meas = pos_truth;
gps_vel_meas = vel_truth;
attitude_meas = eul_truth;

% C_NB is the body-to-NED DCM in this model, so transpose it to map the
% nominal Earth magnetic-field vector into body axes.
mag_body_meas = C_NB' * mag_ned;

if enable_noise ~= 0
    airspeed_meas = airspeed_meas + localPseudoNoiseScalar(1, airdata_sigma(1));
    alpha_meas = alpha_meas + localPseudoNoiseScalar(2, airdata_sigma(2));
    beta_meas = beta_meas + localPseudoNoiseScalar(3, airdata_sigma(3));
    gyro_meas = gyro_meas + localPseudoNoiseVector(10, gyro_sigma(:));
    accel_meas = accel_meas + localPseudoNoiseVector(20, accel_sigma(:));
    gps_pos_meas = gps_pos_meas + localPseudoNoiseVector(30, gps_pos_sigma(:));
    gps_vel_meas = gps_vel_meas + localPseudoNoiseVector(40, gps_vel_sigma(:));
    mag_body_meas = mag_body_meas + localPseudoNoiseVector(50, mag_sigma(:));
    attitude_meas = attitude_meas + localPseudoNoiseVector(60, attitude_sigma(:));
end

if enable_bias ~= 0
    airspeed_meas = airspeed_meas + airdata_bias(1);
    alpha_meas = alpha_meas + airdata_bias(2);
    beta_meas = beta_meas + airdata_bias(3);
    gyro_meas = gyro_meas + gyro_bias;
    accel_meas = accel_meas + accel_bias;
    gps_pos_meas = gps_pos_meas + gps_pos_bias;
    gps_vel_meas = gps_vel_meas + gps_vel_bias;
    mag_body_meas = mag_body_meas + mag_bias;
    attitude_meas = attitude_meas + attitude_bias;
end
end

function n = localPseudoNoiseScalar(channelId, sigma)
n = sigma * localPseudoCore(channelId);
end

function n = localPseudoNoiseVector(baseChannelId, sigmaVec)
n = zeros(size(sigmaVec));
for idx = 1:numel(sigmaVec)
    n(idx) = sigmaVec(idx) * localPseudoCore(baseChannelId + idx - 1);
end
end

function y = localPseudoCore(channelId)
persistent k
if isempty(k)
    k = 0;
end
k = k + 1;

w1 = 0.17 + 0.013 * channelId;
w2 = 0.53 + 0.021 * channelId;
phi1 = 0.41 * channelId;
phi2 = 0.77 * channelId;
y = 0.65 * sin(w1 * k + phi1) + 0.35 * sin(w2 * k + phi2);
end
