function results = run_sensor_checkout(varargin)
%RUN_SENSOR_CHECKOUT Short trimmed-cruise sensor comparison run.

p = inputParser;
p.addParameter('StopTime', 2, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BiasAirspeedMps', 2.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BiasRollRateRadS', 0.02, @(x) isnumeric(x) && isscalar(x));
p.addParameter('RebuildWrapper', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
ensure_control_wrapper_model('RebuildWrapper', p.Results.RebuildWrapper);
setup_cruise_trim_case('AirspeedStep', 0, 'BankStepDeg', 0);

assignin('base', 'controller_enable', false);
assignin('base', 'controller_mode', 0);
assignin('base', 'sensor_enable_bias', false);
assignin('base', 'sensor_enable_noise', false);

simPerfect = sim('Brown_Control_Sim', 'StopTime', num2str(p.Results.StopTime));

assignin('base', 'sensor_enable_bias', true);
assignin('base', 'sensor_enable_noise', true);
assignin('base', 'sensor_airdata_bias', [p.Results.BiasAirspeedMps; 0; 0]);
assignin('base', 'sensor_gyro_bias', [p.Results.BiasRollRateRadS; 0; 0]);
simBiased = sim('Brown_Control_Sim', 'StopTime', num2str(p.Results.StopTime));

[t_truth, vinf_truth] = extract_logged_signal(simPerfect, 'vinf');
[~, alpha_truth] = extract_logged_signal(simPerfect, 'alpha');
[~, beta_truth] = extract_logged_signal(simPerfect, 'beta');
[~, omega_truth] = extract_logged_signal(simPerfect, 'omega');
[~, eul_truth] = extract_logged_signal(simPerfect, 'eul');
[t_meas, airspeed_meas] = extract_logged_signal(simPerfect, 'airspeed_meas');
[~, alpha_meas] = extract_logged_signal(simPerfect, 'alpha_meas');
[~, beta_meas] = extract_logged_signal(simPerfect, 'beta_meas');
[~, gyro_meas] = extract_logged_signal(simPerfect, 'gyro_meas');
[~, attitude_meas] = extract_logged_signal(simPerfect, 'attitude_meas');
[~, gps_pos_meas] = extract_logged_signal(simPerfect, 'gps_pos_meas');
[~, gps_vel_meas] = extract_logged_signal(simPerfect, 'gps_vel_meas');
[~, accel_meas] = extract_logged_signal(simPerfect, 'accel_meas');
[~, mag_meas] = extract_logged_signal(simPerfect, 'mag_body_meas');

[~, airspeed_meas_biased] = extract_logged_signal(simBiased, 'airspeed_meas');
[~, gyro_meas_biased] = extract_logged_signal(simBiased, 'gyro_meas');

results = struct();
results.t_truth = t_truth;
results.t_meas = t_meas;
results.truth = struct( ...
    'vinf', vinf_truth, ...
    'alpha', alpha_truth, ...
    'beta', beta_truth, ...
    'omega', omega_truth, ...
    'eul', eul_truth);
results.perfect = struct( ...
    'airspeed', airspeed_meas, ...
    'alpha', alpha_meas, ...
    'beta', beta_meas, ...
    'gyro', gyro_meas, ...
    'attitude', attitude_meas, ...
    'gps_pos', gps_pos_meas, ...
    'gps_vel', gps_vel_meas, ...
    'accel', accel_meas, ...
    'mag', mag_meas);
results.biased = struct( ...
    'airspeed', airspeed_meas_biased, ...
    'gyro', gyro_meas_biased);
end
