function cfg = default_sensor_gps_cfg()
%DEFAULT_SENSOR_GPS_CFG Default config for the GPS Sensor block.

cfg = struct();
cfg.pos_sensor = default_sensor_vec3_cfg();
cfg.pos_sensor.Ts = 0.20;
cfg.pos_sensor.seed = 301;
cfg.pos_sensor.noise.white_sigma = [0.75; 0.75; 1.00];
cfg.pos_sensor.bias.drift_tau = 30.0;

cfg.vel_sensor = default_sensor_vec3_cfg();
cfg.vel_sensor.Ts = 0.20;
cfg.vel_sensor.seed = 401;
cfg.vel_sensor.noise.white_sigma = [0.05; 0.05; 0.08];
cfg.vel_sensor.bias.drift_tau = 15.0;
end
