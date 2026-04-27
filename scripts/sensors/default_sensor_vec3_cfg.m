function cfg = default_sensor_vec3_cfg()
%DEFAULT_SENSOR_VEC3_CFG Default config for the 3-axis Sensor block.

cfg = struct();
cfg.Ts = 0.01;
cfg.seed = 201;
cfg.scale = eye(3);
cfg.bias = struct( ...
    'constant', zeros(3,1), ...
    'drift_sigma', zeros(3,1), ...
    'drift_tau', 60.0);
cfg.noise = struct( ...
    'white_sigma', zeros(3,1));
cfg.quant_lsb = zeros(3,1);
cfg.limits = repmat([-inf inf], 3, 1);
end
