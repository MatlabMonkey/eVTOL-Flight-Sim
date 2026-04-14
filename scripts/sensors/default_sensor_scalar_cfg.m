function cfg = default_sensor_scalar_cfg()
%DEFAULT_SENSOR_SCALAR_CFG Default config for the Scalar Sensor block.

cfg = struct();
cfg.Ts = 0.02;
cfg.seed = 101;
cfg.scale = 1.0;
cfg.bias = struct( ...
    'constant', 0.0, ...
    'drift_sigma', 0.0, ...
    'drift_tau', 60.0);
cfg.noise = struct( ...
    'white_sigma', 0.0);
cfg.quant_lsb = 0.0;
cfg.limits = [-inf inf];
end
