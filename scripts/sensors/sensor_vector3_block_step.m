function meas = sensor_vector3_block_step(t, truth, cfg)
%SENSOR_VECTOR3_BLOCK_STEP Discrete 3-axis sensor model for Simulink blocks.

persistent seeds lastUpdate measHold driftState sampleIdx

cfg = localFillDefaults(cfg);
truth = truth(:);
sensorSeed = cfg.seed;
[sensorIdx, seeds, lastUpdate, measHold, driftState, sampleIdx] = ...
    localSelectState(sensorSeed, truth, t, seeds, lastUpdate, measHold, driftState, sampleIdx);

if (t - lastUpdate(sensorIdx)) + 1.0e-12 < cfg.Ts
    meas = measHold(:,sensorIdx);
    return;
end

sampleIdx(sensorIdx) = sampleIdx(sensorIdx) + 1;
lastUpdate(sensorIdx) = t;

for idx = 1:3
    driftState(idx,sensorIdx) = localUpdateDrift( ...
        driftState(idx,sensorIdx), cfg.bias.drift_sigma(idx), cfg.bias.drift_tau, ...
        cfg.Ts, sampleIdx(sensorIdx), cfg.seed + 10 + idx);
end

whiteNoise = zeros(3,1);
for idx = 1:3
    whiteNoise(idx) = cfg.noise.white_sigma(idx) * localPseudoNoise(sampleIdx(sensorIdx), cfg.seed + idx);
end

meas = cfg.scale * truth + cfg.bias.constant(:) + driftState(:,sensorIdx) + whiteNoise;

for idx = 1:3
    if cfg.quant_lsb(idx) > 0
        meas(idx) = cfg.quant_lsb(idx) * round(meas(idx) / cfg.quant_lsb(idx));
    end
    if size(cfg.limits,1) >= idx
        meas(idx) = min(max(meas(idx), cfg.limits(idx,1)), cfg.limits(idx,2));
    end
end

measHold(:,sensorIdx) = meas;
end

function cfg = localFillDefaults(cfg)
base = default_sensor_vec3_cfg();
cfg = localMergeStruct(base, cfg);
cfg.bias.constant = cfg.bias.constant(:);
cfg.bias.drift_sigma = cfg.bias.drift_sigma(:);
cfg.noise.white_sigma = cfg.noise.white_sigma(:);
cfg.quant_lsb = cfg.quant_lsb(:);
end

function driftState = localUpdateDrift(driftState, driftSigma, driftTau, Ts, sampleIdx, seed)
if driftSigma <= 0
    driftState = 0.0;
    return;
end

a = exp(-Ts / max(driftTau, 1.0e-6));
eta = localPseudoNoise(sampleIdx, seed);
driftState = a * driftState + driftSigma * sqrt(max(1 - a^2, 0.0)) * eta;
end

function y = localPseudoNoise(k, seed)
y = 0.65 * sin((0.17 + 0.013 * seed) * k + 0.41 * seed) + ...
    0.35 * sin((0.53 + 0.021 * seed) * k + 0.77 * seed);
end

function out = localMergeStruct(base, in)
out = base;
if isempty(in)
    return;
end

fields = fieldnames(in);
for idx = 1:numel(fields)
    name = fields{idx};
    value = in.(name);
    if isstruct(value) && isfield(out, name) && isstruct(out.(name))
        out.(name) = localMergeStruct(out.(name), value);
    else
        out.(name) = value;
    end
end
end

function [sensorIdx, seeds, lastUpdate, measHold, driftState, sampleIdx] = ...
    localSelectState(sensorSeed, truth, t, seeds, lastUpdate, measHold, driftState, sampleIdx)
if isempty(seeds)
    seeds = sensorSeed;
    sensorIdx = 1;
    lastUpdate = t;
    measHold = truth;
    driftState = zeros(3,1);
    sampleIdx = 0;
    return;
end

sensorIdx = find(seeds == sensorSeed, 1, 'first');
if isempty(sensorIdx)
    seeds(end+1) = sensorSeed; %#ok<AGROW>
    sensorIdx = numel(seeds);
    lastUpdate(sensorIdx,1) = t;
    measHold(:,sensorIdx) = truth;
    driftState(:,sensorIdx) = zeros(3,1);
    sampleIdx(sensorIdx,1) = 0;
elseif t <= 0
    lastUpdate(sensorIdx) = t;
    measHold(:,sensorIdx) = truth;
    driftState(:,sensorIdx) = zeros(3,1);
    sampleIdx(sensorIdx) = 0;
end
end
