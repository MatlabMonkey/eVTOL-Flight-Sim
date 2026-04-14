function meas = sensor_scalar_block_step(t, truth, cfg)
%SENSOR_SCALAR_BLOCK_STEP Discrete scalar sensor model for Simulink blocks.

persistent seeds lastUpdate measHold driftState sampleIdx

cfg = localFillDefaults(cfg);
sensorSeed = cfg.seed;
[sensorIdx, seeds, lastUpdate, measHold, driftState, sampleIdx] = ...
    localSelectState(sensorSeed, truth, t, seeds, lastUpdate, measHold, driftState, sampleIdx);

if (t - lastUpdate(sensorIdx)) + 1.0e-12 < cfg.Ts
    meas = measHold(sensorIdx);
    return;
end

sampleIdx(sensorIdx) = sampleIdx(sensorIdx) + 1;
lastUpdate(sensorIdx) = t;

driftState(sensorIdx) = localUpdateDrift( ...
    driftState(sensorIdx), cfg.bias.drift_sigma, cfg.bias.drift_tau, ...
    cfg.Ts, sampleIdx(sensorIdx), cfg.seed + 11);
whiteNoise = cfg.noise.white_sigma * localPseudoNoise(sampleIdx(sensorIdx), cfg.seed + 1);

meas = cfg.scale * truth + cfg.bias.constant + driftState(sensorIdx) + whiteNoise;

if cfg.quant_lsb > 0
    meas = cfg.quant_lsb * round(meas / cfg.quant_lsb);
end

if numel(cfg.limits) == 2
    meas = min(max(meas, cfg.limits(1)), cfg.limits(2));
end

measHold(sensorIdx) = meas;
end

function cfg = localFillDefaults(cfg)
base = default_sensor_scalar_cfg();
cfg = localMergeStruct(base, cfg);
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
    driftState = 0.0;
    sampleIdx = 0;
    return;
end

sensorIdx = find(seeds == sensorSeed, 1, 'first');
if isempty(sensorIdx)
    seeds(end+1) = sensorSeed; %#ok<AGROW>
    sensorIdx = numel(seeds);
    lastUpdate(sensorIdx,1) = t;
    measHold(sensorIdx,1) = truth;
    driftState(sensorIdx,1) = 0.0;
    sampleIdx(sensorIdx,1) = 0;
elseif t <= 0
    lastUpdate(sensorIdx) = t;
    measHold(sensorIdx) = truth;
    driftState(sensorIdx) = 0.0;
    sampleIdx(sensorIdx) = 0;
end
end
