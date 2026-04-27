function [posMeas, velMeas] = sensor_gps_block_step(t, posTruth, velTruth, cfg)
%SENSOR_GPS_BLOCK_STEP Discrete GPS-like sensor model for Simulink blocks.

persistent lastUpdate posHold velHold posDrift velDrift sampleIdx

cfg = localFillDefaults(cfg);
posTruth = posTruth(:);
velTruth = velTruth(:);

if isempty(lastUpdate) || t <= 0
    lastUpdate = t;
    posHold = posTruth;
    velHold = velTruth;
    posDrift = zeros(3,1);
    velDrift = zeros(3,1);
    sampleIdx = 0;
end

if (t - lastUpdate) + 1.0e-12 < cfg.Ts
    posMeas = posHold;
    velMeas = velHold;
    return;
end

sampleIdx = sampleIdx + 1;
lastUpdate = t;

for idx = 1:3
    posDrift(idx) = localUpdateDrift(posDrift(idx), cfg.pos.drift_sigma(idx), cfg.pos.drift_tau, cfg.Ts, sampleIdx, cfg.seed + idx);
    velDrift(idx) = localUpdateDrift(velDrift(idx), cfg.vel.drift_sigma(idx), cfg.vel.drift_tau, cfg.Ts, sampleIdx, cfg.seed + 20 + idx);
end

posNoise = zeros(3,1);
velNoise = zeros(3,1);
for idx = 1:3
    posNoise(idx) = cfg.pos.white_sigma(idx) * localPseudoNoise(sampleIdx, cfg.seed + 40 + idx);
    velNoise(idx) = cfg.vel.white_sigma(idx) * localPseudoNoise(sampleIdx, cfg.seed + 60 + idx);
end

posMeas = posTruth + cfg.pos.bias(:) + posDrift + posNoise;
velMeas = velTruth + cfg.vel.bias(:) + velDrift + velNoise;

posHold = posMeas;
velHold = velMeas;
end

function cfg = localFillDefaults(cfg)
base = default_sensor_gps_cfg();
cfg = localMergeStruct(base, cfg);
cfg.pos.bias = cfg.pos.bias(:);
cfg.pos.drift_sigma = cfg.pos.drift_sigma(:);
cfg.pos.white_sigma = cfg.pos.white_sigma(:);
cfg.vel.bias = cfg.vel.bias(:);
cfg.vel.drift_sigma = cfg.vel.drift_sigma(:);
cfg.vel.white_sigma = cfg.vel.white_sigma(:);
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
