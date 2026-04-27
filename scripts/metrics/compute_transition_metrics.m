function metrics = compute_transition_metrics(logData, opts)
%COMPUTE_TRANSITION_METRICS Compute hover<->cruise transition quality metrics.
%
% Required fields in logData:
%   t           [Nx1] time (s)
%   speed       [Nx1] scalar speed (m/s)
%   tilt_deg    [Nx1] nacelle tilt (deg)
% Optional fields:
%   accel_xyz   [Nx3] acceleration (m/s^2)
%   jerk_xyz    [Nx3] jerk (m/s^3)
%   control     [NxM] actuator command vector
%
% opts (optional):
%   target_speed (default: final speed sample)
%   settle_band_frac (default: 0.02)
%   settle_window_s (default: 2.0)

if nargin < 2
    opts = struct();
end
if ~isfield(opts, 'settle_band_frac'); opts.settle_band_frac = 0.02; end
if ~isfield(opts, 'settle_window_s');  opts.settle_window_s  = 2.0; end

required = {'t','speed','tilt_deg'};
for i = 1:numel(required)
    if ~isfield(logData, required{i})
        error('Missing required field: %s', required{i});
    end
end

t = logData.t(:);
speed = logData.speed(:);
tilt = logData.tilt_deg(:);

if ~isfield(opts, 'target_speed')
    targetSpeed = speed(end);
else
    targetSpeed = opts.target_speed;
end

if isfield(logData, 'accel_xyz')
    accel = logData.accel_xyz;
else
    accel = zeros(numel(t), 3);
end

if isfield(logData, 'jerk_xyz')
    jerk = logData.jerk_xyz;
else
    dt = diff(t);
    dacc = diff(accel, 1, 1);
    jerk = [dacc ./ max(dt, eps), zeros(size(dacc,1), 0)]; %#ok<AGROW>
    jerk = [jerk; jerk(end,:)];
end

if isfield(logData, 'control')
    u = logData.control;
else
    u = zeros(numel(t), 1);
end

% Energy proxy from actuator usage integral.
energyProxy = trapz(t, sum(abs(u), 2));

% Smoothness proxies.
accNorm = sqrt(sum(accel.^2, 2));
jerkNorm = sqrt(sum(jerk.^2, 2));
accRMS = sqrt(mean(accNorm.^2));
jerkRMS = sqrt(mean(jerkNorm.^2));

if size(u,1) > 1
    dt = diff(t);
    du = diff(u, 1, 1);
    uRate = du ./ max(dt, eps);
    ctrlSmooth = trapz(t(1:end-1), sum(uRate.^2, 2));
else
    ctrlSmooth = 0;
end

% Transition start/end heuristics.
startIdx = find(tilt > (min(tilt) + 0.05*(max(tilt)-min(tilt))), 1, 'first');
if isempty(startIdx), startIdx = 1; end

band = opts.settle_band_frac * max(abs(targetSpeed), eps);
settleMask = abs(speed - targetSpeed) <= band;

windowSamps = max(1, round(opts.settle_window_s / max(mean(diff(t)), eps)));
endIdx = NaN;
for k = startIdx:numel(t)-windowSamps+1
    if all(settleMask(k:k+windowSamps-1))
        endIdx = k;
        break;
    end
end

if isnan(endIdx)
    transitionTime = NaN;
    settlingTime = NaN;
else
    transitionTime = t(endIdx) - t(startIdx);
    settlingTime = t(endIdx) - t(startIdx);
end

overshoot = max(0, max(speed(startIdx:end)) - targetSpeed);

metrics = struct();
metrics.energy_proxy = energyProxy;
metrics.accel_rms = accRMS;
metrics.jerk_rms = jerkRMS;
metrics.control_smoothness = ctrlSmooth;
metrics.transition_start_s = t(startIdx);
metrics.transition_time_s = transitionTime;
metrics.overshoot_mps = overshoot;
metrics.settling_time_s = settlingTime;
metrics.target_speed_mps = targetSpeed;

end
