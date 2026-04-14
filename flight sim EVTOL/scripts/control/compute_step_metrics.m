function metrics = compute_step_metrics(t, y, cmd)
%COMPUTE_STEP_METRICS Simple rise/settle/overshoot helper for reports.

t = t(:);
y = y(:);
cmd = cmd(:);

metrics = struct('initial', NaN, 'final', NaN, 'overshoot_pct', NaN, ...
    'rise_time_s', NaN, 'settling_time_s', NaN);

if isempty(t) || isempty(y) || isempty(cmd)
    return;
end

metrics.initial = y(1);
metrics.final = cmd(end);
stepMag = metrics.final - metrics.initial;
if abs(stepMag) < 1.0e-9
    return;
end

lo = metrics.initial + 0.10 * stepMag;
hi = metrics.initial + 0.90 * stepMag;
cross10 = find(sign(stepMag) * (y - lo) >= 0, 1, 'first');
cross90 = find(sign(stepMag) * (y - hi) >= 0, 1, 'first');
if ~isempty(cross10) && ~isempty(cross90) && cross90 >= cross10
    metrics.rise_time_s = t(cross90) - t(cross10);
end

if stepMag > 0
    peak = max(y);
else
    peak = min(y);
end
metrics.overshoot_pct = 100.0 * abs((peak - metrics.final) / stepMag);

tol = 0.02 * abs(stepMag);
settled = abs(y - metrics.final) <= tol;
idx = find(settled, 1, 'first');
if ~isempty(idx)
    tailAllSettled = find(arrayfun(@(k) all(settled(k:end)), 1:numel(t)), 1, 'first');
    if ~isempty(tailAllSettled)
        metrics.settling_time_s = t(tailAllSettled) - t(1);
    end
end
end
