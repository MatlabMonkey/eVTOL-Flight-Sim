% scripts/metrics/example_metrics_run.m
% Generates synthetic transition data and computes metric table.

fprintf('== example_metrics_run ==\n');
fprintf('Timestamp: %s\n', datestr(now, 31));

% Synthetic profile: hover -> cruise transition over ~20 s.
dt = 0.05;
t = (0:dt:30)';
N = numel(t);

sig = @(x) 1 ./ (1 + exp(-x));
transitionCore = sig((t-10)/1.5);

tilt_deg = 90 * transitionCore;
speed = 55 * transitionCore + 2.5 * exp(-0.15*(t-12).^2); % slight overshoot bump

accel_x = [diff(speed)./dt; 0];
accel_xyz = [accel_x, zeros(N,1), 0.05*sin(0.8*t)];
jerk_xyz = [[diff(accel_x)./dt; 0], zeros(N,1), [diff(accel_xyz(:,3))./dt; 0]];

% 12 pseudo actuator channels (normalized command)
u_front = repmat(0.3 + 0.5*transitionCore, 1, 6);
u_rear  = repmat(0.7 - 0.4*transitionCore, 1, 6);
control = [u_front, u_rear];

logData = struct();
logData.t = t;
logData.speed = speed;
logData.tilt_deg = tilt_deg;
logData.accel_xyz = accel_xyz;
logData.jerk_xyz = jerk_xyz;
logData.control = control;

metrics = compute_transition_metrics(logData, struct('target_speed', 55));

metricNames = fieldnames(metrics);
metricVals = struct2cell(metrics);
T = table(metricNames, metricVals, 'VariableNames', {'Metric', 'Value'});

disp(T);

outDir = fullfile('docs', 'evidence');
if ~exist(outDir, 'dir'); mkdir(outDir); end

save(fullfile(outDir, 'metrics_example_latest.mat'), 'metrics', 'T');

try
    writetable(T, fullfile(outDir, 'metrics_example_latest.csv'));
catch
    % If writetable unavailable, skip CSV export.
end

fprintf('METRICS_EXAMPLE: PASS\n');
