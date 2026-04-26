function [u_cmd, tilt_cmd_deg, scheduler_debug] = controller_lqr_path_schedule_gated( ...
    airData_cmd, tilt_angles_cmd, x_meas, x_ref_schedule, trim_cmd_schedule, K_schedule)
%CONTROLLER_LQR_PATH_SCHEDULE_GATED
% State-gated scheduled LQR:
%   1) follow the ordered path points in sequence,
%   2) do not advance to the next segment until the current point is settled,
%   3) when advancing, ramp smoothly within the next segment.
%
% The external guide command still acts as an upper bound through the
% projected path progress, but it no longer forces advancement.

%#codegen

nPts = size(trim_cmd_schedule, 2);
u_cmd = controller_hold_trim(zeros(6, 1));
tilt_cmd_deg = 0.0;
scheduler_debug = zeros(4, 1); % [progress; idxLo; idxHi; lambda]

if nPts <= 0
    return;
end

if nPts == 1
    trim_cmd = localTrimColumn(trim_cmd_schedule, 1);
    K_lqr = localGainPage(K_schedule, 1);
    x_ref = localStateColumn(x_ref_schedule, 1);
    u_cmd = controller_lqr_full_hold(x_meas, x_ref, trim_cmd, K_lqr);
    tilt_cmd_deg = localScheduleValue(x_ref_schedule, 10, 1);
    scheduler_debug = [0.0; 1.0; 1.0; 0.0];
    return;
end

tilt_sched = localScheduleRow(x_ref_schedule, 10, nPts);
vinf_sched = localScheduleRow(x_ref_schedule, 11, nPts);
progress_sched = localScheduleRow(x_ref_schedule, 12, nPts);
if localAllZero(progress_sched)
    progress_sched = localUnitProgress(nPts);
end

settle_theta_deg = localScalarWithDefault(x_ref_schedule, 13, 4.0);
settle_u_mps = localScalarWithDefault(x_ref_schedule, 14, 2.0);
settle_w_mps = localScalarWithDefault(x_ref_schedule, 15, 1.5);
settle_q_deg_s = localScalarWithDefault(x_ref_schedule, 16, 4.0);
settle_time_s = localScalarWithDefault(x_ref_schedule, 17, 0.5);
segment_ramp_time_s = localScalarWithDefault(x_ref_schedule, 18, 4.0);

cmd_tilt_deg = 0.0;
if ~isempty(tilt_angles_cmd)
    cmd_tilt_deg = tilt_angles_cmd(1);
end
cmd_vinf_mps = 0.0;
if ~isempty(airData_cmd)
    cmd_vinf_mps = airData_cmd(1);
end
desired_progress = localProjectToScheduledPath(cmd_tilt_deg, cmd_vinf_mps, tilt_sched, vinf_sched, progress_sched);

persistent initialized path_signature current_idx segment_lambda settled_counter in_transition
if isempty(initialized)
    initialized = false;
end
if isempty(path_signature)
    path_signature = 0.0;
end
if isempty(current_idx)
    current_idx = int32(1);
end
if isempty(segment_lambda)
    segment_lambda = 0.0;
end
if isempty(settled_counter)
    settled_counter = int32(0);
end
if isempty(in_transition)
    in_transition = false;
end

signature_now = localScheduleSignature(tilt_sched, vinf_sched, progress_sched);
if ~initialized || path_signature ~= signature_now
    initialized = true;
    path_signature = signature_now;
    current_idx = int32(1);
    segment_lambda = 0.0;
    settled_counter = int32(0);
    in_transition = false;
end

current_idx = min(max(current_idx, int32(1)), int32(nPts));

if current_idx >= nPts
    idxLo = nPts;
    idxHi = nPts;
    lambda = 0.0;
else
    idxLo = double(current_idx);
    idxHi = idxLo;
    lambda = 0.0;

    if localIsSettledAtPoint(x_meas, x_ref_schedule, idxLo, ...
            settle_theta_deg, settle_u_mps, settle_w_mps, settle_q_deg_s)
        settled_counter = settled_counter + 1;
    else
        settled_counter = int32(0);
    end

    next_progress = progress_sched(idxLo + 1);
    if ~in_transition && ...
            settled_counter >= localSettleSampleCount(settle_time_s) && ...
            desired_progress >= next_progress - 1e-9
        in_transition = true;
    end

    if in_transition
        desired_lambda_bound = localDesiredLambdaBound(desired_progress, progress_sched, idxLo);
        lambda_step = localLambdaStep(segment_ramp_time_s);
        segment_lambda = min(segment_lambda + lambda_step, max(desired_lambda_bound, segment_lambda));

        idxHi = idxLo + 1;
        lambda = segment_lambda;

        if segment_lambda >= 1.0 - 1e-9
            current_idx = int32(idxLo + 1);
            segment_lambda = 0.0;
            settled_counter = int32(0);
            in_transition = false;
            idxLo = double(current_idx);
            idxHi = idxLo;
            lambda = 0.0;
        end
    end
end

x_ref = localBlendState(x_ref_schedule, idxLo, idxHi, lambda);
trim_cmd = localBlendTrim(trim_cmd_schedule, idxLo, idxHi, lambda);
K_lqr = localBlendGain(K_schedule, idxLo, idxHi, lambda);
tilt_cmd_deg = localBlendScalar(tilt_sched, idxLo, idxHi, lambda);

u_cmd = controller_lqr_full_hold(x_meas, x_ref, trim_cmd, K_lqr);
scheduler_debug = [localBlendScalar(progress_sched, idxLo, idxHi, lambda); double(idxLo); double(idxHi); lambda];
end

function tf = localAllZero(vec)
tf = true;
for i = 1:numel(vec)
    if abs(vec(i)) > 1e-12
        tf = false;
        return;
    end
end
end

function vec = localUnitProgress(nPts)
vec = zeros(1, nPts);
if nPts <= 1
    return;
end
for i = 1:nPts
    vec(i) = double(i - 1) / double(nPts - 1);
end
end

function row = localScheduleRow(x_ref_schedule, rowIdx, nPts)
row = zeros(1, nPts);
if size(x_ref_schedule, 1) >= rowIdx
    row(1, :) = x_ref_schedule(rowIdx, 1:nPts);
end
end

function value = localScheduleValue(x_ref_schedule, rowIdx, idx)
value = 0.0;
if size(x_ref_schedule, 1) >= rowIdx && size(x_ref_schedule, 2) >= idx
    value = x_ref_schedule(rowIdx, idx);
end
end

function x_ref = localStateColumn(x_ref_schedule, idx)
x_ref = zeros(9, 1);
rows = min(size(x_ref_schedule, 1), 9);
x_ref(1:rows) = x_ref_schedule(1:rows, idx);
end

function trim_cmd = localTrimColumn(trim_cmd_schedule, idx)
trim_cmd = zeros(6, 1);
rows = min(size(trim_cmd_schedule, 1), 6);
trim_cmd(1:rows) = trim_cmd_schedule(1:rows, idx);
end

function K_lqr = localGainPage(K_schedule, idx)
K_lqr = zeros(6, 9);
sz = size(K_schedule);
if numel(sz) < 3
    rows = min(sz(1), 6);
    cols = min(sz(2), 9);
    K_lqr(1:rows, 1:cols) = K_schedule(1:rows, 1:cols);
    return;
end

rows = min(sz(1), 6);
cols = min(sz(2), 9);
K_lqr(1:rows, 1:cols) = K_schedule(1:rows, 1:cols, idx);
end

function progress_cmd = localProjectToScheduledPath(cmd_tilt_deg, cmd_vinf_mps, tilt_sched, vinf_sched, progress_sched)
nPts = numel(progress_sched);
point = [cmd_tilt_deg, cmd_vinf_mps];

bestDist = inf;
bestProgress = progress_sched(1);

for i = 1:(nPts - 1)
    p0 = [tilt_sched(i), vinf_sched(i)];
    p1 = [tilt_sched(i + 1), vinf_sched(i + 1)];
    seg = p1 - p0;
    segNormSq = seg(1) * seg(1) + seg(2) * seg(2);
    if segNormSq <= eps
        continue;
    end

    rel = point - p0;
    tau = (rel(1) * seg(1) + rel(2) * seg(2)) / segNormSq;
    tau = min(max(tau, 0.0), 1.0);
    proj = p0 + tau * seg;
    d = sqrt((point(1) - proj(1))^2 + (point(2) - proj(2))^2);
    if d < bestDist
        bestDist = d;
        bestProgress = (1.0 - tau) * progress_sched(i) + tau * progress_sched(i + 1);
    end
end

progress_cmd = bestProgress;
end

function lambda = localDesiredLambdaBound(desired_progress, progress_sched, idxLo)
segStart = progress_sched(idxLo);
segEnd = progress_sched(idxLo + 1);
if segEnd <= segStart
    lambda = 1.0;
    return;
end
desired_progress = min(max(desired_progress, segStart), segEnd);
lambda = (desired_progress - segStart) / (segEnd - segStart);
lambda = min(max(lambda, 0.0), 1.0);
end

function tf = localIsSettledAtPoint(x_meas, x_ref_schedule, idx, ...
    settle_theta_deg, settle_u_mps, settle_w_mps, settle_q_deg_s)
x_ref = localStateColumn(x_ref_schedule, idx);
theta_err_deg = abs(rad2deg(x_meas(2) - x_ref(2)));
u_err = abs(x_meas(4) - x_ref(4));
w_err = abs(x_meas(6) - x_ref(6));
q_err_deg_s = abs(rad2deg(x_meas(8) - x_ref(8)));

tf = theta_err_deg <= settle_theta_deg && ...
     u_err <= settle_u_mps && ...
     w_err <= settle_w_mps && ...
     q_err_deg_s <= settle_q_deg_s;
end

function n = localSettleSampleCount(settle_time_s)
n = int32(max(1.0, round(settle_time_s / 0.01)));
end

function step = localLambdaStep(segment_ramp_time_s)
step = 0.01 / max(segment_ramp_time_s, 0.25);
end

function sig = localScheduleSignature(tilt_sched, vinf_sched, progress_sched)
sig = sum(tilt_sched) + 3.7 * sum(vinf_sched) + 11.0 * sum(progress_sched) + 101.0 * numel(progress_sched);
end

function value = localScalarWithDefault(x_ref_schedule, rowIdx, fallback)
value = fallback;
if size(x_ref_schedule, 1) >= rowIdx && size(x_ref_schedule, 2) >= 1
    candidate = x_ref_schedule(rowIdx, 1);
    if isfinite(candidate) && abs(candidate) > 0
        value = candidate;
    end
end
end

function x_ref = localBlendState(x_ref_schedule, idxLo, idxHi, lambda)
x_ref = zeros(9, 1);
x0 = localStateColumn(x_ref_schedule, idxLo);
x1 = localStateColumn(x_ref_schedule, idxHi);
for i = 1:9
    x_ref(i) = (1.0 - lambda) * x0(i) + lambda * x1(i);
end
end

function trim_cmd = localBlendTrim(trim_cmd_schedule, idxLo, idxHi, lambda)
trim_cmd = zeros(6, 1);
u0 = localTrimColumn(trim_cmd_schedule, idxLo);
u1 = localTrimColumn(trim_cmd_schedule, idxHi);
for i = 1:6
    trim_cmd(i) = (1.0 - lambda) * u0(i) + lambda * u1(i);
end
end

function K_lqr = localBlendGain(K_schedule, idxLo, idxHi, lambda)
K_lqr = zeros(6, 9);
K0 = localGainPage(K_schedule, idxLo);
K1 = localGainPage(K_schedule, idxHi);
for i = 1:6
    for j = 1:9
        K_lqr(i, j) = (1.0 - lambda) * K0(i, j) + lambda * K1(i, j);
    end
end
end

function value = localBlendScalar(vec, idxLo, idxHi, lambda)
value = (1.0 - lambda) * vec(idxLo) + lambda * vec(idxHi);
end
