function [u_cmd, tilt_cmd_deg, scheduler_debug] = controller_indi_transition( ...
    airData_cmd, tilt_angles_cmd, x_meas, accel_meas, actuator_state_meas, ...
    angular_accel_meas, x_ref_schedule, trim_cmd_schedule, indi_schedule, ...
    surface_limit_rad, front_collective_min_rpm, front_collective_max_rpm, ...
    rear_collective_min_rpm, rear_collective_max_rpm)
%CONTROLLER_INDI_TRANSITION Longitudinal transition controller using INDI.
%
% Schedule packing:
%   x_ref_schedule(1:9,:)      = [phi theta psi u v w p q r]
%   x_ref_schedule(10,:)       = scheduled front tilt [deg]
%   x_ref_schedule(11,:)       = scheduled Vinf [m/s]
%   x_ref_schedule(12,:)       = path progress [0,1]
%   x_ref_schedule(13:19,:)    = settle/ramp settings
%   x_ref_schedule(20:24,:)    = outer-loop settings
%   trim_cmd_schedule(:,i)     = [front rear df da de dr]
%   indi_schedule(1:3,1:4,i)   = G for [ax az qdot] vs
%                                [front_rpm^2 rear_rpm^2 df de]
%   indi_schedule(4,1:4,i)     = control regularization weights
%   indi_schedule(5,1:3,i)     = virtual-axis weights
%   indi_schedule(6,1:4,i)     = per-sample delta-eta limits
%
% The acceleration input is treated as body-frame specific force. Therefore
% the trim feedforward target includes gravity projected into body axes.

%#codegen

nPts = localInferScheduleCount(x_ref_schedule, trim_cmd_schedule);
u_cmd = controller_hold_trim(zeros(6, 1));
tilt_cmd_deg = 0.0;
scheduler_debug = zeros(12, 1);

if nPts <= 0
    return;
end

tilt_sched = localScheduleRow(x_ref_schedule, 10, nPts);
vinf_sched = localScheduleRow(x_ref_schedule, 11, nPts);
progress_sched = localScheduleRow(x_ref_schedule, 12, nPts);
if localAllZero(progress_sched)
    progress_sched = localUnitProgress(nPts);
end

cmd_tilt_deg = localFirstOrDefault(tilt_angles_cmd, 0.0);
cmd_vinf_mps = localFirstOrDefault(airData_cmd, 0.0);
cmd_progress = localProjectToScheduledPath( ...
    cmd_tilt_deg, cmd_vinf_mps, tilt_sched, vinf_sched, progress_sched);

persistent initialized path_signature current_idx segment_lambda settled_counter in_transition
if isempty(initialized)
    initialized = false;
    path_signature = 0.0;
    current_idx = int32(1);
    segment_lambda = 0.0;
    settled_counter = int32(0);
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
[idxLo, idxHi, lambda, current_idx, segment_lambda, settled_counter, in_transition] = ...
    localAdvanceScheduler(x_meas, x_ref_schedule, current_idx, ...
    segment_lambda, settled_counter, in_transition, nPts);

x_ref = localBlendState(x_ref_schedule, idxLo, idxHi, lambda);
trim_cmd = localBlendTrim(trim_cmd_schedule, idxLo, idxHi, lambda);
G = localBlendEffectiveness(indi_schedule, idxLo, idxHi, lambda);
w_eta = localBlendScheduleVector(indi_schedule, 4, 4, idxLo, idxHi, lambda, ...
    [2.0e-7; 2.0e-7; 6.0; 6.0]);
w_nu = localBlendScheduleVector(indi_schedule, 5, 3, idxLo, idxHi, lambda, ...
    [1.0; 1.0; 0.9]);
delta_eta_limit = localBlendScheduleVector(indi_schedule, 6, 4, idxLo, idxHi, lambda, ...
    [2.0e6; 2.0e6; deg2rad(10.0); deg2rad(10.0)]);

outer = localOuterLoopSettings(x_ref_schedule, idxLo, idxHi, lambda);
nu_meas = localMeasuredVirtualAcceleration(accel_meas, angular_accel_meas);
nu_cmd = localCommandedVirtualAcceleration(x_ref, x_meas, outer);
nu_err = localClipVector(nu_cmd - nu_meas, outer.accel_error_clip);

eta_base = localMeasuredEta(actuator_state_meas, trim_cmd);
delta_eta = localWeightedLeastSquares(G, w_nu, w_eta, nu_err);
delta_eta = localClipVector(delta_eta, delta_eta_limit);
eta_cmd = eta_base + delta_eta;

u_cmd = localEtaToMixedCommand(eta_cmd, surface_limit_rad, ...
    front_collective_min_rpm, front_collective_max_rpm, ...
    rear_collective_min_rpm, rear_collective_max_rpm);
tilt_cmd_deg = localBlendScalar(tilt_sched, idxLo, idxHi, lambda);

scheduler_debug(1) = localBlendScalar(progress_sched, idxLo, idxHi, lambda);
scheduler_debug(2) = double(idxLo);
scheduler_debug(3) = double(idxHi);
scheduler_debug(4) = lambda;
scheduler_debug(5) = cmd_progress;
scheduler_debug(6:8) = nu_err;
scheduler_debug(9:12) = delta_eta;
end

function nPts = localInferScheduleCount(x_ref_schedule, trim_cmd_schedule)
nMax = size(trim_cmd_schedule, 2);
nPts = nMax;
if nMax <= 0
    return;
end

if size(x_ref_schedule, 1) >= 12
    progress = x_ref_schedule(12, 1:nMax);
    if ~localAllZero(progress)
        nPts = 1;
        for i = 2:nMax
            if progress(i) > progress(i - 1) + 1e-9
                nPts = i;
            elseif abs(progress(i) - progress(i - 1)) <= 1e-9 && i > 2
                break;
            end
        end
    end
end
nPts = max(1, nPts);
end

function [idxLo, idxHi, lambda, current_idx, segment_lambda, settled_counter, in_transition] = ...
    localAdvanceScheduler(x_meas, x_ref_schedule, current_idx, segment_lambda, ...
    settled_counter, in_transition, nPts)
idxLo = double(current_idx);
idxHi = idxLo;
lambda = 0.0;

if current_idx >= int32(nPts)
    return;
end

settle = localSettleSettings(x_ref_schedule, idxLo);
if ~in_transition
    if localIsSettledAtPoint(x_meas, x_ref_schedule, idxLo, settle)
        settled_counter = settled_counter + 1;
    else
        settled_counter = int32(0);
    end

    if settled_counter >= localSettleSampleCount(settle.settle_time_s)
        in_transition = true;
        segment_lambda = 0.0;
    end
end

if in_transition
    segment_lambda = min(segment_lambda + localLambdaStep(settle.segment_ramp_time_s), 1.0);
    idxHi = idxLo + 1;
    lambda = segment_lambda;

    if segment_lambda >= 1.0 - 1e-9
        current_idx = current_idx + 1;
        current_idx = min(current_idx, int32(nPts));
        segment_lambda = 0.0;
        settled_counter = int32(0);
        in_transition = false;
        idxLo = double(current_idx);
        idxHi = idxLo;
        lambda = 0.0;
    end
end
end

function settle = localSettleSettings(x_ref_schedule, idx)
settle = struct();
settle.theta_deg = localScheduleScalarWithDefault(x_ref_schedule, 13, idx, 3.0);
settle.u_mps = localScheduleScalarWithDefault(x_ref_schedule, 14, idx, 1.5);
settle.w_mps = localScheduleScalarWithDefault(x_ref_schedule, 15, idx, 1.5);
settle.q_deg_s = localScheduleScalarWithDefault(x_ref_schedule, 16, idx, 4.0);
settle.settle_time_s = localScheduleScalarWithDefault(x_ref_schedule, 18, idx, 0.75);
settle.segment_ramp_time_s = localScheduleScalarWithDefault(x_ref_schedule, 19, idx, 8.0);
end

function tf = localIsSettledAtPoint(x_meas, x_ref_schedule, idx, settle)
x_ref = localStateColumn(x_ref_schedule, idx);
theta_err_deg = abs((x_meas(2) - x_ref(2)) * 180.0 / pi);
u_err = abs(x_meas(4) - x_ref(4));
w_err = abs(x_meas(6) - x_ref(6));
q_err_deg_s = abs((x_meas(8) - x_ref(8)) * 180.0 / pi);

tf = theta_err_deg <= settle.theta_deg && ...
     u_err <= settle.u_mps && ...
     w_err <= settle.w_mps && ...
     q_err_deg_s <= settle.q_deg_s;
end

function outer = localOuterLoopSettings(x_ref_schedule, idxLo, idxHi, lambda)
outer = struct();
outer.ku = localBlendScheduleScalarWithDefault(x_ref_schedule, 20, idxLo, idxHi, lambda, 0.45);
outer.kw = localBlendScheduleScalarWithDefault(x_ref_schedule, 21, idxLo, idxHi, lambda, 0.45);
outer.kq = localBlendScheduleScalarWithDefault(x_ref_schedule, 22, idxLo, idxHi, lambda, 1.40);
outer.ktheta = localBlendScheduleScalarWithDefault(x_ref_schedule, 23, idxLo, idxHi, lambda, 1.80);
outer.accel_error_clip = localBlendScheduleScalarWithDefault(x_ref_schedule, 24, idxLo, idxHi, lambda, 8.0);
end

function nu_meas = localMeasuredVirtualAcceleration(accel_meas, angular_accel_meas)
accel = localVec(accel_meas, 3);
angular_accel = localVec(angular_accel_meas, 3);
nu_meas = [accel(1); accel(3); angular_accel(2)];
end

function nu_cmd = localCommandedVirtualAcceleration(x_ref, x_meas, outer)
g = 9.81;
theta_ref = x_ref(2);

% Steady body specific force for zero inertial acceleration:
% Fx/m = g sin(theta), Fz/m = -g cos(theta) for phi approximately zero.
specific_force_ff = [g * sin(theta_ref); -g * cos(theta_ref)];

nu_cmd = zeros(3, 1);
nu_cmd(1) = specific_force_ff(1) + outer.ku * (x_ref(4) - x_meas(4));
nu_cmd(2) = specific_force_ff(2) + outer.kw * (x_ref(6) - x_meas(6));
nu_cmd(3) = outer.kq * (x_ref(8) - x_meas(8)) + ...
    outer.ktheta * (x_ref(2) - x_meas(2));
end

function eta = localMeasuredEta(actuator_state_meas, trim_cmd)
act = localVec(actuator_state_meas, 10);
front_rpm = trim_cmd(1);
rear_rpm = trim_cmd(2);
delta_f = trim_cmd(3);
delta_e = trim_cmd(5);

if abs(act(1)) + abs(act(2)) > 1.0
    front_rpm = 0.5 * (act(1) + act(2));
end
if abs(act(3)) + abs(act(4)) > 1.0
    rear_rpm = 0.5 * (act(3) + act(4));
end
if any(isfinite(act(7:10)))
    delta_f = 0.5 * (act(7) + act(8));
    delta_e = 0.5 * (act(9) + act(10));
end

front_rpm = max(front_rpm, 0.0);
rear_rpm = max(rear_rpm, 0.0);
eta = [front_rpm * front_rpm; rear_rpm * rear_rpm; delta_f; delta_e];
end

function delta_eta = localWeightedLeastSquares(G, w_nu, w_eta, nu_err)
A = zeros(4, 4);
b = zeros(4, 1);

for i = 1:3
    wi2 = w_nu(i) * w_nu(i);
    for j = 1:4
        b(j) = b(j) + wi2 * G(i, j) * nu_err(i);
        for k = 1:4
            A(j, k) = A(j, k) + wi2 * G(i, j) * G(i, k);
        end
    end
end

for j = 1:4
    A(j, j) = A(j, j) + w_eta(j) * w_eta(j) + 1e-12;
end

delta_eta = A \ b;
end

function u_cmd = localEtaToMixedCommand(eta_cmd, surface_limit_rad, ...
    front_min_rpm, front_max_rpm, rear_min_rpm, rear_max_rpm)
front_sq = min(max(eta_cmd(1), max(front_min_rpm, 0.0)^2), max(front_max_rpm, 0.0)^2);
rear_sq = min(max(eta_cmd(2), max(rear_min_rpm, 0.0)^2), max(rear_max_rpm, 0.0)^2);

u_cmd = zeros(6, 1);
u_cmd(1) = sqrt(max(front_sq, 0.0));
u_cmd(2) = sqrt(max(rear_sq, 0.0));
u_cmd(3) = min(max(eta_cmd(3), -surface_limit_rad), surface_limit_rad);
u_cmd(4) = 0.0;
u_cmd(5) = min(max(eta_cmd(4), -surface_limit_rad), surface_limit_rad);
u_cmd(6) = 0.0;
end

function vec = localClipVector(vec, limits)
if isscalar(limits)
    for i = 1:numel(vec)
        vec(i) = min(max(vec(i), -limits), limits);
    end
    return;
end

for i = 1:min(numel(vec), numel(limits))
    vec(i) = min(max(vec(i), -limits(i)), limits(i));
end
end

function row = localScheduleRow(x_ref_schedule, rowIdx, nPts)
row = zeros(1, nPts);
if size(x_ref_schedule, 1) >= rowIdx
    row(1, :) = x_ref_schedule(rowIdx, 1:nPts);
end
end

function value = localScheduleScalarWithDefault(x_ref_schedule, rowIdx, idx, fallback)
value = fallback;
if size(x_ref_schedule, 1) >= rowIdx && size(x_ref_schedule, 2) >= idx
    candidate = x_ref_schedule(rowIdx, idx);
    if isfinite(candidate) && abs(candidate) > 0
        value = candidate;
    end
end
end

function value = localBlendScheduleScalarWithDefault(x_ref_schedule, rowIdx, idxLo, idxHi, lambda, fallback)
v0 = localScheduleScalarWithDefault(x_ref_schedule, rowIdx, idxLo, fallback);
v1 = localScheduleScalarWithDefault(x_ref_schedule, rowIdx, idxHi, fallback);
value = (1.0 - lambda) * v0 + lambda * v1;
end

function vec = localBlendScheduleVector(schedule, rowIdx, n, idxLo, idxHi, lambda, fallback)
vec = fallback(:);
if size(schedule, 1) < rowIdx
    return;
end

v0 = zeros(n, 1);
v1 = zeros(n, 1);
for i = 1:n
    v0(i) = schedule(rowIdx, i, idxLo);
    v1(i) = schedule(rowIdx, i, idxHi);
end
candidate = (1.0 - lambda) * v0 + lambda * v1;
for i = 1:n
    if isfinite(candidate(i)) && abs(candidate(i)) > 0
        vec(i) = candidate(i);
    end
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

function G = localEffectivenessPage(schedule, idx)
G = zeros(3, 4);
rows = min(size(schedule, 1), 3);
cols = min(size(schedule, 2), 4);
G(1:rows, 1:cols) = schedule(1:rows, 1:cols, idx);
end

function x_ref = localBlendState(x_ref_schedule, idxLo, idxHi, lambda)
x0 = localStateColumn(x_ref_schedule, idxLo);
x1 = localStateColumn(x_ref_schedule, idxHi);
x_ref = (1.0 - lambda) * x0 + lambda * x1;
end

function trim_cmd = localBlendTrim(trim_cmd_schedule, idxLo, idxHi, lambda)
u0 = localTrimColumn(trim_cmd_schedule, idxLo);
u1 = localTrimColumn(trim_cmd_schedule, idxHi);
trim_cmd = (1.0 - lambda) * u0 + lambda * u1;
end

function G = localBlendEffectiveness(schedule, idxLo, idxHi, lambda)
G0 = localEffectivenessPage(schedule, idxLo);
G1 = localEffectivenessPage(schedule, idxHi);
G = (1.0 - lambda) * G0 + lambda * G1;
end

function value = localBlendScalar(vec, idxLo, idxHi, lambda)
value = (1.0 - lambda) * vec(idxLo) + lambda * vec(idxHi);
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

function step = localLambdaStep(segment_ramp_time_s)
step = 0.01 / max(segment_ramp_time_s, 0.25);
end

function n = localSettleSampleCount(settle_time_s)
n = int32(max(1.0, round(settle_time_s / 0.01)));
end

function sig = localScheduleSignature(tilt_sched, vinf_sched, progress_sched)
sig = sum(tilt_sched) + 3.7 * sum(vinf_sched) + ...
    11.0 * sum(progress_sched) + 101.0 * numel(progress_sched);
end

function value = localFirstOrDefault(in, fallback)
value = fallback;
if ~isempty(in)
    value = in(1);
end
end

function vec = localVec(in, n)
vec = zeros(n, 1);
count = min(numel(in), n);
if count > 0
    vec(1:count) = in(1:count);
end
end
