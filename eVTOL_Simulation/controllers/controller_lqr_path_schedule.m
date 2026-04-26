function u_cmd = controller_lqr_path_schedule(airData_cmd, tilt_angles_cmd, x_meas, x_ref_schedule, trim_cmd_schedule, K_schedule)
%CONTROLLER_LQR_PATH_SCHEDULE Interpolate an LQR along a scheduled trim path.
% Inputs:
%   airData_cmd       3x1 command = [Vinf; alpha; beta]
%   tilt_angles_cmd   2x1 grouped front tilt command [FR; FL] in deg
%   x_meas            9x1 measured controller state
%   x_ref_schedule    12xN schedule array:
%                       1:9  = x_ref columns
%                       10   = front tilt command [deg]
%                       11   = Vinf command [m/s]
%                       12   = path progress in [0,1]
%   trim_cmd_schedule 6xN = [front; rear; df; da; de; dr]
%   K_schedule        6x9xN schedule gain stack
%
% Output:
%   u_cmd             6x1 mixed control command

%#codegen

nPts = size(trim_cmd_schedule, 2);
if nPts <= 0
    u_cmd = controller_hold_trim(zeros(6, 1));
    return;
end

if nPts == 1
    trim_cmd = localTrimColumn(trim_cmd_schedule, 1);
    K_lqr = localGainPage(K_schedule, 1);
    x_ref = localStateColumn(x_ref_schedule, 1);
    u_cmd = controller_lqr_full_hold(x_meas, x_ref, trim_cmd, K_lqr);
    return;
end

tilt_sched = localScheduleRow(x_ref_schedule, 10, nPts);
vinf_sched = localScheduleRow(x_ref_schedule, 11, nPts);
progress_sched = localScheduleRow(x_ref_schedule, 12, nPts);
if localAllZero(progress_sched)
    progress_sched = localUnitProgress(nPts);
end

cmd_tilt_deg = 0.0;
if ~isempty(tilt_angles_cmd)
    cmd_tilt_deg = tilt_angles_cmd(1);
end
cmd_vinf_mps = 0.0;
if ~isempty(airData_cmd)
    cmd_vinf_mps = airData_cmd(1);
end

progress_cmd = localProjectToScheduledPath(cmd_tilt_deg, cmd_vinf_mps, tilt_sched, vinf_sched, progress_sched);
[idxLo, idxHi, lambda] = localBracketProgress(progress_cmd, progress_sched);

x_ref = localBlendState(x_ref_schedule, idxLo, idxHi, lambda);
trim_cmd = localBlendTrim(trim_cmd_schedule, idxLo, idxHi, lambda);
K_lqr = localBlendGain(K_schedule, idxLo, idxHi, lambda);

u_cmd = controller_lqr_full_hold(x_meas, x_ref, trim_cmd, K_lqr);
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

function [idxLo, idxHi, lambda] = localBracketProgress(progress_cmd, progress_sched)
nPts = numel(progress_sched);
idxLo = 1;
idxHi = 1;
lambda = 0.0;

if progress_cmd <= progress_sched(1)
    return;
end
if progress_cmd >= progress_sched(nPts)
    idxLo = nPts;
    idxHi = nPts;
    return;
end

for i = 1:(nPts - 1)
    p0 = progress_sched(i);
    p1 = progress_sched(i + 1);
    if progress_cmd >= p0 && progress_cmd <= p1
        idxLo = i;
        idxHi = i + 1;
        if p1 > p0
            lambda = (progress_cmd - p0) / (p1 - p0);
        end
        return;
    end
end

idxLo = nPts;
idxHi = nPts;
lambda = 0.0;
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
