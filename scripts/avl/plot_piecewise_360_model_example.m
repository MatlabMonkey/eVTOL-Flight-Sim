%% Example of the piecewise 360-deg aerodynamic model
% This script implements the exact CL/CD/Cm structure discussed in chat:
% - use the wind-tunnel table directly in attached flow
% - use a flat-plate approximation after stall
% - blend over a small window to avoid a hard kink
%
% The goal is not to declare this the final model. The goal is to make the
% math concrete and easy to inspect.

clear; clc;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
workbookPath = fullfile(repoRoot, 'Airfoil360_wind_tunnel_data_v2022.xlsx');

if exist(workbookPath, 'file') ~= 2
    error('plot_piecewise_360_model_example:MissingWorkbook', ...
        'Could not find %s', workbookPath);
end

% Settings to try. These x_cg/c values are example tuning knobs for the
% moment model only. The CL/CD logic does not depend on them.
cfg = struct();
cfg.blendWidthDeg = 3.0;
cfg.xacOverC = 0.25;
cfg.wingXcgOverC = 0.05;
cfg.tailXcgOverC = 0.25;

wing = localReadAirfoil360Sheet(workbookPath, 'NACA 2412 Re=100K');
tail = localReadAirfoil360Sheet(workbookPath, 'NACA 0012 Re=100K');

wing.alphaStallDeg = localEstimatePositiveStall(wing.alpha_deg_signed, wing.CL);
tail.alphaStallDeg = localEstimatePositiveStall(tail.alpha_deg_signed, tail.CL);

wing.CDmax = interp1(wing.alpha_deg_signed, wing.CD, 90, 'linear');
tail.CDmax = interp1(tail.alpha_deg_signed, tail.CD, 90, 'linear');

alphaDeg = -180:1:180;
alphaRad = deg2rad(alphaDeg);

wingModel = localPiecewiseModel(alphaRad, wing, cfg.wingXcgOverC, cfg.xacOverC, cfg.blendWidthDeg);
tailModel = localPiecewiseModel(alphaRad, tail, cfg.tailXcgOverC, cfg.xacOverC, cfg.blendWidthDeg);

figure('Color', 'w', 'Position', [100 100 1400 750]);
tiledlayout(2, 4, 'Padding', 'compact', 'TileSpacing', 'compact');

localPlotComparison(wing, wingModel, alphaDeg, 'Wing NACA 2412 Re=100k');
localPlotComparison(tail, tailModel, alphaDeg, 'Tail NACA 0012 Re=100k');

sgtitle('Piecewise 360-Degree Aerodynamic Model Example');

fprintf('\nWing setup\n');
fprintf('  alpha_stall_deg = %.3f\n', wing.alphaStallDeg);
fprintf('  CDmax @ 90 deg  = %.4f\n', wing.CDmax);
fprintf('  x_ac/c          = %.3f\n', cfg.xacOverC);
fprintf('  x_cg/c          = %.3f\n', cfg.wingXcgOverC);
fprintf('  attached CL rmse = %.6f\n', localAttachedRMSE(wing, wingModel, 'CL'));
fprintf('  attached CD rmse = %.6f\n', localAttachedRMSE(wing, wingModel, 'CD'));
fprintf('  boundary jump CL = %.6e\n', localBoundaryJump(wing, cfg.wingXcgOverC, cfg.xacOverC, cfg.blendWidthDeg, 'CL'));
fprintf('  boundary jump CD = %.6e\n', localBoundaryJump(wing, cfg.wingXcgOverC, cfg.xacOverC, cfg.blendWidthDeg, 'CD'));
fprintf('  boundary jump Cm = %.6e\n', localBoundaryJump(wing, cfg.wingXcgOverC, cfg.xacOverC, cfg.blendWidthDeg, 'Cm'));

fprintf('\nTail setup\n');
fprintf('  alpha_stall_deg = %.3f\n', tail.alphaStallDeg);
fprintf('  CDmax @ 90 deg  = %.4f\n', tail.CDmax);
fprintf('  x_ac/c          = %.3f\n', cfg.xacOverC);
fprintf('  x_cg/c          = %.3f\n', cfg.tailXcgOverC);
fprintf('  attached CL rmse = %.6f\n', localAttachedRMSE(tail, tailModel, 'CL'));
fprintf('  attached CD rmse = %.6f\n', localAttachedRMSE(tail, tailModel, 'CD'));
fprintf('  boundary jump CL = %.6e\n', localBoundaryJump(tail, cfg.tailXcgOverC, cfg.xacOverC, cfg.blendWidthDeg, 'CL'));
fprintf('  boundary jump CD = %.6e\n', localBoundaryJump(tail, cfg.tailXcgOverC, cfg.xacOverC, cfg.blendWidthDeg, 'CD'));
fprintf('  boundary jump Cm = %.6e\n', localBoundaryJump(tail, cfg.tailXcgOverC, cfg.xacOverC, cfg.blendWidthDeg, 'Cm'));

function localPlotComparison(data, model, alphaDeg, rowTitle)
nexttile;
plot(data.alpha_deg_signed, data.CL, 'o', 'Color', [0 0.45 0.74], ...
    'MarkerSize', 4, 'LineWidth', 1.0); hold on;
plot(alphaDeg, model.CL, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
xline([-data.alphaStallDeg data.alphaStallDeg], ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_L');
title(sprintf('%s: C_L', rowTitle));
legend('Table data', 'Piecewise model', 'Location', 'best');

nexttile;
plot(data.alpha_deg_signed, data.CD, 'o', 'Color', [0 0.45 0.74], ...
    'MarkerSize', 4, 'LineWidth', 1.0); hold on;
plot(alphaDeg, model.CD, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
xline([-data.alphaStallDeg data.alphaStallDeg], ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_D');
title(sprintf('%s: C_D', rowTitle));
legend('Table data', 'Piecewise model', 'Location', 'best');

nexttile;
plot(alphaDeg, model.CN, '-', 'Color', [0.47 0.67 0.19], 'LineWidth', 1.8);
xline([-data.alphaStallDeg data.alphaStallDeg], ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_N');
title(sprintf('%s: C_N', rowTitle));

nexttile;
plot(alphaDeg, model.Cm, '-', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
xline([-data.alphaStallDeg data.alphaStallDeg], ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_m');
title(sprintf('%s: C_m', rowTitle));
end

function out = localReadAirfoil360Sheet(workbookPath, sheetName)
tbl = readtable(workbookPath, 'Sheet', sheetName);
alpha0360 = tbl.AOA(:);
alphaSigned = alpha0360;
mask = alphaSigned > 180;
alphaSigned(mask) = alphaSigned(mask) - 360;
[alphaSigned, idx] = sort(alphaSigned);

% Drop duplicate endpoint if present (0 and 360 map to the same place).
[alphaSigned, uniqueIdx] = unique(alphaSigned, 'stable');
idx = idx(uniqueIdx);

out = struct();
out.sheet = sheetName;
out.alpha_deg_signed = alphaSigned;
out.CL = tbl.CL(idx);
out.CD = tbl.CD(idx);
end

function alphaStallDeg = localEstimatePositiveStall(alphaDeg, CL)
mask = alphaDeg >= 0 & alphaDeg <= 45;
alphaPos = alphaDeg(mask);
CLPos = CL(mask);
[~, idx] = max(CLPos);
alphaStallDeg = alphaPos(idx);
end

function model = localPiecewiseModel(alphaRad, data, xcgOverC, xacOverC, blendWidthDeg)
alphaWrapped = mod(alphaRad + pi, 2*pi) - pi;
alphaDeg = rad2deg(alphaWrapped);
absAlphaDeg = abs(alphaDeg);

alphaStallDeg = data.alphaStallDeg;
CDmax = data.CDmax;

CLtable = interp1(data.alpha_deg_signed, data.CL, alphaDeg, 'linear', 'extrap');
CDtable = interp1(data.alpha_deg_signed, data.CD, alphaDeg, 'linear', 'extrap');

CLplate = 0.5 * CDmax .* sin(2 * alphaWrapped);
CDplate = CDmax .* (sin(alphaWrapped).^2);

blendStart = alphaStallDeg;
blendEnd = alphaStallDeg + blendWidthDeg;
blendW = zeros(size(alphaDeg));

midMask = absAlphaDeg > blendStart & absAlphaDeg < blendEnd;
hiMask = absAlphaDeg >= blendEnd;
blendW(hiMask) = 1.0;

t = (absAlphaDeg(midMask) - blendStart) / max(blendWidthDeg, eps);
blendW(midMask) = t.^2 .* (3 - 2*t); % smoothstep

CL = (1 - blendW) .* CLtable + blendW .* CLplate;
CD = (1 - blendW) .* CDtable + blendW .* CDplate;

CN = CL .* cos(alphaWrapped) + CD .* sin(alphaWrapped);

CmAttached = ((xcgOverC - xacOverC)) .* CN;
CmPlate = ((xcgOverC - 0.5)) .* CN;
Cm = (1 - blendW) .* CmAttached + blendW .* CmPlate;

model = struct();
model.alpha_deg = alphaDeg;
model.CL = CL;
model.CD = CD;
model.CN = CN;
model.Cm = Cm;
model.blendW = blendW;
end

function rmse = localAttachedRMSE(data, model, fieldName)
mask = abs(data.alpha_deg_signed) <= data.alphaStallDeg;
[alphaUnique, idxUnique] = unique(model.alpha_deg, 'stable');
fieldUnique = model.(fieldName)(idxUnique);
pred = interp1(alphaUnique, fieldUnique, data.alpha_deg_signed(mask), 'linear');
truth = data.(fieldName)(mask);
rmse = sqrt(mean((pred - truth).^2));
end

function jump = localBoundaryJump(data, xcgOverC, xacOverC, blendWidthDeg, fieldName)
epsDeg = 1e-4;
left = localPiecewiseModel(deg2rad(data.alphaStallDeg - epsDeg), data, xcgOverC, xacOverC, blendWidthDeg);
right = localPiecewiseModel(deg2rad(data.alphaStallDeg + epsDeg), data, xcgOverC, xacOverC, blendWidthDeg);
jump = right.(fieldName) - left.(fieldName);
end
