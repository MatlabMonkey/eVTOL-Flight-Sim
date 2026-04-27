%% Quick Cm comparison: AVL vs analytical models
% This is intentionally lightweight. It compares:
%   1) aircraft-level AVL Cm(alpha) from alpha_sweep.csv (only where AVL exists)
%   2) the old analytical/simple aircraft Cm(alpha) from alpha_sweep.csv
%   3) a new CN-based Cm candidate built from the Airfoil360 Re=100k tables
%
% Important note:
%   The AVL curve is whole-aircraft low-alpha data.
%   The wing/tail CN-based candidates are per-airfoil section models.

clear; clc;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
workbookPath = fullfile(repoRoot, 'Airfoil360_wind_tunnel_data_v2022.xlsx');
alphaSweepPath = fullfile(repoRoot, 'docs', 'avl_homework', 'tables', 'alpha_sweep.csv');

alphaTbl = readtable(alphaSweepPath);

alphaAvlDeg = alphaTbl.alpha_deg(:);
CmAvl = alphaTbl.Cm_avl(:);
CmSimple = alphaTbl.Cm_simple(:);
alphaFullDeg = (-180:1:180).';

cfg = struct();
cfg.xacOverC = 0.25;
cfg.wingXcgOverC = 0.05;
cfg.tailXcgOverC = 0.25;

wing = localReadAirfoil360Sheet(workbookPath, 'NACA 2412 Re=100K');
tail = localReadAirfoil360Sheet(workbookPath, 'NACA 0012 Re=100K');

wing.alphaStallDeg = localEstimatePositiveStall(wing.alpha_deg_signed, wing.CL);
tail.alphaStallDeg = localEstimatePositiveStall(tail.alpha_deg_signed, tail.CL);

wingModel = localCmCandidate(alphaFullDeg, wing, cfg.wingXcgOverC, cfg.xacOverC);
tailModel = localCmCandidate(alphaFullDeg, tail, cfg.tailXcgOverC, cfg.xacOverC);

figure('Color', 'w', 'Position', [120 120 1200 500]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(alphaFullDeg, wingModel.Cm, '-', 'Color', [0.47 0.67 0.19], ...
    'LineWidth', 1.8); hold on;
plot(alphaAvlDeg, CmAvl, 'o', 'Color', [0 0.45 0.74], ...
    'MarkerSize', 6, 'LineWidth', 1.5);
plot(alphaAvlDeg, CmSimple, 's', 'Color', [0.85 0.33 0.10], ...
    'MarkerSize', 6, 'LineWidth', 1.5);
xline([-wing.alphaStallDeg wing.alphaStallDeg], ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_m');
title('Wing Cm Comparison');
xlim([-180 180]);
legend('New CN-based candidate (full 360)', ...
    'AVL Cm (whole aircraft, low-\alpha only)', ...
    'Old analytical Cm_{simple} (low-\alpha only)', ...
    'Location', 'best');

nexttile;
plot(alphaFullDeg, tailModel.Cm, '-', 'Color', [0.49 0.18 0.56], ...
    'LineWidth', 1.8); hold on;
plot(alphaAvlDeg, CmAvl, 'o', 'Color', [0 0.45 0.74], ...
    'MarkerSize', 6, 'LineWidth', 1.5);
plot(alphaAvlDeg, CmSimple, 's', 'Color', [0.85 0.33 0.10], ...
    'MarkerSize', 6, 'LineWidth', 1.5);
xline([-tail.alphaStallDeg tail.alphaStallDeg], ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_m');
title('Tail Cm Comparison');
 xlim([-180 180]);
legend('New CN-based candidate (full 360)', ...
    'AVL Cm (whole aircraft, low-\alpha only)', ...
    'Old analytical Cm_{simple} (low-\alpha only)', ...
    'Location', 'best');

sgtitle('Cm Comparison: AVL vs Analytical Candidates');

fprintf('\nWing comparison\n');
fprintf('  wing alpha_stall_deg = %.3f\n', wing.alphaStallDeg);
fprintf('  wing x_cg/c          = %.3f\n', cfg.wingXcgOverC);
fprintf('  wing x_ac/c          = %.3f\n', cfg.xacOverC);
wingCmAtAvl = interp1(alphaFullDeg, wingModel.Cm, alphaAvlDeg, 'linear');
fprintf('  wing candidate vs AVL RMSE = %.6f\n', sqrt(mean((wingCmAtAvl - CmAvl).^2)));

fprintf('\nTail comparison\n');
fprintf('  tail alpha_stall_deg = %.3f\n', tail.alphaStallDeg);
fprintf('  tail x_cg/c          = %.3f\n', cfg.tailXcgOverC);
fprintf('  tail x_ac/c          = %.3f\n', cfg.xacOverC);
tailCmAtAvl = interp1(alphaFullDeg, tailModel.Cm, alphaAvlDeg, 'linear');
fprintf('  tail candidate vs AVL RMSE = %.6f\n', sqrt(mean((tailCmAtAvl - CmAvl).^2)));

function out = localReadAirfoil360Sheet(workbookPath, sheetName)
tbl = readtable(workbookPath, 'Sheet', sheetName);
alpha0360 = tbl.AOA(:);
alphaSigned = alpha0360;
mask = alphaSigned > 180;
alphaSigned(mask) = alphaSigned(mask) - 360;
[alphaSigned, idx] = sort(alphaSigned);
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

function model = localCmCandidate(alphaDeg, data, xcgOverC, xacOverC)
alphaRad = deg2rad(alphaDeg);
CL = interp1(data.alpha_deg_signed, data.CL, alphaDeg, 'linear', 'extrap');
CD = interp1(data.alpha_deg_signed, data.CD, alphaDeg, 'linear', 'extrap');
CN = CL .* cos(alphaRad) + CD .* sin(alphaRad);
Cm = (xcgOverC - xacOverC) .* CN;
model = struct('CL', CL, 'CD', CD, 'CN', CN, 'Cm', Cm);
end
