%% Compare Airfoil360 wind-tunnel polars against the existing AVL alpha sweep
% This script is meant to help choose a blend window for a future
% low-alpha / high-alpha aero model.
%
% Important note:
% - The Airfoil360 workbook contains 2D airfoil data for NACA 2412 and
%   NACA 0012 through 360 deg.
% - The AVL alpha sweep currently on disk is a simplified whole-aircraft
%   result, not a pure section polar.
% - So this comparison is best used as a visual low-alpha reference, not as
%   a one-to-one "section polar validation."

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));

airfoil360Path = fullfile(repoRoot, 'Airfoil360_wind_tunnel_data_v2022.xlsx');
avlAlphaPath = fullfile(repoRoot, 'docs', 'avl_homework', 'tables', 'alpha_sweep.csv');

if exist(airfoil360Path, 'file') ~= 2
    error('plot_airfoil360_vs_avl:MissingWorkbook', ...
        'Could not find %s', airfoil360Path);
end

if exist(avlAlphaPath, 'file') ~= 2
    error('plot_airfoil360_vs_avl:MissingAVLSweep', ...
        'Could not find %s', avlAlphaPath);
end

% Airfoil360 sheets. Re = 100k is a reasonable first comparison because it
% is closer to the "cleaner" side of the low-Re data set, but both are
% shown so we can see the spread.
wing50 = localReadAirfoil360Sheet(airfoil360Path, 'NACA 2412 Re=50K');
wing100 = localReadAirfoil360Sheet(airfoil360Path, 'NACA 2412 Re=100K');
tail50 = localReadAirfoil360Sheet(airfoil360Path, 'NACA 0012 Re=50K');
tail100 = localReadAirfoil360Sheet(airfoil360Path, 'NACA 0012 Re=100K');

avlAlpha = readtable(avlAlphaPath);

comparison = struct();
comparison.paths = struct( ...
    'airfoil360', airfoil360Path, ...
    'avlAlpha', avlAlphaPath);
comparison.airfoil360 = struct( ...
    'wing50', wing50, ...
    'wing100', wing100, ...
    'tail50', tail50, ...
    'tail100', tail100);
comparison.avlAlpha = avlAlpha;
comparison.avlCLFit = localLinearFit(avlAlpha.alpha_deg, avlAlpha.CL_avl);

fitRangeDeg = [-6, 10];
comparison.cdShift = struct();
comparison.cdShift.fit_range_deg = fitRangeDeg;
comparison.cdShift.wing = localBestCDShift(avlAlpha.alpha_deg, avlAlpha.CD_avl, ...
    wing50.alpha_deg_signed, wing50.CD, wing100.alpha_deg_signed, wing100.CD, fitRangeDeg);
comparison.cdShift.tail = localBestCDShift(avlAlpha.alpha_deg, avlAlpha.CD_avl, ...
    tail50.alpha_deg_signed, tail50.CD, tail100.alpha_deg_signed, tail100.CD, fitRangeDeg);

figure('Color', 'k', 'Position', [120 100 1200 760]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(wing50.alpha_deg_signed, wing50.CL, '-o', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(wing50.alpha_deg_signed)); hold on;
plot(wing100.alpha_deg_signed, wing100.CL, '-s', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(wing100.alpha_deg_signed));
plot(avlAlpha.alpha_deg, avlAlpha.CL_avl, '-^', 'LineWidth', 1.2, ...
    'MarkerSize', 4, 'MarkerIndices', 1:height(avlAlpha), 'Color', [1 1 1]);
plot(avlAlpha.alpha_deg, comparison.avlCLFit.yhat, '-', 'LineWidth', 1.2, ...
    'Color', [1.0 0.55 0.15]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_L');
title('Wing CL Comparison');
legend({'Airfoil360 NACA 2412 Re=50k', ...
        'Airfoil360 NACA 2412 Re=100k', ...
        'AVL alpha sweep (whole aircraft)', ...
        'AVL linear fit'}, ...
        'Location', 'best');
localStyleAxes(gca);
localAddFitText(gca, comparison.avlCLFit);

nexttile;
plot(wing50.alpha_deg_signed, wing50.CD, '-o', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(wing50.alpha_deg_signed)); hold on;
plot(wing100.alpha_deg_signed, wing100.CD, '-s', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(wing100.alpha_deg_signed));
plot(avlAlpha.alpha_deg, avlAlpha.CD_avl, '-^', 'LineWidth', 1.2, ...
    'MarkerSize', 4, 'MarkerIndices', 1:height(avlAlpha), 'Color', [1 1 1]);
plot(avlAlpha.alpha_deg, comparison.cdShift.wing.cd_shifted, '-d', 'LineWidth', 1.2, ...
    'MarkerSize', 4, 'MarkerIndices', 1:height(avlAlpha), 'Color', [0.2 1.0 0.8]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_D');
title('Wing CD Comparison');
legend({'Airfoil360 NACA 2412 Re=50k', ...
        'Airfoil360 NACA 2412 Re=100k', ...
        'AVL alpha sweep (whole aircraft)', ...
        'AVL CD shifted by LS offset'}, ...
        'Location', 'best');
localStyleAxes(gca);
localAddShiftText(gca, comparison.cdShift.wing);

nexttile;
plot(tail50.alpha_deg_signed, tail50.CL, '-o', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(tail50.alpha_deg_signed)); hold on;
plot(tail100.alpha_deg_signed, tail100.CL, '-s', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(tail100.alpha_deg_signed));
plot(avlAlpha.alpha_deg, avlAlpha.CL_avl, '-^', 'LineWidth', 1.2, ...
    'MarkerSize', 4, 'MarkerIndices', 1:height(avlAlpha), 'Color', [1 1 1]);
plot(avlAlpha.alpha_deg, comparison.avlCLFit.yhat, '-', 'LineWidth', 1.2, ...
    'Color', [1.0 0.55 0.15]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_L');
title('Tail CL Comparison');
legend({'Airfoil360 NACA 0012 Re=50k', ...
        'Airfoil360 NACA 0012 Re=100k', ...
        'AVL alpha sweep (whole aircraft)', ...
        'AVL linear fit'}, ...
        'Location', 'best');
localStyleAxes(gca);
localAddFitText(gca, comparison.avlCLFit);

nexttile;
plot(tail50.alpha_deg_signed, tail50.CD, '-o', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(tail50.alpha_deg_signed)); hold on;
plot(tail100.alpha_deg_signed, tail100.CD, '-s', 'LineWidth', 1.0, ...
    'MarkerSize', 3, 'MarkerIndices', 1:numel(tail100.alpha_deg_signed));
plot(avlAlpha.alpha_deg, avlAlpha.CD_avl, '-^', 'LineWidth', 1.2, ...
    'MarkerSize', 4, 'MarkerIndices', 1:height(avlAlpha), 'Color', [1 1 1]);
plot(avlAlpha.alpha_deg, comparison.cdShift.tail.cd_shifted, '-d', 'LineWidth', 1.2, ...
    'MarkerSize', 4, 'MarkerIndices', 1:height(avlAlpha), 'Color', [0.2 1.0 0.8]);
grid on;
xlabel('\alpha (deg)');
ylabel('C_D');
title('Tail CD Comparison');
legend({'Airfoil360 NACA 0012 Re=50k', ...
        'Airfoil360 NACA 0012 Re=100k', ...
        'AVL alpha sweep (whole aircraft)', ...
        'AVL CD shifted by LS offset'}, ...
        'Location', 'best');
localStyleAxes(gca);
localAddShiftText(gca, comparison.cdShift.tail);

sgtitle('Airfoil360 vs AVL Low-Alpha Reference');

assignin('base', 'airfoil360_avl_comparison', comparison);

fprintf('Loaded Airfoil360 workbook: %s\n', airfoil360Path);
fprintf('Loaded AVL alpha sweep: %s\n', avlAlphaPath);
fprintf('Comparison struct assigned to workspace as airfoil360_avl_comparison\n');

function out = localReadAirfoil360Sheet(workbookPath, sheetName)
tbl = readtable(workbookPath, 'Sheet', sheetName);

if ~all(ismember({'AOA', 'CL', 'CD'}, tbl.Properties.VariableNames))
    error('plot_airfoil360_vs_avl:BadSheetFormat', ...
        'Sheet %s does not have expected columns AOA, CL, CD.', sheetName);
end

alpha0_360 = tbl.AOA(:);
alphaSigned = alpha0_360;
wrapMask = alphaSigned > 180;
alphaSigned(wrapMask) = alphaSigned(wrapMask) - 360;

[alphaSigned, sortIdx] = sort(alphaSigned);

out = struct();
out.sheet = sheetName;
out.alpha_deg_0_360 = alpha0_360;
out.alpha_deg_signed = alphaSigned;
out.CL = tbl.CL(sortIdx);
out.CD = tbl.CD(sortIdx);
end

function fit = localLinearFit(x, y)
x = x(:);
y = y(:);
p = polyfit(x, y, 1);
yhat = polyval(p, x);
ssRes = sum((y - yhat).^2);
ssTot = sum((y - mean(y)).^2);
if ssTot < eps
    r2 = 1.0;
else
    r2 = 1 - ssRes / ssTot;
end
fit = struct('slope', p(1), 'intercept', p(2), 'yhat', yhat, 'r2', r2);
end

function out = localBestCDShift(alphaAvl, cdAvl, alpha50, cd50, alpha100, cd100, fitRangeDeg)
mask = alphaAvl >= fitRangeDeg(1) & alphaAvl <= fitRangeDeg(2);
xFit = alphaAvl(mask);
yAvl = cdAvl(mask);

y50 = interp1(alpha50, cd50, xFit, 'linear', 'extrap');
y100 = interp1(alpha100, cd100, xFit, 'linear', 'extrap');
yTarget = 0.5 * (y50 + y100);

shift = mean(yTarget - yAvl);
yShifted = cdAvl + shift;

resid = yTarget - (yAvl + shift);
rmse = sqrt(mean(resid.^2));

out = struct();
out.shift = shift;
out.rmse = rmse;
out.fit_range_deg = fitRangeDeg;
out.cd_shifted = yShifted;
out.x_fit = xFit;
out.y_target = yTarget;
end

function localStyleAxes(ax)
set(ax, 'Color', 'k', ...
    'XColor', [1 1 1], ...
    'YColor', [1 1 1], ...
    'GridColor', [0.35 0.35 0.35], ...
    'MinorGridColor', [0.25 0.25 0.25], ...
    'FontName', 'Helvetica');
title(ax.Title.String, 'Color', [1 1 1]);
xlabel(ax.XLabel.String, 'Color', [1 1 1]);
ylabel(ax.YLabel.String, 'Color', [1 1 1]);
leg = legend(ax);
set(leg, 'TextColor', [1 1 1], 'Color', [0 0 0], 'EdgeColor', [0.5 0.5 0.5]);
end

function localAddFitText(ax, fit)
txt = sprintf('AVL fit: C_L = %.4f\\alpha + %.4f\\nR^2 = %.5f', ...
    fit.slope, fit.intercept, fit.r2);
text(ax, 0.03, 0.97, txt, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'Color', [1 1 1], ...
    'BackgroundColor', [0 0 0], ...
    'Margin', 4, ...
    'FontSize', 9);
end

function localAddShiftText(ax, shiftData)
txt = sprintf('Shift fit over [%g, %g] deg\\n\\DeltaC_D = %+0.5f\\nRMSE = %0.5f', ...
    shiftData.fit_range_deg(1), shiftData.fit_range_deg(2), ...
    shiftData.shift, shiftData.rmse);
text(ax, 0.03, 0.97, txt, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'Color', [1 1 1], ...
    'BackgroundColor', [0 0 0], ...
    'Margin', 4, ...
    'FontSize', 9);
end
