%% Compare pasted XFOIL polars against Airfoil360, analytical Cm, and blended Cm table
% This is intentionally lightweight and uses the XFOIL data pasted in chat.
% It overlays:
%   1) XFOIL CL against Airfoil360 CL for the same airfoil
%   2) XFOIL Cm against:
%      - a fitted analytical Cm(alpha) model over full 360 deg
%      - a blended one-table Cm built from low-alpha XFOIL and
%        high-alpha flat-plate-style local moment
%
% Notes:
% - XFOIL is low-alpha only here.
% - Airfoil360 is full 360-deg data.
% - The fitted Cm model is just a compact analytical approximation to the
%   local section moment curve, not an aircraft-level AVL surrogate.

clear; clc;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
workbookPath = fullfile(repoRoot, 'Airfoil360_wind_tunnel_data_v2022.xlsx');

wing360 = localReadAirfoil360Sheet(workbookPath, 'NACA 2412 Re=100K');
tail360 = localReadAirfoil360Sheet(workbookPath, 'NACA 0012 Re=100K');

wingXfoil = localWingXfoilData();
tailXfoil = localTailXfoilData();

alphaFullDeg = (-180:1:180).';
alphaFullRad = deg2rad(alphaFullDeg);

wingFit = localFitPiecewiseCmModel(wingXfoil.alpha_deg(:), wingXfoil.CM(:), 4);
tailFit = localFitPiecewiseCmModel(tailXfoil.alpha_deg(:), tailXfoil.CM(:), 4);

wingCmFull = localEvalPiecewiseCmModel(alphaFullDeg, wingFit);
tailCmFull = localEvalPiecewiseCmModel(alphaFullDeg, tailFit);
wingCmAtXfoil = localEvalPiecewiseCmModel(wingXfoil.alpha_deg(:), wingFit);
tailCmAtXfoil = localEvalPiecewiseCmModel(tailXfoil.alpha_deg(:), tailFit);

wingBlend = localBuildBlendedCmTable(alphaFullDeg, wing360, wingXfoil, 12, 20);
tailBlend = localBuildBlendedCmTable(alphaFullDeg, tail360, tailXfoil, 10, 18);
wingBlendAtXfoil = interp1(alphaFullDeg, wingBlend.Cm, wingXfoil.alpha_deg(:), 'linear');
tailBlendAtXfoil = interp1(alphaFullDeg, tailBlend.Cm, tailXfoil.alpha_deg(:), 'linear');

figure('Color', 'w', 'Position', [120 120 1300 700]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(wing360.alpha_deg_signed, wing360.CL, 'o', ...
    'Color', [0 0.45 0.74], 'MarkerSize', 4, 'LineWidth', 1.0); hold on;
plot(wingXfoil.alpha_deg, wingXfoil.CL, '-', ...
    'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
grid on;
xlabel('\alpha (deg)');
ylabel('C_L');
title('Wing Lift Comparison');
legend('Airfoil360 NACA 2412 Re=100k', 'XFOIL NACA 2412 Re=100k', 'Location', 'best');
xlim([-180 180]);

nexttile;
plot(tail360.alpha_deg_signed, tail360.CL, 'o', ...
    'Color', [0 0.45 0.74], 'MarkerSize', 4, 'LineWidth', 1.0); hold on;
plot(tailXfoil.alpha_deg, tailXfoil.CL, '-', ...
    'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
grid on;
xlabel('\alpha (deg)');
ylabel('C_L');
title('Tail Lift Comparison');
legend('Airfoil360 NACA 0012 Re=100k', 'XFOIL NACA 0012 Re=100k', 'Location', 'best');
xlim([-180 180]);

nexttile;
plot(wingXfoil.alpha_deg, wingXfoil.CM, 'o-', ...
    'Color', [0.47 0.67 0.19], 'MarkerSize', 5, 'LineWidth', 1.5); hold on;
plot(alphaFullDeg, wingCmFull, '-', ...
    'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
plot(alphaFullDeg, wingBlend.Cm, '-', ...
    'Color', [0.93 0.69 0.13], 'LineWidth', 1.8);
grid on;
xlabel('\alpha (deg)');
ylabel('C_m');
title('Wing Cm Comparison');
xlim([-180 180]);
legend('XFOIL NACA 2412 Re=100k', 'Analytical Cm fit (full 360)', ...
    'Blended Cm table (XFOIL + flat plate)', 'Location', 'best');

nexttile;
plot(tailXfoil.alpha_deg, tailXfoil.CM, 'o-', ...
    'Color', [0.47 0.67 0.19], 'MarkerSize', 5, 'LineWidth', 1.5); hold on;
plot(alphaFullDeg, tailCmFull, '-', ...
    'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
plot(alphaFullDeg, tailBlend.Cm, '-', ...
    'Color', [0.93 0.69 0.13], 'LineWidth', 1.8);
grid on;
xlabel('\alpha (deg)');
ylabel('C_m');
title('Tail Cm Comparison');
 xlim([-180 180]);
legend('XFOIL NACA 0012 Re=100k', 'Analytical Cm fit (full 360)', ...
    'Blended Cm table (XFOIL + flat plate)', 'Location', 'best');

sgtitle('XFOIL vs Airfoil360 / Analytical Comparison');

fprintf('\nWing CL overlap RMSE (XFOIL vs Airfoil360 interp) = %.6f\n', ...
    localOverlapRMSE(wingXfoil.alpha_deg, wingXfoil.CL, wing360.alpha_deg_signed, wing360.CL));
fprintf('Tail CL overlap RMSE (XFOIL vs Airfoil360 interp) = %.6f\n', ...
    localOverlapRMSE(tailXfoil.alpha_deg, tailXfoil.CL, tail360.alpha_deg_signed, tail360.CL));
fprintf('Wing Cm RMSE (XFOIL vs analytical fit) = %.6f\n', ...
    sqrt(mean((wingXfoil.CM(:) - wingCmAtXfoil).^2)));
fprintf('Tail Cm RMSE (XFOIL vs analytical fit) = %.6f\n', ...
    sqrt(mean((tailXfoil.CM(:) - tailCmAtXfoil).^2)));
fprintf('Wing Cm RMSE (XFOIL vs blended table) = %.6f\n', ...
    sqrt(mean((wingXfoil.CM(:) - wingBlendAtXfoil).^2)));
fprintf('Tail Cm RMSE (XFOIL vs blended table) = %.6f\n', ...
    sqrt(mean((tailXfoil.CM(:) - tailBlendAtXfoil).^2)));

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

function rmse = localOverlapRMSE(alphaA, yA, alphaB, yB)
yBinterp = interp1(alphaB, yB, alphaA, 'linear');
rmse = sqrt(mean((yA - yBinterp).^2));
end

function fit = localFitPiecewiseCmModel(alphaDeg, cmData, polyOrder)
alphaDeg = alphaDeg(:);
cmData = cmData(:);

negMask = alphaDeg <= 0;
posMask = alphaDeg >= 0;

fit = struct();
fit.polyOrder = polyOrder;
fit.alphaMin = min(alphaDeg);
fit.alphaMax = max(alphaDeg);
fit.negCoeff = polyfit(alphaDeg(negMask), cmData(negMask), polyOrder);
fit.posCoeff = polyfit(alphaDeg(posMask), cmData(posMask), polyOrder);
end

function out = localBuildBlendedCmTable(alphaDeg, airfoil360, xfoil, alphaBlendStartDeg, alphaBlendEndDeg)
alphaDeg = alphaDeg(:);
alphaRad = deg2rad(alphaDeg);
absAlpha = abs(alphaDeg);

CL = localPeriodicInterp(alphaDeg, airfoil360.alpha_deg_signed, airfoil360.CL);
CD = localPeriodicInterp(alphaDeg, airfoil360.alpha_deg_signed, airfoil360.CD);
CN = CL .* cos(alphaRad) + CD .* sin(alphaRad);

% Local section moment about quarter-chord from a simple high-alpha
% center-of-pressure assumption. This is only the local moment term, so it
% does not include the separate CG force-arm moment already handled by
% cross(r_arm, F) in the main model.
CmFlat = -0.25 .* CN;

alphaX = xfoil.alpha_deg(:);
cmX = xfoil.CM(:);
CmXfoil = interp1(alphaX, cmX, min(max(alphaDeg, min(alphaX)), max(alphaX)), 'pchip');

Cm = CmFlat;
insideMask = absAlpha <= alphaBlendStartDeg;
Cm(insideMask) = CmXfoil(insideMask);

% Use separate Hermite blends on the negative and positive sides so we can
% match both value and slope at the XFOIL edge and the flat-plate edge.
negMask = alphaDeg < -alphaBlendStartDeg & alphaDeg > -alphaBlendEndDeg;
posMask = alphaDeg > alphaBlendStartDeg & alphaDeg < alphaBlendEndDeg;

if any(negMask)
    x0 = -alphaBlendStartDeg;
    x1 = -alphaBlendEndDeg;
    y0 = interp1(alphaDeg, CmXfoil, x0, 'linear');
    y1 = interp1(alphaDeg, CmFlat, x1, 'linear');
    m0 = localNumericalSlope(alphaDeg, CmXfoil, x0);
    m1 = localNumericalSlope(alphaDeg, CmFlat, x1);
    Cm(negMask) = localHermiteSegment(alphaDeg(negMask), x0, y0, m0, x1, y1, m1);
end

if any(posMask)
    x0 = alphaBlendStartDeg;
    x1 = alphaBlendEndDeg;
    y0 = interp1(alphaDeg, CmXfoil, x0, 'linear');
    y1 = interp1(alphaDeg, CmFlat, x1, 'linear');
    m0 = localNumericalSlope(alphaDeg, CmXfoil, x0);
    m1 = localNumericalSlope(alphaDeg, CmFlat, x1);
    Cm(posMask) = localHermiteSegment(alphaDeg(posMask), x0, y0, m0, x1, y1, m1);
end

% Enforce periodic endpoint agreement exactly.
endVal = 0.5 * (Cm(alphaDeg == -180) + Cm(alphaDeg == 180));
Cm(alphaDeg == -180) = endVal;
Cm(alphaDeg == 180) = endVal;

out = struct('Cm', Cm, 'CmXfoilExt', CmXfoil, 'CmFlat', CmFlat);
end

function cm = localEvalPiecewiseCmModel(alphaDeg, fit)
alphaDeg = alphaDeg(:);
cm = zeros(size(alphaDeg));

negInside = alphaDeg >= fit.alphaMin & alphaDeg <= 0;
posInside = alphaDeg >= 0 & alphaDeg <= fit.alphaMax;
negOuter = alphaDeg < fit.alphaMin;
posOuter = alphaDeg > fit.alphaMax;

cm(negInside) = polyval(fit.negCoeff, alphaDeg(negInside));
cm(posInside) = polyval(fit.posCoeff, alphaDeg(posInside));

negSlopeCoeff = polyder(fit.negCoeff);
posSlopeCoeff = polyder(fit.posCoeff);

cmNegEdge = polyval(fit.negCoeff, fit.alphaMin);
cmPosEdge = polyval(fit.posCoeff, fit.alphaMax);
slopeNegEdge = polyval(negSlopeCoeff, fit.alphaMin);
slopePosEdge = polyval(posSlopeCoeff, fit.alphaMax);

if any(negOuter)
    cm(negOuter) = localHermiteSegment(alphaDeg(negOuter), ...
        -180, 0, 0, fit.alphaMin, cmNegEdge, slopeNegEdge);
end

if any(posOuter)
    cm(posOuter) = localHermiteSegment(alphaDeg(posOuter), ...
        fit.alphaMax, cmPosEdge, slopePosEdge, 180, 0, 0);
end
end

function y = localHermiteSegment(x, x0, y0, m0, x1, y1, m1)
t = (x - x0) ./ (x1 - x0);
h00 = 2 .* t.^3 - 3 .* t.^2 + 1;
h10 = t.^3 - 2 .* t.^2 + t;
h01 = -2 .* t.^3 + 3 .* t.^2;
h11 = t.^3 - t.^2;
y = h00 .* y0 + h10 .* (x1 - x0) .* m0 + h01 .* y1 + h11 .* (x1 - x0) .* m1;
end

function yq = localPeriodicInterp(xq, xData, yData)
xqWrapped = mod(xq + 180, 360) - 180;

xAug = [xData(:) - 360; xData(:); xData(:) + 360];
yAug = [yData(:); yData(:); yData(:)];
[xAug, sortIdx] = sort(xAug);
yAug = yAug(sortIdx);

yq = interp1(xAug, yAug, xqWrapped, 'pchip');
end

function m = localNumericalSlope(x, y, x0)
dx = 1e-3;
yPlus = interp1(x, y, x0 + dx, 'pchip');
yMinus = interp1(x, y, x0 - dx, 'pchip');
m = (yPlus - yMinus) / (2 * dx);
end

function out = localWingXfoilData()
out = struct();
out.alpha_deg = [ ...
    -9.25 -9.00 -8.75 -8.50 -8.25 -8.00 -7.75 -7.50 -7.25 -7.00 ...
    -6.75 -6.50 -6.25 -6.00 -5.75 -5.50 -5.25 -5.00 -4.75 -4.50 ...
    -4.25 -4.00 -3.75 -3.50 -3.25 -3.00 -2.75 -2.50 -2.25 -2.00 ...
    -1.75 -1.50 -1.25 -1.00 -0.75 -0.50 -0.25 0.00 0.25 0.50 ...
    0.75 1.00 1.25 1.50 1.75 2.00 2.25 2.50 2.75 3.00 ...
    3.25 3.50 3.75 4.00 4.25 4.50 4.75 5.00 5.25 5.50 ...
    5.75 6.00 6.25 6.50 6.75 7.00 7.25 7.50 7.75 8.00 ...
    8.25 8.50 8.75 9.00 9.25 9.50 9.75 10.00 10.25 10.50 ...
    10.75 11.00 11.25 11.50 11.75 12.00 12.25 12.50 12.75 13.00 ...
    13.25 13.50 13.75 14.00 14.25 14.50];
out.CL = [ ...
    -0.4676 -0.6490 -0.6415 -0.6490 -0.6597 -0.6647 -0.6640 -0.6588 -0.6495 -0.6370 ...
    -0.6223 -0.6052 -0.5859 -0.5659 -0.5454 -0.5240 -0.5023 -0.4796 -0.4571 -0.4346 ...
    -0.4122 -0.3902 -0.3680 -0.3461 -0.3240 -0.3019 -0.2753 -0.2306 -0.1869 -0.1494 ...
    -0.1161 -0.0695 0.0156 0.0782 0.1209 0.1665 0.2237 0.2623 0.3002 0.3370 ...
    0.3725 0.4063 0.4371 0.4637 0.4896 0.5150 0.5396 0.5637 0.5877 0.6117 ...
    0.6359 0.6601 0.6843 0.7087 0.7331 0.7575 0.7821 0.8043 0.8272 0.8503 ...
    0.8718 0.8941 0.9140 0.9336 0.9522 0.9695 0.9849 0.9979 1.0091 1.0201 ...
    1.0338 1.0496 1.0677 1.0874 1.1079 1.1314 1.1526 1.1784 1.1961 1.2140 ...
    1.2391 1.2486 1.2553 1.2639 1.2761 1.2907 1.2781 1.2610 1.2411 1.2187 ...
    1.1933 1.1650 1.1339 1.1009 1.0677 1.0377];
out.CM = [ ...
    -0.0237 -0.0496 -0.0485 -0.0471 -0.0453 -0.0432 -0.0410 -0.0389 -0.0369 -0.0350 ...
    -0.0332 -0.0320 -0.0308 -0.0297 -0.0285 -0.0277 -0.0267 -0.0260 -0.0253 -0.0246 ...
    -0.0241 -0.0235 -0.0229 -0.0225 -0.0221 -0.0217 -0.0222 -0.0259 -0.0291 -0.0304 ...
    -0.0302 -0.0322 -0.0429 -0.0503 -0.0539 -0.0577 -0.0633 -0.0654 -0.0672 -0.0685 ...
    -0.0695 -0.0699 -0.0698 -0.0689 -0.0679 -0.0668 -0.0656 -0.0643 -0.0631 -0.0618 ...
    -0.0607 -0.0595 -0.0584 -0.0573 -0.0562 -0.0551 -0.0540 -0.0525 -0.0511 -0.0498 ...
    -0.0484 -0.0470 -0.0453 -0.0436 -0.0418 -0.0398 -0.0376 -0.0352 -0.0327 -0.0302 ...
    -0.0281 -0.0264 -0.0250 -0.0240 -0.0230 -0.0225 -0.0216 -0.0217 -0.0203 -0.0192 ...
    -0.0194 -0.0172 -0.0148 -0.0128 -0.0115 -0.0112 -0.0072 -0.0031 0.0004 0.0029 ...
    0.0044 0.0045 0.0032 0.0004 -0.0035 -0.0080];
end

function out = localTailXfoilData()
out = struct();
out.alpha_deg = [ ...
    -12.75 -12.50 -12.25 -12.00 -11.75 -11.50 -11.25 -11.00 -10.75 -10.50 ...
    -10.25 -10.00 -9.75 -9.50 -9.25 -9.00 -8.75 -8.50 -8.25 -8.00 ...
    -7.75 -7.50 -7.25 -7.00 -6.75 -6.50 -6.25 -6.00 -5.75 -5.50 ...
    -5.25 -5.00 -4.75 -4.50 -4.25 -4.00 -3.75 -3.50 -3.25 -3.00 ...
    -2.75 -2.50 -2.25 -2.00 -1.75 -1.50 -1.25 -1.00 -0.75 -0.50 ...
    -0.25 0.00 0.25 0.50 0.75 1.00 1.25 1.50 1.75 2.00 ...
    2.25 2.50 2.75 3.00 3.25 3.50 3.75 4.00 4.25 4.50 ...
    4.75 5.00 5.25 5.50 5.75 6.00 6.25 6.50 6.75 7.00 ...
    7.25 7.50 7.75 8.00 8.25 8.50 8.75 9.00 9.25 9.50 ...
    9.75 10.00 10.25 10.50 10.75 11.00 11.25 11.50 11.75];
out.CL = [ ...
    -0.5002 -0.5184 -0.7724 -0.7713 -0.7896 -0.8377 -0.9695 -0.9842 -0.9915 -0.9911 ...
    -0.9765 -0.9659 -0.9597 -0.9523 -0.9430 -0.9245 -0.9056 -0.8883 -0.8673 -0.8471 ...
    -0.8256 -0.8050 -0.7836 -0.7631 -0.7430 -0.7235 -0.7047 -0.6862 -0.6682 -0.6504 ...
    -0.6324 -0.6141 -0.5953 -0.5761 -0.5565 -0.5366 -0.5162 -0.4957 -0.4751 -0.4545 ...
    -0.4340 -0.4137 -0.3935 -0.3737 -0.3539 -0.3320 -0.3021 -0.2578 -0.2041 -0.1345 ...
    -0.0651 0.0000 0.0651 0.1344 0.2040 0.2578 0.3020 0.3320 0.3538 0.3737 ...
    0.3934 0.4136 0.4339 0.4544 0.4750 0.4957 0.5162 0.5365 0.5565 0.5761 ...
    0.5953 0.6141 0.6323 0.6503 0.6681 0.6862 0.7046 0.7235 0.7430 0.7631 ...
    0.7836 0.8050 0.8256 0.8471 0.8673 0.8884 0.9057 0.9246 0.9431 0.9524 ...
    0.9598 0.9661 0.9768 0.9913 0.9917 0.9845 0.9697 0.8378 0.7898];
out.CM = [ ...
    -0.0001 -0.0027 -0.0153 -0.0162 -0.0207 -0.0287 -0.0300 -0.0278 -0.0258 -0.0241 ...
    -0.0233 -0.0219 -0.0199 -0.0177 -0.0153 -0.0145 -0.0134 -0.0117 -0.0109 -0.0096 ...
    -0.0086 -0.0074 -0.0064 -0.0051 -0.0038 -0.0023 -0.0008 0.0008 0.0025 0.0042 ...
    0.0060 0.0077 0.0095 0.0112 0.0129 0.0145 0.0162 0.0179 0.0196 0.0214 ...
    0.0232 0.0252 0.0272 0.0294 0.0317 0.0335 0.0338 0.0309 0.0259 0.0174 ...
    0.0086 0.0000 -0.0086 -0.0174 -0.0258 -0.0309 -0.0338 -0.0335 -0.0317 -0.0294 ...
    -0.0272 -0.0252 -0.0232 -0.0214 -0.0196 -0.0179 -0.0162 -0.0145 -0.0128 -0.0111 ...
    -0.0094 -0.0077 -0.0060 -0.0042 -0.0025 -0.0008 0.0008 0.0023 0.0038 0.0051 ...
    0.0064 0.0074 0.0086 0.0096 0.0109 0.0117 0.0134 0.0145 0.0153 0.0176 ...
    0.0198 0.0219 0.0233 0.0241 0.0257 0.0278 0.0300 0.0285 0.0204];
end
