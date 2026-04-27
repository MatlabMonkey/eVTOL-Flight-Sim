%% Build final wing and tail aerodynamic coefficient tables
% Produces final lookup-table structs for:
%   wingPolar.alpha_deg / alpha_rad / CL / CD / Cm
%   tailPolar.alpha_deg / alpha_rad / CL / CD / Cm
%
% Final source choices:
% - CL/CD: Airfoil360 wind-tunnel data at Re = 100k
% - Cm: low-alpha XFOIL data blended into a high-alpha flat-plate-style
%       local section moment model

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
workbookPath = fullfile(repoRoot, 'Airfoil360_wind_tunnel_data_v2022.xlsx');
outputDir = fullfile(repoRoot, 'databases', 'aero_polars');
outputMat = fullfile(outputDir, 'final_airfoil_polar_tables.mat');

if exist(workbookPath, 'file') ~= 2
    error('build_final_airfoil_polar_tables:MissingWorkbook', ...
        'Could not find %s', workbookPath);
end

if ~isfolder(outputDir)
    mkdir(outputDir);
end

wing360 = localReadAirfoil360Sheet(workbookPath, 'NACA 2412 Re=100K');
tail360 = localReadAirfoil360Sheet(workbookPath, 'NACA 0012 Re=100K');

wingXfoil = localWingXfoilData();
tailXfoil = localTailXfoilData();

alphaDeg = (-180:1:180).';
alphaRad = deg2rad(alphaDeg);

wingBlend = localBuildBlendedCmTable(alphaDeg, wing360, wingXfoil, 12, 20);
tailBlend = localBuildBlendedCmTable(alphaDeg, tail360, tailXfoil, 10, 18);

wingPolar = struct();
wingPolar.name = 'Wing';
wingPolar.airfoil = 'NACA 2412';
wingPolar.reynolds = 100000;
wingPolar.alpha_deg = alphaDeg;
wingPolar.alpha_rad = alphaRad;
wingPolar.CL = localPeriodicInterp(alphaDeg, wing360.alpha_deg_signed, wing360.CL);
wingPolar.CD = localPeriodicInterp(alphaDeg, wing360.alpha_deg_signed, wing360.CD);
wingPolar.Cm = wingBlend.Cm;
wingPolar.meta = struct( ...
    'CLCDSource', 'Airfoil360 wind-tunnel data', ...
    'CmLowAlphaSource', 'XFOIL pasted polar', ...
    'CmHighAlphaSource', 'Flat-plate local quarter-chord moment', ...
    'CmBlendStartDeg', 12, ...
    'CmBlendEndDeg', 20);

tailPolar = struct();
tailPolar.name = 'Tail';
tailPolar.airfoil = 'NACA 0012';
tailPolar.reynolds = 100000;
tailPolar.alpha_deg = alphaDeg;
tailPolar.alpha_rad = alphaRad;
tailPolar.CL = localPeriodicInterp(alphaDeg, tail360.alpha_deg_signed, tail360.CL);
tailPolar.CD = localPeriodicInterp(alphaDeg, tail360.alpha_deg_signed, tail360.CD);
tailPolar.Cm = tailBlend.Cm;
tailPolar.meta = struct( ...
    'CLCDSource', 'Airfoil360 wind-tunnel data', ...
    'CmLowAlphaSource', 'XFOIL pasted polar', ...
    'CmHighAlphaSource', 'Flat-plate local quarter-chord moment', ...
    'CmBlendStartDeg', 10, ...
    'CmBlendEndDeg', 18);

assignin('base', 'wingPolar', wingPolar);
assignin('base', 'tailPolar', tailPolar);
save(outputMat, 'wingPolar', 'tailPolar');

fprintf('Built final airfoil polar tables.\n');
fprintf('Saved MAT file: %s\n', outputMat);
fprintf('Workspace variables: wingPolar, tailPolar\n');

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

function yq = localPeriodicInterp(xq, xData, yData)
xqWrapped = mod(xq + 180, 360) - 180;
xAug = [xData(:) - 360; xData(:); xData(:) + 360];
yAug = [yData(:); yData(:); yData(:)];
[xAug, sortIdx] = sort(xAug);
yAug = yAug(sortIdx);
yq = interp1(xAug, yAug, xqWrapped, 'pchip');
end

function out = localBuildBlendedCmTable(alphaDeg, airfoil360, xfoil, alphaBlendStartDeg, alphaBlendEndDeg)
alphaDeg = alphaDeg(:);
alphaRad = deg2rad(alphaDeg);
absAlpha = abs(alphaDeg);

CL = localPeriodicInterp(alphaDeg, airfoil360.alpha_deg_signed, airfoil360.CL);
CD = localPeriodicInterp(alphaDeg, airfoil360.alpha_deg_signed, airfoil360.CD);
CN = CL .* cos(alphaRad) + CD .* sin(alphaRad);
CmFlat = -0.25 .* CN;

alphaX = xfoil.alpha_deg(:);
cmX = xfoil.CM(:);
CmXfoil = interp1(alphaX, cmX, min(max(alphaDeg, min(alphaX)), max(alphaX)), 'pchip');

Cm = CmFlat;
insideMask = absAlpha <= alphaBlendStartDeg;
Cm(insideMask) = CmXfoil(insideMask);

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

endVal = 0.5 * (Cm(alphaDeg == -180) + Cm(alphaDeg == 180));
Cm(alphaDeg == -180) = endVal;
Cm(alphaDeg == 180) = endVal;

out = struct('Cm', Cm, 'CmXfoilExt', CmXfoil, 'CmFlat', CmFlat);
end

function y = localHermiteSegment(x, x0, y0, m0, x1, y1, m1)
t = (x - x0) ./ (x1 - x0);
h00 = 2 .* t.^3 - 3 .* t.^2 + 1;
h10 = t.^3 - 2 .* t.^2 + t;
h01 = -2 .* t.^3 + 3 .* t.^2;
h11 = t.^3 - t.^2;
y = h00 .* y0 + h10 .* (x1 - x0) .* m0 + h01 .* y1 + h11 .* (x1 - x0) .* m1;
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
    13.25 13.50 13.75 14.00 14.25 14.50]';
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
    0.0044 0.0045 0.0032 0.0004 -0.0035 -0.0080]';
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
    9.75 10.00 10.25 10.50 10.75 11.00 11.25 11.50 11.75]';
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
    0.0198 0.0219 0.0233 0.0241 0.0257 0.0278 0.0300 0.0285 0.0204]';
end
