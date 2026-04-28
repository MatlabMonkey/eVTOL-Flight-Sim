function [targetTable, summary] = Build_NewMap_StraightLineCorridorTargets(userOptions)
%BUILD_NEWMAP_STRAIGHTLINECORRIDORTARGETS Build straight-line corridor targets.
% The corridor is centered on a neutral straight line between hover and
% cruise in the Vinf/tilt map, then widened by requested tilt offsets.

if nargin < 1 || ~isstruct(userOptions)
    userOptions = struct();
end

rootDir = localRootDir();
opts = localOverlayStruct(localDefaultOptions(rootDir), userOptions);
attemptData = localLoadAttemptData(opts.database_file);

rows = repmat(localRowTemplate(), 0, 1);
rawCount = 0;
skippedExistingCount = 0;
seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');

for vinf = opts.vinf_grid_mps(:).'
    centerTilt = localStraightLineTilt(vinf, opts);
    for offset = opts.tilt_offsets_deg(:).'
        tilt = localRoundToStep(centerTilt + offset, opts.tilt_step_deg);
        for alpha = opts.alpha_grid_deg(:).'
            rawCount = rawCount + 1;
            if ~localWithinBounds(vinf, tilt, alpha, opts)
                continue;
            end

            key = localTargetKey(vinf, tilt, alpha);
            if isKey(seen, key)
                continue;
            end
            if opts.skip_existing_targets && localHasExistingAttempt(vinf, tilt, alpha, attemptData, opts)
                skippedExistingCount = skippedExistingCount + 1;
                continue;
            end

            seen(key) = true;
            row = localMakeRow(vinf, tilt, alpha, centerTilt, opts);
            rows(end + 1, 1) = row; %#ok<AGROW>
        end
    end
end

targetTable = localRowsToTable(rows);
if ~isempty(targetTable)
    targetTable = sortrows(targetTable, 'sort_key', 'ascend');
    if isfinite(opts.max_targets)
        targetTable = targetTable(1:min(height(targetTable), opts.max_targets), :);
    end
end

if strlength(string(opts.preview_csv)) > 0
    previewDir = fileparts(char(opts.preview_csv));
    if exist(previewDir, 'dir') ~= 7
        mkdir(previewDir);
    end
    writetable(targetTable, char(opts.preview_csv));
end

summary = struct();
summary.database_file = string(opts.database_file);
summary.raw_target_count = rawCount;
summary.skipped_existing_count = skippedExistingCount;
summary.target_rows = height(targetTable);
summary.vinf_grid_mps = opts.vinf_grid_mps;
summary.tilt_offsets_deg = opts.tilt_offsets_deg;
summary.alpha_grid_deg = opts.alpha_grid_deg;
summary.preview_csv = string(opts.preview_csv);
end

function row = localMakeRow(vinf, tilt, alpha, centerTilt, opts)
row = localRowTemplate();
row.name = sprintf('StraightCorridor_Tilt%s_V%s_A%s', ...
    localValueLabel(tilt), localValueLabel(vinf), localValueLabel(alpha));
row.tilt_deg = tilt;
row.vinf_mps = vinf;
row.alpha_target_deg = alpha;
row.center_tilt_deg = centerTilt;
row.front_guide_rpm = localInterpGuide(vinf, opts.front_guide_vinf_mps, opts.front_guide_rpm);
row.rear_guide_rpm = localInterpGuide(vinf, opts.rear_guide_vinf_mps, opts.rear_guide_rpm);
row.sort_key = vinf + 0.04 * abs(tilt - centerTilt) + 0.01 * abs(alpha);
end

function attemptData = localLoadAttemptData(databaseFile)
attemptData = struct('vinf', [], 'tilt', [], 'alpha', []);
if exist(databaseFile, 'file') ~= 2
    return;
end

tbl = readtable(databaseFile, 'TextType', 'string');
attemptData.vinf = localNumericColumn(tbl, 'vinf_mps', NaN);
attemptData.tilt = localNumericColumn(tbl, 'tilt_deg', NaN);
targetAlpha = localNumericColumn(tbl, 'alpha_target_deg', NaN);
actualAlpha = localNumericColumn(tbl, 'alpha_deg', NaN);
targetAlpha(~isfinite(targetAlpha)) = actualAlpha(~isfinite(targetAlpha));
targetAlpha(~isfinite(targetAlpha)) = 0.0;
attemptData.alpha = targetAlpha;
end

function tf = localHasExistingAttempt(vinf, tilt, alpha, attemptData, opts)
if isempty(attemptData.vinf)
    tf = false;
    return;
end
mask = abs(attemptData.vinf - vinf) <= opts.existing_vinf_tol_mps + 1e-9 & ...
    abs(attemptData.tilt - tilt) <= opts.existing_tilt_tol_deg + 1e-9 & ...
    abs(attemptData.alpha - alpha) <= opts.existing_alpha_tol_deg + 1e-9;
tf = any(mask);
end

function tf = localWithinBounds(vinf, tilt, alpha, opts)
tf = vinf >= opts.vinf_bounds_mps(1) - 1e-9 && ...
    vinf <= opts.vinf_bounds_mps(2) + 1e-9 && ...
    tilt >= opts.tilt_bounds_deg(1) - 1e-9 && ...
    tilt <= opts.tilt_bounds_deg(2) + 1e-9 && ...
    alpha >= opts.alpha_bounds_deg(1) - 1e-9 && ...
    alpha <= opts.alpha_bounds_deg(2) + 1e-9;
end

function tilt = localStraightLineTilt(vinf, opts)
tilt = opts.cruise_tilt_deg * vinf / max(opts.cruise_vinf_mps, eps);
end

function opts = localDefaultOptions(rootDir)
dbPaths = TrimDB_Paths(rootDir);
databaseName = "trim_vinf_alpha_v1";
opts = struct();
opts.root_dir = rootDir;
opts.database_name = databaseName;
opts.database_file = fullfile(dbPaths.database_dir, char(databaseName), 'trim_attempts.csv');
opts.preview_csv = "";
opts.max_targets = inf;
opts.skip_existing_targets = true;

opts.vinf_grid_mps = 0:2.5:70;
opts.vinf_bounds_mps = [0 70];
opts.tilt_bounds_deg = [0 95];
opts.alpha_bounds_deg = [0 10];
opts.alpha_grid_deg = [2.5 5 7.5 10];
opts.tilt_offsets_deg = [-10 0 10];
opts.tilt_step_deg = 2.5;
opts.cruise_vinf_mps = 70;
opts.cruise_tilt_deg = 90;
opts.existing_vinf_tol_mps = 0.25;
opts.existing_tilt_tol_deg = 0.25;
opts.existing_alpha_tol_deg = 0.20;

opts.front_guide_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.front_guide_rpm = [1865 1850 1750 1550 1250 1000 875 820 780];
opts.rear_guide_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.rear_guide_rpm = [1755 1650 1400 1100 800 650 550 450 350];
end

function tableOut = localRowsToTable(rows)
if isempty(rows)
    tableOut = struct2table(repmat(localRowTemplate(), 0, 1));
else
    tableOut = struct2table(rows);
end
end

function row = localRowTemplate()
row = struct( ...
    'name', "", ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'alpha_target_deg', NaN, ...
    'center_tilt_deg', NaN, ...
    'front_guide_rpm', NaN, ...
    'rear_guide_rpm', NaN, ...
    'sort_key', inf);
end

function column = localNumericColumn(tbl, varName, defaultValue)
column = repmat(defaultValue, height(tbl), 1);
if ~ismember(varName, tbl.Properties.VariableNames)
    return;
end
raw = tbl.(varName);
if isnumeric(raw) || islogical(raw)
    column = double(raw);
else
    column = str2double(string(raw));
end
end

function value = localInterpGuide(x, xk, yk)
value = interp1(xk, yk, x, 'pchip', 'extrap');
end

function value = localRoundToStep(value, step)
if step <= 0
    return;
end
value = step * round(value / step);
end

function key = localTargetKey(vinf, tilt, alpha)
key = sprintf('V%s_T%s_A%s', localValueLabel(vinf), localValueLabel(tilt), localValueLabel(alpha));
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end

function out = localOverlayStruct(defaults, overrides)
out = defaults;
if ~isstruct(overrides)
    return;
end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    out.(fields{i}) = overrides.(fields{i});
end
end

function rootDir = localRootDir()
scriptPath = mfilename('fullpath');
rootDir = fileparts(fileparts(fileparts(scriptPath)));
end
