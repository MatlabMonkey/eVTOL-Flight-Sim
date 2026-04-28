function [targetTable, summary] = Build_NewMap_FullAlphaRectangleTargets(userOptions)
%BUILD_NEWMAP_FULLALPHARECTANGLETARGETS Build full V/tilt/alpha rectangle.
% Generates all requested grid points and drops targets already present in
% the trim-attempt DB, including previous failed attempts.

if nargin < 1 || ~isstruct(userOptions)
    userOptions = struct();
end

rootDir = localRootDir();
opts = localOverlayStruct(localDefaultOptions(rootDir), userOptions);
existingKeys = localLoadExistingTargetKeys(opts.database_file, opts);

rows = repmat(localRowTemplate(), 0, 1);
rawCount = 0;
skippedExistingCount = 0;
seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');

for vinf = opts.vinf_grid_mps(:).'
    for tilt = opts.tilt_grid_deg(:).'
        for alpha = opts.alpha_grid_deg(:).'
            rawCount = rawCount + 1;
            key = localTargetKey(vinf, tilt, alpha);
            if opts.skip_existing_targets && isKey(existingKeys, key)
                skippedExistingCount = skippedExistingCount + 1;
                continue;
            end
            if isKey(seen, key)
                continue;
            end
            seen(key) = true;
            row = localMakeRow(vinf, tilt, alpha, opts);
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
summary.tilt_grid_deg = opts.tilt_grid_deg;
summary.alpha_grid_deg = opts.alpha_grid_deg;
summary.preview_csv = string(opts.preview_csv);
end

function row = localMakeRow(vinf, tilt, alpha, opts)
guideTilt = localInterpGuide(vinf, opts.guide_vinf_mps, opts.guide_tilt_deg);
row = localRowTemplate();
row.name = sprintf('FullAlphaRect_Tilt%s_V%s_A%s', ...
    localValueLabel(tilt), localValueLabel(vinf), localValueLabel(alpha));
row.tilt_deg = tilt;
row.vinf_mps = vinf;
row.alpha_target_deg = alpha;
row.center_tilt_deg = guideTilt;
row.front_guide_rpm = localInterpGuide(vinf, opts.front_guide_vinf_mps, opts.front_guide_rpm);
row.rear_guide_rpm = localInterpGuide(vinf, opts.rear_guide_vinf_mps, opts.rear_guide_rpm);
row.sort_key = abs(tilt - guideTilt) + 0.02 * vinf + 0.01 * abs(alpha);
end

function keys = localLoadExistingTargetKeys(databaseFile, opts)
keys = containers.Map('KeyType', 'char', 'ValueType', 'logical');
if exist(databaseFile, 'file') ~= 2
    return;
end

tbl = readtable(databaseFile, 'TextType', 'string');
if isempty(tbl) || ~all(ismember({'vinf_mps','tilt_deg'}, tbl.Properties.VariableNames))
    return;
end

vinfValues = localNumericColumn(tbl, 'vinf_mps', NaN);
tiltValues = localNumericColumn(tbl, 'tilt_deg', NaN);
alphaValues = localNumericColumn(tbl, 'alpha_target_deg', NaN);
if any(~isfinite(alphaValues))
    actualAlpha = localNumericColumn(tbl, 'alpha_deg', NaN);
    alphaValues(~isfinite(alphaValues)) = actualAlpha(~isfinite(alphaValues));
end
alphaValues(~isfinite(alphaValues)) = 0.0;

for i = 1:height(tbl)
    if ~isfinite(vinfValues(i)) || ~isfinite(tiltValues(i)) || ~isfinite(alphaValues(i))
        continue;
    end
    vinf = localRoundToStep(vinfValues(i), opts.vinf_existing_step_mps);
    tilt = localRoundToStep(tiltValues(i), opts.tilt_existing_step_deg);
    alpha = localRoundToStep(alphaValues(i), opts.alpha_existing_step_deg);
    keys(localTargetKey(vinf, tilt, alpha)) = true;
end
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

opts.vinf_grid_mps = 0:5:75;
opts.tilt_grid_deg = 0:10:90;
opts.alpha_grid_deg = [0 5 10];
opts.vinf_existing_step_mps = 5.0;
opts.tilt_existing_step_deg = 10.0;
opts.alpha_existing_step_deg = 5.0;

opts.guide_vinf_mps = [0 10 20 30 40 50 60 70];
opts.guide_tilt_deg = [0 15 35 55 70 80 85 90];
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
