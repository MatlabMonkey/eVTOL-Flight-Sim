function [targetTable, summary] = Build_NewMap_AdaptiveFrontierTargets(userOptions)
%BUILD_NEWMAP_ADAPTIVEFRONTIERTARGETS Propose next trim-map targets from DB.
% The intent is to expand from exact/acceptable Vinf/tilt/alpha points rather
% than brute-force broad rectangles that are already known to fail densely.

if nargin < 1 || ~isstruct(userOptions)
    userOptions = struct();
end

rootDir = localRootDir();
opts = localOverlayStruct(localDefaultOptions(rootDir), userOptions);
attempts = localLoadAttempts(opts.database_file);
data = localAttemptData(attempts);

anchorMask = data.good;
anchors = attempts(anchorMask, :);
anchorData = localAttemptData(anchors);
if isempty(anchors)
    error('Build_NewMap_AdaptiveFrontierTargets:NoAnchors', ...
        'No exact/acceptable anchors found in %s.', opts.database_file);
end

candidateRows = localBuildCandidates(anchorData, data, opts);
targetTable = localRowsToTable(candidateRows);

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
summary.attempt_rows = height(attempts);
summary.anchor_rows = height(anchors);
summary.candidate_rows = height(targetTable);
summary.max_targets = opts.max_targets;
summary.vinf_bounds_mps = opts.vinf_bounds_mps;
summary.tilt_bounds_deg = opts.tilt_bounds_deg;
summary.alpha_bounds_deg = opts.alpha_bounds_deg;
summary.preview_csv = string(opts.preview_csv);
end

function rows = localBuildCandidates(anchorData, attemptData, opts)
rows = repmat(localRowTemplate(), 0, 1);
seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');

for iAnchor = 1:numel(anchorData.vinf)
    anchorV = localRoundToStep(anchorData.vinf(iAnchor), opts.vinf_step_mps);
    anchorTilt = localRoundToStep(anchorData.tilt(iAnchor), opts.tilt_step_deg);
    anchorAlpha = localRoundToStep(anchorData.alpha(iAnchor), opts.alpha_step_deg);

    for dv = opts.vinf_offsets_mps(:).'
        for dt = opts.tilt_offsets_deg(:).'
            for da = opts.alpha_offsets_deg(:).'
                vinf = localRoundToStep(anchorV + dv, opts.vinf_step_mps);
                tilt = localRoundToStep(anchorTilt + dt, opts.tilt_step_deg);
                alpha = localRoundToStep(anchorAlpha + da, opts.alpha_step_deg);

                if ~localWithinBounds(vinf, tilt, alpha, opts)
                    continue;
                end

                guideTilt = localInterpGuide(vinf, opts.guide_vinf_mps, opts.guide_tilt_deg);
                if opts.enforce_guide_corridor && abs(tilt - guideTilt) > opts.max_guide_tilt_error_deg
                    continue;
                end

                key = localTargetKey(vinf, tilt, alpha);
                if isKey(seen, key)
                    continue;
                end

                [attemptCount, failedCount, goodCount] = localNeighborhoodCounts(vinf, tilt, alpha, attemptData, opts);
                if opts.skip_existing_good_targets && localHasExactGoodTarget(vinf, tilt, alpha, attemptData, opts)
                    continue;
                end
                if opts.skip_dense_failures && failedCount >= opts.max_local_failed_attempts && goodCount == 0
                    continue;
                end

                seen(key) = true;
                row = localRowTemplate();
                row.name = sprintf('AdaptiveFrontier_Tilt%s_V%s_A%s', ...
                    localValueLabel(tilt), localValueLabel(vinf), localValueLabel(alpha));
                row.tilt_deg = tilt;
                row.vinf_mps = vinf;
                row.alpha_target_deg = alpha;
                row.center_tilt_deg = guideTilt;
                row.front_guide_rpm = localInterpGuide(vinf, opts.front_guide_vinf_mps, opts.front_guide_rpm);
                row.rear_guide_rpm = localInterpGuide(vinf, opts.rear_guide_vinf_mps, opts.rear_guide_rpm);
                row.nearest_success_distance = localNearestSuccessDistance(vinf, tilt, alpha, anchorData, opts);
                row.local_attempt_count = attemptCount;
                row.local_failed_count = failedCount;
                row.sort_key = row.nearest_success_distance + ...
                    0.20 * abs(tilt - guideTilt) + ...
                    1.50 * failedCount + ...
                    0.10 * attemptCount + ...
                    0.01 * abs(alpha);
                rows(end + 1, 1) = row; %#ok<AGROW>
            end
        end
    end
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
opts.max_targets = 120;

opts.vinf_bounds_mps = [0 75];
opts.tilt_bounds_deg = [0 95];
opts.alpha_bounds_deg = [-5 17.5];
opts.vinf_step_mps = 2.5;
opts.tilt_step_deg = 2.5;
opts.alpha_step_deg = 2.5;

opts.vinf_offsets_mps = [-5 -2.5 0 2.5 5];
opts.tilt_offsets_deg = [-5 -2.5 0 2.5 5];
opts.alpha_offsets_deg = [-2.5 0 2.5];

opts.neighborhood_vinf_radius_mps = 2.5;
opts.neighborhood_tilt_radius_deg = 2.5;
opts.neighborhood_alpha_radius_deg = 2.5;
opts.exact_vinf_tol_mps = 0.25;
opts.exact_tilt_tol_deg = 0.25;
opts.exact_alpha_tol_deg = 0.25;
opts.max_local_failed_attempts = 4;
opts.skip_existing_good_targets = true;
opts.skip_dense_failures = true;

opts.enforce_guide_corridor = true;
opts.max_guide_tilt_error_deg = 22.5;
opts.guide_vinf_mps = [0 10 20 30 40 50 60 70];
opts.guide_tilt_deg = [0 15 35 55 70 80 85 90];
opts.front_guide_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.front_guide_rpm = [1865 1850 1750 1550 1250 1000 875 820 780];
opts.rear_guide_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.rear_guide_rpm = [1755 1650 1400 1100 800 650 550 450 350];

opts.distance_vinf_scale_mps = 5.0;
opts.distance_tilt_scale_deg = 5.0;
opts.distance_alpha_scale_deg = 5.0;
end

function attempts = localLoadAttempts(filename)
if exist(filename, 'file') ~= 2
    error('Build_NewMap_AdaptiveFrontierTargets:MissingDatabase', ...
        'Trim-attempt database does not exist: %s', filename);
end
attempts = readtable(filename, 'TextType', 'string');
end

function data = localAttemptData(tbl)
data = struct();
data.vinf = localNumericColumn(tbl, 'vinf_mps', NaN);
data.tilt = localNumericColumn(tbl, 'tilt_deg', NaN);
targetAlpha = localNumericColumn(tbl, 'alpha_target_deg', NaN);
actualAlpha = localNumericColumn(tbl, 'alpha_deg', NaN);
targetAlpha(~isfinite(targetAlpha)) = actualAlpha(~isfinite(targetAlpha));
targetAlpha(~isfinite(targetAlpha)) = 0.0;
data.alpha = targetAlpha;
success = localLogicalColumn(tbl, 'success');
acceptable = localLogicalColumn(tbl, 'acceptable');
data.good = success | acceptable;
data.failed = ~data.good;
end

function [attemptCount, failedCount, goodCount] = localNeighborhoodCounts(vinf, tilt, alpha, data, opts)
nearMask = abs(data.vinf - vinf) <= opts.neighborhood_vinf_radius_mps + 1e-9 & ...
    abs(data.tilt - tilt) <= opts.neighborhood_tilt_radius_deg + 1e-9 & ...
    abs(data.alpha - alpha) <= opts.neighborhood_alpha_radius_deg + 1e-9;
attemptCount = nnz(nearMask);
failedCount = nnz(nearMask & data.failed);
goodCount = nnz(nearMask & data.good);
end

function tf = localHasExactGoodTarget(vinf, tilt, alpha, data, opts)
exactMask = abs(data.vinf - vinf) <= opts.exact_vinf_tol_mps + 1e-9 & ...
    abs(data.tilt - tilt) <= opts.exact_tilt_tol_deg + 1e-9 & ...
    abs(data.alpha - alpha) <= opts.exact_alpha_tol_deg + 1e-9;
tf = any(exactMask & data.good);
end

function distance = localNearestSuccessDistance(vinf, tilt, alpha, anchorData, opts)
dv = (anchorData.vinf - vinf) / opts.distance_vinf_scale_mps;
dt = (anchorData.tilt - tilt) / opts.distance_tilt_scale_deg;
da = (anchorData.alpha - alpha) / opts.distance_alpha_scale_deg;
distance = min(sqrt(dv.^2 + dt.^2 + da.^2));
end

function tf = localWithinBounds(vinf, tilt, alpha, opts)
tf = vinf >= opts.vinf_bounds_mps(1) - 1e-9 && ...
    vinf <= opts.vinf_bounds_mps(2) + 1e-9 && ...
    tilt >= opts.tilt_bounds_deg(1) - 1e-9 && ...
    tilt <= opts.tilt_bounds_deg(2) + 1e-9 && ...
    alpha >= opts.alpha_bounds_deg(1) - 1e-9 && ...
    alpha <= opts.alpha_bounds_deg(2) + 1e-9;
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
    'sort_key', inf, ...
    'nearest_success_distance', inf, ...
    'local_attempt_count', 0, ...
    'local_failed_count', 0);
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

function column = localLogicalColumn(tbl, varName)
column = false(height(tbl), 1);
if ~ismember(varName, tbl.Properties.VariableNames)
    return;
end
raw = tbl.(varName);
if islogical(raw)
    column = raw;
elseif isnumeric(raw)
    column = raw ~= 0;
else
    value = lower(strtrim(string(raw)));
    column = value == "1" | value == "true" | value == "yes";
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
