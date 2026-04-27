function [targetTable, summary] = Build_NewMap_RectangleTargets(userOptions)
%BUILD_NEWMAP_RECTANGLETARGETS Build a coarse rectangular trim target table.
% This is for coverage/visualization, not the main exploitative search.

if nargin < 1 || ~isstruct(userOptions)
    userOptions = struct();
end

rootDir = localRootDir();
opts = localOverlayStruct(localDefaultOptions(rootDir), userOptions);
rows = repmat(localRowTemplate(), 0, 1);
seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');

rows = localAppendPrimaryRectangle(rows, seen, opts);
rows = localAppendExtraAlphaSubset(rows, seen, opts);
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
summary.target_rows = height(targetTable);
summary.vinf_grid_mps = opts.vinf_grid_mps;
summary.tilt_grid_deg = opts.tilt_grid_deg;
summary.extra_alpha_vinf_bounds_mps = opts.extra_alpha_vinf_bounds_mps;
summary.extra_alpha_tilt_bounds_deg = opts.extra_alpha_tilt_bounds_deg;
summary.preview_csv = string(opts.preview_csv);
end

function rows = localAppendPrimaryRectangle(rows, seen, opts)
for vinf = opts.vinf_grid_mps(:).'
    alpha = localScheduledAlpha(vinf, opts);
    for tilt = opts.tilt_grid_deg(:).'
        [rows, seen] = localAppendRow(rows, seen, opts, vinf, tilt, alpha, "rect");
    end
end
end

function rows = localAppendExtraAlphaSubset(rows, seen, opts)
for vinf = opts.vinf_grid_mps(:).'
    if vinf < opts.extra_alpha_vinf_bounds_mps(1) - 1e-9 || ...
            vinf > opts.extra_alpha_vinf_bounds_mps(2) + 1e-9
        continue;
    end
    baseAlpha = localScheduledAlpha(vinf, opts);
    for tilt = opts.tilt_grid_deg(:).'
        if tilt < opts.extra_alpha_tilt_bounds_deg(1) - 1e-9 || ...
                tilt > opts.extra_alpha_tilt_bounds_deg(2) + 1e-9
            continue;
        end
        for offset = opts.extra_alpha_offsets_deg(:).'
            alpha = min(opts.alpha_bounds_deg(2), max(opts.alpha_bounds_deg(1), baseAlpha + offset));
            [rows, seen] = localAppendRow(rows, seen, opts, vinf, tilt, alpha, "rectExtraAlpha");
        end
    end
end
end

function [rows, seen] = localAppendRow(rows, seen, opts, vinf, tilt, alpha, sourceName)
if vinf < opts.vinf_bounds_mps(1) - 1e-9 || vinf > opts.vinf_bounds_mps(2) + 1e-9 || ...
        tilt < opts.tilt_bounds_deg(1) - 1e-9 || tilt > opts.tilt_bounds_deg(2) + 1e-9 || ...
        alpha < opts.alpha_bounds_deg(1) - 1e-9 || alpha > opts.alpha_bounds_deg(2) + 1e-9
    return;
end

key = localTargetKey(vinf, tilt, alpha);
if isKey(seen, key)
    return;
end
seen = localMarkSeen(seen, key);

guideTilt = localInterpGuide(vinf, opts.guide_vinf_mps, opts.guide_tilt_deg);
row = localRowTemplate();
row.name = sprintf('%s_Tilt%s_V%s_A%s', ...
    char(sourceName), localValueLabel(tilt), localValueLabel(vinf), localValueLabel(alpha));
row.tilt_deg = tilt;
row.vinf_mps = vinf;
row.alpha_target_deg = alpha;
row.center_tilt_deg = guideTilt;
row.front_guide_rpm = localInterpGuide(vinf, opts.front_guide_vinf_mps, opts.front_guide_rpm);
row.rear_guide_rpm = localInterpGuide(vinf, opts.rear_guide_vinf_mps, opts.rear_guide_rpm);
row.sort_key = abs(tilt - guideTilt) + 0.02 * vinf + 0.01 * abs(alpha);
rows(end + 1, 1) = row;
end

function seen = localMarkSeen(seen, key)
seen(key) = true;
end

function alpha = localScheduledAlpha(vinf, opts)
alpha = localInterpGuide(vinf, opts.alpha_schedule_vinf_mps, opts.alpha_schedule_deg);
alpha = opts.alpha_step_deg * round(alpha / opts.alpha_step_deg);
alpha = min(opts.alpha_bounds_deg(2), max(opts.alpha_bounds_deg(1), alpha));
end

function opts = localDefaultOptions(rootDir)
opts = struct();
opts.root_dir = rootDir;
opts.preview_csv = "";
opts.max_targets = inf;

opts.vinf_grid_mps = 0:5:75;
opts.tilt_grid_deg = 0:10:90;
opts.vinf_bounds_mps = [0 75];
opts.tilt_bounds_deg = [0 95];
opts.alpha_bounds_deg = [-5 17.5];
opts.alpha_step_deg = 2.5;

opts.alpha_schedule_vinf_mps = [0 10 20 30 40 50 60 70 75];
opts.alpha_schedule_deg = [0 5 10 10 7.5 5 2.5 0 0];
opts.extra_alpha_vinf_bounds_mps = [20 55];
opts.extra_alpha_tilt_bounds_deg = [30 80];
opts.extra_alpha_offsets_deg = 5.0;

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

function value = localInterpGuide(x, xk, yk)
value = interp1(xk, yk, x, 'pchip', 'extrap');
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
