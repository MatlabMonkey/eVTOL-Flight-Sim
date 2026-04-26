% Plot_Transition_Reference_Line_Scored_Merged_Map.m
% Plot the combined points from the main reference-line scored sweep and
% the focused mid-band reference-line scored sweep.
%
% Optional workspace overrides before running:
%   transitionReferenceLineMergedPlotInputCsvs = { ...
%       '/abs/path/to/transition_trim_reference_line_scored_latest.csv', ...
%       '/abs/path/to/transition_trim_reference_line_midband_scored_latest.csv'};
%   transitionReferenceLineMergedPlotOptions = struct('show_popup', true);
%
% Outputs left in the base workspace:
%   - transitionReferenceLineMergedPlotDir
%   - transitionReferenceLineMergedPlotSummary

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

defaultCsvs = { ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_midband_scored_latest.csv')};

if exist('transitionReferenceLineMergedPlotInputCsvs', 'var') && ~isempty(transitionReferenceLineMergedPlotInputCsvs)
    inputCsvs = transitionReferenceLineMergedPlotInputCsvs;
else
    inputCsvs = defaultCsvs;
end

plotOptions = struct('show_popup', true);
if exist('transitionReferenceLineMergedPlotOptions', 'var') && isstruct(transitionReferenceLineMergedPlotOptions)
    plotOptions = localMergeStruct(plotOptions, transitionReferenceLineMergedPlotOptions);
end

validCsvs = {};
for i = 1:numel(inputCsvs)
    candidate = char(string(inputCsvs{i}));
    if exist(candidate, 'file') == 2
        validCsvs{end + 1} = candidate; %#ok<AGROW>
    end
end

if isempty(validCsvs)
    error('Plot_Transition_Reference_Line_Scored_Merged_Map:MissingCsvs', ...
        'None of the merged reference-line CSV files were found.');
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionReferenceLineMergedPlotDir = fullfile(root_dir, 'workspace_plots', ...
    ['transition_reference_line_scored_merged_visuals_' timestamp]);
if exist(transitionReferenceLineMergedPlotDir, 'dir') ~= 7
    mkdir(transitionReferenceLineMergedPlotDir);
end

mergedTbl = table();
for i = 1:numel(validCsvs)
    tbl = readtable(validCsvs{i}, 'TextType', 'string');
    if isempty(tbl)
        continue;
    end
    tbl.source_csv = repmat(string(validCsvs{i}), height(tbl), 1);
    mergedTbl = [mergedTbl; tbl]; %#ok<AGROW>
end

if isempty(mergedTbl)
    error('Plot_Transition_Reference_Line_Scored_Merged_Map:EmptyCsvs', ...
        'Merged reference-line CSV set is empty.');
end

summaryTbl = localCollapseBestRows(mergedTbl);

successMask = localAsLogical(summaryTbl.success);
acceptableMask = localAsLogical(summaryTbl.acceptable);
classification = localAsString(summaryTbl.classification);
vinf_mps = summaryTbl.vinf_mps;
tilt_deg = summaryTbl.tilt_deg;

isExact = classification == "exact_trim" | successMask;
isAcceptable = acceptableMask & ~isExact;
isBorderline = classification == "near_trim_borderline" & ~isExact & ~isAcceptable;
isOther = ~(isExact | isAcceptable | isBorderline);

[~, order] = sortrows([vinf_mps, tilt_deg], [1 2]);

referenceKnotVinf = [0 2.5 5 10 20 30 40 50 60 70];
referenceKnotTilt = [0 18 30 44 62 72 80 85 88 90];
referenceVinfDense = linspace(0, 70, 400);
referenceTiltDense = interp1(referenceKnotVinf, referenceKnotTilt, referenceVinfDense, 'pchip');

transitionReferenceLineMergedPlotSummary = struct();
transitionReferenceLineMergedPlotSummary.input_csvs = string(validCsvs);
transitionReferenceLineMergedPlotSummary.output_dir = transitionReferenceLineMergedPlotDir;
transitionReferenceLineMergedPlotSummary.total_input_rows = height(mergedTbl);
transitionReferenceLineMergedPlotSummary.unique_points = height(summaryTbl);
transitionReferenceLineMergedPlotSummary.exact_count = nnz(isExact);
transitionReferenceLineMergedPlotSummary.acceptable_count = nnz(isAcceptable);
transitionReferenceLineMergedPlotSummary.borderline_count = nnz(isBorderline);
transitionReferenceLineMergedPlotSummary.other_count = nnz(isOther);

figVisible = localFigureVisibility(plotOptions.show_popup);

f1 = figure('Color', 'w', 'Position', [100 100 980 700], 'Visible', figVisible);
hold on;
plot(referenceVinfDense, referenceTiltDense, '-', 'Color', [0.45 0.85 0.20], 'LineWidth', 3.0);
plot(vinf_mps(order), tilt_deg(order), '--', 'Color', [0.45 0.75 0.95], 'LineWidth', 1.2);

if any(isOther)
    scatter(vinf_mps(isOther), tilt_deg(isOther), 52, [0.70 0.70 0.70], 'x', 'LineWidth', 1.0);
end
if any(isBorderline)
    scatter(vinf_mps(isBorderline), tilt_deg(isBorderline), 64, [0.95 0.55 0.15], '^', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
if any(isAcceptable)
    scatter(vinf_mps(isAcceptable), tilt_deg(isAcceptable), 66, [0.98 0.82 0.18], 's', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
if any(isExact)
    scatter(vinf_mps(isExact), tilt_deg(isExact), 58, [0.12 0.60 0.18], 'o', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end

scatter(0.0, 0.0, 140, 'd', 'MarkerFaceColor', [0.10 0.35 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
scatter(70.0, 90.0, 140, 'p', 'MarkerFaceColor', [0.90 0.90 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);

xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Merged Reference-Line Transition Points');
legend(localLegendEntries(isOther, isBorderline, isAcceptable, isExact), ...
    'Location', 'northwest');
grid on;
box on;
saveas(f1, fullfile(transitionReferenceLineMergedPlotDir, 'reference_line_scored_merged_classification_map.png'));

f2 = figure('Color', 'w', 'Position', [120 120 980 700], 'Visible', figVisible);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(vinf_mps(order), summaryTbl.front_collective_rpm(order), '-', ...
    'Color', [0.10 0.35 0.90], 'LineWidth', 1.2);
scatter(vinf_mps(isExact), summaryTbl.front_collective_rpm(isExact), 42, [0.12 0.60 0.18], 'filled');
scatter(vinf_mps(isAcceptable), summaryTbl.front_collective_rpm(isAcceptable), 48, [0.98 0.82 0.18], 's', 'filled');
if any(isOther | isBorderline)
    scatter(vinf_mps(isOther | isBorderline), summaryTbl.front_collective_rpm(isOther | isBorderline), ...
        42, [0.70 0.70 0.70], 'x', 'LineWidth', 1.0);
end
ylabel('Front collective (rpm)');
title('Merged Reference-Line Propeller Values');
grid on;
box on;

nexttile;
hold on;
plot(vinf_mps(order), summaryTbl.rear_collective_rpm(order), '-', ...
    'Color', [0.80 0.25 0.20], 'LineWidth', 1.2);
scatter(vinf_mps(isExact), summaryTbl.rear_collective_rpm(isExact), 42, [0.12 0.60 0.18], 'filled');
scatter(vinf_mps(isAcceptable), summaryTbl.rear_collective_rpm(isAcceptable), 48, [0.98 0.82 0.18], 's', 'filled');
if any(isOther | isBorderline)
    scatter(vinf_mps(isOther | isBorderline), summaryTbl.rear_collective_rpm(isOther | isBorderline), ...
        42, [0.70 0.70 0.70], 'x', 'LineWidth', 1.0);
end
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Rear collective (rpm)');
grid on;
box on;
saveas(f2, fullfile(transitionReferenceLineMergedPlotDir, 'reference_line_scored_merged_prop_rpm.png'));

disp(transitionReferenceLineMergedPlotDir);

if ~plotOptions.show_popup
    close(f1);
    close(f2);
end

function out = localCollapseBestRows(tbl)
keys = strings(height(tbl), 1);
for i = 1:height(tbl)
    keys(i) = string(sprintf('%s|%s', localValueLabel(tbl.tilt_deg(i)), localValueLabel(tbl.vinf_mps(i))));
end
tbl.point_key = keys;
uniqueKeys = unique(keys, 'stable');
rows = [];
for iKey = 1:numel(uniqueKeys)
    idx = find(keys == uniqueKeys(iKey));
    bestIdx = idx(1);
    bestRank = localRowRank(tbl(bestIdx, :));
    for j = 2:numel(idx)
        candIdx = idx(j);
        candRank = localRowRank(tbl(candIdx, :));
        if candRank < bestRank || (candRank == bestRank && localRowScore(tbl(candIdx, :)) < localRowScore(tbl(bestIdx, :)))
            bestIdx = candIdx;
            bestRank = candRank;
        end
    end
    rows(end + 1, 1) = bestIdx; %#ok<AGROW>
end
out = tbl(rows, :);
end

function rank = localRowRank(row)
success = localAsLogical(row.success);
acceptable = localAsLogical(row.acceptable);
classification = localAsString(row.classification);
if success
    rank = 0;
elseif acceptable
    rank = 1;
elseif classification == "near_trim_borderline"
    rank = 2;
else
    rank = 3;
end
end

function score = localRowScore(row)
score = row.score;
if isempty(score) || isnan(score)
    score = inf;
end
end

function out = localAsLogical(col)
if islogical(col)
    out = col;
elseif isnumeric(col)
    out = col ~= 0;
elseif isstring(col)
    lowered = lower(strtrim(col));
    out = lowered == "1" | lowered == "true";
elseif iscellstr(col) || iscell(col)
    lowered = lower(strtrim(string(col)));
    out = lowered == "1" | lowered == "true";
elseif istable(col)
    out = localAsLogical(col{1,1});
else
    error('Plot_Transition_Reference_Line_Scored_Merged_Map:UnsupportedLogicalColumn', ...
        'Unsupported logical-like column type.');
end
out = logical(out);
end

function out = localAsString(col)
if istable(col)
    col = col{1,1};
end
if isstring(col)
    out = col;
elseif iscellstr(col) || iscell(col)
    out = string(col);
elseif isnumeric(col)
    out = string(col);
else
    out = string(col);
end
end

function entries = localLegendEntries(isOther, isBorderline, isAcceptable, isExact)
entries = {'Reference line', 'Merged point order'};
if any(isOther)
    entries{end + 1} = 'Failed / not usable'; %#ok<AGROW>
end
if any(isBorderline)
    entries{end + 1} = 'Borderline near-trim'; %#ok<AGROW>
end
if any(isAcceptable)
    entries{end + 1} = 'Acceptable near-trim'; %#ok<AGROW>
end
if any(isExact)
    entries{end + 1} = 'Exact trim'; %#ok<AGROW>
end
entries{end + 1} = 'Hover anchor'; %#ok<AGROW>
entries{end + 1} = 'Cruise anchor'; %#ok<AGROW>
end

function out = localMergeStruct(defaults, overrides)
out = defaults;
if isempty(overrides)
    return;
end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    out.(fields{i}) = overrides.(fields{i});
end
end

function vis = localFigureVisibility(showPopup)
if showPopup
    vis = 'on';
else
    vis = 'off';
end
end

function label = localValueLabel(value)
label = strrep(sprintf('%.3f', value), '.', 'p');
label = regexprep(label, 'p?0+$', '');
if endsWith(label, 'p')
    label = extractBefore(label, strlength(label));
end
if isempty(label)
    label = '0';
end
end
