% Plot_Transition_Path_Scored_Map.m
% Plot the points found by the path-following scored transition sweep.
%
% Optional workspace overrides before running:
%   transitionPathScoredPlotInputCsv = '/abs/path/to/transition_trim_path_scored_latest.csv';
%   transitionPathScoredPlotOptions = struct('show_popup', true);
%
% Outputs left in the base workspace:
%   - transitionPathScoredPlotDir
%   - transitionPathScoredPlotSummary
%
% Main output files:
%   workspace_plots/transition_path_scored_visuals_<timestamp>/
%     path_scored_classification_map.png
%     path_scored_prop_rpm.png

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('transitionPathScoredPlotInputCsv', 'var') && ...
        (isstring(transitionPathScoredPlotInputCsv) || ischar(transitionPathScoredPlotInputCsv))
    inputCsv = char(transitionPathScoredPlotInputCsv);
else
    inputCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_path_scored_latest.csv');
end

plotOptions = struct('show_popup', true);
if exist('transitionPathScoredPlotOptions', 'var') && isstruct(transitionPathScoredPlotOptions)
    plotOptions = localMergeStruct(plotOptions, transitionPathScoredPlotOptions);
end

if exist(inputCsv, 'file') ~= 2
    error('Plot_Transition_Path_Scored_Map:MissingCsv', ...
        'Missing path scored CSV: %s', inputCsv);
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionPathScoredPlotDir = fullfile(root_dir, 'workspace_plots', ...
    ['transition_path_scored_visuals_' timestamp]);
if exist(transitionPathScoredPlotDir, 'dir') ~= 7
    mkdir(transitionPathScoredPlotDir);
end

tbl = readtable(inputCsv, 'TextType', 'string');
if isempty(tbl)
    error('Plot_Transition_Path_Scored_Map:EmptyCsv', 'Path scored CSV is empty.');
end

successMask = localAsLogical(tbl.success);
acceptableMask = localAsLogical(tbl.acceptable);
classification = localAsString(tbl.classification);
vinf_mps = tbl.vinf_mps;
tilt_deg = tbl.tilt_deg;

isExact = classification == "exact_trim" | successMask;
isAcceptable = acceptableMask & ~isExact;
isBorderline = classification == "near_trim_borderline" & ~isExact & ~isAcceptable;
isOther = ~(isExact | isAcceptable | isBorderline);

[~, pathOrder] = sortrows([tilt_deg, vinf_mps], [1 2]);

transitionPathScoredPlotSummary = struct();
transitionPathScoredPlotSummary.input_csv = inputCsv;
transitionPathScoredPlotSummary.output_dir = transitionPathScoredPlotDir;
transitionPathScoredPlotSummary.total_points = height(tbl);
transitionPathScoredPlotSummary.exact_count = nnz(isExact);
transitionPathScoredPlotSummary.acceptable_count = nnz(isAcceptable);
transitionPathScoredPlotSummary.borderline_count = nnz(isBorderline);
transitionPathScoredPlotSummary.other_count = nnz(isOther);

figVisible = localFigureVisibility(plotOptions.show_popup);

% Plot 1: path points in (Vinf, tilt) space.
f1 = figure('Color', 'w', 'Position', [100 100 980 700], 'Visible', figVisible);
hold on;
plot(vinf_mps(pathOrder), tilt_deg(pathOrder), '--', 'Color', [0.45 0.75 0.95], 'LineWidth', 1.2);

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

for i = 1:height(tbl)
    text(vinf_mps(i) + 0.4, tilt_deg(i) + 0.4, char(tbl.name(i)), ...
        'FontSize', 8, 'Color', [0.15 0.15 0.15]);
end

xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Path-Scored Transition Points Found');
legend(localLegendEntries(isOther, isBorderline, isAcceptable, isExact), ...
    'Location', 'northwest');
grid on;
box on;
saveas(f1, fullfile(transitionPathScoredPlotDir, 'path_scored_classification_map.png'));

% Plot 2: front and rear prop RPM across the found path.
f2 = figure('Color', 'w', 'Position', [120 120 980 700], 'Visible', figVisible);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(vinf_mps(pathOrder), tbl.front_collective_rpm(pathOrder), '-', 'Color', [0.10 0.35 0.90], 'LineWidth', 1.2);
scatter(vinf_mps(isExact), tbl.front_collective_rpm(isExact), 42, [0.12 0.60 0.18], 'filled');
scatter(vinf_mps(isAcceptable), tbl.front_collective_rpm(isAcceptable), 48, [0.98 0.82 0.18], 's', 'filled');
if any(isOther | isBorderline)
    scatter(vinf_mps(isOther | isBorderline), tbl.front_collective_rpm(isOther | isBorderline), ...
        42, [0.70 0.70 0.70], 'x', 'LineWidth', 1.0);
end
ylabel('Front collective (rpm)');
title('Path-Scored Propeller Values');
grid on;
box on;

nexttile;
hold on;
plot(vinf_mps(pathOrder), tbl.rear_collective_rpm(pathOrder), '-', 'Color', [0.80 0.25 0.20], 'LineWidth', 1.2);
scatter(vinf_mps(isExact), tbl.rear_collective_rpm(isExact), 42, [0.12 0.60 0.18], 'filled');
scatter(vinf_mps(isAcceptable), tbl.rear_collective_rpm(isAcceptable), 48, [0.98 0.82 0.18], 's', 'filled');
if any(isOther | isBorderline)
    scatter(vinf_mps(isOther | isBorderline), tbl.rear_collective_rpm(isOther | isBorderline), ...
        42, [0.70 0.70 0.70], 'x', 'LineWidth', 1.0);
end
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Rear collective (rpm)');
grid on;
box on;
saveas(f2, fullfile(transitionPathScoredPlotDir, 'path_scored_prop_rpm.png'));

disp(transitionPathScoredPlotDir);

if ~plotOptions.show_popup
    close(f1);
    close(f2);
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
else
    error('Plot_Transition_Path_Scored_Map:UnsupportedLogicalColumn', ...
        'Unsupported logical-like column type.');
end
out = logical(out);
end

function out = localAsString(col)
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
entries = {'Path order'};
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
