% Plot_Transition_Trim_Map_Scored.m
% Generate quick visual summaries from the latest scored low-speed trim-map CSV.
%
% Optional workspace override before running:
%   transitionTrimScoredPlotInputCsv = '/abs/path/to/some_scored_summary.csv';
%
% Outputs left in the base workspace:
%   - transitionTrimScoredPlotDir
%   - transitionTrimScoredPlotSummary
%
% Main output files:
%   workspace_plots/transition_trim_scored_visuals_<timestamp>/
%     scored_trim_classification_map.png
%     scored_trim_score_map.png
%     scored_trim_control_effort_map.png
%     scored_trim_prop_rpm_vs_airspeed.png

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('transitionTrimScoredPlotInputCsv', 'var') && ...
        (isstring(transitionTrimScoredPlotInputCsv) || ischar(transitionTrimScoredPlotInputCsv))
    inputCsv = char(transitionTrimScoredPlotInputCsv);
else
    inputCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv');
end

if exist(inputCsv, 'file') ~= 2
    error('Plot_Transition_Trim_Map_Scored:MissingCsv', ...
        'Missing scored trim-map CSV: %s', inputCsv);
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionTrimScoredPlotDir = fullfile(root_dir, 'workspace_plots', ...
    ['transition_trim_scored_visuals_' timestamp]);
if exist(transitionTrimScoredPlotDir, 'dir') ~= 7
    mkdir(transitionTrimScoredPlotDir);
end

tbl = readtable(inputCsv, 'TextType', 'string');
if isempty(tbl)
    error('Plot_Transition_Trim_Map_Scored:EmptyCsv', 'Scored trim-map CSV is empty.');
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

scoreVals = tbl.score;
scoreVals(~isfinite(scoreVals) | scoreVals <= 0) = eps;
maxNormVals = tbl.max_normalized;
maxNormVals(~isfinite(maxNormVals) | maxNormVals <= 0) = eps;

controlEffort = zeros(height(tbl), 1);
if ismember('delta_f_deg', tbl.Properties.VariableNames)
    controlEffort = controlEffort + abs(tbl.delta_f_deg);
end
if ismember('delta_e_deg', tbl.Properties.VariableNames)
    controlEffort = controlEffort + abs(tbl.delta_e_deg);
end
if ismember('delta_a_deg', tbl.Properties.VariableNames)
    controlEffort = controlEffort + abs(tbl.delta_a_deg);
end
if ismember('delta_r_deg', tbl.Properties.VariableNames)
    controlEffort = controlEffort + abs(tbl.delta_r_deg);
end

transitionTrimScoredPlotSummary = struct();
transitionTrimScoredPlotSummary.input_csv = inputCsv;
transitionTrimScoredPlotSummary.output_dir = transitionTrimScoredPlotDir;
transitionTrimScoredPlotSummary.total_points = height(tbl);
transitionTrimScoredPlotSummary.exact_trim_count = nnz(isExact);
transitionTrimScoredPlotSummary.acceptable_count = nnz(isAcceptable);
transitionTrimScoredPlotSummary.borderline_count = nnz(isBorderline);
transitionTrimScoredPlotSummary.other_count = nnz(isOther);
transitionTrimScoredPlotSummary.best_score = min(scoreVals);
transitionTrimScoredPlotSummary.best_max_normalized = min(maxNormVals);

% Plot 1: classification map in (Vinf, tilt) space.
f1 = figure('Color', 'w', 'Position', [100 100 980 680]);
hold on;
if any(isOther)
    scatter(vinf_mps(isOther), tilt_deg(isOther), 42, [0.70 0.70 0.70], 'x', 'LineWidth', 1.0);
end
if any(isBorderline)
    scatter(vinf_mps(isBorderline), tilt_deg(isBorderline), 54, [0.95 0.55 0.15], '^', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
if any(isAcceptable)
    scatter(vinf_mps(isAcceptable), tilt_deg(isAcceptable), 56, [0.98 0.82 0.18], 's', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
if any(isExact)
    scatter(vinf_mps(isExact), tilt_deg(isExact), 44, [0.12 0.60 0.18], 'o', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
scatter(0.0, 0.0, 140, 'd', 'MarkerFaceColor', [0.10 0.35 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Scored Low-Speed Trim Classification Map');
legend(localLegendEntries(isOther, isBorderline, isAcceptable, isExact), ...
    'Location', 'northwest');
grid on;
box on;
saveas(f1, fullfile(transitionTrimScoredPlotDir, 'scored_trim_classification_map.png'));

% Plot 2: score heatmap-style scatter.
f2 = figure('Color', 'w', 'Position', [110 110 980 680]);
scatter(vinf_mps, tilt_deg, 64, log10(scoreVals), 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.15);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Scored Low-Speed Trim Score Map (log_{10} score)');
cb = colorbar;
cb.Label.String = 'log_{10}(score)';
grid on;
box on;
saveas(f2, fullfile(transitionTrimScoredPlotDir, 'scored_trim_score_map.png'));

% Plot 3: control effort map.
f3 = figure('Color', 'w', 'Position', [120 120 980 680]);
scatter(vinf_mps, tilt_deg, 70, controlEffort, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.15);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Scored Low-Speed Trim Control Effort Map (sum abs deflections)');
cb = colorbar;
cb.Label.String = 'Total control effort (deg)';
grid on;
box on;
saveas(f3, fullfile(transitionTrimScoredPlotDir, 'scored_trim_control_effort_map.png'));

% Plot 4: front and rear RPM vs airspeed, split by exact/acceptable.
f4 = figure('Color', 'w', 'Position', [130 130 980 680]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
if any(~isExact)
    scatter(vinf_mps(~isExact), tbl.front_collective_rpm(~isExact), 36, [0.80 0.80 0.80], 'x', 'LineWidth', 0.8);
end
if any(isAcceptable)
    scatter(vinf_mps(isAcceptable), tbl.front_collective_rpm(isAcceptable), 44, [0.98 0.82 0.18], 's', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
if any(isExact)
    scatter(vinf_mps(isExact), tbl.front_collective_rpm(isExact), 40, [0.12 0.60 0.18], 'o', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
ylabel('Front collective (rpm)');
title('Front Prop RPM vs Airspeed');
grid on;
box on;

nexttile;
hold on;
if any(~isExact)
    scatter(vinf_mps(~isExact), tbl.rear_collective_rpm(~isExact), 36, [0.80 0.80 0.80], 'x', 'LineWidth', 0.8);
end
if any(isAcceptable)
    scatter(vinf_mps(isAcceptable), tbl.rear_collective_rpm(isAcceptable), 44, [0.98 0.82 0.18], 's', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
if any(isExact)
    scatter(vinf_mps(isExact), tbl.rear_collective_rpm(isExact), 40, [0.12 0.60 0.18], 'o', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Rear collective (rpm)');
title('Rear Prop RPM vs Airspeed');
grid on;
box on;
saveas(f4, fullfile(transitionTrimScoredPlotDir, 'scored_trim_prop_rpm_vs_airspeed.png'));

disp(transitionTrimScoredPlotDir);

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
    error('Plot_Transition_Trim_Map_Scored:UnsupportedLogicalColumn', ...
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
entries = {};
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
end
