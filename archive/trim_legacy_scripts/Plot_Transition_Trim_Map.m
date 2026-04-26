% Plot_Transition_Trim_Map.m
% Generate quick visual summaries from the latest merged trim-map CSV.
%
% Outputs left in the base workspace:
%   - transitionTrimPlotDir
%   - transitionTrimPlotSummary
%
% Main output files:
%   workspace_plots/transition_trim_visuals_<timestamp>/
%     transition_success_map.png

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

inputCsv = fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv');
if exist(inputCsv, 'file') ~= 2
    error('Plot_Transition_Trim_Map:MissingCsv', ...
        'Missing merged trim-map CSV: %s', inputCsv);
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionTrimPlotDir = fullfile(root_dir, 'workspace_plots', ['transition_trim_visuals_' timestamp]);
if exist(transitionTrimPlotDir, 'dir') ~= 7
    mkdir(transitionTrimPlotDir);
end

tbl = readtable(inputCsv, 'TextType', 'string');
if isempty(tbl)
    error('Plot_Transition_Trim_Map:EmptyCsv', 'Merged trim-map CSV is empty.');
end

successMask = logical(tbl.success);
tilt_deg = tbl.tilt_deg;
vinf_mps = tbl.vinf_mps;

transitionTrimPlotSummary = struct();
transitionTrimPlotSummary.input_csv = inputCsv;
transitionTrimPlotSummary.output_dir = transitionTrimPlotDir;
transitionTrimPlotSummary.total_points = height(tbl);
transitionTrimPlotSummary.exact_successes = nnz(successMask);
transitionTrimPlotSummary.failures_or_inexact = height(tbl) - nnz(successMask);
transitionTrimPlotSummary.reference_hover_point = [0.0, 0.0];
transitionTrimPlotSummary.reference_cruise_point = [70.0, 90.0];

% Plot 1: exact / failed points in tilt-speed space.
f1 = figure('Color', 'w', 'Position', [100 100 900 650]);
hold on;
scatter(vinf_mps(~successMask), tilt_deg(~successMask), 28, ...
    [0.75 0.75 0.75], 'x', 'LineWidth', 0.9);
scatter(vinf_mps(successMask), tilt_deg(successMask), 42, ...
    [0.12 0.60 0.18], 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
scatter(70.0, 90.0, 140, 'p', ...
    'MarkerFaceColor', [0.92 0.55 0.05], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
scatter(0.0, 0.0, 140, 'd', ...
    'MarkerFaceColor', [0.10 0.35 0.90], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Transition Trim Success Map');
legend({'Inexact / failed', 'Exact success', 'Cruise anchor (70 m/s, 90 deg)', ...
    'Hover anchor (0 m/s, 0 deg)'}, 'Location', 'northwest');
grid on;
box on;
saveas(f1, fullfile(transitionTrimPlotDir, 'transition_success_map.png'));

disp(transitionTrimPlotDir);
