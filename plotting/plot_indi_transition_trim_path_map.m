function plotSummary = plot_indi_transition_trim_path_map(controllerData, opts)
%PLOT_INDI_TRANSITION_TRIM_PATH_MAP Plot current INDI path over trim DB.
%
% Usage:
%   Init_Main
%   plotSummary = plot_indi_transition_trim_path_map();
%
% Optional:
%   plotSummary = plot_indi_transition_trim_path_map(controllerData);
%   plotSummary = plot_indi_transition_trim_path_map(controllerData, opts);
%
% The selected path comes from controllerData.schedule_table. If controllerData
% is not provided, this function builds it with build_indi_transition_controller.

if nargin < 1
    controllerData = [];
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
if isempty(controllerData)
    controllerData = localGetOrBuildControllerData();
elseif isstruct(controllerData) && ~isfield(controllerData, 'schedule_table')
    opts = controllerData;
    controllerData = localGetOrBuildControllerData();
end
opts = localApplyDefaults(opts, controllerData);

[allRows, bestRows] = localLoadTrimRows(opts.trim_db_file);
if isempty(allRows)
    error('plot_indi_transition_trim_path_map:EmptyDb', ...
        'No trim rows found in %s.', opts.trim_db_file);
end

allRows.plot_alpha_deg = localPlotAlpha(allRows);
bestRows.plot_alpha_deg = localPlotAlpha(bestRows);
pathRows = controllerData.schedule_table;
pathRows.plot_alpha_deg = localPlotAlpha(pathRows);

outputDir = localOutputDir(opts);
visibleMode = "off";
if opts.show_popup
    visibleMode = "on";
end

style = localStyle();
bestClass = localClassMasks(bestRows);

f2 = figure('Color', 'w', 'Position', [80 80 1060 720], ...
    'Visible', visibleMode);
ax2 = axes(f2);
hold(ax2, 'on');
scatter(ax2, allRows.vinf_mps, allRows.tilt_deg, 18, style.allGray, 'x', ...
    'DisplayName', 'All trim attempts');
localScatter2(ax2, bestRows, bestClass.isOther, style.fail, 'x', ...
    'Failed / not usable');
localScatter2(ax2, bestRows, bestClass.isBorderline, style.borderline, '^', ...
    'Borderline near-trim');
localScatter2(ax2, bestRows, bestClass.isUsableOnly, style.acceptable, 'o', ...
    'Usable near-trim');
localScatter2(ax2, bestRows, bestClass.isExact, style.exact, 'o', ...
    'Exact trim');
localDrawGuide2(ax2, controllerData, style);
localDrawPath2(ax2, pathRows, style);
xlabel(ax2, 'Airspeed V_{\infty} (m/s)', 'Color', 'k');
ylabel(ax2, 'Front tilt (deg)', 'Color', 'k');
localTitle(ax2, 'Current INDI Transition Trim Path');
localFinishAxes2(ax2, opts, allRows, pathRows);
localLegend(ax2, 'northwest');

output2d = fullfile(outputDir, 'indi_transition_trim_path_2d.png');
output2dFig = fullfile(outputDir, 'indi_transition_trim_path_2d.fig');
exportgraphics(f2, output2d, 'Resolution', 300);
savefig(f2, output2dFig);

f3 = figure('Color', 'w', 'Position', [120 120 1120 800], ...
    'Visible', visibleMode);
ax3 = axes(f3);
hold(ax3, 'on');
scatter3(ax3, allRows.vinf_mps, allRows.tilt_deg, allRows.plot_alpha_deg, ...
    15, style.allGray, 'x', 'DisplayName', 'All trim attempts');
localScatter3(ax3, bestRows, bestClass.isOther, style.fail, 'x', ...
    'Failed / not usable');
localScatter3(ax3, bestRows, bestClass.isBorderline, style.borderline, '^', ...
    'Borderline near-trim');
localScatter3(ax3, bestRows, bestClass.isUsableOnly, style.acceptable, 'o', ...
    'Usable near-trim');
localScatter3(ax3, bestRows, bestClass.isExact, style.exact, 'o', ...
    'Exact trim');
localDrawGuide3(ax3, controllerData, style);
localDrawPath3(ax3, pathRows, style);
xlabel(ax3, 'Airspeed V_{\infty} (m/s)', 'Color', 'k');
ylabel(ax3, 'Front tilt (deg)', 'Color', 'k');
zlabel(ax3, 'Alpha / target alpha (deg)', 'Color', 'k');
localTitle(ax3, 'Current INDI Transition Trim Path: V_{\infty}, Tilt, Alpha');
localFinishAxes3(ax3, opts, allRows, pathRows);
view(ax3, 42, 24);
localLegend(ax3, 'northeast');

output3d = fullfile(outputDir, 'indi_transition_trim_path_3d.png');
output3dFig = fullfile(outputDir, 'indi_transition_trim_path_3d.fig');
exportgraphics(f3, output3d, 'Resolution', 300);
savefig(f3, output3dFig);

plotSummary = struct();
plotSummary.trim_db_file = string(opts.trim_db_file);
plotSummary.output_dir = string(outputDir);
plotSummary.all_rows = height(allRows);
plotSummary.best_unique_rows = height(bestRows);
plotSummary.path_points = height(pathRows);
plotSummary.generated_files = string({output2d; output2dFig; output3d; output3dFig});

assignin('base', 'indiTransitionTrimPathPlotSummary', plotSummary);
assignin('base', 'indiTransitionTrimPathPlotDir', outputDir);
disp(outputDir);

if ~opts.show_popup
    close(f2);
    close(f3);
end
end

function controllerData = localGetOrBuildControllerData()
if evalin('base', "exist('controllerData', 'var')") == 1
    controllerData = evalin('base', 'controllerData');
    if isstruct(controllerData) && isfield(controllerData, 'schedule_table')
        return;
    end
end
controllerData = build_indi_transition_controller();
end

function opts = localApplyDefaults(opts, controllerData)
repoRoot = localRepoRoot();
if ~isfield(opts, 'trim_db_file') || isempty(opts.trim_db_file)
    if isfield(controllerData, 'trim_db_file') && ~isempty(controllerData.trim_db_file)
        opts.trim_db_file = controllerData.trim_db_file;
    else
        opts.trim_db_file = fullfile(repoRoot, 'databases', ...
            'trim_vinf_alpha_v1', 'trim_attempts.mat');
    end
end
if ~isfield(opts, 'output_dir') || isempty(opts.output_dir)
    opts.output_dir = "";
end
if ~isfield(opts, 'show_popup') || isempty(opts.show_popup)
    opts.show_popup = true;
end
if ~isfield(opts, 'vinf_xlim_mps') || isempty(opts.vinf_xlim_mps)
    opts.vinf_xlim_mps = [];
end
if ~isfield(opts, 'tilt_ylim_deg') || isempty(opts.tilt_ylim_deg)
    opts.tilt_ylim_deg = [0 95];
end
if ~isfield(opts, 'alpha_zlim_deg') || isempty(opts.alpha_zlim_deg)
    opts.alpha_zlim_deg = [];
end
if ~isfield(opts, 'label_path_points') || isempty(opts.label_path_points)
    opts.label_path_points = true;
end
end

function rootDir = localRepoRoot()
stack = dbstack('-completenames');
if isempty(stack)
    rootDir = pwd;
else
    rootDir = fileparts(fileparts(stack(1).file));
end
end

function [allRows, bestRows] = localLoadTrimRows(dbFile)
if exist(dbFile, 'file') ~= 2
    error('plot_indi_transition_trim_path_map:MissingDb', ...
        'Trim DB not found: %s', dbFile);
end
raw = load(dbFile, 'transitionTrimMasterAttemptDB');
if ~isfield(raw, 'transitionTrimMasterAttemptDB')
    error('plot_indi_transition_trim_path_map:MissingVariable', ...
        'Expected transitionTrimMasterAttemptDB in %s.', dbFile);
end
db = raw.transitionTrimMasterAttemptDB;
if isfield(db, 'master_attempt_db_all_rows')
    allRows = db.master_attempt_db_all_rows;
else
    allRows = table();
end
if isfield(db, 'master_attempt_db_best_unique_points')
    bestRows = db.master_attempt_db_best_unique_points;
else
    bestRows = allRows;
end
end

function outputDir = localOutputDir(opts)
if strlength(string(opts.output_dir)) > 0
    outputDir = char(opts.output_dir);
else
    repoRoot = localRepoRoot();
    timestamp = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(repoRoot, 'workspace_plots', ...
        ['indi_transition_trim_path_' timestamp]);
end
if exist(outputDir, 'dir') ~= 7
    mkdir(outputDir);
end
end

function alphaDeg = localPlotAlpha(tbl)
alphaDeg = localNumericColumn(tbl, 'alpha_target_deg', NaN);
fallback = localNumericColumn(tbl, 'alpha_deg', NaN);
replaceMask = ~isfinite(alphaDeg) & isfinite(fallback);
alphaDeg(replaceMask) = fallback(replaceMask);
if all(~isfinite(alphaDeg))
    alphaDeg = zeros(height(tbl), 1);
end
end

function data = localNumericColumn(tbl, name, fallback)
data = fallback * ones(height(tbl), 1);
if ~istable(tbl) || ~ismember(name, tbl.Properties.VariableNames)
    return;
end
raw = tbl.(name);
if isnumeric(raw) || islogical(raw)
    data = double(raw(:));
else
    data = str2double(string(raw(:)));
end
end

function masks = localClassMasks(tbl)
classification = lower(strtrim(localStringColumn(tbl, 'classification')));
success = localLogicalColumn(tbl, 'success');
acceptable = localLogicalColumn(tbl, 'acceptable');
masks.isExact = success | classification == "exact_trim";
masks.isUsable = acceptable | masks.isExact | classification == "quasi_trim_usable";
masks.isUsableOnly = masks.isUsable & ~masks.isExact;
masks.isBorderline = classification == "near_trim_borderline";
masks.isOther = ~(masks.isExact | masks.isUsableOnly | masks.isBorderline);
end

function data = localStringColumn(tbl, name)
data = strings(height(tbl), 1);
if istable(tbl) && ismember(name, tbl.Properties.VariableNames)
    data = string(tbl.(name));
end
end

function out = localLogicalColumn(tbl, name)
out = false(height(tbl), 1);
if ~istable(tbl) || ~ismember(name, tbl.Properties.VariableNames)
    return;
end
column = tbl.(name);
if islogical(column)
    out = column(:);
elseif isnumeric(column)
    out = column(:) ~= 0;
else
    lowered = lower(strtrim(string(column(:))));
    out = lowered == "true" | lowered == "1" | lowered == "yes";
end
end

function style = localStyle()
style.allGray = [0.76 0.76 0.76];
style.fail = [0.35 0.35 0.35];
style.borderline = [0.95 0.55 0.15];
style.acceptable = [0.12 0.42 0.90];
style.exact = [0.05 0.55 0.20];
style.guide = [0.05 0.05 0.05];
style.path = [0.90 0.05 0.04];
style.start = [0.08 0.22 0.90];
style.finish = [0.05 0.05 0.05];
end

function localScatter2(ax, tbl, mask, color, marker, label)
if any(mask)
    scatter(ax, tbl.vinf_mps(mask), tbl.tilt_deg(mask), 52, color, marker, ...
        'LineWidth', 1.2, 'DisplayName', label);
end
end

function localScatter3(ax, tbl, mask, color, marker, label)
if any(mask)
    scatter3(ax, tbl.vinf_mps(mask), tbl.tilt_deg(mask), ...
        tbl.plot_alpha_deg(mask), 58, color, marker, ...
        'LineWidth', 1.2, 'DisplayName', label);
end
end

function localDrawGuide2(ax, controllerData, style)
[guideV, guideTilt] = localGuideFromController(controllerData);
if isempty(guideV)
    return;
end
plot(ax, guideV, guideTilt, '--', 'Color', style.guide, ...
    'LineWidth', 1.7, 'DisplayName', 'Path guide');
end

function localDrawGuide3(ax, controllerData, style)
[guideV, guideTilt] = localGuideFromController(controllerData);
if isempty(guideV)
    return;
end
plot3(ax, guideV, guideTilt, zeros(size(guideV)), '--', ...
    'Color', style.guide, 'LineWidth', 1.7, ...
    'DisplayName', 'Path guide at alpha = 0');
end

function [guideV, guideTilt] = localGuideFromController(controllerData)
guideV = [];
guideTilt = [];
if ~isfield(controllerData, 'path_debug') || ...
        ~isfield(controllerData.path_debug, 'candidate_meta')
    return;
end
meta = controllerData.path_debug.candidate_meta;
if isempty(meta) || ~isfield(meta, 'guide_vinf_mps') || ...
        ~isfield(meta, 'guide_tilt_deg')
    return;
end
guideV = [meta.guide_vinf_mps];
guideTilt = [meta.guide_tilt_deg];
end

function localDrawPath2(ax, pathRows, style)
plot(ax, pathRows.vinf_mps, pathRows.tilt_deg, '-', ...
    'Color', style.path, 'LineWidth', 2.8, ...
    'DisplayName', 'Selected INDI path');
scatter(ax, pathRows.vinf_mps, pathRows.tilt_deg, 70, style.path, 'o', ...
    'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
scatter(ax, pathRows.vinf_mps(1), pathRows.tilt_deg(1), 95, style.start, ...
    'd', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Path start');
scatter(ax, pathRows.vinf_mps(end), pathRows.tilt_deg(end), 105, style.finish, ...
    'p', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Path end');
localLabelPath2(ax, pathRows);
end

function localDrawPath3(ax, pathRows, style)
plot3(ax, pathRows.vinf_mps, pathRows.tilt_deg, pathRows.plot_alpha_deg, '-', ...
    'Color', style.path, 'LineWidth', 3.0, ...
    'DisplayName', 'Selected INDI path');
scatter3(ax, pathRows.vinf_mps, pathRows.tilt_deg, pathRows.plot_alpha_deg, ...
    72, style.path, 'o', 'filled', 'MarkerEdgeColor', 'k', ...
    'LineWidth', 0.8, 'HandleVisibility', 'off');
scatter3(ax, pathRows.vinf_mps(1), pathRows.tilt_deg(1), ...
    pathRows.plot_alpha_deg(1), 100, style.start, 'd', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Path start');
scatter3(ax, pathRows.vinf_mps(end), pathRows.tilt_deg(end), ...
    pathRows.plot_alpha_deg(end), 112, style.finish, 'p', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Path end');
localLabelPath3(ax, pathRows);
end

function localLabelPath2(ax, pathRows)
for i = 1:height(pathRows)
    text(ax, pathRows.vinf_mps(i), pathRows.tilt_deg(i), ...
        sprintf('  %d', i), 'Color', 'k', 'FontSize', 9, ...
        'FontWeight', 'bold', 'Clipping', 'on');
end
end

function localLabelPath3(ax, pathRows)
for i = 1:height(pathRows)
    text(ax, pathRows.vinf_mps(i), pathRows.tilt_deg(i), ...
        pathRows.plot_alpha_deg(i), sprintf('  %d', i), ...
        'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Clipping', 'on');
end
end

function localFinishAxes2(ax, opts, allRows, pathRows)
set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.82 0.82 0.82], ...
    'MinorGridColor', [0.90 0.90 0.90], ...
    'FontName', 'Helvetica', 'FontSize', 12, 'LineWidth', 1.0);
grid(ax, 'on');
box(ax, 'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
xlim(ax, localVinfLimits(opts, allRows, pathRows));
ylim(ax, opts.tilt_ylim_deg);
end

function localFinishAxes3(ax, opts, allRows, pathRows)
set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'GridColor', [0.82 0.82 0.82], ...
    'MinorGridColor', [0.90 0.90 0.90], ...
    'FontName', 'Helvetica', 'FontSize', 12, 'LineWidth', 1.0);
grid(ax, 'on');
box(ax, 'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.ZMinorGrid = 'on';
xlim(ax, localVinfLimits(opts, allRows, pathRows));
ylim(ax, opts.tilt_ylim_deg);
zlim(ax, localAlphaLimits(opts, allRows, pathRows));
end

function limits = localVinfLimits(opts, allRows, pathRows)
if ~isempty(opts.vinf_xlim_mps)
    limits = opts.vinf_xlim_mps;
    return;
end
values = [allRows.vinf_mps; pathRows.vinf_mps];
maxV = max(values, [], 'omitnan');
if ~isfinite(maxV)
    maxV = 80;
end
limits = [0, max(10, 5 * ceil((maxV + 2.5) / 5))];
end

function limits = localAlphaLimits(opts, allRows, pathRows)
if ~isempty(opts.alpha_zlim_deg)
    limits = opts.alpha_zlim_deg;
    return;
end
values = [allRows.plot_alpha_deg; pathRows.plot_alpha_deg];
values = values(isfinite(values));
if isempty(values)
    limits = [-10 10];
    return;
end
limits = [5 * floor((min(values) - 2.5) / 5), ...
    5 * ceil((max(values) + 2.5) / 5)];
if limits(1) == limits(2)
    limits = limits + [-5 5];
end
end

function localTitle(ax, label)
t = title(ax, label);
set(t, 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 15);
end

function localLegend(ax, location)
leg = legend(ax, 'Location', location);
set(leg, 'Color', 'w', 'TextColor', 'k', ...
    'EdgeColor', [0.72 0.72 0.72], 'FontSize', 10, 'Box', 'on');
end
