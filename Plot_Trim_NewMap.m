% plot new trim map

if ~exist('trimNewMapPlotOptions', 'var') || ~isstruct(trimNewMapPlotOptions)
    trimNewMapPlotOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

dbPaths = TrimDB_Paths(root_dir);
opts = localApplyDefaults(trimNewMapPlotOptions, dbPaths);

if exist(opts.master_db_file, 'file') ~= 2
    error('Plot_Trim_NewMap:MissingDb', 'Master DB not found: %s', opts.master_db_file);
end

raw = load(opts.master_db_file, 'transitionTrimMasterAttemptDB');
if ~isfield(raw, 'transitionTrimMasterAttemptDB')
    error('Plot_Trim_NewMap:MissingVariable', ...
        'Expected transitionTrimMasterAttemptDB in %s.', opts.master_db_file);
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

if isempty(allRows)
    error('Plot_Trim_NewMap:EmptyDb', 'No trim rows found in %s.', opts.master_db_file);
end

if ~ismember('alpha_target_deg', allRows.Properties.VariableNames)
    allRows.alpha_target_deg = nan(height(allRows), 1);
end
if ~ismember('alpha_target_deg', bestRows.Properties.VariableNames)
    bestRows.alpha_target_deg = nan(height(bestRows), 1);
end

if strlength(string(opts.output_dir)) > 0
    outputDir = char(opts.output_dir);
else
    timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(dbPaths.workspace_plots_dir, ...
        sprintf('%s_plots_%s', char(opts.database_name), timestamp));
end
if exist(outputDir, 'dir') ~= 7
    mkdir(outputDir);
end

visibleMode = "off";
if logical(opts.show_popup)
    visibleMode = "on";
end

style = localStyle();
bestClass = localClassMasks(bestRows);
allClass = localClassMasks(allRows);

f2 = figure('Color', 'w', 'Position', [80 80 980 680], 'Visible', visibleMode);
hold on;
scatter(allRows.vinf_mps, allRows.tilt_deg, 18, style.allGray, 'x', ...
    'DisplayName', 'All attempts');
localScatter2(bestRows, bestClass.isOther, style.fail, 'x', 'Failed / not usable');
localScatter2(bestRows, bestClass.isBorderline, style.borderline, '^', 'Borderline near-trim');
localScatter2(bestRows, bestClass.isAcceptableOnly, style.acceptable, 'o', 'Acceptable near-trim');
localScatter2(bestRows, bestClass.isExact, style.exact, 'o', 'Exact trim');
localDrawGuideCurve2(gca, opts, style);
localDrawEndpointMarkers2(gca, opts, style);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front tilt (deg)');
localTitle(gca, 'New Trim Map: V_{\infty} vs Tilt');
localFinishAxes(gca, opts, allRows);
localLegend(gca, 'northwest');
grid on;
output2d = fullfile(outputDir, 'trim_newmap_vinf_tilt.png');
exportgraphics(f2, output2d, 'Resolution', 300);

f3 = figure('Color', 'w', 'Position', [120 120 1080 760], 'Visible', visibleMode);
hold on;
scatter3(allRows.vinf_mps, allRows.tilt_deg, allRows.alpha_target_deg, 15, style.allGray, 'x', ...
    'DisplayName', 'All attempts');
localScatter3(bestRows, bestClass.isOther, style.fail, 'x', 'Failed / not usable');
localScatter3(bestRows, bestClass.isBorderline, style.borderline, '^', 'Borderline near-trim');
localScatter3(bestRows, bestClass.isAcceptableOnly, style.acceptable, 'o', 'Acceptable near-trim');
localScatter3(bestRows, bestClass.isExact, style.exact, 'o', 'Exact trim');
localDrawGuideCurve3(gca, opts, style);
localDrawEndpointLines3(gca, opts, style, allRows);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front tilt (deg)');
zlabel('Target alpha (deg)');
localTitle(gca, 'New Trim Map: V_{\infty}, Tilt, Alpha');
localFinishAxes(gca, opts, allRows);
view(42, 24);
localLegend(gca, 'northeast');
grid on;
output3d = fullfile(outputDir, 'trim_newmap_vinf_tilt_alpha_3d.png');
exportgraphics(f3, output3d, 'Resolution', 300);

trimNewMapPlotSummary = struct();
trimNewMapPlotSummary.master_db_file = string(opts.master_db_file);
trimNewMapPlotSummary.output_dir = string(outputDir);
trimNewMapPlotSummary.all_rows = height(allRows);
trimNewMapPlotSummary.best_unique_rows = height(bestRows);
trimNewMapPlotSummary.exact_count = nnz(bestClass.isExact);
trimNewMapPlotSummary.acceptable_count = nnz(bestClass.isAcceptableOnly);
trimNewMapPlotSummary.borderline_count = nnz(bestClass.isBorderline);
trimNewMapPlotSummary.generated_files = string({output2d; output3d});

assignin('base', 'trimNewMapPlotSummary', trimNewMapPlotSummary);
assignin('base', 'trimNewMapPlotDir', outputDir);
disp(outputDir);

if ~logical(opts.show_popup)
    close(f2);
    close(f3);
end

function opts = localApplyDefaults(opts, dbPaths)
if ~isfield(opts, 'database_name') || isempty(opts.database_name)
    opts.database_name = "trim_vinf_alpha_v1";
else
    opts.database_name = string(opts.database_name);
end
if ~isfield(opts, 'master_db_file') || isempty(opts.master_db_file)
    opts.master_db_file = fullfile(dbPaths.database_dir, char(opts.database_name), 'trim_attempts.mat');
end
if ~isfield(opts, 'output_dir')
    opts.output_dir = "";
end
if ~isfield(opts, 'show_popup') || isempty(opts.show_popup)
    opts.show_popup = true;
end
if ~isfield(opts, 'tilt_ylim_deg') || isempty(opts.tilt_ylim_deg)
    opts.tilt_ylim_deg = [0 95];
end
if ~isfield(opts, 'vinf_xlim_mps') || isempty(opts.vinf_xlim_mps)
    opts.vinf_xlim_mps = [];
end
if ~isfield(opts, 'alpha_zlim_deg') || isempty(opts.alpha_zlim_deg)
    opts.alpha_zlim_deg = [];
end
if ~isfield(opts, 'endpoint_points') || isempty(opts.endpoint_points)
    opts.endpoint_points = localDefaultEndpointPoints();
end

% Guide-curve overlay section. Toggle with:
%   trimNewMapPlotOptions.show_guide_curve = false;
if ~isfield(opts, 'show_guide_curve') || isempty(opts.show_guide_curve)
    opts.show_guide_curve = true;
end
if ~isfield(opts, 'guide_curve_vinf_mps') || isempty(opts.guide_curve_vinf_mps)
    opts.guide_curve_vinf_mps = [0 10 20 30 40 50 60 70];
end
if ~isfield(opts, 'guide_curve_tilt_deg') || isempty(opts.guide_curve_tilt_deg)
    opts.guide_curve_tilt_deg = [0 15 35 55 70 80 85 90];
end
if ~isfield(opts, 'guide_curve_alpha_deg') || isempty(opts.guide_curve_alpha_deg)
    opts.guide_curve_alpha_deg = 0.0;
end
if ~isfield(opts, 'guide_curve_label') || isempty(opts.guide_curve_label)
    opts.guide_curve_label = "Search guide curve";
else
    opts.guide_curve_label = string(opts.guide_curve_label);
end
end

function style = localStyle()
style.allGray = [0.76 0.76 0.76];
style.fail = [0.35 0.35 0.35];
style.borderline = [0.95 0.55 0.15];
style.acceptable = [0.10 0.45 0.90];
style.exact = [0.05 0.55 0.20];
style.endpoint = [0.10 0.10 0.10];
style.endpointHover = [0.08 0.25 0.85];
style.endpointCruise = [0.75 0.10 0.10];
style.guide = [0.02 0.02 0.02];
end

function endpointPoints = localDefaultEndpointPoints()
endpointPoints = struct( ...
    'label', {'Hover endpoint', 'Cruise endpoint'}, ...
    'vinf_mps', {0.0, 70.0}, ...
    'tilt_deg', {0.0, 90.0});
end

function masks = localClassMasks(tbl)
classification = lower(string(tbl.classification));
success = localLogicalColumn(tbl, 'success');
acceptable = localLogicalColumn(tbl, 'acceptable');
masks.isExact = success | classification == "exact_trim";
masks.isAcceptable = acceptable | masks.isExact | classification == "quasi_trim_usable";
masks.isAcceptableOnly = masks.isAcceptable & ~masks.isExact;
masks.isBorderline = classification == "near_trim_borderline";
masks.isOther = ~(masks.isExact | masks.isAcceptableOnly | masks.isBorderline);
end

function out = localLogicalColumn(tbl, name)
if ~ismember(name, tbl.Properties.VariableNames)
    out = false(height(tbl), 1);
    return;
end
column = tbl.(name);
if islogical(column)
    out = column;
elseif isnumeric(column)
    out = column ~= 0;
else
    lowered = lower(strtrim(string(column)));
    out = lowered == "true" | lowered == "1" | lowered == "yes";
end
end

function localScatter2(tbl, mask, color, marker, label)
if any(mask)
    scatter(tbl.vinf_mps(mask), tbl.tilt_deg(mask), 52, color, marker, ...
        'LineWidth', 1.2, 'DisplayName', label);
end
end

function localScatter3(tbl, mask, color, marker, label)
if any(mask)
    scatter3(tbl.vinf_mps(mask), tbl.tilt_deg(mask), tbl.alpha_target_deg(mask), ...
        62, color, marker, 'LineWidth', 1.2, 'DisplayName', label);
end
end

function localDrawGuideCurve2(ax, opts, style)
if ~logical(opts.show_guide_curve)
    return;
end
[guideV, guideTilt] = localGuideCurvePoints(opts);
if isempty(guideV)
    return;
end
plot(ax, guideV, guideTilt, '-', ...
    'Color', style.guide, ...
    'LineWidth', 2.2, ...
    'DisplayName', char(opts.guide_curve_label));
plot(ax, opts.guide_curve_vinf_mps, opts.guide_curve_tilt_deg, 's', ...
    'Color', style.guide, ...
    'MarkerFaceColor', 'w', ...
    'MarkerSize', 5, ...
    'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
end

function localDrawGuideCurve3(ax, opts, style)
if ~logical(opts.show_guide_curve)
    return;
end
[guideV, guideTilt] = localGuideCurvePoints(opts);
if isempty(guideV)
    return;
end
guideAlpha = opts.guide_curve_alpha_deg * ones(size(guideV));
plot3(ax, guideV, guideTilt, guideAlpha, '-', ...
    'Color', style.guide, ...
    'LineWidth', 2.2, ...
    'DisplayName', char(opts.guide_curve_label));
plot3(ax, opts.guide_curve_vinf_mps, opts.guide_curve_tilt_deg, ...
    opts.guide_curve_alpha_deg * ones(size(opts.guide_curve_vinf_mps)), 's', ...
    'Color', style.guide, ...
    'MarkerFaceColor', 'w', ...
    'MarkerSize', 5, ...
    'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
end

function [guideV, guideTilt] = localGuideCurvePoints(opts)
guideVKnots = opts.guide_curve_vinf_mps(:).';
guideTiltKnots = opts.guide_curve_tilt_deg(:).';
if numel(guideVKnots) ~= numel(guideTiltKnots)
    error('Plot_Trim_NewMap:GuideCurveSizeMismatch', ...
        'guide_curve_vinf_mps and guide_curve_tilt_deg must have the same length.');
end
if numel(guideVKnots) < 2
    guideV = guideVKnots;
    guideTilt = guideTiltKnots;
    return;
end
[guideVKnots, order] = sort(guideVKnots);
guideTiltKnots = guideTiltKnots(order);
guideV = linspace(min(guideVKnots), max(guideVKnots), 220);
guideTilt = interp1(guideVKnots, guideTiltKnots, guideV, 'pchip');
end

function localFinishAxes(ax, opts, allRows)
set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'GridColor', [0.82 0.82 0.82], 'MinorGridColor', [0.90 0.90 0.90], ...
    'FontName', 'Helvetica', 'FontSize', 12, 'LineWidth', 1.0);
box(ax, 'on');
xlim(ax, localResolveVinfLimits(opts, allRows));
ylim(ax, opts.tilt_ylim_deg);
if isa(ax, 'matlab.graphics.axis.Axes') && ~isempty(ax.ZLabel.String)
    zlim(ax, localResolveAlphaLimits(opts, allRows));
end
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
if ~isempty(ax.ZLabel.String)
    ax.ZMinorGrid = 'on';
end
end

function localTitle(ax, label)
t = title(ax, label);
set(t, 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 15);
end

function localLegend(ax, location)
leg = legend(ax, 'Location', location);
set(leg, ...
    'Color', 'w', ...
    'TextColor', 'k', ...
    'EdgeColor', [0.72 0.72 0.72], ...
    'FontSize', 10, ...
    'Box', 'on');
end

function limits = localResolveVinfLimits(opts, allRows)
if ~isempty(opts.vinf_xlim_mps)
    limits = opts.vinf_xlim_mps;
    return;
end
maxV = max(allRows.vinf_mps, [], 'omitnan');
if ~isfinite(maxV)
    maxV = 80;
end
limits = [0, max(10, 5 * ceil((maxV + 2.5) / 5))];
end

function limits = localResolveAlphaLimits(opts, allRows)
if ~isempty(opts.alpha_zlim_deg)
    limits = opts.alpha_zlim_deg;
    return;
end
alpha = allRows.alpha_target_deg(isfinite(allRows.alpha_target_deg));
if isempty(alpha)
    limits = [-10 10];
    return;
end
limits = [5 * floor((min(alpha) - 2.5) / 5), 5 * ceil((max(alpha) + 2.5) / 5)];
if limits(1) == limits(2)
    limits = limits + [-5 5];
end
end

function localDrawEndpointMarkers2(ax, opts, style)
points = opts.endpoint_points;
for i = 1:numel(points)
    point = points(i);
    color = localEndpointColor(point.label, style);
    plot(ax, point.vinf_mps, point.tilt_deg, 'p', ...
        'MarkerSize', 15, ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.0, ...
        'DisplayName', char(string(point.label)));
end
end

function localDrawEndpointLines3(ax, opts, style, allRows)
points = opts.endpoint_points;
alphaLimits = localResolveAlphaLimits(opts, allRows);
for i = 1:numel(points)
    point = points(i);
    color = localEndpointColor(point.label, style);
    plot3(ax, ...
        [point.vinf_mps point.vinf_mps], ...
        [point.tilt_deg point.tilt_deg], ...
        alphaLimits, ...
        '--', ...
        'Color', color, ...
        'LineWidth', 1.8, ...
        'DisplayName', char(string(point.label) + " alpha sweep"));
    plot3(ax, point.vinf_mps, point.tilt_deg, alphaLimits(1), 'p', ...
        'MarkerSize', 13, ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
    text(ax, point.vinf_mps, point.tilt_deg, alphaLimits(2), ...
        "  " + string(point.label), ...
        'Color', 'k', ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'left', ...
        'Clipping', 'on');
end
end

function color = localEndpointColor(label, style)
label = lower(string(label));
if contains(label, "hover")
    color = style.endpointHover;
elseif contains(label, "cruise")
    color = style.endpointCruise;
else
    color = style.endpoint;
end
end
