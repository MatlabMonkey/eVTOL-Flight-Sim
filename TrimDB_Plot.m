% plot trim DB

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end
workspace_plots_dir = fullfile(root_dir, 'workspace_plots');
dbPaths = TrimDB_Paths(root_dir);

plotOptions = localApplyDefaults(struct(), root_dir, workspace_plots_dir, dbPaths);
if exist('transitionTrimDbPlotOptions', 'var') && isstruct(transitionTrimDbPlotOptions)
    plotOptions = localMergeStruct(plotOptions, transitionTrimDbPlotOptions);
    plotOptions = localApplyDefaults(plotOptions, root_dir, workspace_plots_dir, dbPaths);
end

masterDbData = load(plotOptions.master_db_file, 'transitionTrimMasterAttemptDB');
controllerDbData = load(plotOptions.controller_db_file, 'controllerScheduleDB');

if ~isfield(masterDbData, 'transitionTrimMasterAttemptDB')
    error('TrimDB_Plot:MissingMasterDb', ...
        'Could not find transitionTrimMasterAttemptDB in %s.', plotOptions.master_db_file);
end
if ~isfield(controllerDbData, 'controllerScheduleDB')
    error('TrimDB_Plot:MissingControllerDb', ...
        'Could not find controllerScheduleDB in %s.', plotOptions.controller_db_file);
end

masterDb = masterDbData.transitionTrimMasterAttemptDB;
controllerDb = controllerDbData.controllerScheduleDB;

switch lower(string(plotOptions.master_view))
    case "all_rows"
        masterTbl = masterDb.master_attempt_db_all_rows;
    case "best_unique"
        masterTbl = masterDb.master_attempt_db_best_unique_points;
    otherwise
        error('TrimDB_Plot:UnknownMasterView', ...
            'Unknown master_view "%s".', string(plotOptions.master_view));
end

controllerTbl = controllerDb.summary_table;

if plotOptions.exclude_zero_rear_master
    masterTbl = masterTbl(masterTbl.rear_collective_rpm > plotOptions.min_rear_collective_rpm, :);
end
if plotOptions.exclude_zero_rear_controller
    controllerTbl = controllerTbl(controllerTbl.rear_collective_rpm > plotOptions.min_rear_collective_rpm, :);
end

if isempty(masterTbl)
    error('TrimDB_Plot:EmptyMasterSelection', ...
        'No master rows available after filtering.');
end

allRowsTbl = masterDb.master_attempt_db_all_rows;
if plotOptions.exclude_zero_rear_master
    allRowsTbl = allRowsTbl(allRowsTbl.rear_collective_rpm > plotOptions.min_rear_collective_rpm, :);
end

if ~isempty(plotOptions.output_dir)
    transitionTrimDbPlotDir = char(plotOptions.output_dir);
elseif plotOptions.save_to_report_dir
    transitionTrimDbPlotDir = fullfile(root_dir, 'report_plots_final');
else
    timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
    transitionTrimDbPlotDir = fullfile(workspace_plots_dir, ['transition_trim_db_visuals_' timestamp]);
end
if exist(transitionTrimDbPlotDir, 'dir') ~= 7
    mkdir(transitionTrimDbPlotDir);
end

figVisible = localFigureVisibility(plotOptions.show_popup);
style = localReportStyle();
masterClass = localClassificationMasks(masterTbl);
controllerClass = localClassificationMasks(controllerTbl);

transitionTrimDbPlotSummary = struct();
transitionTrimDbPlotSummary.master_db_file = string(plotOptions.master_db_file);
transitionTrimDbPlotSummary.controller_db_file = string(plotOptions.controller_db_file);
transitionTrimDbPlotSummary.master_view = string(plotOptions.master_view);
transitionTrimDbPlotSummary.output_dir = string(transitionTrimDbPlotDir);
transitionTrimDbPlotSummary.master_rows_plotted = height(masterTbl);
transitionTrimDbPlotSummary.controller_rows_plotted = height(controllerTbl);
transitionTrimDbPlotSummary.master_exact_count = nnz(masterClass.isExact);
transitionTrimDbPlotSummary.master_acceptable_count = nnz(masterClass.isAcceptable);
transitionTrimDbPlotSummary.master_borderline_count = nnz(masterClass.isBorderline);
transitionTrimDbPlotSummary.master_other_count = nnz(masterClass.isOther);
transitionTrimDbPlotSummary.controller_exact_count = nnz(controllerClass.isExact);
transitionTrimDbPlotSummary.controller_acceptable_count = nnz(controllerClass.isAcceptable);
writtenFiles = strings(0, 1);

f1 = figure('Color', 'w', 'Position', [100 100 920 620], 'Visible', figVisible);
localConfigureFigure(f1, style);
hold on;
localScatterMasterByClassification(masterTbl, masterClass);
if plotOptions.show_controller_overlay
    localOverlayController(controllerTbl, plotOptions.label_controller_points);
end
localDrawAnchors();
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title(sprintf('Transition Trim Database Map (%s master view)', strrep(char(plotOptions.master_view), '_', ' ')));
leg = legend('Location', 'northwest');
localFinishLegend(leg, style);
localFinishAxes(gca, style, '');
writtenFiles(end + 1, 1) = string(fullfile(transitionTrimDbPlotDir, ...
    localBuildOutputName(plotOptions.report_filename_prefix, 'transition_trim_db_master_plus_controller.png')));
exportgraphics(f1, writtenFiles(end), 'Resolution', 300);

if plotOptions.plot_all_attempts
    allClass = localClassificationMasks(allRowsTbl);

    f2 = figure('Color', 'w', 'Position', [120 120 920 620], 'Visible', figVisible);
    localConfigureFigure(f2, style);
    hold on;
    scatter(allRowsTbl.vinf_mps, allRowsTbl.tilt_deg, 16, [0.85 0.85 0.85], '.', ...
        'MarkerEdgeAlpha', 0.65, 'DisplayName', 'All attempts');
    localScatterMasterByClassification(masterTbl, masterClass);
    if plotOptions.show_controller_overlay
        localOverlayController(controllerTbl, false);
    end
    localDrawAnchors();
    xlabel('Airspeed V_{\infty} (m/s)');
    ylabel('Front Tilt (deg)');
    if plotOptions.show_controller_overlay
        title('All Attempts Coverage With Best-Unique / Controller Overlay');
    else
        title('All Attempts Coverage With Best-Unique Classification');
    end
    leg = legend('Location', 'northwest');
    localFinishLegend(leg, style);
    localFinishAxes(gca, style, '');
    writtenFiles(end + 1, 1) = string(fullfile(transitionTrimDbPlotDir, ...
        localBuildOutputName(plotOptions.report_filename_prefix, 'transition_trim_db_all_attempts_coverage.png')));
    exportgraphics(f2, writtenFiles(end), 'Resolution', 300);
else
    f2 = [];
end

if plotOptions.plot_prop_maps
    fProp = figure('Color', 'w', 'Position', [130 130 980 700], 'Visible', figVisible);
    localConfigureFigure(fProp, style);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    hold on;
    localPlotBackgroundPropMap(allRowsTbl, 'front_collective_rpm');
    localScatterMasterPropByClassification(masterTbl, masterClass, 'front_collective_rpm');
    if plotOptions.show_controller_overlay
        localOverlayControllerProp(controllerTbl, 'front_collective_rpm');
    end
    ylabel('Front collective (rpm)');
    title('Transition Trim Database Propeller Values');
    leg = legend('Location', 'northeast');
    localFinishLegend(leg, style);
    localFinishAxes(gca, style, '');

    nexttile;
    hold on;
    localPlotBackgroundPropMap(allRowsTbl, 'rear_collective_rpm');
    localScatterMasterPropByClassification(masterTbl, masterClass, 'rear_collective_rpm');
    if plotOptions.show_controller_overlay
        localOverlayControllerProp(controllerTbl, 'rear_collective_rpm');
    end
    xlabel('Airspeed V_{\infty} (m/s)');
    ylabel('Rear collective (rpm)');
    localFinishAxes(gca, style, 'Airspeed V_{\infty} (m/s)');

    writtenFiles(end + 1, 1) = string(fullfile(transitionTrimDbPlotDir, ...
        localBuildOutputName(plotOptions.report_filename_prefix, 'transition_trim_db_prop_rpm.png')));
    exportgraphics(fProp, writtenFiles(end), 'Resolution', 300);
else
    fProp = [];
end

if plotOptions.plot_score_maps
    f3 = figure('Color', 'w', 'Position', [140 140 1080 760], 'Visible', figVisible);
    localConfigureFigure(f3, style);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    localScatterMetric(masterTbl, masterTbl.score, 'Score', style);
    title('Best-Unique Score');

    nexttile;
    localScatterMetric(masterTbl, masterTbl.max_normalized, 'Max normalized', style);
    title('Best-Unique Max Normalized');

    nexttile;
    localScatterMetric(masterTbl, masterTbl.rear_collective_rpm, 'Rear collective (rpm)', style);
    title('Best-Unique Rear Collective');

    nexttile;
    localScatterMetric(masterTbl, masterTbl.front_collective_rpm, 'Front collective (rpm)', style);
    title('Best-Unique Front Collective');

    writtenFiles(end + 1, 1) = string(fullfile(transitionTrimDbPlotDir, ...
        localBuildOutputName(plotOptions.report_filename_prefix, 'transition_trim_db_metric_maps.png')));
    exportgraphics(f3, writtenFiles(end), 'Resolution', 300);
else
    f3 = [];
end

transitionTrimDbPlotSummary.generated_files = writtenFiles;
assignin('base', 'transitionTrimDbPlotDir', transitionTrimDbPlotDir);
assignin('base', 'transitionTrimDbPlotSummary', transitionTrimDbPlotSummary);

disp(transitionTrimDbPlotDir);

if ~plotOptions.show_popup
    close(f1);
    if ~isempty(f2), close(f2); end
    if ~isempty(fProp), close(fProp); end
    if ~isempty(f3), close(f3); end
end

function opts = localApplyDefaults(opts, root_dir, workspace_plots_dir, dbPaths)
if ~isfield(opts, 'master_db_file') || isempty(opts.master_db_file)
    opts.master_db_file = dbPaths.master_mat_file;
end
if ~isfield(opts, 'controller_db_file') || isempty(opts.controller_db_file)
    opts.controller_db_file = dbPaths.controller_mat_file;
end
if ~isfield(opts, 'master_view') || isempty(opts.master_view)
    opts.master_view = 'best_unique';
end
if ~isfield(opts, 'show_popup') || isempty(opts.show_popup)
    opts.show_popup = true;
end
if ~isfield(opts, 'save_to_report_dir') || isempty(opts.save_to_report_dir)
    opts.save_to_report_dir = false;
end
if ~isfield(opts, 'report_filename_prefix') || isempty(opts.report_filename_prefix)
    opts.report_filename_prefix = '';
end
if ~isfield(opts, 'output_dir') || isempty(opts.output_dir)
    opts.output_dir = '';
end
if ~isfield(opts, 'show_controller_overlay') || isempty(opts.show_controller_overlay)
    opts.show_controller_overlay = false;
end
if ~isfield(opts, 'exclude_zero_rear_master') || isempty(opts.exclude_zero_rear_master)
    opts.exclude_zero_rear_master = false;
end
if ~isfield(opts, 'exclude_zero_rear_controller') || isempty(opts.exclude_zero_rear_controller)
    opts.exclude_zero_rear_controller = true;
end
if ~isfield(opts, 'min_rear_collective_rpm') || isempty(opts.min_rear_collective_rpm)
    opts.min_rear_collective_rpm = 1.0;
end
if ~isfield(opts, 'plot_all_attempts') || isempty(opts.plot_all_attempts)
    opts.plot_all_attempts = true;
end
if ~isfield(opts, 'plot_prop_maps') || isempty(opts.plot_prop_maps)
    opts.plot_prop_maps = true;
end
if ~isfield(opts, 'plot_score_maps') || isempty(opts.plot_score_maps)
    opts.plot_score_maps = true;
end
if ~isfield(opts, 'label_controller_points') || isempty(opts.label_controller_points)
    opts.label_controller_points = true;
end
if opts.save_to_report_dir && isempty(opts.output_dir)
    opts.output_dir = fullfile(root_dir, 'report_plots_final');
end
end

function masks = localClassificationMasks(tbl)
if isempty(tbl)
    masks = struct('isExact', false(0, 1), 'isAcceptable', false(0, 1), ...
        'isBorderline', false(0, 1), 'isOther', false(0, 1));
    return;
end

classification = localTableStringColumn(tbl, 'classification', "");
acceptable = localTableLogicalColumn(tbl, 'acceptable', false(height(tbl), 1));
success = localTableLogicalColumn(tbl, 'success', false(height(tbl), 1));
isExact = classification == "exact_trim" | success;
isAcceptable = acceptable & ~isExact;
isBorderline = classification == "near_trim_borderline" & ~isExact & ~isAcceptable;
isOther = ~(isExact | isAcceptable | isBorderline);

masks = struct();
masks.isExact = isExact;
masks.isAcceptable = isAcceptable;
masks.isBorderline = isBorderline;
masks.isOther = isOther;
end

function localScatterMasterByClassification(tbl, masks)
hold on;
if any(masks.isOther)
    scatter(tbl.vinf_mps(masks.isOther), tbl.tilt_deg(masks.isOther), 20, ...
        [0.70 0.70 0.70], 'x', 'LineWidth', 0.9, 'DisplayName', 'Failed / not usable');
end
if any(masks.isBorderline)
    scatter(tbl.vinf_mps(masks.isBorderline), tbl.tilt_deg(masks.isBorderline), 28, ...
        [0.95 0.55 0.15], '^', 'filled', 'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.2, 'DisplayName', 'Borderline near-trim');
end
if any(masks.isAcceptable)
    scatter(tbl.vinf_mps(masks.isAcceptable), tbl.tilt_deg(masks.isAcceptable), 30, ...
        [0.98 0.82 0.18], 's', 'filled', 'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.2, 'DisplayName', 'Acceptable near-trim');
end
if any(masks.isExact)
    scatter(tbl.vinf_mps(masks.isExact), tbl.tilt_deg(masks.isExact), 26, ...
        [0.12 0.60 0.18], 'o', 'filled', 'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.2, 'DisplayName', 'Exact trim');
end
end

function localOverlayController(controllerTbl, labelPoints)
if isempty(controllerTbl)
    return;
end

hold on;
scatter(controllerTbl.vinf_mps, controllerTbl.tilt_deg, 76, ...
    'd', 'MarkerFaceColor', [0.05 0.35 0.90], 'MarkerEdgeColor', 'k', ...
    'LineWidth', 0.8, 'DisplayName', 'Controller schedule');

if ~labelPoints
    return;
end

for i = 1:height(controllerTbl)
    text(controllerTbl.vinf_mps(i) + 0.45, controllerTbl.tilt_deg(i) + 0.35, ...
        sprintf('%d', i), 'FontSize', 7, 'Color', [0.05 0.35 0.90], 'Clipping', 'on');
end
end

function localDrawAnchors()
scatter(0.0, 0.0, 140, 'd', 'MarkerFaceColor', [0.10 0.35 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'DisplayName', 'Hover anchor');
scatter(70.0, 90.0, 140, 'p', 'MarkerFaceColor', [0.90 0.90 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'DisplayName', 'Cruise anchor');
end

function localPlotBackgroundPropMap(tbl, varName)
if isempty(tbl) || ~ismember(varName, tbl.Properties.VariableNames)
    return;
end
mask = isfinite(tbl.vinf_mps) & isfinite(tbl.(varName));
if ~any(mask)
    return;
end
scatter(tbl.vinf_mps(mask), tbl.(varName)(mask), 14, [0.72 0.72 0.72], 'x', ...
    'LineWidth', 0.8, 'MarkerEdgeAlpha', 0.55, 'DisplayName', 'Full DB background');
end

function localScatterMasterPropByClassification(tbl, masks, varName)
if isempty(tbl) || ~ismember(varName, tbl.Properties.VariableNames)
    return;
end
if any(masks.isOther)
    scatter(tbl.vinf_mps(masks.isOther), tbl.(varName)(masks.isOther), 20, ...
        [0.70 0.70 0.70], 'x', 'LineWidth', 0.9, 'DisplayName', 'Failed / not usable');
end
if any(masks.isBorderline)
    scatter(tbl.vinf_mps(masks.isBorderline), tbl.(varName)(masks.isBorderline), 30, ...
        [0.95 0.55 0.15], '^', 'filled', 'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.2, 'DisplayName', 'Borderline near-trim');
end
if any(masks.isAcceptable)
    scatter(tbl.vinf_mps(masks.isAcceptable), tbl.(varName)(masks.isAcceptable), 32, ...
        [0.98 0.82 0.18], 's', 'filled', 'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.2, 'DisplayName', 'Acceptable near-trim');
end
if any(masks.isExact)
    scatter(tbl.vinf_mps(masks.isExact), tbl.(varName)(masks.isExact), 26, ...
        [0.12 0.60 0.18], 'o', 'filled', 'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.2, 'DisplayName', 'Exact trim');
end
end

function localOverlayControllerProp(controllerTbl, varName)
if isempty(controllerTbl) || ~ismember(varName, controllerTbl.Properties.VariableNames)
    return;
end
mask = isfinite(controllerTbl.vinf_mps) & isfinite(controllerTbl.(varName));
if ~any(mask)
    return;
end
scatter(controllerTbl.vinf_mps(mask), controllerTbl.(varName)(mask), 72, ...
    'd', 'MarkerFaceColor', [0.05 0.35 0.90], 'MarkerEdgeColor', 'k', ...
    'LineWidth', 0.8, 'DisplayName', 'Controller schedule');
end

function localScatterMetric(tbl, values, labelText, style)
mask = isfinite(tbl.vinf_mps) & isfinite(tbl.tilt_deg) & isfinite(values);
scatter(tbl.vinf_mps(mask), tbl.tilt_deg(mask), 32, values(mask), 'filled', 'MarkerEdgeColor', 'k');
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
cb = colorbar;
cb.Label.String = labelText;
colormap(gca, parula);
localFinishColorbar(cb, style);
localFinishAxes(gca, style, '');
end

function name = localBuildOutputName(prefix, baseName)
prefix = char(string(prefix));
if isempty(strtrim(prefix))
    name = baseName;
else
    name = sprintf('%s_%s', localSanitizeFileComponent(prefix), baseName);
end
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

function style = localReportStyle()
style = struct();
style.gridColor = [0.86 0.86 0.86];
style.axisColor = [0.15 0.15 0.15];
style.figureColor = 'w';
style.axesColor = 'w';
style.fontName = 'Helvetica';
style.fontSize = 10;
style.titleFontSize = 13;
style.labelFontSize = 11;
style.legendFontSize = 12;
style.lineWidth = 0.8;
end

function localConfigureFigure(fig, style)
fig.Color = style.figureColor;
fig.InvertHardcopy = 'off';
end

function localFinishAxes(ax, style, xLabelText)
grid(ax, 'on');
ax.GridColor = style.gridColor;
ax.GridAlpha = 0.55;
ax.MinorGridAlpha = 0.35;
ax.XColor = style.axisColor;
ax.YColor = style.axisColor;
ax.Box = 'on';
ax.LineWidth = style.lineWidth;
ax.FontName = style.fontName;
ax.FontSize = style.fontSize;
ax.Color = style.axesColor;
ax.Title.Color = 'k';
ax.Title.FontSize = style.titleFontSize;
ax.XLabel.Color = 'k';
ax.XLabel.FontSize = style.labelFontSize;
ax.YLabel.Color = 'k';
ax.YLabel.FontSize = style.labelFontSize;
if nargin >= 3 && ~isempty(xLabelText)
    xlabel(ax, xLabelText);
end
end

function localFinishLegend(leg, style)
if isempty(leg) || ~isgraphics(leg)
    return;
end
leg.Box = 'off';
leg.TextColor = 'k';
leg.Color = 'none';
leg.FontName = 'Helvetica';
leg.FontSize = style.legendFontSize;
end

function localFinishColorbar(cb, style)
if isempty(cb) || ~isgraphics(cb)
    return;
end
cb.Color = 'k';
cb.FontName = style.fontName;
cb.FontSize = style.fontSize;
cb.Label.Color = 'k';
end

function out = localAsLogical(col)
if isempty(col)
    out = false(0, 1);
elseif islogical(col)
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
    out = logical(col);
end
out = logical(out);
out = out(:);
end

function out = localTableLogicalColumn(tbl, varName, fallback)
if istable(tbl) && ismember(varName, tbl.Properties.VariableNames)
    out = localAsLogical(tbl.(varName));
else
    out = logical(fallback);
    out = out(:);
end
end

function out = localTableStringColumn(tbl, varName, fallback)
if istable(tbl) && ismember(varName, tbl.Properties.VariableNames)
    out = string(tbl.(varName));
else
    out = repmat(string(fallback), height(tbl), 1);
end
out = out(:);
end

function name = localSanitizeFileComponent(rawName)
name = regexprep(char(string(rawName)), '[^A-Za-z0-9_-]+', '_');
name = regexprep(name, '_+', '_');
name = regexprep(name, '^_|_$', '');
end
