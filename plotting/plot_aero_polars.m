function out = plot_aero_polars(opts)
%PLOT_AERO_POLARS Plot the active wing/tail aerodynamic polar database.
%
% Usage:
%   Init_Main
%   plot_aero_polars
%   plot_aero_polars(struct('saveFigures', true))
%
% This reads the runtime polar MAT directly. It does not depend on the
% restored legacy scripts folder or the original Airfoil360 workbook.

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

repoRoot = fileparts(fileparts(mfilename('fullpath')));
if strlength(string(opts.polarDataPath)) == 0
    opts.polarDataPath = fullfile(repoRoot, 'databases', 'aero_polars', ...
        'final_airfoil_polar_tables.mat');
end
if exist(opts.polarDataPath, 'file') ~= 2
    error('plot_aero_polars:MissingPolarData', ...
        'Could not find polar data MAT file: %s', opts.polarDataPath);
end

polarData = load(opts.polarDataPath, 'wingPolar', 'tailPolar');
localValidatePolar(polarData.wingPolar, 'wingPolar');
localValidatePolar(polarData.tailPolar, 'tailPolar');
style = localReportStyle();

out = struct();
out.polarDataPath = opts.polarDataPath;
out.wingPolar = polarData.wingPolar;
out.tailPolar = polarData.tailPolar;
out.figures = struct();
out.savedFiles = strings(0, 1);

out.figures.fullRange = localPlotCoefficientFigure( ...
    polarData.wingPolar, polarData.tailPolar, [-180 180], ...
    'Full 360-Degree Aero Polars', opts.figureVisibility, style);

out.figures.transitionRange = localPlotCoefficientFigure( ...
    polarData.wingPolar, polarData.tailPolar, opts.transitionAlphaLimits_deg, ...
    'Transition-Relevant Aero Polars', opts.figureVisibility, style);

out.figures.derived = localPlotDerivedFigure( ...
    polarData.wingPolar, polarData.tailPolar, opts.transitionAlphaLimits_deg, ...
    opts.figureVisibility, style);

if opts.saveFigures
    outDir = opts.outputDir;
    if strlength(string(outDir)) == 0
        outDir = fullfile(repoRoot, 'workspace_plots');
    end
    if exist(outDir, 'dir') ~= 7
        mkdir(outDir);
    end

    out.savedFiles = [ ...
        localSaveFigure(out.figures.fullRange, fullfile(outDir, 'aero_polars_full_range'), opts); ...
        localSaveFigure(out.figures.transitionRange, fullfile(outDir, 'aero_polars_transition_range'), opts); ...
        localSaveFigure(out.figures.derived, fullfile(outDir, 'aero_polars_derived'), opts)];
end

assignin('base', 'aeroPolarPlotData', out);

fprintf('Loaded aero polar data: %s\n', opts.polarDataPath);
fprintf('Wing alpha range: %.1f to %.1f deg (%d points)\n', ...
    min(polarData.wingPolar.alpha_deg), max(polarData.wingPolar.alpha_deg), ...
    numel(polarData.wingPolar.alpha_deg));
fprintf('Tail alpha range: %.1f to %.1f deg (%d points)\n', ...
    min(polarData.tailPolar.alpha_deg), max(polarData.tailPolar.alpha_deg), ...
    numel(polarData.tailPolar.alpha_deg));
if opts.saveFigures
    fprintf('Saved aero polar figures under: %s\n', outDir);
end
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'polarDataPath')
    opts.polarDataPath = "";
end
if ~isfield(opts, 'transitionAlphaLimits_deg') || isempty(opts.transitionAlphaLimits_deg)
    opts.transitionAlphaLimits_deg = [-30 90];
end
if ~isfield(opts, 'figureVisibility') || isempty(opts.figureVisibility)
    opts.figureVisibility = 'on';
end
if ~isfield(opts, 'saveFigures') || isempty(opts.saveFigures)
    opts.saveFigures = false;
end
if ~isfield(opts, 'saveFigFiles') || isempty(opts.saveFigFiles)
    opts.saveFigFiles = false;
end
if ~isfield(opts, 'outputDir')
    opts.outputDir = "";
end
end

function localValidatePolar(polar, name)
requiredFields = {'alpha_deg', 'alpha_rad', 'CL', 'CD', 'Cm'};
if ~isstruct(polar)
    error('plot_aero_polars:BadPolar', '%s must be a struct.', name);
end
for i = 1:numel(requiredFields)
    if ~isfield(polar, requiredFields{i}) || isempty(polar.(requiredFields{i}))
        error('plot_aero_polars:BadPolar', ...
            '%s is missing required field %s.', name, requiredFields{i});
    end
end
end

function fig = localPlotCoefficientFigure(wingPolar, tailPolar, alphaLimits, figureTitle, visibility, style)
fig = figure( ...
    'Name', figureTitle, ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Visible', visibility, ...
    'Position', [80 80 1180 520]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
titleHandle = title(layout, figureTitle, 'FontWeight', 'bold');
titleHandle.Color = 'k';

labels = {'C_L', 'C_D', 'C_m'};
fields = {'CL', 'CD', 'Cm'};
surfaces = {wingPolar, tailPolar};
surfaceNames = {'Wing', 'Tail'};

for col = 1:numel(surfaces)
    ax = nexttile(layout);
    alphaDeg = surfaces{col}.alpha_deg(:);
    hold(ax, 'on');

    for idx = 1:numel(fields)
        values = surfaces{col}.(fields{idx})(:);
        color = style.colorOrder(idx, :);
        plot(ax, alphaDeg, values, ...
            'LineWidth', style.actualLineWidth, ...
            'LineStyle', '-', ...
            'Marker', 'o', ...
            'MarkerSize', style.markerSize, ...
            'MarkerFaceColor', 'w', ...
            'MarkerEdgeColor', color, ...
            'Color', color);
    end

    xlim(ax, alphaLimits);
    xline(ax, 0, style.commandLineStyle, ...
        'Color', [0.45 0.45 0.45], ...
        'LineWidth', style.commandLineWidth, ...
        'HandleVisibility', 'off');
    if alphaLimits(2) >= 60
        xline(ax, 60, '--', ...
            'Color', [0.35 0.35 0.35], ...
            'LineWidth', style.commandLineWidth, ...
            'HandleVisibility', 'off');
    end

    title(ax, surfaceNames{col});
    xlabel(ax, '\alpha (deg)');
    ylabel(ax, 'Coefficient value');
    localAddReportLegend(ax, labels);
    localFinishAxes(ax, style);
end

localAddMetadata(layout, wingPolar, tailPolar, style);
end

function fig = localPlotDerivedFigure(wingPolar, tailPolar, alphaLimits, visibility, style)
fig = figure( ...
    'Name', 'Aero Polar Derived Views', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Visible', visibility, ...
    'Position', [100 100 1180 440]);
layout = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titleHandle = title(layout, 'Aero Polar Derived Views', 'FontWeight', 'bold');
titleHandle.Color = 'k';

wing = localDerivedPolar(wingPolar, alphaLimits);
tail = localDerivedPolar(tailPolar, alphaLimits);

ax = nexttile(layout);
plot(ax, wing.CD, wing.CL, ...
    'LineWidth', style.actualLineWidth, ...
    'Color', style.colorOrder(1, :));
hold(ax, 'on');
plot(ax, tail.CD, tail.CL, ...
    'LineWidth', style.actualLineWidth, ...
    'Color', style.colorOrder(2, :));
xlabel(ax, 'C_D');
ylabel(ax, 'C_L');
title(ax, 'Drag Polar');
localAddReportLegend(ax, {'Wing', 'Tail'});
localFinishAxes(ax, style);

ax = nexttile(layout);
plot(ax, wing.alpha_deg, wing.L_over_D, ...
    'LineWidth', style.actualLineWidth, ...
    'Color', style.colorOrder(1, :));
hold(ax, 'on');
plot(ax, tail.alpha_deg, tail.L_over_D, ...
    'LineWidth', style.actualLineWidth, ...
    'Color', style.colorOrder(2, :));
xline(ax, 0, style.commandLineStyle, ...
    'Color', [0.45 0.45 0.45], ...
    'LineWidth', style.commandLineWidth);
xlabel(ax, '\alpha (deg)');
ylabel(ax, 'C_L / C_D');
title(ax, 'Lift-to-Drag Ratio');
localAddReportLegend(ax, {'Wing', 'Tail'});
localFinishAxes(ax, style);

ax = nexttile(layout);
plot(ax, wing.CL, wing.Cm, ...
    'LineWidth', style.actualLineWidth, ...
    'Color', style.colorOrder(1, :));
hold(ax, 'on');
plot(ax, tail.CL, tail.Cm, ...
    'LineWidth', style.actualLineWidth, ...
    'Color', style.colorOrder(2, :));
xlabel(ax, 'C_L');
ylabel(ax, 'C_m');
title(ax, 'Moment Versus Lift');
localAddReportLegend(ax, {'Wing', 'Tail'});
localFinishAxes(ax, style);
end

function derived = localDerivedPolar(polar, alphaLimits)
alphaDeg = polar.alpha_deg(:);
mask = alphaDeg >= alphaLimits(1) & alphaDeg <= alphaLimits(2);
derived = struct();
derived.alpha_deg = alphaDeg(mask);
derived.CL = polar.CL(mask);
derived.CD = polar.CD(mask);
derived.Cm = polar.Cm(mask);
derived.L_over_D = derived.CL ./ max(derived.CD, 1.0e-6);
end

function localAddMetadata(layout, wingPolar, tailPolar, style)
wingLabel = localSurfaceLabel(wingPolar);
tailLabel = localSurfaceLabel(tailPolar);
subtitleHandle = subtitle(layout, sprintf('%s   |   %s', wingLabel, tailLabel), ...
    'Interpreter', 'none', 'FontSize', 10);
subtitleHandle.Color = style.axisColor;
end

function label = localSurfaceLabel(polar)
name = localGetField(polar, 'name', 'surface');
airfoil = localGetField(polar, 'airfoil', 'unknown airfoil');
reynolds = localGetField(polar, 'reynolds', NaN);
if isnan(reynolds)
    label = sprintf('%s: %s', name, airfoil);
else
    label = sprintf('%s: %s, Re=%g', name, airfoil, reynolds);
end
end

function value = localGetField(s, fieldName, fallback)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = fallback;
end
end

function style = localReportStyle()
style = struct();
style.colorOrder = [ ...
    0.00 0.45 0.74; ...
    0.85 0.33 0.10; ...
    0.47 0.67 0.19; ...
    0.49 0.18 0.56; ...
    0.93 0.69 0.13; ...
    0.30 0.75 0.93];
style.actualLineWidth = 1.5;
style.commandLineWidth = 1.2;
style.commandLineStyle = ':';
style.markerSize = 3.0;
style.gridColor = [0.86 0.86 0.86];
style.axisColor = [0.15 0.15 0.15];
end

function localFinishAxes(ax, style)
grid(ax, 'on');
ax.GridColor = style.gridColor;
ax.GridAlpha = 0.55;
ax.MinorGridAlpha = 0.35;
ax.XColor = style.axisColor;
ax.YColor = style.axisColor;
ax.Box = 'on';
ax.LineWidth = 0.8;
ax.FontName = 'Helvetica';
ax.FontSize = 10;
ax.Color = 'w';
ax.Title.Color = 'k';
ax.XLabel.Color = 'k';
ax.YLabel.Color = 'k';
hold(ax, 'off');
end

function localAddReportLegend(ax, labels)
leg = legend(ax, labels, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
end

function savedFiles = localSaveFigure(fig, filenameStem, opts)
pngPath = string(filenameStem) + ".png";
exportgraphics(fig, pngPath, 'Resolution', 180);
savedFiles = string(pngPath);

if opts.saveFigFiles
    figPath = string(filenameStem) + ".fig";
    savefig(fig, figPath);
    savedFiles = [savedFiles; string(figPath)];
end
end
