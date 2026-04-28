function out = plot_indi_surface_presentation_figures(opts)
%PLOT_INDI_SURFACE_PRESENTATION_FIGURES Make presentation INDI map figures.
%
% Generates:
%   1. 3D sampled-grid cloud for all database entries.
%   2. Control-effectiveness surface at a fixed Vinf.
%   3. Stall-boundary allocator heatmap at a fixed Vinf.
%   4. Effectiveness-vs-delta curves across selected alpha values.

if nargin < 1 || isempty(opts)
    opts = struct();
end

repoRoot = fileparts(fileparts(mfilename('fullpath')));
opts = localApplyDefaults(opts, repoRoot);
if exist(opts.outputDir, 'dir') ~= 7
    mkdir(opts.outputDir);
end

db = load(opts.databasePath, 'indiMaps');
entries = db.indiMaps.entries;
[map, mapLabel] = localGetEntry(entries, opts.effectivenessEntryName);
[~, iV] = min(abs(map.grid.vinf_mps(:) - opts.targetVinf_mps));
vinf = map.grid.vinf_mps(iV);

out = struct();
out.outputDir = opts.outputDir;
out.databasePath = opts.databasePath;
out.effectivenessEntryName = mapLabel;
out.targetVinf_mps = vinf;
out.files = strings(0, 1);

if opts.plotSet == "delta_curves"
    for k = 1:numel(opts.curveVinf_mps)
        [~, iVCurve] = min(abs(map.grid.vinf_mps(:) - opts.curveVinf_mps(k)));
        vinfCurve = map.grid.vinf_mps(iVCurve);
        [fig, pngPath, figPath] = localPlotEffectivenessDeltaCurves( ...
            map, mapLabel, vinfCurve, iVCurve, opts);
        out.files = localSaveFigure(fig, pngPath, figPath, opts, out.files);
    end
    assignin('base', 'indi_surface_presentation_figures', out);
    localPrintOutputSummary(out);
    return;
end

if opts.plotSet == "surfaces"
    for k = 1:numel(opts.surfaceVinf_mps)
        [~, iVSurface] = min(abs(map.grid.vinf_mps(:) - opts.surfaceVinf_mps(k)));
        vinfSurface = map.grid.vinf_mps(iVSurface);
        [fig, pngPath, figPath] = localPlotEffectivenessSurface( ...
            map, mapLabel, vinfSurface, iVSurface, opts);
        out.files = localSaveFigure(fig, pngPath, figPath, opts, out.files);
    end
    assignin('base', 'indi_surface_presentation_figures', out);
    localPrintOutputSummary(out);
    return;
end

[fig, pngPath, figPath] = localPlotSamplingCloud(entries, opts);
out.files = localSaveFigure(fig, pngPath, figPath, opts, out.files);

[fig, pngPath, figPath] = localPlotEffectivenessSurface(map, mapLabel, vinf, iV, opts);
out.files = localSaveFigure(fig, pngPath, figPath, opts, out.files);

[fig, pngPath, figPath] = localPlotStallBoundaryHeatmap(map, mapLabel, vinf, iV, opts);
out.files = localSaveFigure(fig, pngPath, figPath, opts, out.files);

[fig, pngPath, figPath] = localPlotEffectivenessDeltaCurves(map, mapLabel, vinf, iV, opts);
out.files = localSaveFigure(fig, pngPath, figPath, opts, out.files);

assignin('base', 'indi_surface_presentation_figures', out);
localPrintOutputSummary(out);
end

function localPrintOutputSummary(out)
if isempty(out.files)
    fprintf('Opened INDI presentation figures without saving files.\n');
    return;
end
fprintf('Saved INDI presentation figures:\n');
for k = 1:numel(out.files)
    fprintf('  %s\n', out.files(k));
end
end

function opts = localApplyDefaults(opts, repoRoot)
if ~isfield(opts, 'databasePath') || isempty(opts.databasePath)
    opts.databasePath = fullfile(repoRoot, 'databases', ...
        'indi_surface_effectiveness_maps_polar_fast.mat');
end
if ~isfield(opts, 'outputDir') || isempty(opts.outputDir)
    opts.outputDir = fullfile(repoRoot, 'workspace_plots', ...
        'indi_surface_presentation');
end
if ~isfield(opts, 'effectivenessEntryName') || isempty(opts.effectivenessEntryName)
    opts.effectivenessEntryName = 'stall_boundary_delta_alpha_dense';
end
if ~isfield(opts, 'targetVinf_mps') || isempty(opts.targetVinf_mps)
    opts.targetVinf_mps = 40;
end
if ~isfield(opts, 'plotSet') || isempty(opts.plotSet)
    opts.plotSet = "all";
else
    opts.plotSet = lower(string(opts.plotSet));
end
if ~isfield(opts, 'saveFigures') || isempty(opts.saveFigures)
    opts.saveFigures = true;
end
if ~isfield(opts, 'curveVinf_mps') || isempty(opts.curveVinf_mps)
    opts.curveVinf_mps = opts.targetVinf_mps;
end
if ~isfield(opts, 'surfaceVinf_mps') || isempty(opts.surfaceVinf_mps)
    opts.surfaceVinf_mps = opts.targetVinf_mps;
end
if ~isfield(opts, 'showDatabaseSubtitle') || isempty(opts.showDatabaseSubtitle)
    opts.showDatabaseSubtitle = false;
end
if ~isfield(opts, 'resolutionDpi') || isempty(opts.resolutionDpi)
    opts.resolutionDpi = 220;
end
if ~isfield(opts, 'closeFigure') || isempty(opts.closeFigure)
    opts.closeFigure = false;
end
if ~isfield(opts, 'cloudMarkerSize') || isempty(opts.cloudMarkerSize)
    opts.cloudMarkerSize = 5;
end
if ~isfield(opts, 'cloudMarkerAlpha') || isempty(opts.cloudMarkerAlpha)
    opts.cloudMarkerAlpha = 0.23;
end
if ~isfield(opts, 'cloudEntries') || isempty(opts.cloudEntries)
    opts.cloudEntries = {};
end
if ~isfield(opts, 'curveAlpha_deg') || isempty(opts.curveAlpha_deg)
    opts.curveAlpha_deg = [-2.5, 0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5];
end
end

function [map, label] = localGetEntry(entries, entryName)
names = string({entries.name});
idx = find(names == string(entryName), 1);
if isempty(idx)
    error('plot_indi_surface_presentation_figures:MissingEntry', ...
        'Combined INDI map database has no entry named "%s".', entryName);
end
map = entries(idx).map;
label = entries(idx).name;
end

function [fig, pngPath, figPath] = localPlotSamplingCloud(entries, opts)
if isempty(opts.cloudEntries)
    selected = true(1, numel(entries));
else
    selectedNames = string(opts.cloudEntries);
    selected = ismember(string({entries.name}), selectedNames);
end
items = entries(selected);

fig = figure('Name', 'INDI Presentation - Sampling Cloud', ...
    'Color', 'w', 'Position', [80, 80, 1350, 900]);
ax = axes(fig);
hold(ax, 'on');

colors = localCampaignColors(numel(items));
labels = strings(numel(items), 1);
totalPoints = 0;
for k = 1:numel(items)
    mapGrid = items(k).map.grid;
    [V, A, D] = ndgrid(mapGrid.vinf_mps, mapGrid.alpha_deg, mapGrid.delta_deg);
    totalPoints = totalPoints + numel(V);
    labels(k) = string(items(k).name);
    scatter3(ax, V(:), A(:), D(:), opts.cloudMarkerSize, colors(k, :), ...
        'filled', 'MarkerFaceAlpha', opts.cloudMarkerAlpha, ...
        'MarkerEdgeAlpha', opts.cloudMarkerAlpha);
end

hold(ax, 'off');
grid(ax, 'on');
box(ax, 'on');
view(ax, 42, 24);
xlabel(ax, 'Airspeed V_\infty (m/s)');
ylabel(ax, 'Angle of attack \alpha (deg)');
zlabel(ax, 'Surface deflection \delta (deg)');
title(ax, 'INDI Surface-Effectiveness Map Coverage', ...
    'FontWeight', 'bold');
subtitle(ax, sprintf('%d sampled grid points per surface across %d campaigns', ...
    totalPoints, numel(items)));
legend(ax, labels, 'Interpreter', 'none', 'Location', 'eastoutside');
axis(ax, 'tight');
localFinalizeFigure(fig);

pngPath = fullfile(opts.outputDir, 'indi_presentation_sampling_cloud.png');
figPath = fullfile(opts.outputDir, 'indi_presentation_sampling_cloud.fig');
end

function [fig, pngPath, figPath] = localPlotEffectivenessSurface(map, label, vinf, iV, opts)
alpha = map.grid.alpha_deg(:);
delta = map.grid.delta_deg(:);
[D, A] = meshgrid(delta, alpha);

flapFz = squeeze(map.flap.dF_drad_N_per_rad(iV, :, :, 3));
elevMy = squeeze(map.elevator.dM_drad_Nm_per_rad(iV, :, :, 2));

fig = figure('Name', 'INDI Presentation - Effectiveness Surface', ...
    'Color', 'w', 'Position', [70, 70, 1500, 760]);
layout = tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('Control-Effectiveness Surfaces at V_\\infty = %.1f m/s', vinf), ...
    'FontWeight', 'bold', 'Color', 'k');
if opts.showDatabaseSubtitle
    subtitle(layout, sprintf('database entry: %s', label), ...
        'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);
end

ax = nexttile(layout);
surf(ax, D, A, flapFz, 'EdgeColor', 'none', 'FaceAlpha', 0.97);
localStyleSurfaceAxes(ax, 'Flap vertical-force effectiveness', ...
    'dF_z/d\delta (N/rad)');

ax = nexttile(layout);
surf(ax, D, A, elevMy, 'EdgeColor', 'none', 'FaceAlpha', 0.97);
localStyleSurfaceAxes(ax, 'Elevator pitch-moment effectiveness', ...
    'dM_y/d\delta (N*m/rad)');

localFinalizeFigure(fig);
if opts.plotSet == "surfaces" && numel(opts.surfaceVinf_mps) > 1
    fileToken = localNumberToken(vinf);
    pngName = sprintf('indi_presentation_effectiveness_surface_V%s.png', fileToken);
    figName = sprintf('indi_presentation_effectiveness_surface_V%s.fig', fileToken);
else
    pngName = 'indi_presentation_effectiveness_surface.png';
    figName = 'indi_presentation_effectiveness_surface.fig';
end
pngPath = fullfile(opts.outputDir, pngName);
figPath = fullfile(opts.outputDir, figName);
end

function localStyleSurfaceAxes(ax, plotTitle, zText)
grid(ax, 'on');
box(ax, 'on');
colormap(ax, turbo);
colorbar(ax);
view(ax, 38, 29);
xlabel(ax, 'Surface deflection \delta (deg)');
ylabel(ax, 'Angle of attack \alpha (deg)');
zlabel(ax, zText);
title(ax, plotTitle, 'FontWeight', 'bold');
axis(ax, 'tight');
end

function [fig, pngPath, figPath] = localPlotStallBoundaryHeatmap(map, label, vinf, iV, opts)
alpha = map.grid.alpha_deg(:);
delta = map.grid.delta_deg(:);
[sigmaMin, conditionNumber] = localAllocatorMetrics(map, iV);
logCondition = log10(conditionNumber);

fig = figure('Name', 'INDI Presentation - Stall Boundary Heatmap', ...
    'Color', 'w', 'Position', [70, 70, 1500, 760]);
layout = tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('Allocator Health Near the Stall Boundary at V_\\infty = %.1f m/s', vinf), ...
    'FontWeight', 'bold', 'Color', 'k');
if opts.showDatabaseSubtitle
    subtitle(layout, sprintf('database entry: %s', label), ...
        'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);
end

ax = nexttile(layout);
imagesc(ax, delta, alpha, sigmaMin);
set(ax, 'YDir', 'normal');
grid(ax, 'on');
colormap(ax, turbo);
colorbar(ax);
xlabel(ax, 'Surface deflection \delta (deg)');
ylabel(ax, 'Angle of attack \alpha (deg)');
title(ax, 'Weakest control direction, \sigma_{min}', 'FontWeight', 'bold');

ax = nexttile(layout);
imagesc(ax, delta, alpha, logCondition);
set(ax, 'YDir', 'normal');
grid(ax, 'on');
colormap(ax, turbo);
cb = colorbar(ax);
cb.Label.String = 'log_{10}(\kappa)';
xlabel(ax, 'Surface deflection \delta (deg)');
ylabel(ax, 'Angle of attack \alpha (deg)');
title(ax, 'Allocator conditioning, log_{10}(\kappa)', 'FontWeight', 'bold');

localFinalizeFigure(fig);
pngPath = fullfile(opts.outputDir, 'indi_presentation_stall_boundary_heatmap.png');
figPath = fullfile(opts.outputDir, 'indi_presentation_stall_boundary_heatmap.fig');
end

function [sigmaMin, conditionNumber] = localAllocatorMetrics(map, iV)
nAlpha = numel(map.grid.alpha_deg);
nDelta = numel(map.grid.delta_deg);
sigmaMin = zeros(nAlpha, nDelta);
conditionNumber = zeros(nAlpha, nDelta);

for iA = 1:nAlpha
    for iD = 1:nDelta
        G = [ ...
            map.flap.dF_drad_N_per_rad(iV, iA, iD, 3), ...
            map.elevator.dF_drad_N_per_rad(iV, iA, iD, 3); ...
            map.flap.dM_drad_Nm_per_rad(iV, iA, iD, 2), ...
            map.elevator.dM_drad_Nm_per_rad(iV, iA, iD, 2)];
        s = svd(double(G));
        sigmaMin(iA, iD) = min(s);
        if min(s) <= eps(max(s))
            conditionNumber(iA, iD) = inf;
        else
            conditionNumber(iA, iD) = max(s) / min(s);
        end
    end
end
end

function [fig, pngPath, figPath] = localPlotEffectivenessDeltaCurves(map, label, vinf, iV, opts)
alphaIdx = localNearestIndices(map.grid.alpha_deg, opts.curveAlpha_deg);
alpha = map.grid.alpha_deg(alphaIdx);
delta = map.grid.delta_deg(:);

flapFz = squeeze(map.flap.dF_drad_N_per_rad(iV, alphaIdx, :, 3)).';
flapMy = squeeze(map.flap.dM_drad_Nm_per_rad(iV, alphaIdx, :, 2)).';
elevFz = squeeze(map.elevator.dF_drad_N_per_rad(iV, alphaIdx, :, 3)).';
elevMy = squeeze(map.elevator.dM_drad_Nm_per_rad(iV, alphaIdx, :, 2)).';

fig = figure('Name', 'INDI Presentation - Effectiveness vs Delta', ...
    'Color', 'w', 'Position', [70, 70, 1500, 950]);
layout = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('Control Effectiveness vs. Surface Deflection at V_\\infty = %.1f m/s', vinf), ...
    'FontWeight', 'bold', 'Color', 'k');
if opts.showDatabaseSubtitle
    subtitle(layout, sprintf('database entry: %s', label), ...
        'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);
end

legendLabels = compose('\\alpha = %.1f deg', alpha);
colors = turbo(numel(alpha));

localCurvePanel(nexttile(layout), delta, flapFz, colors, ...
    'Flap vertical-force effectiveness', 'dF_z/d\delta (N/rad)', legendLabels);
localCurvePanel(nexttile(layout), delta, flapMy, colors, ...
    'Flap pitch-moment effectiveness', 'dM_y/d\delta (N*m/rad)', legendLabels);
localCurvePanel(nexttile(layout), delta, elevFz, colors, ...
    'Elevator vertical-force effectiveness', 'dF_z/d\delta (N/rad)', legendLabels);
localCurvePanel(nexttile(layout), delta, elevMy, colors, ...
    'Elevator pitch-moment effectiveness', 'dM_y/d\delta (N*m/rad)', legendLabels);

localFinalizeFigure(fig);
if opts.plotSet == "delta_curves" && numel(opts.curveVinf_mps) > 1
    fileToken = localNumberToken(vinf);
    pngName = sprintf('indi_presentation_effectiveness_vs_delta_V%s.png', fileToken);
    figName = sprintf('indi_presentation_effectiveness_vs_delta_V%s.fig', fileToken);
else
    pngName = 'indi_presentation_effectiveness_vs_delta.png';
    figName = 'indi_presentation_effectiveness_vs_delta.fig';
end
pngPath = fullfile(opts.outputDir, pngName);
figPath = fullfile(opts.outputDir, figName);
end

function localCurvePanel(ax, delta, values, colors, plotTitle, yText, legendLabels)
hold(ax, 'on');
for k = 1:size(values, 2)
    plot(ax, delta, values(:, k), 'LineWidth', 1.8, 'Color', colors(k, :));
end
hold(ax, 'off');
grid(ax, 'on');
box(ax, 'on');
xlabel(ax, 'Surface deflection \delta (deg)');
ylabel(ax, yText);
title(ax, plotTitle, 'FontWeight', 'bold');
legend(ax, legendLabels, 'Location', 'eastoutside');
end

function idx = localNearestIndices(gridValues, requestedValues)
idx = zeros(numel(requestedValues), 1);
for k = 1:numel(requestedValues)
    [~, idx(k)] = min(abs(gridValues(:) - requestedValues(k)));
end
idx = unique(idx, 'stable');
end

function files = localSaveFigure(fig, pngPath, figPath, opts, files)
if ~opts.saveFigures
    if opts.closeFigure
        close(fig);
    end
    return;
end
print(fig, pngPath, '-dpng', sprintf('-r%d', opts.resolutionDpi));
savefig(fig, figPath);
files(end + 1, 1) = string(pngPath);
files(end + 1, 1) = string(figPath);
if opts.closeFigure
    close(fig);
end
end

function token = localNumberToken(value)
token = regexprep(sprintf('%.3g', value), '\.', 'p');
token = regexprep(token, '-', 'm');
end

function colors = localCampaignColors(n)
base = [ ...
    0.0000, 0.4470, 0.7410; ...
    0.8500, 0.3250, 0.0980; ...
    0.9290, 0.6940, 0.1250; ...
    0.4940, 0.1840, 0.5560; ...
    0.4660, 0.6740, 0.1880; ...
    0.3010, 0.7450, 0.9330; ...
    0.6350, 0.0780, 0.1840; ...
    0.2500, 0.2500, 0.2500; ...
    0.0000, 0.6000, 0.5000; ...
    0.7000, 0.4000, 0.9000];
if n <= size(base, 1)
    colors = base(1:n, :);
else
    colors = lines(n);
end
end

function localFinalizeFigure(fig)
axesHandles = findall(fig, 'Type', 'axes');
for k = 1:numel(axesHandles)
    set(axesHandles(k), ...
        'Color', 'w', ...
        'XColor', 'k', ...
        'YColor', 'k', ...
        'ZColor', 'k', ...
        'GridColor', [0.55 0.55 0.55], ...
        'MinorGridColor', [0.75 0.75 0.75], ...
        'FontSize', 12, ...
        'LineWidth', 0.9);
    if isprop(axesHandles(k), 'Toolbar') && ~isempty(axesHandles(k).Toolbar)
        axesHandles(k).Toolbar.Visible = 'off';
    end
end

textHandles = findall(fig, 'Type', 'text');
for k = 1:numel(textHandles)
    set(textHandles(k), 'Color', 'k');
end
end
