function out = plot_indi_surface_map_grid_preview(opts)
%PLOT_INDI_SURFACE_MAP_GRID_PREVIEW Plot sampled INDI map grid points.
%
% This diagnostic reads the combined INDI map database and visualizes the
% actual Vinf-alpha-delta sampling grids for one or more database entries.

if nargin < 1 || isempty(opts)
    opts = struct();
end

repoRoot = fileparts(fileparts(mfilename('fullpath')));
opts = localApplyDefaults(opts, repoRoot);
if exist(opts.outputDir, 'dir') ~= 7
    mkdir(opts.outputDir);
end

items = localLoadMapItems(opts);
fig = figure('Name', 'INDI Surface Map Grid Preview', 'Color', 'w', ...
    'Position', [80, 80, 1500, 920]);
layout = tiledlayout(fig, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, 'INDI Surface Effectiveness Map Sampling Points', ...
    'Color', 'k', 'FontWeight', 'bold');
subtitle(layout, localRepoRelativePath(opts.databasePath), ...
    'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);

colors = lines(max(numel(items), 1));
entryLabels = strings(numel(items), 1);

ax3d = nexttile(layout, [2, 2]);
hold(ax3d, 'on');
for k = 1:numel(items)
    mapGrid = items(k).map.grid;
    [V, A, D] = ndgrid(mapGrid.vinf_mps, mapGrid.alpha_deg, mapGrid.delta_deg);
    entryLabels(k) = string(items(k).label);
    scatter3(ax3d, V(:), A(:), D(:), opts.markerSize, colors(k, :), ...
        'filled', 'MarkerFaceAlpha', opts.markerAlpha, ...
        'MarkerEdgeAlpha', opts.markerAlpha);
end
hold(ax3d, 'off');
grid(ax3d, 'on');
view(ax3d, 38, 24);
xlabel(ax3d, 'V_\infty (m/s)');
ylabel(ax3d, '\alpha (deg)');
zlabel(ax3d, '\delta (deg)');
title(ax3d, 'Actual sampled grids');
legend(ax3d, entryLabels, 'Interpreter', 'none', 'Location', 'bestoutside');

axVA = nexttile(layout, 3);
hold(axVA, 'on');
for k = 1:numel(items)
    mapGrid = items(k).map.grid;
    [V, A] = ndgrid(mapGrid.vinf_mps, mapGrid.alpha_deg);
    scatter(axVA, V(:), A(:), opts.markerSize, colors(k, :), ...
        'filled', 'MarkerFaceAlpha', opts.markerAlpha, ...
        'MarkerEdgeAlpha', opts.markerAlpha);
end
hold(axVA, 'off');
grid(axVA, 'on');
xlabel(axVA, 'V_\infty (m/s)');
ylabel(axVA, '\alpha (deg)');
title(axVA, 'V_\infty-\alpha coverage');

axVD = nexttile(layout, 6);
hold(axVD, 'on');
for k = 1:numel(items)
    mapGrid = items(k).map.grid;
    [V, D] = ndgrid(mapGrid.vinf_mps, mapGrid.delta_deg);
    scatter(axVD, V(:), D(:), opts.markerSize, colors(k, :), ...
        'filled', 'MarkerFaceAlpha', opts.markerAlpha, ...
        'MarkerEdgeAlpha', opts.markerAlpha);
end
hold(axVD, 'off');
grid(axVD, 'on');
xlabel(axVD, 'V_\infty (m/s)');
ylabel(axVD, '\delta (deg)');
title(axVD, 'V_\infty-\delta coverage');

out = struct();
out.files = strings(0, 1);
out.entries = struct([]);
for k = 1:numel(items)
    mapGrid = items(k).map.grid;
    out.entries(k).name = items(k).label;
    out.entries(k).grid_size = [numel(mapGrid.vinf_mps), ...
        numel(mapGrid.alpha_deg), numel(mapGrid.delta_deg)];
    out.entries(k).points = prod(out.entries(k).grid_size);
end
out.png_path = fullfile(opts.outputDir, 'indi_surface_map_grid_preview.png');
out.fig_path = fullfile(opts.outputDir, 'indi_surface_map_grid_preview.fig');
out.files = [string(out.png_path); string(out.fig_path)];

localFinalizeFigure(fig);
print(fig, out.png_path, '-dpng', sprintf('-r%d', opts.resolutionDpi));
savefig(fig, out.fig_path);
if opts.closeFigure
    close(fig);
end
assignin('base', 'indi_surface_map_grid_preview', out);

fprintf('Saved INDI sampled-grid preview:\n');
fprintf('  %s\n', out.png_path);
fprintf('  %s\n', out.fig_path);
for k = 1:numel(out.entries)
    fprintf('  %-36s [%d %d %d] = %d points/surface\n', ...
        out.entries(k).name, out.entries(k).grid_size, out.entries(k).points);
end
end

function opts = localApplyDefaults(opts, repoRoot)
if ~isfield(opts, 'databasePath') || isempty(opts.databasePath)
    opts.databasePath = fullfile(repoRoot, 'databases', ...
        'indi_surface_effectiveness_maps_polar_fast.mat');
end
if ~isfield(opts, 'databaseEntryNames') || isempty(opts.databaseEntryNames)
    opts.databaseEntryNames = localDatabaseEntryNames(opts.databasePath);
end
if ~isfield(opts, 'outputDir') || isempty(opts.outputDir)
    opts.outputDir = fullfile(repoRoot, 'workspace_plots', ...
        'indi_surface_map_slices');
end
if ~isfield(opts, 'resolutionDpi') || isempty(opts.resolutionDpi)
    opts.resolutionDpi = 180;
end
if ~isfield(opts, 'markerSize') || isempty(opts.markerSize)
    opts.markerSize = 8;
end
if ~isfield(opts, 'markerAlpha') || isempty(opts.markerAlpha)
    opts.markerAlpha = 0.45;
end
if ~isfield(opts, 'closeFigure') || isempty(opts.closeFigure)
    opts.closeFigure = false;
end
end

function items = localLoadMapItems(opts)
loaded = load(opts.databasePath, 'indiMaps');
names = string({loaded.indiMaps.entries.name});
items = struct('label', {}, 'map', {});
for k = 1:numel(opts.databaseEntryNames)
    entryName = string(opts.databaseEntryNames{k});
    idx = find(names == entryName, 1);
    if isempty(idx)
        error('plot_indi_surface_map_grid_preview:MissingDatabaseEntry', ...
            'Combined INDI map database has no entry named "%s".', entryName);
    end
    items(end + 1).label = char(entryName); %#ok<AGROW>
    items(end).map = loaded.indiMaps.entries(idx).map;
end
end

function entryNames = localDatabaseEntryNames(databasePath)
loaded = load(databasePath, 'indiMaps');
entryNames = cellstr(string({loaded.indiMaps.entries.name}));
end

function relPath = localRepoRelativePath(absPath)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
relPath = strrep(absPath, [repoRoot filesep], '');
relPath = strrep(relPath, filesep, '/');
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
        'MinorGridColor', [0.75 0.75 0.75]);
    if isprop(axesHandles(k), 'Toolbar') && ~isempty(axesHandles(k).Toolbar)
        axesHandles(k).Toolbar.Visible = 'off';
    end
end

textHandles = findall(fig, 'Type', 'text');
for k = 1:numel(textHandles)
    set(textHandles(k), 'Color', 'k');
end
end
