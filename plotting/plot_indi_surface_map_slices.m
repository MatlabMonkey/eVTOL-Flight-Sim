function out = plot_indi_surface_map_slices(opts)
%PLOT_INDI_SURFACE_MAP_SLICES Plot INDI surface effectiveness map slices.
%
% This diagnostic reads the combined INDI map database. By default it plots
% flap/elevator dFz/ddelta and dMy/ddelta over the Vinf-alpha grid at
% representative surface deflections. Set opts.plotMode = 'control_point' to
% inspect the local surface-control effectiveness versus actuator deflection,
% or 'alpha_sweep_control' to compare allocator metrics over alpha at one Vinf.

if nargin < 1 || isempty(opts)
    opts = struct();
end

repoRoot = fileparts(fileparts(mfilename('fullpath')));
opts = localApplyDefaults(opts, repoRoot);

if exist(opts.outputDir, 'dir') ~= 7
    mkdir(opts.outputDir);
end

out = struct();
out.files = strings(0, 1);
out.maps = struct([]);
mapItems = localLoadMapItems(opts);

for iMap = 1:numel(mapItems)
    map = mapItems(iMap).map;
    mapPath = mapItems(iMap).source;
    label = mapItems(iMap).label;

    switch opts.plotMode
        case "slices"
            pngPath = fullfile(opts.outputDir, sprintf('indi_surface_map_slices_%s.png', label));
            figPath = fullfile(opts.outputDir, sprintf('indi_surface_map_slices_%s.fig', label));
            fig = localPlotOneMap(map, label, mapPath, opts);
        case "control_point"
            pngPath = fullfile(opts.outputDir, sprintf('indi_surface_control_effectiveness_%s.png', label));
            figPath = fullfile(opts.outputDir, sprintf('indi_surface_control_effectiveness_%s.fig', label));
            fig = localPlotControlPoint(map, label, mapPath, opts);
        case "alpha_sweep_control"
            pngPath = fullfile(opts.outputDir, sprintf('indi_surface_alpha_sweep_control_%s.png', label));
            figPath = fullfile(opts.outputDir, sprintf('indi_surface_alpha_sweep_control_%s.fig', label));
            fig = localPlotAlphaSweepControl(map, label, mapPath, opts);
        otherwise
            error('plot_indi_surface_map_slices:UnknownPlotMode', ...
                ['Unknown plotMode "%s". Use "slices", "control_point", ', ...
                'or "alpha_sweep_control".'], opts.plotMode);
    end

    print(fig, pngPath, '-dpng', sprintf('-r%d', opts.resolutionDpi));
    savefig(fig, figPath);
    if opts.closeFigure
        close(fig);
    end

    out.files(end + 1, 1) = string(pngPath);
    out.files(end + 1, 1) = string(figPath);
    out.maps(iMap).map_path = mapPath;
    out.maps(iMap).png_path = pngPath;
    out.maps(iMap).fig_path = figPath;
    out.maps(iMap).delta_slices_deg = localChooseDeltaSlices(map.grid.delta_deg, opts.deltaSlices_deg);
end

assignin('base', 'indi_surface_map_slice_plots', out);

fprintf('Saved INDI surface map slice plots:\n');
for iFile = 1:numel(out.files)
    fprintf('  %s\n', out.files(iFile));
end
end

function opts = localApplyDefaults(opts, repoRoot)
databaseDir = fullfile(repoRoot, 'databases');
if ~isfield(opts, 'databasePath') || isempty(opts.databasePath)
    opts.databasePath = fullfile(databaseDir, 'indi_surface_effectiveness_maps_polar_fast.mat');
end
if ~isfield(opts, 'databaseEntryNames') || isempty(opts.databaseEntryNames)
    opts.databaseEntryNames = localDatabaseEntryNames(opts.databasePath);
end
if ~isfield(opts, 'plotMode') || isempty(opts.plotMode)
    opts.plotMode = "slices";
else
    opts.plotMode = lower(string(opts.plotMode));
end
if ~isfield(opts, 'deltaSlices_deg') || isempty(opts.deltaSlices_deg)
    opts.deltaSlices_deg = [-20, 0, 20];
end
if ~isfield(opts, 'resolutionDpi') || isempty(opts.resolutionDpi)
    opts.resolutionDpi = 180;
end
if ~isfield(opts, 'outputDir') || isempty(opts.outputDir)
    opts.outputDir = fullfile(repoRoot, 'workspace_plots', 'indi_surface_map_slices');
end
if ~isfield(opts, 'closeFigure') || isempty(opts.closeFigure)
    opts.closeFigure = false;
end
if ~isfield(opts, 'targetVinf_mps') || isempty(opts.targetVinf_mps)
    opts.targetVinf_mps = 40;
end
if ~isfield(opts, 'targetAlpha_deg') || isempty(opts.targetAlpha_deg)
    opts.targetAlpha_deg = 10;
end
if ~isfield(opts, 'sweepAlpha_deg') || isempty(opts.sweepAlpha_deg)
    opts.sweepAlpha_deg = [];
end
if ~isfield(opts, 'mass_kg') || isempty(opts.mass_kg)
    opts.mass_kg = NaN;
end
if ~isfield(opts, 'Iyy_kgm2') || isempty(opts.Iyy_kgm2)
    opts.Iyy_kgm2 = NaN;
end
end

function mapItems = localLoadMapItems(opts)
mapItems = struct('label', {}, 'source', {}, 'map', {});

loadedDb = load(opts.databasePath, 'indiMaps');
names = string({loadedDb.indiMaps.entries.name});
for iEntry = 1:numel(opts.databaseEntryNames)
    entryName = string(opts.databaseEntryNames{iEntry});
    idx = find(names == entryName, 1);
    if isempty(idx)
        error('plot_indi_surface_map_slices:MissingDatabaseEntry', ...
            'Combined INDI map database has no entry named "%s".', entryName);
    end

    mapItems(end + 1).label = char(regexprep(entryName, '[^A-Za-z0-9_]', '_')); %#ok<AGROW>
    mapItems(end).source = sprintf('%s#%s', opts.databasePath, entryName);
    mapItems(end).map = loadedDb.indiMaps.entries(idx).map;
end
end

function entryNames = localDatabaseEntryNames(databasePath)
loadedDb = load(databasePath, 'indiMaps');
entryNames = cellstr(string({loadedDb.indiMaps.entries.name}));
end

function fig = localPlotOneMap(map, label, mapPath, opts)
deltaSlices = localChooseDeltaSlices(map.grid.delta_deg, opts.deltaSlices_deg);
sliceIdx = localNearestIndices(map.grid.delta_deg, deltaSlices);

series = localSeries(map);
fig = figure('Name', sprintf('INDI Surface Map Slices - %s', label), ...
    'Color', 'w', 'Position', [50, 50, 1500, 1050]);
layout = tiledlayout(fig, numel(series), numel(sliceIdx), ...
    'Padding', 'compact', 'TileSpacing', 'compact');

title(layout, sprintf('INDI Surface Effectiveness Slices: %s', label), ...
    'Interpreter', 'none', 'FontWeight', 'bold', 'Color', 'k');
subtitle(layout, localRepoRelativePath(mapPath), 'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);

for iSeries = 1:numel(series)
    allValues = series(iSeries).values(:, :, sliceIdx);
    colorLimits = localRobustColorLimits(allValues);

    for iSlice = 1:numel(sliceIdx)
        ax = nexttile(layout);
        if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
            ax.Toolbar.Visible = 'off';
        end
        values = squeeze(series(iSeries).values(:, :, sliceIdx(iSlice))).';
        imagesc(ax, map.grid.vinf_mps, map.grid.alpha_deg, values);
        set(ax, 'YDir', 'normal');
        grid(ax, 'on');
        colormap(ax, turbo);
        clim(ax, colorLimits);

        xlabel(ax, 'V_\infty (m/s)');
        ylabel(ax, '\alpha (deg)');
        title(ax, sprintf('%s, \\delta = %.2g deg', ...
            series(iSeries).title, map.grid.delta_deg(sliceIdx(iSlice))), ...
            'Color', 'k');
        set(ax, 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.22);

        cb = colorbar(ax);
        cb.Label.String = series(iSeries).units;
    end
end
end

function series = localSeries(map)
series = struct([]);

series(1).title = 'flap dF_z/d\delta';
series(1).units = 'N/rad';
series(1).values = map.flap.dF_drad_N_per_rad(:, :, :, 3);

series(2).title = 'flap dM_y/d\delta';
series(2).units = 'N*m/rad';
series(2).values = map.flap.dM_drad_Nm_per_rad(:, :, :, 2);

series(3).title = 'elevator dF_z/d\delta';
series(3).units = 'N/rad';
series(3).values = map.elevator.dF_drad_N_per_rad(:, :, :, 3);

series(4).title = 'elevator dM_y/d\delta';
series(4).units = 'N*m/rad';
series(4).values = map.elevator.dM_drad_Nm_per_rad(:, :, :, 2);
end

function fig = localPlotControlPoint(map, label, mapPath, opts)
[~, iV] = min(abs(map.grid.vinf_mps(:) - opts.targetVinf_mps));
[~, iA] = min(abs(map.grid.alpha_deg(:) - opts.targetAlpha_deg));
vinf = map.grid.vinf_mps(iV);
alpha = map.grid.alpha_deg(iA);
delta = map.grid.delta_deg(:);

flapFz = squeeze(map.flap.dF_drad_N_per_rad(iV, iA, :, 3));
flapMy = squeeze(map.flap.dM_drad_Nm_per_rad(iV, iA, :, 2));
elevFz = squeeze(map.elevator.dF_drad_N_per_rad(iV, iA, :, 3));
elevMy = squeeze(map.elevator.dM_drad_Nm_per_rad(iV, iA, :, 2));

if isfinite(opts.mass_kg) && opts.mass_kg > 0
    flapAz = flapFz ./ opts.mass_kg;
    elevAz = elevFz ./ opts.mass_kg;
    azUnits = '(m/s^2)/rad';
else
    flapAz = flapFz;
    elevAz = elevFz;
    azUnits = 'N/rad';
end

if isfinite(opts.Iyy_kgm2) && opts.Iyy_kgm2 > 0
    flapQdot = flapMy ./ opts.Iyy_kgm2;
    elevQdot = elevMy ./ opts.Iyy_kgm2;
    qdotUnits = '(rad/s^2)/rad';
else
    flapQdot = flapMy;
    elevQdot = elevMy;
    qdotUnits = 'N*m/rad';
end

[sigmaMin, sigmaMax, conditionNumber] = localSurfaceMatrixMetrics(flapAz, elevAz, flapQdot, elevQdot);

fig = figure('Name', sprintf('INDI Surface Control Effectiveness - %s', label), ...
    'Color', 'w', 'Position', [80, 80, 1320, 850]);
layout = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('INDI Surface Control Effectiveness: %s', label), ...
    'Interpreter', 'none', 'FontWeight', 'bold', 'Color', 'k');
subtitle(layout, sprintf('%s | V=%.1f m/s, alpha=%.1f deg', ...
    localRepoRelativePath(mapPath), vinf, alpha), 'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);

nexttile(layout);
plot(delta, flapAz, '-o', delta, elevAz, '-s', 'LineWidth', 1.4);
grid on;
xlabel('\delta actual (deg)');
ylabel(['d a_z / d\delta ', azUnits]);
title('Vertical acceleration / force column');
legend('flap', 'elevator', 'Location', 'best');

nexttile(layout);
plot(delta, flapQdot, '-o', delta, elevQdot, '-s', 'LineWidth', 1.4);
grid on;
xlabel('\delta actual (deg)');
ylabel(['d qdot / d\delta ', qdotUnits]);
title('Pitch acceleration / moment column');
legend('flap', 'elevator', 'Location', 'best');

nexttile(layout);
plot(delta, sigmaMin, '-o', delta, sigmaMax, '-s', 'LineWidth', 1.4);
grid on;
xlabel('\delta actual (deg)');
ylabel('singular value');
title('Surface effectiveness matrix singular values');
legend('\sigma_{min}', '\sigma_{max}', 'Location', 'best');

nexttile(layout);
semilogy(delta, conditionNumber, '-o', 'LineWidth', 1.4);
grid on;
xlabel('\delta actual (deg)');
ylabel('condition number');
title('2x2 surface allocator conditioning');

localFinalizeAxes(fig);
end

function fig = localPlotAlphaSweepControl(map, label, mapPath, opts)
[~, iV] = min(abs(map.grid.vinf_mps(:) - opts.targetVinf_mps));
vinf = map.grid.vinf_mps(iV);
delta = map.grid.delta_deg(:);

if isempty(opts.sweepAlpha_deg)
    alphaIdx = 1:numel(map.grid.alpha_deg);
else
    alphaIdx = localNearestIndices(map.grid.alpha_deg, opts.sweepAlpha_deg);
end
alpha = map.grid.alpha_deg(alphaIdx);

nAlpha = numel(alphaIdx);
sigmaMin = zeros(numel(delta), nAlpha);
sigmaMax = zeros(numel(delta), nAlpha);
conditionNumber = zeros(numel(delta), nAlpha);
flapFz = zeros(numel(delta), nAlpha);
elevFz = zeros(numel(delta), nAlpha);
flapMy = zeros(numel(delta), nAlpha);
elevMy = zeros(numel(delta), nAlpha);

for iAlpha = 1:nAlpha
    iA = alphaIdx(iAlpha);
    flapFz(:, iAlpha) = squeeze(map.flap.dF_drad_N_per_rad(iV, iA, :, 3));
    elevFz(:, iAlpha) = squeeze(map.elevator.dF_drad_N_per_rad(iV, iA, :, 3));
    flapMy(:, iAlpha) = squeeze(map.flap.dM_drad_Nm_per_rad(iV, iA, :, 2));
    elevMy(:, iAlpha) = squeeze(map.elevator.dM_drad_Nm_per_rad(iV, iA, :, 2));

    if isfinite(opts.mass_kg) && opts.mass_kg > 0
        flapAz = flapFz(:, iAlpha) ./ opts.mass_kg;
        elevAz = elevFz(:, iAlpha) ./ opts.mass_kg;
    else
        flapAz = flapFz(:, iAlpha);
        elevAz = elevFz(:, iAlpha);
    end

    if isfinite(opts.Iyy_kgm2) && opts.Iyy_kgm2 > 0
        flapQdot = flapMy(:, iAlpha) ./ opts.Iyy_kgm2;
        elevQdot = elevMy(:, iAlpha) ./ opts.Iyy_kgm2;
    else
        flapQdot = flapMy(:, iAlpha);
        elevQdot = elevMy(:, iAlpha);
    end

    [sigmaMin(:, iAlpha), sigmaMax(:, iAlpha), conditionNumber(:, iAlpha)] = ...
        localSurfaceMatrixMetrics(flapAz, elevAz, flapQdot, elevQdot);
end

fig = figure('Name', sprintf('INDI Surface Alpha Sweep - %s', label), ...
    'Color', 'w', 'Position', [60, 60, 1500, 950]);
layout = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('INDI Surface Alpha Sweep Metrics: %s', label), ...
    'Interpreter', 'none', 'FontWeight', 'bold', 'Color', 'k');
subtitle(layout, sprintf('%s | V=%.1f m/s', ...
    localRepoRelativePath(mapPath), vinf), 'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);

legendLabels = compose('\\alpha = %.1f deg', alpha);

nexttile(layout);
plot(delta, sigmaMin, 'LineWidth', 1.4);
grid on;
xlabel('\delta actual (deg)');
ylabel('\sigma_{min}');
title('Weakest allocator direction');
legend(legendLabels, 'Location', 'bestoutside');

nexttile(layout);
semilogy(delta, conditionNumber, 'LineWidth', 1.4);
grid on;
xlabel('\delta actual (deg)');
ylabel('condition number');
title('2x2 allocator conditioning');
legend(legendLabels, 'Location', 'bestoutside');

nexttile(layout);
imagesc(delta, alpha, sigmaMin.');
set(gca, 'YDir', 'normal');
grid on;
colormap(gca, turbo);
colorbar;
xlabel('\delta actual (deg)');
ylabel('\alpha (deg)');
title('\sigma_{min} heatmap');

nexttile(layout);
imagesc(delta, alpha, log10(conditionNumber).');
set(gca, 'YDir', 'normal');
grid on;
colormap(gca, turbo);
colorbar;
xlabel('\delta actual (deg)');
ylabel('\alpha (deg)');
title('log_{10}(condition number) heatmap');

localFinalizeAxes(fig);
end

function [sigmaMin, sigmaMax, conditionNumber] = localSurfaceMatrixMetrics(flapAz, elevAz, flapQdot, elevQdot)
n = numel(flapAz);
sigmaMin = zeros(n, 1);
sigmaMax = zeros(n, 1);
conditionNumber = zeros(n, 1);
for k = 1:n
    G = [flapAz(k), elevAz(k); flapQdot(k), elevQdot(k)];
    s = svd(G);
    sigmaMin(k) = min(s);
    sigmaMax(k) = max(s);
    conditionNumber(k) = sigmaMax(k) ./ max(sigmaMin(k), eps);
end
end

function deltaSlices = localChooseDeltaSlices(gridDelta, requested)
deltaSlices = zeros(size(requested));
for k = 1:numel(requested)
    [~, idx] = min(abs(gridDelta(:) - requested(k)));
    deltaSlices(k) = gridDelta(idx);
end
deltaSlices = unique(deltaSlices, 'stable');
end

function idx = localNearestIndices(gridValues, requested)
idx = zeros(size(requested));
for k = 1:numel(requested)
    [~, idx(k)] = min(abs(gridValues(:) - requested(k)));
end
idx = unique(idx, 'stable');
end

function limits = localRobustColorLimits(values)
values = values(:);
values = values(isfinite(values));
if isempty(values)
    limits = [-1, 1];
    return;
end

maxAbs = prctile(abs(values), 98);
if maxAbs <= 0
    maxAbs = max(abs(values));
end
if maxAbs <= 0
    maxAbs = 1;
end

limits = [-maxAbs, maxAbs];
end

function relPath = localRepoRelativePath(absPath)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
relPath = strrep(absPath, [repoRoot filesep], '');
relPath = strrep(relPath, filesep, '/');
end

function localFinalizeAxes(fig)
axesHandles = findall(fig, 'Type', 'axes');
for k = 1:numel(axesHandles)
    set(axesHandles(k), 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridAlpha', 0.22);
    if isprop(axesHandles(k), 'Toolbar') && ~isempty(axesHandles(k).Toolbar)
        axesHandles(k).Toolbar.Visible = 'off';
    end
end
end
