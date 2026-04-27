function report = check_indi_fd_perturbation_sensitivity(points, perturbations_deg, opts)
%CHECK_INDI_FD_PERTURBATION_SENSITIVITY Compare finite-difference step sizes.
%
% This diagnostic does not save durable map data. It calls the normal
% Trim_Plant map builder on one exact point at a time and writes temporary
% map artifacts under tempdir. Optional summary artifacts can be written
% under workspace_plots for review.

if nargin < 1 || isempty(points)
    points = [ ...
        10,  0,   0; ...
        20, 10, -10; ...
        30, 20,   0; ...
        50, 15,  10; ...
        70,  5,  -5; ...
        15, 30,  15];
end
if nargin < 2 || isempty(perturbations_deg)
    perturbations_deg = [0.25, 0.5, 1.0];
end
if nargin < 3 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

values = zeros(size(points, 1), numel(perturbations_deg), 4);

for iPoint = 1:size(points, 1)
    vinf = points(iPoint, 1);
    alpha = points(iPoint, 2);
    delta = points(iPoint, 3);

    for iPerturb = 1:numel(perturbations_deg)
        h = perturbations_deg(iPerturb);
        tag = sprintf('indi_fd_sensitivity_V%g_A%g_D%g_h%g', vinf, alpha, delta, h);
        tag = regexprep(tag, '[^A-Za-z0-9_]', 'p');
        outMat = fullfile(tempdir, [tag, '.mat']);

        buildOpts = struct();
        buildOpts.vinf_mps = vinf;
        buildOpts.alpha_deg = alpha;
        buildOpts.delta_deg = delta;
        buildOpts.perturbation_deg = h;
        buildOpts.outputMatPath = outMat;
        buildOpts.outputSummaryPath = strrep(outMat, '.mat', '.md');
        buildOpts.checkpointEveryCalls = 0;
        buildOpts.progressEveryCalls = 0;

        map = build_indi_surface_effectiveness_map(buildOpts);
        values(iPoint, iPerturb, :) = localExtractKeyValues(map);
    end
end

report = struct();
report.points = points;
report.perturbations_deg = perturbations_deg;
report.values = values;
report.value_names = {'flap_dFz_N_per_rad', 'flap_dMy_Nm_per_rad', ...
    'elevator_dFz_N_per_rad', 'elevator_dMy_Nm_per_rad'};
report.relative_spread = localRelativeSpread(values);
report.output = struct();

assignin('base', 'indi_fd_perturbation_sensitivity_report', report);
localPrintReport(report);
if opts.saveArtifacts
    report = localSaveArtifacts(report, opts);
    assignin('base', 'indi_fd_perturbation_sensitivity_report', report);
end
end

function opts = localApplyDefaults(opts)
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if ~isfield(opts, 'saveArtifacts') || isempty(opts.saveArtifacts)
    opts.saveArtifacts = true;
end
if ~isfield(opts, 'outputDir') || isempty(opts.outputDir)
    opts.outputDir = fullfile(repoRoot, 'workspace_plots', 'indi_fd_sensitivity');
end
if ~isfield(opts, 'tag') || isempty(opts.tag)
    opts.tag = 'targeted';
end
opts.tag = regexprep(char(string(opts.tag)), '[^A-Za-z0-9_]', '_');
end

function values = localExtractKeyValues(map)
values = zeros(1, 4);
values(1) = squeeze(map.flap.dF_drad_N_per_rad(1, 1, 1, 3));
values(2) = squeeze(map.flap.dM_drad_Nm_per_rad(1, 1, 1, 2));
values(3) = squeeze(map.elevator.dF_drad_N_per_rad(1, 1, 1, 3));
values(4) = squeeze(map.elevator.dM_drad_Nm_per_rad(1, 1, 1, 2));
end

function spread = localRelativeSpread(values)
spread = zeros(size(values, 1), size(values, 3));
for iPoint = 1:size(values, 1)
    for iValue = 1:size(values, 3)
        series = squeeze(values(iPoint, :, iValue));
        reference = max(abs(series(perturbationsReferenceIndex(numel(series)))), 1.0);
        spread(iPoint, iValue) = (max(series) - min(series)) / reference;
    end
end
end

function idx = perturbationsReferenceIndex(n)
idx = min(2, n);
end

function localPrintReport(report)
fprintf('\nFD_PERTURBATION_SENSITIVITY_RESULTS\n');
fprintf('columns: flap_dFz, flap_dMy, elevator_dFz, elevator_dMy\n');

for iPoint = 1:size(report.points, 1)
    p = report.points(iPoint, :);
    fprintf('\nPOINT V=%.1f alpha=%.1f delta=%.1f\n', p(1), p(2), p(3));
    for iPerturb = 1:numel(report.perturbations_deg)
        h = report.perturbations_deg(iPerturb);
        v = squeeze(report.values(iPoint, iPerturb, :));
        fprintf('  h=%4.2f deg: % .6g % .6g % .6g % .6g\n', ...
            h, v(1), v(2), v(3), v(4));
    end
    s = squeeze(report.relative_spread(iPoint, :));
    fprintf('  relative spread: % .3g % .3g % .3g % .3g\n', ...
        s(1), s(2), s(3), s(4));
end
end

function report = localSaveArtifacts(report, opts)
if exist(opts.outputDir, 'dir') ~= 7
    mkdir(opts.outputDir);
end

matPath = fullfile(opts.outputDir, sprintf('indi_fd_sensitivity_%s.mat', opts.tag));
mdPath = fullfile(opts.outputDir, sprintf('indi_fd_sensitivity_%s.md', opts.tag));
pngPath = fullfile(opts.outputDir, sprintf('indi_fd_sensitivity_%s.png', opts.tag));
figPath = fullfile(opts.outputDir, sprintf('indi_fd_sensitivity_%s.fig', opts.tag));

report.output.mat_path = matPath;
report.output.summary_path = mdPath;
report.output.png_path = pngPath;
report.output.fig_path = figPath;

save(matPath, 'report', '-v7.3');
localWriteSummary(report, mdPath);
fig = localPlotReport(report, opts.tag);
savefig(fig, figPath);
print(fig, pngPath, '-dpng', '-r180');
close(fig);

fprintf('\nSaved FD perturbation sensitivity artifacts:\n');
fprintf('  %s\n', matPath);
fprintf('  %s\n', mdPath);
fprintf('  %s\n', pngPath);
fprintf('  %s\n', figPath);
end

function localWriteSummary(report, mdPath)
fid = fopen(mdPath, 'w');
if fid < 0
    error('check_indi_fd_perturbation_sensitivity:SummaryOpenFailed', ...
        'Unable to write %s.', mdPath);
end
cleanupFile = onCleanup(@() fclose(fid));

fprintf(fid, '# INDI FD Perturbation Sensitivity\n\n');
fprintf(fid, 'This is a targeted diagnostic, not a production map database entry.\n\n');
fprintf(fid, '## Setup\n\n');
fprintf(fid, '- Perturbations deg: `%s`\n', mat2str(report.perturbations_deg));
fprintf(fid, '- Points rows `[Vinf_mps alpha_deg delta_deg]`: `%d`\n\n', size(report.points, 1));
fprintf(fid, '## Worst Relative Spread\n\n');
for iValue = 1:numel(report.value_names)
    [worstSpread, idx] = max(report.relative_spread(:, iValue));
    p = report.points(idx, :);
    fprintf(fid, '- `%s`: %.4g at V=%.3g, alpha=%.3g, delta=%.3g\n', ...
        report.value_names{iValue}, worstSpread, p(1), p(2), p(3));
end
end

function fig = localPlotReport(report, tag)
uniqueV = unique(report.points(:, 1));
uniqueAlpha = unique(report.points(:, 2));
if numel(uniqueV) ~= 1 || numel(uniqueAlpha) ~= 1
    fig = localPlotGenericReport(report, tag);
    return;
end

delta = report.points(:, 3);
[delta, order] = sort(delta);
values = report.values(order, :, :);

fig = figure('Name', sprintf('INDI FD Sensitivity - %s', tag), ...
    'Color', 'w', 'Position', [80, 80, 1450, 900]);
layout = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('INDI FD Perturbation Sensitivity: %s', tag), ...
    'Interpreter', 'none', 'FontWeight', 'bold');
subtitle(layout, sprintf('V=%.1f m/s, alpha=%.1f deg', uniqueV, uniqueAlpha));

for iValue = 1:numel(report.value_names)
    ax = nexttile(layout);
    hold(ax, 'on');
    for iPerturb = 1:numel(report.perturbations_deg)
        plot(ax, delta, values(:, iPerturb, iValue), '-o', 'LineWidth', 1.3);
    end
    hold(ax, 'off');
    grid(ax, 'on');
    xlabel(ax, '\delta actual (deg)');
    ylabel(ax, strrep(report.value_names{iValue}, '_', '\_'));
    title(ax, strrep(report.value_names{iValue}, '_', '\_'));
    legend(ax, compose('h=%.2g deg', report.perturbations_deg), 'Location', 'best');
end
localFinalizeFigure(fig);
end

function fig = localPlotGenericReport(report, tag)
fig = figure('Name', sprintf('INDI FD Sensitivity - %s', tag), ...
    'Color', 'w', 'Position', [80, 80, 1200, 800]);
layout = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, sprintf('INDI FD Perturbation Sensitivity: %s', tag), ...
    'Interpreter', 'none', 'FontWeight', 'bold');

for iValue = 1:numel(report.value_names)
    ax = nexttile(layout);
    plot(ax, report.relative_spread(:, iValue), '-o', 'LineWidth', 1.3);
    grid(ax, 'on');
    xlabel(ax, 'point index');
    ylabel(ax, 'relative spread');
    title(ax, strrep(report.value_names{iValue}, '_', '\_'));
end
localFinalizeFigure(fig);
end

function localFinalizeFigure(fig)
axesHandles = findall(fig, 'Type', 'axes');
for k = 1:numel(axesHandles)
    set(axesHandles(k), ...
        'Color', 'w', ...
        'XColor', 'k', ...
        'YColor', 'k', ...
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
