% TrimSearch_Preview.m
% Preview the simpler mid-band guide-grid search before launching it.
%
% Optional workspace overrides before running:
%   transitionMidbandGuideGridPreviewOptions = struct(...);
%
% Outputs left in the base workspace:
%   - transitionMidbandGuideGridPreviewDir
%   - transitionMidbandGuideGridPreviewSummary
%
% Main output files:
%   workspace_plots/transition_midband_guidegrid_preview_<timestamp>/
%     midband_guidegrid_preview.png

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

if exist('transitionMidbandGuideGridPreviewOptions', 'var') && ...
        isstruct(transitionMidbandGuideGridPreviewOptions)
    previewOptions = transitionMidbandGuideGridPreviewOptions;
else
    previewOptions = struct();
end

plan = TrimSearch_BuildPlan(previewOptions, root_dir);
config = plan.config;
anchorTable = plan.anchor_table;
targets = plan.targets;
seedPreview = plan.seed_preview;

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
transitionMidbandGuideGridPreviewDir = fullfile(root_dir, 'workspace_plots', ...
    ['transition_midband_guidegrid_preview_' timestamp]);
if exist(transitionMidbandGuideGridPreviewDir, 'dir') ~= 7
    mkdir(transitionMidbandGuideGridPreviewDir);
end

minAnchorVinf = 0.0;
minAnchorTilt = 0.0;
if ~isempty(anchorTable)
    if ismember('vinf_mps', anchorTable.Properties.VariableNames)
        minAnchorVinf = min([0.0; anchorTable.vinf_mps]);
    end
    if ismember('tilt_deg', anchorTable.Properties.VariableNames)
        minAnchorTilt = min([0.0; anchorTable.tilt_deg]);
    end
end

referenceVinfDense = linspace(min([min(config.vinf_grid_mps), minAnchorVinf]), max(config.vinf_grid_mps), 500);
referenceTiltDense = interp1(config.reference_knot_vinf_mps, config.reference_knot_tilt_deg, ...
    referenceVinfDense, 'pchip');
frontGuideDense = interp1(config.front_guide_knot_vinf_mps, config.front_guide_knot_rpm, ...
    referenceVinfDense, 'pchip');
rearGuideDense = interp1(config.rear_guide_knot_vinf_mps, config.rear_guide_knot_rpm, ...
    referenceVinfDense, 'pchip');

anchorExactMask = false(height(anchorTable), 1);
anchorAcceptableMask = false(height(anchorTable), 1);
if ~isempty(anchorTable)
    if ismember('success', anchorTable.Properties.VariableNames)
        anchorExactMask = localAsLogical(anchorTable.success);
    end
    if ismember('acceptable', anchorTable.Properties.VariableNames)
        anchorAcceptableMask = localAsLogical(anchorTable.acceptable) & ~anchorExactMask;
    end
end

targetCenterTilt = [targets.center_tilt_deg]';
targetTilt = [targets.tilt_deg]';
targetVinf = [targets.vinf_mps]';
targetFrontGuide = [targets.front_guide_rpm]';
targetRearGuide = [targets.rear_guide_rpm]';

seedFrontGuess = [seedPreview.front_guess_rpm]';
seedRearGuess = [seedPreview.rear_guess_rpm]';
seedVinf = [seedPreview.vinf_mps]';

previewFig = figure('Color', 'w', 'Position', [60 60 1360 840], 'Visible', 'on');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile([2 1]);
hold on;
plot(referenceVinfDense, referenceTiltDense, '-', 'Color', [0.35 0.80 0.18], 'LineWidth', 3.0);
scatter(targetVinf, targetTilt, 34, [0.75 0.75 0.75], 'x', 'LineWidth', 0.8);
scatter(targetVinf, targetCenterTilt, 34, [0.92 0.68 0.18], 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.2);

if ~isempty(anchorTable)
    if any(anchorAcceptableMask)
        scatter(anchorTable.vinf_mps(anchorAcceptableMask), anchorTable.tilt_deg(anchorAcceptableMask), ...
            52, [0.98 0.82 0.18], 's', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
    end
    if any(anchorExactMask)
        scatter(anchorTable.vinf_mps(anchorExactMask), anchorTable.tilt_deg(anchorExactMask), ...
            48, [0.12 0.60 0.18], 'o', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
    end
end

scatter(0.0, 0.0, 130, 'd', 'MarkerFaceColor', [0.10 0.35 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
scatter(70.0, 90.0, 130, 'p', 'MarkerFaceColor', [0.95 0.95 0.95], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);

xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Planned Mid-band Guide-grid Search Region');
legendEntries = {'Reference tilt line', 'Tilt search lattice', 'Line-center targets'};
if any(anchorAcceptableMask)
    legendEntries{end + 1} = 'Low-speed acceptable anchors'; %#ok<AGROW>
end
if any(anchorExactMask)
    legendEntries{end + 1} = 'Low-speed exact anchors'; %#ok<AGROW>
end
legendEntries{end + 1} = 'Hover anchor'; %#ok<AGROW>
legendEntries{end + 1} = 'Cruise anchor'; %#ok<AGROW>
legend(legendEntries, 'Location', 'northwest');
grid on;
box on;
xlim([min([min(config.vinf_grid_mps), minAnchorVinf]) - 2.5, max(config.vinf_grid_mps) + 2.5]);
ylim([max(0, minAnchorTilt - 2.5), 92.5]);

nexttile;
hold on;
plot(referenceVinfDense, frontGuideDense, '-', 'Color', [0.35 0.80 0.18], 'LineWidth', 3.0);
scatter(seedVinf, seedFrontGuess, 16, [0.70 0.70 0.70], '.', 'MarkerEdgeAlpha', 0.35);
scatter(targetVinf, targetFrontGuide, 42, [0.92 0.68 0.18], 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
if ~isempty(anchorTable)
    scatter(anchorTable.vinf_mps, anchorTable.front_collective_rpm, 36, [0.10 0.35 0.90], ...
        'o', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front collective (rpm)');
title('Front Collective Guide and Seed Grid');
legend({'Front guide line', 'Seed grid', 'Guide centers', 'Low-speed anchors'}, 'Location', 'northeast');
grid on;
box on;
xlim([min(config.vinf_grid_mps) - 2.5, max(config.vinf_grid_mps) + 2.5]);

nexttile;
hold on;
plot(referenceVinfDense, rearGuideDense, '-', 'Color', [0.35 0.80 0.18], 'LineWidth', 3.0);
scatter(seedVinf, seedRearGuess, 16, [0.70 0.70 0.70], '.', 'MarkerEdgeAlpha', 0.35);
scatter(targetVinf, targetRearGuide, 42, [0.92 0.68 0.18], 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
if ~isempty(anchorTable)
    scatter(anchorTable.vinf_mps, anchorTable.rear_collective_rpm, 36, [0.10 0.35 0.90], ...
        'o', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
end
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Rear collective (rpm)');
title('Rear Collective Guide and Seed Grid');
legend({'Rear guide line', 'Seed grid', 'Guide centers', 'Low-speed anchors'}, 'Location', 'northeast');
grid on;
box on;
xlim([min(config.vinf_grid_mps) - 2.5, max(config.vinf_grid_mps) + 2.5]);

previewPng = fullfile(transitionMidbandGuideGridPreviewDir, 'midband_guidegrid_preview.png');
saveas(previewFig, previewPng);

transitionMidbandGuideGridPreviewSummary = struct();
transitionMidbandGuideGridPreviewSummary.output_dir = transitionMidbandGuideGridPreviewDir;
transitionMidbandGuideGridPreviewSummary.output_png = previewPng;
transitionMidbandGuideGridPreviewSummary.target_count = numel(targets);
transitionMidbandGuideGridPreviewSummary.seed_preview_count = numel(seedPreview);
transitionMidbandGuideGridPreviewSummary.anchor_count = height(anchorTable);
transitionMidbandGuideGridPreviewSummary.anchor_history_csv = config.anchor_history_csv;
transitionMidbandGuideGridPreviewSummary.front_seed_offsets_rpm = config.front_seed_offsets_rpm;
transitionMidbandGuideGridPreviewSummary.rear_seed_offsets_rpm = config.rear_seed_offsets_rpm;

disp(transitionMidbandGuideGridPreviewDir);

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
    out = false(numel(col), 1);
end
out = logical(out);
end
