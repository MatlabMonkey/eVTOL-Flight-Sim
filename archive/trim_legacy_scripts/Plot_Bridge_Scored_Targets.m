% Plot_Bridge_Scored_Targets.m
% Plot the planned bridge-scored target points over the existing exact and
% acceptable trim maps.
%
% Outputs left in the base workspace:
%   - bridgeTargetPlotData
%   - bridgeTargetPlotOutputDir

if ~exist('bridgeTargetPlotOptions', 'var') || ~isstruct(bridgeTargetPlotOptions)
    bridgeTargetPlotOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
bridgeTargetPlotOutputDir = fullfile(root_dir, 'workspace_plots', ...
    ['bridge_scored_target_plot_' timestamp]);
if exist(bridgeTargetPlotOutputDir, 'dir') ~= 7
    mkdir(bridgeTargetPlotOutputDir);
end

config = localBuildConfig(bridgeTargetPlotOptions, root_dir);
[knownExact, knownScored] = localLoadKnownPoints(config);
targetPoints = localBuildTargets(config, [knownExact; knownScored]);
[bridgeSolved, bridgeAcceptable] = localLoadBridgeResults(config.bridge_latest_csv);

showPopup = localGetField(bridgeTargetPlotOptions, 'show_popup', true);
fig = figure( ...
    'Visible', localOnOff(showPopup), ...
    'Color', 'w', ...
    'Position', [100 100 1100 700], ...
    'WindowStyle', 'normal');
hold on;
grid on;
box on;

if ~isempty(knownExact)
    scatter(knownExact(:, 2), knownExact(:, 1), 28, [0.15 0.65 0.25], 'filled', ...
        'DisplayName', sprintf('Existing Exact (%d)', size(knownExact, 1)));
end
if ~isempty(knownScored)
    scatter(knownScored(:, 2), knownScored(:, 1), 34, [0.15 0.45 0.9], 'o', ...
        'LineWidth', 1.2, 'DisplayName', sprintf('Existing Acceptable (%d)', size(knownScored, 1)));
end
if ~isempty(bridgeAcceptable)
    scatter(bridgeAcceptable(:, 2), bridgeAcceptable(:, 1), 42, [0.95 0.7 0.15], 'square', ...
        'LineWidth', 1.4, 'DisplayName', sprintf('Bridge Accepted So Far (%d)', size(bridgeAcceptable, 1)));
end
if ~isempty(bridgeSolved)
    scatter(bridgeSolved(:, 2), bridgeSolved(:, 1), 48, [0.7 0.1 0.8], 'filled', ...
        'DisplayName', sprintf('Bridge Exact So Far (%d)', size(bridgeSolved, 1)));
end
if ~isempty(targetPoints)
    scatter(targetPoints(:, 2), targetPoints(:, 1), 42, [0.85 0.2 0.2], 'x', ...
        'LineWidth', 1.4, 'DisplayName', sprintf('Planned Bridge Targets (%d)', size(targetPoints, 1)));
end

scatter(0, 0, 90, [0 0 0], '^', 'filled', 'DisplayName', 'Hover Anchor');
scatter(70, 90, 90, [0 0 0], 'v', 'filled', 'DisplayName', 'Cruise Anchor');

xlabel('Airspeed V_\infty (m/s)');
ylabel('Tilt (deg)');
title('Bridge Scored Targets vs Existing Transition Map Coverage');
xlim([0 75]);
ylim([0 95]);
legend('Location', 'eastoutside');

pngFile = fullfile(bridgeTargetPlotOutputDir, 'bridge_scored_targets.png');
saveas(fig, pngFile);
drawnow;
if ~showPopup
    close(fig);
end

bridgeTargetPlotData = struct();
bridgeTargetPlotData.config = config;
bridgeTargetPlotData.existing_exact = knownExact;
bridgeTargetPlotData.existing_acceptable = knownScored;
bridgeTargetPlotData.bridge_exact = bridgeSolved;
bridgeTargetPlotData.bridge_acceptable = bridgeAcceptable;
bridgeTargetPlotData.targets = targetPoints;
bridgeTargetPlotData.png_file = pngFile;

fprintf('Bridge target plot saved to:\n  %s\n', pngFile);
fprintf('  existing exact = %d\n', size(knownExact, 1));
fprintf('  existing acceptable = %d\n', size(knownScored, 1));
fprintf('  bridge exact so far = %d\n', size(bridgeSolved, 1));
fprintf('  bridge acceptable so far = %d\n', size(bridgeAcceptable, 1));
fprintf('  planned targets = %d\n', size(targetPoints, 1));

function config = localBuildConfig(userOptions, root_dir)
config = struct();
config.root_dir = root_dir;
config.tilt_grid_deg = localGetField(userOptions, 'tilt_grid_deg', 15:5:90);
config.vinf_grid_mps = localGetField(userOptions, 'vinf_grid_mps', 20:2.5:50);
config.max_vinf_mps = localGetField(userOptions, 'max_vinf_mps', 50.0);
config.exact_csv = localGetField(userOptions, 'exact_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv'));
config.scored_csv = localGetField(userOptions, 'scored_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'));
config.bridge_latest_csv = localGetField(userOptions, 'bridge_latest_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_bridge_scored_latest.csv'));
end

function [knownExact, knownScored] = localLoadKnownPoints(config)
knownExact = localReadPoints(config.exact_csv, 'success_only');
knownScored = localReadPoints(config.scored_csv, 'acceptable_or_better');
end

function points = localReadPoints(filename, mode)
points = zeros(0, 2);
if exist(filename, 'file') ~= 2
    return;
end

tbl = readtable(filename, 'TextType', 'string');
if ~all(ismember({'tilt_deg', 'vinf_mps'}, tbl.Properties.VariableNames))
    return;
end

switch mode
    case 'success_only'
        keepMask = localAsLogical(tbl.success);
    case 'acceptable_or_better'
        if ismember('acceptable', tbl.Properties.VariableNames)
            keepMask = localAsLogical(tbl.acceptable) | localAsLogical(tbl.success);
        else
            keepMask = localAsLogical(tbl.success);
        end
    otherwise
        keepMask = true(height(tbl), 1);
end

tbl = tbl(keepMask, :);
if isempty(tbl)
    return;
end
points = unique([tbl.tilt_deg, tbl.vinf_mps], 'rows');
end

function [bridgeExact, bridgeAcceptable] = localLoadBridgeResults(filename)
bridgeExact = zeros(0, 2);
bridgeAcceptable = zeros(0, 2);
if exist(filename, 'file') ~= 2
    return;
end

tbl = readtable(filename, 'TextType', 'string');
if ~all(ismember({'tilt_deg', 'vinf_mps', 'success', 'acceptable'}, tbl.Properties.VariableNames))
    return;
end

bridgeExact = unique([tbl.tilt_deg(localAsLogical(tbl.success)), ...
    tbl.vinf_mps(localAsLogical(tbl.success))], 'rows');
bridgeAcceptable = unique([tbl.tilt_deg(localAsLogical(tbl.acceptable)), ...
    tbl.vinf_mps(localAsLogical(tbl.acceptable))], 'rows');
end

function targetPoints = localBuildTargets(config, knownGoodPoints)
targetPoints = zeros(0, 2);
for tilt_deg = config.tilt_grid_deg
    for vinf_mps = config.vinf_grid_mps
        if vinf_mps > config.max_vinf_mps + 1e-9
            continue;
        end
        if localHasPoint(knownGoodPoints, tilt_deg, vinf_mps)
            continue;
        end
        targetPoints(end + 1, :) = [tilt_deg, vinf_mps]; %#ok<AGROW>
    end
end
end

function tf = localHasPoint(points, tilt_deg, vinf_mps)
if isempty(points)
    tf = false;
    return;
end
tf = any(abs(points(:, 1) - tilt_deg) < 1e-9 & abs(points(:, 2) - vinf_mps) < 1e-9);
end

function out = localAsLogical(col)
if islogical(col)
    out = col;
elseif isnumeric(col)
    out = col ~= 0;
else
    lowered = lower(strtrim(string(col)));
    out = lowered == "1" | lowered == "true";
end
out = logical(out);
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function value = localOnOff(tf)
if tf
    value = 'on';
else
    value = 'off';
end
end
