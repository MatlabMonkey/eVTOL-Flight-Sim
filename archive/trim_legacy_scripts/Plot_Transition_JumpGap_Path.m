% Plot_Transition_JumpGap_Path.m
% Plot the full transition trim map and overlay the selected jump-gap path
% used for scheduled LQR interpolation.

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

plotOptions = struct( ...
    'show_popup', true, ...
    'exclude_zero_rear_rows', true, ...
    'min_rear_collective_rpm', 1.0);
if exist('jumpGapTransitionPlotOptions', 'var') && isstruct(jumpGapTransitionPlotOptions)
    plotOptions = localMergeStruct(plotOptions, jumpGapTransitionPlotOptions);
end

[pathSpec, pathPoints, pathTable] = load_transition_jumpgap_path_points();
tbl = localLoadMasterAttemptTable(pathSpec.master_db_file);

if plotOptions.exclude_zero_rear_rows
    rearMask = ~isnan(tbl.rear_collective_rpm) & ...
        (tbl.rear_collective_rpm > plotOptions.min_rear_collective_rpm);
    tbl = tbl(rearMask, :);
end

[idealGuide, selectedGuide] = localGetGuideOverlays(pathSpec, pathPoints);
timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
plotDir = fullfile(root_dir, 'workspace_plots', ['transition_jumpgap_path_visuals_' timestamp]);
if exist(plotDir, 'dir') ~= 7
    mkdir(plotDir);
end

successMask = localAsLogical(tbl.success);
acceptableMask = localAsLogical(tbl.acceptable);
classification = localAsString(tbl.classification);
vinf_mps = tbl.vinf_mps;
tilt_deg = tbl.tilt_deg;

isExact = classification == "exact_trim" | successMask;
isAcceptable = acceptableMask & ~isExact;
isBorderline = classification == "near_trim_borderline" & ~isExact & ~isAcceptable;
isOther = ~(isExact | isAcceptable | isBorderline);

pathV = [pathPoints.vinf_mps];
pathTilt = [pathPoints.tilt_deg];
pathFront = [pathPoints.front_collective_rpm];
pathRear = [pathPoints.rear_collective_rpm];
jumpAfterIdx = pathSpec.jump_after_index;

figVisible = localFigureVisibility(plotOptions.show_popup);

f1 = figure('Color', 'w', 'Position', [100 100 1120 780], 'Visible', figVisible);
hold on;
legendHandles = gobjects(0);
legendLabels = {};
if any(isOther)
    h = scatter(vinf_mps(isOther), tilt_deg(isOther), 34, [0.70 0.70 0.70], 'x', 'LineWidth', 0.9);
    legendHandles(end + 1) = h; %#ok<AGROW>
    legendLabels{end + 1} = 'Failed / not usable'; %#ok<AGROW>
end
if any(isBorderline)
    h = scatter(vinf_mps(isBorderline), tilt_deg(isBorderline), 40, [0.95 0.55 0.15], '^', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
    legendHandles(end + 1) = h; %#ok<AGROW>
    legendLabels{end + 1} = 'Borderline near-trim'; %#ok<AGROW>
end
if any(isAcceptable)
    h = scatter(vinf_mps(isAcceptable), tilt_deg(isAcceptable), 42, [0.98 0.82 0.18], 's', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
    legendHandles(end + 1) = h; %#ok<AGROW>
    legendLabels{end + 1} = 'Acceptable near-trim'; %#ok<AGROW>
end
if any(isExact)
    h = scatter(vinf_mps(isExact), tilt_deg(isExact), 36, [0.12 0.60 0.18], 'o', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.2);
    legendHandles(end + 1) = h; %#ok<AGROW>
    legendLabels{end + 1} = 'Exact trim'; %#ok<AGROW>
end

h = scatter(0.0, 0.0, 140, 'd', 'MarkerFaceColor', [0.10 0.35 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
legendHandles(end + 1) = h; %#ok<AGROW>
legendLabels{end + 1} = 'Hover anchor'; %#ok<AGROW>
h = scatter(70.0, 90.0, 140, 'p', 'MarkerFaceColor', [0.90 0.90 0.90], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
legendHandles(end + 1) = h; %#ok<AGROW>
legendLabels{end + 1} = 'Cruise anchor'; %#ok<AGROW>

hIdeal = plot(idealGuide.vinf_mps, idealGuide.tilt_deg, '--', ...
    'Color', [0.07 0.39 0.95], 'LineWidth', 2.4);
legendHandles(end + 1) = hIdeal; %#ok<AGROW>
legendLabels{end + 1} = 'Ideal hand-picked guide'; %#ok<AGROW>

[hPath, hJump] = localPlotPathSegments(pathV, pathTilt, jumpAfterIdx);
legendHandles(end + 1) = hPath; %#ok<AGROW>
legendLabels{end + 1} = 'Selected exact-point path'; %#ok<AGROW>
legendHandles(end + 1) = hJump; %#ok<AGROW>
legendLabels{end + 1} = 'Jump segment'; %#ok<AGROW>

h = scatter(pathV, pathTilt, 92, [0.15 0.85 0.95], 'o', ...
    'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
legendHandles(end + 1) = h; %#ok<AGROW>
legendLabels{end + 1} = 'Scheduled LQR points'; %#ok<AGROW>
for i = 1:numel(pathPoints)
    text(pathV(i) + 0.6, pathTilt(i) + 0.6, sprintf('%d', i), ...
        'Color', [0.92 0.95 1.0], 'FontWeight', 'bold', 'FontSize', 8);
end

xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Front Tilt (deg)');
title('Transition Trim Map with Jump-Gap LQR Path');
legend(legendHandles, legendLabels, 'Location', 'northwest');
grid on;
box on;
saveas(f1, fullfile(plotDir, 'transition_jumpgap_path_map.png'));

f2 = figure('Color', 'w', 'Position', [120 120 1120 780], 'Visible', figVisible);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
if any(isOther)
    scatter(vinf_mps(isOther), tbl.front_collective_rpm(isOther), 30, [0.70 0.70 0.70], 'x', 'LineWidth', 0.9);
end
if any(isBorderline)
    scatter(vinf_mps(isBorderline), tbl.front_collective_rpm(isBorderline), 36, [0.95 0.55 0.15], '^', 'filled');
end
if any(isAcceptable)
    scatter(vinf_mps(isAcceptable), tbl.front_collective_rpm(isAcceptable), 38, [0.98 0.82 0.18], 's', 'filled');
end
if any(isExact)
    scatter(vinf_mps(isExact), tbl.front_collective_rpm(isExact), 34, [0.12 0.60 0.18], 'o', 'filled');
end
hIdealFront = plot(idealGuide.vinf_mps, idealGuide.front_collective_rpm, '--', ...
    'Color', [0.07 0.39 0.95], 'LineWidth', 2.4);
[hFrontPath, hFrontJump] = localPlotPathSegments(pathV, pathFront, jumpAfterIdx);
scatter(pathV, pathFront, 88, [0.15 0.85 0.95], 'o', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
ylabel('Front collective (rpm)');
title('Transition Jump-Gap Path Propeller Schedule');
legend([hIdealFront, hFrontPath, hFrontJump], ...
    {'Ideal guide trend', 'Selected exact-point path', 'Jump segment'}, ...
    'Location', 'northeast');
grid on;
box on;

nexttile;
hold on;
if any(isOther)
    scatter(vinf_mps(isOther), tbl.rear_collective_rpm(isOther), 30, [0.70 0.70 0.70], 'x', 'LineWidth', 0.9);
end
if any(isBorderline)
    scatter(vinf_mps(isBorderline), tbl.rear_collective_rpm(isBorderline), 36, [0.95 0.55 0.15], '^', 'filled');
end
if any(isAcceptable)
    scatter(vinf_mps(isAcceptable), tbl.rear_collective_rpm(isAcceptable), 38, [0.98 0.82 0.18], 's', 'filled');
end
if any(isExact)
    scatter(vinf_mps(isExact), tbl.rear_collective_rpm(isExact), 34, [0.12 0.60 0.18], 'o', 'filled');
end
hIdealRear = plot(idealGuide.vinf_mps, idealGuide.rear_collective_rpm, '--', ...
    'Color', [0.07 0.39 0.95], 'LineWidth', 2.4);
[hRearPath, hRearJump] = localPlotPathSegments(pathV, pathRear, jumpAfterIdx);
scatter(pathV, pathRear, 88, [0.15 0.85 0.95], 'o', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
xlabel('Airspeed V_{\infty} (m/s)');
ylabel('Rear collective (rpm)');
legend([hIdealRear, hRearPath, hRearJump], ...
    {'Ideal guide trend', 'Selected exact-point path', 'Jump segment'}, ...
    'Location', 'northeast');
grid on;
box on;
saveas(f2, fullfile(plotDir, 'transition_jumpgap_path_prop_rpm.png'));

jumpGapTransitionPlotDir = plotDir;
jumpGapTransitionPlotSummary = struct();
jumpGapTransitionPlotSummary.output_dir = plotDir;
jumpGapTransitionPlotSummary.path_name = pathSpec.name;
jumpGapTransitionPlotSummary.path_table = pathTable;
jumpGapTransitionPlotSummary.ideal_guide = idealGuide;
jumpGapTransitionPlotSummary.selected_path = selectedGuide;
assignin('base', 'jumpGapTransitionPlotDir', jumpGapTransitionPlotDir);
assignin('base', 'jumpGapTransitionPlotSummary', jumpGapTransitionPlotSummary);

disp(plotDir);

if ~plotOptions.show_popup
    close(f1);
    close(f2);
end

function tbl = localLoadMasterAttemptTable(masterDbFile)
if exist(masterDbFile, 'file') ~= 2
    error('Plot_Transition_JumpGap_Path:MissingMasterDb', ...
        'Master attempt DB not found: %s', masterDbFile);
end

data = load(masterDbFile, 'transitionTrimMasterAttemptDB');
if ~isfield(data, 'transitionTrimMasterAttemptDB') || ...
        ~isfield(data.transitionTrimMasterAttemptDB, 'master_attempt_db_all_rows')
    error('Plot_Transition_JumpGap_Path:BadMasterDb', ...
        'Master attempt DB does not contain master_attempt_db_all_rows.');
end

tbl = data.transitionTrimMasterAttemptDB.master_attempt_db_all_rows;
if isempty(tbl)
    error('Plot_Transition_JumpGap_Path:EmptyMasterDb', ...
        'Master attempt DB did not contain any rows.');
end
end

function value = localAsString(x)
if isstring(x)
    value = x;
elseif iscellstr(x) || iscell(x)
    value = string(x);
elseif ischar(x)
    value = string(cellstr(x));
else
    value = string(x);
end
value = strip(value);
end

function tf = localAsLogical(x)
if islogical(x)
    tf = x;
    return;
end
if isnumeric(x)
    tf = x ~= 0;
    return;
end
str = lower(strip(localAsString(x)));
tf = str == "true" | str == "1" | str == "yes";
end

function figVisible = localFigureVisibility(showPopup)
if showPopup
    figVisible = 'on';
else
    figVisible = 'off';
end
end

function out = localMergeStruct(base, override)
out = base;
fields = fieldnames(override);
for i = 1:numel(fields)
    out.(fields{i}) = override.(fields{i});
end
end

function [hPath, hJump] = localPlotPathSegments(xData, yData, jumpAfterIdx)
hPath = gobjects(1);
hJump = gobjects(1);
for i = 1:(numel(xData) - 1)
    style = '-';
    color = [0.12 0.62 0.92];
    width = 2.8;
    if i == jumpAfterIdx
        style = '--';
        color = [0.92 0.18 0.72];
        width = 3.2;
    end
    h = plot(xData(i:i+1), yData(i:i+1), style, 'Color', color, 'LineWidth', width);
    if i == 1
        hPath = h;
    end
    if i == jumpAfterIdx
        hJump = h;
    end
end
end

function [idealGuide, selectedGuide] = localGetGuideOverlays(pathSpec, pathPoints)
selectedGuide = struct();
selectedGuide.vinf_mps = [pathPoints.vinf_mps];
selectedGuide.tilt_deg = [pathPoints.tilt_deg];
selectedGuide.front_collective_rpm = [pathPoints.front_collective_rpm];
selectedGuide.rear_collective_rpm = [pathPoints.rear_collective_rpm];

idealGuide = pathSpec.ideal_guide;
end
