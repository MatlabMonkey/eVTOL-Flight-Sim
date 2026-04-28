function reportData = plot_indi_transition_debug(simSource, filenameStem, titlePrefix)
%PLOT_INDI_TRANSITION_DEBUG Plot the core INDI transition controller signals.
%
% Usage:
%   plot_indi_transition_debug
%   plot_indi_transition_debug(out)
%   plot_indi_transition_debug(out, true, 'INDI Segment 1 to 2')
%
% This plot is intentionally controller-centric. It compares the four
% longitudinal tracking states used by the INDI outer loop, the virtual
% acceleration command/measurement pair, and the eta control variables:
%
%   x_track = [theta, u, w, q]
%   nu      = [a_x, a_z, qdot]
%   eta     = [Omega_f^2, Omega_r^2, delta_f, delta_e]

if nargin < 1 || isempty(simSource)
    if evalin('base', 'exist(''out'', ''var'')')
        simSource = evalin('base', 'out');
    else
        error('plot_indi_transition_debug:MissingSimSource', ...
            'No simSource was provided and no base-workspace variable "out" was found.');
    end
end

if nargin < 2 || isempty(filenameStem)
    filenameStem = '';
end

if nargin < 3 || isempty(titlePrefix)
    titlePrefix = 'INDI Transition Debug';
end

controllerData = localGetControllerData();
style = localIndiStyle();
filenameStem = localResolveFilenameStem(filenameStem, titlePrefix, simSource);

reportData = localBuildIndiReportData(simSource, controllerData);
reportData.controller_data = controllerData;

fig = figure( ...
    'Name', [titlePrefix ' - INDI Core'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [80 60 1320 980]);
tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

xLimits = localIndiXLimits(reportData);

nexttile;
localPlotStateTracking(reportData, style, 1, '\theta', 'deg');
title('\theta Tracking');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotStateTracking(reportData, style, 2, 'u', 'm/s');
title('u Tracking');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotStateTracking(reportData, style, 3, 'w', 'm/s');
title('w Tracking');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotStateTracking(reportData, style, 4, 'q', 'deg/s');
title('q Tracking');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotNuTracking(reportData, style);
title('\nu Command vs Measurement');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotNuError(reportData, style);
title('\nu Error');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotEtaTracking(reportData, style);
title('\eta Control Variable');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotDeltaEta(reportData, style);
title('\Delta\eta Allocator Request');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

titleHandle = sgtitle(fig, [titlePrefix ' - INDI Core Debug'], ...
    'FontWeight', 'bold', 'Interpreter', 'none');
titleHandle.Color = 'k';

figAirdata = localCreateAirdataFigure(reportData, style, titlePrefix);

reportData.figures = struct('indi_core', fig, 'airdata', figAirdata);

if ~isempty(filenameStem)
    [outDir, baseName, ext] = fileparts(filenameStem);
    if isempty(outDir)
        outDir = localWorkspacePlotsDir();
    end
    if ~isempty(ext)
        baseName = [baseName ext];
    end
    if isempty(baseName)
        baseName = 'indi_transition_debug';
    end
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    outFile = fullfile(outDir, [baseName '_indi_core_debug.png']);
    exportgraphics(fig, outFile, 'Resolution', 300);
    airdataFile = fullfile(outDir, [baseName '_airdata_debug.png']);
    exportgraphics(figAirdata, airdataFile, 'Resolution', 300);
    reportData.export = struct('dir', outDir, ...
        'indi_core_file', outFile, ...
        'airdata_file', airdataFile);
end
end

function reportData = localBuildIndiReportData(simSource, controllerData)
t = localChooseTimeBase(simSource);
actual = localBuildActualStateHistory(simSource, t);
nuMeas = localBuildMeasuredVirtualAcceleration(simSource, t);
schedulerDebug = localBuildSchedulerDebugSeries(simSource, controllerData, t);
switchEvents = localBuildSwitchEvents(simSource, controllerData, schedulerDebug);

[idxLoHistory, idxHiHistory, lambdaHistory] = ...
    localBuildBlendHistory(simSource, t, controllerData, schedulerDebug, switchEvents);

ref = localBuildReferenceHistory(controllerData, actual, t, ...
    idxLoHistory, idxHiHistory, lambdaHistory);
control = localBuildControlVariableHistory(simSource, t, ref, schedulerDebug, ...
    controllerData, idxLoHistory, idxHiHistory, lambdaHistory);
airdata = localBuildAirdataHistory(simSource, controllerData, actual, t, ...
    idxLoHistory, idxHiHistory, lambdaHistory);

reportData = struct();
reportData.time = t;
reportData.actual = actual;
reportData.ref = ref;
reportData.state_error = ref.states - actual.states;
reportData.nu_meas = nuMeas;
reportData.nu_error = ref.nu_cmd - nuMeas.values;
reportData.scheduler_debug = schedulerDebug;
reportData.control = control;
reportData.airdata = airdata;
reportData.switch_events = switchEvents;
end

function controllerData = localGetControllerData()
if ~evalin('base', 'exist(''controllerData'', ''var'')')
    error('plot_indi_transition_debug:MissingControllerData', ...
        'Base workspace variable "controllerData" is required.');
end

controllerData = evalin('base', 'controllerData');
if ~isstruct(controllerData) || ~isfield(controllerData, 'controller_id') || controllerData.controller_id ~= 6
    error('plot_indi_transition_debug:NotIndiController', ...
        'controllerData must be the controller_id = 6 INDI transition controller.');
end
if ~isfield(controllerData, 'controller_state_ref') || ~isfield(controllerData, 'controller_trim_cmd')
    error('plot_indi_transition_debug:BadControllerData', ...
        'controllerData must contain controller_state_ref and controller_trim_cmd.');
end
end

function style = localIndiStyle()
style = struct();
style.colors = struct( ...
    'theta', [0.47 0.67 0.19], ...
    'u', [0.00 0.45 0.74], ...
    'w', [0.85 0.33 0.10], ...
    'q', [0.49 0.18 0.56], ...
    'ax', [0.00 0.45 0.74], ...
    'az', [0.85 0.33 0.10], ...
    'qdot', [0.49 0.18 0.56], ...
    'front', [0.00 0.45 0.74], ...
    'rear', [0.85 0.33 0.10], ...
    'deltaf', [0.47 0.67 0.19], ...
    'deltae', [0.49 0.18 0.56]);
style.measuredLineStyle = '-';
style.commandLineStyle = '--';
style.errorLineStyle = ':';
style.incrementLineStyle = '-.';
style.measuredLineWidth = 1.45;
style.commandLineWidth = 1.25;
style.errorLineWidth = 1.25;
style.gridColor = [0.86 0.86 0.86];
style.axisColor = [0.15 0.15 0.15];
end

function localPlotStateTracking(reportData, style, colIdx, signalName, unitsText)
t = reportData.time;
[actual, ref, err] = localStatePlotColumns(reportData, colIdx);
color = localSignalColor(style, signalName);

hold(gca, 'on');
plot(t, actual, 'Color', color, 'LineStyle', style.measuredLineStyle, ...
    'LineWidth', style.measuredLineWidth);
plot(t, ref, 'Color', color, 'LineStyle', style.commandLineStyle, ...
    'LineWidth', style.commandLineWidth);
plot(t, err, 'Color', color, 'LineStyle', style.errorLineStyle, ...
    'LineWidth', style.errorLineWidth);
ylabel(unitsText);
leg = legend({[signalName ' meas'], [signalName ' ref'], [signalName ' err']}, ...
    'Location', 'best', 'Box', 'off', 'Interpreter', 'tex');
leg.TextColor = 'k';
leg.Color = 'none';
hold(gca, 'off');
end

function [actual, ref, err] = localStatePlotColumns(reportData, colIdx)
actual = reportData.actual.states(:, colIdx);
ref = reportData.ref.states(:, colIdx);
err = reportData.state_error(:, colIdx);
if colIdx == 1 || colIdx == 4
    actual = rad2deg(actual);
    ref = rad2deg(ref);
    err = rad2deg(err);
end
end

function localPlotNuTracking(reportData, style)
t = reportData.time;
nuCmd = reportData.ref.nu_cmd;
nuMeas = reportData.nu_meas.values;

hold(gca, 'on');
yyaxis left;
plot(t, nuMeas(:, 1), 'Color', localSignalColor(style, 'ax'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, nuCmd(:, 1), 'Color', localSignalColor(style, 'ax'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
plot(t, nuMeas(:, 2), 'Color', localSignalColor(style, 'az'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, nuCmd(:, 2), 'Color', localSignalColor(style, 'az'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
ylabel('a_x, a_z [m/s^2]');

yyaxis right;
plot(t, rad2deg(nuMeas(:, 3)), 'Color', localSignalColor(style, 'qdot'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, rad2deg(nuCmd(:, 3)), 'Color', localSignalColor(style, 'qdot'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
ylabel('qdot [deg/s^2]');
localForceYAxisBlack(gca);

leg = legend({'a_x meas', 'a_x cmd', 'a_z meas', 'a_z cmd', ...
    'qdot meas', 'qdot cmd'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(gca, 'off');
end

function localPlotNuError(reportData, style)
t = reportData.time;
nuErr = reportData.nu_error;

hold(gca, 'on');
yyaxis left;
plot(t, nuErr(:, 1), 'Color', localSignalColor(style, 'ax'), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.errorLineWidth);
plot(t, nuErr(:, 2), 'Color', localSignalColor(style, 'az'), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.errorLineWidth);
ylabel('a cmd - meas [m/s^2]');

yyaxis right;
plot(t, rad2deg(nuErr(:, 3)), 'Color', localSignalColor(style, 'qdot'), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.errorLineWidth);
ylabel('qdot cmd - meas [deg/s^2]');
localForceYAxisBlack(gca);

leg = legend({'a_x err', 'a_z err', 'qdot err'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(gca, 'off');
end

function localPlotEtaTracking(reportData, style)
control = reportData.control;
if ~control.available
    localPlotNoData(gca, 'No eta data: log actuator_state_* and controller command outputs.');
    return;
end

t = control.time;
hold(gca, 'on');
yyaxis left;
plot(t, control.eta_actual(:, 1), 'Color', localSignalColor(style, 'front'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, control.eta_cmd(:, 1), 'Color', localSignalColor(style, 'front'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
plot(t, control.eta_actual(:, 2), 'Color', localSignalColor(style, 'rear'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, control.eta_cmd(:, 2), 'Color', localSignalColor(style, 'rear'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
ylabel('\Omega^2 [RPM^2]');

yyaxis right;
plot(t, rad2deg(control.eta_actual(:, 3)), 'Color', localSignalColor(style, 'deltaf'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, rad2deg(control.eta_cmd(:, 3)), 'Color', localSignalColor(style, 'deltaf'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
plot(t, rad2deg(control.eta_actual(:, 4)), 'Color', localSignalColor(style, 'deltae'), ...
    'LineStyle', style.measuredLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, rad2deg(control.eta_cmd(:, 4)), 'Color', localSignalColor(style, 'deltae'), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
ylabel('\delta_f, \delta_e [deg]');
localForceYAxisBlack(gca);

leg = legend({'\Omega_f^2 act', '\Omega_f^2 cmd', '\Omega_r^2 act', '\Omega_r^2 cmd', ...
    '\delta_f act', '\delta_f cmd', '\delta_e act', '\delta_e cmd'}, ...
    'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(gca, 'off');
end

function localPlotDeltaEta(reportData, style)
debug = reportData.scheduler_debug;
if ~debug.available
    localPlotNoData(gca, ...
        'No scheduler_debug log. Log controller_indi_transition scheduler_debug output.');
    return;
end

t = debug.time;
deltaEta = debug.delta_eta;
hold(gca, 'on');
yyaxis left;
plot(t, deltaEta(:, 1), 'Color', localSignalColor(style, 'front'), ...
    'LineStyle', style.incrementLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, deltaEta(:, 2), 'Color', localSignalColor(style, 'rear'), ...
    'LineStyle', style.incrementLineStyle, 'LineWidth', style.measuredLineWidth);
ylabel('\Delta \Omega^2 [RPM^2]');

yyaxis right;
plot(t, rad2deg(deltaEta(:, 3)), 'Color', localSignalColor(style, 'deltaf'), ...
    'LineStyle', style.incrementLineStyle, 'LineWidth', style.measuredLineWidth);
plot(t, rad2deg(deltaEta(:, 4)), 'Color', localSignalColor(style, 'deltae'), ...
    'LineStyle', style.incrementLineStyle, 'LineWidth', style.measuredLineWidth);
ylabel('\Delta surface [deg]');
localForceYAxisBlack(gca);

leg = legend({'\Delta\Omega_f^2', '\Delta\Omega_r^2', ...
    '\Delta\delta_f', '\Delta\delta_e'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(gca, 'off');
end

function figAirdata = localCreateAirdataFigure(reportData, style, titlePrefix)
figAirdata = figure( ...
    'Name', [titlePrefix ' - Airdata'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [120 90 980 780]);
tiledlayout(figAirdata, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

xLimits = localIndiXLimits(reportData);

nexttile;
localPlotAirdataChannel(reportData, style, 1, 'V_\infty', 'm/s', false);
title('Airspeed');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotAirdataChannel(reportData, style, 2, '\alpha', 'deg', true);
title('Angle of Attack');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

nexttile;
localPlotAirdataChannel(reportData, style, 3, '\beta', 'deg', true);
title('Sideslip');
localFinishIndiAxes(gca, style, reportData.switch_events, xLimits);

titleHandle = sgtitle(figAirdata, [titlePrefix ' - Airdata Debug'], ...
    'FontWeight', 'bold', 'Interpreter', 'none');
titleHandle.Color = 'k';
end

function localPlotAirdataChannel(reportData, style, colIdx, signalName, unitsText, convertToDeg)
airdata = reportData.airdata;
if ~airdata.available
    localPlotNoData(gca, 'No airdata log or body-velocity fallback was available.');
    return;
end

t = reportData.time;
actual = airdata.actual(:, colIdx);
ref = airdata.ref(:, colIdx);
err = ref - actual;
if convertToDeg
    actual = rad2deg(actual);
    ref = rad2deg(ref);
    err = rad2deg(err);
end

color = localAirdataColor(style, colIdx);
hold(gca, 'on');
plot(t, actual, 'Color', color, 'LineStyle', style.measuredLineStyle, ...
    'LineWidth', style.measuredLineWidth);
plot(t, ref, 'Color', color, 'LineStyle', style.commandLineStyle, ...
    'LineWidth', style.commandLineWidth);
plot(t, err, 'Color', color, 'LineStyle', style.errorLineStyle, ...
    'LineWidth', style.errorLineWidth);
ylabel(unitsText);
leg = legend({[signalName ' meas'], [signalName ' ref'], [signalName ' err']}, ...
    'Location', 'best', 'Box', 'off', 'Interpreter', 'tex');
leg.TextColor = 'k';
leg.Color = 'none';
hold(gca, 'off');
end

function color = localAirdataColor(style, colIdx)
switch colIdx
    case 1
        color = localSignalColor(style, 'u');
    case 2
        color = localSignalColor(style, 'w');
    otherwise
        color = localSignalColor(style, 'q');
end
end

function localFinishIndiAxes(ax, style, switchEvents, xLimits)
grid(ax, 'on');
ax.GridColor = style.gridColor;
ax.GridAlpha = 0.55;
ax.XColor = style.axisColor;
ax.YColor = style.axisColor;
ax.Box = 'on';
ax.LineWidth = 0.8;
ax.FontName = 'Helvetica';
ax.FontSize = 9;
ax.Color = 'w';
ax.Title.Color = 'k';
ax.XLabel.Color = 'k';
ax.YLabel.Color = 'k';
xlabel(ax, 'Time [s]');
if ~isempty(xLimits) && all(isfinite(xLimits)) && xLimits(2) > xLimits(1)
    xlim(ax, xLimits);
end
localOverlaySwitchEvents(ax, switchEvents);
end

function localOverlaySwitchEvents(ax, switchEvents)
if isempty(switchEvents) || ~isstruct(switchEvents) || ...
        ~isfield(switchEvents, 'times') || isempty(switchEvents.times)
    return;
end

yl = ylim(ax);
for idx = 1:numel(switchEvents.times)
    line(ax, [switchEvents.times(idx), switchEvents.times(idx)], yl, ...
        'Color', [0.15 0.15 0.15], ...
        'LineStyle', '--', ...
        'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
    if isfield(switchEvents, 'labels') && numel(switchEvents.labels) >= idx
        text(ax, switchEvents.times(idx), yl(2), [' ' char(string(switchEvents.labels{idx}))], ...
            'Color', [0.15 0.15 0.15], ...
            'FontSize', 8, ...
            'VerticalAlignment', 'top', ...
            'HorizontalAlignment', 'left', ...
            'Clipping', 'on', ...
            'HandleVisibility', 'off');
    end
end
ylim(ax, yl);
end

function color = localSignalColor(style, key)
key = lower(char(string(key)));
key = strrep(key, '\theta', 'theta');
key = strrep(key, '\delta', 'delta');
key = regexprep(key, '[^a-z0-9]+', '');
if contains(key, 'theta')
    color = style.colors.theta;
elseif strcmp(key, 'u')
    color = style.colors.u;
elseif strcmp(key, 'w')
    color = style.colors.w;
elseif strcmp(key, 'q')
    color = style.colors.q;
elseif contains(key, 'ax')
    color = style.colors.ax;
elseif contains(key, 'az')
    color = style.colors.az;
elseif contains(key, 'qdot')
    color = style.colors.qdot;
elseif contains(key, 'front') || contains(key, 'omegaf')
    color = style.colors.front;
elseif contains(key, 'rear') || contains(key, 'omegar')
    color = style.colors.rear;
elseif contains(key, 'deltaf')
    color = style.colors.deltaf;
elseif contains(key, 'deltae')
    color = style.colors.deltae;
else
    color = [0.15 0.15 0.15];
end
end

function localForceYAxisBlack(ax)
if numel(ax.YAxis) >= 1
    ax.YAxis(1).Color = 'k';
end
if numel(ax.YAxis) >= 2
    ax.YAxis(2).Color = 'k';
end
end

function localPlotNoData(ax, message)
text(ax, 0.5, 0.5, message, ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'Color', [0.35 0.35 0.35], ...
    'Interpreter', 'none');
end

function ref = localBuildReferenceHistory(controllerData, actual, t, idxLoHistory, idxHiHistory, lambdaHistory)
stateSchedule = controllerData.controller_state_ref;
trimSchedule = controllerData.controller_trim_cmd;
nPts = localInferScheduleCount(stateSchedule, trimSchedule, controllerData);
stateSchedule = stateSchedule(:, 1:nPts);
trimSchedule = trimSchedule(:, 1:nPts);

states = zeros(numel(t), 4);
nuCmd = zeros(numel(t), 3);
etaTrim = zeros(numel(t), 4);
progressSchedule = localGetScheduleVector(stateSchedule, 12, nPts);
if all(abs(progressSchedule) <= 1e-12)
    progressSchedule = linspace(0, 1, nPts);
end
progress = zeros(numel(t), 1);

for k = 1:numel(t)
    idxLo = min(max(idxLoHistory(k), 1), nPts);
    idxHi = min(max(idxHiHistory(k), 1), nPts);
    lambda = min(max(lambdaHistory(k), 0.0), 1.0);

    xRef = localBlendStateColumn(stateSchedule, idxLo, idxHi, lambda);
    trimRef = localBlendTrimColumn(trimSchedule, idxLo, idxHi, lambda);
    outer = localOuterSettings(stateSchedule, idxLo, idxHi, lambda);
    xMeas = localFullStateFromTrack(actual.states(k, :));

    states(k, :) = [xRef(2), xRef(4), xRef(6), xRef(8)];
    nuCmd(k, :) = localCommandedVirtualAcceleration(xRef, xMeas, outer).';
    etaTrim(k, :) = localTrimEta(trimRef).';
    progress(k) = localBlendScalar(progressSchedule, idxLo, idxHi, lambda);
end

ref = struct();
ref.states = states;
ref.nu_cmd = nuCmd;
ref.eta_trim = etaTrim;
ref.progress = progress;
end

function x = localFullStateFromTrack(trackRow)
x = zeros(9, 1);
x(2) = trackRow(1);
x(4) = trackRow(2);
x(6) = trackRow(3);
x(8) = trackRow(4);
end

function nuCmd = localCommandedVirtualAcceleration(xRef, xMeas, outer)
g = 9.81;
thetaRef = xRef(2);
nuCmd = zeros(3, 1);
nuCmd(1) = g * sin(thetaRef) + outer.ku * (xRef(4) - xMeas(4));
nuCmd(2) = -g * cos(thetaRef) + outer.kw * (xRef(6) - xMeas(6));
thetaCmd = localVelocityScheduledTheta(xRef, xMeas);
nuCmd(3) = outer.kq * (xRef(8) - xMeas(8)) + outer.ktheta * (thetaCmd - xMeas(2));
end

function thetaCmd = localVelocityScheduledTheta(xRef, xMeas)
thetaRef = xRef(2);
uRef = xRef(4);
if abs(uRef) < 1.0
    thetaCmd = thetaRef;
    return;
end
uProgress = xMeas(4) / uRef;
uProgress = min(max(uProgress, 0.0), 1.0);
thetaCmd = uProgress * thetaRef;
end

function actual = localBuildActualStateHistory(simSource, t)
actual = struct();
state = zeros(numel(t), 4);

try
    [tEul, yEul, ~] = localGetFirstAvailable(simSource, { ...
        'eul_meas_log', 'attitude_meas', 'eul_truth'});
    eul = localResampleToTime(tEul, localPadColumns(yEul, 3), t);
    state(:, 1) = eul(:, 2);
catch
end

try
    [tAir, yAir, ~] = localGetFirstAvailable(simSource, { ...
        'airDataMeas_log', 'airdata_meas', 'airData_meas'});
    velBody = localAirdataToBodyVelocity(localResampleToTime(tAir, localPadColumns(yAir, 3), t));
catch
    try
        [tVel, yVel, ~] = localGetFirstAvailable(simSource, { ...
            'V_B_truth', 'V_BA_truth', 'V_E_truth'});
        velBody = localResampleToTime(tVel, localPadColumns(yVel, 3), t);
    catch
        velBody = zeros(numel(t), 3);
    end
end
state(:, 2) = velBody(:, 1);
state(:, 3) = velBody(:, 3);

try
    [tRates, yRates, ~] = localGetFirstAvailable(simSource, { ...
        'omega_Meas_log', 'gyro_meas', 'omega_truth'});
    rates = localResampleToTime(tRates, localPadColumns(yRates, 3), t);
    state(:, 4) = rates(:, 2);
catch
end

actual.states = state;
actual.labels = {'theta', 'u', 'w', 'q'};
end

function airdata = localBuildAirdataHistory(simSource, controllerData, actual, t, ...
    idxLoHistory, idxHiHistory, lambdaHistory)
airdata = struct('available', false, 'source', '', 'actual', [], 'ref', [], ...
    'labels', {{'Vinf', 'alpha', 'beta'}});
try
    [tAir, yAir, usedName] = localGetFirstAvailable(simSource, { ...
        'airDataMeas_log', 'airdata_meas', 'airData_meas', ...
        'airData_truth', 'airdata_truth'});
    actualAirdata = localResampleToTime(tAir, localPadColumns(yAir, 3), t);
    sourceName = usedName;
catch
    try
        body = actual.states(:, 2:3);
        u = body(:, 1);
        w = body(:, 2);
        actualAirdata = [sqrt(u.^2 + w.^2), atan2(w, u), zeros(numel(t), 1)];
        sourceName = 'derived_from_u_w';
    catch
        return;
    end
end

refAirdata = localBuildAirdataReference(simSource, controllerData, t, ...
    idxLoHistory, idxHiHistory, lambdaHistory);

airdata.available = true;
airdata.source = sourceName;
airdata.actual = actualAirdata;
airdata.ref = refAirdata;
end

function refAirdata = localBuildAirdataReference(simSource, controllerData, t, ...
    idxLoHistory, idxHiHistory, lambdaHistory)
try
    [tAir, yAir, ~] = localGetFirstAvailable(simSource, {'airData_cmd', 'airdata_cmd'});
    refAirdata = localResampleToTime(tAir, localPadColumns(yAir, 3), t);
    return;
catch
end

nPts = localInferScheduleCount(controllerData.controller_state_ref, ...
    controllerData.controller_trim_cmd, controllerData);
vinf = localGetScheduleVector(controllerData.controller_state_ref, 11, nPts);
if all(abs(vinf) <= 1e-12) && isfield(controllerData, 'schedule_vinf_mps')
    vinf = localRowVector(controllerData.schedule_vinf_mps, nPts);
end
alpha = zeros(1, nPts);
if isfield(controllerData, 'schedule_alpha_deg')
    alpha = deg2rad(localRowVector(controllerData.schedule_alpha_deg, nPts));
end
beta = zeros(1, nPts);
if isfield(controllerData, 'schedule_beta_deg')
    beta = deg2rad(localRowVector(controllerData.schedule_beta_deg, nPts));
end

refAirdata = zeros(numel(t), 3);
for k = 1:numel(t)
    idxLo = min(max(idxLoHistory(k), 1), nPts);
    idxHi = min(max(idxHiHistory(k), 1), nPts);
    lambda = min(max(lambdaHistory(k), 0.0), 1.0);
    refAirdata(k, :) = [ ...
        localBlendScalar(vinf, idxLo, idxHi, lambda), ...
        localBlendScalar(alpha, idxLo, idxHi, lambda), ...
        localBlendScalar(beta, idxLo, idxHi, lambda)];
end
end

function nuMeas = localBuildMeasuredVirtualAcceleration(simSource, t)
values = nan(numel(t), 3);
try
    [tAccel, yAccel, ~] = localGetFirstAvailable(simSource, { ...
        'accel_Meas_log', 'accel_meas', 'specific_force_truth', 'accel'});
    accel = localResampleToTime(tAccel, localPadColumns(yAccel, 3), t);
    values(:, 1) = accel(:, 1);
    values(:, 2) = accel(:, 3);
catch
end

try
    [tQdot, yQdot, ~] = localGetFirstAvailable(simSource, { ...
        'angular_accel_meas', 'angular_accel_truth'});
    qdot = localResampleToTime(tQdot, localPadColumns(yQdot, 3), t);
    values(:, 3) = qdot(:, 2);
catch
end

nuMeas = struct('values', values, 'labels', {{'a_x', 'a_z', 'qdot'}});
end

function control = localBuildControlVariableHistory(simSource, t, ref, schedulerDebug, ...
    controllerData, idxLoHistory, idxHiHistory, lambdaHistory)
control = struct('available', false, 'time', t, ...
    'eta_actual', [], 'eta_cmd', [], 'delta_eta', []);

etaActual = localBuildActualEta(simSource, t);
etaCmd = localBuildCommandEta(simSource, t);
if isempty(etaCmd) && schedulerDebug.available && ~isempty(etaActual)
    deltaEta = localResampleToTime(schedulerDebug.time, schedulerDebug.delta_eta, t);
    etaCmd = localReconstructEtaCommand(etaActual, ref.eta_trim, deltaEta, ...
        controllerData, idxLoHistory, idxHiHistory, lambdaHistory);
end

if isempty(etaActual) || isempty(etaCmd)
    return;
end

control.available = true;
control.eta_actual = etaActual;
control.eta_cmd = etaCmd;
if schedulerDebug.available
    control.delta_eta = localResampleToTime(schedulerDebug.time, schedulerDebug.delta_eta, t);
else
    control.delta_eta = etaCmd - etaActual;
end
end

function etaCmd = localReconstructEtaCommand(etaActual, etaTrim, deltaEta, ...
    controllerData, idxLoHistory, idxHiHistory, lambdaHistory)
etaCmd = [];
if isempty(etaActual) || isempty(etaTrim) || isempty(deltaEta)
    return;
end

n = size(etaActual, 1);
if size(etaTrim, 1) ~= n || size(deltaEta, 1) ~= n
    return;
end

rotorBlend = zeros(n, 1);
surfaceBlend = zeros(n, 1);
if isfield(controllerData, 'controller_gain_lqr') && ~isempty(controllerData.controller_gain_lqr)
    indiSchedule = controllerData.controller_gain_lqr;
    for k = 1:n
        idxLo = idxLoHistory(k);
        idxHi = idxHiHistory(k);
        lambda = lambdaHistory(k);
        rotorBlend(k) = localBlendIndiScheduleScalar( ...
            indiSchedule, 6, 5, idxLo, idxHi, lambda, 0.0);
        surfaceBlend(k) = localBlendIndiScheduleScalar( ...
            indiSchedule, 6, 6, idxLo, idxHi, lambda, 0.0);
    end
end

etaBase = etaActual;
etaBase(:, 1:2) = etaActual(:, 1:2) + ...
    rotorBlend .* (etaTrim(:, 1:2) - etaActual(:, 1:2));
etaBase(:, 3:4) = etaActual(:, 3:4) + ...
    surfaceBlend .* (etaTrim(:, 3:4) - etaActual(:, 3:4));
etaCmd = etaBase + deltaEta;
end

function value = localBlendIndiScheduleScalar(schedule, rowIdx, colIdx, idxLo, idxHi, lambda, fallback)
value = fallback;
if ismatrix(schedule) || size(schedule, 1) < rowIdx || size(schedule, 2) < colIdx
    return;
end

nPts = size(schedule, 3);
idxLo = min(max(round(idxLo), 1), nPts);
idxHi = min(max(round(idxHi), 1), nPts);
lambda = min(max(lambda, 0.0), 1.0);

v0 = schedule(rowIdx, colIdx, idxLo);
v1 = schedule(rowIdx, colIdx, idxHi);
if ~isfinite(v0)
    v0 = fallback;
end
if ~isfinite(v1)
    v1 = fallback;
end
value = (1.0 - lambda) * v0 + lambda * v1;
end

function eta = localBuildActualEta(simSource, t)
eta = [];
try
    [tAct, yAct, ~] = localGetFirstAvailable(simSource, { ...
        'actuator_state_truth', 'actuator_state_meas'});
catch
    return;
end
act = localResampleToTime(tAct, localPadColumns(yAct, 10), t);
frontRpm = 0.5 * (act(:, 1) + act(:, 2));
rearRpm = 0.5 * (act(:, 3) + act(:, 4));
deltaF = 0.5 * (act(:, 7) + act(:, 8));
deltaE = 0.5 * (act(:, 9) + act(:, 10));
eta = [frontRpm.^2, rearRpm.^2, deltaF, deltaE];
end

function eta = localBuildCommandEta(simSource, t)
eta = [];
try
    [tFront, yFront, ~] = localGetFirstAvailable(simSource, { ...
        'front_collective_rpm_out', 'front_collective_rpm_cmd'});
    [tRear, yRear, ~] = localGetFirstAvailable(simSource, { ...
        'rear_collective_rpm_out', 'rear_collective_rpm_cmd'});
    frontCmd = localResampleToTime(tFront, yFront(:, 1), t);
    rearCmd = localResampleToTime(tRear, yRear(:, 1), t);
catch
    return;
end

try
    [tLW, yLW, srcLW] = localGetFirstAvailable(simSource, {'deltaLW_cmd'});
    [tRW, yRW, ~] = localGetFirstAvailable(simSource, {'deltaRW_cmd'});
    [tLT, yLT, ~] = localGetFirstAvailable(simSource, {'deltaLT_cmd'});
    [tRT, yRT, ~] = localGetFirstAvailable(simSource, {'deltaRT_cmd'});
    dLW = localResampleToTime(tLW, yLW(:, 1), t);
    dRW = localResampleToTime(tRW, yRW(:, 1), t);
    dLT = localResampleToTime(tLT, yLT(:, 1), t);
    dRT = localResampleToTime(tRT, yRT(:, 1), t);
    if ~localSignalIsRadians(srcLW)
        dLW = deg2rad(dLW);
        dRW = deg2rad(dRW);
        dLT = deg2rad(dLT);
        dRT = deg2rad(dRT);
    end
    deltaF = 0.5 * (dLW + dRW);
    deltaE = 0.5 * (dLT + dRT);
catch
    deltaF = nan(size(t));
    deltaE = nan(size(t));
end

eta = [frontCmd.^2, rearCmd.^2, deltaF, deltaE];
end

function tf = localSignalIsRadians(sourceName)
tf = any(strcmp(sourceName, {'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd', ...
    'viz_surface_states'}));
end

function schedulerDebug = localBuildSchedulerDebugSeries(simSource, controllerData, t)
schedulerDebug = struct('available', false, 'source', '', 'time', [], 'raw', [], ...
    'nu_err', [], 'delta_eta', []);
try
    [tDebug, yDebug, ~] = localGetFirstAvailable(simSource, { ...
        'scheduler_debug', 'indi_scheduler_debug', ...
        'controller_scheduler_debug', 'controller_indi_scheduler_debug'});
catch
    schedulerDebug = localReplaySchedulerDebugSeries(simSource, controllerData, t);
    return;
end

yDebug = localPadColumns(yDebug, 12);
schedulerDebug.available = true;
schedulerDebug.source = 'logged';
schedulerDebug.time = tDebug;
schedulerDebug.raw = yDebug;
schedulerDebug.nu_err = yDebug(:, 6:8);
schedulerDebug.delta_eta = yDebug(:, 9:12);
end

function schedulerDebug = localReplaySchedulerDebugSeries(simSource, controllerData, t)
schedulerDebug = struct('available', false, 'source', 'replay_failed', ...
    'time', [], 'raw', [], 'nu_err', [], 'delta_eta', []);
try
    actual = localBuildActualStateHistory(simSource, t);
    accel = localBuildAccelReplayInput(simSource, t);
    angularAccel = localBuildAngularAccelReplayInput(simSource, t);
    actuatorState = localBuildActuatorStateReplayInput(simSource, t);
    airDataCmd = localBuildAirDataCommandReplayInput(simSource, controllerData, t);
    tiltCmd = localBuildTiltCommandReplayInput(simSource, controllerData, t);
    params = localIndiLimitParams();
    runtimeGMap = localRuntimeGMapOrDefault(controllerData);

    clear controller_indi_transition;
    raw = zeros(numel(t), 12);
    for k = 1:numel(t)
        xMeas = localFullStateFromTrack(actual.states(k, :));
        [~, ~, debug] = controller_indi_transition( ...
            airDataCmd(k, :).', tiltCmd(k, :).', xMeas, accel(k, :).', ...
            actuatorState(k, :).', angularAccel(k, :).', ...
            controllerData.controller_state_ref, controllerData.controller_trim_cmd, ...
            controllerData.controller_gain_lqr, runtimeGMap, params.surface_limit_rad, ...
            params.front_min_rpm, params.front_max_rpm, ...
            params.rear_min_rpm, params.rear_max_rpm);
        raw(k, :) = localPadColumns(debug(:).', 12);
    end

    schedulerDebug.available = true;
    schedulerDebug.source = 'replayed';
    schedulerDebug.time = t(:);
    schedulerDebug.raw = raw;
    schedulerDebug.nu_err = raw(:, 6:8);
    schedulerDebug.delta_eta = raw(:, 9:12);
catch ME
    schedulerDebug.error = ME.message;
end
end

function runtimeGMap = localRuntimeGMapOrDefault(controllerData)
runtimeGMap = zeros(6, 9, 1);
if isstruct(controllerData) && isfield(controllerData, 'controller_runtime_g_map') && ...
        ~isempty(controllerData.controller_runtime_g_map)
    runtimeGMap = controllerData.controller_runtime_g_map;
end
end

function accel = localBuildAccelReplayInput(simSource, t)
[tAccel, yAccel, ~] = localGetFirstAvailable(simSource, { ...
    'accel_Meas_log', 'accel_meas', 'specific_force_truth', 'accel'});
accel = localResampleToTime(tAccel, localPadColumns(yAccel, 3), t);
end

function angularAccel = localBuildAngularAccelReplayInput(simSource, t)
[tAngular, yAngular, ~] = localGetFirstAvailable(simSource, { ...
    'angular_accel_meas', 'angular_accel_truth'});
angularAccel = localResampleToTime(tAngular, localPadColumns(yAngular, 3), t);
end

function actuatorState = localBuildActuatorStateReplayInput(simSource, t)
[tAct, yAct, ~] = localGetFirstAvailable(simSource, { ...
    'actuator_state_meas', 'actuator_state_truth'});
actuatorState = localResampleToTime(tAct, localPadColumns(yAct, 10), t);
end

function airDataCmd = localBuildAirDataCommandReplayInput(simSource, controllerData, t)
try
    [tAir, yAir, ~] = localGetFirstAvailable(simSource, { ...
        'airData_cmd', 'airdata_cmd'});
    airDataCmd = localResampleToTime(tAir, localPadColumns(yAir, 3), t);
    return;
catch
end

nPts = localInferScheduleCount(controllerData.controller_state_ref, ...
    controllerData.controller_trim_cmd, controllerData);
vinf = localGetScheduleVector(controllerData.controller_state_ref, 11, nPts);
if all(abs(vinf) <= 1e-12) && isfield(controllerData, 'schedule_vinf_mps')
    vinf = localRowVector(controllerData.schedule_vinf_mps, nPts);
end
alpha = zeros(1, nPts);
if isfield(controllerData, 'schedule_alpha_deg')
    alpha = deg2rad(localRowVector(controllerData.schedule_alpha_deg, nPts));
end
airDataCmd = repmat([vinf(1), alpha(1), 0.0], numel(t), 1);
end

function tiltCmd = localBuildTiltCommandReplayInput(simSource, controllerData, t)
try
    [tTilt, yTilt, ~] = localGetFirstAvailable(simSource, { ...
        'tilt_angles_cmd', 'tilt_cmd'});
    tiltCmd = localResampleToTime(tTilt, localPadColumns(yTilt, 2), t);
    return;
catch
end

nPts = localInferScheduleCount(controllerData.controller_state_ref, ...
    controllerData.controller_trim_cmd, controllerData);
tilt = localGetScheduleVector(controllerData.controller_state_ref, 10, nPts);
tiltCmd = repmat([tilt(1), tilt(1)], numel(t), 1);
end

function params = localIndiLimitParams()
params = struct();
params.surface_limit_rad = localWorkspaceScalar('controller_surface_limit_rad', deg2rad(25.0));
params.front_min_rpm = localWorkspaceScalar('front_collective_min_rpm', 0.0);
params.front_max_rpm = localWorkspaceScalar('front_collective_max_rpm', 2400.0);
params.rear_min_rpm = localWorkspaceScalar('rear_collective_min_rpm', 0.0);
params.rear_max_rpm = localWorkspaceScalar('rear_collective_max_rpm', 2400.0);
end

function value = localWorkspaceScalar(varName, fallback)
value = fallback;
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
        candidate = evalin('base', varName);
        if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
            value = candidate;
        end
    end
catch
end
end

function values = localRowVector(valuesIn, nPts)
values = zeros(1, nPts);
count = min(numel(valuesIn), nPts);
if count > 0
    values(1:count) = reshape(valuesIn(1:count), 1, count);
end
if count < nPts && count >= 1
    values(count + 1:nPts) = values(count);
end
end

function [idxLoHistory, idxHiHistory, lambdaHistory] = ...
    localBuildBlendHistory(simSource, t, controllerData, schedulerDebug, switchEvents)
nPts = localInferScheduleCount(controllerData.controller_state_ref, ...
    controllerData.controller_trim_cmd, controllerData);
idxLoHistory = ones(numel(t), 1);
idxHiHistory = ones(numel(t), 1);
lambdaHistory = zeros(numel(t), 1);

if schedulerDebug.available && size(schedulerDebug.raw, 2) >= 4
    idxLoHistory = round(localResampleToTime(schedulerDebug.time, schedulerDebug.raw(:, 2), t));
    idxHiHistory = round(localResampleToTime(schedulerDebug.time, schedulerDebug.raw(:, 3), t));
    lambdaHistory = localResampleToTime(schedulerDebug.time, schedulerDebug.raw(:, 4), t);
    idxLoHistory = min(max(idxLoHistory, 1), nPts);
    idxHiHistory = min(max(idxHiHistory, 1), nPts);
    lambdaHistory = min(max(lambdaHistory, 0.0), 1.0);
    return;
end

segmentRampTime = localSegmentRampTime(controllerData);
if ~isempty(switchEvents.times)
    for k = 1:numel(t)
        [idxLoHistory(k), idxHiHistory(k), lambdaHistory(k)] = ...
            localInternalScheduleBlend(t(k), switchEvents.times, segmentRampTime, nPts);
    end
    return;
end

try
    [tTilt, yTilt, ~] = localGetFirstAvailable(simSource, {'tilt_angles_cmd'});
    tiltActual = localResampleToTime(tTilt, localPadColumns(yTilt, 2), t);
catch
    try
        [tAct, yAct, ~] = localGetFirstAvailable(simSource, {'actuator_state_truth', 'actuator_state_meas'});
        act = localResampleToTime(tAct, localPadColumns(yAct, 10), t);
        tiltActual = act(:, 5:6);
    catch
        return;
    end
end

tiltSchedule = localGetScheduleVector(controllerData.controller_state_ref, 10, nPts);
for k = 1:numel(t)
    [idxLoHistory(k), idxHiHistory(k), lambdaHistory(k)] = ...
        localProjectTiltToBlend(tiltActual(k, 1), tiltSchedule, nPts);
end
end

function switchEvents = localBuildSwitchEvents(simSource, controllerData, schedulerDebug)
switchEvents = struct('times', [], 'labels', {{}});
if schedulerDebug.available && size(schedulerDebug.raw, 2) >= 3
    idxHi = round(schedulerDebug.raw(:, 3));
    dIdx = [0; diff(idxHi)];
    eventIdx = find(dIdx > 0);
    switchEvents.times = schedulerDebug.time(eventIdx);
else
    try
        [tTilt, yTilt, ~] = localGetFirstAvailable(simSource, {'tilt_angles_cmd'});
        tilt = localPadColumns(yTilt, 2);
    catch
        switchEvents.labels = {};
        return;
    end
    dt = diff(tTilt(:));
    if any(dt <= 0)
        switchEvents.labels = {};
        return;
    end
    active = abs(diff(tilt(:, 1)) ./ dt) >= 0.15;
    starts = active & [true; ~active(1:end-1)];
    switchEvents.times = tTilt(starts);
end

nPts = localInferScheduleCount(controllerData.controller_state_ref, ...
    controllerData.controller_trim_cmd, controllerData);
switchEvents.times = switchEvents.times(:);
switchEvents.times = switchEvents.times(1:min(numel(switchEvents.times), max(nPts - 1, 0)));
switchEvents.labels = cell(numel(switchEvents.times), 1);
for idx = 1:numel(switchEvents.times)
    switchEvents.labels{idx} = sprintf('to %d', idx + 1);
end
end

function xLimits = localIndiXLimits(reportData)
t = reportData.time(:);
if isempty(t)
    xLimits = [];
    return;
end

tMin = min(t);
tMax = max(t);
xLimits = [tMin, tMax];
end

function segmentRampTime = localSegmentRampTime(controllerData)
segmentRampTime = 5.0;
if isfield(controllerData, 'gating_opts') && isstruct(controllerData.gating_opts) && ...
        isfield(controllerData.gating_opts, 'segment_ramp_time_s') && ...
        ~isempty(controllerData.gating_opts.segment_ramp_time_s)
    segmentRampTime = controllerData.gating_opts.segment_ramp_time_s;
end
end

function [idxLo, idxHi, lambda] = localInternalScheduleBlend(tNow, transitionStarts, segmentRampTime, nPts)
idxLo = 1;
idxHi = 1;
lambda = 0.0;
if isempty(transitionStarts)
    return;
end

for idx = 1:numel(transitionStarts)
    t0 = transitionStarts(idx);
    t1 = t0 + segmentRampTime;
    if tNow < t0
        idxLo = idx;
        idxHi = idx;
        return;
    end
    if tNow >= t0 && tNow < t1
        idxLo = idx;
        idxHi = min(idx + 1, nPts);
        lambda = min(max((tNow - t0) / max(segmentRampTime, 1e-6), 0.0), 1.0);
        return;
    end
end

idxLo = min(numel(transitionStarts) + 1, nPts);
idxHi = idxLo;
end

function [idxLo, idxHi, lambda] = localProjectTiltToBlend(tiltValue, tiltSchedule, nPts)
idxLo = 1;
idxHi = 1;
lambda = 0.0;
bestErr = inf;
for idx = 1:(nPts - 1)
    t0 = tiltSchedule(idx);
    t1 = tiltSchedule(idx + 1);
    denom = t1 - t0;
    if abs(denom) <= 1e-9
        candidateLambda = 0.0;
        candidateTilt = t0;
    else
        candidateLambda = min(max((tiltValue - t0) / denom, 0.0), 1.0);
        candidateTilt = (1.0 - candidateLambda) * t0 + candidateLambda * t1;
    end
    err = abs(candidateTilt - tiltValue);
    if err < bestErr
        bestErr = err;
        idxLo = idx;
        idxHi = idx + 1;
        lambda = candidateLambda;
    end
end
if lambda >= 1.0 - 1e-8
    idxLo = idxHi;
    lambda = 0.0;
end
end

function nPts = localInferScheduleCount(stateSchedule, trimSchedule, controllerData)
nPts = min(size(stateSchedule, 2), size(trimSchedule, 2));
if isfield(controllerData, 'schedule_count') && ~isempty(controllerData.schedule_count)
    nPts = min(nPts, max(1, round(controllerData.schedule_count)));
    return;
end
if size(stateSchedule, 1) < 12 || nPts <= 1
    return;
end
progress = stateSchedule(12, 1:nPts);
if all(abs(progress) <= 1e-12)
    return;
end
last = 1;
for idx = 2:nPts
    if progress(idx) > progress(idx - 1) + 1e-9
        last = idx;
    elseif abs(progress(idx) - progress(idx - 1)) <= 1e-9 && idx > 2
        break;
    else
        break;
    end
end
nPts = max(1, last);
end

function values = localGetScheduleVector(stateSchedule, rowIdx, nPts)
values = zeros(1, nPts);
if size(stateSchedule, 1) >= rowIdx
    values = stateSchedule(rowIdx, 1:nPts);
end
end

function xRef = localBlendStateColumn(stateSchedule, idxLo, idxHi, lambda)
x0 = zeros(9, 1);
x1 = zeros(9, 1);
rows = min(size(stateSchedule, 1), 9);
x0(1:rows) = stateSchedule(1:rows, idxLo);
x1(1:rows) = stateSchedule(1:rows, idxHi);
xRef = (1.0 - lambda) * x0 + lambda * x1;
end

function trimRef = localBlendTrimColumn(trimSchedule, idxLo, idxHi, lambda)
trimRef = (1.0 - lambda) * trimSchedule(:, idxLo) + lambda * trimSchedule(:, idxHi);
end

function value = localBlendScalar(vec, idxLo, idxHi, lambda)
value = (1.0 - lambda) * vec(idxLo) + lambda * vec(idxHi);
end

function outer = localOuterSettings(stateSchedule, idxLo, idxHi, lambda)
outer = struct();
outer.ku = localBlendScheduleScalarWithDefault(stateSchedule, 20, idxLo, idxHi, lambda, 0.45);
outer.kw = localBlendScheduleScalarWithDefault(stateSchedule, 21, idxLo, idxHi, lambda, 0.45);
outer.kq = localBlendScheduleScalarWithDefault(stateSchedule, 22, idxLo, idxHi, lambda, 1.40);
outer.ktheta = localBlendScheduleScalarWithDefault(stateSchedule, 23, idxLo, idxHi, lambda, 1.80);
end

function value = localBlendScheduleScalarWithDefault(stateSchedule, rowIdx, idxLo, idxHi, lambda, fallback)
if size(stateSchedule, 1) < rowIdx
    value = fallback;
    return;
end
v0 = stateSchedule(rowIdx, idxLo);
v1 = stateSchedule(rowIdx, idxHi);
if ~isfinite(v0) || abs(v0) <= 0
    v0 = fallback;
end
if ~isfinite(v1) || abs(v1) <= 0
    v1 = fallback;
end
value = (1.0 - lambda) * v0 + lambda * v1;
end

function eta = localTrimEta(trimCmd)
frontRpm = max(trimCmd(1), 0.0);
rearRpm = max(trimCmd(2), 0.0);
eta = [frontRpm * frontRpm; rearRpm * rearRpm; trimCmd(3); trimCmd(5)];
end

function velBody = localAirdataToBodyVelocity(airData)
airData = localPadColumns(airData, 3);
vinf = airData(:, 1);
alpha = airData(:, 2);
beta = airData(:, 3);
velBody = [ ...
    vinf .* cos(alpha) .* cos(beta), ...
    vinf .* sin(beta), ...
    vinf .* sin(alpha) .* cos(beta)];
end

function t = localChooseTimeBase(simSource)
try
    t = localGetTout(simSource);
    return;
catch
end

candidateNames = {'eul_meas_log', 'eul_truth', 'airDataMeas_log', 'V_B_truth', 'omega_Meas_log', 'omega_truth'};
for idx = 1:numel(candidateNames)
    try
        [t, ~, ~] = localGetFirstAvailable(simSource, candidateNames(idx));
        return;
    catch
    end
end
error('plot_indi_transition_debug:MissingTimeBase', 'Could not find a usable time base.');
end

function [t, y, usedName] = localGetFirstAvailable(simSource, candidateNames)
for idx = 1:numel(candidateNames)
    name = candidateNames{idx};
    try
        [t, y] = localExtractLoggedSignal(simSource, name);
        usedName = name;
        return;
    catch
    end
end
error('plot_indi_transition_debug:MissingSignal', ...
    'None of these logged signals were found: %s', strjoin(candidateNames, ', '));
end

function [t, y] = localExtractLoggedSignal(simSource, varName)
raw = localGetLoggedField(simSource, varName);
if isa(raw, 'timeseries')
    t = raw.Time(:);
    y = localFormatLoggedArray(raw.Data);
elseif isstruct(raw) && isfield(raw, 'time') && isfield(raw, 'signals') && ...
        isstruct(raw.signals) && isfield(raw.signals, 'values')
    t = raw.time(:);
    y = localFormatLoggedArray(raw.signals.values);
elseif isnumeric(raw)
    t = localGetTout(simSource);
    y = localFormatLoggedArray(raw);
else
    error('plot_indi_transition_debug:UnsupportedLoggedType', ...
        'Unsupported logged type for %s: %s', varName, class(raw));
end

if size(y, 1) ~= numel(t) && size(y, 2) == numel(t)
    y = y.';
end
if size(y, 1) ~= numel(t)
    if numel(t) >= 2
        t = linspace(t(1), t(end), size(y, 1)).';
    else
        t = (0:size(y, 1)-1).';
    end
end
end

function raw = localGetLoggedField(simSource, varName)
raw = [];
if isstruct(simSource) && isfield(simSource, varName)
    raw = simSource.(varName);
    return;
end
if isstruct(simSource) && isfield(simSource, 'out') && ...
        isstruct(simSource.out) && isfield(simSource.out, varName)
    raw = simSource.out.(varName);
    return;
end
try
    raw = simSource.get(varName);
catch
end
if isempty(raw)
    try
        raw = simSource.get(['out.' varName]);
    catch
    end
end
if isempty(raw)
    try
        outStruct = simSource.get('out');
        if isstruct(outStruct) && isfield(outStruct, varName)
            raw = outStruct.(varName);
        end
    catch
    end
end
if isempty(raw)
    error('plot_indi_transition_debug:MissingField', 'Signal %s was not found.', varName);
end
end

function t = localGetTout(simSource)
t = localGetLoggedField(simSource, 'tout');
t = t(:);
end

function y = localFormatLoggedArray(raw)
raw = double(raw);
if isvector(raw)
    y = raw(:);
    return;
end
if ndims(raw) == 3 && size(raw, 2) == 1
    y = squeeze(raw).';
    return;
end
if ndims(raw) == 3
    y = reshape(raw, size(raw, 1) * size(raw, 2), size(raw, 3)).';
    return;
end
y = raw;
end

function yOut = localResampleToTime(tIn, yIn, tOut)
if isequal(numel(tIn), numel(tOut)) && all(abs(tIn(:) - tOut(:)) < 1e-12)
    yOut = yIn;
    return;
end
yOut = interp1(tIn(:), yIn, tOut(:), 'linear', 'extrap');
end

function data = localPadColumns(dataIn, targetCols)
data = dataIn;
if size(data, 2) >= targetCols
    data = data(:, 1:targetCols);
    return;
end
data(:, end + 1:targetCols) = 0;
end

function filenameStem = localResolveFilenameStem(filenameStem, titlePrefix, simSource)
if isempty(filenameStem)
    filenameStem = '';
    return;
end

if islogical(filenameStem) || (isnumeric(filenameStem) && isscalar(filenameStem))
    if logical(filenameStem)
        filenameStem = localDefaultDebugFilenameStem(simSource, titlePrefix);
    else
        filenameStem = '';
    end
    return;
end

filenameStem = char(string(filenameStem));
trimmedStem = strtrim(filenameStem);
if any(strcmpi(trimmedStem, {'save', 'report', 'default'}))
    filenameStem = localDefaultDebugFilenameStem(simSource, titlePrefix);
    return;
end

[outDir, baseName, ext] = fileparts(trimmedStem);
if isempty(outDir)
    filenameStem = fullfile(localWorkspacePlotsDir(), baseName, [baseName ext]);
else
    filenameStem = trimmedStem;
end
end

function filenameStem = localDefaultDebugFilenameStem(simSource, titlePrefix)
baseName = localDefaultReportBaseName(simSource, titlePrefix);
filenameStem = fullfile(localWorkspacePlotsDir(), baseName, baseName);
end

function plotsDir = localWorkspacePlotsDir()
plotsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'workspace_plots');
end

function baseName = localDefaultReportBaseName(simSource, titlePrefix)
candidateNames = { ...
    localWorkspaceStructFieldText('runSpec', 'name'), ...
    localWorkspaceStructFieldText('runResult', 'name'), ...
    localWorkspaceStructFieldText('runCase', 'name'), ...
    localWorkspaceStructFieldText('scenario', 'name'), ...
    localWorkspaceTextValue('scenario_name'), ...
    localSimSourceMetadataName(simSource), ...
    titlePrefix};

for idx = 1:numel(candidateNames)
    if isempty(strtrim(char(string(candidateNames{idx}))))
        continue;
    end
    candidate = localSanitizeFileComponent(candidateNames{idx});
    if ~isempty(candidate)
        baseName = candidate;
        return;
    end
end
baseName = 'indi_transition_debug';
end

function name = localSanitizeFileComponent(rawName)
name = regexprep(char(string(rawName)), '[^A-Za-z0-9_-]+', '_');
name = regexprep(name, '_+', '_');
name = regexprep(name, '^_|_$', '');
end

function txt = localWorkspaceStructFieldText(varName, fieldName)
txt = '';
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
        value = evalin('base', varName);
        if isstruct(value) && isfield(value, fieldName)
            txt = localAsTextScalar(value.(fieldName));
        end
    end
catch
end
end

function txt = localWorkspaceTextValue(varName)
txt = '';
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
        txt = localAsTextScalar(evalin('base', varName));
    end
catch
end
end

function txt = localSimSourceMetadataName(simSource)
txt = '';
try
    metadata = simSource.SimulationMetadata;
    if isprop(metadata, 'ModelInfo') || isfield(metadata, 'ModelInfo')
        modelInfo = metadata.ModelInfo;
        if isprop(modelInfo, 'ModelName') || isfield(modelInfo, 'ModelName')
            txt = localAsTextScalar(modelInfo.ModelName);
        end
    end
catch
end
end

function txt = localAsTextScalar(value)
txt = '';
if isstring(value) && isscalar(value)
    txt = char(value);
elseif ischar(value)
    txt = value;
end
txt = strtrim(txt);
end
