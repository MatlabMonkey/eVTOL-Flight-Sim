function reportData = plot_transition_debug(simSource, cmdSource, filenameStem, titlePrefix)
%PLOT_REPORT_RESPONSES_TRANSITION_DEBUG Create report figures with transition-debug overlays.
%
% Usage:
%   plot_transition_debug
%   plot_transition_debug(out)
%   plot_transition_debug(out, cmds)
%   plot_transition_debug(out, cmds, true)
%   plot_transition_debug(out, cmds, 'save', 'Case 1')
%   plot_transition_debug(out, cmds, 'workspace_plots/report_case1', 'Case 1')
%
% This helper creates:
%   Figure 1: position, velocity, Euler angles, angular rates, angular accel
%   Figure 2: airspeed, tilt states, rotor states, surface states
%   Figure 3: scheduled/reference channels used by the transition controller
%
% For rotor and surface states, if matching command traces are available,
% the command is drawn as a dotted line in the same color as the actual.
%
% Inputs:
%   simSource    Simulink.SimulationOutput, plain logged struct, or omitted
%   cmdSource    cmds struct, plain struct, or omitted
%   filenameStem Optional export stem; saves *_states.png and *_actuators.png.
%                If true or 'save', uses report_plots_final/<title-based-name>.
%                If a bare filename is provided, it is saved under report_plots_final.
%   titlePrefix  Figure prefix title
%
% Output:
%   reportData   Normalized data bundle used for plotting

if nargin < 1 || isempty(simSource)
    if evalin('base', 'exist(''out'', ''var'')')
        simSource = evalin('base', 'out');
    else
        error('plot_transition_debug:MissingSimSource', ...
            'No simSource was provided and no base-workspace variable "out" was found.');
    end
end

if nargin < 2 || isempty(cmdSource)
    if evalin('base', 'exist(''cmds'', ''var'')')
        cmdSource = evalin('base', 'cmds');
    else
        cmdSource = [];
    end
end

if nargin < 3 || isempty(filenameStem)
    filenameStem = '';
end

if nargin < 4 || isempty(titlePrefix)
    titlePrefix = 'eVTOL Response Summary';
end

filenameStem = localResolveFilenameStem(filenameStem, titlePrefix, simSource);

reportData = localNormalizeReportData(simSource, cmdSource);
reportData.transition_path_table = localGetOptionalTransitionPathTable();
reportData.transition_path_markers = localBuildTransitionPathMarkers(reportData.transition_path_table, cmdSource);
reportData.transition_switch_events = localBuildTransitionSwitchEvents(simSource, reportData.transition_path_table);
reportData.controller_data = localGetOptionalControllerData();
reportData.external_scheduled = localBuildScheduledReferenceData(cmdSource);
reportData.internal_scheduled = localBuildInternalGatedReferenceData(simSource, reportData.controller_data, reportData.transition_switch_events);
if reportData.internal_scheduled.available
    reportData.scheduled = reportData.internal_scheduled;
else
    reportData.scheduled = reportData.external_scheduled;
end

style = localReportStyle();

figStates = figure( ...
    'Name', [titlePrefix ' - States'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [80 80 1120 760]);
tiledlayout(figStates, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
localPlotMultichannel(reportData.pos.time, reportData.pos.data, reportData.pos.labels, style);
title('Position');
ylabel('m');
localFinishAxes(gca, style);

nexttile;
localPlotMultichannel(reportData.vel.time, reportData.vel.data, reportData.vel.labels, style);
title(reportData.vel.title);
ylabel('m/s');
localFinishAxes(gca, style);

nexttile;
localPlotMultichannel(reportData.att.time, rad2deg(reportData.att.data(:, 1:3)), {'\phi', '\theta', '\psi'}, style);
title(reportData.att.title);
ylabel('deg');
localFinishAxes(gca, style);

nexttile;
localPlotMultichannel(reportData.rates.time, rad2deg(reportData.rates.data(:, 1:3)), reportData.rates.labels, style);
title(reportData.rates.title);
ylabel('deg/s');
localFinishAxes(gca, style);

nexttile;
localPlotTruthMeasSeries(reportData.angular_accel, style);
title(reportData.angular_accel.title);
ylabel('deg/s^2');
localFinishAxes(gca, style);

titleHandle = sgtitle(figStates, [titlePrefix ' - States'], ...
    'FontWeight', 'bold', 'Interpreter', 'none');
titleHandle.Color = 'k';

figAct = figure( ...
    'Name', [titlePrefix ' - Actuators'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [140 120 1120 760]);
tiledlayout(figAct, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(reportData.airspeed.time, reportData.airspeed.data(:, 1), ...
    'Color', style.colorOrder(1, :), 'LineWidth', style.actualLineWidth);
localOverlayTransitionMarkers(gca, reportData.transition_path_markers, 'airspeed', style);
title('Airspeed');
ylabel('m/s');
leg = legend({'V_\infty'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
localFinishAxes(gca, style);
localOverlaySwitchEvents(gca, reportData.transition_switch_events);
localOverlaySwitchTargets(gca, reportData.transition_switch_events, 'airspeed', style);

nexttile;
localPlotActualAndCommand(reportData.tilt, style, false);
localOverlayTransitionMarkers(gca, reportData.transition_path_markers, 'tilt', style);
title('Tilt States');
ylabel('deg');
localFinishAxes(gca, style);
localOverlaySwitchEvents(gca, reportData.transition_switch_events);
localOverlaySwitchTargets(gca, reportData.transition_switch_events, 'tilt', style);

nexttile;
localPlotActualAndCommand(reportData.rotor, style, true);
localOverlayTransitionMarkers(gca, reportData.transition_path_markers, 'rotor', style);
title(reportData.rotor.title);
ylabel('RPM');
localFinishAxes(gca, style);
localOverlaySwitchEvents(gca, reportData.transition_switch_events);
localOverlaySwitchTargets(gca, reportData.transition_switch_events, 'rotor', style);

nexttile;
localPlotActualAndCommand(reportData.surfaces, style, true);
localOverlayTransitionMarkers(gca, reportData.transition_path_markers, 'surfaces', style);
title('Control Surface States');
ylabel('deg');
localFinishAxes(gca, style);
localOverlaySwitchEvents(gca, reportData.transition_switch_events);
localOverlaySwitchTargets(gca, reportData.transition_switch_events, 'surfaces', style);

titleHandle = sgtitle(figAct, [titlePrefix ' - Airspeed And Actuators'], ...
    'FontWeight', 'bold', 'Interpreter', 'none');
titleHandle.Color = 'k';

reportData.figures = struct('states', figStates, 'actuators', figAct);

if reportData.internal_scheduled.available
    figTracking = localCreateIndiTrackingFigure(reportData, style, titlePrefix);
    reportData.figures.indi_tracking = figTracking;
    figDiagnostics = localCreateIndiDiagnosticsFigure(reportData, style, titlePrefix);
    reportData.figures.indi_diagnostics = figDiagnostics;
end

if reportData.scheduled.available
    figSched = figure( ...
        'Name', [titlePrefix ' - Scheduled Targets'], ...
        'NumberTitle', 'off', ...
        'Color', 'w', ...
        'Position', [200 150 1260 840]);
    tiledlayout(figSched, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    localPlotScheduledSeries(reportData.scheduled.airdata, style);
    title('Scheduled Airdata');
    ylabel('m/s, deg');
    localFinishAxes(gca, style);

    nexttile;
    localPlotScheduledSeries(reportData.scheduled.euler_deg, style);
    title('Scheduled Euler Angles');
    ylabel('deg');
    localFinishAxes(gca, style);

    nexttile;
    localPlotScheduledSeries(reportData.scheduled.tilt_deg, style);
    title('Scheduled Tilt');
    ylabel('deg');
    localFinishAxes(gca, style);

    nexttile;
    localPlotScheduledSeries(reportData.scheduled.rotor_rpm, style);
    title('Scheduled Rotor Commands');
    ylabel('RPM');
    localFinishAxes(gca, style);

    nexttile;
    localPlotScheduledSeries(reportData.scheduled.surface_deg, style);
    title('Scheduled Local Surfaces');
    ylabel('deg');
    localFinishAxes(gca, style);

    nexttile;
    localPlotScheduledSeries(reportData.scheduled.progress, style);
    title('Scheduled Progress');
    ylabel('norm');
    localFinishAxes(gca, style);

    titleSuffix = '';
    if reportData.internal_scheduled.available
        titleSuffix = ' (Internal Gated Targets)';
    elseif reportData.external_scheduled.available
        titleSuffix = ' (External Guide Targets)';
    end
    titleHandle = sgtitle(figSched, [titlePrefix ' - Scheduled Targets' titleSuffix], ...
        'FontWeight', 'bold', 'Interpreter', 'none');
    titleHandle.Color = 'k';
    reportData.figures.scheduled = figSched;
end

if ~isempty(reportData.transition_path_table)
    figPath = localCreateTransitionPathTableFigure(reportData.transition_path_table, titlePrefix);
    reportData.figures.transition_path = figPath;
end

if ~isempty(filenameStem)
    [outDir, baseName, ext] = fileparts(filenameStem);
    if isempty(outDir)
        outDir = localDefaultReportDir();
    end
    if ~isempty(ext)
        baseName = [baseName ext];
    end
    if isempty(baseName)
        baseName = 'report_responses';
    end
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    statesFile = fullfile(outDir, [baseName '_states.png']);
    actuatorsFile = fullfile(outDir, [baseName '_actuators.png']);
    exportgraphics(figStates, statesFile, 'Resolution', 300);
    exportgraphics(figAct, actuatorsFile, 'Resolution', 300);
    reportData.export = struct( ...
        'dir', outDir, ...
        'states_file', statesFile, ...
        'actuators_file', actuatorsFile);
    if isfield(reportData.figures, 'scheduled') && isgraphics(reportData.figures.scheduled)
        scheduledFile = fullfile(outDir, [baseName '_scheduled.png']);
        exportgraphics(reportData.figures.scheduled, scheduledFile, 'Resolution', 300);
        reportData.export.scheduled_file = scheduledFile;
    end
    if isfield(reportData.figures, 'indi_tracking') && isgraphics(reportData.figures.indi_tracking)
        trackingFile = fullfile(outDir, [baseName '_indi_tracking.png']);
        exportgraphics(reportData.figures.indi_tracking, trackingFile, 'Resolution', 300);
        reportData.export.indi_tracking_file = trackingFile;
    end
    if isfield(reportData.figures, 'indi_diagnostics') && isgraphics(reportData.figures.indi_diagnostics)
        diagnosticsFile = fullfile(outDir, [baseName '_indi_diagnostics.png']);
        exportgraphics(reportData.figures.indi_diagnostics, diagnosticsFile, 'Resolution', 300);
        reportData.export.indi_diagnostics_file = diagnosticsFile;
    end
    if isfield(reportData.figures, 'transition_path') && isgraphics(reportData.figures.transition_path)
        transitionPathFile = fullfile(outDir, [baseName '_transition_path.png']);
        exportgraphics(reportData.figures.transition_path, transitionPathFile, 'Resolution', 300);
        reportData.export.transition_path_file = transitionPathFile;
    end
end
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
    filenameStem = fullfile(localDefaultReportDir(), [baseName ext]);
else
    filenameStem = trimmedStem;
end
end

function reportDir = localDefaultReportDir()
reportDir = localWorkspacePlotsDir();
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
    if ~isempty(candidate) && ~strcmp(candidate, 'eVTOL_Response_Summary')
        baseName = candidate;
        return;
    end
end

baseName = 'report_responses';
end

function name = localSanitizeFileComponent(rawName)
name = regexprep(char(string(rawName)), '[^A-Za-z0-9_-]+', '_');
name = regexprep(name, '_+', '_');
name = regexprep(name, '^_|_$', '');
if isempty(name)
    name = 'report_responses';
end
end

function txt = localWorkspaceStructFieldText(varName, fieldName)
txt = '';
existsInBase = evalin('base', sprintf('exist(''%s'', ''var'')', varName));
if ~existsInBase
    return;
end

value = evalin('base', varName);
if isstruct(value) && isfield(value, fieldName)
    txt = localAsTextScalar(value.(fieldName));
end
end

function txt = localWorkspaceTextValue(varName)
txt = '';
existsInBase = evalin('base', sprintf('exist(''%s'', ''var'')', varName));
if ~existsInBase
    return;
end

value = evalin('base', varName);
txt = localAsTextScalar(value);
end

function txt = localSimSourceMetadataName(simSource)
txt = '';
if isempty(simSource)
    return;
end

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

function reportData = localNormalizeReportData(simSource, cmdSource)
reportData = struct();

[tPos, posData, posSource] = localGetFirstAvailable(simSource, { ...
    'pos_NED', 'gps_Pos_Meas_log', 'gpsPosMeas_log', 'gps_pos_meas', 'pos'});
if any(strcmp(posSource, {'gps_Pos_Meas_log', 'gpsPosMeas_log', 'gps_pos_meas', 'pos_NED', 'pos'})) && size(posData, 2) >= 3
    posData(:, 3) = -posData(:, 3);
    posLabels = {'N', 'E', 'alt'};
else
    posLabels = {'x', 'y', 'z'};
end
reportData.pos = struct('time', tPos, 'data', localPadColumns(posData, 3), 'labels', {posLabels});

[tV, V, sourceV] = localGetFirstAvailable(simSource, { ...
    'airDataMeas_log', 'airspeed_meas', 'vinf_truth', 'vinf'});
[tAlpha, alpha, ~] = localGetFirstAvailable(simSource, { ...
    'airDataMeas_log', 'alpha_truth', 'alpha_meas', 'alpha'});
[tBeta, beta, ~] = localGetFirstAvailable(simSource, { ...
    'airDataMeas_log', 'beta_truth', 'beta_meas', 'beta'});
airT = localPickTimeBase(tV, tAlpha, tBeta);
airV = localResampleToTime(tV, V, airT);
airAlpha = localResampleToTime(tAlpha, alpha, airT);
airBeta = localResampleToTime(tBeta, beta, airT);
reportData.airspeed = struct('time', airT, 'data', airV(:, 1), 'source', sourceV);
reportData.airdata = struct('time', airT, 'data', [airV(:, 1), localPickColumn(airAlpha, 2), localPickColumn(airBeta, 3)]);

[tVel, velData, velSource] = localGetFirstAvailable(simSource, { ...
    'V_B_truth', 'V_BA_truth', 'V_E_truth', 'gps_vel_meas', 'vel'});
if strcmp(velSource, 'V_B_truth')
    velLabels = {'u', 'v', 'w'};
    velTitle = 'Body Velocity';
elseif strcmp(velSource, 'V_BA_truth')
    velLabels = {'u_A', 'v_A', 'w_A'};
    velTitle = 'Body-Relative Air Velocity';
else
    velLabels = {'V_N', 'V_E', 'V_D'};
    velTitle = 'Velocity';
end
reportData.vel = struct('time', tVel, 'data', localPadColumns(velData, 3), ...
    'labels', {velLabels}, 'title', velTitle);

[tAtt, attData, attSource] = localGetFirstAvailable(simSource, { ...
    'eul_truth', 'eul_meas_log', 'attitude_meas', 'eul_hat', 'eul'});
if strcmp(attSource, 'eul_truth')
    attTitle = 'Euler Angles Truth';
elseif any(strcmp(attSource, {'eul_meas_log', 'attitude_meas'}))
    attTitle = 'Measured Attitude';
elseif strcmp(attSource, 'eul_hat')
    attTitle = 'Estimated Euler Angles';
else
    attTitle = 'Euler Angles';
end
reportData.att = struct('time', tAtt, 'data', localPadColumns(attData, 3), 'title', attTitle);

[tRates, ratesData, ratesSource] = localGetFirstAvailable(simSource, { ...
    'omega_truth', 'omega_Meas_log', 'gyro_meas', 'omega'});
if strcmp(ratesSource, 'omega_truth')
    ratesTitle = 'Angular Rates Truth';
else
    ratesTitle = 'Angular Rates';
end
reportData.rates = struct('time', tRates, 'data', localPadColumns(ratesData, 3), ...
    'labels', {{'P', 'Q', 'R'}}, 'title', ratesTitle);

reportData.tilt = localBuildTiltSeries(simSource, cmdSource);
reportData.rotor = localBuildRotorSeries(simSource, cmdSource);
reportData.surfaces = localBuildSurfaceSeries(simSource, cmdSource);
reportData.accel = localBuildAccelSeries(simSource);
reportData.angular_accel = localBuildAngularAccelSeries(simSource);
reportData.scheduler_debug = localBuildSchedulerDebugSeries(simSource);
end

function tilt = localBuildTiltSeries(simSource, cmdSource)
actuatorState = localBuildActuatorStateSeries(simSource);
tilt = struct();
if actuatorState.available
    tilt.time = actuatorState.time;
    tilt.actual = actuatorState.tilt_deg;
    tilt.actual_labels = {'FR', 'FL'};
else
    [tActual, yActual, sourceActual] = localGetFirstAvailable(simSource, {'viz_tilt_states', 'tilt_angles_cmd'});
    tilt.time = tActual;
    tilt.actual = localConvertIfNeeded(yActual, sourceActual, 'deg');
    tilt.actual_labels = localMakeDefaultLabels(size(tilt.actual, 2), {'FR', 'FL'});
end
tilt.title = 'Tilt States';
tilt.units = 'deg';
tilt.data = tilt.actual;
tilt.labels = tilt.actual_labels;
tilt.cmd = [];
tilt.cmd_labels = {};

if isempty(cmdSource)
    return;
end

try
    tCmd = localCommandTime(cmdSource.tilt_cmd);
    yCmd = localCommandData(cmdSource.tilt_cmd, 2);
    tilt.cmd = localResampleToTime(tCmd, yCmd, tilt.time);
    tilt.cmd_labels = tilt.actual_labels;
catch
end
end

function rotor = localBuildRotorSeries(simSource, cmdSource)
rotor = struct();
rotor.title = 'Rotor Speed States';
rotor.units = 'rpm';
actuatorState = localBuildActuatorStateSeries(simSource);

if actuatorState.available
    rotor.time = actuatorState.time;
    rotor.actual = actuatorState.rpm;
    rotor.actual_labels = {'FR', 'FL', 'RR', 'RL'};
else
    try
    [tFront, yFront, ~] = localGetFirstAvailable(simSource, {'front_collective_rpm_out'});
    [tRear, yRear, ~] = localGetFirstAvailable(simSource, {'rear_collective_rpm_out'});
    tActual = localPickTimeBase(tFront, tRear);
    frontActual = localResampleToTime(tFront, yFront(:, 1), tActual);
    rearActual = localResampleToTime(tRear, yRear(:, 1), tActual);
    rotor.time = tActual;
    rotor.actual = [frontActual, rearActual];
    rotor.actual_labels = {'Front', 'Rear'};
catch
    [tRotor, yRotor, ~] = localGetFirstAvailable(simSource, {'viz_prop_rpm'});
    rotor.time = tRotor;
    rotor.actual = yRotor;
    rotor.actual_labels = localMakeDefaultLabels(size(yRotor, 2), {'FR', 'FL', 'RR', 'RL'});
    end
end

rotor.data = rotor.actual;
rotor.labels = rotor.actual_labels;
rotor.cmd = [];
rotor.cmd_labels = {};
rotor.controller_cmd_time = [];
rotor.controller_cmd = [];
rotor.controller_cmd_labels = {};

try
    [tCollectiveCmd, yCollectiveCmd] = localTryBuildCollectiveCommandSeries(simSource);
    rotor.controller_cmd_time = tCollectiveCmd;
    rotor.controller_cmd = yCollectiveCmd;
    rotor.controller_cmd_labels = {'Front', 'Rear'};
catch
end

if isempty(cmdSource)
    return;
end

try
    if isfield(cmdSource, 'front_cmd') && isfield(cmdSource, 'rear_cmd') && ...
            (size(rotor.actual, 2) == 2 || size(rotor.actual, 2) == 4)
        tFront = localCommandTime(cmdSource.front_cmd);
        tRear = localCommandTime(cmdSource.rear_cmd);
        yFront = localCommandData(cmdSource.front_cmd, 1);
        yRear = localCommandData(cmdSource.rear_cmd, 1);
        frontCmd = localResampleToTime(tFront, yFront, rotor.time);
        rearCmd = localResampleToTime(tRear, yRear, rotor.time);
        if size(rotor.actual, 2) == 4
            rotor.cmd = [frontCmd, frontCmd, rearCmd, rearCmd];
        else
            rotor.cmd = [frontCmd, rearCmd];
        end
        rotor.cmd_labels = rotor.actual_labels;
    elseif isfield(cmdSource, 'motor_cmd') && size(rotor.actual, 2) == max(1, size(cmdSource.motor_cmd, 2) - 1)
        tCmd = localCommandTime(cmdSource.motor_cmd);
        yCmd = localCommandData(cmdSource.motor_cmd, size(rotor.actual, 2));
        rotor.cmd = localResampleToTime(tCmd, yCmd, rotor.time);
        rotor.cmd_labels = rotor.actual_labels;
    end
catch
end
end

function surfaces = localBuildSurfaceSeries(simSource, cmdSource)
surfaces = struct();
surfaces.title = 'Control Surface States';
surfaces.units = 'deg';
actuatorState = localBuildActuatorStateSeries(simSource);

if actuatorState.available
    surfaces.time = actuatorState.time;
    surfaces.actual = actuatorState.surface_deg;
    surfaces.actual_labels = {'LW', 'RW', 'LT', 'RT'};
else
    try
        [tSurf, ySurf, sourceSurf] = localGetFirstAvailable(simSource, {'deltaLW_cmd'});
        surfaceNames = {'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'};
        Y = ySurf(:, 1);
        for idx = 1:numel(surfaceNames)
            [t_i, y_i, ~] = localGetFirstAvailable(simSource, surfaceNames(idx));
            Y(:, end + 1) = localResampleToTime(t_i, y_i(:, 1), tSurf); %#ok<AGROW>
        end
        surfaces.time = tSurf;
        surfaces.actual = localConvertIfNeeded(Y, sourceSurf, 'deg');
        surfaces.actual_labels = {'LW', 'RW', 'LT', 'RT'};
    catch
        [tSurf, ySurf, sourceSurf] = localGetFirstAvailable(simSource, {'viz_surface_states'});
        surfaces.time = tSurf;
        surfaces.actual = localConvertIfNeeded(localPadColumns(ySurf, 4), sourceSurf, 'deg');
        surfaces.actual_labels = {'LW', 'RW', 'LT', 'RT'};
    end
end

surfaces.data = surfaces.actual;
surfaces.labels = surfaces.actual_labels;
surfaces.cmd = [];
surfaces.cmd_labels = {};
surfaces.controller_cmd_time = [];
surfaces.controller_cmd = [];
surfaces.controller_cmd_labels = {};

try
    [tSurfaceCmd, ySurfaceCmd] = localTryBuildSurfaceCommandSeries(simSource);
    surfaces.controller_cmd_time = tSurfaceCmd;
    surfaces.controller_cmd = ySurfaceCmd;
    surfaces.controller_cmd_labels = {'LW', 'RW', 'LT', 'RT'};
catch
end

if isempty(cmdSource)
    return;
end

try
    if all(isfield(cmdSource, {'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'}))
        cmdNames = {'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'};
        cmdMat = zeros(numel(surfaces.time), 4);
        for idx = 1:4
            tCmd = localCommandTime(cmdSource.(cmdNames{idx}));
            yCmd = localCommandData(cmdSource.(cmdNames{idx}), 1);
            cmdMat(:, idx) = localResampleToTime(tCmd, yCmd, surfaces.time);
        end
        surfaces.cmd = rad2deg(cmdMat);
        surfaces.cmd_labels = surfaces.actual_labels;
    elseif isfield(cmdSource, 'surface_local_cmd')
        tCmd = localCommandTime(cmdSource.surface_local_cmd);
        yCmd = localCommandData(cmdSource.surface_local_cmd, 4);
        surfaces.cmd = rad2deg(localResampleToTime(tCmd, yCmd, surfaces.time));
        surfaces.cmd_labels = surfaces.actual_labels;
    end
catch
end
end

function [tCmd, yCmd] = localTryBuildCollectiveCommandSeries(simSource)
[tFront, yFront, ~] = localGetFirstAvailable(simSource, { ...
    'front_collective_rpm_out', 'front_collective_rpm_cmd'});
[tRear, yRear, ~] = localGetFirstAvailable(simSource, { ...
    'rear_collective_rpm_out', 'rear_collective_rpm_cmd'});
tCmd = localPickTimeBase(tFront, tRear);
frontCmd = localResampleToTime(tFront, yFront(:, 1), tCmd);
rearCmd = localResampleToTime(tRear, yRear(:, 1), tCmd);
yCmd = [frontCmd, rearCmd];
end

function [tCmd, yCmd] = localTryBuildSurfaceCommandSeries(simSource)
[tLW, yLW, srcLW] = localGetFirstAvailable(simSource, {'deltaLW_cmd'});
surfaceNames = {'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'};
tCmd = tLW;
yCmd = yLW(:, 1);
for idx = 1:numel(surfaceNames)
    [tThis, yThis, ~] = localGetFirstAvailable(simSource, surfaceNames(idx));
    yCmd(:, end + 1) = localResampleToTime(tThis, yThis(:, 1), tCmd); %#ok<AGROW>
end
yCmd = localConvertIfNeeded(yCmd, srcLW, 'deg');
end

function actuatorState = localBuildActuatorStateSeries(simSource)
actuatorState = struct( ...
    'available', false, ...
    'time', [], ...
    'rpm', [], ...
    'tilt_deg', [], ...
    'surface_deg', [], ...
    'source', '');

try
    [tAct, yAct, sourceName] = localGetFirstAvailable(simSource, { ...
        'actuator_state_truth', 'actuator_state_meas'});
catch
    return;
end

yAct = localPadColumns(yAct, 10);
actuatorState.available = true;
actuatorState.time = tAct;
actuatorState.rpm = yAct(:, 1:4);
actuatorState.tilt_deg = yAct(:, 5:6);
actuatorState.surface_deg = rad2deg(yAct(:, 7:10));
actuatorState.source = sourceName;
end

function angularAccel = localBuildAngularAccelSeries(simSource)
angularAccel = struct( ...
    'available', false, ...
    'time', [], ...
    'truth', [], ...
    'meas', [], ...
    'truth_labels', {{'Pdot truth', 'Qdot truth', 'Rdot truth'}}, ...
    'meas_labels', {{'Pdot meas', 'Qdot meas', 'Rdot meas'}}, ...
    'title', 'Angular Acceleration');

try
    [tTruth, yTruth, ~] = localGetFirstAvailable(simSource, {'angular_accel_truth'});
    angularAccel.time = tTruth;
    angularAccel.truth = rad2deg(localPadColumns(yTruth, 3));
    angularAccel.available = true;
catch
end

try
    [tMeas, yMeas, ~] = localGetFirstAvailable(simSource, {'angular_accel_meas'});
    yMeas = rad2deg(localPadColumns(yMeas, 3));
    if isempty(angularAccel.time)
        angularAccel.time = tMeas;
        angularAccel.meas = yMeas;
    else
        angularAccel.meas = localResampleToTime(tMeas, yMeas, angularAccel.time);
    end
    angularAccel.available = true;
catch
end
end

function accel = localBuildAccelSeries(simSource)
accel = struct( ...
    'available', false, ...
    'time', [], ...
    'actual', [], ...
    'actual_labels', {{'a_x', 'a_y', 'a_z'}}, ...
    'title', 'Body Specific Force');

try
    [tAccel, yAccel, sourceName] = localGetFirstAvailable(simSource, { ...
        'specific_force_truth', 'accel_Meas_log', 'accel_meas', 'accel'});
catch
    return;
end

accel.available = true;
accel.time = tAccel;
accel.actual = localPadColumns(yAccel, 3);
if strcmp(sourceName, 'accel_Meas_log') || strcmp(sourceName, 'accel_meas')
    accel.title = 'Measured Body Specific Force';
end
end

function schedulerDebug = localBuildSchedulerDebugSeries(simSource)
schedulerDebug = struct( ...
    'available', false, ...
    'time', [], ...
    'nu_err', [], ...
    'delta_eta', [], ...
    'source', '');

try
    [tDebug, yDebug, sourceName] = localGetFirstAvailable(simSource, { ...
        'scheduler_debug', ...
        'indi_scheduler_debug', ...
        'controller_scheduler_debug', ...
        'controller_indi_scheduler_debug'});
catch
    return;
end

yDebug = localPadColumns(yDebug, 12);
schedulerDebug.available = true;
schedulerDebug.time = tDebug;
schedulerDebug.nu_err = yDebug(:, 6:8);
schedulerDebug.delta_eta = yDebug(:, 9:12);
schedulerDebug.source = sourceName;
end

function scheduled = localBuildScheduledReferenceData(cmdSource)
scheduled = struct( ...
    'available', false, ...
    'airdata', localEmptySeries(), ...
    'euler_deg', localEmptySeries(), ...
    'body_velocity', localEmptySeries(), ...
    'body_rates_deg_s', localEmptySeries(), ...
    'tilt_deg', localEmptySeries(), ...
    'rotor_rpm', localEmptySeries(), ...
    'surface_deg', localEmptySeries(), ...
    'accel_cmd', localEmptySeries(), ...
    'angular_accel_cmd', localEmptySeries(), ...
    'progress', localEmptySeries());

if isempty(cmdSource) || ~isstruct(cmdSource)
    return;
end

scheduled.airdata = localCommandSeriesOrEmpty(cmdSource, 'airData_cmd', ...
    {'V_\infty', '\alpha', '\beta'}, [1 2 3], [1, 180/pi, 180/pi]);
scheduled.euler_deg = localCommandSeriesOrEmpty(cmdSource, 'eul_cmd', ...
    {'\phi_{cmd}', '\theta_{cmd}', '\psi_{cmd}'}, [1 2 3], 180/pi);
scheduled.tilt_deg = localCommandSeriesOrEmpty(cmdSource, 'tilt_cmd', ...
    {'FR cmd', 'FL cmd'}, [1 2], 1.0);
scheduled.rotor_rpm = localBuildScheduledRotorSeries(cmdSource);
scheduled.surface_deg = localBuildScheduledSurfaceSeries(cmdSource);
scheduled.progress = localCommandSeriesOrEmpty(cmdSource, 'demo_progress', ...
    {'s'}, 1, 1.0);

seriesList = {scheduled.airdata, scheduled.euler_deg, scheduled.tilt_deg, ...
    scheduled.rotor_rpm, scheduled.surface_deg, scheduled.progress};
scheduled.available = any(cellfun(@(s) ~isempty(s.time) && ~isempty(s.data), seriesList));
end

function series = localEmptySeries()
series = struct('time', [], 'data', [], 'labels', {{}});
end

function series = localCommandSeriesOrEmpty(cmdSource, fieldName, labels, channelIdx, scale)
series = localEmptySeries();
if ~isfield(cmdSource, fieldName)
    return;
end

cmdMatrix = cmdSource.(fieldName);
if isempty(cmdMatrix) || ~isnumeric(cmdMatrix) || size(cmdMatrix, 2) < (max(channelIdx) + 1)
    return;
end

series.time = localCommandTime(cmdMatrix);
series.data = localCommandData(cmdMatrix, max(channelIdx));
series.data = series.data(:, channelIdx);
if isscalar(scale)
    series.data = series.data * scale;
else
    series.data = series.data .* reshape(scale, 1, []);
end
series.labels = labels(:).';
end

function series = localBuildScheduledRotorSeries(cmdSource)
series = localEmptySeries();
if isfield(cmdSource, 'front_cmd') && isfield(cmdSource, 'rear_cmd')
    tFront = localCommandTime(cmdSource.front_cmd);
    tRear = localCommandTime(cmdSource.rear_cmd);
    tCommon = localPickTimeBase(tFront, tRear);
    front = localResampleToTime(tFront, localCommandData(cmdSource.front_cmd, 1), tCommon);
    rear = localResampleToTime(tRear, localCommandData(cmdSource.rear_cmd, 1), tCommon);
    series.time = tCommon;
    series.data = [front(:, 1), rear(:, 1)];
    series.labels = {'Front cmd', 'Rear cmd'};
end
end

function series = localBuildScheduledSurfaceSeries(cmdSource)
series = localEmptySeries();
if isfield(cmdSource, 'surface_local_cmd')
    series = localCommandSeriesOrEmpty(cmdSource, 'surface_local_cmd', ...
        {'LW cmd', 'RW cmd', 'LT cmd', 'RT cmd'}, [1 2 3 4], 180/pi);
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
style.actualLineStyle = '-';
style.commandLineStyle = '--';
style.errorLineStyle = '-';
style.rateLineStyle = ':';
style.referenceLineStyle = '-.';
style.gridColor = [0.86 0.86 0.86];
style.axisColor = [0.15 0.15 0.15];
end

function localPlotMultichannel(t, Y, labels, style)
ax = gca;
colororder(ax, style.colorOrder);
plot(ax, t, Y, 'LineWidth', style.actualLineWidth);
leg = legend(labels, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
end

function localPlotActualAndCommand(series, style, overlayCommand)
ax = gca;
hold(ax, 'on');

nChan = size(series.actual, 2);
colors = style.colorOrder;
for idx = 1:nChan
    color = colors(mod(idx - 1, size(colors, 1)) + 1, :);
    plot(ax, series.time, series.actual(:, idx), ...
        'Color', color, 'LineWidth', style.actualLineWidth);

    if overlayCommand && ~isempty(series.cmd) && size(series.cmd, 2) >= idx
        plot(ax, series.time, series.cmd(:, idx), ...
            'Color', color, ...
            'LineStyle', style.commandLineStyle, ...
            'LineWidth', style.commandLineWidth);
    end
end

legendEntries = localLegendEntries(series, overlayCommand);
leg = legend(legendEntries, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function localPlotTruthMeasSeries(series, style)
ax = gca;
hold(ax, 'on');

if ~isfield(series, 'available') || ~series.available
    text(ax, 0.5, 0.5, 'No angular acceleration data', ...
        'Units', 'normalized', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'Color', [0.35 0.35 0.35]);
    hold(ax, 'off');
    return;
end

legendEntries = {};
colors = style.colorOrder;
if isfield(series, 'truth') && ~isempty(series.truth)
    for idx = 1:size(series.truth, 2)
        color = colors(mod(idx - 1, size(colors, 1)) + 1, :);
        plot(ax, series.time, series.truth(:, idx), ...
            'Color', color, 'LineWidth', style.actualLineWidth);
        legendEntries{end + 1} = series.truth_labels{idx}; %#ok<AGROW>
    end
end

if isfield(series, 'meas') && ~isempty(series.meas)
    for idx = 1:size(series.meas, 2)
        color = colors(mod(idx - 1, size(colors, 1)) + 1, :);
        plot(ax, series.time, series.meas(:, idx), ...
            'Color', color, ...
            'LineStyle', style.commandLineStyle, ...
            'LineWidth', style.commandLineWidth);
        legendEntries{end + 1} = series.meas_labels{idx}; %#ok<AGROW>
    end
end

if ~isempty(legendEntries)
    leg = legend(legendEntries, 'Location', 'best', 'Box', 'off');
    leg.TextColor = 'k';
    leg.Color = 'none';
end
hold(ax, 'off');
end

function localPlotScheduledSeries(series, style)
ax = gca;
if isempty(series.time) || isempty(series.data)
    text(ax, 0.5, 0.5, 'No scheduled data', ...
        'Units', 'normalized', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'Color', [0.35 0.35 0.35]);
    return;
end

colororder(ax, style.colorOrder);
plot(ax, series.time, series.data, 'LineWidth', style.actualLineWidth);
leg = legend(series.labels, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
end

function figTracking = localCreateIndiTrackingFigure(reportData, style, titlePrefix)
figTracking = figure( ...
    'Name', [titlePrefix ' - INDI Tracking'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [120 80 1320 980]);
tiledlayout(figTracking, 5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

scheduled = reportData.internal_scheduled;

nexttile;
localPlotActualVsReference( ...
    reportData.airspeed.time, reportData.airspeed.data(:, 1), {'V_\infty'}, ...
    scheduled.airdata.time, scheduled.airdata.data(:, 1), {'V_\infty'}, style);
title('Airspeed Tracking');
ylabel('m/s');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.airdata.time, rad2deg(reportData.airdata.data(:, 2)), {'\alpha'}, ...
    scheduled.airdata.time, scheduled.airdata.data(:, 2), {'\alpha'}, style);
title('Alpha Tracking');
ylabel('deg');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.att.time, rad2deg(reportData.att.data(:, 2)), {'\theta'}, ...
    scheduled.euler_deg.time, scheduled.euler_deg.data(:, 2), {'\theta'}, style);
title('Pitch Angle Tracking');
ylabel('deg');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.vel.time, reportData.vel.data(:, [1 3]), reportData.vel.labels([1 3]), ...
    scheduled.body_velocity.time, scheduled.body_velocity.data(:, 1:2), {'u', 'w'}, style);
title('Body Velocity Tracking');
ylabel('m/s');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.rates.time, rad2deg(reportData.rates.data(:, 2)), {'q'}, ...
    scheduled.body_rates_deg_s.time, scheduled.body_rates_deg_s.data(:, 2), {'q'}, style);
title('Pitch Rate Tracking');
ylabel('deg/s');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.tilt.time, reportData.tilt.actual, reportData.tilt.actual_labels, ...
    scheduled.tilt_deg.time, scheduled.tilt_deg.data, {'FR', 'FL'}, style);
title('Tilt Tracking');
ylabel('deg');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.rotor.time, reportData.rotor.actual, reportData.rotor.actual_labels, ...
    scheduled.rotor_rpm.time, scheduled.rotor_rpm.data, {'FR', 'FL', 'RR', 'RL'}, style);
title('Rotor RPM Tracking');
ylabel('RPM');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
localPlotActualVsReference( ...
    reportData.surfaces.time, reportData.surfaces.actual, reportData.surfaces.actual_labels, ...
    scheduled.surface_deg.time, scheduled.surface_deg.data, {'LW', 'RW', 'LT', 'RT'}, style);
title('Surface Tracking');
ylabel('deg');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
if reportData.accel.available
    localPlotActualVsReference( ...
        reportData.accel.time, reportData.accel.actual(:, [1 3]), {'a_x', 'a_z'}, ...
        scheduled.accel_cmd.time, scheduled.accel_cmd.data, {'a_x', 'a_z'}, style);
else
    localPlotNoData(gca, 'No acceleration data');
end
title('Specific Force Target');
ylabel('m/s^2');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

nexttile;
[tQdot, qdotActual, qdotLabel] = localBestPitchAngularAccel(reportData.angular_accel);
if ~isempty(tQdot)
    localPlotActualVsReference( ...
        tQdot, qdotActual, {qdotLabel}, ...
        scheduled.angular_accel_cmd.time, scheduled.angular_accel_cmd.data(:, 1), {'qdot'}, style);
else
    localPlotNoData(gca, 'No angular acceleration data');
end
title('Pitch Angular Accel Target');
ylabel('deg/s^2');
localFinishTrackingAxes(gca, style, reportData.transition_switch_events);

titleHandle = sgtitle(figTracking, ...
    [titlePrefix ' - INDI Actual vs Internal Target'], ...
    'FontWeight', 'bold', 'Interpreter', 'none');
titleHandle.Color = 'k';
end

function localFinishTrackingAxes(ax, style, switchEvents)
localFinishAxes(ax, style);
localOverlaySwitchEvents(ax, switchEvents);
end

function localPlotActualVsReference(tActual, actualData, actualLabels, tRef, refData, refLabels, style)
ax = gca;
if isempty(tActual) || isempty(actualData) || isempty(tRef) || isempty(refData)
    localPlotNoData(ax, 'No tracking data');
    return;
end

actualData = localEnsureMatrix(actualData);
refData = localEnsureMatrix(refData);
[actualData, actualLabels, refData, refLabels] = ...
    localAlignActualReferenceChannels(actualData, actualLabels, refData, refLabels);

hold(ax, 'on');
legendEntries = {};
for idx = 1:size(actualData, 2)
    color = localSignalColor(style, actualLabels{idx}, idx);
    plot(ax, tActual, actualData(:, idx), ...
        'Color', color, ...
        'LineStyle', style.actualLineStyle, ...
        'LineWidth', style.actualLineWidth);
    plot(ax, tRef, refData(:, idx), ...
        'Color', color, ...
        'LineStyle', style.commandLineStyle, ...
        'LineWidth', style.commandLineWidth);
    legendEntries{end + 1} = sprintf('%s actual', actualLabels{idx}); %#ok<AGROW>
    legendEntries{end + 1} = sprintf('%s target', refLabels{idx}); %#ok<AGROW>
end

leg = legend(legendEntries, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function data = localEnsureMatrix(data)
if isvector(data)
    data = data(:);
end
end

function [actualData, actualLabels, refData, refLabels] = ...
    localAlignActualReferenceChannels(actualData, actualLabels, refData, refLabels)
nActual = size(actualData, 2);
nRef = size(refData, 2);

if nActual == nRef
    actualLabels = localMakeDefaultLabels(nActual, actualLabels);
    refLabels = localMakeDefaultLabels(nRef, refLabels);
    return;
end

if nActual == 2 && nRef == 4
    refData = [mean(refData(:, 1:2), 2), mean(refData(:, 3:4), 2)];
    refLabels = {'Front', 'Rear'};
elseif nActual == 4 && nRef == 2
    refData = [refData(:, 1), refData(:, 1), refData(:, 2), refData(:, 2)];
    refLabels = {'FR', 'FL', 'RR', 'RL'};
else
    keepCount = min(nActual, nRef);
    actualData = actualData(:, 1:keepCount);
    refData = refData(:, 1:keepCount);
end

nKeep = min(size(actualData, 2), size(refData, 2));
actualData = actualData(:, 1:nKeep);
refData = refData(:, 1:nKeep);
actualLabels = localMakeDefaultLabels(nKeep, actualLabels);
refLabels = localMakeDefaultLabels(nKeep, refLabels);
end

function [tQdot, qdotActual, qdotLabel] = localBestPitchAngularAccel(angularAccel)
tQdot = [];
qdotActual = [];
qdotLabel = 'qdot';

if ~isstruct(angularAccel) || ~isfield(angularAccel, 'available') || ~angularAccel.available
    return;
end

if isfield(angularAccel, 'truth') && ~isempty(angularAccel.truth)
    tQdot = angularAccel.time;
    qdotActual = angularAccel.truth(:, 2);
    qdotLabel = 'qdot truth';
elseif isfield(angularAccel, 'meas') && ~isempty(angularAccel.meas)
    tQdot = angularAccel.time;
    qdotActual = angularAccel.meas(:, 2);
    qdotLabel = 'qdot meas';
end
end

function localPlotNoData(ax, message)
text(ax, 0.5, 0.5, message, ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'Color', [0.35 0.35 0.35]);
end

function figDiagnostics = localCreateIndiDiagnosticsFigure(reportData, style, titlePrefix)
figDiagnostics = figure( ...
    'Name', [titlePrefix ' - INDI Diagnostics'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [130 80 1320 1050]);
tiledlayout(figDiagnostics, 5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

xLimits = localIndiDiagnosticsXLimits(reportData);
virtualDiag = localBuildVirtualAccelerationDiagnostics(reportData);
rotorDiag = localBuildRotorDiagnostics(reportData);
surfaceDiag = localBuildSurfaceDiagnostics(reportData);
limitDiag = localBuildLimitDiagnostics(rotorDiag, surfaceDiag);

nexttile([1 2]);
localPlotVirtualAccelerationTracking(virtualDiag, style);
title('Virtual Acceleration Tracking');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

nexttile([1 2]);
localPlotVirtualAccelerationError(virtualDiag, style);
title('Virtual Acceleration Error');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

nexttile;
localPlotRotorCommandActual(rotorDiag, style);
title('Rotor Command vs Actual');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

nexttile;
localPlotRotorErrorAndRate(rotorDiag, style);
title('Rotor Tracking Error and Rate');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

nexttile;
localPlotSurfaceCommandActual(surfaceDiag, style);
title('Surface Command vs Actual');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

nexttile;
localPlotIndiIncrementRequest(reportData.scheduler_debug, style);
title('INDI Increment Request');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

nexttile([1 2]);
localPlotLimitIndicators(limitDiag, style);
title('Saturation / Limit Indicators');
localFinishDiagnosticsAxes(gca, style, reportData.transition_switch_events, xLimits);

titleHandle = sgtitle(figDiagnostics, ...
    [titlePrefix ' - INDI Diagnostics'], ...
    'FontWeight', 'bold', 'Interpreter', 'none');
titleHandle.Color = 'k';
end

function localFinishDiagnosticsAxes(ax, style, switchEvents, xLimits)
localFinishAxes(ax, style);
localApplyDiagnosticsXLimits(ax, xLimits);
localOverlaySwitchEvents(ax, switchEvents);
end

function xLimits = localIndiDiagnosticsXLimits(reportData)
t = [];
if isfield(reportData, 'internal_scheduled') && ...
        isfield(reportData.internal_scheduled, 'airdata') && ...
        ~isempty(reportData.internal_scheduled.airdata.time)
    t = reportData.internal_scheduled.airdata.time(:);
elseif isfield(reportData, 'airspeed') && ~isempty(reportData.airspeed.time)
    t = reportData.airspeed.time(:);
end

if isempty(t)
    xLimits = [];
    return;
end

tMin = min(t);
tMax = max(t);
windowLength = 15.0;
if isfield(reportData, 'transition_switch_events') && ...
        isfield(reportData.transition_switch_events, 'times') && ...
        ~isempty(reportData.transition_switch_events.times)
    tStart = max(tMin, reportData.transition_switch_events.times(1) - 2.0);
else
    tStart = max(tMin, 0.0);
end
tEnd = min(tMax, tStart + windowLength);
if tEnd <= tStart
    xLimits = [tMin, tMax];
    return;
end
if tEnd - tStart < min(windowLength, tMax - tMin)
    tStart = max(tMin, tEnd - windowLength);
end
xLimits = [tStart, tEnd];
end

function localApplyDiagnosticsXLimits(ax, xLimits)
if ~isempty(xLimits) && all(isfinite(xLimits)) && xLimits(2) > xLimits(1)
    xlim(ax, xLimits);
end
end

function virtualDiag = localBuildVirtualAccelerationDiagnostics(reportData)
virtualDiag = struct('available', false, 'time', [], 'actual', [], 'cmd', []);

if isempty(reportData.internal_scheduled.accel_cmd.time) || ...
        isempty(reportData.internal_scheduled.accel_cmd.data)
    return;
end

t = reportData.internal_scheduled.accel_cmd.time(:);
cmdAccel = localPadColumns(reportData.internal_scheduled.accel_cmd.data, 2);
cmdQdot = nan(numel(t), 1);
if ~isempty(reportData.internal_scheduled.angular_accel_cmd.time) && ...
        ~isempty(reportData.internal_scheduled.angular_accel_cmd.data)
    cmdQdot = localResampleToTime( ...
        reportData.internal_scheduled.angular_accel_cmd.time, ...
        reportData.internal_scheduled.angular_accel_cmd.data(:, 1), t);
end

actualAccel = nan(numel(t), 2);
if reportData.accel.available
    actualAccel = localResampleToTime( ...
        reportData.accel.time, reportData.accel.actual(:, [1 3]), t);
end

actualQdot = nan(numel(t), 1);
[tQdot, qdotActual, ~] = localBestPitchAngularAccel(reportData.angular_accel);
if ~isempty(tQdot)
    actualQdot = localResampleToTime(tQdot, qdotActual, t);
end

virtualDiag.available = true;
virtualDiag.time = t;
virtualDiag.actual = [actualAccel, actualQdot];
virtualDiag.cmd = [cmdAccel(:, 1:2), cmdQdot];
end

function localPlotVirtualAccelerationTracking(virtualDiag, style)
ax = gca;
if ~virtualDiag.available
    localPlotNoData(ax, 'No virtual acceleration data');
    return;
end

hold(ax, 'on');
yyaxis(ax, 'left');
plot(ax, virtualDiag.time, virtualDiag.actual(:, 1), ...
    'Color', localSignalColor(style, 'a_x', 1), ...
    'LineStyle', style.actualLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, virtualDiag.time, virtualDiag.cmd(:, 1), ...
    'Color', localSignalColor(style, 'a_x', 1), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
plot(ax, virtualDiag.time, virtualDiag.actual(:, 2), ...
    'Color', localSignalColor(style, 'a_z', 2), ...
    'LineStyle', style.actualLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, virtualDiag.time, virtualDiag.cmd(:, 2), ...
    'Color', localSignalColor(style, 'a_z', 2), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
ylabel(ax, 'a_x, a_z [m/s^2]');

yyaxis(ax, 'right');
plot(ax, virtualDiag.time, virtualDiag.actual(:, 3), ...
    'Color', localSignalColor(style, 'qdot', 3), ...
    'LineStyle', style.actualLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, virtualDiag.time, virtualDiag.cmd(:, 3), ...
    'Color', localSignalColor(style, 'qdot', 3), ...
    'LineStyle', style.commandLineStyle, 'LineWidth', style.commandLineWidth);
ylabel(ax, 'qdot [deg/s^2]');
localForceYAxisBlack(ax);

leg = legend({'a_x actual', 'a_x cmd', 'a_z actual', 'a_z cmd', ...
    'qdot actual', 'qdot cmd'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function localPlotVirtualAccelerationError(virtualDiag, style)
ax = gca;
if ~virtualDiag.available
    localPlotNoData(ax, 'No virtual acceleration data');
    return;
end

err = virtualDiag.cmd - virtualDiag.actual;
hold(ax, 'on');
yyaxis(ax, 'left');
plot(ax, virtualDiag.time, err(:, 1), ...
    'Color', localSignalColor(style, 'a_x', 1), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, virtualDiag.time, err(:, 2), ...
    'Color', localSignalColor(style, 'a_z', 2), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
ylabel(ax, 'a cmd - actual [m/s^2]');

yyaxis(ax, 'right');
plot(ax, virtualDiag.time, err(:, 3), ...
    'Color', localSignalColor(style, 'qdot', 3), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
ylabel(ax, 'qdot cmd - actual [deg/s^2]');
localForceYAxisBlack(ax);

leg = legend({'a_x err', 'a_z err', 'qdot err'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function rotorDiag = localBuildRotorDiagnostics(reportData)
rotorDiag = struct( ...
    'available', false, ...
    'time_actual', [], ...
    'actual', [], ...
    'time_cmd', [], ...
    'cmd', [], ...
    'actual_on_cmd', [], ...
    'error', [], ...
    'rate_time', [], ...
    'rate', [], ...
    'rate_on_cmd', []);

if isempty(reportData.rotor.time) || isempty(reportData.rotor.actual)
    return;
end

rotorDiag.time_actual = reportData.rotor.time(:);
rotorDiag.actual = localRotorCollectives(reportData.rotor.actual);

[tCmd, cmd] = localRotorCommandForDiagnostics(reportData);
if isempty(tCmd) || isempty(cmd)
    return;
end

rotorDiag.time_cmd = tCmd(:);
rotorDiag.cmd = localRotorCollectives(cmd);
rotorDiag.actual_on_cmd = localResampleToTime( ...
    rotorDiag.time_actual, rotorDiag.actual, rotorDiag.time_cmd);
rotorDiag.error = rotorDiag.cmd - rotorDiag.actual_on_cmd;
rotorDiag.rate_time = rotorDiag.time_actual;
rotorDiag.rate = localTimeDerivative(rotorDiag.time_actual, rotorDiag.actual);
rotorDiag.rate_on_cmd = localResampleToTime( ...
    rotorDiag.rate_time, rotorDiag.rate, rotorDiag.time_cmd);
rotorDiag.available = true;
end

function collectives = localRotorCollectives(data)
data = localEnsureMatrix(data);
if size(data, 2) >= 4
    collectives = [mean(data(:, 1:2), 2), mean(data(:, 3:4), 2)];
elseif size(data, 2) >= 2
    collectives = data(:, 1:2);
else
    collectives = [data(:, 1), data(:, 1)];
end
end

function [tCmd, cmd] = localRotorCommandForDiagnostics(reportData)
tCmd = [];
cmd = [];
if isfield(reportData.rotor, 'controller_cmd') && ~isempty(reportData.rotor.controller_cmd)
    tCmd = reportData.rotor.controller_cmd_time;
    cmd = reportData.rotor.controller_cmd;
elseif isfield(reportData.rotor, 'cmd') && ~isempty(reportData.rotor.cmd)
    tCmd = reportData.rotor.time;
    cmd = reportData.rotor.cmd;
elseif isfield(reportData, 'internal_scheduled') && ...
        ~isempty(reportData.internal_scheduled.rotor_rpm.time)
    tCmd = reportData.internal_scheduled.rotor_rpm.time;
    cmd = reportData.internal_scheduled.rotor_rpm.data;
end
end

function localPlotRotorCommandActual(rotorDiag, style)
ax = gca;
if ~rotorDiag.available
    localPlotNoData(ax, 'No rotor command/actual data');
    return;
end

hold(ax, 'on');
labels = {'Front', 'Rear'};
for idx = 1:2
    color = localSignalColor(style, labels{idx}, idx);
    plot(ax, rotorDiag.time_actual, rotorDiag.actual(:, idx), ...
        'Color', color, ...
        'LineStyle', style.actualLineStyle, ...
        'LineWidth', style.actualLineWidth);
    plot(ax, rotorDiag.time_cmd, rotorDiag.cmd(:, idx), ...
        'Color', color, ...
        'LineStyle', style.commandLineStyle, ...
        'LineWidth', style.commandLineWidth);
end
ylabel(ax, 'RPM');
leg = legend({[labels{1} ' actual'], [labels{1} ' cmd'], ...
    [labels{2} ' actual'], [labels{2} ' cmd']}, ...
    'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function localPlotRotorErrorAndRate(rotorDiag, style)
ax = gca;
if ~rotorDiag.available
    localPlotNoData(ax, 'No rotor error/rate data');
    return;
end

hold(ax, 'on');
yyaxis(ax, 'left');
plot(ax, rotorDiag.time_cmd, rotorDiag.error(:, 1), ...
    'Color', localSignalColor(style, 'front', 1), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, rotorDiag.time_cmd, rotorDiag.error(:, 2), ...
    'Color', localSignalColor(style, 'rear', 2), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
ylabel(ax, 'cmd - actual [RPM]');

yyaxis(ax, 'right');
plot(ax, rotorDiag.rate_time, rotorDiag.rate(:, 1), ...
    'Color', localSignalColor(style, 'front', 1), ...
    'LineStyle', style.rateLineStyle, 'LineWidth', style.commandLineWidth);
plot(ax, rotorDiag.rate_time, rotorDiag.rate(:, 2), ...
    'Color', localSignalColor(style, 'rear', 2), ...
    'LineStyle', style.rateLineStyle, 'LineWidth', style.commandLineWidth);
yline(ax, 1000, 'Color', [0.15 0.15 0.15], ...
    'LineStyle', style.referenceLineStyle, 'Label', '+1000 RPM/s', ...
    'HandleVisibility', 'off');
yline(ax, -1000, 'Color', [0.15 0.15 0.15], ...
    'LineStyle', style.referenceLineStyle, 'Label', '-1000 RPM/s', ...
    'HandleVisibility', 'off');
ylabel(ax, 'actual rate [RPM/s]');
localForceYAxisBlack(ax);

leg = legend({'Front err', 'Rear err', 'Front rate', 'Rear rate'}, ...
    'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function surfaceDiag = localBuildSurfaceDiagnostics(reportData)
surfaceDiag = struct( ...
    'available', false, ...
    'time_actual', [], ...
    'actual', [], ...
    'time_cmd', [], ...
    'cmd', [], ...
    'actual_on_cmd', [], ...
    'error', []);

if isempty(reportData.surfaces.time) || isempty(reportData.surfaces.actual)
    return;
end

surfaceDiag.time_actual = reportData.surfaces.time(:);
surfaceDiag.actual = localLongitudinalSurfaces(reportData.surfaces.actual);

[tCmd, cmd] = localSurfaceCommandForDiagnostics(reportData);
if isempty(tCmd) || isempty(cmd)
    return;
end

surfaceDiag.time_cmd = tCmd(:);
surfaceDiag.cmd = localLongitudinalSurfaces(cmd);
surfaceDiag.actual_on_cmd = localResampleToTime( ...
    surfaceDiag.time_actual, surfaceDiag.actual, surfaceDiag.time_cmd);
surfaceDiag.error = surfaceDiag.cmd - surfaceDiag.actual_on_cmd;
surfaceDiag.available = true;
end

function longitudinal = localLongitudinalSurfaces(data)
data = localEnsureMatrix(data);
if size(data, 2) >= 4
    longitudinal = [0.5 * (data(:, 1) + data(:, 2)), ...
        0.5 * (data(:, 3) + data(:, 4))];
elseif size(data, 2) >= 2
    longitudinal = data(:, 1:2);
else
    longitudinal = [data(:, 1), data(:, 1)];
end
end

function [tCmd, cmd] = localSurfaceCommandForDiagnostics(reportData)
tCmd = [];
cmd = [];
if isfield(reportData.surfaces, 'controller_cmd') && ~isempty(reportData.surfaces.controller_cmd)
    tCmd = reportData.surfaces.controller_cmd_time;
    cmd = reportData.surfaces.controller_cmd;
elseif isfield(reportData.surfaces, 'cmd') && ~isempty(reportData.surfaces.cmd)
    tCmd = reportData.surfaces.time;
    cmd = reportData.surfaces.cmd;
elseif isfield(reportData, 'internal_scheduled') && ...
        ~isempty(reportData.internal_scheduled.surface_deg.time)
    tCmd = reportData.internal_scheduled.surface_deg.time;
    cmd = reportData.internal_scheduled.surface_deg.data;
end
end

function localPlotSurfaceCommandActual(surfaceDiag, style)
ax = gca;
if ~surfaceDiag.available
    localPlotNoData(ax, 'No surface command/actual data');
    return;
end

hold(ax, 'on');
labels = {'\delta_f', '\delta_e'};
for idx = 1:2
    color = localSignalColor(style, labels{idx}, idx);
    plot(ax, surfaceDiag.time_actual, surfaceDiag.actual(:, idx), ...
        'Color', color, ...
        'LineStyle', style.actualLineStyle, ...
        'LineWidth', style.actualLineWidth);
    plot(ax, surfaceDiag.time_cmd, surfaceDiag.cmd(:, idx), ...
        'Color', color, ...
        'LineStyle', style.commandLineStyle, ...
        'LineWidth', style.commandLineWidth);
end
ylabel(ax, 'deg');
leg = legend({[labels{1} ' actual'], [labels{1} ' cmd'], ...
    [labels{2} ' actual'], [labels{2} ' cmd']}, ...
    'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function localPlotIndiIncrementRequest(schedulerDebug, style)
ax = gca;
if ~isstruct(schedulerDebug) || ~isfield(schedulerDebug, 'available') || ...
        ~schedulerDebug.available
    localPlotNoData(ax, ...
        'No scheduler_debug log. Log controller_indi_transition scheduler_debug output.');
    return;
end

deltaEta = schedulerDebug.delta_eta;
hold(ax, 'on');
yyaxis(ax, 'left');
plot(ax, schedulerDebug.time, deltaEta(:, 1), ...
    'Color', localSignalColor(style, 'front', 1), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, schedulerDebug.time, deltaEta(:, 2), ...
    'Color', localSignalColor(style, 'rear', 2), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
ylabel(ax, '\Delta RPM^2');

yyaxis(ax, 'right');
plot(ax, schedulerDebug.time, rad2deg(deltaEta(:, 3)), ...
    'Color', localSignalColor(style, '\delta_f', 3), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
plot(ax, schedulerDebug.time, rad2deg(deltaEta(:, 4)), ...
    'Color', localSignalColor(style, '\delta_e', 4), ...
    'LineStyle', style.errorLineStyle, 'LineWidth', style.actualLineWidth);
ylabel(ax, '\Delta surface [deg]');
localForceYAxisBlack(ax);

leg = legend({'\Delta front RPM^2', '\Delta rear RPM^2', ...
    '\Delta \delta_f', '\Delta \delta_e'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
hold(ax, 'off');
end

function limitDiag = localBuildLimitDiagnostics(rotorDiag, surfaceDiag)
limitDiag = struct('available', false, 'time', [], 'flags', [], 'labels', {{}});

if rotorDiag.available
    t = rotorDiag.time_cmd(:);
elseif surfaceDiag.available
    t = surfaceDiag.time_cmd(:);
else
    return;
end

flags = [];
labels = {};

if rotorDiag.available
    rotorLimits = localRotorLimitValues();
    frontCmd = rotorDiag.cmd(:, 1);
    rearCmd = rotorDiag.cmd(:, 2);
    frontRate = rotorDiag.rate_on_cmd(:, 1);
    rearRate = rotorDiag.rate_on_cmd(:, 2);
    flags(:, end + 1) = localRotorAtLimit(frontCmd, rotorLimits.front_min, rotorLimits.front_max);
    labels{end + 1} = 'front rotor cmd limit';
    flags(:, end + 1) = localRotorAtLimit(rearCmd, rotorLimits.rear_min, rotorLimits.rear_max);
    labels{end + 1} = 'rear rotor cmd limit';
    flags(:, end + 1) = abs(frontRate) >= 950.0;
    labels{end + 1} = 'front rate near 1000';
    flags(:, end + 1) = abs(rearRate) >= 950.0;
    labels{end + 1} = 'rear rate near 1000';
end

if surfaceDiag.available
    surfaceLimitDeg = localSurfaceLimitDeg();
    surfaceCmd = localResampleToTime(surfaceDiag.time_cmd, surfaceDiag.cmd, t);
    flags(:, end + 1) = any(abs(surfaceCmd) >= 0.98 * surfaceLimitDeg, 2);
    labels{end + 1} = 'surface near limit';
end

if isempty(flags)
    return;
end

limitDiag.available = true;
limitDiag.time = t;
limitDiag.flags = double(flags);
limitDiag.labels = labels;
end

function limits = localRotorLimitValues()
limits = struct();
limits.front_min = localWorkspaceNumericScalar('front_collective_min_rpm', 0.0);
limits.front_max = localWorkspaceNumericScalar('front_collective_max_rpm', 7000.0);
limits.rear_min = localWorkspaceNumericScalar('rear_collective_min_rpm', 0.0);
limits.rear_max = localWorkspaceNumericScalar('rear_collective_max_rpm', 7000.0);
end

function flag = localRotorAtLimit(cmd, minValue, maxValue)
range = max(maxValue - minValue, 1.0);
tol = max(5.0, 0.002 * range);
flag = (cmd <= minValue + tol) | (cmd >= maxValue - tol);
end

function surfaceLimitDeg = localSurfaceLimitDeg()
surfaceLimitRad = localWorkspaceNumericScalar('controller_surface_limit_rad', deg2rad(25.0));
surfaceLimitDeg = rad2deg(surfaceLimitRad);
if ~isfinite(surfaceLimitDeg) || surfaceLimitDeg <= 0
    surfaceLimitDeg = 25.0;
end
end

function value = localWorkspaceNumericScalar(varName, fallback)
value = fallback;
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
        candidate = evalin('base', varName);
        if isnumeric(candidate) && ~isempty(candidate) && isfinite(candidate(1))
            value = double(candidate(1));
        end
    end
catch
end
end

function localPlotLimitIndicators(limitDiag, style)
ax = gca;
if ~limitDiag.available
    localPlotNoData(ax, 'No limit indicator data');
    return;
end

hold(ax, 'on');
nFlags = size(limitDiag.flags, 2);
for idx = 1:nFlags
    color = localSignalColor(style, limitDiag.labels{idx}, idx);
    stairs(ax, limitDiag.time, (idx - 1) + 0.8 * limitDiag.flags(:, idx), ...
        'Color', color, ...
        'LineStyle', style.actualLineStyle, ...
        'LineWidth', style.actualLineWidth);
end
ylim(ax, [-0.2, nFlags]);
yticks(ax, (0:nFlags - 1) + 0.4);
yticklabels(ax, limitDiag.labels);
ylabel(ax, 'active');
hold(ax, 'off');
end

function derivative = localTimeDerivative(t, data)
t = t(:);
data = localEnsureMatrix(data);
derivative = zeros(size(data));
if numel(t) < 2
    return;
end

for idx = 1:size(data, 2)
    derivative(:, idx) = gradient(data(:, idx), t);
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

function color = localSignalColor(style, signalName, fallbackIdx)
key = localSignalKey(signalName);

if contains(key, 'vinf') || strcmp(key, 'v') || strcmp(key, 'u') || ...
        contains(key, 'ax') || contains(key, 'front') || ...
        strcmp(key, 'fr') || strcmp(key, 'fl') || contains(key, 'deltaf') || ...
        strcmp(key, 'df') || strcmp(key, 'lw') || strcmp(key, 'rw')
    color = style.colorOrder(1, :);
elseif contains(key, 'alpha') || strcmp(key, 'w') || contains(key, 'az') || ...
        contains(key, 'rear') || strcmp(key, 'rr') || strcmp(key, 'rl') || ...
        contains(key, 'deltae') || strcmp(key, 'de') || strcmp(key, 'lt') || strcmp(key, 'rt')
    color = style.colorOrder(2, :);
elseif contains(key, 'qdot') || strcmp(key, 'q') || contains(key, 'theta')
    color = style.colorOrder(3, :);
elseif contains(key, 'beta') || strcmp(key, 'vbody') || strcmp(key, 'v_b')
    color = style.colorOrder(4, :);
else
    color = style.colorOrder(mod(fallbackIdx - 1, size(style.colorOrder, 1)) + 1, :);
end
end

function key = localSignalKey(signalName)
key = lower(char(string(signalName)));
key = strrep(key, '\delta', 'delta');
key = strrep(key, '\theta', 'theta');
key = strrep(key, '\alpha', 'alpha');
key = strrep(key, '\beta', 'beta');
key = strrep(key, '\infty', 'inf');
key = regexprep(key, '[^a-z0-9]+', '');
end

function entries = localLegendEntries(series, overlayCommand)
entries = {};
for idx = 1:numel(series.actual_labels)
    entries{end + 1} = sprintf('%s act', series.actual_labels{idx}); %#ok<AGROW>
    if overlayCommand && ~isempty(series.cmd) && size(series.cmd, 2) >= idx
        entries{end + 1} = sprintf('%s sched', series.actual_labels{idx}); %#ok<AGROW>
    end
end
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
xlabel(ax, 'Time [s]');
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

error('plot_report_responses:MissingSignal', ...
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
    error('plot_report_responses:UnsupportedLoggedType', ...
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
    error('plot_report_responses:MissingField', 'Signal %s was not found.', varName);
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

function t = localPickTimeBase(varargin)
for idx = 1:nargin
    if ~isempty(varargin{idx})
        t = varargin{idx}(:);
        return;
    end
end
error('plot_report_responses:MissingTimeBase', 'No valid time base found.');
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

function column = localPickColumn(dataIn, preferredIdx)
if size(dataIn, 2) >= preferredIdx
    column = dataIn(:, preferredIdx);
else
    column = dataIn(:, 1);
end
end

function labels = localMakeDefaultLabels(count, preferred)
labels = preferred(1:min(count, numel(preferred)));
if numel(labels) < count
    for idx = (numel(labels) + 1):count
        labels{idx} = sprintf('ch_%d', idx);
    end
end
end

function pathTable = localGetOptionalTransitionPathTable()
pathTable = table();

candidateNames = {'selectedTransitionPathTable', 'indiTransitionPathTable'};
candidate = table();
for idx = 1:numel(candidateNames)
    existsInBase = evalin('base', sprintf('exist(''%s'', ''var'')', candidateNames{idx}));
    if existsInBase
        candidate = evalin('base', candidateNames{idx});
        if istable(candidate) && ~isempty(candidate)
            break;
        end
    end
end
if ~istable(candidate) || isempty(candidate)
    existsInBase = evalin('base', 'exist(''controllerData'', ''var'')');
    if existsInBase
        controllerData = evalin('base', 'controllerData');
        if isstruct(controllerData) && isfield(controllerData, 'schedule_table') && ...
                istable(controllerData.schedule_table)
            candidate = controllerData.schedule_table;
        end
    end
end
if ~istable(candidate) || isempty(candidate)
    return;
end

candidate = localNormalizeTransitionPathColumns(candidate);
if isempty(candidate)
    return;
end

preferredVars = { ...
    'path_index', 'path_progress', ...
    'guide_vinf_mps', 'guide_tilt_deg', ...
    'selected_vinf_mps', 'selected_tilt_deg', ...
    'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_e_deg', 'theta_deg', ...
    'classification', 'name'};
keepMask = ismember(preferredVars, candidate.Properties.VariableNames);
if any(keepMask)
    pathTable = candidate(:, preferredVars(keepMask));
else
    pathTable = candidate;
end
end

function pathTable = localNormalizeTransitionPathColumns(pathTable)
if ~ismember('selected_vinf_mps', pathTable.Properties.VariableNames) && ...
        ismember('vinf_mps', pathTable.Properties.VariableNames)
    pathTable.selected_vinf_mps = pathTable.vinf_mps;
end
if ~ismember('selected_tilt_deg', pathTable.Properties.VariableNames) && ...
        ismember('tilt_deg', pathTable.Properties.VariableNames)
    pathTable.selected_tilt_deg = pathTable.tilt_deg;
end
if ~ismember('path_index', pathTable.Properties.VariableNames)
    pathTable.path_index = (1:height(pathTable)).';
end
end

function controllerData = localGetOptionalControllerData()
controllerData = struct();

existsInBase = evalin('base', 'exist(''controllerData'', ''var'')');
if ~existsInBase
    return;
end

candidate = evalin('base', 'controllerData');
if ~isstruct(candidate) || isempty(candidate)
    return;
end
controllerData = candidate;
end

function markers = localBuildTransitionPathMarkers(pathTable, cmdSource)
markers = struct();
if isempty(pathTable) || ~istable(pathTable)
    return;
end
if localUseGatedPathController()
    return;
end
requiredVars = {'path_index', 'selected_vinf_mps', 'selected_tilt_deg', ...
    'front_collective_rpm', 'rear_collective_rpm', 'delta_f_deg', 'delta_e_deg'};
if ~all(ismember(requiredVars, pathTable.Properties.VariableNames))
    return;
end
if isempty(cmdSource) || ~isstruct(cmdSource) || ...
        ~isfield(cmdSource, 'airData_cmd') || ~isfield(cmdSource, 'tilt_cmd')
    return;
end

try
    tAir = localCommandTime(cmdSource.airData_cmd);
    vinfCmd = localCommandData(cmdSource.airData_cmd, 1);
    tTilt = localCommandTime(cmdSource.tilt_cmd);
    tiltCmd = localCommandData(cmdSource.tilt_cmd, 1);
catch
    return;
end

tCommon = tAir(:);
vinfCommon = vinfCmd(:, 1);
tiltCommon = localResampleToTime(tTilt, tiltCmd(:, 1), tCommon);

nPts = height(pathTable);
markerTime = zeros(nPts, 1);
lastIdx = 1;

vSpan = max(max(vinfCommon) - min(vinfCommon), 1.0);
tSpan = max(max(tiltCommon) - min(tiltCommon), 1.0);

for i = 1:nPts
    targetV = pathTable.selected_vinf_mps(i);
    targetTilt = pathTable.selected_tilt_deg(i);
    searchIdx = lastIdx:numel(tCommon);
    score = ((vinfCommon(searchIdx) - targetV) ./ vSpan).^2 + ...
        ((tiltCommon(searchIdx) - targetTilt) ./ tSpan).^2;
    [~, localMinIdx] = min(score);
    chosenIdx = searchIdx(localMinIdx);
    markerTime(i) = tCommon(chosenIdx);
    lastIdx = chosenIdx;
end

markers.path_index = pathTable.path_index;
markers.time = markerTime;
markers.airspeed = pathTable.selected_vinf_mps;
markers.tilt = [pathTable.selected_tilt_deg, pathTable.selected_tilt_deg];
markers.rotor = [pathTable.front_collective_rpm, pathTable.rear_collective_rpm];
markers.surfaces = [pathTable.delta_f_deg, pathTable.delta_f_deg, ...
    pathTable.delta_e_deg, pathTable.delta_e_deg];
end

function switchEvents = localBuildTransitionSwitchEvents(simSource, pathTable)
switchEvents = struct('times', [], 'labels', {{}}, ...
    'airspeed', [], 'tilt', [], 'rotor', [], 'surfaces', []);
if ~localUseGatedPathController()
    return;
end

try
    [tCmd, yCmd, ~] = localGetFirstAvailable(simSource, {'tilt_angles_cmd'});
    tiltCmd = localPadColumns(yCmd, 2);
catch
    actuatorState = localBuildActuatorStateSeries(simSource);
    if ~actuatorState.available
        return;
    end
    tCmd = actuatorState.time;
    tiltCmd = actuatorState.tilt_deg;
end

if isempty(tiltCmd) || numel(tCmd) < 3
    return;
end

tiltLead = tiltCmd(:, 1);
dt = diff(tCmd(:));
if any(dt <= 0)
    return;
end
dTiltDt = abs(diff(tiltLead) ./ dt);
activeMask = dTiltDt >= 0.15;
if ~any(activeMask)
    return;
end

startMask = activeMask & [true; ~activeMask(1:end-1)];
switchTimes = tCmd(startMask);

labels = cell(numel(switchTimes), 1);
for i = 1:numel(switchTimes)
    labels{i} = sprintf('to %d', i + 1);
end
if ~isempty(pathTable) && istable(pathTable) && ismember('path_index', pathTable.Properties.VariableNames)
    maxLabels = min(numel(labels), height(pathTable) - 1);
    for i = 1:maxLabels
        labels{i} = sprintf('to %d', pathTable.path_index(i + 1));
    end
end

switchEvents.times = switchTimes(:);
switchEvents.labels = labels(:);
if ~isempty(pathTable) && istable(pathTable)
    targetIdx = 2:min(height(pathTable), numel(switchTimes) + 1);
    if ~isempty(targetIdx)
        switchEvents.airspeed = pathTable.selected_vinf_mps(targetIdx);
        switchEvents.tilt = [pathTable.selected_tilt_deg(targetIdx), pathTable.selected_tilt_deg(targetIdx)];
        if all(ismember({'front_collective_rpm', 'rear_collective_rpm'}, pathTable.Properties.VariableNames))
            switchEvents.rotor = [ ...
                pathTable.front_collective_rpm(targetIdx), ...
                pathTable.rear_collective_rpm(targetIdx)];
        end
        if all(ismember({'delta_f_deg', 'delta_e_deg'}, pathTable.Properties.VariableNames))
            switchEvents.surfaces = [ ...
                pathTable.delta_f_deg(targetIdx), pathTable.delta_f_deg(targetIdx), ...
                pathTable.delta_e_deg(targetIdx), pathTable.delta_e_deg(targetIdx)];
        end
    end
end
end

function tf = localUseGatedPathController()
tf = false;
existsInBase = evalin('base', 'exist(''controllerData'', ''var'')');
if ~existsInBase
    return;
end
controllerData = evalin('base', 'controllerData');
if ~isstruct(controllerData) || ~isfield(controllerData, 'controller_id')
    return;
end
tf = any(controllerData.controller_id == [5 6]);
end

function scheduled = localBuildInternalGatedReferenceData(simSource, controllerData, switchEvents)
scheduled = struct( ...
    'available', false, ...
    'airdata', localEmptySeries(), ...
    'euler_deg', localEmptySeries(), ...
    'body_velocity', localEmptySeries(), ...
    'body_rates_deg_s', localEmptySeries(), ...
    'tilt_deg', localEmptySeries(), ...
    'rotor_rpm', localEmptySeries(), ...
    'surface_deg', localEmptySeries(), ...
    'accel_cmd', localEmptySeries(), ...
    'angular_accel_cmd', localEmptySeries(), ...
    'progress', localEmptySeries());

if ~isstruct(controllerData) || isempty(controllerData)
    return;
end
if ~isfield(controllerData, 'controller_id') || ~any(controllerData.controller_id == [5 6])
    return;
end
if ~isfield(controllerData, 'controller_state_ref') || ~isfield(controllerData, 'controller_trim_cmd')
    return;
end

stateSchedule = controllerData.controller_state_ref;
trimSchedule = controllerData.controller_trim_cmd;
nPts = localInferControllerScheduleCount(stateSchedule, trimSchedule);
if nPts < 1
    return;
end
stateSchedule = stateSchedule(:, 1:nPts);
trimSchedule = trimSchedule(:, 1:nPts);

try
    t = localGetTout(simSource);
catch
    return;
end
if isempty(t)
    return;
end

gatingOpts = struct();
if isfield(controllerData, 'gating_opts') && isstruct(controllerData.gating_opts)
    gatingOpts = controllerData.gating_opts;
end
segmentRampTime = localGetStructField(gatingOpts, 'segment_ramp_time_s', 4.0);
progressSchedule = localGetScheduleVector(stateSchedule, 12, nPts);
if all(abs(progressSchedule) <= 1e-12)
    progressSchedule = linspace(0.0, 1.0, nPts);
end

[idxLoHistory, idxHiHistory, lambdaHistory, transitionStarts] = ...
    localInferInternalBlendHistory(simSource, t, stateSchedule, nPts, switchEvents, segmentRampTime);

phi = zeros(numel(t), 1);
theta = zeros(numel(t), 1);
psi = zeros(numel(t), 1);
uBody = zeros(numel(t), 1);
vBody = zeros(numel(t), 1);
wBody = zeros(numel(t), 1);
pRate = zeros(numel(t), 1);
qRate = zeros(numel(t), 1);
rRate = zeros(numel(t), 1);
tilt = zeros(numel(t), 1);
front = zeros(numel(t), 1);
rear = zeros(numel(t), 1);
surf = zeros(numel(t), 4);
accelCmd = zeros(numel(t), 2);
qdotCmd = zeros(numel(t), 1);
progress = zeros(numel(t), 1);
actualState = localBuildActualStateHistoryForIndi(simSource, t);

for k = 1:numel(t)
    idxLo = idxLoHistory(k);
    idxHi = idxHiHistory(k);
    lambda = lambdaHistory(k);
    xRef = localBlendStateColumn(stateSchedule, idxLo, idxHi, lambda);
    trimRef = localBlendTrimColumn(trimSchedule, idxLo, idxHi, lambda);
    outer = localReconstructOuterSettings(stateSchedule, idxLo, idxHi, lambda);
    nuCmd = localReconstructIndiNuCommand(xRef, actualState(k, :).', outer);

    phi(k) = rad2deg(xRef(1));
    theta(k) = rad2deg(xRef(2));
    psi(k) = rad2deg(xRef(3));
    uBody(k) = xRef(4);
    vBody(k) = xRef(5);
    wBody(k) = xRef(6);
    pRate(k) = rad2deg(xRef(7));
    qRate(k) = rad2deg(xRef(8));
    rRate(k) = rad2deg(xRef(9));
    tilt(k) = localBlendRow(stateSchedule, 10, idxLo, idxHi, lambda);
    front(k) = trimRef(1);
    rear(k) = trimRef(2);
    surf(k, :) = rad2deg(localMixedToLocalRow(trimRef(3), trimRef(4), trimRef(5), trimRef(6)));
    accelCmd(k, :) = nuCmd(1:2).';
    qdotCmd(k) = rad2deg(nuCmd(3));
    progress(k) = localBlendScalar(progressSchedule, idxLo, idxHi, lambda);
end

vinf = sqrt(uBody.^2 + vBody.^2 + wBody.^2);
alpha = rad2deg(atan2(wBody, max(uBody, 1e-6)));
beta = zeros(size(vinf));
valid = vinf > 1e-6;
beta(valid) = rad2deg(asin(max(min(vBody(valid) ./ vinf(valid), 1.0), -1.0)));

scheduled.airdata = struct('time', t, 'data', [vinf alpha beta], 'labels', {{'V_\infty', '\alpha', '\beta'}});
scheduled.euler_deg = struct('time', t, 'data', [phi theta psi], 'labels', {{'\phi_{int}', '\theta_{int}', '\psi_{int}'}});
scheduled.body_velocity = struct('time', t, 'data', [uBody wBody], 'labels', {{'u_{int}', 'w_{int}'}});
scheduled.body_rates_deg_s = struct('time', t, 'data', [pRate qRate rRate], 'labels', {{'p_{int}', 'q_{int}', 'r_{int}'}});
scheduled.tilt_deg = struct('time', t, 'data', [tilt tilt], 'labels', {{'FR int', 'FL int'}});
scheduled.rotor_rpm = struct('time', t, 'data', [front front rear rear], 'labels', {{'FR int', 'FL int', 'RR int', 'RL int'}});
scheduled.surface_deg = struct('time', t, 'data', surf, 'labels', {{'LW int', 'RW int', 'LT int', 'RT int'}});
scheduled.accel_cmd = struct('time', t, 'data', accelCmd, 'labels', {{'a_x cmd', 'a_z cmd'}});
scheduled.angular_accel_cmd = struct('time', t, 'data', qdotCmd, 'labels', {{'qdot cmd'}});
scheduled.progress = struct('time', t, 'data', progress, 'labels', {{'s_{int}'}});
scheduled.transition_starts = transitionStarts(:);
scheduled.available = true;
end

function values = localGetScheduleVector(stateSchedule, rowIdx, nPts)
values = zeros(1, nPts);
if size(stateSchedule, 1) >= rowIdx
    values = stateSchedule(rowIdx, 1:nPts);
end
end

function nPts = localInferControllerScheduleCount(stateSchedule, trimSchedule)
nPts = min(size(stateSchedule, 2), size(trimSchedule, 2));
if nPts <= 1
    return;
end

if size(stateSchedule, 1) < 12
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

function [idxLoHistory, idxHiHistory, lambdaHistory, transitionStarts] = ...
    localInferInternalBlendHistory(simSource, t, stateSchedule, nPts, switchEvents, segmentRampTime)
idxLoHistory = ones(numel(t), 1);
idxHiHistory = ones(numel(t), 1);
lambdaHistory = zeros(numel(t), 1);
transitionStarts = [];

if nPts <= 1
    return;
end

if isstruct(switchEvents) && isfield(switchEvents, 'times') && ~isempty(switchEvents.times)
    transitionStarts = switchEvents.times(:);
    transitionStarts = transitionStarts(transitionStarts >= t(1) & transitionStarts <= t(end));
    transitionStarts = transitionStarts(1:min(numel(transitionStarts), nPts - 1));
end

if ~isempty(transitionStarts)
    for k = 1:numel(t)
        [idxLoHistory(k), idxHiHistory(k), lambdaHistory(k)] = ...
            localInternalScheduleBlend(t(k), transitionStarts, segmentRampTime, nPts);
    end
    return;
end

% Fallback for cases where the ramp is already underway at t=0 or the
% derivative-based switch detector cannot identify clean transition starts.
try
    [tTilt, yTilt, ~] = localGetFirstAvailable(simSource, {'tilt_angles_cmd'});
    tiltActual = localResampleToTime(tTilt, localPadColumns(yTilt, 2), t);
catch
    actuatorState = localBuildActuatorStateSeries(simSource);
    if ~actuatorState.available
        return;
    end
    tiltActual = localResampleToTime(actuatorState.time, actuatorState.tilt_deg, t);
end

tiltSchedule = localGetScheduleVector(stateSchedule, 10, nPts);
if max(tiltSchedule) - min(tiltSchedule) <= 1e-9
    return;
end

tiltLead = tiltActual(:, 1);
for k = 1:numel(t)
    [idxLoHistory(k), idxHiHistory(k), lambdaHistory(k)] = ...
        localProjectTiltToInternalBlend(tiltLead(k), tiltSchedule, nPts);
end
end

function [idxLo, idxHi, lambda] = localProjectTiltToInternalBlend(tiltValue, tiltSchedule, nPts)
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

function actualState = localBuildActualStateHistoryForIndi(simSource, t)
actualState = zeros(numel(t), 9);

try
    [tEul, yEul, ~] = localGetFirstAvailable(simSource, { ...
        'eul_meas_log', 'attitude_meas', 'eul_truth'});
    actualState(:, 1:3) = localResampleToTime(tEul, localPadColumns(yEul, 3), t);
catch
end

try
    [tAir, yAir, ~] = localGetFirstAvailable(simSource, { ...
        'airDataMeas_log', 'airdata_meas', 'airData_meas'});
    actualState(:, 4:6) = localAirdataMatrixToBodyVelocity( ...
        localResampleToTime(tAir, localPadColumns(yAir, 3), t));
catch
    try
        [tVel, yVel, ~] = localGetFirstAvailable(simSource, { ...
            'V_B_truth', 'V_BA_truth', 'V_E_truth'});
        actualState(:, 4:6) = localResampleToTime(tVel, localPadColumns(yVel, 3), t);
    catch
        try
            [tAir, yAir, ~] = localGetFirstAvailable(simSource, {'vinf_truth'});
            actualState(:, 4:6) = localAirdataMatrixToBodyVelocity( ...
                localResampleToTime(tAir, localPadColumns(yAir, 3), t));
        catch
        end
    end
end

try
    [tRates, yRates, ~] = localGetFirstAvailable(simSource, { ...
        'omega_Meas_log', 'gyro_meas', 'omega_truth'});
    actualState(:, 7:9) = localResampleToTime(tRates, localPadColumns(yRates, 3), t);
catch
end
end

function velBody = localAirdataMatrixToBodyVelocity(airData)
airData = localPadColumns(airData, 3);
vinf = airData(:, 1);
alpha = airData(:, 2);
beta = airData(:, 3);
velBody = [ ...
    vinf .* cos(alpha) .* cos(beta), ...
    vinf .* sin(beta), ...
    vinf .* sin(alpha) .* cos(beta)];
end

function outer = localReconstructOuterSettings(stateSchedule, idxLo, idxHi, lambda)
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

function nuCmd = localReconstructIndiNuCommand(xRef, xMeas, outer)
g = 9.81;
thetaRef = xRef(2);
nuCmd = zeros(3, 1);
nuCmd(1) = g * sin(thetaRef) + outer.ku * (xRef(4) - xMeas(4));
nuCmd(2) = -g * cos(thetaRef) + outer.kw * (xRef(6) - xMeas(6));
nuCmd(3) = outer.kq * (xRef(8) - xMeas(8)) + ...
    outer.ktheta * (xRef(2) - xMeas(2));
end

function [idxLo, idxHi, lambda] = localInternalScheduleBlend(tNow, transitionStarts, segmentRampTime, nPts)
if isempty(transitionStarts)
    idxLo = 1;
    idxHi = 1;
    lambda = 0.0;
    return;
end

for i = 1:numel(transitionStarts)
    t0 = transitionStarts(i);
    t1 = t0 + segmentRampTime;
    if tNow < t0
        idxLo = i;
        idxHi = i;
        lambda = 0.0;
        return;
    end
    if tNow >= t0 && tNow < t1
        idxLo = i;
        idxHi = min(i + 1, nPts);
        lambda = min(max((tNow - t0) / max(segmentRampTime, 1e-6), 0.0), 1.0);
        return;
    end
end

idxLo = min(numel(transitionStarts) + 1, nPts);
idxHi = idxLo;
lambda = 0.0;
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

function value = localBlendRow(stateSchedule, rowIdx, idxLo, idxHi, lambda)
if size(stateSchedule, 1) < rowIdx
    value = 0.0;
    return;
end
value = (1.0 - lambda) * stateSchedule(rowIdx, idxLo) + lambda * stateSchedule(rowIdx, idxHi);
end

function value = localBlendScalar(vec, idxLo, idxHi, lambda)
value = (1.0 - lambda) * vec(idxLo) + lambda * vec(idxHi);
end

function value = localGetStructField(s, fieldName, fallback)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = fallback;
end
end

function surfaceLocal = localMixedToLocalRow(deltaF, deltaA, deltaE, deltaR)
surfaceLocal = [deltaF + deltaA, deltaF - deltaA, deltaE - deltaR, deltaE + deltaR];
end

function localOverlayTransitionMarkers(ax, markers, seriesName, style)
if isempty(markers) || ~isstruct(markers) || ~isfield(markers, 'time') || isempty(markers.time)
    return;
end
if ~isfield(markers, seriesName)
    return;
end

yData = markers.(seriesName);
if isempty(yData)
    return;
end
if isvector(yData)
    yData = yData(:);
end

hold(ax, 'on');
colors = style.colorOrder;
nChan = size(yData, 2);
for idx = 1:nChan
    color = colors(mod(idx - 1, size(colors, 1)) + 1, :);
    plot(ax, markers.time, yData(:, idx), 'o', ...
        'Color', color, ...
        'MarkerFaceColor', 'w', ...
        'MarkerSize', 5, ...
        'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
end

for rowIdx = 1:numel(markers.time)
    text(ax, markers.time(rowIdx), yData(rowIdx, 1), sprintf('%d', markers.path_index(rowIdx)), ...
        'Color', [0.15 0.15 0.15], ...
        'FontSize', 8, ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'left', ...
        'Clipping', 'on', ...
        'HandleVisibility', 'off');
end
hold(ax, 'off');
end

function localOverlaySwitchEvents(ax, switchEvents)
if isempty(switchEvents) || ~isstruct(switchEvents) || ...
        ~isfield(switchEvents, 'times') || isempty(switchEvents.times)
    return;
end

yl = ylim(ax);
for i = 1:numel(switchEvents.times)
    t = switchEvents.times(i);
    line(ax, [t t], yl, ...
        'Color', [0.15 0.15 0.15], ...
        'LineStyle', '--', ...
        'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
    if isfield(switchEvents, 'labels') && numel(switchEvents.labels) >= i
        text(ax, t, yl(2), [' ' char(string(switchEvents.labels{i}))], ...
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

function localOverlaySwitchTargets(ax, switchEvents, seriesName, style)
if isempty(switchEvents) || ~isstruct(switchEvents) || ...
        ~isfield(switchEvents, 'times') || isempty(switchEvents.times) || ...
        ~isfield(switchEvents, seriesName)
    return;
end

yData = switchEvents.(seriesName);
if isempty(yData)
    return;
end
if isvector(yData)
    yData = yData(:);
end

nPts = min(numel(switchEvents.times), size(yData, 1));
hold(ax, 'on');
xLimits = xlim(ax);
xEnd = xLimits(2);
colors = style.colorOrder;
for idx = 1:size(yData, 2)
    color = colors(mod(idx - 1, size(colors, 1)) + 1, :);
    for rowIdx = 1:nPts
        t0 = switchEvents.times(rowIdx);
        if rowIdx < numel(switchEvents.times)
            t1 = switchEvents.times(rowIdx + 1);
        else
            t1 = xEnd;
        end
        yTarget = yData(rowIdx, idx);
        line(ax, [t0 t1], [yTarget yTarget], ...
            'Color', color, ...
            'LineStyle', '-.', ...
            'LineWidth', 1.1, ...
            'HandleVisibility', 'off');
    end
end
hold(ax, 'off');
end

function figPath = localCreateTransitionPathTableFigure(pathTable, titlePrefix)
figPath = figure( ...
    'Name', [titlePrefix ' - Transition Path'], ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [200 160 1220 420]);

    gridData = localTableToCell(pathTable);
    uitable(figPath, ...
        'Data', gridData, ...
        'ColumnName', pathTable.Properties.VariableNames, ...
        'RowName', {}, ...
        'Units', 'normalized', ...
        'Position', [0.015 0.05 0.97 0.88]);

    annotation(figPath, 'textbox', [0.015 0.93 0.97 0.06], ...
        'String', [titlePrefix ' - Transition Path Table'], ...
        'EdgeColor', 'none', ...
        'Color', 'k', ...
        'FontWeight', 'bold', ...
        'FontSize', 12, ...
        'Interpreter', 'none');
end

function cellData = localTableToCell(tbl)
cellData = table2cell(tbl);
for rowIdx = 1:size(cellData, 1)
    for colIdx = 1:size(cellData, 2)
        value = cellData{rowIdx, colIdx};
        if isstring(value) || ischar(value)
            cellData{rowIdx, colIdx} = char(string(value));
        elseif isnumeric(value) && isscalar(value) && isfinite(value)
            cellData{rowIdx, colIdx} = value;
        elseif isnumeric(value) && isscalar(value) && ~isfinite(value)
            cellData{rowIdx, colIdx} = '';
        elseif islogical(value) && isscalar(value)
            cellData{rowIdx, colIdx} = logical(value);
        else
            cellData{rowIdx, colIdx} = char(string(value));
        end
    end
end
end

function y = localConvertIfNeeded(yIn, sourceName, targetUnits)
y = yIn;

if strcmp(targetUnits, 'deg') && ~isempty(sourceName)
    isRadians = any(strcmp(sourceName, {'viz_surface_states', 'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'}));
    if isRadians
        y = rad2deg(yIn);
    end
end
end

function t = localCommandTime(cmdMatrix)
if ~isnumeric(cmdMatrix) || size(cmdMatrix, 2) < 2
    error('plot_report_responses:BadCommandMatrix', 'Command matrix must have at least 2 columns.');
end
t = cmdMatrix(:, 1);
end

function Y = localCommandData(cmdMatrix, nChannels)
if ~isnumeric(cmdMatrix) || size(cmdMatrix, 2) < (nChannels + 1)
    error('plot_report_responses:BadCommandMatrix', ...
        'Command matrix does not contain %d channels.', nChannels);
end
Y = cmdMatrix(:, 2:(nChannels + 1));
end
