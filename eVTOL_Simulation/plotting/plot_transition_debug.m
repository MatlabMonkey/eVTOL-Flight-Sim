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
%   Figure 1: position, velocity, Euler angles, angular rates
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
tiledlayout(figStates, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

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

titleHandle = sgtitle(figStates, [titlePrefix ' - States'], 'FontWeight', 'bold');
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

titleHandle = sgtitle(figAct, [titlePrefix ' - Airspeed And Actuators'], 'FontWeight', 'bold');
titleHandle.Color = 'k';

reportData.figures = struct('states', figStates, 'actuators', figAct);

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
    titleHandle = sgtitle(figSched, [titlePrefix ' - Scheduled Targets' titleSuffix], 'FontWeight', 'bold');
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
        filenameStem = fullfile(localDefaultReportDir(), localDefaultReportBaseName(simSource, titlePrefix));
    else
        filenameStem = '';
    end
    return;
end

filenameStem = char(string(filenameStem));
trimmedStem = strtrim(filenameStem);
if any(strcmpi(trimmedStem, {'save', 'report', 'default'}))
    filenameStem = fullfile(localDefaultReportDir(), localDefaultReportBaseName(simSource, titlePrefix));
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
reportDir = fullfile(fileparts(mfilename('fullpath')), 'report_plots_final');
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

baseName = '';
for idx = 1:numel(candidateNames)
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
end

function tilt = localBuildTiltSeries(simSource, cmdSource)
[tActual, yActual, sourceActual] = localGetFirstAvailable(simSource, {'viz_tilt_states', 'tilt_angles_cmd'});
tilt = struct();
tilt.time = tActual;
tilt.actual = localConvertIfNeeded(yActual, sourceActual, 'deg');
tilt.actual_labels = localMakeDefaultLabels(size(tilt.actual, 2), {'FR', 'FL'});
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
    tilt.cmd = localResampleToTime(tCmd, yCmd, tActual);
    tilt.cmd_labels = tilt.actual_labels;
catch
end
end

function rotor = localBuildRotorSeries(simSource, cmdSource)
rotor = struct();
rotor.title = 'Rotor Speed States';
rotor.units = 'rpm';

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

rotor.data = rotor.actual;
rotor.labels = rotor.actual_labels;
rotor.cmd = [];
rotor.cmd_labels = {};

if isempty(cmdSource)
    return;
end

try
    if isfield(cmdSource, 'front_cmd') && isfield(cmdSource, 'rear_cmd') && size(rotor.actual, 2) == 2
        tFront = localCommandTime(cmdSource.front_cmd);
        tRear = localCommandTime(cmdSource.rear_cmd);
        yFront = localCommandData(cmdSource.front_cmd, 1);
        yRear = localCommandData(cmdSource.rear_cmd, 1);
        rotor.cmd = [ ...
            localResampleToTime(tFront, yFront, rotor.time), ...
            localResampleToTime(tRear, yRear, rotor.time)];
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

try
    [tSurf, ySurf, sourceSurf] = localGetFirstAvailable(simSource, {'deltaLW_cmd'});
    surfaceNames = {'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'};
    Y = ySurf(:, 1);
    for idx = 1:numel(surfaceNames)
        [t_i, y_i, ~] = localGetFirstAvailable(simSource, {surfaceNames{idx}});
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

surfaces.data = surfaces.actual;
surfaces.labels = surfaces.actual_labels;
surfaces.cmd = [];
surfaces.cmd_labels = {};

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

function scheduled = localBuildScheduledReferenceData(cmdSource)
scheduled = struct( ...
    'available', false, ...
    'airdata', localEmptySeries(), ...
    'euler_deg', localEmptySeries(), ...
    'tilt_deg', localEmptySeries(), ...
    'rotor_rpm', localEmptySeries(), ...
    'surface_deg', localEmptySeries(), ...
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
style.commandLineStyle = ':';
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

try
    raw = simSource.get(varName);
catch
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
        labels{idx} = sprintf('ch_%d', idx); %#ok<AGROW>
    end
end
end

function pathTable = localGetOptionalTransitionPathTable()
pathTable = table();

existsInBase = evalin('base', 'exist(''selectedTransitionPathTable'', ''var'')');
if ~existsInBase
    return;
end

candidate = evalin('base', 'selectedTransitionPathTable');
if ~istable(candidate) || isempty(candidate)
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
catch
    return;
end

tiltCmd = localPadColumns(yCmd, 2);
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

startIdx = find(activeMask & [true; ~activeMask(1:end-1)]);
switchTimes = tCmd(startIdx);

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
tf = isequal(controllerData.controller_id, 5);
end

function scheduled = localBuildInternalGatedReferenceData(simSource, controllerData, switchEvents)
scheduled = struct( ...
    'available', false, ...
    'airdata', localEmptySeries(), ...
    'euler_deg', localEmptySeries(), ...
    'tilt_deg', localEmptySeries(), ...
    'rotor_rpm', localEmptySeries(), ...
    'surface_deg', localEmptySeries(), ...
    'progress', localEmptySeries());

if ~isstruct(controllerData) || isempty(controllerData)
    return;
end
if ~isfield(controllerData, 'controller_id') || controllerData.controller_id ~= 5
    return;
end
if ~isfield(controllerData, 'controller_state_ref') || ~isfield(controllerData, 'controller_trim_cmd')
    return;
end

stateSchedule = controllerData.controller_state_ref;
trimSchedule = controllerData.controller_trim_cmd;
nPts = min(size(stateSchedule, 2), size(trimSchedule, 2));
if nPts < 1
    return;
end

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

transitionStarts = [];
if isstruct(switchEvents) && isfield(switchEvents, 'times') && ~isempty(switchEvents.times)
    transitionStarts = switchEvents.times(:);
end
transitionStarts = transitionStarts(transitionStarts >= t(1) & transitionStarts <= t(end));
maxTransitions = min(numel(transitionStarts), nPts - 1);
transitionStarts = transitionStarts(1:maxTransitions);

phi = zeros(numel(t), 1);
theta = zeros(numel(t), 1);
psi = zeros(numel(t), 1);
uBody = zeros(numel(t), 1);
vBody = zeros(numel(t), 1);
wBody = zeros(numel(t), 1);
tilt = zeros(numel(t), 1);
front = zeros(numel(t), 1);
rear = zeros(numel(t), 1);
surf = zeros(numel(t), 4);
progress = zeros(numel(t), 1);

for k = 1:numel(t)
    tk = t(k);
    [idxLo, idxHi, lambda] = localInternalScheduleBlend(tk, transitionStarts, segmentRampTime, nPts);
    xRef = localBlendStateColumn(stateSchedule, idxLo, idxHi, lambda);
    trimRef = localBlendTrimColumn(trimSchedule, idxLo, idxHi, lambda);

    phi(k) = rad2deg(xRef(1));
    theta(k) = rad2deg(xRef(2));
    psi(k) = rad2deg(xRef(3));
    uBody(k) = xRef(4);
    vBody(k) = xRef(5);
    wBody(k) = xRef(6);
    tilt(k) = localBlendRow(stateSchedule, 10, idxLo, idxHi, lambda);
    front(k) = trimRef(1);
    rear(k) = trimRef(2);
    surf(k, :) = rad2deg(localMixedToLocalRow(trimRef(3), trimRef(4), trimRef(5), trimRef(6)));
    progress(k) = localBlendScalar(progressSchedule, idxLo, idxHi, lambda);
end

vinf = sqrt(uBody.^2 + vBody.^2 + wBody.^2);
alpha = rad2deg(atan2(wBody, max(uBody, 1e-6)));
beta = zeros(size(vinf));
valid = vinf > 1e-6;
beta(valid) = rad2deg(asin(max(min(vBody(valid) ./ vinf(valid), 1.0), -1.0)));

scheduled.airdata = struct('time', t, 'data', [vinf alpha beta], 'labels', {{'V_\infty', '\alpha', '\beta'}});
scheduled.euler_deg = struct('time', t, 'data', [phi theta psi], 'labels', {{'\phi_{int}', '\theta_{int}', '\psi_{int}'}});
scheduled.tilt_deg = struct('time', t, 'data', [tilt tilt], 'labels', {{'FR int', 'FL int'}});
scheduled.rotor_rpm = struct('time', t, 'data', [front rear], 'labels', {{'Front int', 'Rear int'}});
scheduled.surface_deg = struct('time', t, 'data', surf, 'labels', {{'LW int', 'RW int', 'LT int', 'RT int'}});
scheduled.progress = struct('time', t, 'data', progress, 'labels', {{'s_{int}'}});
scheduled.available = true;
end

function values = localGetScheduleVector(stateSchedule, rowIdx, nPts)
values = zeros(1, nPts);
if size(stateSchedule, 1) >= rowIdx
    values = stateSchedule(rowIdx, 1:nPts);
end
end

function [idxLo, idxHi, lambda] = localInternalScheduleBlend(tNow, transitionStarts, segmentRampTime, nPts)
idxLo = 1;
idxHi = 1;
lambda = 0.0;

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
