function reportData = plot_report_responses(simSource, cmdSource, filenameStem, titlePrefix)
%PLOT_REPORT_RESPONSES Create concise white-background report figures.
%
% Usage:
%   plot_report_responses
%   plot_report_responses(out)
%   plot_report_responses(out, cmds)
%   plot_report_responses(out, cmds, true)
%   plot_report_responses(out, cmds, 'save', 'Case 1')
%   plot_report_responses(out, cmds, 'workspace_plots/report_case1', 'Case 1')
%
% This helper creates two figures with four subplots each:
%   Figure 1: position, velocity, Euler angles, angular rates
%   Figure 2: airspeed, tilt states, rotor states, surface states
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
        error('plot_report_responses:MissingSimSource', ...
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
title('Airspeed');
ylabel('m/s');
leg = legend({'V_\infty'}, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
localFinishAxes(gca, style);

nexttile;
localPlotActualAndCommand(reportData.tilt, style, false);
title('Tilt States');
ylabel('deg');
localFinishAxes(gca, style);

nexttile;
localPlotActualAndCommand(reportData.rotor, style, true);
title(reportData.rotor.title);
ylabel('RPM');
localFinishAxes(gca, style);

nexttile;
localPlotActualAndCommand(reportData.surfaces, style, true);
title('Control Surface States');
ylabel('deg');
localFinishAxes(gca, style);

titleHandle = sgtitle(figAct, [titlePrefix ' - Airspeed And Actuators'], 'FontWeight', 'bold');
titleHandle.Color = 'k';

reportData.figures = struct('states', figStates, 'actuators', figAct);

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
