function plot_outputs(mode)
%PLOT_OUTPUTS Plot fixed Wrapper outputs from base-workspace `out`.
%
% Usage:
%   plot_outputs
%   plot_outputs('important')
%   plot_outputs('all')
%
% Modes:
%   longitudinal (default)  Core longitudinal truth signals
%   important               Broader truth/visualization set
%   all                     All known top-level Wrapper outputs

if nargin < 1 || isempty(mode)
    mode = 'longitudinal';
end

mode = validatestring(lower(string(mode)), {'longitudinal', 'important', 'all'});

fprintf('\nLooking for Wrapper output variable "out" in the base workspace...\n');

if ~evalin('base', 'exist(''out'', ''var'')')
    warning(['No base-workspace variable named "out" was found. ', ...
        'Run the Wrapper simulation first so it writes its SimulationOutput to "out".']);
    return
end

out = evalin('base', 'out');
if ~isa(out, 'Simulink.SimulationOutput') && ~isstruct(out)
    error(['Expected base-workspace variable "out" to be either a ', ...
        'Simulink.SimulationOutput or a plain struct with matching top-level fields.']);
end

specs = localSignalSpecs();
signalNames = localSelectSignals(mode);

if isa(out, 'Simulink.SimulationOutput')
    outKind = 'SimulationOutput';
else
    outKind = 'struct';
end
fprintf('Plotting output container: out [%s] (%s mode)\n', outKind, mode);

for k = 1:numel(signalNames)
    signalName = signalNames{k};

    if ~isfield(specs, signalName)
        warning('No plotting specification was defined for "%s". Skipping it.', signalName);
        continue
    end

    try
        signalStruct = localGetOutputField(out, signalName);
    catch
        warning('Wrapper output "out" does not contain top-level signal "%s".', signalName);
        continue
    end

    localPlotSignalStruct(signalStruct, specs.(signalName));
end

localPlotAlwaysUsefulDiagnostics(out);

if any(strcmp(char(mode), {'important', 'all'}))
    localPlotCmds(mode);
end

fprintf('Done. Processed Wrapper output target "out" in %s mode.\n', mode);
end

function signalNames = localSelectSignals(mode)
switch char(mode)
    case 'longitudinal'
        signalNames = { ...
            'V_B_truth', ...
            'eul_truth', ...
            'omega_truth', ...
            'vinf_truth', ...
            'alpha_truth', ...
            'specific_force_truth'};
    case 'important'
        signalNames = { ...
            'pos_NED', ...
            'V_B_truth', ...
            'V_E_truth', ...
            'eul_truth', ...
            'omega_truth', ...
            'V_BA_truth', ...
            'vinf_truth', ...
            'alpha_truth', ...
            'beta_truth', ...
            'specific_force_truth', ...
            'tilt_angles_cmd', ...
            'front_collective_rpm_out', ...
            'rear_collective_rpm_out'};
    case 'all'
        signalNames = { ...
            'C_NB_truth', ...
            'V_BA_truth', ...
            'V_B_truth', ...
            'V_E_truth', ...
            'accel_Meas_log', ...
            'airDataMeas_log', ...
            'alpha_truth', ...
            'beta_truth', ...
            'eul_meas_log', ...
            'eul_truth', ...
            'gpsVelMeas_log', ...
            'gps_Pos_Meas_log', ...
            'magHdgMeas_log', ...
            'magMeasBody_log', ...
            'omega_Meas_log', ...
            'omega_truth', ...
            'pos_NED', ...
            'specific_force_truth', ...
            'vinf_truth', ...
            'tilt_angles_cmd', ...
            'front_collective_rpm_out', ...
            'rear_collective_rpm_out', ...
            'deltaLW_cmd', ...
            'deltaRW_cmd', ...
            'deltaLT_cmd', ...
            'deltaRT_cmd'};
    otherwise
        error('Unsupported plotting mode "%s".', mode);
end
end

function specs = localSignalSpecs()
specs = struct();

specs.C_NB_truth = localMakeSpec( ...
    'DCM truth', ...
    {'C11','C12','C13','C21','C22','C23','C31','C32','C33'}, ...
    {'','','','','','','','',''});

specs.V_BA_truth = localMakeSpec( ...
    'Body-relative air velocity truth', ...
    {'u_A','v_A','w_A'}, ...
    {'m/s','m/s','m/s'});

specs.V_B_truth = localMakeSpec( ...
    'Body velocity truth', ...
    {'u','v','w'}, ...
    {'m/s','m/s','m/s'});

specs.V_E_truth = localMakeSpec( ...
    'Earth velocity truth', ...
    {'V_N','V_E','V_D'}, ...
    {'m/s','m/s','m/s'});

specs.accel_Meas_log = localMakeSpec( ...
    'Accelerometer measurement', ...
    {'a_x','a_y','a_z'}, ...
    {'m/s^2','m/s^2','m/s^2'});

specs.airDataMeas_log = localMakeSpec( ...
    'Air-data measurement', ...
    {'V_inf','alpha','beta'}, ...
    {'m/s','rad','rad'});

specs.alpha_truth = localMakeSpec( ...
    'Angle of attack truth', ...
    {'alpha'}, ...
    {'rad'});

specs.beta_truth = localMakeSpec( ...
    'Sideslip truth', ...
    {'beta'}, ...
    {'rad'});

specs.eul_meas_log = localMakeSpec( ...
    'Euler attitude measurement', ...
    {'phi','theta','psi'}, ...
    {'rad','rad','rad'});

specs.eul_truth = localMakeSpec( ...
    'Euler attitude truth', ...
    {'phi','theta','psi'}, ...
    {'rad','rad','rad'});

specs.gpsVelMeas_log = localMakeSpec( ...
    'GPS velocity measurement', ...
    {'gps_speed'}, ...
    {'m/s'});

specs.gps_Pos_Meas_log = localMakeSpec( ...
    'GPS position measurement', ...
    {'N','E','D'}, ...
    {'m','m','m'});

specs.magHdgMeas_log = localMakeSpec( ...
    'Magnetic heading measurement', ...
    {'psi_mag'}, ...
    {'rad'});

specs.magMeasBody_log = localMakeSpec( ...
    'Body magnetometer measurement', ...
    {'mag_x','mag_y','mag_z'}, ...
    {'','',''});

specs.omega_Meas_log = localMakeSpec( ...
    'Body-rate measurement', ...
    {'P','Q','R'}, ...
    {'rad/s','rad/s','rad/s'});

specs.omega_truth = localMakeSpec( ...
    'Body-rate truth', ...
    {'P','Q','R'}, ...
    {'rad/s','rad/s','rad/s'});

specs.pos_NED = localMakeSpec( ...
    'Position NED truth', ...
    {'N','E','D'}, ...
    {'m','m','m'});

specs.specific_force_truth = localMakeSpec( ...
    'Specific force truth', ...
    {'f_x','f_y','f_z'}, ...
    {'m/s^2','m/s^2','m/s^2'});

specs.vinf_truth = localMakeSpec( ...
    'Airspeed magnitude truth', ...
    {'V_inf'}, ...
    {'m/s'});

specs.tilt_angles_cmd = localMakeSpec( ...
    'Tilt states', ...
    {'front_right','front_left'}, ...
    {'deg','deg'});

specs.front_collective_rpm_out = localMakeSpec( ...
    'Front collective RPM output', ...
    {'front_collective_rpm'}, ...
    {'rpm'});

specs.rear_collective_rpm_out = localMakeSpec( ...
    'Rear collective RPM output', ...
    {'rear_collective_rpm'}, ...
    {'rpm'});

specs.deltaLW_cmd = localMakeSpec( ...
    'Left wing actuator state', ...
    {'deltaLW'}, ...
    {'rad'});

specs.deltaRW_cmd = localMakeSpec( ...
    'Right wing actuator state', ...
    {'deltaRW'}, ...
    {'rad'});

specs.deltaLT_cmd = localMakeSpec( ...
    'Left tail actuator state', ...
    {'deltaLT'}, ...
    {'rad'});

specs.deltaRT_cmd = localMakeSpec( ...
    'Right tail actuator state', ...
    {'deltaRT'}, ...
    {'rad'});
end

function localPlotCmds(mode)
if ~evalin('base', 'exist(''cmds'', ''var'')')
    warning('Base-workspace variable "cmds" was not found. Skipping command plots.');
    return
end

cmds = evalin('base', 'cmds');
if ~isstruct(cmds)
    warning('Base-workspace variable "cmds" is not a struct. Skipping command plots.');
    return
end

commandSpecs = localCommandSpecs();
commandNames = fieldnames(commandSpecs);

if strcmp(char(mode), 'important')
    commandNames = { ...
        'airData_cmd', ...
        'eul_cmd', ...
        'omega_cmd', ...
        'motor_cmd', ...
        'tilt_cmd'};
elseif strcmp(char(mode), 'all')
    commandNames = setdiff(commandNames, {'front_cmd', 'rear_cmd'}, 'stable');
end

for k = 1:numel(commandNames)
    cmdName = commandNames{k};
    if ~isfield(cmds, cmdName)
        warning('Command struct "cmds" does not contain "%s".', cmdName);
        continue
    end

    cmdMatrix = cmds.(cmdName);
    localPlotCommandMatrix(cmdMatrix, commandSpecs.(cmdName));
end
end

function localPlotAlwaysUsefulDiagnostics(out)
localPlotCollectiveCommands(out);
localPlotSurfaceStates(out);
end

function localPlotCollectiveCommands(out)
[tFront, yFront, tRear, yRear] = localGetCollectiveSeries(out);
if isempty(tFront) || isempty(tRear)
    return
end

figure('Name', 'Collective commands', 'NumberTitle', 'off');
plot(tFront, yFront, 'LineWidth', 1.4);
hold on;
plot(tRear, yRear, 'LineWidth', 1.4);
grid on;
localApplyYAxis([yFront(:); yRear(:)], 'rpm');
xlabel('Time [s]');
ylabel('Collective command [rpm]');
title('Front and rear collective commands', 'Interpreter', 'none');
legend({'front collective cmd', 'rear collective cmd'}, 'Location', 'best');
hold off;
end

function localPlotSurfaceStates(out)
[t, Y] = localGetSurfaceStateMatrix(out);
if isempty(t) || isempty(Y) || size(Y, 2) < 4
    return
end

wingsDeg = Y(:, 1:2) * (180 / pi);
tailsDeg = Y(:, 3:4) * (180 / pi);

figure('Name', 'Surface states', 'NumberTitle', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, wingsDeg(:, 1), 'LineWidth', 1.4);
hold on;
plot(t, wingsDeg(:, 2), 'LineWidth', 1.4);
grid on;
localApplyYAxis(wingsDeg(:), 'deg');
xlabel('Time [s]');
ylabel('Wing surface state [deg]');
title('Wing surface states', 'Interpreter', 'none');
legend({'dLW', 'dRW'}, 'Location', 'best');
hold off;

nexttile;
plot(t, tailsDeg(:, 1), 'LineWidth', 1.4);
hold on;
plot(t, tailsDeg(:, 2), 'LineWidth', 1.4);
grid on;
localApplyYAxis(tailsDeg(:), 'deg');
xlabel('Time [s]');
ylabel('Tail surface state [deg]');
title('Tail surface states', 'Interpreter', 'none');
legend({'dLT', 'dRT'}, 'Location', 'best');
hold off;

sgtitle('Surface states', 'Interpreter', 'none');
end

function [tFront, yFront, tRear, yRear] = localGetCollectiveSeries(out)
tFront = [];
yFront = [];
tRear = [];
yRear = [];

try
    frontStruct = localGetOutputField(out, 'front_collective_rpm_out');
    rearStruct = localGetOutputField(out, 'rear_collective_rpm_out');
    [tFront, yFront] = localExtractSingleChannelSeries(frontStruct, 'front_collective_rpm_out');
    [tRear, yRear] = localExtractSingleChannelSeries(rearStruct, 'rear_collective_rpm_out');
    return
catch
end

if ~evalin('base', 'exist(''cmds'', ''var'')')
    return
end

cmds = evalin('base', 'cmds');
if ~isstruct(cmds) || ~isfield(cmds, 'front_cmd') || ~isfield(cmds, 'rear_cmd')
    return
end

frontCmd = cmds.front_cmd;
rearCmd = cmds.rear_cmd;
if ~isnumeric(frontCmd) || size(frontCmd, 2) < 2 || ~isnumeric(rearCmd) || size(rearCmd, 2) < 2
    return
end

tFront = frontCmd(:, 1);
yFront = frontCmd(:, 2);
tRear = rearCmd(:, 1);
yRear = rearCmd(:, 2);
end

function [t, Y] = localGetSurfaceStateMatrix(out)
t = [];
Y = [];

surfaceNames = {'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'};
surfaceSeries = cell(1, numel(surfaceNames));

try
    for idx = 1:numel(surfaceNames)
        signalStruct = localGetOutputField(out, surfaceNames{idx});
        [t_i, y_i] = localExtractSingleChannelSeries(signalStruct, surfaceNames{idx});
        surfaceSeries{idx} = struct('time', t_i, 'data', y_i);
    end

    t = surfaceSeries{1}.time;
    Y = zeros(numel(t), numel(surfaceNames));
    Y(:, 1) = surfaceSeries{1}.data;
    for idx = 2:numel(surfaceNames)
        Y(:, idx) = interp1(surfaceSeries{idx}.time, surfaceSeries{idx}.data, t, 'linear');
    end
    return
catch
end

try
    signalStruct = localGetOutputField(out, 'viz_surface_states');
catch
    return
end

if ~isstruct(signalStruct) || ~isfield(signalStruct, 'time') || ...
        ~isfield(signalStruct, 'signals') || ~isstruct(signalStruct.signals) || ...
        ~isfield(signalStruct.signals, 'values')
    return
end

t = signalStruct.time(:);
Y = localExtractSignalMatrix(signalStruct.signals.values, numel(t), 'Surface states');
end

function [t, y] = localExtractSingleChannelSeries(signalStruct, signalName)
if ~isstruct(signalStruct) || ~isfield(signalStruct, 'time') || ...
        ~isfield(signalStruct, 'signals') || ~isstruct(signalStruct.signals) || ...
        ~isfield(signalStruct.signals, 'values')
    error('Signal "%s" is not a recognized Structure With Time signal.', signalName);
end

t = signalStruct.time(:);
Y = localExtractSignalMatrix(signalStruct.signals.values, numel(t), signalName);
if size(Y, 2) ~= 1
    error('Signal "%s" must contain exactly one channel.', signalName);
end
y = Y(:, 1);
end

function specs = localCommandSpecs()
specs = struct();

specs.airData_cmd = localMakeSpec( ...
    'Command: air data', ...
    {'V_inf_cmd','alpha_cmd','beta_cmd'}, ...
    {'m/s','rad','rad'});

specs.eul_cmd = localMakeSpec( ...
    'Command: Euler attitude', ...
    {'phi_cmd','theta_cmd','psi_cmd'}, ...
    {'rad','rad','rad'});

specs.gps_Pos_cmd = localMakeSpec( ...
    'Command: GPS position', ...
    {'north_cmd','east_cmd','down_cmd'}, ...
    {'m','m','m'});

specs.omega_cmd = localMakeSpec( ...
    'Command: body rates', ...
    {'P_cmd','Q_cmd','R_cmd'}, ...
    {'rad/s','rad/s','rad/s'});

specs.accel_cmd = localMakeSpec( ...
    'Command: acceleration', ...
    {'a_x_cmd','a_y_cmd','a_z_cmd'}, ...
    {'m/s^2','m/s^2','m/s^2'});

specs.motor_cmd = localMakeSpec( ...
    'Command: motor RPM', ...
    {'FR_cmd','FL_cmd','RR_cmd','RL_cmd'}, ...
    {'rpm','rpm','rpm','rpm'});

specs.tilt_cmd = localMakeSpec( ...
    'Command: tilt angles', ...
    {'front_right_cmd','front_left_cmd'}, ...
    {'deg','deg'});

specs.front_cmd = localMakeSpec( ...
    'Command: front collective', ...
    {'front_collective_cmd'}, ...
    {'rpm'});

specs.rear_cmd = localMakeSpec( ...
    'Command: rear collective', ...
    {'rear_collective_cmd'}, ...
    {'rpm'});
end

function spec = localMakeSpec(displayName, channelNames, units)
spec = struct();
spec.displayName = displayName;
spec.channelNames = channelNames;
spec.units = units;
end

function value = localGetOutputField(out, fieldName)
if isa(out, 'Simulink.SimulationOutput')
    value = out.get(fieldName);
    return
end

if isstruct(out) && isfield(out, fieldName)
    value = out.(fieldName);
    return
end

error('Output container does not contain field "%s".', fieldName);
end

function localPlotSignalStruct(signalStruct, spec)
if ~isstruct(signalStruct) || ~isfield(signalStruct, 'time') || ...
        ~isfield(signalStruct, 'signals') || ~isstruct(signalStruct.signals) || ...
        ~isfield(signalStruct.signals, 'values')
    warning('Skipping "%s" because it is not a recognized Structure With Time signal.', ...
        spec.displayName);
    return
end

t = signalStruct.time(:);
Y = localExtractSignalMatrix(signalStruct.signals.values, numel(t), spec.displayName);
nChan = size(Y, 2);

if nChan ~= numel(spec.channelNames)
    warning(['Signal "%s" produced %d channels, but the plotting spec expects %d. ', ...
        'Using fallback channel names for this plot.'], ...
        spec.displayName, nChan, numel(spec.channelNames));
    spec.channelNames = arrayfun(@(idx) sprintf('channel %d', idx), 1:nChan, 'UniformOutput', false);
    spec.units = repmat({''}, 1, nChan);
end

nCols = min(3, nChan);
nRows = ceil(nChan / nCols);

figure('Name', spec.displayName, 'NumberTitle', 'off');
tiledlayout(nRows, nCols, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:nChan
    [channelData, channelUnit] = localConvertPlotUnits(Y(:, i), spec.units{i});
    nexttile;
    plot(t, channelData, 'LineWidth', 1.2);
    grid on;
    localApplyYAxis(channelData, channelUnit);
    xlabel('Time [s]');
    ylabel(localMakeAxisLabel(spec.channelNames{i}, channelUnit), 'Interpreter', 'none');
    title(localMakeAxisLabel(spec.channelNames{i}, channelUnit), 'Interpreter', 'none');
end

sgtitle(spec.displayName, 'Interpreter', 'none');
end

function localPlotCommandMatrix(cmdMatrix, spec)
if ~isnumeric(cmdMatrix) || size(cmdMatrix, 2) < 2
    warning('Skipping "%s" because it is not a valid command matrix.', spec.displayName);
    return
end

t = cmdMatrix(:, 1);
Y = cmdMatrix(:, 2:end);
nChan = size(Y, 2);

if nChan ~= numel(spec.channelNames)
    warning(['Command "%s" produced %d channels, but the plotting spec expects %d. ', ...
        'Using fallback channel names for this plot.'], ...
        spec.displayName, nChan, numel(spec.channelNames));
    spec.channelNames = arrayfun(@(idx) sprintf('channel %d', idx), 1:nChan, 'UniformOutput', false);
    spec.units = repmat({''}, 1, nChan);
end

nCols = min(3, nChan);
nRows = ceil(nChan / nCols);

figure('Name', spec.displayName, 'NumberTitle', 'off');
tiledlayout(nRows, nCols, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:nChan
    [channelData, channelUnit] = localConvertPlotUnits(Y(:, i), spec.units{i});
    nexttile;
    plot(t, channelData, 'LineWidth', 1.2);
    grid on;
    localApplyYAxis(channelData, channelUnit);
    xlabel('Time [s]');
    ylabel(localMakeAxisLabel(spec.channelNames{i}, channelUnit), 'Interpreter', 'none');
    title(localMakeAxisLabel(spec.channelNames{i}, channelUnit), 'Interpreter', 'none');
end

sgtitle(spec.displayName, 'Interpreter', 'none');
end

function Y = localExtractSignalMatrix(rawValues, nTime, signalName)
sz = size(rawValues);

if ismatrix(rawValues)
    if sz(1) == nTime
        Y = rawValues;
        return
    end

    if sz(2) == nTime
        Y = rawValues.';
        return
    end

    error('Signal "%s" does not match time length in either matrix dimension.', signalName);
end

if ndims(rawValues) == 3 && sz(3) == nTime
    if sz(2) == 1
        Y = squeeze(rawValues(:, 1, :)).';
        return
    end

    Y = zeros(nTime, sz(1) * sz(2));
    idx = 1;
    for row = 1:sz(1)
        for col = 1:sz(2)
            Y(:, idx) = squeeze(rawValues(row, col, :));
            idx = idx + 1;
        end
    end
    return
end

error('Signal "%s" has unsupported values size %s.', signalName, mat2str(sz));
end

function label = localMakeAxisLabel(channelName, unitName)
if isempty(unitName)
    label = channelName;
else
    label = sprintf('%s [%s]', channelName, unitName);
end
end

function [plotData, plotUnit] = localConvertPlotUnits(rawData, rawUnit)
plotData = rawData;
plotUnit = rawUnit;

if strcmp(rawUnit, 'rad')
    plotData = rawData * (180 / pi);
    plotUnit = 'deg';
elseif strcmp(rawUnit, 'rad/s')
    plotData = rawData * (180 / pi);
    plotUnit = 'deg/s';
end
end

function localApplyYAxis(plotData, plotUnit)
finiteData = plotData(isfinite(plotData));
if isempty(finiteData)
    return
end

[minSpan, zeroSnap] = localGetMinimumSpan(plotUnit);
if isempty(minSpan)
    return
end

yMin = min(finiteData);
yMax = max(finiteData);
span = yMax - yMin;
center = 0.5 * (yMin + yMax);

if abs(center) < zeroSnap
    center = 0;
end

targetSpan = max(span, minSpan);
margin = 0.05 * targetSpan;
ylim([center - 0.5 * targetSpan - margin, center + 0.5 * targetSpan + margin]);
end

function [minSpan, zeroSnap] = localGetMinimumSpan(plotUnit)
switch plotUnit
    case 'deg/s'
        minSpan = 1.0;
    case 'deg'
        minSpan = 2.0;
    case 'm/s'
        minSpan = 1.0;
    case 'm/s^2'
        minSpan = 0.5;
    case 'rpm'
        minSpan = 20.0;
    case 'm'
        minSpan = 1.0;
    otherwise
        minSpan = [];
end

if isempty(minSpan)
    zeroSnap = [];
else
    zeroSnap = 0.25 * minSpan;
end
end
