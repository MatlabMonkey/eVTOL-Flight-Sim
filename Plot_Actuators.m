%% Actuator command and state plots

if exist('simOut', 'var') && ~isempty(simOut)
    simForPlot = simOut;
elseif exist('out', 'var') && ~isempty(out)
    simForPlot = out;
else
    error('No simulation output found. Run Run_Main first so simOut exists.');
end

if ~exist('cmds', 'var') || ~isstruct(cmds)
    error('No cmds struct found. Run the command-schedule section first.');
end

[tAct, actuatorState] = getSignal(simForPlot, ["actuator_state_truth", "actuator_state_meas"]);
actuatorState = padColumns(actuatorState, 10);

rpmActual = actuatorState(:, 1:4);
tiltActualDeg = actuatorState(:, 5:6);
surfaceActualDeg = rad2deg(actuatorState(:, 7:10));

tiltCmdDeg = resampleCmd(cmds.tilt_cmd, 2, tAct);

rpmCmd = buildControllerRotorCommand(simForPlot, cmds, tAct);
surfaceCmdDeg = buildControllerSurfaceCommand(simForPlot, tAct);

hasAirspeed = true;
try
    [tAir, airspeedActual] = getSignal(simForPlot, ["airDataMeas_log", "airspeed_meas", "vinf_truth", "vinf"]);
    airspeedActual = airspeedActual(:, 1);
    airspeedActual = interp1(tAir, airspeedActual, tAct, 'linear', 'extrap');
catch
    hasAirspeed = false;
    airspeedActual = zeros(numel(tAct), 1);
end
airspeedCmd = resampleCmd(cmds.airData_cmd, 1, tAct);

fig = figure('Color', 'w', 'Name', 'Actuator commands versus actual states');
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
if hasAirspeed
    plot(tAct, airspeedActual, 'LineWidth', 1.6);
    hold on;
end
plot(tAct, airspeedCmd, '--', 'LineWidth', 1.2);
grid on;
title('Airspeed');
ylabel('m/s');
xlabel('Time [s]');
legend({'actual', 'command'}, 'Location', 'best');
styleAxes(gca);

nexttile;
plot(tAct, tiltActualDeg(:, 1), 'LineWidth', 1.6);
hold on;
plot(tAct, tiltCmdDeg(:, 1), '--', 'LineWidth', 1.2);
grid on;
title('Tilt');
ylabel('deg');
xlabel('Time [s]');
legend({'actual', 'command'}, 'Location', 'best');
styleAxes(gca);

nexttile;
plot(tAct, rpmActual(:, [1 3]), 'LineWidth', 1.6);
hold on;
plot(tAct, rpmCmd(:, [1 3]), '--', 'LineWidth', 1.2);
grid on;
title('Rotor Speed');
ylabel('RPM');
xlabel('Time [s]');
legend({'front actual', 'rear actual', 'front command', 'rear command'}, 'Location', 'best');
styleAxes(gca);

nexttile;
plot(tAct, surfaceActualDeg(:, [1 3]), 'LineWidth', 1.6);
hold on;
plot(tAct, surfaceCmdDeg(:, [1 3]), '--', 'LineWidth', 1.2);
grid on;
title('Control Surface Deflection');
ylabel('deg');
xlabel('Time [s]');
legend({'LW actual', 'LT actual', 'LW command', 'LT command'}, 'Location', 'best');
styleAxes(gca);

sg = sgtitle('Actuator Commands Versus Actual States');
sg.Color = 'k';
sg.FontWeight = 'bold';

function [t, y] = getSignal(simSource, names)
for iName = 1:numel(names)
    name = char(names(iName));
    try
        raw = simSource.get(name);
    catch
        raw = [];
    end

    if isempty(raw)
        try
            raw = simSource.get(['out.' name]);
        catch
            raw = [];
        end
    end

    if isempty(raw) && isstruct(simSource) && isfield(simSource, name)
        raw = simSource.(name);
    end

    if ~isempty(raw)
        [t, y] = unpackSignal(raw, simSource);
        return;
    end
end

try
    disp('Logged signals available:');
    disp(string(simSource.who).');
catch
end
error('None of these signals were found: %s', strjoin(cellstr(names), ', '));
end

function rpmCmd = buildControllerRotorCommand(simSource, cmds, t)
try
    [tFront, frontCmd] = getSignal(simSource, ["front_collective_rpm_out", "front_collective_rpm_cmd"]);
    [tRear, rearCmd] = getSignal(simSource, ["rear_collective_rpm_out", "rear_collective_rpm_cmd"]);
    frontCmd = interp1(tFront, frontCmd(:, 1), t, 'linear', 'extrap');
    rearCmd = interp1(tRear, rearCmd(:, 1), t, 'linear', 'extrap');
catch
    warning('Plot_Actuators:UsingScheduledRotorCommand', ...
        'Controller rotor command logs were not found. Falling back to scheduled trim commands.');
    frontCmd = resampleCmd(cmds.front_cmd, 1, t);
    rearCmd = resampleCmd(cmds.rear_cmd, 1, t);
end
rpmCmd = [frontCmd, frontCmd, rearCmd, rearCmd];
end

function surfaceCmdDeg = buildControllerSurfaceCommand(simSource, t)
try
    [tLW, dLW, srcLW] = getSignalWithSource(simSource, "deltaLW_cmd");
    [tRW, dRW] = getSignal(simSource, "deltaRW_cmd");
    [tLT, dLT] = getSignal(simSource, "deltaLT_cmd");
    [tRT, dRT] = getSignal(simSource, "deltaRT_cmd");
    surfaceCmd = [ ...
        interp1(tLW, dLW(:, 1), t, 'linear', 'extrap'), ...
        interp1(tRW, dRW(:, 1), t, 'linear', 'extrap'), ...
        interp1(tLT, dLT(:, 1), t, 'linear', 'extrap'), ...
        interp1(tRT, dRT(:, 1), t, 'linear', 'extrap')];
    surfaceCmdDeg = convertSurfaceCmdToDeg(surfaceCmd, srcLW);
catch
    warning('Plot_Actuators:MissingSurfaceCommand', ...
        'Controller surface command logs were not found. Plotting NaN command traces.');
    surfaceCmdDeg = nan(numel(t), 4);
end
end

function [t, y, sourceName] = getSignalWithSource(simSource, name)
sourceName = char(name);
try
    raw = simSource.get(sourceName);
catch
    raw = [];
end

if isempty(raw)
    try
        raw = simSource.get(['out.' sourceName]);
    catch
        raw = [];
    end
end

if isempty(raw) && isstruct(simSource) && isfield(simSource, sourceName)
    raw = simSource.(sourceName);
end

if isempty(raw)
    error('Signal not found: %s', sourceName);
end

[t, y] = unpackSignal(raw, simSource);
end

function yDeg = convertSurfaceCmdToDeg(y, sourceName)
if any(strcmp(sourceName, {'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd', ...
        'viz_surface_states', 'surface_local_cmd'}))
    yDeg = rad2deg(y);
else
    yDeg = y;
end
end

function [t, y] = unpackSignal(raw, simSource)
if isa(raw, 'timeseries')
    t = raw.Time(:);
    y = formatData(raw.Data);
elseif isstruct(raw) && isfield(raw, 'time') && isfield(raw, 'signals')
    t = raw.time(:);
    y = formatData(raw.signals.values);
elseif isnumeric(raw)
    y = formatData(raw);
    try
        t = simSource.get('tout');
        t = t(:);
    catch
        t = (0:size(y, 1)-1).';
    end
else
    error('Unsupported logged signal type: %s', class(raw));
end

if size(y, 1) ~= numel(t) && size(y, 2) == numel(t)
    y = y.';
end
if size(y, 1) ~= numel(t)
    t = linspace(t(1), t(end), size(y, 1)).';
end
end

function y = formatData(data)
y = squeeze(data);
if isvector(y)
    y = y(:);
elseif ~ismatrix(y)
    y = reshape(y, size(y, 1), []);
end
end

function y = padColumns(y, nCols)
if size(y, 2) < nCols
    y(:, end+1:nCols) = 0;
elseif size(y, 2) > nCols
    y = y(:, 1:nCols);
end
end

function y = resampleCmd(cmd, nCols, t)
tCmd = cmd(:, 1);
yCmd = cmd(:, 2:1+nCols);
if size(yCmd, 2) < nCols
    yCmd(:, end+1:nCols) = yCmd(:, end);
end
y = interp1(tCmd, yCmd(:, 1:nCols), t, 'linear', 'extrap');
end

function styleAxes(ax)
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';
ax.GridColor = [0.75 0.75 0.75];
ax.MinorGridColor = [0.85 0.85 0.85];
ax.Title.Color = 'k';
ax.XLabel.Color = 'k';
ax.YLabel.Color = 'k';
leg = legend(ax);
if ~isempty(leg)
    leg.TextColor = 'k';
    leg.Color = 'w';
    leg.EdgeColor = [0.2 0.2 0.2];
end
end
