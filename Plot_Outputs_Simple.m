%% Basic longitudinal output plots

if exist('simOut', 'var') && ~isempty(simOut)
    simForPlot = simOut;
elseif exist('out', 'var') && ~isempty(out)
    simForPlot = out;
elseif evalin('base', 'exist(''out'', ''var'')')
    simForPlot = evalin('base', 'out');
else
    error('No simulation output found. Run Wrapper first so out exists.');
end

[tVel, vel] = getSignal(simForPlot, ["V_B_truth", "V_BA_truth", "vel"]);
vel = padColumns(vel, 3);

hasEul = true;
try
    [tEul, eul] = getSignal(simForPlot, ["eul_truth", "eul_meas_log", "eul"]);
    eul = padColumns(eul, 3);
catch
    hasEul = false;
    tEul = tVel;
    eul = zeros(numel(tVel), 3);
end

hasRates = true;
try
    [tRates, rates] = getSignal(simForPlot, ["omega_truth", "omega_Meas_log", "omega"]);
    rates = padColumns(rates, 3);
catch
    hasRates = false;
    tRates = tVel;
    rates = zeros(numel(tVel), 3);
end

hasAirspeed = true;
try
    [tAir, airspeed] = getSignal(simForPlot, ["vinf_truth", "airDataMeas_log", "airspeed_meas", "vinf"]);
    airspeed = airspeed(:, 1);
catch
    hasAirspeed = false;
    tAir = tVel;
    airspeed = zeros(numel(tVel), 1);
end

fig = figure('Color', 'w', 'Name', 'Simple Wrapper outputs');
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(tVel, vel(:, 1), 'LineWidth', 1.4);
hold on;
plot(tVel, vel(:, 3), 'LineWidth', 1.4);
grid on;
title('Body Velocity');
ylabel('m/s');
xlabel('Time [s]');
legend({'u', 'w'}, 'Location', 'best');
styleAxes(gca);

nexttile;
if hasAirspeed
    plot(tAir, airspeed, 'LineWidth', 1.4);
end
grid on;
title('Airspeed');
ylabel('m/s');
xlabel('Time [s]');
styleAxes(gca);

nexttile;
if hasEul
    plot(tEul, rad2deg(eul(:, 2)), 'LineWidth', 1.4);
end
grid on;
title('Pitch Angle');
ylabel('\theta [deg]');
xlabel('Time [s]');
styleAxes(gca);

nexttile;
if hasRates
    plot(tRates, rad2deg(rates(:, 2)), 'LineWidth', 1.4);
end
grid on;
title('Pitch Rate');
ylabel('q [deg/s]');
xlabel('Time [s]');
styleAxes(gca);

sg = sgtitle('Wrapper Output Summary');
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
error('None of these signals were found: %s', strjoin(cellstr(names), ', '));
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

function styleAxes(ax)
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';
ax.GridColor = [0.75 0.75 0.75];
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
