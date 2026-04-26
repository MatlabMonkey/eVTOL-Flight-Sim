%% plot_evtol_workspace_outputs.m
% This script scans the base workspace for simulation outputs and makes
% figures for everything that was written out by the Simulink run.
%
% It is set up around the EVTOL_6DOF_Sim_Wrapper model you uploaded, but it
% also handles generic Simulink outputs like:
%   - tout
%   - yout
%   - logsout
%   - SimulationOutput objects (simOut)
%   - To Workspace variables saved as "Structure With Time"
%   - timeseries objects
%   - numeric arrays if they line up with tout
%
% Usage:
%   1) Run Full_Sim_Init.m
%   2) Run the simulation
%   3) Run this script
%
% Optional:
%   Set saveFigures = true if you want PNGs written to disk.

clear plotterCfg

plotterCfg.saveFigures = false;
plotterCfg.outputDir   = fullfile(pwd, 'workspace_plots');
plotterCfg.closeFirst  = false;
plotterCfg.maxTilesPerFigure = 12;

% Known outputs from EVTOL_6DOF_Sim_Wrapper.slx
plotterCfg.preferredVars = {
    'tout'
    'yout'
    'logsout'
    'pos_NED'
    'V_B_truth'
    'V_E_truth'
    'eul_truth'
    'omega_truth'
    'C_NB_truth'
    'V_BA_truth'
    'vinf_truth'
    'alpha_truth'
    'beta_truth'
    'specific_force_truth'
    'magHdgMeas_log'
    'magMeasBody_log'
    'gpsVelMeas_log'
    'airDataMeas_log'
    'eul_meas_log'
    'gps_Pos_Meas_log'
    'omega_Meas_log'
    'accel_Meas_log'
};

if plotterCfg.closeFirst
    close all
end

if plotterCfg.saveFigures && ~exist(plotterCfg.outputDir, 'dir')
    mkdir(plotterCfg.outputDir);
end

baseVars = evalin('base', 'whos');
baseNames = {baseVars.name};

fprintf('\nSearching base workspace for plottable simulation outputs...\n');

% Build a target list. Start with the known EVTOL outputs, then append any
% other simulation-like objects that are present.
targetNames = {};
for i = 1:numel(plotterCfg.preferredVars)
    if ismember(plotterCfg.preferredVars{i}, baseNames)
        targetNames{end+1} = plotterCfg.preferredVars{i}; %#ok<SAGROW>
    end
end

for i = 1:numel(baseVars)
    className = baseVars(i).class;
    varName   = baseVars(i).name;

    isGenericSimOutput = strcmp(className, 'Simulink.SimulationOutput') || ...
                         strcmp(className, 'Simulink.SimulationData.Dataset') || ...
                         strcmp(className, 'timeseries');

    if isGenericSimOutput
        if ~ismember(varName, targetNames)
            targetNames{end+1} = varName; %#ok<SAGROW>
        end
    end
end

if isempty(targetNames)
    warning('No plottable simulation outputs were found in the base workspace.');
    return
end

for i = 1:numel(targetNames)
    varName = targetNames{i};
    try
        data = evalin('base', varName);
        plotAnyVariable(data, varName, plotterCfg);
    catch ME
        warning('Could not plot "%s": %s', varName, ME.message);
    end
end

fprintf('Done. Processed %d workspace targets.\n', numel(targetNames));

%% Local functions
function plotAnyVariable(data, varName, cfg)
    if isa(data, 'Simulink.SimulationOutput')
        plotSimulationOutput(data, varName, cfg);
        return
    end

    if isa(data, 'Simulink.SimulationData.Dataset')
        plotDataset(data, varName, cfg);
        return
    end

    if isa(data, 'timeseries')
        plotTimeseries(data, varName, cfg);
        return
    end

    if isStructWithTime(data)
        plotStructWithTime(data, varName, cfg);
        return
    end

    if isnumeric(data) || islogical(data)
        plotNumericVariable(data, varName, cfg);
        return
    end

    if isstruct(data)
        plotPlainStruct(data, varName, cfg);
        return
    end

    fprintf('Skipping "%s" of class %s\n', varName, class(data));
end

function plotSimulationOutput(simOut, simOutName, cfg)
    fprintf('Plotting SimulationOutput: %s\n', simOutName);

    tryPlotSimOutField(simOut, simOutName, 'logsout', cfg);
    tryPlotSimOutField(simOut, simOutName, 'yout', cfg);
    tryPlotSimOutField(simOut, simOutName, 'xout', cfg);
    tryPlotSimOutField(simOut, simOutName, 'tout', cfg);

    names = who(simOut);
    for i = 1:numel(names)
        fieldName = names{i};
        if ismember(fieldName, {'logsout','yout','xout','tout'})
            continue
        end
        try
            value = simOut.get(fieldName);
            plotAnyVariable(value, [simOutName '.' fieldName], cfg);
        catch
        end
    end
end

function tryPlotSimOutField(simOut, simOutName, fieldName, cfg)
    try
        value = simOut.get(fieldName);
        plotAnyVariable(value, [simOutName '.' fieldName], cfg);
    catch
    end
end

function plotDataset(ds, dsName, cfg)
    fprintf('Plotting Dataset: %s\n', dsName);
    n = numElements(ds);
    if n == 0
        fprintf('  Dataset "%s" is empty.\n', dsName);
        return
    end

    for i = 1:n
        elem = getElement(ds, i);
        elemName = getDatasetElementName(ds, i);

        if isprop(elem, 'Values')
            values = elem.Values;
            plotAnyVariable(values, sprintf('%s.%s', dsName, elemName), cfg);
        else
            plotAnyVariable(elem, sprintf('%s.%s', dsName, elemName), cfg);
        end
    end
end

function name = getDatasetElementName(ds, idx)
    try
        tmp = ds.getElement(idx);
        name = tmp.Name;
        if isempty(name)
            name = sprintf('element_%d', idx);
        end
    catch
        name = sprintf('element_%d', idx);
    end
    name = matlab.lang.makeValidName(name);
end

function plotTimeseries(ts, tsName, cfg)
    t = ts.Time;
    y = ts.Data;
    plotTimeMatrix(t, y, tsName, cfg);
end

function plotStructWithTime(s, structName, cfg)
    if ~isStructWithTime(s)
        error('Input is not a Structure With Time style struct.');
    end

    t = s.time;
    y = s.signals.values;
    plotTimeMatrix(t, y, structName, cfg);
end

function plotNumericVariable(x, varName, cfg)
    if isscalar(x)
        fprintf('Skipping scalar numeric variable: %s\n', varName);
        return
    end

    t = [];
    if evalin('base', 'exist(''tout'', ''var'')')
        tout = evalin('base', 'tout');
        if isvector(tout) && size(x,1) == numel(tout)
            t = tout(:);
        elseif isvector(tout) && isvector(x) && numel(x) == numel(tout)
            t = tout(:);
            x = x(:);
        end
    end

    if isempty(t)
        t = (1:size(x,1)).';
    end

    plotTimeMatrix(t, x, varName, cfg);
end

function plotPlainStruct(s, structName, cfg)
    fields = fieldnames(s);
    if isempty(fields)
        fprintf('Skipping empty struct: %s\n', structName);
        return
    end

    for i = 1:numel(fields)
        f = fields{i};
        value = s.(f);
        childName = sprintf('%s.%s', structName, f);

        if isa(value, 'timeseries') || isa(value, 'Simulink.SimulationData.Dataset') || ...
           isa(value, 'Simulink.SimulationOutput') || isStructWithTime(value) || ...
           isnumeric(value) || islogical(value) || isstruct(value)
            try
                plotAnyVariable(value, childName, cfg);
            catch ME
                warning('Could not plot struct field "%s": %s', childName, ME.message);
            end
        end
    end
end

function tf = isStructWithTime(s)
    tf = isstruct(s) && isfield(s, 'time') && isfield(s, 'signals') && ...
         isstruct(s.signals) && isfield(s.signals, 'values');
end

function plotTimeMatrix(t, y, plotName, cfg)
    y = squeeze(y);

    if isempty(y)
        fprintf('Skipping empty data: %s\n', plotName);
        return
    end

    if isvector(y)
        y = y(:);
    end

    sz = size(y);

    if numel(sz) == 2
        nRows = sz(1);
        nCols = sz(2);
        if numel(t) ~= nRows && nCols == numel(t)
            y = y.';
            sz = size(y);
            nRows = sz(1);
            nCols = sz(2);
        end

        if numel(t) ~= nRows
            error('Time length does not match data length for %s.', plotName);
        end

        plotColumnsOverPages(t, y, plotName, cfg);
        return
    end

    % Higher-dimensional data. Flatten all channels except time dimension.
    nRows = sz(1);
    if numel(t) ~= nRows
        error('Time length does not match first dimension for %s.', plotName);
    end

    y2 = reshape(y, nRows, []);
    plotColumnsOverPages(t, y2, plotName, cfg);
end

function plotColumnsOverPages(t, y, plotName, cfg)
    nChan = size(y, 2);
    if nChan == 0
        fprintf('Skipping empty channel set: %s\n', plotName);
        return
    end

    nPerFig = max(1, cfg.maxTilesPerFigure);
    nPages = ceil(nChan / nPerFig);

    for page = 1:nPages
        idx1 = (page - 1) * nPerFig + 1;
        idx2 = min(page * nPerFig, nChan);
        chanIdx = idx1:idx2;
        nThis = numel(chanIdx);

        nCols = min(3, nThis);
        nRows = ceil(nThis / nCols);

        figure('Name', makeFigureTitle(plotName, page, nPages), 'NumberTitle', 'off');
        tiledlayout(nRows, nCols, 'TileSpacing', 'compact', 'Padding', 'compact');

        for k = 1:nThis
            c = chanIdx(k);
            nexttile;
            plot(t, y(:, c), 'LineWidth', 1.2);
            grid on;
            xlabel('Time [s]');
            ylabel(sprintf('Channel %d', c));
            title(makePlotTitle(plotName, page, nPages, c), 'Interpreter', 'none');
        end

        sgtitle(makeFigureTitle(plotName, page, nPages), 'Interpreter', 'none');

        if cfg.saveFigures
            fileBase = sanitizeFileName(makeFigureTitle(plotName, page, nPages));
            exportgraphics(gcf, fullfile(cfg.outputDir, [fileBase '.png']), 'Resolution', 200);
        end
    end
end

function txt = makePlotTitle(plotName, page, nPages, channelIdx)
    if nPages == 1
        txt = sprintf('%s | channel %d', plotName, channelIdx);
    else
        txt = sprintf('%s | page %d/%d | channel %d', plotName, page, nPages, channelIdx);
    end
end

function txt = makeFigureTitle(plotName, page, nPages)
    if nPages == 1
        txt = plotName;
    else
        txt = sprintf('%s | page %d of %d', plotName, page, nPages);
    end
end

function out = sanitizeFileName(in)
    out = regexprep(in, '[^a-zA-Z0-9_\-\. ]', '_');
    out = strrep(out, ' ', '_');
end
