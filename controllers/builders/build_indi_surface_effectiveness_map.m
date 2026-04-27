function map = build_indi_surface_effectiveness_map(opts)
%BUILD_INDI_SURFACE_EFFECTIVENESS_MAP Build INDI flap/elevator maps.
%
% The map is generated through Trim_Plant so the data follows the same
% Simulink aero path used by trim/simulation. The sampled derivatives use
% individual logged surface loads:
%   d[(LW_F + RW_F); (LW_M + RW_M)] / d(delta_f)
%   d[(LT_F + RT_F); (LT_M + RT_M)] / d(delta_e)
%
% Default output:
%   databases/indi_surface_effectiveness_map_coarse.mat

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
databaseDir = fullfile(repoRoot, 'databases');
if exist(databaseDir, 'dir') ~= 7
    mkdir(databaseDir);
end

previousDir = pwd;
cleanupDir = onCleanup(@() cd(previousDir));
cd(repoRoot);

evalin('base', 'Init_Main');

plant = localLoadPlantInputsFromBase();
polarSource = localRequirePolarTables( ...
    plant.wingPolar, plant.tailPolar, evalin('base', 'polar_data_file'));

load_system('Trim_Plant');

map = localInitializeMap(opts, polarSource);
map.metadata.started_at = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
map.metadata.output_mat_path = opts.outputMatPath;
map.metadata.output_summary_path = opts.outputSummaryPath;

totalCalls = 2 * numel(opts.vinf_mps) * numel(opts.alpha_deg) * numel(opts.delta_deg);
callIndex = 0;
runTimer = tic;

for iV = 1:numel(opts.vinf_mps)
    for iA = 1:numel(opts.alpha_deg)
        for iD = 1:numel(opts.delta_deg)
            vinf = opts.vinf_mps(iV);
            alpha = opts.alpha_deg(iA);
            delta = opts.delta_deg(iD);

            [minusDelta, plusDelta, denomRad] = localDerivativeStencil(delta, opts);

            sampleMinus = localSampleCombinedSurfaces(vinf, alpha, minusDelta, opts, plant);
            callIndex = callIndex + 1;
            localPrintProgress(callIndex, totalCalls, runTimer, opts.progressEveryCalls);

            samplePlus = localSampleCombinedSurfaces(vinf, alpha, plusDelta, opts, plant);
            callIndex = callIndex + 1;

            map.flap.dF_drad_N_per_rad(iV, iA, iD, :) = ...
                (samplePlus.flap.force_N - sampleMinus.flap.force_N) ./ denomRad;
            map.flap.dM_drad_Nm_per_rad(iV, iA, iD, :) = ...
                (samplePlus.flap.moment_Nm - sampleMinus.flap.moment_Nm) ./ denomRad;
            map.elevator.dF_drad_N_per_rad(iV, iA, iD, :) = ...
                (samplePlus.elevator.force_N - sampleMinus.elevator.force_N) ./ denomRad;
            map.elevator.dM_drad_Nm_per_rad(iV, iA, iD, :) = ...
                (samplePlus.elevator.moment_Nm - sampleMinus.elevator.moment_Nm) ./ denomRad;
            localPrintProgress(callIndex, totalCalls, runTimer, opts.progressEveryCalls);

            if opts.checkpointEveryCalls > 0 && mod(callIndex, opts.checkpointEveryCalls) == 0
                map.metadata.calls_completed = callIndex;
                map.metadata.elapsed_s = toc(runTimer);
                localSaveMap(map, opts.outputMatPath);
            end
        end
    end
end

map.metadata.calls_completed = callIndex;
map.metadata.elapsed_s = toc(runTimer);
map.metadata.seconds_per_call = map.metadata.elapsed_s / max(callIndex, 1);
map.metadata.finished_at = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));

localSaveMap(map, opts.outputMatPath);
localWriteSummary(map, opts.outputSummaryPath);
assignin('base', 'indi_surface_effectiveness_map', map);

fprintf('\nINDI surface effectiveness map complete.\n');
fprintf('  calls          : %d\n', callIndex);
fprintf('  elapsed        : %.1f s (%.1f min)\n', map.metadata.elapsed_s, map.metadata.elapsed_s / 60.0);
fprintf('  seconds/call   : %.4f\n', map.metadata.seconds_per_call);
fprintf('  map            : %s\n', opts.outputMatPath);
fprintf('  summary        : %s\n\n', opts.outputSummaryPath);
end

function opts = localApplyDefaults(opts)
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
databaseDir = fullfile(repoRoot, 'databases');

if ~isfield(opts, 'vinf_mps') || isempty(opts.vinf_mps)
    opts.vinf_mps = [2.5, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80];
end
if ~isfield(opts, 'alpha_deg') || isempty(opts.alpha_deg)
    opts.alpha_deg = -10:5:45;
end
if ~isfield(opts, 'delta_deg') || isempty(opts.delta_deg)
    opts.delta_deg = -25:5:25;
end
if ~isfield(opts, 'perturbation_deg') || isempty(opts.perturbation_deg)
    opts.perturbation_deg = 0.5;
end
if ~isfield(opts, 'surfaceLimit_deg') || isempty(opts.surfaceLimit_deg)
    opts.surfaceLimit_deg = 25.0;
end
if ~isfield(opts, 'stopTime_s') || isempty(opts.stopTime_s)
    opts.stopTime_s = 0.02;
end
if ~isfield(opts, 'outputMatPath') || isempty(opts.outputMatPath)
    opts.outputMatPath = fullfile(databaseDir, 'indi_surface_effectiveness_map_coarse.mat');
end
if ~isfield(opts, 'outputSummaryPath') || isempty(opts.outputSummaryPath)
    opts.outputSummaryPath = fullfile(databaseDir, 'indi_surface_effectiveness_map_coarse.md');
end
if ~isfield(opts, 'checkpointEveryCalls') || isempty(opts.checkpointEveryCalls)
    opts.checkpointEveryCalls = 500;
end
if ~isfield(opts, 'progressEveryCalls') || isempty(opts.progressEveryCalls)
    opts.progressEveryCalls = 100;
end
end

function plant = localLoadPlantInputsFromBase()
plant = struct();
plant.prop = evalin('base', 'prop');
plant.wingL = evalin('base', 'wingL');
plant.wingR = evalin('base', 'wingR');
plant.tailL = evalin('base', 'tailL');
plant.tailR = evalin('base', 'tailR');
plant.CG = evalin('base', 'CG');
plant.wingPolar = evalin('base', 'wingPolar');
plant.tailPolar = evalin('base', 'tailPolar');
end

function map = localInitializeMap(opts, polarSource)
nV = numel(opts.vinf_mps);
nA = numel(opts.alpha_deg);
nD = numel(opts.delta_deg);

map = struct();
map.metadata = struct();
map.metadata.description = 'INDI longitudinal surface effectiveness map from Trim_Plant';
map.metadata.model = 'Trim_Plant';
map.metadata.force_signals = {'LW_F', 'RW_F', 'LT_F', 'RT_F'};
map.metadata.moment_signals = {'LW_M', 'RW_M', 'LT_M', 'RT_M'};
map.metadata.surface_signal_mode = 'individual surface load sums';
map.metadata.surface_inputs = {'delta_f', 'delta_e'};
map.metadata.units = struct( ...
    'vinf', 'm/s', ...
    'alpha', 'deg', ...
    'delta', 'deg', ...
    'derivative', 'per rad', ...
    'force_derivative', 'N/rad', ...
    'moment_derivative', 'N*m/rad');
map.metadata.stop_time_s = opts.stopTime_s;
map.metadata.perturbation_deg = opts.perturbation_deg;
map.metadata.polar_source = polarSource;
map.grid = struct( ...
    'vinf_mps', opts.vinf_mps(:), ...
    'alpha_deg', opts.alpha_deg(:), ...
    'delta_deg', opts.delta_deg(:));

template = struct();
template.dF_drad_N_per_rad = zeros(nV, nA, nD, 3);
template.dM_drad_Nm_per_rad = zeros(nV, nA, nD, 3);
map.flap = template;
map.elevator = template;
end

function [minusDelta, plusDelta, denomRad] = localDerivativeStencil(deltaDeg, opts)
limit = opts.surfaceLimit_deg;
h = opts.perturbation_deg;
minusDelta = max(deltaDeg - h, -limit);
plusDelta = min(deltaDeg + h, limit);

if plusDelta == minusDelta
    error('build_indi_surface_effectiveness_map:BadStencil', ...
        'Degenerate finite-difference stencil at delta %.3f deg.', deltaDeg);
end

denomRad = deg2rad(plusDelta - minusDelta);
end

function sample = localSampleCombinedSurfaces(vinf, alphaDeg, deltaDeg, opts, plant)
surfaceLocalRad = deg2rad(deltaDeg) * ones(4, 1);

velocityBody = [ ...
    vinf * cosd(alphaDeg); ...
    0.0; ...
    vinf * sind(alphaDeg)];

simOut = localRunOneSample(velocityBody, surfaceLocalRad, opts.stopTime_s, plant);

sample = struct();
sample.flap = struct( ...
    'force_N', localReadLoggedVector(simOut, 'LW_F') + localReadLoggedVector(simOut, 'RW_F'), ...
    'moment_Nm', localReadLoggedVector(simOut, 'LW_M') + localReadLoggedVector(simOut, 'RW_M'));
sample.elevator = struct( ...
    'force_N', localReadLoggedVector(simOut, 'LT_F') + localReadLoggedVector(simOut, 'RT_F'), ...
    'moment_Nm', localReadLoggedVector(simOut, 'LT_M') + localReadLoggedVector(simOut, 'RT_M'));
end

function simOut = localRunOneSample(velocityBody, surfaceLocalRad, stopTime_s, plant)
wingL = plant.wingL;
wingR = plant.wingR;
tailL = plant.tailL;
tailR = plant.tailR;

wingL.init = surfaceLocalRad(1);
wingR.init = surfaceLocalRad(2);
tailL.init = surfaceLocalRad(3);
tailR.init = surfaceLocalRad(4);

prop = plant.prop;
prop.rotor.init = zeros(4, 1);
prop.tilt.init = zeros(2, 1);

simIn = Simulink.SimulationInput('Trim_Plant');
simIn = simIn.setVariable('prop', prop);
simIn = simIn.setVariable('wingL', wingL);
simIn = simIn.setVariable('wingR', wingR);
simIn = simIn.setVariable('tailL', tailL);
simIn = simIn.setVariable('tailR', tailR);
simIn = simIn.setVariable('CG', plant.CG);
simIn = simIn.setVariable('wingPolar', plant.wingPolar);
simIn = simIn.setVariable('tailPolar', plant.tailPolar);
simIn = simIn.setVariable('V_init', velocityBody);
simIn = simIn.setVariable('eul_init', zeros(3, 1));
simIn = simIn.setVariable('omega_init', zeros(3, 1));
simIn = simIn.setExternalInput(localMakeInputDataset(surfaceLocalRad, stopTime_s));
simIn = simIn.setModelParameter( ...
    'StopTime', num2str(stopTime_s), ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout');

simOut = sim(simIn);
end

function ds = localMakeInputDataset(surfaceLocalRad, stopTime_s)
t = [0.0; stopTime_s];
surface = surfaceLocalRad(:).';

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(zeros(numel(t), 4), t), 'Motor_RPM_cmd');
ds = ds.addElement(timeseries(zeros(numel(t), 2), t), 'Tilt_angles_cmd');
ds = ds.addElement(timeseries(zeros(numel(t), 1), t), 'Front_RPM_collective');
ds = ds.addElement(timeseries(zeros(numel(t), 1), t), 'Rear_RPM_collective');
ds = ds.addElement(timeseries(surface(1) * ones(numel(t), 1), t), 'delta_LW');
ds = ds.addElement(timeseries(surface(2) * ones(numel(t), 1), t), 'delta_RW');
ds = ds.addElement(timeseries(surface(3) * ones(numel(t), 1), t), 'delta_LT');
ds = ds.addElement(timeseries(surface(4) * ones(numel(t), 1), t), 'delta_RT');
ds = ds.addElement(timeseries(zeros(numel(t), 1), t), 'delta_f');
ds = ds.addElement(timeseries(zeros(numel(t), 1), t), 'delta_a');
ds = ds.addElement(timeseries(zeros(numel(t), 1), t), 'delta_e');
ds = ds.addElement(timeseries(zeros(numel(t), 1), t), 'delta_r');
end

function vec = localReadLoggedVector(simOut, signalName)
sig = simOut.logsout.get(signalName);
if isempty(sig)
    error('build_indi_surface_effectiveness_map:MissingSignal', ...
        'Did not find logged signal "%s".', signalName);
end

data = sig.Values.Data;
timeValues = sig.Values.Time;
sz = size(data);
timeDim = find(sz == numel(timeValues), 1, 'last');

if isempty(timeDim)
    slice = squeeze(data);
else
    indices = repmat({':'}, 1, ndims(data));
    indices{timeDim} = sz(timeDim);
    slice = squeeze(data(indices{:}));
end

vec = slice(:);
end

function polarSource = localRequirePolarTables(wingPolar, tailPolar, polarDataFile)
if ~localHasPolarTable(wingPolar) || ~localHasPolarTable(tailPolar)
    error('build_indi_surface_effectiveness_map:MissingPolarTables', ...
        ['Generated AVL polar tables are required. Run Init_Main after restoring %s. ', ...
        'Coefficient-generated aircraft_def fallback polars are intentionally disabled.'], ...
        char(string(polarDataFile)));
end

polarSource = "generated AVL polar tables: " + string(polarDataFile);
end

function tf = localHasPolarTable(polar)
tf = isstruct(polar) && isfield(polar, 'alpha_rad') && isfield(polar, 'CL') && ...
    isfield(polar, 'CD') && isfield(polar, 'Cm') && ~isempty(polar.alpha_rad);
end

function localPrintProgress(callIndex, totalCalls, runTimer, everyCalls)
if everyCalls <= 0 || (mod(callIndex, everyCalls) ~= 0 && callIndex ~= totalCalls)
    return;
end

elapsed = toc(runTimer);
rate = elapsed / max(callIndex, 1);
remaining = max(totalCalls - callIndex, 0) * rate;
fprintf('  %5d/%5d calls | elapsed %.1f min | remaining %.1f min | %.3f s/call\n', ...
    callIndex, totalCalls, elapsed / 60.0, remaining / 60.0, rate);
end

function localSaveMap(map, outputPath)
save(outputPath, 'map', '-v7.3');
end

function localWriteSummary(map, outputPath)
fid = fopen(outputPath, 'w');
if fid < 0
    error('build_indi_surface_effectiveness_map:SummaryOpenFailed', ...
        'Unable to write %s.', outputPath);
end
cleanupFile = onCleanup(@() fclose(fid));

fprintf(fid, '# INDI Surface Effectiveness Map\n\n');
fprintf(fid, 'Last reviewed: %s\n\n', char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf(fid, 'Generated with `controllers/builders/build_indi_surface_effectiveness_map.m`.\n\n');
fprintf(fid, '## Output\n\n');
fprintf(fid, '- MAT file: `%s`\n', localRepoRelativePath(map.metadata.output_mat_path));
fprintf(fid, '- Model sampled: `%s`\n', map.metadata.model);
fprintf(fid, '- Polar source: `%s`\n', map.metadata.polar_source);
fprintf(fid, '- Calls completed: `%d`\n', map.metadata.calls_completed);
fprintf(fid, '- Elapsed: `%.1f s` (`%.1f min`)\n', map.metadata.elapsed_s, map.metadata.elapsed_s / 60.0);
fprintf(fid, '- Seconds per call: `%.4f`\n\n', map.metadata.seconds_per_call);
fprintf(fid, '## Grid\n\n');
fprintf(fid, '- `Vinf_mps`: `%s`\n', mat2str(map.grid.vinf_mps.'));
fprintf(fid, '- `alpha_deg`: `%s`\n', mat2str(map.grid.alpha_deg.'));
fprintf(fid, '- `delta_deg`: `%s`\n', mat2str(map.grid.delta_deg.'));
fprintf(fid, '- Perturbation: `%.3f deg`\n', map.metadata.perturbation_deg);
fprintf(fid, '- Stop time per sample: `%.4f s`\n\n', map.metadata.stop_time_s);
fprintf(fid, '## Stored Fields\n\n');
fprintf(fid, '- `map.flap.dF_drad_N_per_rad`: size `[nV nAlpha nDelta 3]`\n');
fprintf(fid, '- `map.flap.dM_drad_Nm_per_rad`: size `[nV nAlpha nDelta 3]`\n');
fprintf(fid, '- `map.elevator.dF_drad_N_per_rad`: size `[nV nAlpha nDelta 3]`\n');
fprintf(fid, '- `map.elevator.dM_drad_Nm_per_rad`: size `[nV nAlpha nDelta 3]`\n\n');
fprintf(fid, '## Notes\n\n');
fprintf(fid, 'Each derivative uses two Trim_Plant samples. Each sample sets wing and tail surface states to the same perturbation and reads individual logged surface loads, so one plus/minus pair produces both flap and elevator derivatives.\n\n');
fprintf(fid, 'Interior points use central differences; edge deflections use one-sided stencils clipped to the sampled deflection limits.\n\n');
fprintf(fid, 'Surface servo initial states and commands are both set to the sampled local deflection so the map represents actual actuator state, not a command transient.\n\n');
fprintf(fid, 'Prop inputs are zeroed so the logged loads are aerodynamic surface loads only.\n');
end

function relPath = localRepoRelativePath(absPath)
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
relPath = strrep(absPath, [repoRoot filesep], '');
relPath = strrep(relPath, filesep, '/');
end
