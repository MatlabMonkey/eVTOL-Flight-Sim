function result = benchmark_trim_plant_surface_sampling(sampleCount, opts)
%BENCHMARK_TRIM_PLANT_SURFACE_SAMPLING Time Trim_Plant aero-surface samples.
%
% This is a small feasibility check for building INDI surface-effectiveness
% maps from the Simulink trim plant instead of only from a copied aero
% equation. Each sample seeds the surface servo states at the tested local
% deflection, so the logged loads correspond to actual actuator state.

if nargin < 1 || isempty(sampleCount)
    sampleCount = 100;
end
if nargin < 2 || isempty(opts)
    opts = struct();
end

opts = localApplyDefaults(opts);

repoRoot = fileparts(fileparts(mfilename('fullpath')));
previousDir = pwd;
cleanupDir = onCleanup(@() cd(previousDir));
cd(repoRoot);

evalin('base', 'Init_Main');

prop = evalin('base', 'prop');
wingL = evalin('base', 'wingL');
wingR = evalin('base', 'wingR');
tailL = evalin('base', 'tailL');
tailR = evalin('base', 'tailR');
CG = evalin('base', 'CG');
wingPolar = evalin('base', 'wingPolar');
tailPolar = evalin('base', 'tailPolar');
polarDataFile = evalin('base', 'polar_data_file');

polarSource = localRequirePolarTables(wingPolar, tailPolar, polarDataFile);

load_system('Trim_Plant');
localEnableNamedSignalLogging('Trim_Plant', {'aero_force_sum', 'aero_moment_sum'});

samples = localBuildSamples(sampleCount, opts);
forces = zeros(sampleCount, 3);
moments = zeros(sampleCount, 3);

elapsedTimer = tic;
for k = 1:sampleCount
    simOut = localRunOneSample(samples(k), opts.stopTime_s, ...
        prop, wingL, wingR, tailL, tailR, CG, wingPolar, tailPolar);
    forces(k, :) = localReadLoggedVector(simOut, 'aero_force_sum').';
    moments(k, :) = localReadLoggedVector(simOut, 'aero_moment_sum').';
end
elapsed_s = toc(elapsedTimer);

result = struct();
result.sample_count = sampleCount;
result.stop_time_s = opts.stopTime_s;
result.elapsed_s = elapsed_s;
result.samples_per_second = sampleCount / max(elapsed_s, eps);
result.seconds_per_sample = elapsed_s / max(sampleCount, 1);
result.samples = samples;
result.aero_force_N = forces;
result.aero_moment_Nm = moments;
result.polar_source = polarSource;
result.notes = [ ...
    "Surface initial conditions are set to the tested local deflections."; ...
    "Prop inputs are zeroed so logged loads are aerodynamic surface loads."; ...
    "This benchmark measures Simulink setup+execution overhead, not only aero math." ...
    ];

assignin('base', 'indi_surface_sampling_benchmark', result);

fprintf('\nINDI surface sampling benchmark:\n');
fprintf('  samples          : %d\n', result.sample_count);
fprintf('  Trim_Plant stop  : %.4f s\n', result.stop_time_s);
fprintf('  elapsed          : %.3f s\n', result.elapsed_s);
fprintf('  seconds/sample   : %.4f s\n', result.seconds_per_sample);
fprintf('  samples/second   : %.2f\n\n', result.samples_per_second);

if opts.saveResult
    outDir = fullfile(repoRoot, 'workspace_plots');
    if exist(outDir, 'dir') ~= 7
        mkdir(outDir);
    end
    outPath = fullfile(outDir, 'indi_surface_sampling_benchmark.mat');
    save(outPath, 'result');
    fprintf('Saved benchmark result: %s\n', outPath);
end
end

function polarSource = localRequirePolarTables(wingPolar, tailPolar, polarDataFile)
if ~localHasPolarTable(wingPolar) || ~localHasPolarTable(tailPolar)
    error('benchmark_trim_plant_surface_sampling:MissingPolarTables', ...
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

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'stopTime_s') || isempty(opts.stopTime_s)
    opts.stopTime_s = 0.02;
end
if ~isfield(opts, 'saveResult') || isempty(opts.saveResult)
    opts.saveResult = true;
end
if ~isfield(opts, 'vinfRange_mps') || isempty(opts.vinfRange_mps)
    opts.vinfRange_mps = [2.5, 80.0];
end
if ~isfield(opts, 'alphaRange_deg') || isempty(opts.alphaRange_deg)
    opts.alphaRange_deg = [-10.0, 45.0];
end
if ~isfield(opts, 'deltaRange_deg') || isempty(opts.deltaRange_deg)
    opts.deltaRange_deg = [-25.0, 25.0];
end
end

function samples = localBuildSamples(sampleCount, opts)
samples = repmat(struct( ...
    'vinf_mps', 0.0, ...
    'alpha_deg', 0.0, ...
    'delta_f_deg', 0.0, ...
    'delta_e_deg', 0.0, ...
    'velocity_body_mps', zeros(3, 1), ...
    'omega_body_radps', zeros(3, 1), ...
    'surface_local_rad', zeros(4, 1)), sampleCount, 1);

if sampleCount <= 0
    return;
end

phase = linspace(0.0, 1.0, sampleCount).';
vinf = opts.vinfRange_mps(1) + diff(opts.vinfRange_mps) .* phase;
alpha = opts.alphaRange_deg(1) + diff(opts.alphaRange_deg) .* mod(3.0 * phase, 1.0);
deltaF = opts.deltaRange_deg(1) + diff(opts.deltaRange_deg) .* mod(5.0 * phase, 1.0);
deltaE = opts.deltaRange_deg(2) - diff(opts.deltaRange_deg) .* mod(7.0 * phase, 1.0);

for k = 1:sampleCount
    velocityBody = [ ...
        vinf(k) * cosd(alpha(k)); ...
        0.0; ...
        vinf(k) * sind(alpha(k))];
    localSurfaceRad = deg2rad([deltaF(k); deltaF(k); deltaE(k); deltaE(k)]);

    samples(k).vinf_mps = vinf(k);
    samples(k).alpha_deg = alpha(k);
    samples(k).delta_f_deg = deltaF(k);
    samples(k).delta_e_deg = deltaE(k);
    samples(k).velocity_body_mps = velocityBody;
    samples(k).omega_body_radps = zeros(3, 1);
    samples(k).surface_local_rad = localSurfaceRad;
end
end

function simOut = localRunOneSample(sample, stopTime_s, ...
    propBase, wingLBase, wingRBase, tailLBase, tailRBase, CG, wingPolar, tailPolar)

wingL = wingLBase;
wingR = wingRBase;
tailL = tailLBase;
tailR = tailRBase;

wingL.init = sample.surface_local_rad(1);
wingR.init = sample.surface_local_rad(2);
tailL.init = sample.surface_local_rad(3);
tailR.init = sample.surface_local_rad(4);

prop = propBase;
prop.rotor.init = zeros(4, 1);
prop.tilt.init = zeros(2, 1);

simIn = Simulink.SimulationInput('Trim_Plant');
simIn = simIn.setVariable('prop', prop);
simIn = simIn.setVariable('wingL', wingL);
simIn = simIn.setVariable('wingR', wingR);
simIn = simIn.setVariable('tailL', tailL);
simIn = simIn.setVariable('tailR', tailR);
simIn = simIn.setVariable('CG', CG);
simIn = simIn.setVariable('wingPolar', wingPolar);
simIn = simIn.setVariable('tailPolar', tailPolar);
simIn = simIn.setVariable('V_init', sample.velocity_body_mps);
simIn = simIn.setVariable('eul_init', zeros(3, 1));
simIn = simIn.setVariable('omega_init', sample.omega_body_radps);
simIn = simIn.setExternalInput(localMakeInputDataset(sample, stopTime_s));
simIn = simIn.setModelParameter( ...
    'StopTime', num2str(stopTime_s), ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout');

simOut = sim(simIn);
end

function ds = localMakeInputDataset(sample, stopTime_s)
t = [0.0; stopTime_s];
surface = sample.surface_local_rad(:).';

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
    error('benchmark_trim_plant_surface_sampling:MissingSignal', ...
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

function localEnableNamedSignalLogging(modelName, signalNames)
lines = find_system(modelName, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(lines)
    lineName = get_param(lines(k), 'Name');
    if ismember(lineName, signalNames)
        try
            set_param(lines(k), 'DataLogging', 'on');
        catch
        end
    end
end
end
