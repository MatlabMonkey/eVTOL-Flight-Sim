function report = Run_INDI_Qdot_Effectiveness_Audit(pathIndex, opts)
% qdot check

if nargin < 1 || isempty(pathIndex)
    pathIndex = [];
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

repoRoot = fileparts(mfilename('fullpath'));
previousDir = pwd;
cleanupDir = onCleanup(@() cd(previousDir));
cd(repoRoot);

evalin('base', 'Init_Main');
if isempty(opts.builder_opts)
    controllerData = build_indi_transition_controller();
else
    controllerData = build_indi_transition_controller(opts.builder_opts);
end

nPts = localScheduleCount(controllerData);
if isempty(pathIndex)
    pathIndex = nPts;
end
if pathIndex < 1 || pathIndex > nPts
    error('Run_INDI_Qdot_Effectiveness_Audit:BadPathIndex', ...
        'pathIndex must be in [1, %d]. Received %d.', nPts, pathIndex);
end

row = controllerData.schedule_table(pathIndex, :);
scheduledG = controllerData.controller_gain_lqr(1:3, 1:4, pathIndex);
directG = localBuildDirectEffectiveness(row, opts);

scheduledQ = scheduledG(3, :).';
directQ = directG(3, :).';
ratio = directQ ./ scheduledQ;
ratio(abs(scheduledQ) < 1e-12) = NaN;

summary = table( ...
    ["front_rpm2"; "rear_rpm2"; "flap_rad"; "elevator_rad"], ...
    scheduledQ, directQ, directQ - scheduledQ, ratio, ...
    'VariableNames', {'eta_channel', 'scheduled_Gq', 'direct_Gq', ...
    'direct_minus_scheduled', 'direct_over_scheduled'});

report = struct();
report.path_index = pathIndex;
report.path_row = row;
report.scheduled_G = scheduledG;
report.direct_G = directG;
report.summary = summary;
report.opts = opts;

assignin('base', 'indiQdotEffectivenessAudit', report);

fprintf('\n=== INDI qdot Effectiveness Audit ===\n');
fprintf('Path point: %d of %d\n', pathIndex, nPts);
fprintf('Vinf %.3f m/s | alpha %.3f deg | tilt %.3f deg | theta %.3f deg\n', ...
    row.vinf_mps, row.alpha_deg, row.tilt_deg, row.theta_deg);
fprintf('delta_f %.3f deg | delta_e %.3f deg | h %.3f deg\n\n', ...
    row.delta_f_deg, row.delta_e_deg, opts.surface_perturbation_deg);
disp(summary);
fprintf('Units: prop columns are rad/s^2 per RPM^2; surface columns are rad/s^2 per rad.\n');
fprintf('=== INDI qdot Effectiveness Audit Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'surface_perturbation_deg') || isempty(opts.surface_perturbation_deg)
    opts.surface_perturbation_deg = 0.5;
end
if ~isfield(opts, 'surface_limit_deg') || isempty(opts.surface_limit_deg)
    opts.surface_limit_deg = 25.0;
end
if ~isfield(opts, 'stopTime_s') || isempty(opts.stopTime_s)
    opts.stopTime_s = 0.02;
end
if ~isfield(opts, 'builder_opts')
    opts.builder_opts = [];
end
end

function nPts = localScheduleCount(controllerData)
if isfield(controllerData, 'schedule_count') && ~isempty(controllerData.schedule_count)
    nPts = controllerData.schedule_count;
else
    nPts = size(controllerData.controller_trim_cmd, 2);
end
end

function G = localBuildDirectEffectiveness(row, opts)
plant = localLoadPlant();
mass = plant.Mass;
iyy = plant.J(2, 2);

G = zeros(3, 4);
G(:, 1) = localPropColumn(plant.prop, plant.CG, mass, iyy, row.tilt_deg, "front");
G(:, 2) = localPropColumn(plant.prop, plant.CG, mass, iyy, 0.0, "rear");

[flapForce, flapMoment] = localSurfaceDerivative(row, opts, plant, "flap");
[elevForce, elevMoment] = localSurfaceDerivative(row, opts, plant, "elevator");

G(1, 3) = flapForce(1) / mass;
G(2, 3) = flapForce(3) / mass;
G(3, 3) = flapMoment(2) / iyy;
G(1, 4) = elevForce(1) / mass;
G(2, 4) = elevForce(3) / mass;
G(3, 4) = elevMoment(2) / iyy;
end

function plant = localLoadPlant()
plant = struct();
plant.Mass = evalin('base', 'Mass');
plant.J = evalin('base', 'J');
plant.CG = evalin('base', 'CG');
plant.prop = evalin('base', 'prop');
plant.wingL = evalin('base', 'wingL');
plant.wingR = evalin('base', 'wingR');
plant.tailL = evalin('base', 'tailL');
plant.tailR = evalin('base', 'tailR');
plant.wingPolar = evalin('base', 'wingPolar');
plant.tailPolar = evalin('base', 'tailPolar');
end

function col = localPropColumn(prop, cg, mass, iyy, tiltDeg, groupName)
kT = prop.k_Thrust;
if groupName == "front"
    positions = [prop.posFR; prop.posFL];
    forceDir = [sind(tiltDeg); 0.0; -cosd(tiltDeg)];
else
    positions = [prop.posRR; prop.posRL];
    forceDir = [0.0; 0.0; -1.0];
end

dFtotal = zeros(3, 1);
dMtotal = zeros(3, 1);
for i = 1:size(positions, 1)
    r = positions(i, :).'-cg(:);
    dF = kT * forceDir;
    dM = cross(r, dF);
    dFtotal = dFtotal + dF;
    dMtotal = dMtotal + dM;
end

col = [dFtotal(1) / mass; dFtotal(3) / mass; dMtotal(2) / iyy];
end

function [forceDeriv, momentDeriv] = localSurfaceDerivative(row, opts, plant, surfaceName)
h = opts.surface_perturbation_deg;
limit = opts.surface_limit_deg;
if surfaceName == "flap"
    baseDelta = row.delta_f_deg;
else
    baseDelta = row.delta_e_deg;
end

minusDelta = max(baseDelta - h, -limit);
plusDelta = min(baseDelta + h, limit);
denomRad = deg2rad(plusDelta - minusDelta);
if denomRad <= 0
    error('Run_INDI_Qdot_Effectiveness_Audit:BadStencil', ...
        'Degenerate finite-difference stencil for %s.', surfaceName);
end

sampleMinus = localSampleSurfaces(row, opts, plant, surfaceName, minusDelta);
samplePlus = localSampleSurfaces(row, opts, plant, surfaceName, plusDelta);

forceDeriv = (samplePlus.force_N - sampleMinus.force_N) ./ denomRad;
momentDeriv = (samplePlus.moment_Nm - sampleMinus.moment_Nm) ./ denomRad;
end

function sample = localSampleSurfaces(row, opts, plant, surfaceName, perturbedDeltaDeg)
surfaceRad = deg2rad([row.delta_f_deg; row.delta_f_deg; ...
    row.delta_e_deg; row.delta_e_deg]);
if surfaceName == "flap"
    surfaceRad(1:2) = deg2rad(perturbedDeltaDeg);
    signalNames = {'LW_F', 'RW_F', 'LW_M', 'RW_M'};
else
    surfaceRad(3:4) = deg2rad(perturbedDeltaDeg);
    signalNames = {'LT_F', 'RT_F', 'LT_M', 'RT_M'};
end

velocityBody = [row.vinf_mps * cosd(row.alpha_deg); ...
    0.0; ...
    row.vinf_mps * sind(row.alpha_deg)];

simOut = localRunTrimPlantSample(velocityBody, surfaceRad, opts.stopTime_s, plant);

sample = struct();
sample.force_N = localReadLoggedVector(simOut, signalNames{1}) + ...
    localReadLoggedVector(simOut, signalNames{2});
sample.moment_Nm = localReadLoggedVector(simOut, signalNames{3}) + ...
    localReadLoggedVector(simOut, signalNames{4});
end

function simOut = localRunTrimPlantSample(velocityBody, surfaceRad, stopTime_s, plant)
wingL = plant.wingL;
wingR = plant.wingR;
tailL = plant.tailL;
tailR = plant.tailR;

wingL.init = surfaceRad(1);
wingR.init = surfaceRad(2);
tailL.init = surfaceRad(3);
tailR.init = surfaceRad(4);

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
simIn = simIn.setExternalInput(localMakeInputDataset(surfaceRad, stopTime_s));
simIn = simIn.setModelParameter( ...
    'StopTime', num2str(stopTime_s), ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout');

simOut = sim(simIn);
end

function ds = localMakeInputDataset(surfaceRad, stopTime_s)
t = [0.0; stopTime_s];
surface = surfaceRad(:).';

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
    error('Run_INDI_Qdot_Effectiveness_Audit:MissingSignal', ...
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
