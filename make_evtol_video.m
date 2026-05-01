function videoInfo = make_evtol_video(varargin)
% make video

p = inputParser;
p.addParameter('sim_out', [], @(x) isempty(x) || isstruct(x) || isobject(x));
p.addParameter('sim_out_var', 'out', @(x) ischar(x) || (isstring(x) && isscalar(x)));
p.addParameter('video_name', '', @(x) ischar(x) || (isstring(x) && isscalar(x)));
p.addParameter('output_file', '', @(x) ischar(x) || (isstring(x) && isscalar(x)));
p.addParameter('save_to_report', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
p.addParameter('fps', 20, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && (x > 0));
p.addParameter('playback_speed', 1, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && (x > 0));
p.addParameter('t_start', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x)));
p.addParameter('t_end', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x)));
p.addParameter('show_aero_normals', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_trajectory', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('save_trajectory_plot', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opts = p.Results;

simOut = localResolveWorkspaceValue(opts.sim_out, opts.sim_out_var, 'sim_out');

aircraft = aircraft_def('flight_mode', 0);
truthData = localParseTruthFromSimOut(simOut);
internalData = localParseInternalFromSimOut(simOut);
frameData = localBuildFrameData(truthData, internalData, opts, aircraft.prop);
outputFile = localResolveOutputFile(opts.output_file, opts.video_name, simOut, logical(opts.save_to_report));
outputDir = fileparts(outputFile);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

scene = struct();
writer = [];
writerOpened = false;
trajectoryFile = '';
numFrames = numel(frameData.time);
progressTimer = tic;
progressStride = max(1, ceil(numFrames / 40));

fprintf('Rendering eVTOL video to %s\n', outputFile);
fprintf('  Frames: %d at %.2f fps\n', numFrames, opts.fps);
fprintf('  Playback speed: %.2fx real time\n', opts.playback_speed);
fprintf('  Time window: [%.3f, %.3f] s\n', frameData.time(1), frameData.time(end));
localPrintProgress(0, numFrames, 0);

try
    scene = localCreateScene(aircraft, frameData, opts);
    writer = VideoWriter(outputFile, 'MPEG-4');
    writer.FrameRate = opts.fps;
    open(writer);
    writerOpened = true;

    for idx = 1:numFrames
        scene = localUpdateScene(scene, aircraft, frameData, idx, opts);
        drawnow;
        rgbFrame = print(scene.fig, '-RGBImage');
        rgbFrame = localMakeEvenFrame(rgbFrame);
        writeVideo(writer, rgbFrame);

        if idx == 1 || idx == numFrames || mod(idx, progressStride) == 0
            localPrintProgress(idx, numFrames, toc(progressTimer));
        end
    end

    close(writer);
    writerOpened = false;
    fprintf('  Render complete in %s.\n', localFormatDuration(toc(progressTimer)));

    if logical(opts.save_trajectory_plot)
        trajectoryFile = localSaveTrajectoryPlot(frameData, outputFile);
        fprintf('  Trajectory plot: %s\n', trajectoryFile);
    end

    if isfield(scene, 'fig') && isgraphics(scene.fig)
        close(scene.fig);
    end
catch ME
    if writerOpened
        close(writer);
    end
    if isfield(scene, 'fig') && isgraphics(scene.fig)
        close(scene.fig);
    end
    rethrow(ME);
end

videoInfo = struct();
videoInfo.output_file = outputFile;
videoInfo.frame_count = numel(frameData.time);
videoInfo.fps = opts.fps;
videoInfo.playback_speed = opts.playback_speed;
videoInfo.t_start = frameData.time(1);
videoInfo.t_end = frameData.time(end);
videoInfo.duration_s = frameData.time(end) - frameData.time(1);
videoInfo.video_duration_s = numel(frameData.time) / opts.fps;
videoInfo.trajectory_plot_file = trajectoryFile;
end

function value = localResolveWorkspaceValue(directValue, varName, label)
if ~isempty(directValue)
    value = directValue;
else
    varName = char(string(varName));
    existsInBase = evalin('base', sprintf('exist(''%s'', ''var'')', varName));
    if ~existsInBase
        error('make_evtol_video:MissingInput', ...
            'Could not find %s in the base workspace as variable "%s".', label, varName);
    end
    value = evalin('base', varName);
end
end

function truth = localParseTruthFromSimOut(simOut)
if isempty(simOut)
    error('make_evtol_video:MissingInput', 'sim_out must not be empty.');
end

posNed = localGetSimOutField(simOut, 'pos_NED');
eulTruth = localGetSimOutField(simOut, 'eul_truth');
vBodyTruth = localGetSimOutField(simOut, 'V_B_truth');
omegaTruth = localGetSimOutField(simOut, 'omega_truth');

vizTruth = localCombineStructWithTime( ...
    {posNed, eulTruth, vBodyTruth, omegaTruth}, ...
    {'pos_NED', 'eul_truth', 'V_B_truth', 'omega_truth'}, ...
    [3, 3, 3, 3], ...
    'truth series');

truth = localParseVizTruth(vizTruth);
end

function internal = localParseInternalFromSimOut(simOut)
actuatorState = localTryGetSimOutField(simOut, 'actuator_state_truth');
if isempty(actuatorState)
    actuatorState = localTryGetSimOutField(simOut, 'actuator_state_meas');
end
if ~isempty(actuatorState)
    internal = localParseActuatorStateInternal(actuatorState);
    return;
end

tiltStates = localGetSimOutField(simOut, 'tilt_angles_cmd');
frontCollective = localGetSimOutField(simOut, 'front_collective_rpm_out');
rearCollective = localGetSimOutField(simOut, 'rear_collective_rpm_out');
deltaLW = localGetSimOutField(simOut, 'deltaLW_cmd');
deltaRW = localGetSimOutField(simOut, 'deltaRW_cmd');
deltaLT = localGetSimOutField(simOut, 'deltaLT_cmd');
deltaRT = localGetSimOutField(simOut, 'deltaRT_cmd');

vizInternal = localCombineStructWithTime( ...
    {tiltStates, frontCollective, rearCollective, deltaLW, deltaRW, deltaLT, deltaRT}, ...
    {'tilt_angles_cmd', 'front_collective_rpm_out', 'rear_collective_rpm_out', ...
        'deltaLW_cmd', 'deltaRW_cmd', 'deltaLT_cmd', 'deltaRT_cmd'}, ...
    [2, 1, 1, 1, 1, 1, 1], ...
    'internal series');

internal = localParseVizInternal(vizInternal);
end

function value = localGetSimOutField(simOut, fieldName)
value = localTryGetSimOutField(simOut, fieldName);
if isempty(value)
    error('make_evtol_video:MissingSimOutField', ...
        'sim_out does not contain the required field "%s".', fieldName);
end
end

function value = localTryGetSimOutField(simOut, fieldName)
value = [];

if isstruct(simOut) && isfield(simOut, fieldName)
    value = simOut.(fieldName);
    return;
end

if isstruct(simOut) && isfield(simOut, 'out') && ...
        isstruct(simOut.out) && isfield(simOut.out, fieldName)
    value = simOut.out.(fieldName);
    return;
end

try
    value = simOut.get(fieldName);
catch
end

if isempty(value)
    try
        value = simOut.get(['out.' fieldName]);
    catch
    end
end

if isempty(value)
    try
        outStruct = simOut.get('out');
        if isstruct(outStruct) && isfield(outStruct, fieldName)
            value = outStruct.(fieldName);
            return;
        end
    catch
    end
end

if isempty(value)
    try
        value = simOut.(fieldName);
    catch
    end
end
end

function combined = localCombineStructWithTime(parts, labels, widths, combinedLabel)
series = cell(size(parts));
starts = zeros(numel(parts), 1);
ends = zeros(numel(parts), 1);

for idx = 1:numel(parts)
    series{idx} = localPrepareStructSeries(parts{idx}, labels{idx}, widths(idx));
    starts(idx) = series{idx}.time(1);
    ends(idx) = series{idx}.time(end);
end

tStart = max(starts);
tEnd = min(ends);
if tEnd < tStart
    error('make_evtol_video:NoOverlap', ...
        '%s inputs do not overlap in time.', combinedLabel);
end

timeBase = series{1}.time;
timeBase = timeBase(timeBase >= tStart & timeBase <= tEnd);
if isempty(timeBase)
    timeBase = [tStart; tEnd];
    timeBase = unique(timeBase(:));
end

dataParts = cell(size(parts));
for idx = 1:numel(parts)
    dataParts{idx} = localInterpSeries(series{idx}.time, series{idx}.data, timeBase, labels{idx});
end

combined = struct();
combined.time = timeBase(:);
combined.signals = struct('values', [dataParts{:}]);
end

function series = localPrepareStructSeries(s, label, expectedWidth)
[time, values] = localExtractLoggedTimeAndValues(s, label);
time = localPrepareTimeVector(time, label);
values = localPrepareLoggedValueMatrix(values, numel(time), expectedWidth, label);
if size(values, 2) ~= expectedWidth
    error('make_evtol_video:BadInputLayout', ...
        '%s must have exactly %d channels after reshaping; got %d.', ...
        label, expectedWidth, size(values, 2));
end
series = localPrepareSampleSeries(time, values, label);
end

function values = localPrepareLoggedValueMatrix(values, sampleCount, expectedWidth, label)
if ~isnumeric(values) || isempty(values)
    error('make_evtol_video:EmptyValues', '%s.signals.values must be a non-empty numeric array.', label);
end

values = double(values);
dims = size(values);

if ismatrix(values)
    if size(values, 1) == sampleCount
        return;
    end
    if size(values, 2) == sampleCount && size(values, 1) == expectedWidth
        values = values.';
        return;
    end

    if numel(values) == sampleCount * expectedWidth
        values = reshape(values, sampleCount, expectedWidth);
        return;
    end

    error('make_evtol_video:BadInputLayout', ...
        '%s could not be reshaped to [%d x %d] from size [%s].', ...
        label, sampleCount, expectedWidth, num2str(dims));
end

matchingDims = find(dims == sampleCount);
for idx = 1:numel(matchingDims)
    timeDim = matchingDims(idx);
    perm = [timeDim, 1:timeDim-1, timeDim+1:ndims(values)];
    candidate = permute(values, perm);
    candidate = reshape(candidate, sampleCount, []);
    if size(candidate, 2) == expectedWidth
        values = candidate;
        return;
    end
end

matchingDims = find(dims == expectedWidth);
for idx = 1:numel(matchingDims)
    channelDim = matchingDims(idx);
    remainingDims = dims;
    remainingDims(channelDim) = [];
    if prod(remainingDims) ~= sampleCount
        continue;
    end

    perm = [setdiff(1:ndims(values), channelDim, 'stable'), channelDim];
    candidate = permute(values, perm);
    candidate = reshape(candidate, sampleCount, expectedWidth);
    values = candidate;
    return;
end

if numel(values) == sampleCount * expectedWidth
    values = reshape(values, sampleCount, expectedWidth);
    return;
end

error('make_evtol_video:BadInputLayout', ...
    '%s could not be reshaped to [%d x %d] from size [%s].', ...
    label, sampleCount, expectedWidth, num2str(dims));
end

function truth = localParseVizTruth(s)
if isa(s, 'timeseries')
    error('make_evtol_video:BadVizTruthLayout', ...
        'viz_truth must be a combined struct-like packet, not a raw timeseries.');
end
[time, values] = localExtractLoggedTimeAndValues(s, 'viz_truth');
if size(values, 2) ~= 12
    error('make_evtol_video:BadVizTruthLayout', ...
        'viz_truth must have exactly 12 channels: [pos_NED(3), eul_truth(3), V_B_truth(3), omega_truth(3)].');
end

truth = struct();
truth.time = localPrepareTimeVector(time, 'viz_truth');
truth.values = localPrepareValueMatrix(values, numel(truth.time), 'viz_truth');
truth.values = localPrepareSampleSeries(truth.time, truth.values, 'viz_truth');
truth.time = truth.values.time;
truth.values = truth.values.data;
truth.pos_NED = truth.values(:, 1:3);
truth.eul_truth = truth.values(:, 4:6);
truth.V_B_truth = truth.values(:, 7:9);
truth.omega_truth = truth.values(:, 10:12);
end

function internal = localParseVizInternal(s)
if isa(s, 'timeseries')
    error('make_evtol_video:BadVizInternalLayout', ...
        'viz_internal must be a combined struct-like packet, not a raw timeseries.');
end
[time, values] = localExtractLoggedTimeAndValues(s, 'viz_internal');
if size(values, 2) ~= 8
    error('make_evtol_video:BadVizInternalLayout', ...
        ['viz_internal must have exactly 8 channels: ' ...
         '[tilt_FR, tilt_FL, front_rpm, rear_rpm, deltaLW, deltaRW, deltaLT, deltaRT].']);
end

internal = struct();
internal.time = localPrepareTimeVector(time, 'viz_internal');
internal.values = localPrepareValueMatrix(values, numel(internal.time), 'viz_internal');
internal.values = localPrepareSampleSeries(internal.time, internal.values, 'viz_internal');
internal.time = internal.values.time;
internal.values = internal.values.data;

internal.tilt_FR = internal.values(:, 1);
internal.tilt_FL = internal.values(:, 2);
frontGroupRpm = internal.values(:, 3);
rearGroupRpm = internal.values(:, 4);
internal.deltaLW = internal.values(:, 5);
internal.deltaRW = internal.values(:, 6);
internal.deltaLT = internal.values(:, 7);
internal.deltaRT = internal.values(:, 8);
internal.group_rpm = [frontGroupRpm, frontGroupRpm, rearGroupRpm, rearGroupRpm];
end

function internal = localParseActuatorStateInternal(s)
series = localPrepareStructSeries(s, 'actuator_state_truth/meas', 10);

internal = struct();
internal.time = series.time;
internal.values = series.data;
internal.source = 'actuator_state';
internal.group_rpm = internal.values(:, 1:4);
internal.tilt_FR = internal.values(:, 5);
internal.tilt_FL = internal.values(:, 6);
internal.deltaLW = internal.values(:, 7);
internal.deltaRW = internal.values(:, 8);
internal.deltaLT = internal.values(:, 9);
internal.deltaRT = internal.values(:, 10);
end

function [time, values] = localExtractLoggedTimeAndValues(s, label)
if isa(s, 'timeseries')
    time = s.Time;
    values = s.Data;
    return;
end

if isstruct(s) && isfield(s, 'time') && isfield(s, 'signals') && ...
        isstruct(s.signals) && isfield(s.signals, 'values')
    time = s.time;
    values = s.signals.values;
    return;
end

error('make_evtol_video:UnsupportedFormat', ...
    '%s must be either a Simulink \"Structure With Time\" value or a timeseries.', label);
end

function t = localPrepareTimeVector(t, label)
if ~isnumeric(t) || isempty(t)
    error('make_evtol_video:EmptyTime', '%s.time must be a non-empty numeric vector.', label);
end
if ~isvector(t)
    error('make_evtol_video:BadTimeShape', '%s.time must be a vector.', label);
end
if any(~isfinite(t))
    error('make_evtol_video:BadTimeValues', '%s.time contains non-finite values.', label);
end
t = double(t(:));
end

function values = localPrepareValueMatrix(values, sampleCount, label)
if ~isnumeric(values) || isempty(values)
    error('make_evtol_video:EmptyValues', '%s.signals.values must be a non-empty numeric array.', label);
end

values = double(values);
if ~ismatrix(values)
    values = reshape(values, sampleCount, []);
elseif size(values, 1) ~= sampleCount
    if size(values, 2) == sampleCount
        values = values.';
    else
        values = reshape(values, sampleCount, []);
    end
end

if size(values, 1) ~= sampleCount
    error('make_evtol_video:BadValueShape', ...
        '%s.signals.values must have one row per time sample.', label);
end
end

function series = localPrepareSampleSeries(time, data, label)
sortNeeded = any(diff(time) < 0);
if sortNeeded
    [time, sortIdx] = sort(time);
    data = data(sortIdx, :);
end

[timeUnique, uniqueIdx] = unique(time, 'last');
dataUnique = data(uniqueIdx, :);

if isempty(timeUnique)
    error('make_evtol_video:EmptySeries', '%s has no usable samples.', label);
end

series = struct('time', timeUnique(:), 'data', dataUnique);
end

function frameData = localBuildFrameData(truth, internal, opts, prop)
tStart = max(truth.time(1), internal.time(1));
tEnd = min(truth.time(end), internal.time(end));

if ~isempty(opts.t_start)
    tStart = max(tStart, opts.t_start);
end
if ~isempty(opts.t_end)
    tEnd = min(tEnd, opts.t_end);
end

if tEnd < tStart
    error('make_evtol_video:NoOverlap', ...
        'viz_truth and viz_internal do not overlap in time over the requested window.');
end

if tEnd == tStart
    frameTime = tStart;
else
    dt = opts.playback_speed / opts.fps;
    frameTime = (tStart:dt:tEnd).';
    if isempty(frameTime) || frameTime(end) < tEnd
        frameTime(end + 1, 1) = tEnd;
    end
end

eulUnwrapped = unwrap(truth.eul_truth);

frameData = struct();
frameData.time = frameTime;
frameData.body_eul = localInterpSeries(truth.time, eulUnwrapped, frameTime, 'body Euler history');
frameData.tilt_FR = localInterpSeries(internal.time, internal.tilt_FR, frameTime, 'tilt_FR');
frameData.tilt_FL = localInterpSeries(internal.time, internal.tilt_FL, frameTime, 'tilt_FL');
frameData.deltaLW = localInterpSeries(internal.time, internal.deltaLW, frameTime, 'deltaLW');
frameData.deltaRW = localInterpSeries(internal.time, internal.deltaRW, frameTime, 'deltaRW');
frameData.deltaLT = localInterpSeries(internal.time, internal.deltaLT, frameTime, 'deltaLT');
frameData.deltaRT = localInterpSeries(internal.time, internal.deltaRT, frameTime, 'deltaRT');
frameData.pos_NED = localInterpSeries(truth.time, truth.pos_NED, frameTime, 'pos_NED');
frameData.V_B_truth = localInterpSeries(truth.time, truth.V_B_truth, frameTime, 'V_B_truth');
frameData.omega_truth = localInterpSeries(truth.time, truth.omega_truth, frameTime, 'omega_truth');
frameData.group_rpm = localInterpSeries(internal.time, internal.group_rpm, frameTime, 'group RPM history');
frameData.group_rpm = max(frameData.group_rpm, 0);
frameData.rotor_spin = localBuildRotorSpinHistory(frameTime, frameData.group_rpm, prop);
frameData.F_FR = localGroupForceFromRpm(frameData.group_rpm(:, 1), frameData.tilt_FR, true, prop);
frameData.F_FL = localGroupForceFromRpm(frameData.group_rpm(:, 2), frameData.tilt_FL, true, prop);
frameData.F_RR = localGroupForceFromRpm(frameData.group_rpm(:, 3), zeros(size(frameTime)), false, prop);
frameData.F_RL = localGroupForceFromRpm(frameData.group_rpm(:, 4), zeros(size(frameTime)), false, prop);
end

function yq = localInterpSeries(t, y, tq, label)
if size(y, 1) ~= numel(t)
    error('make_evtol_video:BadInterpolationInput', ...
        'Signal "%s" does not have one row per time sample.', label);
end

if isscalar(t)
    yq = repmat(y, numel(tq), 1);
    return;
end

yq = interp1(t, y, tq, 'linear');

if any(~isfinite(yq(:)))
    error('make_evtol_video:InterpolationFailed', ...
        'Could not interpolate signal "%s" over the requested frame times.', label);
end
end

function outputFile = localResolveOutputFile(outputFile, videoName, simOut, saveToReport)
outputFile = char(string(outputFile));
videoName = char(string(videoName));

if isempty(strtrim(outputFile))
    if saveToReport
        folderPath = localReportPlotsDir();
    else
        folderPath = fullfile(pwd, 'workspace_plots');
    end
    if isempty(strtrim(videoName))
        baseName = localDefaultVideoBaseName(simOut);
    else
        baseName = localSanitizeFileStem(videoName);
    end

    if isempty(baseName)
        baseName = 'evtol_state_video';
    end

    outputFile = fullfile(folderPath, [baseName '.mp4']);
    outputFile = localMakeUniqueOutputFile(outputFile);
else
    outputFile = localNormalizeOutputFile(outputFile, saveToReport);
end
end

function outputFile = localNormalizeOutputFile(outputFile, saveToReport)
outputFile = char(string(outputFile));
if isempty(strtrim(outputFile))
    error('make_evtol_video:BadOutputFile', 'output_file must not be empty.');
end

[folderPath, baseName, ext] = fileparts(outputFile);
if isempty(folderPath)
    if saveToReport
        folderPath = localReportPlotsDir();
    else
        folderPath = pwd;
    end
end
if isempty(ext)
    ext = '.mp4';
end
if ~strcmpi(ext, '.mp4')
    error('make_evtol_video:BadOutputExtension', 'output_file must use the .mp4 extension.');
end

outputFile = fullfile(folderPath, [baseName ext]);
end

function folderPath = localReportPlotsDir()
folderPath = fullfile(fileparts(mfilename('fullpath')), 'report_plots_final');
end

function baseName = localDefaultVideoBaseName(simOut)
candidateNames = { ...
    localWorkspaceStructFieldText('runSpec', 'name'), ...
    localWorkspaceStructFieldText('runResult', 'name'), ...
    localWorkspaceStructFieldText('runCase', 'name'), ...
    localWorkspaceStructFieldText('scenario', 'name'), ...
    localWorkspaceTextValue('scenario_name'), ...
    localSimOutMetadataName(simOut)};

for idx = 1:numel(candidateNames)
    candidate = localSanitizeFileStem(candidateNames{idx});
    if ~isempty(candidate)
        baseName = [candidate '_video'];
        return;
    end
end

baseName = 'evtol_state_video';
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

function txt = localSimOutMetadataName(simOut)
txt = '';
if isempty(simOut)
    return;
end

try
    metadata = simOut.SimulationMetadata;
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

function stem = localSanitizeFileStem(name)
stem = localAsTextScalar(name);
if isempty(stem)
    return;
end

stem = regexprep(stem, '[^A-Za-z0-9_-]+', '_');
stem = regexprep(stem, '_+', '_');
stem = regexprep(stem, '^_+|_+$', '');
end

function outputFile = localMakeUniqueOutputFile(outputFile)
[folderPath, baseName, ext] = fileparts(outputFile);
candidate = fullfile(folderPath, [baseName ext]);
if ~exist(candidate, 'file')
    outputFile = candidate;
    return;
end

suffix = 2;
while true
    candidate = fullfile(folderPath, sprintf('%s_%02d%s', baseName, suffix, ext));
    if ~exist(candidate, 'file')
        outputFile = candidate;
        return;
    end
    suffix = suffix + 1;
end
end

function scene = localCreateScene(aircraft, frameData, opts)
scene = struct();
scene.radius = localComputeSceneRadius(aircraft);
scene.viewRadius = max(4.0, 0.82 * scene.radius);
scene.arrowScale = localComputeArrowScale(aircraft.prop, scene.radius);
scene.velocityScale = localComputeVelocityScale(scene.radius);
scene.bodyAxisLength = 0.19 * scene.radius;
scene.arrowLabelOffset = 0.06 * scene.viewRadius;
scene.velocityLabelOffset = 0.045 * scene.viewRadius;
scene.cgOverlayOffset = 0.02 * scene.radius;

scene.fig = figure( ...
    'Name', 'eVTOL State Video', ...
    'Color', [0.10 0.10 0.10], ...
    'Position', [80 80 1280 720], ...
    'Visible', 'off');

scene.ax = axes('Parent', scene.fig);
if logical(opts.show_trajectory)
    scene.ax.Position = [0.32 0.04 0.64 0.76];
else
    scene.ax.Position = [0.04 0.04 0.92 0.76];
end
hold(scene.ax, 'on');
axis(scene.ax, 'equal');
axis(scene.ax, 'vis3d');
axis(scene.ax, 'off');
set(scene.ax, ...
    'Color', [0.10 0.10 0.10], ...
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ...
    'ZDir', 'reverse');
xlim(scene.ax, [-scene.viewRadius, scene.viewRadius]);
ylim(scene.ax, [-scene.viewRadius, scene.viewRadius]);
zlim(scene.ax, [-scene.viewRadius, scene.viewRadius]);
view(scene.ax, 130, 25);
camlight(scene.ax, 'right');
lighting(scene.ax, 'flat');

scene.titleBox = annotation(scene.fig, 'textbox', [0.18 0.90 0.64 0.05], ...
    'String', 'Hover to Cruise Transition', ...
    'FitBoxToText', 'off', ...
    'EdgeColor', 'none', ...
    'Color', 'w', ...
    'FontSize', 17, ...
    'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle');
scene.telemetryBox = annotation(scene.fig, 'textbox', [0.02 0.54 0.38 0.34], ...
    'String', '', ...
    'FitBoxToText', 'off', ...
    'EdgeColor', 'none', ...
    'Color', 'w', ...
    'FontSize', 11, ...
    'FontWeight', 'bold', ...
    'FontName', 'Courier', ...
    'Interpreter', 'none', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top');

scene.partTemplates = localBuildPartTemplates(aircraft);
initialState = localFrameState(frameData, 1, aircraft);
scene.parts = scene.partTemplates;

for idx = 1:numel(scene.parts)
    vertices = localComputePartVertices(scene.parts(idx), initialState, aircraft);
    scene.parts(idx).handle = patch( ...
        'Parent', scene.ax, ...
        'Vertices', vertices, ...
        'Faces', scene.parts(idx).faces, ...
        'FaceColor', scene.parts(idx).faceColor, ...
        'FaceAlpha', scene.parts(idx).faceAlpha, ...
        'EdgeColor', 'k', ...
        'LineWidth', 1.0);
end

scene.arrowStyles = localArrowStyles();
[arrowOrigins, arrowVectors] = localRepresentativeArrowSet(initialState, aircraft, scene.arrowScale);
scene.arrowHandles = gobjects(size(arrowOrigins, 1), 1);
scene.arrowLabelHandles = gobjects(size(arrowOrigins, 1), 1);
for idx = 1:size(arrowOrigins, 1)
    scene.arrowHandles(idx) = quiver3(scene.ax, ...
        arrowOrigins(idx, 1), arrowOrigins(idx, 2), arrowOrigins(idx, 3), ...
        arrowVectors(idx, 1), arrowVectors(idx, 2), arrowVectors(idx, 3), 0, ...
        'Color', scene.arrowStyles(idx).color, ...
        'LineWidth', scene.arrowStyles(idx).lineWidth, ...
        'MaxHeadSize', 0.6);

    labelPos = localArrowLabelPosition( ...
        arrowOrigins(idx, :), arrowVectors(idx, :), scene.arrowLabelOffset, scene.arrowStyles(idx).labelOffset);
    scene.arrowLabelHandles(idx) = text(scene.ax, ...
        labelPos(1), labelPos(2), labelPos(3), ...
        scene.arrowStyles(idx).label, ...
        'Color', scene.arrowStyles(idx).color, ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'top', ...
        'Clipping', 'off');
end

scene.aeroHandles = gobjects(0);
scene.aeroScale = 0.16 * scene.radius;
if opts.show_aero_normals
    surfaceData = aircraft.render_surfaces;
    scene.aeroHandles = gobjects(numel(surfaceData), 1);
    for idx = 1:numel(surfaceData)
        [posWorld, dirWorld] = localAeroNormalState(surfaceData(idx), initialState, aircraft);
        scene.aeroHandles(idx) = quiver3(scene.ax, ...
            posWorld(1), posWorld(2), posWorld(3), ...
            scene.aeroScale * dirWorld(1), scene.aeroScale * dirWorld(2), scene.aeroScale * dirWorld(3), 0, ...
            'Color', [1.0 0.4 1.0], 'LineWidth', 1.6, 'MaxHeadSize', 0.5);
    end
end

scene.bodyAxisColors = [ ...
    1.00 0.35 0.35; ...
    0.35 1.00 0.45; ...
    0.45 0.75 1.00];
scene.bodyAxisHandles = gobjects(3, 1);
[bodyAxisOrigin, bodyAxisVectors] = localBodyAxisVectorSet( ...
    scene.ax, initialState, aircraft, scene.bodyAxisLength, scene.cgOverlayOffset);
for idx = 1:3
    scene.bodyAxisHandles(idx) = quiver3(scene.ax, ...
        bodyAxisOrigin(1), bodyAxisOrigin(2), bodyAxisOrigin(3), ...
        bodyAxisVectors(idx, 1), bodyAxisVectors(idx, 2), bodyAxisVectors(idx, 3), 0, ...
        'Color', scene.bodyAxisColors(idx, :), ...
        'LineWidth', 3.0, ...
        'MaxHeadSize', 0.65);
end

[velocityOrigin, velocityVector] = localVelocityVectorSet( ...
    scene.ax, initialState, aircraft, scene.velocityScale, scene.cgOverlayOffset);
scene.velocityHandle = quiver3(scene.ax, ...
    velocityOrigin(1), velocityOrigin(2), velocityOrigin(3), ...
    velocityVector(1), velocityVector(2), velocityVector(3), 0, ...
    'Color', [1.0 1.0 1.0], ...
    'LineWidth', 3.4, ...
    'LineStyle', '-', ...
    'MaxHeadSize', 1.00);
velocityLabelPos = localVelocityLabelPosition(scene.ax, velocityOrigin, velocityVector, scene.velocityLabelOffset);
scene.velocityLabelHandle = text(scene.ax, ...
    velocityLabelPos(1), velocityLabelPos(2), velocityLabelPos(3), ...
    'V_B', ...
    'Color', [1.0 1.0 1.0], ...
    'FontSize', 12, ...
    'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'bottom', ...
    'Clipping', 'off');

scene.trajectory = struct('available', false);
if logical(opts.show_trajectory)
    scene.trajectory = localCreateTrajectoryInset(scene.fig, frameData);
end

scene.telemetryBox.String = localTelemetryString(initialState, aircraft);
end

function scene = localUpdateScene(scene, aircraft, frameData, frameIdx, opts)
state = localFrameState(frameData, frameIdx, aircraft);

for idx = 1:numel(scene.parts)
    vertices = localComputePartVertices(scene.parts(idx), state, aircraft);
    set(scene.parts(idx).handle, 'Vertices', vertices);
end

[arrowOrigins, arrowVectors] = localRepresentativeArrowSet(state, aircraft, scene.arrowScale);
for idx = 1:numel(scene.arrowHandles)
    set(scene.arrowHandles(idx), ...
        'XData', arrowOrigins(idx, 1), ...
        'YData', arrowOrigins(idx, 2), ...
        'ZData', arrowOrigins(idx, 3), ...
        'UData', arrowVectors(idx, 1), ...
        'VData', arrowVectors(idx, 2), ...
        'WData', arrowVectors(idx, 3));

    labelPos = localArrowLabelPosition( ...
        arrowOrigins(idx, :), arrowVectors(idx, :), scene.arrowLabelOffset, scene.arrowStyles(idx).labelOffset);
    set(scene.arrowLabelHandles(idx), ...
        'Position', labelPos, ...
        'String', scene.arrowStyles(idx).label);
end

if opts.show_aero_normals
    surfaceData = aircraft.render_surfaces;
    for idx = 1:numel(surfaceData)
        [posWorld, dirWorld] = localAeroNormalState(surfaceData(idx), state, aircraft);
        set(scene.aeroHandles(idx), ...
            'XData', posWorld(1), 'YData', posWorld(2), 'ZData', posWorld(3), ...
            'UData', scene.aeroScale * dirWorld(1), ...
            'VData', scene.aeroScale * dirWorld(2), ...
            'WData', scene.aeroScale * dirWorld(3));
    end
end

[bodyAxisOrigin, bodyAxisVectors] = localBodyAxisVectorSet( ...
    scene.ax, state, aircraft, scene.bodyAxisLength, scene.cgOverlayOffset);
for idx = 1:3
    set(scene.bodyAxisHandles(idx), ...
        'XData', bodyAxisOrigin(1), ...
        'YData', bodyAxisOrigin(2), ...
        'ZData', bodyAxisOrigin(3), ...
        'UData', bodyAxisVectors(idx, 1), ...
        'VData', bodyAxisVectors(idx, 2), ...
        'WData', bodyAxisVectors(idx, 3));
end

[velocityOrigin, velocityVector] = localVelocityVectorSet( ...
    scene.ax, state, aircraft, scene.velocityScale, scene.cgOverlayOffset);
set(scene.velocityHandle, ...
    'XData', velocityOrigin(1), ...
    'YData', velocityOrigin(2), ...
    'ZData', velocityOrigin(3), ...
    'UData', velocityVector(1), ...
    'VData', velocityVector(2), ...
    'WData', velocityVector(3));
velocityLabelPos = localVelocityLabelPosition(scene.ax, velocityOrigin, velocityVector, scene.velocityLabelOffset);
set(scene.velocityLabelHandle, ...
    'Position', velocityLabelPos, ...
    'String', 'V_B');

scene.telemetryBox.String = localTelemetryString(state, aircraft);
if isfield(scene, 'trajectory') && isfield(scene.trajectory, 'available') && scene.trajectory.available
    scene.trajectory = localUpdateTrajectoryInset(scene.trajectory, frameData, frameIdx);
end
end

function state = localFrameState(frameData, idx, ~)
state = struct();
state.time = frameData.time(idx);
state.body_eul = frameData.body_eul(idx, :).';
state.body_eul_deg = rad2deg(state.body_eul);
state.body_R = localRotationMatrix(state.body_eul);
state.front_tilts_deg = [ ...
    frameData.tilt_FR(idx) * ones(3, 1); ...
    frameData.tilt_FL(idx) * ones(3, 1)];
state.surface = struct( ...
    'deltaLW', frameData.deltaLW(idx), ...
    'deltaRW', frameData.deltaRW(idx), ...
    'deltaLT', frameData.deltaLT(idx), ...
    'deltaRT', frameData.deltaRT(idx));
state.surface_deg = rad2deg([ ...
    state.surface.deltaLW; ...
    state.surface.deltaRW; ...
    state.surface.deltaLT; ...
    state.surface.deltaRT]);
state.V_B_truth = frameData.V_B_truth(idx, :).';
state.V_B_mag = norm(state.V_B_truth);
state.omega_truth = frameData.omega_truth(idx, :).';
state.front_force_body = frameData.F_FR(idx, :).';
state.front_force_left_body = frameData.F_FL(idx, :).';
state.rear_force_body = frameData.F_RR(idx, :).';
state.rear_force_left_body = frameData.F_RL(idx, :).';
state.group_thrust_mag = [ ...
    norm(state.front_force_body); ...
    norm(state.front_force_left_body); ...
    norm(state.rear_force_body); ...
    norm(state.rear_force_left_body)];
state.group_rpm = frameData.group_rpm(idx, :).';
state.rotor_spin = struct( ...
    'FR', frameData.rotor_spin.FR(idx, :), ...
    'FL', frameData.rotor_spin.FL(idx, :), ...
    'RR', frameData.rotor_spin.RR(idx, :), ...
    'RL', frameData.rotor_spin.RL(idx, :));
end

function partTemplates = localBuildPartTemplates(aircraft)
compData = aircraft.compData;
controls = aircraft.controls;
partTemplates = struct([]);

for idx = 1:size(compData, 1)
    name = compData{idx, 1};
    type = compData{idx, 2};
    dim = compData{idx, 4};
    posBody = compData{idx, 5};
    eulBody = deg2rad(compData{idx, 6});
    color = localComponentColor(name, type);

    if strcmp(type, 'box') && contains(name, 'Main Wing')
        fixedPart = localBasePartTemplate(name, 'wing_fixed', dim, posBody, eulBody, color, 0.85);
        fixedPart.controlChordFraction = localWingControlChordFraction(controls);
        controlPart = localBasePartTemplate(name, 'wing_control', dim, posBody, eulBody, [0.85 1.00 0.25], 0.92);
        controlPart.controlChordFraction = fixedPart.controlChordFraction;
        [~, fixedPart.faces, ~, controlPart.faces] = ...
            localBuildWingWithFlaperon(dim, fixedPart.controlChordFraction, 0);
        partTemplates = [partTemplates; fixedPart; controlPart]; %#ok<AGROW>
        continue;
    end

    if strcmp(type, 'box') && contains(name, 'V-Tail')
        fixedPart = localBasePartTemplate(name, 'tail_fixed', dim, posBody, eulBody, color, 0.85);
        fixedPart.controlChordFraction = localTailControlChordFraction(controls);
        controlPart = localBasePartTemplate(name, 'tail_control', dim, posBody, eulBody, [1.0 0.95 0.20], 0.92);
        controlPart.controlChordFraction = fixedPart.controlChordFraction;
        [~, fixedPart.faces, ~, controlPart.faces] = ...
            localBuildTailWithRuddervator(dim, fixedPart.controlChordFraction, 0);
        partTemplates = [partTemplates; fixedPart; controlPart]; %#ok<AGROW>
        continue;
    end

    if strcmp(type, 'fuselage')
        [vertices, faces] = genFacetedFuselage(dim(1), dim(2), dim(3));
    elseif strcmp(type, 'box')
        [vertices, faces] = genCleanBox(dim(1), dim(2), dim(3));
    elseif strcmp(type, 'crossprop')
        [vertices, faces] = genCrossProp(dim(1), dim(2), dim(3));
    else
        continue;
    end

    plainPart = localBasePartTemplate(name, 'plain', dim, posBody, eulBody, ...
        color, localDefaultFaceAlpha(name, type));
    plainPart.baseVertices = vertices;
    plainPart.faces = faces;
    partTemplates = [partTemplates; plainPart]; %#ok<AGROW>
end
end

function part = localBasePartTemplate(name, kind, dim, posBody, eulBody, faceColor, faceAlpha)
part = struct();
part.name = name;
part.kind = kind;
part.dim = dim;
part.posBody = posBody;
part.eulBody = eulBody;
part.faceColor = faceColor;
part.faceAlpha = faceAlpha;
part.controlChordFraction = [];
part.baseVertices = [];
part.faces = [];
part.handle = [];
end

function verticesWorld = localComputePartVertices(part, state, aircraft)
name = part.name;
dim = part.dim;
posBody = part.posBody;
eulBody = part.eulBody;

if contains(name, 'F-Arm')
    rotorTiltDeg = localFrontComponentTilt(name, state.front_tilts_deg, 90);
    pivot = localFrontPivot(name, posBody, aircraft.prop);
    thrustDirBody = localThrustDirection(rotorTiltDeg);
    hubOffset = localHubOffset(aircraft.prop);
    dim = [hubOffset, 0.20, 0.20];
    posBody = pivot + (0.5 * hubOffset * thrustDirBody);
    eulBody = deg2rad([0, 90 - rotorTiltDeg, 0]);
end

if contains(name, 'F-Rotor')
    rotorTiltDeg = localFrontComponentTilt(name, state.front_tilts_deg, 90);
    pivot = localFrontPivot(name, posBody, aircraft.prop);
    thrustDirBody = localThrustDirection(rotorTiltDeg);
    posBody = pivot + (localHubOffset(aircraft.prop) * thrustDirBody);
    eulBody = deg2rad([0, -rotorTiltDeg, 0]);
end

switch part.kind
    case 'plain'
        verticesLocal = part.baseVertices;

    case 'wing_fixed'
        [verticesFixed, ~] = localBuildWingWithFlaperon(dim, part.controlChordFraction, ...
            localSurfaceDeflection(name, state.surface));
        verticesLocal = verticesFixed;

    case 'wing_control'
        [~, ~, verticesCtrl, ~] = localBuildWingWithFlaperon(dim, part.controlChordFraction, ...
            localSurfaceDeflection(name, state.surface));
        verticesLocal = verticesCtrl;

    case 'tail_fixed'
        [verticesFixed, ~] = localBuildTailWithRuddervator(dim, part.controlChordFraction, ...
            localSurfaceDeflection(name, state.surface));
        verticesLocal = verticesFixed;

    case 'tail_control'
        [~, ~, verticesCtrl, ~] = localBuildTailWithRuddervator(dim, part.controlChordFraction, ...
            localSurfaceDeflection(name, state.surface));
        verticesLocal = verticesCtrl;

    otherwise
        error('make_evtol_video:UnknownPartKind', 'Unknown part kind "%s".', part.kind);
end

spinRad = localRotorSpinAngle(name, state);
if ~isempty(spinRad)
    Rspin = localRotationMatrix([0; 0; spinRad]);
    verticesLocal = (Rspin * verticesLocal.').';
end

Rcomp = localRotationMatrix(eulBody);
verticesBody = (Rcomp * verticesLocal.').'+ posBody;
verticesWorld = localApplyCenteredPose(verticesBody, state.body_R, aircraft.CG);
end

function radius = localComputeSceneRadius(aircraft)
sampleTilts = [0, 90];
maxNorm = 0;

for idx = 1:numel(sampleTilts)
    state = struct();
    state.body_R = eye(3);
    state.front_tilts_deg = sampleTilts(idx) * ones(6, 1);
    state.surface = struct('deltaLW', 0, 'deltaRW', 0, 'deltaLT', 0, 'deltaRT', 0);

    partTemplates = localBuildPartTemplates(aircraft);
    for partIdx = 1:numel(partTemplates)
        verticesWorld = localComputePartVertices(partTemplates(partIdx), state, aircraft);
        if ~isempty(verticesWorld)
            maxNorm = max(maxNorm, max(vecnorm(verticesWorld, 2, 2)));
        end
    end

    frontHubBody = aircraft.prop.posFR(1, :) + localHubOffset(aircraft.prop) * localThrustDirection(sampleTilts(idx));
    rearHubBody = aircraft.prop.posRR(1, :);
    frontHubWorld = localApplyCenteredPose(frontHubBody, state.body_R, aircraft.CG);
    rearHubWorld = localApplyCenteredPose(rearHubBody, state.body_R, aircraft.CG);
    maxNorm = max(maxNorm, norm(frontHubWorld));
    maxNorm = max(maxNorm, norm(rearHubWorld));
end

radius = max(6.0, 1.15 * maxNorm + 0.5);
end

function arrowScale = localComputeArrowScale(prop, radius)
arrowScale = struct();
arrowScale.refForce = 3 * localPropKThrust(prop) * (1000 ^ 2);
if arrowScale.refForce <= eps
    arrowScale.refForce = 1.0;
end
arrowScale.maxLength = max(1.2, 0.22 * radius);
arrowScale.minLength = 0.18 * arrowScale.maxLength;
end

function velocityScale = localComputeVelocityScale(radius)
velocityScale = struct();
velocityScale.refSpeed = 100;
velocityScale.maxLength = max(1.1, 0.22 * radius);
end

function styles = localArrowStyles()
styles = struct('color', {}, 'lineWidth', {}, 'label', {}, 'labelOffset', {});
styles(1).color = [1.0 0.70 0.00];
styles(2).color = [1.0 0.88 0.35];
styles(3).color = [0.20 0.90 1.00];
styles(4).color = [0.55 1.00 1.00];
styles(1).label = 'FR thrust';
styles(2).label = 'FL thrust';
styles(3).label = 'RR thrust';
styles(4).label = 'RL thrust';
styles(1).labelOffset = [0.85, 0.55, 0.20];
styles(2).labelOffset = [0.95, 0.45, 0.20];
styles(3).labelOffset = [1.00, 0.78, 0.42];
styles(4).labelOffset = [1.05, 0.72, 0.42];
for idx = 1:4
    styles(idx).lineWidth = 2.5;
end
end

function [originsWorld, vectorsWorld] = localRepresentativeArrowSet(state, aircraft, arrowScale)
originsWorld = zeros(4, 3);
vectorsWorld = zeros(4, 3);

[originsWorld(1, :), vectorsWorld(1, :)] = localRepresentativeFrontArrow( ...
    aircraft.prop.posFR(2, :), state.front_tilts_deg(2), state.front_force_body, state, aircraft, arrowScale);
[originsWorld(2, :), vectorsWorld(2, :)] = localRepresentativeFrontArrow( ...
    aircraft.prop.posFL(2, :), state.front_tilts_deg(5), state.front_force_left_body, state, aircraft, arrowScale);
[originsWorld(3, :), vectorsWorld(3, :)] = localRepresentativeRearArrow( ...
    aircraft.prop.posRR(2, :), state.rear_force_body, state, aircraft, arrowScale);
[originsWorld(4, :), vectorsWorld(4, :)] = localRepresentativeRearArrow( ...
    aircraft.prop.posRL(2, :), state.rear_force_left_body, state, aircraft, arrowScale);
end

function [originWorld, vectorWorld] = localRepresentativeFrontArrow(hubPivotBody, tiltDeg, groupForceBody, state, aircraft, arrowScale)
hubBody = hubPivotBody + localHubOffset(aircraft.prop) * localThrustDirection(tiltDeg);
originWorld = localApplyCenteredPose(hubBody, state.body_R, aircraft.CG);
vectorWorld = localScaledArrowVector(groupForceBody / 3, state.body_R, arrowScale);
end

function [originWorld, vectorWorld] = localRepresentativeRearArrow(hubBody, groupForceBody, state, aircraft, arrowScale)
originWorld = localApplyCenteredPose(hubBody, state.body_R, aircraft.CG);
vectorWorld = localScaledArrowVector(groupForceBody / 3, state.body_R, arrowScale);
end

function vectorWorld = localScaledArrowVector(forceBody, bodyR, arrowScale)
forceBody = forceBody(:);
mag = norm(forceBody);
if mag <= eps
    vectorWorld = [0, 0, 0];
    return;
end

dirBody = forceBody / mag;
dirWorld = bodyR * dirBody;
arrowLength = arrowScale.maxLength * (mag / arrowScale.refForce);
arrowLength = min(arrowLength, arrowScale.maxLength);
arrowLength = max(arrowLength, arrowScale.minLength);
vectorWorld = (arrowLength * dirWorld).';
end

function labelPos = localArrowLabelPosition(origin, vector, offsetMag, offsetDir)
labelPos = origin + 0.28 * vector + offsetMag * offsetDir;
end

function labelPos = localVelocityLabelPosition(ax, origin, vector, offsetMag)
if norm(vector) <= eps
    labelPos = origin;
    return;
end

camPos = get(ax, 'CameraPosition');
camTarget = get(ax, 'CameraTarget');
camUp = get(ax, 'CameraUpVector');

forward = camTarget - camPos;
forward = forward / max(norm(forward), eps);

camUp = camUp - dot(camUp, forward) * forward;
if norm(camUp) <= eps
    camUp = [0, 0, -1];
    camUp = camUp - dot(camUp, forward) * forward;
end
camUp = camUp / max(norm(camUp), eps);

camRight = cross(camUp, forward);
camRight = camRight / max(norm(camRight), eps);
camUp = cross(forward, camRight);
camUp = camUp / max(norm(camUp), eps);

vectorPlane = vector - dot(vector, forward) * forward;
if norm(vectorPlane) <= eps
    vectorPlane = camUp;
end
vectorPlane = vectorPlane / max(norm(vectorPlane), eps);

vRight = dot(vectorPlane, camRight);
vUp = dot(vectorPlane, camUp);
turnRight = vUp * camRight - vRight * camUp;
if norm(turnRight) <= eps
    turnRight = camRight;
end
turnRight = turnRight / max(norm(turnRight), eps);

labelPos = origin + 1.12 * vector + 0.72 * offsetMag * turnRight;
end

function [originWorld, axisVectors] = localBodyAxisVectorSet(ax, state, aircraft, axisLength, overlayOffset)
originWorld = localCgOverlayOrigin(ax, state, aircraft, overlayOffset);
axisVectors = zeros(3, 3);
for idx = 1:3
    axisVectors(idx, :) = (axisLength * state.body_R(:, idx)).';
end
end

function [originWorld, vectorWorld] = localVelocityVectorSet(ax, state, aircraft, velocityScale, overlayOffset)
originWorld = localCgOverlayOrigin(ax, state, aircraft, overlayOffset);
speed = norm(state.V_B_truth);
if speed <= eps
    vectorWorld = [0, 0, 0];
    return;
end

dirWorld = state.body_R * (state.V_B_truth / speed);
vectorLength = velocityScale.maxLength * (speed / velocityScale.refSpeed);
vectorLength = min(vectorLength, velocityScale.maxLength);
vectorWorld = (vectorLength * dirWorld).';
end

function originWorld = localCgOverlayOrigin(ax, state, aircraft, overlayOffset)
camPos = get(ax, 'CameraPosition');
camTarget = get(ax, 'CameraTarget');
forward = camTarget - camPos;
forward = forward / max(norm(forward), eps);

fuselageVertices = localFuselageVerticesWorld(state, aircraft);
if isempty(fuselageVertices)
    originWorld = -overlayOffset * forward;
    return;
end

cameraDir = -forward(:);
depthAlongCamera = fuselageVertices * cameraDir;
surfaceDepth = max(depthAlongCamera);
originWorld = (surfaceDepth + overlayOffset) * cameraDir.';
end

function verticesWorld = localFuselageVerticesWorld(state, aircraft)
verticesWorld = [];
compData = aircraft.compData;
fuselageIdx = find(strcmp(compData(:, 1), 'Fuselage'), 1);
if isempty(fuselageIdx)
    return;
end

dim = compData{fuselageIdx, 4};
posBody = compData{fuselageIdx, 5};
eulBody = deg2rad(compData{fuselageIdx, 6});
[verticesLocal, ~] = genFacetedFuselage(dim(1), dim(2), dim(3));
Rcomp = localRotationMatrix(eulBody);
verticesBody = (Rcomp * verticesLocal.').'+ posBody;
verticesWorld = localApplyCenteredPose(verticesBody, state.body_R, aircraft.CG);
end

function [posWorld, dirWorld] = localAeroNormalState(surface, state, aircraft)
posWorld = localApplyCenteredPose(surface.pos(:).', state.body_R, aircraft.CG);
dirWorld = localRotateDirection(surface.n(:), state.body_R).';
end

function pointsWorld = localApplyCenteredPose(pointsBody, bodyR, CG)
pointsBody = localAsPointRows(pointsBody);
pointsWorld = (bodyR * (pointsBody - CG(:).').').';
end

function dirWorld = localRotateDirection(dirBody, bodyR)
dirWorld = bodyR * dirBody(:);
dirWorld = dirWorld / max(norm(dirWorld), eps);
end

function forceBody = localGroupForceFromRpm(groupRpm, tiltDeg, isFront, prop)
groupRpm = max(groupRpm(:), 0);
thrustMag = 3 * localPropKThrust(prop) * (groupRpm .^ 2);

if isFront
    dirBody = [sind(tiltDeg(:)), zeros(numel(groupRpm), 1), -cosd(tiltDeg(:))];
else
    dirBody = repmat([0, 0, -1], numel(groupRpm), 1);
end

forceBody = thrustMag .* dirBody;
end

function rotorSpin = localBuildRotorSpinHistory(frameTime, groupRpm, prop)
nFrame = numel(frameTime);
rotorSpin = struct( ...
    'FR', zeros(nFrame, 3), ...
    'FL', zeros(nFrame, 3), ...
    'RR', zeros(nFrame, 3), ...
    'RL', zeros(nFrame, 3));

if nFrame <= 1
    return;
end

rotorSpin.FR = localIntegrateRotorSpin(frameTime, localVisualSpinRps(groupRpm(:, 1)), localSideSpinDir(prop, 'R'));
rotorSpin.FL = localIntegrateRotorSpin(frameTime, localVisualSpinRps(groupRpm(:, 2)), localSideSpinDir(prop, 'L'));
rotorSpin.RR = localIntegrateRotorSpin(frameTime, localVisualSpinRps(groupRpm(:, 3)), localSideSpinDir(prop, 'R'));
rotorSpin.RL = localIntegrateRotorSpin(frameTime, localVisualSpinRps(groupRpm(:, 4)), localSideSpinDir(prop, 'L'));
end

function phase = localIntegrateRotorSpin(frameTime, visualRps, spinDir)
nFrame = numel(frameTime);
phase = zeros(nFrame, numel(spinDir));
spinDir = reshape(spinDir, 1, []);

for idx = 2:nFrame
    dt = max(frameTime(idx) - frameTime(idx - 1), 0.0);
    avgRps = 0.5 * (visualRps(idx - 1) + visualRps(idx));
    phase(idx, :) = phase(idx - 1, :) + (2 * pi * avgRps * dt) * spinDir;
end

phase = mod(phase, 2 * pi);
end

function visualRps = localVisualSpinRps(groupRpm)
groupRpm = max(groupRpm(:), 0.0);
visualRps = 3.0 * (groupRpm / 1000.0);
visualRps = min(visualRps, 6.0);
visualRps(groupRpm <= 1.0) = 0.0;
end

function spinDir = localSideSpinDir(prop, sideCode)
if strcmpi(sideCode, 'L') && isfield(prop, 'Lspin_dir') && ~isempty(prop.Lspin_dir)
    spinDir = prop.Lspin_dir(:).';
    return;
end

if strcmpi(sideCode, 'R') && isfield(prop, 'Rspin_dir') && ~isempty(prop.Rspin_dir)
    spinDir = prop.Rspin_dir(:).';
    return;
end

if strcmpi(sideCode, 'L')
    spinDir = [-1, 1, -1];
else
    spinDir = [1, -1, 1];
end
end

function spinRad = localRotorSpinAngle(name, state)
spinRad = [];
if ~isfield(state, 'rotor_spin') || isempty(state.rotor_spin)
    return;
end

token = regexp(name, '([LR])(\d)$', 'tokens', 'once');
if isempty(token)
    return;
end

sideCode = token{1};
rotorIndex = str2double(token{2});
if ~isfinite(rotorIndex) || rotorIndex < 1 || rotorIndex > 3
    return;
end

if contains(name, 'F-Rotor')
    if strcmpi(sideCode, 'R')
        spinRad = state.rotor_spin.FR(rotorIndex);
    else
        spinRad = state.rotor_spin.FL(rotorIndex);
    end
    return;
end

if contains(name, 'R-Rotor')
    if strcmpi(sideCode, 'R')
        spinRad = state.rotor_spin.RR(rotorIndex);
    else
        spinRad = state.rotor_spin.RL(rotorIndex);
    end
end
end

function kThrust = localPropKThrust(prop)
kThrust = 1.0;
if isfield(prop, 'k_Thrust') && ~isempty(prop.k_Thrust)
    kThrust = prop.k_Thrust;
end
end

function faceAlpha = localDefaultFaceAlpha(name, type)
faceAlpha = 0.85;
if strcmp(type, 'fuselage') || contains(name, 'Fuselage')
    faceAlpha = 0.45;
elseif contains(name, 'Ballast')
    faceAlpha = 0.65;
end
end

function txt = localTelemetryString(state, ~)
groupRpm = state.group_rpm(:);
txt = sprintf([ ...
    't = %5.2f s\n' ...
    'Euler [deg]     : [%6.1f %6.1f %6.1f]\n' ...
    'V_B [m/s]       : [%6.2f %6.2f %6.2f]\n' ...
    '|V| [m/s]       : %6.2f\n' ...
    'Tilt [deg]\n' ...
    '  FR           = %6.1f\n' ...
    '  FL           = %6.1f\n' ...
    'Surf [deg]\n' ...
    '  LW           = %6.1f\n' ...
    '  RW           = %6.1f\n' ...
    '  LT           = %6.1f\n' ...
    '  RT           = %6.1f\n' ...
    '%s\n' ...
    '  FR           = %6.0f\n' ...
    '  FL           = %6.0f\n' ...
    '  RR           = %6.0f\n' ...
    '  RL           = %6.0f\n' ...
    'Thrust |F| [N]\n' ...
    '  FR           = %6.0f\n' ...
    '  FL           = %6.0f\n' ...
    '  RR           = %6.0f\n' ...
    '  RL           = %6.0f'], ...
    state.time, ...
    state.body_eul_deg(1), state.body_eul_deg(2), state.body_eul_deg(3), ...
    state.V_B_truth(1), state.V_B_truth(2), state.V_B_truth(3), state.V_B_mag, ...
    state.front_tilts_deg(1), state.front_tilts_deg(4), ...
    state.surface_deg(1), state.surface_deg(2), state.surface_deg(3), state.surface_deg(4), ...
    'RPM', groupRpm(1), groupRpm(2), groupRpm(3), groupRpm(4), ...
    state.group_thrust_mag(1), state.group_thrust_mag(2), state.group_thrust_mag(3), state.group_thrust_mag(4));
end

function trajectory = localCreateTrajectoryInset(parentFig, frameData)
trajectory = struct('available', false);
trajData = localBuildTrajectoryData(frameData.pos_NED);

trajectory.ax = axes('Parent', parentFig, 'Position', [0.055 0.075 0.245 0.335]);
hold(trajectory.ax, 'on');
grid(trajectory.ax, 'on');
box(trajectory.ax, 'on');
set(trajectory.ax, ...
    'Color', [0.12 0.12 0.12], ...
    'GridColor', [0.70 0.70 0.70], ...
    'GridAlpha', 0.55, ...
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', ...
    'FontSize', 7, ...
    'LineWidth', 0.8);

trajectory.fullHandle = plot3(trajectory.ax, ...
    trajData.xyz(:, 1), trajData.xyz(:, 2), trajData.xyz(:, 3), ...
    '--', 'Color', [0.82 0.82 0.82], 'LineWidth', 1.0);
trajectory.historyHandle = plot3(trajectory.ax, ...
    trajData.xyz(1, 1), trajData.xyz(1, 2), trajData.xyz(1, 3), ...
    'Color', [0.00 0.35 0.85], 'LineWidth', 1.8);
trajectory.currentHandle = plot3(trajectory.ax, ...
    trajData.xyz(1, 1), trajData.xyz(1, 2), trajData.xyz(1, 3), ...
    'o', 'MarkerSize', 5, ...
    'MarkerFaceColor', [0.85 0.05 0.05], ...
    'MarkerEdgeColor', 'k');

xlabel(trajectory.ax, 'N rel [m]', 'Color', 'w');
ylabel(trajectory.ax, 'E rel [m]', 'Color', 'w');
zlabel(trajectory.ax, 'Alt rel [m]', 'Color', 'w');
trajectory.titleHandle = title(trajectory.ax, 'Trajectory', 'Color', 'w', 'FontWeight', 'bold');

xlim(trajectory.ax, trajData.limits(1, :));
ylim(trajectory.ax, trajData.limits(2, :));
zlim(trajectory.ax, trajData.limits(3, :));
daspect(trajectory.ax, [1 1 1]);
view(trajectory.ax, 132, 24);
axis(trajectory.ax, 'vis3d');

trajectory.data = trajData;
trajectory.available = true;
end

function trajectory = localUpdateTrajectoryInset(trajectory, frameData, frameIdx)
idx = min(max(frameIdx, 1), size(trajectory.data.xyz, 1));
xyz = trajectory.data.xyz;

set(trajectory.historyHandle, ...
    'XData', xyz(1:idx, 1), ...
    'YData', xyz(1:idx, 2), ...
    'ZData', xyz(1:idx, 3));
set(trajectory.currentHandle, ...
    'XData', xyz(idx, 1), ...
    'YData', xyz(idx, 2), ...
    'ZData', xyz(idx, 3));
set(trajectory.titleHandle, 'String', sprintf('Trajectory  t = %.2f s', frameData.time(idx)));
end

function trajectoryFile = localSaveTrajectoryPlot(frameData, outputFile)
[folderPath, baseName, ~] = fileparts(outputFile);
trajectoryFile = fullfile(folderPath, [baseName '_trajectory.png']);
trajData = localBuildTrajectoryData(frameData.pos_NED);

fig = figure( ...
    'Name', 'eVTOL Trajectory', ...
    'Color', 'w', ...
    'Position', [120 120 900 760], ...
    'Visible', 'off');
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

plot3(ax, trajData.xyz(:, 1), trajData.xyz(:, 2), trajData.xyz(:, 3), ...
    'k--', 'LineWidth', 1.15, 'DisplayName', 'Full path');
plot3(ax, trajData.xyz(:, 1), trajData.xyz(:, 2), trajData.xyz(:, 3), ...
    'Color', [0.00 0.35 0.85], 'LineWidth', 2.0, 'DisplayName', 'Trajectory');
plot3(ax, trajData.xyz(1, 1), trajData.xyz(1, 2), trajData.xyz(1, 3), ...
    'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.10 0.65 0.20], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Start');
plot3(ax, trajData.xyz(end, 1), trajData.xyz(end, 2), trajData.xyz(end, 3), ...
    'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.85 0.05 0.05], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', 'End');

xlabel(ax, 'North rel [m]');
ylabel(ax, 'East rel [m]');
zlabel(ax, 'Altitude rel [m]');
titleHandle = title(ax, 'eVTOL Trajectory Relative To Start');
titleHandle.Color = 'k';
xlim(ax, trajData.limits(1, :));
ylim(ax, trajData.limits(2, :));
zlim(ax, trajData.limits(3, :));
daspect(ax, [1 1 1]);
view(ax, 132, 24);
axis(ax, 'vis3d');
leg = legend(ax, 'Location', 'best', 'Box', 'off');
leg.TextColor = 'k';
leg.Color = 'none';
set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'GridColor', [0.82 0.82 0.82], 'FontSize', 11, 'LineWidth', 1.0);

exportgraphics(fig, trajectoryFile, 'Resolution', 300);
close(fig);
end

function trajData = localBuildTrajectoryData(posNed)
if isempty(posNed) || size(posNed, 2) < 3
    error('make_evtol_video:BadTrajectoryData', ...
        'pos_NED must contain at least 3 columns to build the trajectory plot.');
end

posNed = posNed(:, 1:3);
posRel = posNed - posNed(1, :);
trajData = struct();
trajData.xyz = [posRel(:, 1), posRel(:, 2), -posRel(:, 3)];
trajData.limits = localTrajectoryLimits(trajData.xyz);
end

function limits = localTrajectoryLimits(xyz)
limits = zeros(3, 2);
for idx = 1:3
    lo = min(xyz(:, idx));
    hi = max(xyz(:, idx));
    span = hi - lo;
    if span <= 1e-9
        pad = 1.0;
    else
        pad = max(0.08 * span, 1.0);
    end
    limits(idx, :) = [lo - pad, hi + pad];
end
end

function rgbFrame = localMakeEvenFrame(rgbFrame)
[nRows, nCols, ~] = size(rgbFrame);
if mod(nRows, 2) ~= 0
    rgbFrame(end + 1, :, :) = rgbFrame(end, :, :);
end
if mod(nCols, 2) ~= 0
    rgbFrame(:, end + 1, :) = rgbFrame(:, end, :);
end
end

function localPrintProgress(frameIdx, totalFrames, elapsedSeconds)
progress = 0;
if totalFrames > 0
    progress = frameIdx / totalFrames;
end
progress = min(max(progress, 0), 1);

barLength = 24;
filled = floor(progress * barLength);
barText = [repmat('#', 1, filled), repmat('-', 1, barLength - filled)];

if frameIdx <= 0 || elapsedSeconds <= 0 || progress <= 0
    etaText = '--';
else
    etaSeconds = max((elapsedSeconds / progress) - elapsedSeconds, 0);
    etaText = localFormatDuration(etaSeconds);
end

fprintf('  Progress: [%s] %5.1f%%  frame %d/%d  elapsed %s  ETA %s\r', ...
    barText, 100 * progress, frameIdx, totalFrames, localFormatDuration(elapsedSeconds), etaText);

if frameIdx >= totalFrames
    fprintf('\n');
end
end

function txt = localFormatDuration(seconds)
seconds = max(seconds, 0);
if seconds < 60
    txt = sprintf('%.1fs', seconds);
    return;
end

minutes = floor(seconds / 60);
seconds = seconds - 60 * minutes;
if minutes < 60
    txt = sprintf('%dm %04.1fs', minutes, seconds);
    return;
end

hours = floor(minutes / 60);
minutes = minutes - 60 * hours;
txt = sprintf('%dh %02dm %04.1fs', hours, minutes, seconds);
end

function color = localComponentColor(name, type)
if strcmp(type, 'fuselage')
    color = [0.0 1.0 1.0];
elseif contains(name, 'Ballast')
    color = [1.0 1.0 0.0];
elseif contains(name, 'Wing')
    color = [1.0 0.0 1.0];
elseif contains(name, 'Tail')
    color = [1.0 0.5 0.0];
elseif contains(name, 'F-Rotor')
    color = [1.0 0.0 0.0];
elseif contains(name, 'R-Rotor')
    color = [0.0 0.5 1.0];
elseif strcmp(type, 'box')
    color = [0.0 1.0 0.0];
else
    color = [0.5 0.5 0.5];
end
end

function controlChordFraction = localTailControlChordFraction(controls)
controlChordFraction = 0.30;
if isfield(controls, 'ruddervator') && isfield(controls.ruddervator, 'control_chord_fraction')
    controlChordFraction = controls.ruddervator.control_chord_fraction;
end
end

function controlChordFraction = localWingControlChordFraction(controls)
controlChordFraction = 0.28;
if isfield(controls, 'flaperon') && isfield(controls.flaperon, 'control_chord_fraction')
    controlChordFraction = controls.flaperon.control_chord_fraction;
end
end

function deflectionRad = localSurfaceDeflection(name, state)
if contains(name, 'L Main Wing')
    deflectionRad = state.deltaLW;
elseif contains(name, 'R Main Wing')
    deflectionRad = state.deltaRW;
elseif contains(name, 'L V-Tail')
    deflectionRad = state.deltaLT;
elseif contains(name, 'R V-Tail')
    deflectionRad = state.deltaRT;
else
    deflectionRad = 0;
end
end

function hubOffset = localHubOffset(prop)
hubOffset = 0.50;
if isfield(prop, 'hub_offset') && ~isempty(prop.hub_offset)
    hubOffset = prop.hub_offset;
end
end

function pivot = localFrontPivot(name, fallbackPos, prop)
pivot = fallbackPos;
tokens = regexp(name, 'F-(?:Arm|Rotor) ([LR])([123])', 'tokens', 'once');
if isempty(tokens)
    return;
end

side = tokens{1};
index = str2double(tokens{2});
fieldName = 'posFL';
if side == 'R'
    fieldName = 'posFR';
end

if isfield(prop, fieldName) && size(prop.(fieldName), 1) >= index
    pivot = prop.(fieldName)(index, :);
end
end

function tiltDeg = localFrontComponentTilt(name, frontTiltsDeg, defaultTiltDeg)
tiltDeg = defaultTiltDeg;
tokens = regexp(name, 'F-(?:Arm|Rotor) ([LR])([123])', 'tokens', 'once');
if isempty(tokens)
    return;
end

side = tokens{1};
index = str2double(tokens{2});
if side == 'R'
    tiltDeg = frontTiltsDeg(index);
else
    tiltDeg = frontTiltsDeg(index + 3);
end
end

function dirVec = localThrustDirection(tiltDeg)
dirVec = [sind(tiltDeg), 0, -cosd(tiltDeg)];
end

function [verticesFixed, facesFixed, verticesCtrl, facesCtrl] = ...
    localBuildTailWithRuddervator(dim, controlChordFraction, deflectionRad)
[verticesFixed, facesFixed, verticesCtrl, facesCtrl] = ...
    localBuildTrailingEdgeSurface(dim, controlChordFraction, deflectionRad);
end

function [verticesFixed, facesFixed, verticesCtrl, facesCtrl] = ...
    localBuildWingWithFlaperon(dim, controlChordFraction, deflectionRad)
[verticesFixed, facesFixed, verticesCtrl, facesCtrl] = ...
    localBuildTrailingEdgeSurface(dim, controlChordFraction, deflectionRad);
end

function [verticesFixed, facesFixed, verticesCtrl, facesCtrl] = ...
    localBuildTrailingEdgeSurface(dim, controlChordFraction, deflectionRad)
chord = dim(1);
span = dim(2);
thickness = dim(3);

controlChordFraction = min(max(controlChordFraction, 0.05), 0.95);
ctrlChord = chord * controlChordFraction;
fixedChord = chord - ctrlChord;

[verticesFixed, facesFixed] = genCleanBox(fixedChord, span, thickness);
verticesFixed(:, 1) = verticesFixed(:, 1) + 0.5 * ctrlChord;

[verticesCtrl, facesCtrl] = genCleanBox(ctrlChord, span, thickness);
verticesCtrl(:, 1) = verticesCtrl(:, 1) - 0.5 * fixedChord;

hingeLocal = [-0.5 * chord + ctrlChord, 0, 0];
Rdeflect = localRotationMatrix([0; deflectionRad; 0]);
verticesCtrl = (Rdeflect * (verticesCtrl - hingeLocal).').'+ hingeLocal;
end

function R = localRotationMatrix(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rz * Ry * Rx;
end

function points = localAsPointRows(points)
if isempty(points)
    points = zeros(0, 3);
    return;
end
if ~isnumeric(points)
    error('make_evtol_video:BadPointData', 'Point data must be numeric.');
end
if size(points, 2) == 3
    return;
end
if size(points, 1) == 3
    points = points.';
    return;
end
error('make_evtol_video:BadPointData', 'Point data must have three columns.');
end

function [V, F] = genCleanBox(L, W, H)
x = L / 2;
y = W / 2;
z = H / 2;
V = [-x -y -z;  x -y -z;  x  y -z; -x  y -z; ...
     -x -y  z;  x -y  z;  x  y  z; -x  y  z];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function [V, F] = genCrossProp(D, W, T)
[V1, F1] = genCleanBox(D, W, T);
[V2, F2] = genCleanBox(W, D, T);
V = [V1; V2];
F = [F1; F2 + 8];
end

function [V, F] = genFacetedFuselage(L, W, H)
xNorm = [0.5, 0.35, 0.05, -0.3, -0.5];
wNorm = [0.3, 0.8, 1.0, 0.6, 0.2];
zTopNorm = [-0.2, -0.7, -1.0, -0.6, -0.3];
zBotNorm = [0.4, 0.8, 1.0, 0.5, -0.1];
nSec = length(xNorm);
V = [];
for idx = 1:nSec
    x = xNorm(idx) * L;
    y = wNorm(idx) * W / 2;
    zTop = zTopNorm(idx) * H / 2;
    zBot = zBotNorm(idx) * H / 2;
    V = [V; x, y, zTop; x, -y, zTop; x, -y, zBot; x, y, zBot]; %#ok<AGROW>
end

F = [];
for idx = 1:(nSec - 1)
    base = (idx - 1) * 4;
    F = [F; base + 1, base + 2, base + 6, base + 5]; %#ok<AGROW>
    F = [F; base + 2, base + 3, base + 7, base + 6]; %#ok<AGROW>
    F = [F; base + 3, base + 4, base + 8, base + 7]; %#ok<AGROW>
    F = [F; base + 4, base + 1, base + 5, base + 8]; %#ok<AGROW>
end

F = [F; 1, 2, 3, 4];
base = (nSec - 1) * 4;
F = [F; base + 4, base + 3, base + 2, base + 1];
end
