function result = Run_INDI_PathSegment_Test(segmentIndex, opts)
%RUN_INDI_PATHSEGMENT_TEST Run one adjacent segment of the INDI trim path.
%
% Usage:
%   Run_INDI_PathSegment_Test
%   Run_INDI_PathSegment_Test(1)
%   Run_INDI_PathSegment_Test(3, struct('stopTime_s', 90))
%   Run_INDI_PathSegment_Test([], struct('full_path', true, 'stopTime_s', 240))
%
% This is a narrow diagnostic runner. It builds the normal INDI transition
% controller, slices it to path point N -> N+1, then runs the standard
% Run_Main workflow with that two-point schedule.

if nargin < 1 || isempty(segmentIndex)
    segmentIndex = 1;
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
opts = localApplyDefaults(opts);

repoRoot = fileparts(mfilename('fullpath'));
cd(repoRoot);

evalin('base', 'Init_Main');

if isempty(opts.builder_opts)
    fullControllerData = build_indi_transition_controller();
else
    fullControllerData = build_indi_transition_controller(opts.builder_opts);
end

nPts = localScheduleCount(fullControllerData);
if opts.full_path
    cols = 1:nPts;
    controllerData = fullControllerData;
else
    if segmentIndex < 1 || segmentIndex >= nPts
        error('Run_INDI_PathSegment_Test:BadSegmentIndex', ...
            'segmentIndex must be in [1, %d]. Received %d.', nPts - 1, segmentIndex);
    end
    cols = [segmentIndex, segmentIndex + 1];
    controllerData = localSliceControllerData(fullControllerData, cols);
end
trimResult = localBuildStartTrimResult(controllerData);
runCase = localBuildRunCase(segmentIndex, opts);

assignin('base', 'fullIndiTransitionControllerData', fullControllerData);
assignin('base', 'controllerData', controllerData);
assignin('base', 'indiTransitionPathTable', controllerData.schedule_table);
assignin('base', 'trimResult', trimResult);
assignin('base', 'runCase', runCase);

fprintf('\n=== INDI Path Segment Test ===\n');
if opts.full_path
    fprintf('Segment        : full path (%d points)\n', nPts);
else
    fprintf('Segment        : %d -> %d\n', segmentIndex, segmentIndex + 1);
end
fprintf('Run name       : %s\n', runCase.name);
fprintf('Stop time      : %.3f s\n', runCase.stopTime_s);
fprintf('Sim attempt    : %d\n', runCase.attemptSimulation);
disp(controllerData.schedule_table(:, {'path_index', 'vinf_mps', ...
    'tilt_deg', 'alpha_deg', 'theta_deg', 'front_collective_rpm', ...
    'rear_collective_rpm', 'delta_f_deg', 'delta_e_deg', 'classification'}));

evalin('base', 'Run_Main');
runResult = evalin('base', 'runResult');
runContext = evalin('base', 'runContext');

reportData = [];
if opts.plot && strcmp(runResult.status, 'simulated')
    if exist(opts.output_dir, 'dir') ~= 7
        mkdir(opts.output_dir);
    end
    filenameStem = fullfile(opts.output_dir, runCase.name);
    simSource = evalin('base', 'out');
    cmdSource = evalin('base', 'cmds');
    try
        if opts.full_path
            plotTitle = 'INDI Full Path';
        else
            plotTitle = sprintf('INDI Segment %d to %d', segmentIndex, segmentIndex + 1);
        end
        reportData = plot_transition_debug(simSource, cmdSource, filenameStem, ...
            plotTitle);
    catch ME
        warning('Run_INDI_PathSegment_Test:PlotFailed', ...
            'Simulation completed, but plot_transition_debug failed: %s', ME.message);
    end
end

result = struct();
result.segment_index = segmentIndex;
result.segment_columns = cols;
result.controllerData = controllerData;
result.trimResult = trimResult;
result.runCase = runCase;
result.runResult = runResult;
result.runContext = runContext;
result.output_dir = opts.output_dir;

if opts.save_summary
    if exist(opts.output_dir, 'dir') ~= 7
        mkdir(opts.output_dir);
    end
    summaryFile = fullfile(opts.output_dir, [runCase.name '_summary.mat']);
    if isempty(reportData)
        save(summaryFile, 'result', '-v7.3');
    else
        reportExport = localReportExportOnly(reportData);
        save(summaryFile, 'result', 'reportExport', '-v7.3');
    end
    result.summary_file = summaryFile;
    fprintf('Saved summary  : %s\n', summaryFile);
end

assignin('base', 'indiPathSegmentTestResult', result);
fprintf('Status         : %s\n', runResult.status);
fprintf('Output dir     : %s\n', opts.output_dir);
fprintf('=== INDI Path Segment Test Complete ===\n\n');
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'stopTime_s') || isempty(opts.stopTime_s)
    opts.stopTime_s = 60.0;
end
if ~isfield(opts, 'attemptSimulation') || isempty(opts.attemptSimulation)
    opts.attemptSimulation = true;
end
if ~isfield(opts, 'plot') || isempty(opts.plot)
    opts.plot = true;
end
if ~isfield(opts, 'save_summary') || isempty(opts.save_summary)
    opts.save_summary = true;
end
if ~isfield(opts, 'full_path') || isempty(opts.full_path)
    opts.full_path = false;
end
if ~isfield(opts, 'builder_opts')
    opts.builder_opts = [];
end
if ~isfield(opts, 'output_dir') || isempty(opts.output_dir)
    repoRoot = fileparts(mfilename('fullpath'));
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    opts.output_dir = fullfile(repoRoot, 'workspace_plots', ...
        'indi_path_segment_tests', stamp);
end
end

function nPts = localScheduleCount(controllerData)
if isfield(controllerData, 'schedule_count') && ~isempty(controllerData.schedule_count)
    nPts = controllerData.schedule_count;
else
    nPts = size(controllerData.controller_trim_cmd, 2);
end
end

function controllerData = localSliceControllerData(fullControllerData, cols)
controllerData = fullControllerData;
controllerData.name = sprintf('%s_segment_%02d_to_%02d', ...
    fullControllerData.name, cols(1), cols(2));

fullScheduleCount = localScheduleCount(fullControllerData);
controllerData.controller_state_ref = fullControllerData.controller_state_ref(:, cols);
controllerData.controller_trim_cmd = fullControllerData.controller_trim_cmd(:, cols);
controllerData.controller_gain_lqr = fullControllerData.controller_gain_lqr(:, :, cols);
if size(fullControllerData.controller_gain_lqr, 3) > fullScheduleCount
    controllerData.controller_gain_lqr = cat(3, controllerData.controller_gain_lqr, ...
        fullControllerData.controller_gain_lqr(:, :, (fullScheduleCount + 1):end));
end
controllerData.controller_state_ref(12, :) = [0.0, 1.0];

pathTable = fullControllerData.schedule_table(cols, :);
pathTable.path_index = (1:height(pathTable)).';
pathTable.path_progress = [0.0; 1.0];

controllerData.schedule_count = 2;
controllerData.schedule_progress = [0.0; 1.0];
controllerData.schedule_tilt_deg = pathTable.tilt_deg;
controllerData.schedule_vinf_mps = pathTable.vinf_mps;
controllerData.schedule_alpha_deg = pathTable.alpha_deg;
controllerData.schedule_table = pathTable;
controllerData.x_trim = controllerData.controller_state_ref(1:9, :);
controllerData.trim_cmd = controllerData.controller_trim_cmd;
controllerData.x_trim_lqr = controllerData.controller_state_ref(1:9, end);
controllerData.U_trim_lqr = [controllerData.controller_trim_cmd(1, end); ...
    controllerData.controller_trim_cmd(3:6, end)];

controllerData.path_debug = struct();
controllerData.path_debug.full_path_columns = cols(:);
controllerData.path_debug.note = 'Two-point slice from full INDI path.';
end

function trimResult = localBuildStartTrimResult(controllerData)
row = controllerData.schedule_table(1, :);
trim = controllerData.controller_trim_cmd(:, 1);
x = controllerData.controller_state_ref(:, 1);
mixMatrix = evalin('base', 'MixMatrix');
surface = mixMatrix * trim(3:6);

trimResult = struct();
trimResult.name = char("INDI_PathSegment_Start_" + string(row.path_index));
trimResult.mode = 'transition';
trimResult.modelName = 'Trim_Plant';
trimResult.success = true;
trimResult.isExactTrim = true;
trimResult.Att_Trim = [0; deg2rad(row.theta_deg); 0];
trimResult.Att_Trim_deg = [0; row.theta_deg; 0];
trimResult.Vel_B_BA_Trim = [row.u_mps; 0; row.w_mps];
trimResult.Vel_W_Trim = [row.vinf_mps; deg2rad(row.alpha_deg); 0];
trimResult.Rates_Trim = x(7:9);
trimResult.Pos_Trim = zeros(3, 1);
trimResult.X_trim = [zeros(3, 1); trimResult.Vel_B_BA_Trim; ...
    trimResult.Att_Trim; trimResult.Rates_Trim];
trimResult.Act_Trim = surface;
trimResult.U_trim = [trim(1); trim(3:6)];
trimResult.U_surface_trim = [trim(1); surface];
trimResult.U_trim_full = [zeros(4, 1); row.tilt_deg; row.tilt_deg; ...
    trim(1); trim(2); surface];
trimResult.trim = trimResult;
end

function runCase = localBuildRunCase(segmentIndex, opts)
runCase = struct();
if opts.full_path
    runCase.name = sprintf('INDI_FullPath_%gs', opts.stopTime_s);
else
    runCase.name = sprintf('INDI_PathSegment_%02d_to_%02d_%gs', ...
        segmentIndex, segmentIndex + 1, opts.stopTime_s);
end
runCase.useController = true;
runCase.attemptSimulation = opts.attemptSimulation;
runCase.stopTime_s = opts.stopTime_s;

% The INDI controller advances from its internal settle/ramp schedule. Leave
% Run_Main's external commands aligned with the starting trim point.
end

function reportExport = localReportExportOnly(reportData)
reportExport = struct();
fields = {'pos', 'vel', 'att', 'rates', 'airspeed', 'tilt', 'rotor', ...
    'surface', 'angular_accel', 'scheduled', 'transition_switch_events'};
for i = 1:numel(fields)
    fieldName = fields{i};
    if isfield(reportData, fieldName)
        reportExport.(fieldName) = reportData.(fieldName);
    end
end
end
