% Run_Trim_Transition_RearOn_Forever_Scored.m
% Rear-on forever runner.
%
% This script repeatedly launches scored reference-line sweeps in phases.
% Each phase searches a band around the reference corridor, checkpoints the
% results, then widens or densifies the search band and keeps going until
% the user stops MATLAB.
%
% Stop behavior:
%   - Use Ctrl+C / Stop in MATLAB to halt it.
%   - The latest checkpoint files remain on disk.
%
% Stable latest outputs:
%   workspace_plots/transition_trim_rearon_forever_scored_latest.csv
%   workspace_plots/transition_trim_rearon_forever_scored_latest.mat
%   workspace_plots/transition_trim_rearon_forever_scored_latest.md

if ~exist('transitionRearOnForeverOptions', 'var') || ~isstruct(transitionRearOnForeverOptions)
    transitionRearOnForeverOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

base_output_prefix = localGetField(transitionRearOnForeverOptions, ...
    'base_output_prefix', 'transition_trim_rearon_forever_scored');
base_latest_prefix = localGetField(transitionRearOnForeverOptions, ...
    'base_latest_prefix', 'transition_trim_rearon_forever_scored');
phase_log_file = fullfile(root_dir, 'workspace_plots', [base_latest_prefix '_phase_log.md']);
phase_state_mat = fullfile(root_dir, 'workspace_plots', [base_latest_prefix '_phase_state.mat']);
max_phases = localGetField(transitionRearOnForeverOptions, 'max_phases', inf);
pause_between_phases_s = localGetField(transitionRearOnForeverOptions, 'pause_between_phases_s', 0.0);

phase_index = 1;
if exist(phase_state_mat, 'file') == 2
    try
        S = load(phase_state_mat, 'phase_index');
        if isfield(S, 'phase_index') && isnumeric(S.phase_index) && isscalar(S.phase_index)
            phase_index = max(1, floor(S.phase_index) + 1);
        end
    catch
    end
end

fprintf('=== Run_Trim_Transition_RearOn_Forever_Scored ===\n');
fprintf('Base latest prefix: %s\n', base_latest_prefix);
fprintf('Phase log: %s\n', phase_log_file);
fprintf('Starting phase index: %d\n', phase_index);

while phase_index <= max_phases
    phase_options = localBuildPhaseOptions(phase_index, transitionRearOnForeverOptions, ...
        root_dir, base_output_prefix, base_latest_prefix);

    fprintf('\n--- Forever phase %d ---\n', phase_index);
    fprintf('  offsets = [%s]\n', num2str(phase_options.reference_tilt_offsets_deg, ' %.2f'));
    fprintf('  vinf samples = %d points\n', numel(phase_options.reference_vinf_samples));
    fprintf('  max attempts = %d\n', phase_options.max_attempts);
    fprintf('  scored history count = %d\n', numel(phase_options.history_scored_csvs));

    phase_options.phase_index = phase_index;
    transitionPathScoredOptions = phase_options; %#ok<NASGU>
    Run_Trim_Transition_Reference_Line_Scored

    completed_phase_options = transitionPathScoredOptions;
    completed_phase_index = localGetField(completed_phase_options, 'phase_index', 1);
    latestCsv = fullfile(root_dir, 'workspace_plots', [completed_phase_options.latest_prefix '_latest.csv']);
    latestSummary = localReadSummaryOrEmpty(latestCsv);
    new_state = struct();
    new_state.phase_index = completed_phase_index;
    new_state.last_run_finished_on = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd HH:mm:ss Z'));
    new_state.last_latest_csv = latestCsv;
    new_state.last_summary_height = height(latestSummary);
    save(phase_state_mat, '-struct', 'new_state');

    localAppendPhaseLog(phase_log_file, completed_phase_index, completed_phase_options, latestSummary);

    phase_index = completed_phase_index + 1;
    if pause_between_phases_s > 0
        pause(pause_between_phases_s);
    end
end

function phase_options = localBuildPhaseOptions(phase_index, userOptions, root_dir, base_output_prefix, base_latest_prefix)
phase_options = struct();
phase_options.output_prefix = sprintf('%s_phase%03d', base_output_prefix, phase_index);
phase_options.latest_prefix = base_latest_prefix;
phase_options.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 1);
phase_options.max_attempts = localGetField(userOptions, 'phase_max_attempts', 250);

phase_options.min_vinf_mps = 0.0;
phase_options.max_vinf_mps = 75.0;
phase_options.min_tilt_deg = 0.0;
phase_options.max_tilt_deg = 90.0;

phase_options.reference_knot_vinf_mps = localGetField(userOptions, ...
    'reference_knot_vinf_mps', [0 2.5 5 10 15 20 25 30 35 40 45 50 60 70 75]);
phase_options.reference_knot_tilt_deg = localGetField(userOptions, ...
    'reference_knot_tilt_deg', [0 5 10 20 30 40 50 60 67 73 78 82 86 90 90]);

% Expand outward from the line over time, but keep the search concentrated.
offset_step = localGetField(userOptions, 'offset_step_deg', 2.5);
max_radius = localGetField(userOptions, 'max_offset_radius_deg', 25.0);
radius = min(max_radius, offset_step * ceil(phase_index / 2));
if phase_index == 1
    offsets = [0, -offset_step, offset_step];
elseif mod(phase_index, 2) == 0
    offsets = unique([-radius, -radius/2, 0, radius/2, radius]);
else
    offsets = unique([-radius, 0, radius]);
end
phase_options.reference_tilt_offsets_deg = sort(offsets);
phase_options.reference_grid_step_deg = localGetField(userOptions, 'reference_grid_step_deg', 2.5);

% Densify the difficult middle band as phases progress.
if phase_index <= 2
    mid_step = 2.5;
elseif phase_index <= 5
    mid_step = 1.25;
else
    mid_step = 1.0;
end
coarse_low = 0:2.5:20;
mid = 20:mid_step:50;
coarse_high = 50:2.5:75;
phase_options.reference_vinf_samples = unique([coarse_low, mid, coarse_high]);

phase_options.enabled_families = localGetField(userOptions, 'enabled_families', { ...
    'front_rear_free_flap_elevator', ...
    'rear_fixed_flap_elevator', ...
    'hover_zero_surface'});
phase_options.neighbor_seed_count = localGetField(userOptions, 'neighbor_seed_count', 2);
phase_options.history_exact_seed_count = localGetField(userOptions, 'history_exact_seed_count', 2);
phase_options.history_scored_seed_count = localGetField(userOptions, 'history_scored_seed_count', 2);
phase_options.max_prop_nudge_fraction = localGetField(userOptions, 'max_prop_nudge_fraction', 0.06);

phase_options.front_collective_min_rpm = localGetField(userOptions, 'front_collective_min_rpm', 0.0);
phase_options.rear_collective_min_rpm = localGetField(userOptions, 'rear_collective_min_rpm', 100.0);
phase_options.history_min_rear_collective_rpm = localGetField(userOptions, ...
    'history_min_rear_collective_rpm', phase_options.rear_collective_min_rpm);
phase_options.hover_anchor_case_name = localGetField(userOptions, ...
    'hover_anchor_case_name', 'TrimCase_Hover');
phase_options.cruise_anchor_case_name = localGetField(userOptions, ...
    'cruise_anchor_case_name', 'TrimCase_Cruise75_FlapElevator_Rear500');
phase_options.cruise_anchor_seed = localGetField(userOptions, 'cruise_anchor_seed', struct());

phase_options.history_exact_csv = localGetField(userOptions, 'history_exact_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv'));
phase_options.history_exact_csvs = localNormalizeCsvList(localGetField(userOptions, ...
    'history_exact_csvs', {phase_options.history_exact_csv}));

own_latest_csv = fullfile(root_dir, 'workspace_plots', [base_latest_prefix '_latest.csv']);
fallback_scored_csv = localGetField(userOptions, 'fallback_scored_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv'));
low_speed_scored_csv = localGetField(userOptions, 'low_speed_scored_csv', ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'));
scored_csvs = localNormalizeCsvList(localGetField(userOptions, 'history_scored_csvs', ...
    {low_speed_scored_csv, fallback_scored_csv}));
if exist(own_latest_csv, 'file') == 2
    scored_csvs = localNormalizeCsvList([{own_latest_csv}, scored_csvs]);
end
if isempty(scored_csvs)
    scored_csvs = {low_speed_scored_csv, fallback_scored_csv};
end
phase_options.history_scored_csv = scored_csvs{1};
phase_options.history_scored_csvs = scored_csvs;

phase_options.midband_vinf_range_mps = [20.0 50.0];
phase_options.midband_interp_tilt_window_deg = localGetField(userOptions, ...
    'midband_interp_tilt_window_deg', 22.5);
if phase_index <= 2
    phase_options.midband_interp_fracs = [0.25 0.5 0.75];
elseif phase_index <= 5
    phase_options.midband_interp_fracs = [0.15 0.25 0.35 0.5 0.65 0.75 0.85];
else
    phase_options.midband_interp_fracs = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9];
end
end

function summary = localReadSummaryOrEmpty(filename)
if exist(filename, 'file') ~= 2
    summary = table();
    return;
end
try
    summary = readtable(filename);
catch
    summary = table();
end
end

function localAppendPhaseLog(filename, phase_index, phase_options, summary)
fid = fopen(filename, 'a');
if fid < 0
    warning('Run_Trim_Transition_RearOn_Forever_Scored:PhaseLogWriteFailed', ...
        'Could not append phase log: %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '## Phase %d\n\n', phase_index);
fprintf(fid, '- Finished: %s\n', char(datetime('now', 'TimeZone', 'local', ...
    'Format', 'yyyy-MM-dd HH:mm:ss Z')));
fprintf(fid, '- Latest prefix: `%s`\n', phase_options.latest_prefix);
fprintf(fid, '- Offsets: `%s`\n', num2str(phase_options.reference_tilt_offsets_deg, ' %.2f'));
fprintf(fid, '- Vinf sample count: `%d`\n', numel(phase_options.reference_vinf_samples));
fprintf(fid, '- Max attempts: `%d`\n', phase_options.max_attempts);
fprintf(fid, '- Rear floor rpm: `%.1f`\n', phase_options.rear_collective_min_rpm);
fprintf(fid, '- Scored history csv count: `%d`\n', numel(phase_options.history_scored_csvs));
if isempty(summary)
    fprintf(fid, '- Latest summary: empty\n\n');
    return;
end
success_count = 0;
acceptable_count = 0;
if ismember('success', summary.Properties.VariableNames)
    success_count = nnz(localTableLogical(summary.success));
end
if ismember('acceptable', summary.Properties.VariableNames)
    acceptable_count = nnz(localTableLogical(summary.acceptable));
end
fprintf(fid, '- Summary rows: `%d`\n', height(summary));
fprintf(fid, '- Exact trims: `%d`\n', success_count);
fprintf(fid, '- Acceptable points: `%d`\n\n', acceptable_count);
end

function values = localTableLogical(column)
if islogical(column)
    values = column;
elseif isnumeric(column)
    values = column ~= 0;
elseif iscellstr(column) || isstring(column)
    values = ismember(lower(string(column)), ["1", "true", "yes"]);
else
    values = false(size(column));
end
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function csvList = localNormalizeCsvList(rawValue)
csvList = {};
if isempty(rawValue)
    return;
end
if ischar(rawValue) || isstring(rawValue)
    csvList = {char(rawValue)};
elseif iscell(rawValue)
    for i = 1:numel(rawValue)
        item = rawValue{i};
        if ischar(item) || isstring(item)
            csvList{end + 1} = char(item); %#ok<AGROW>
        end
    end
end
if isempty(csvList)
    return;
end
seen = strings(0, 1);
normalized = {};
for i = 1:numel(csvList)
    item = string(csvList{i});
    if strlength(item) == 0 || any(seen == item)
        continue;
    end
    seen(end + 1, 1) = item; %#ok<AGROW>
    normalized{end + 1} = char(item); %#ok<AGROW>
end
csvList = normalized;
end
