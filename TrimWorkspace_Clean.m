% TrimWorkspace_Clean.m
% Dry-run cleanup helper for transition-trim workspace outputs.
%
% Default behavior:
%   - scans workspace_plots
%   - classifies files/dirs into keep vs removable buckets
%   - prints a size summary
%   - does NOT delete anything unless explicitly enabled
%
% Optional workspace override:
%   transitionTrimCleanupOptions = struct( ...
%       'profile', 'standard', ...  % 'standard' or 'controller_db_lean'
%       'apply', false, ...
%       'delete_mode', 'trash', ...   % 'trash' or 'delete'
%       'keep_report_plots_final', true, ...
%       'keep_legacy_linearization_roots', false, ...
%       'include_nontrim_media', false, ...
%       'keep_controller_build_logs', false);
%
% Outputs left in base workspace:
%   - transitionTrimCleanupSummary

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

workspace_plots_dir = fullfile(root_dir, 'workspace_plots');
opts = localDefaultOptions();
if exist('transitionTrimCleanupOptions', 'var') && isstruct(transitionTrimCleanupOptions)
    opts = localMergeStruct(opts, transitionTrimCleanupOptions);
end
opts = localApplyProfile(opts);

items = dir(workspace_plots_dir);
items = items(~ismember({items.name}, {'.', '..'}));

records = struct( ...
    'name', {}, ...
    'full_path', {}, ...
    'is_dir', {}, ...
    'size_bytes', {}, ...
    'bucket', {}, ...
    'reason', {});

for i = 1:numel(items)
    item = items(i);
    full_path = fullfile(workspace_plots_dir, item.name);
    is_dir = item.isdir;
    size_bytes = localPathBytes(full_path, is_dir);
    [bucket, reason] = localClassifyItem(item.name, is_dir, opts);

    records(end + 1) = struct( ... %#ok<AGROW>
        'name', string(item.name), ...
        'full_path', string(full_path), ...
        'is_dir', is_dir, ...
        'size_bytes', size_bytes, ...
        'bucket', string(bucket), ...
        'reason', string(reason));
end

summary = localSummarize(records);
summary.workspace_plots_dir = string(workspace_plots_dir);
summary.apply = opts.apply;
summary.delete_mode = string(opts.delete_mode);

transitionTrimCleanupSummary = summary;
assignin('base', 'transitionTrimCleanupSummary', transitionTrimCleanupSummary);

localPrintSummary(summary);

if ~opts.apply
    fprintf('\nDry run only. No files were modified.\n');
    return;
end

recordBuckets = localRecordBuckets(records);
removable = records(ismember(recordBuckets, ["remove", "remove_media"]));
if isempty(removable)
    fprintf('\nNothing matched the removal policy.\n');
    return;
end

switch lower(string(opts.delete_mode))
    case "trash"
        archive_root = fullfile(root_dir, 'archive', ...
            ['trim_workspace_cleanup_' char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'))]);
        if exist(archive_root, 'dir') ~= 7
            mkdir(archive_root);
        end
        for i = 1:numel(removable)
            source = char(removable(i).full_path);
            target = fullfile(archive_root, char(removable(i).name));
            movefile(source, target);
        end
        fprintf('\nMoved %d items into:\n%s\n', numel(removable), archive_root);
    case "delete"
        for i = 1:numel(removable)
            target = char(removable(i).full_path);
            if removable(i).is_dir
                rmdir(target, 's');
            else
                delete(target);
            end
        end
        fprintf('\nDeleted %d items.\n', numel(removable));
    otherwise
        error('TrimWorkspace_Clean:BadDeleteMode', ...
            'Unsupported delete_mode "%s".', string(opts.delete_mode));
end

function opts = localDefaultOptions()
opts = struct();
opts.profile = 'standard';
opts.apply = false;
opts.delete_mode = 'trash';
opts.keep_report_plots_final = true;
opts.keep_legacy_linearization_roots = false;
opts.include_nontrim_media = false;
opts.keep_selected_trim_source_latest_mats = {};
opts.keep_selected_trim_source_latest_mats_explicit = false;
opts.keep_master_db_csv_md = true;
opts.keep_controller_db_csv_md = true;
opts.keep_controller_build_logs = false;
opts.keep_canonical_linearization_index_files = true;
opts.keep_canonical_linearization_root = true;
opts.keep_backfill_index_files = false;
opts.keep_backfill_summary_files = false;
opts.keep_backfill_linearization_root = false;
opts.keep_trim_map_linearizations = false;
opts.strict_keep_manifest = false;
end

function [bucket, reason] = localClassifyItem(name, is_dir, opts)
name = string(name);

% Durable trim/controller DBs now live in databases/. If old copies appear
% in workspace_plots/, they are treated as misplaced generated artifacts.
misplaced_db_exact = [ ...
    "trim_attempts.mat"; ...
    "trim_attempts.csv"; ...
    "trim_attempts.md"; ...
    "controller_schedule.mat"; ...
    "controller_schedule.csv"; ...
    "controller_schedule.md"; ...
    "controller_schedule_missing_linearizations.csv"; ...
    "controller_schedule_build_log.md"; ...
    "transition_trim_linearization_index.csv"; ...
    "transition_trim_linearization_index.mat"];

keep_exact = strings(0, 1);
if opts.keep_backfill_index_files
    keep_exact = [keep_exact; ...
        "transition_trim_linearization_backfill_index.csv"; ...
        "transition_trim_linearization_backfill_index.mat"; ...
        "transition_trim_linearization_backfill_latest.mat"];
end
if opts.keep_backfill_summary_files
    keep_exact = [keep_exact; ...
        "transition_trim_linearization_backfill_summary_latest.csv"; ...
        "transition_trim_linearization_backfill_summary_latest.md"];
end
if opts.keep_trim_map_linearizations
    keep_exact = [keep_exact; ...
        "trim_map_linearizations_latest.csv"; ...
        "trim_map_linearizations_latest.mat"; ...
        "trim_map_linearizations_latest.md"];
end

keep_dirs = strings(0, 1);
if opts.keep_backfill_linearization_root
    keep_dirs(end + 1, 1) = "transition_trim_linearization_backfill_linearizations"; %#ok<AGROW>
end
if opts.keep_report_plots_final
    keep_dirs(end + 1, 1) = "report_plots_final"; %#ok<AGROW>
end

if any(name == misplaced_db_exact) || (is_dir && name == "transition_trim_linearizations")
    bucket = "remove";
    reason = "misplaced_database_artifact_databases_is_canonical";
    return;
end

if any(name == keep_exact)
    bucket = "keep";
    reason = "canonical_database_or_summary";
    return;
end
if is_dir && any(name == keep_dirs)
    if name == "report_plots_final" && ~opts.keep_report_plots_final
        bucket = "remove_media";
        reason = "report_plots_disabled_by_option";
    else
        bucket = "keep";
        reason = "canonical_directory";
    end
    return;
end

selectedLatestMats = string(opts.keep_selected_trim_source_latest_mats(:));
if ~is_dir && any(name == selectedLatestMats)
    bucket = "keep";
    reason = "selected_trim_source_latest_mat";
    return;
end

if opts.keep_legacy_linearization_roots && ...
        (startsWith(name, "transition_trim_rearon_connector_forever_linearization_") || ...
         name == "transition_trim_rearon_connector_forever_linearizations")
    bucket = "keep";
    reason = "legacy_linearization_root";
    return;
end

if startsWith(name, "transition_trim_") && is_dir && ~isempty(regexp(char(name), '_\d{8}_\d{6}$', 'once'))
    bucket = "remove";
    reason = "timestamped_trim_run_directory";
    return;
end

if startsWith(name, "transition_trim_") && ~is_dir && endsWith(name, "_latest.mat")
    bucket = "remove";
    reason = "run_local_latest_mat";
    return;
end

if startsWith(name, "transition_trim_") && ~is_dir && ...
        (endsWith(name, "_latest.csv") || endsWith(name, "_latest.md"))
    bucket = "remove";
    reason = "run_local_latest_text";
    return;
end

if contains(name, "visuals_") && is_dir
    bucket = "remove";
    reason = "generated_visual_directory";
    return;
end

if contains(lower(name), "smoke")
    bucket = "remove";
    reason = "smoke_artifact";
    return;
end

if endsWith(name, "_launcher.log") || endsWith(name, "_live.log")
    bucket = "remove";
    reason = "launcher_or_live_log";
    return;
end

if startsWith(name, "trim_linearization_db_") || name == "trim_map_linearizations_smoketest"
    bucket = "remove";
    reason = "legacy_linearization_experiment";
    return;
end

if ~opts.include_nontrim_media && ...
        (~is_dir && (endsWith(name, ".mp4") || endsWith(name, ".mov")))
    bucket = "remove_media";
    reason = "generated_media";
    return;
end

if opts.strict_keep_manifest
    bucket = "remove";
    reason = "strict_profile_unmatched";
else
    bucket = "keep";
    reason = "unmatched_keep_by_default";
end
end

function bytes = localPathBytes(pathStr, is_dir)
if ~is_dir
    info = dir(pathStr);
    bytes = info.bytes;
    return;
end

listing = dir(fullfile(pathStr, '**', '*'));
listing = listing(~[listing.isdir]);
bytes = sum([listing.bytes]);
end

function summary = localSummarize(records)
summary = struct();
summary.total_items = numel(records);
if isempty(records)
    summary.total_size_bytes = 0;
    summary.keep_count = 0;
    summary.remove_count = 0;
    summary.remove_media_count = 0;
    summary.keep_size_bytes = 0;
    summary.remove_size_bytes = 0;
    summary.remove_media_size_bytes = 0;
    summary.records = records;
    return;
end
buckets = localRecordBuckets(records);
summary.total_size_bytes = sum([records.size_bytes]);
summary.keep_count = nnz(ismember(buckets, "keep"));
summary.remove_count = nnz(ismember(buckets, "remove"));
summary.remove_media_count = nnz(ismember(buckets, "remove_media"));
summary.keep_size_bytes = sum([records(ismember(buckets, "keep")).size_bytes]);
summary.remove_size_bytes = sum([records(ismember(buckets, "remove")).size_bytes]);
summary.remove_media_size_bytes = sum([records(ismember(buckets, "remove_media")).size_bytes]);
summary.records = records;
end

function buckets = localRecordBuckets(records)
if isempty(records)
    buckets = strings(0, 1);
else
    buckets = string({records.bucket});
end
end

function localPrintSummary(summary)
fprintf('Transition trim workspace cleanup scan\n');
fprintf('workspace_plots: %s\n\n', summary.workspace_plots_dir);
fprintf('Total items:        %4d   (%6.2f GB)\n', summary.total_items, summary.total_size_bytes / 1e9);
fprintf('Keep:               %4d   (%6.2f GB)\n', summary.keep_count, summary.keep_size_bytes / 1e9);
fprintf('Remove:             %4d   (%6.2f GB)\n', summary.remove_count, summary.remove_size_bytes / 1e9);
fprintf('Remove media:       %4d   (%6.2f GB)\n', summary.remove_media_count, summary.remove_media_size_bytes / 1e9);

top = summary.records(~ismember(localRecordBuckets(summary.records), "keep"));
if isempty(top)
    return;
end

[~, idx] = sort([top.size_bytes], 'descend');
top = top(idx);
fprintf('\nLargest removable items:\n');
for i = 1:min(20, numel(top))
    fprintf('  %6.2f GB  %-14s  %s\n', top(i).size_bytes / 1e9, top(i).bucket, top(i).name);
end
end

function out = localMergeStruct(defaults, overrides)
out = defaults;
if isempty(overrides)
    return;
end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    out.(fields{i}) = overrides.(fields{i});
end
if isfield(overrides, 'keep_selected_trim_source_latest_mats')
    out.keep_selected_trim_source_latest_mats_explicit = true;
end
end

function opts = localApplyProfile(opts)
profile = lower(string(opts.profile));
switch profile
    case "standard"
        % Keep the current broad dry-run behavior.
    case "controller_db_lean"
        % Keep only the current master/controller DBs and canonical direct
        % linearization artifacts. Do not preserve old latest/backfill
        % replay products in the active workspace.
        opts.keep_report_plots_final = false;
        opts.keep_legacy_linearization_roots = false;
        opts.keep_master_db_csv_md = true;
        opts.keep_controller_db_csv_md = true;
        opts.keep_controller_build_logs = false;
        opts.keep_canonical_linearization_index_files = true;
        opts.keep_canonical_linearization_root = true;
        opts.keep_backfill_index_files = false;
        opts.keep_backfill_summary_files = false;
        opts.keep_backfill_linearization_root = false;
        opts.keep_trim_map_linearizations = false;
        opts.strict_keep_manifest = true;
        if ~opts.keep_selected_trim_source_latest_mats_explicit
            opts.keep_selected_trim_source_latest_mats = {};
        end
    otherwise
        error('TrimWorkspace_Clean:BadProfile', ...
            'Unsupported cleanup profile "%s".', string(opts.profile));
end
end
