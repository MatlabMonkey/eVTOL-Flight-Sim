function paths = TrimDB_Paths(rootDir)
% trim DB paths

if nargin < 1 || isempty(rootDir)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        rootDir = fileparts(stack(1).file);
    else
        rootDir = pwd;
    end
end

paths = struct();
paths.root_dir = rootDir;
paths.database_dir = fullfile(rootDir, 'databases');
paths.workspace_plots_dir = fullfile(rootDir, 'workspace_plots');

paths.master_mat_file = fullfile(paths.database_dir, 'trim_attempts.mat');
paths.master_csv_file = fullfile(paths.database_dir, 'trim_attempts.csv');
paths.master_md_file = fullfile(paths.database_dir, 'trim_attempts.md');

paths.controller_mat_file = fullfile(paths.database_dir, 'controller_schedule.mat');
paths.controller_csv_file = fullfile(paths.database_dir, 'controller_schedule.csv');
paths.controller_md_file = fullfile(paths.database_dir, 'controller_schedule.md');
paths.controller_missing_linearizations_csv_file = fullfile(paths.database_dir, ...
    'controller_schedule_missing_linearizations.csv');
paths.controller_build_log_file = fullfile(paths.database_dir, ...
    'controller_schedule_build_log.md');

paths.linearization_root = fullfile(paths.database_dir, 'transition_trim_linearizations');
paths.linearization_index_csv = fullfile(paths.database_dir, ...
    'transition_trim_linearization_index.csv');
paths.linearization_index_mat = fullfile(paths.database_dir, ...
    'transition_trim_linearization_index.mat');

paths.backfill_linearization_root = fullfile(paths.database_dir, ...
    'transition_trim_linearization_backfill_linearizations');
paths.backfill_linearization_index_csv = fullfile(paths.database_dir, ...
    'transition_trim_linearization_backfill_index.csv');
paths.backfill_linearization_index_mat = fullfile(paths.database_dir, ...
    'transition_trim_linearization_backfill_index.mat');
end
