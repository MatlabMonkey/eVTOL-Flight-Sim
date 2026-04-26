% Rebuild_Transition_Trim_Global_Attempt_Db.m
% Build the shared transition trim attempt database from the current set of
% latest CSV outputs so new runners can seed/skip against one common store.

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

workspacePlots = fullfile(root_dir, 'workspace_plots');
candidateCsvs = { ...
    fullfile(workspacePlots, 'transition_trim_map_merged_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_map_low_speed_scored_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_reference_line_scored_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_reference_line_midband_scored_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_lowmid_guidegrid_scored_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_lowmid_guidegrid_fast_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_leftbridge_fast_latest.csv'), ...
    fullfile(workspacePlots, 'transition_trim_path_scored_latest.csv') };

outCsv = fullfile(workspacePlots, 'transition_trim_global_attempt_db_latest.csv');
outMat = fullfile(workspacePlots, 'transition_trim_global_attempt_db_latest.mat');
outMd = fullfile(workspacePlots, 'transition_trim_global_attempt_db_latest.md');

summary = localEmptySummary();

for i = 1:numel(candidateCsvs)
    candidate = candidateCsvs{i};
    if exist(candidate, 'file') ~= 2
        continue;
    end
    part = readtable(candidate, 'TextType', 'string');
    part = localNormalize(part, candidate);
    if isempty(part)
        continue;
    end
    summary = [summary; part]; %#ok<AGROW>
end

if ~isempty(summary)
    ids = summary.source_run_dir + "|" + summary.name + "|" + summary.family + "|" + summary.seed_name;
    [~, keepIdx] = unique(ids, 'stable');
    summary = summary(keepIdx, :);
end

save(outMat, 'summary', '-v7.3');
if ~isempty(summary)
    writetable(summary, outCsv);
end
localWriteMarkdown(summary, outMd);

fprintf('Rebuilt global transition trim DB.\n');
fprintf('  rows = %d\n', height(summary));
fprintf('  csv = %s\n', outCsv);

function summary = localNormalize(part, sourceCsv)
summary = localEmptySummary();
if isempty(part)
    return;
end
requiredNames = summary.Properties.VariableNames;
for i = 1:numel(requiredNames)
    name = requiredNames{i};
    if ismember(name, part.Properties.VariableNames)
        continue;
    end
    switch name
        case {'key','name','family','seed_name','classification','worst_component','termination_string','source_run_prefix','source_run_dir','saved_on'}
            part.(name) = repmat("", height(part), 1);
        case {'success','acceptable'}
            part.(name) = false(height(part), 1);
        otherwise
            part.(name) = nan(height(part), 1);
    end
end
part.source_run_prefix = repmat(string(localSourcePrefix(sourceCsv)), height(part), 1);
part.source_run_dir = repmat(string(fileparts(sourceCsv)), height(part), 1);
part.saved_on = repmat(string(char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'))), height(part), 1);
if all(part.key == "")
    part.key = arrayfun(@(t,v) string(sprintf('tilt_%s_v_%s', localValueLabel(t), localValueLabel(v))), ...
        part.tilt_deg, part.vinf_mps);
end
summary = part(:, requiredNames);
end

function prefix = localSourcePrefix(pathStr)
[~, name, ~] = fileparts(pathStr);
prefix = name;
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end

function summary = localEmptySummary()
summary = table();
summary.key = strings(0,1);
summary.name = strings(0,1);
summary.tilt_deg = zeros(0,1);
summary.vinf_mps = zeros(0,1);
summary.family = strings(0,1);
summary.seed_name = strings(0,1);
summary.success = false(0,1);
summary.acceptable = false(0,1);
summary.classification = strings(0,1);
summary.score = zeros(0,1);
summary.max_normalized = zeros(0,1);
summary.worst_component = strings(0,1);
summary.worst_component_normalized = zeros(0,1);
summary.front_collective_rpm = zeros(0,1);
summary.rear_collective_rpm = zeros(0,1);
summary.delta_f_deg = zeros(0,1);
summary.delta_a_deg = zeros(0,1);
summary.delta_e_deg = zeros(0,1);
summary.delta_r_deg = zeros(0,1);
summary.theta_deg = zeros(0,1);
summary.u_mps = zeros(0,1);
summary.w_mps = zeros(0,1);
summary.alpha_deg = zeros(0,1);
summary.termination_string = strings(0,1);
summary.source_run_prefix = strings(0,1);
summary.source_run_dir = strings(0,1);
summary.saved_on = strings(0,1);
end

function localWriteMarkdown(summary, filename)
fid = fopen(filename, 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Global Transition Trim Attempt DB\n\n');
fprintf(fid, '- Total rows: %d\n', height(summary));
if isempty(summary)
    return;
end
fprintf(fid, '- Exact trims: %d\n', nnz(summary.success));
fprintf(fid, '- Acceptable near-trims: %d\n', nnz(summary.acceptable));
fprintf(fid, '- Unique sources: %d\n\n', numel(unique(summary.source_run_prefix)));
end
