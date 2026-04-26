% Run_Trim_Transition_RearOn_Connector_Forever.m
% 1) Search for a rear-on cruise anchor near the desired endpoint.
% 2) Use that anchor to drive the rear-on forever corridor search.
%
% Stable outputs:
%   workspace_plots/rear_on_cruise_anchor_latest.csv
%   workspace_plots/rear_on_cruise_anchor_latest.mat
%   workspace_plots/transition_trim_rearon_connector_forever_latest.csv
%   workspace_plots/transition_trim_rearon_connector_forever_latest.mat

if ~exist('transitionRearOnConnectorOptions', 'var') || ~isstruct(transitionRearOnConnectorOptions)
    transitionRearOnConnectorOptions = struct();
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

initOptions = struct();
initOptions.transitionRearOnConnectorOptions = transitionRearOnConnectorOptions;
Init_EVTOL_Main

if exist('initOptions', 'var') && isstruct(initOptions) && isfield(initOptions, 'transitionRearOnConnectorOptions')
    transitionRearOnConnectorOptions = initOptions.transitionRearOnConnectorOptions;
else
    transitionRearOnConnectorOptions = struct();
end

anchorOptions = localGetField(transitionRearOnConnectorOptions, 'anchor_options', struct());
anchorCfg = localBuildAnchorConfig(anchorOptions, root_dir);

timestamp = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyyMMdd_HHmmss'));
anchorOutputDir = fullfile(root_dir, 'workspace_plots', sprintf('%s_%s', anchorCfg.output_prefix, timestamp));
if exist(anchorOutputDir, 'dir') ~= 7
    mkdir(anchorOutputDir);
end

anchorSummaryCsv = fullfile(anchorOutputDir, 'rear_on_cruise_anchor_summary.csv');
anchorSummaryMd = fullfile(anchorOutputDir, 'rear_on_cruise_anchor_summary.md');
anchorCheckpointMat = fullfile(anchorOutputDir, 'rear_on_cruise_anchor.mat');
anchorLatestCsv = fullfile(root_dir, 'workspace_plots', [anchorCfg.latest_prefix '_latest.csv']);
anchorLatestMd = fullfile(root_dir, 'workspace_plots', [anchorCfg.latest_prefix '_latest.md']);
anchorLatestMat = fullfile(root_dir, 'workspace_plots', [anchorCfg.latest_prefix '_latest.mat']);
anchorSelectedLinearizationMat = fullfile(anchorOutputDir, 'rear_on_cruise_anchor_selected_linearization.mat');
anchorSelectedLinearizationLatestMat = fullfile(root_dir, 'workspace_plots', ...
    [anchorCfg.latest_prefix '_selected_linearization_latest.mat']);

hoverCase = TrimCase_Hover();
cruiseCase = TrimCase_Cruise75_FlapElevator_Rear500();
hoverSeed = localSeedFromCase(hoverCase);
cruiseSeed = localSeedFromCase(cruiseCase);
anchorCandidateSpecs = localBuildAnchorCandidates(anchorCfg, hoverSeed, cruiseSeed);
anchorRows = repmat(localAnchorRowTemplate(), 0, 1);
anchorCandidateResults = repmat(localAnchorResultTemplate(), 0, 1);

fprintf('=== Run_Trim_Transition_RearOn_Connector_Forever ===\n');
fprintf('Searching rear-on cruise anchors: %d candidates\n', numel(anchorCandidateSpecs));

for iAnchor = 1:numel(anchorCandidateSpecs)
    spec = anchorCandidateSpecs(iAnchor);
    trimCase = localBuildAnchorTrimCase(spec, anchorCfg, cruiseCase);
    trimResult = trim_evtol_case(initData, trimCase, anchorCfg.trim_options);
    scoreData = score_trim_point(trimResult, anchorCfg.score_options);
    anchorRows(end + 1, 1) = localBuildAnchorRow(spec, trimResult, scoreData, hoverSeed, anchorCfg); %#ok<SAGROW>
    anchorCandidateResults(end + 1, 1) = localBuildAnchorResult(spec, trimCase, trimResult, scoreData); %#ok<SAGROW>
end

rearOnCruiseAnchorSummary = localBuildAnchorSummary(anchorRows);
bestAnchorIdx = localSelectBestAnchor(rearOnCruiseAnchorSummary);
rearOnCruiseAnchorSummary.selected = false(height(rearOnCruiseAnchorSummary), 1);
rearOnCruiseAnchorSummary.selected(bestAnchorIdx) = true;
bestCruiseAnchorSeed = localSeedFromAnchorSummaryRow(rearOnCruiseAnchorSummary(bestAnchorIdx, :));

rearOnCruiseAnchorSearch = struct();
rearOnCruiseAnchorSearch.config = anchorCfg;
rearOnCruiseAnchorSearch.output_dir = anchorOutputDir;
rearOnCruiseAnchorSearch.summary_csv = anchorSummaryCsv;
rearOnCruiseAnchorSearch.latest_csv = anchorLatestCsv;
rearOnCruiseAnchorSearch.latest_md = anchorLatestMd;
rearOnCruiseAnchorSearch.latest_mat = anchorLatestMat;
rearOnCruiseAnchorSearch.summary_table = rearOnCruiseAnchorSummary;
rearOnCruiseAnchorSearch.best_index = bestAnchorIdx;
rearOnCruiseAnchorSearch.best_row = rearOnCruiseAnchorSummary(bestAnchorIdx, :);
rearOnCruiseAnchorSearch.best_seed = bestCruiseAnchorSeed;
rearOnCruiseAnchorSearch.results = anchorCandidateResults;
rearOnCruiseAnchorSearch.created_on = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));

rearOnCruiseAnchorSelectedLinearization = localBuildAnchorLinearizationPoint( ...
    anchorCandidateResults(bestAnchorIdx), rearOnCruiseAnchorSummary(bestAnchorIdx, :));

save(anchorCheckpointMat, 'rearOnCruiseAnchorSearch', '-v7.3');
save(anchorLatestMat, 'rearOnCruiseAnchorSearch', '-v7.3');
save(anchorSelectedLinearizationMat, 'rearOnCruiseAnchorSelectedLinearization', '-v7.3');
save(anchorSelectedLinearizationLatestMat, 'rearOnCruiseAnchorSelectedLinearization', '-v7.3');
writetable(rearOnCruiseAnchorSummary, anchorSummaryCsv);
writetable(rearOnCruiseAnchorSummary, anchorLatestCsv);
localWriteAnchorSummaryMarkdown(rearOnCruiseAnchorSummary, anchorSummaryMd, rearOnCruiseAnchorSearch);
localWriteAnchorSummaryMarkdown(rearOnCruiseAnchorSummary, anchorLatestMd, rearOnCruiseAnchorSearch);

fprintf('Selected cruise anchor: %s | class=%s | front=%.1f | rear=%.1f | merit=%.4f\n', ...
    bestCruiseAnchorSeed.name, rearOnCruiseAnchorSummary.classification(bestAnchorIdx), ...
    bestCruiseAnchorSeed.front_collective_guess_rpm, bestCruiseAnchorSeed.rear_collective_guess_rpm, ...
    rearOnCruiseAnchorSummary.anchor_merit(bestAnchorIdx));

transitionRearOnForeverOptions = localGetField(transitionRearOnConnectorOptions, 'forever_options', struct());
transitionRearOnForeverOptions.base_output_prefix = localGetField(transitionRearOnForeverOptions, ...
    'base_output_prefix', 'transition_trim_rearon_connector_forever');
transitionRearOnForeverOptions.base_latest_prefix = localGetField(transitionRearOnForeverOptions, ...
    'base_latest_prefix', 'transition_trim_rearon_connector_forever');
transitionRearOnForeverOptions.front_collective_min_rpm = localGetField(transitionRearOnForeverOptions, ...
    'front_collective_min_rpm', 0.0);
transitionRearOnForeverOptions.rear_collective_min_rpm = localGetField(transitionRearOnForeverOptions, ...
    'rear_collective_min_rpm', 100.0);
transitionRearOnForeverOptions.history_min_rear_collective_rpm = localGetField(transitionRearOnForeverOptions, ...
    'history_min_rear_collective_rpm', transitionRearOnForeverOptions.rear_collective_min_rpm);
transitionRearOnForeverOptions.hover_anchor_case_name = localGetField(transitionRearOnForeverOptions, ...
    'hover_anchor_case_name', 'TrimCase_Hover');
transitionRearOnForeverOptions.cruise_anchor_case_name = localGetField(transitionRearOnForeverOptions, ...
    'cruise_anchor_case_name', 'TrimCase_Cruise75_FlapElevator_Rear500');
transitionRearOnForeverOptions.cruise_anchor_seed = bestCruiseAnchorSeed;
transitionRearOnForeverOptions.history_exact_csvs = localGetField(transitionRearOnForeverOptions, ...
    'history_exact_csvs', {anchorLatestCsv, fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv')});
transitionRearOnForeverOptions.history_scored_csvs = localGetField(transitionRearOnForeverOptions, ...
    'history_scored_csvs', { ...
        fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'), ...
        fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv')});
transitionRearOnForeverOptions.reference_knot_vinf_mps = localGetField(transitionRearOnForeverOptions, ...
    'reference_knot_vinf_mps', [0 2.5 5 10 15 20 25 30 35 40 45 50 60 70]);
transitionRearOnForeverOptions.reference_knot_tilt_deg = localGetField(transitionRearOnForeverOptions, ...
    'reference_knot_tilt_deg', [0 5 10 20 30 40 50 60 67 73 78 82 87 90]);
transitionRearOnForeverOptions.phase_max_attempts = localGetField(transitionRearOnForeverOptions, ...
    'phase_max_attempts', 250);
transitionRearOnForeverOptions.offset_step_deg = localGetField(transitionRearOnForeverOptions, ...
    'offset_step_deg', 2.5);
transitionRearOnForeverOptions.max_offset_radius_deg = localGetField(transitionRearOnForeverOptions, ...
    'max_offset_radius_deg', 20.0);
transitionRearOnForeverOptions.midband_interp_fracs = localGetField(transitionRearOnForeverOptions, ...
    'midband_interp_fracs', [0.2 0.35 0.5 0.65 0.8]);

Run_Trim_Transition_RearOn_Forever_Scored

function cfg = localBuildAnchorConfig(opts, root_dir)
cfg = struct();
cfg.root_dir = root_dir;
cfg.output_prefix = localGetField(opts, 'output_prefix', 'rear_on_cruise_anchor');
cfg.latest_prefix = localGetField(opts, 'latest_prefix', 'rear_on_cruise_anchor');
cfg.vinf_grid = localGetField(opts, 'vinf_grid', [67.5 70.0 72.5 75.0]);
cfg.tilt_grid = localGetField(opts, 'tilt_grid', [87.5 90.0]);
cfg.rear_fixed_grid_rpm = localGetField(opts, 'rear_fixed_grid_rpm', 250:100:750);
cfg.rear_min_rpm = localGetField(opts, 'rear_min_rpm', 100.0);
cfg.goal_vinf_mps = localGetField(opts, 'goal_vinf_mps', 70.0);
cfg.goal_tilt_deg = localGetField(opts, 'goal_tilt_deg', 90.0);
cfg.max_candidates = localGetField(opts, 'max_candidates', 100);
cfg.front_adjust_per_rear_rpm = localGetField(opts, 'front_adjust_per_rear_rpm', 0.45);
cfg.trim_options = struct('verbose', false, 'debug', false, 'emitSummary', false, 'emitLinearSummary', false);
cfg.score_options = struct('profile', 'transition', 'hold_horizon_s', 2.0);
end

function candidateSpecs = localBuildAnchorCandidates(cfg, hoverSeed, cruiseSeed)
candidateSpecs = repmat(localAnchorCandidateTemplate(), 0, 1);
families = ["rear_fixed_flap_elevator", "front_rear_free_flap_elevator"];
for iV = 1:numel(cfg.vinf_grid)
    vinf_mps = cfg.vinf_grid(iV);
    for iT = 1:numel(cfg.tilt_grid)
        tilt_deg = cfg.tilt_grid(iT);
        gamma = min(1.0, max(0.0, 0.5 * (vinf_mps / max(cfg.goal_vinf_mps, eps) + tilt_deg / max(cfg.goal_tilt_deg, eps))));
        frontBase = hoverSeed.front_collective_guess_rpm + ...
            gamma * (cruiseSeed.front_collective_guess_rpm - hoverSeed.front_collective_guess_rpm);
        thetaBase = hoverSeed.theta_guess_deg + gamma * (cruiseSeed.theta_guess_deg - hoverSeed.theta_guess_deg);
        deltaFBase = hoverSeed.delta_f_guess_deg + gamma * (cruiseSeed.delta_f_guess_deg - hoverSeed.delta_f_guess_deg);
        deltaEBase = hoverSeed.delta_e_guess_deg + gamma * (cruiseSeed.delta_e_guess_deg - hoverSeed.delta_e_guess_deg);
        for iR = 1:numel(cfg.rear_fixed_grid_rpm)
            rearFixed = cfg.rear_fixed_grid_rpm(iR);
            frontGuess = frontBase - cfg.front_adjust_per_rear_rpm * (rearFixed - cruiseSeed.rear_collective_guess_rpm);
            frontGuess = min(max(frontGuess, 550.0), 1400.0);
            for iF = 1:numel(families)
                spec = localAnchorCandidateTemplate();
                spec.name = sprintf('RearOnAnchor_V%s_Tilt%s_R%s_%s', ...
                    localValueLabel(vinf_mps), localValueLabel(tilt_deg), localValueLabel(rearFixed), families(iF));
                spec.family = char(families(iF));
                spec.vinf_mps = vinf_mps;
                spec.tilt_deg = tilt_deg;
                spec.front_guess_rpm = frontGuess;
                spec.rear_guess_rpm = rearFixed;
                spec.rear_fixed_rpm = rearFixed;
                spec.theta_guess_deg = thetaBase;
                spec.delta_f_guess_deg = deltaFBase;
                spec.delta_e_guess_deg = deltaEBase;
                candidateSpecs(end + 1, 1) = spec; %#ok<SAGROW>
            end
        end
    end
end
if numel(candidateSpecs) > cfg.max_candidates
    candidateSpecs = candidateSpecs(1:cfg.max_candidates);
end
end

function trimCase = localBuildAnchorTrimCase(spec, cfg, cruiseCase)
trimCase = cruiseCase;
trimCase.name = spec.name;
trimCase.mode = 'cruise_anchor_search';
trimCase.Vinf_mps = spec.vinf_mps;
trimCase.u_body_mps = spec.vinf_mps;
trimCase.v_body_mps = 0.0;
trimCase.w_body_mps = 0.0;
trimCase.front_tilt_deg = spec.tilt_deg;
trimCase.front_tilt_cmd_deg = spec.tilt_deg;
trimCase.theta_guess_deg = spec.theta_guess_deg;
trimCase.front_collective_guess_rpm = spec.front_guess_rpm;
trimCase.rear_collective_guess_rpm = spec.rear_guess_rpm;
trimCase.front_collective_min_rpm = 0.0;
trimCase.rear_collective_min_rpm = cfg.rear_min_rpm;
trimCase.validate_nonlinear_hold = false;
trimCase.position_steady = [false; false; false];
trimCase.delta_f_guess_deg = spec.delta_f_guess_deg;
trimCase.delta_e_guess_deg = spec.delta_e_guess_deg;
trimCase.delta_a_fixed_deg = 0.0;
trimCase.delta_r_fixed_deg = 0.0;
trimCase.mixed_control_known = [false; true; false; true];
trimCase.use_vinf_output_constraint = true;
trimCase.use_vertical_speed_output_constraint = true;
switch spec.family
    case 'rear_fixed_flap_elevator'
        trimCase.front_collective_known = false;
        trimCase.rear_collective_known = true;
        trimCase.rear_collective_fixed_rpm = spec.rear_fixed_rpm;
    case 'front_rear_free_flap_elevator'
        trimCase.front_collective_known = false;
        trimCase.rear_collective_known = false;
    otherwise
        error('Unknown anchor family "%s".', spec.family);
end
end

function row = localBuildAnchorRow(spec, trimResult, scoreData, hoverSeed, cfg)
row = localAnchorRowTemplate();
row.name = spec.name;
row.family = spec.family;
row.target_vinf_mps = spec.vinf_mps;
row.target_tilt_deg = spec.tilt_deg;
row.front_guess_rpm = spec.front_guess_rpm;
row.rear_guess_rpm = spec.rear_guess_rpm;
row.rear_fixed_rpm = spec.rear_fixed_rpm;
row.success = localGetField(trimResult, 'success', false);
row.acceptable = localGetField(scoreData, 'acceptable', false);
row.classification = string(localGetField(scoreData, 'classification', 'not_scored'));
row.score = localGetField(scoreData, 'score', inf);
row.max_normalized = localGetField(scoreData, 'max_normalized', inf);
row.termination_string = string(localGetField(trimResult, 'terminationString', ''));
if isstruct(trimResult) && isfield(trimResult, 'scheduling')
    row.front_collective_rpm = localGetField(trimResult.scheduling, 'front_collective_rpm', NaN);
    row.rear_collective_rpm = localGetField(trimResult.scheduling, 'rear_collective_rpm', NaN);
    row.delta_f_deg = rad2deg(localGetField(trimResult.scheduling, 'delta_f_rad', NaN));
    row.delta_e_deg = rad2deg(localGetField(trimResult.scheduling, 'delta_e_rad', NaN));
end
if isstruct(trimResult) && isfield(trimResult, 'Att_Trim_deg') && numel(trimResult.Att_Trim_deg) >= 2
    row.theta_deg = trimResult.Att_Trim_deg(2);
end
if ~isfinite(row.front_collective_rpm)
    row.front_collective_rpm = spec.front_guess_rpm;
end
if ~isfinite(row.rear_collective_rpm)
    row.rear_collective_rpm = spec.rear_guess_rpm;
end
row.goal_distance = abs(row.target_vinf_mps - cfg.goal_vinf_mps) / 2.5 + ...
    abs(row.target_tilt_deg - cfg.goal_tilt_deg) / 2.5;
row.prop_jump_from_hover = hypot( ...
    (row.front_collective_rpm - hoverSeed.front_collective_guess_rpm) / max(hoverSeed.front_collective_guess_rpm, 1.0), ...
    (row.rear_collective_rpm - hoverSeed.rear_collective_guess_rpm) / max(hoverSeed.rear_collective_guess_rpm, 1.0));
row.control_effort_deg = abs(row.delta_f_deg) + abs(row.delta_e_deg);
row.anchor_merit = localAnchorMerit(row);
end

function result = localBuildAnchorResult(spec, trimCase, trimResult, scoreData)
result = localAnchorResultTemplate();
result.spec = spec;
result.trimCase = trimCase;
result.trimResult = trimResult;
result.scoreData = scoreData;
end

function summary = localBuildAnchorSummary(rows)
summary = table();
summary.name = string({rows.name}');
summary.family = string({rows.family}');
summary.target_vinf_mps = [rows.target_vinf_mps]';
summary.target_tilt_deg = [rows.target_tilt_deg]';
summary.front_guess_rpm = [rows.front_guess_rpm]';
summary.rear_guess_rpm = [rows.rear_guess_rpm]';
summary.rear_fixed_rpm = [rows.rear_fixed_rpm]';
summary.success = logical([rows.success]');
summary.acceptable = logical([rows.acceptable]');
summary.classification = string({rows.classification}');
summary.score = [rows.score]';
summary.max_normalized = [rows.max_normalized]';
summary.front_collective_rpm = [rows.front_collective_rpm]';
summary.rear_collective_rpm = [rows.rear_collective_rpm]';
summary.delta_f_deg = [rows.delta_f_deg]';
summary.delta_e_deg = [rows.delta_e_deg]';
summary.theta_deg = [rows.theta_deg]';
summary.goal_distance = [rows.goal_distance]';
summary.prop_jump_from_hover = [rows.prop_jump_from_hover]';
summary.control_effort_deg = [rows.control_effort_deg]';
summary.anchor_merit = [rows.anchor_merit]';
summary.termination_string = string({rows.termination_string}');
summary.selected = false(height(summary), 1);
[~, order] = sortrows([summary.anchor_merit, summary.goal_distance, summary.score], [1 2 3]);
summary = summary(order, :);
end

function idx = localSelectBestAnchor(summary)
if isempty(summary)
    error('Run_Trim_Transition_RearOn_Connector_Forever:NoAnchorCandidates', ...
        'No rear-on cruise anchor candidates were evaluated.');
end
idx = 1;
end

function seed = localSeedFromAnchorSummaryRow(row)
seed = struct();
seed.name = char(row.name);
seed.source = 'rear_on_cruise_anchor';
seed.tilt_deg = row.target_tilt_deg;
seed.vinf_mps = row.target_vinf_mps;
seed.front_collective_guess_rpm = row.front_collective_rpm;
seed.rear_collective_guess_rpm = row.rear_collective_rpm;
seed.theta_guess_deg = row.theta_deg;
seed.delta_f_guess_deg = row.delta_f_deg;
seed.delta_a_guess_deg = 0.0;
seed.delta_e_guess_deg = row.delta_e_deg;
seed.delta_r_guess_deg = 0.0;
seed.success = row.success;
seed.acceptable = row.acceptable;
seed.classification = char(row.classification);
seed.score = row.score;
end

function seed = localSeedFromCase(trimCase)
seed = struct();
seed.name = trimCase.name;
seed.source = 'trim_case';
seed.tilt_deg = localGetField(trimCase, 'front_tilt_deg', NaN);
seed.vinf_mps = localGetField(trimCase, 'Vinf_mps', NaN);
seed.front_collective_guess_rpm = localGetField(trimCase, 'front_collective_guess_rpm', NaN);
seed.rear_collective_guess_rpm = localGetField(trimCase, 'rear_collective_fixed_rpm', ...
    localGetField(trimCase, 'rear_collective_guess_rpm', NaN));
seed.theta_guess_deg = localGetField(trimCase, 'theta_guess_deg', 0.0);
seed.delta_f_guess_deg = localGetField(trimCase, 'delta_f_guess_deg', 0.0);
seed.delta_a_guess_deg = localGetField(trimCase, 'delta_a_guess_deg', 0.0);
seed.delta_e_guess_deg = localGetField(trimCase, 'delta_e_guess_deg', 0.0);
seed.delta_r_guess_deg = localGetField(trimCase, 'delta_r_guess_deg', 0.0);
end

function merit = localAnchorMerit(row)
if row.success
    classRank = 0.0;
elseif row.acceptable
    classRank = 1.0;
elseif row.classification == "near_trim_borderline"
    classRank = 2.0;
else
    classRank = 3.0;
end
merit = classRank * 1000.0 + ...
    row.goal_distance * 100.0 + ...
    min(row.score, 100.0) * 10.0 + ...
    row.prop_jump_from_hover * 20.0 + ...
    row.control_effort_deg * 0.5;
end

function localWriteAnchorSummaryMarkdown(summary, filename, result)
fid = fopen(filename, 'w');
if fid < 0
    warning('Run_Trim_Transition_RearOn_Connector_Forever:AnchorMarkdownWriteFailed', ...
        'Could not write anchor summary markdown: %s', filename);
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Rear-On Cruise Anchor Search\n\n');
fprintf(fid, '- Created: %s\n', result.created_on);
fprintf(fid, '- Output dir: `%s`\n', result.output_dir);
fprintf(fid, '- Candidate count: `%d`\n', height(summary));
if isempty(summary)
    return;
end
best = summary(1, :);
fprintf(fid, '- Best anchor: `%s`\n', best.name{1});
fprintf(fid, '- Best class: `%s`\n', best.classification{1});
fprintf(fid, '- Best front / rear: `%.1f / %.1f rpm`\n', best.front_collective_rpm, best.rear_collective_rpm);
fprintf(fid, '- Best merit: `%.4f`\n\n', best.anchor_merit);
fprintf(fid, '| Selected | Name | Family | Vinf | Tilt | Success | Acceptable | Score | Front | Rear | dF | dE | Merit |\n');
fprintf(fid, '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n');
topN = min(12, height(summary));
for i = 1:topN
    row = summary(i, :);
    fprintf(fid, '| %d | %s | %s | %.1f | %.1f | %d | %d | %.4f | %.1f | %.1f | %.2f | %.2f | %.4f |\n', ...
        row.selected, row.name{1}, row.family{1}, row.target_vinf_mps, row.target_tilt_deg, ...
        row.success, row.acceptable, row.score, row.front_collective_rpm, row.rear_collective_rpm, ...
        row.delta_f_deg, row.delta_e_deg, row.anchor_merit);
end
end

function spec = localAnchorCandidateTemplate()
spec = struct( ...
    'name', '', ...
    'family', '', ...
    'vinf_mps', NaN, ...
    'tilt_deg', NaN, ...
    'front_guess_rpm', NaN, ...
    'rear_guess_rpm', NaN, ...
    'rear_fixed_rpm', NaN, ...
    'theta_guess_deg', 0.0, ...
    'delta_f_guess_deg', 0.0, ...
    'delta_e_guess_deg', 0.0);
end

function row = localAnchorRowTemplate()
row = struct( ...
    'name', '', ...
    'family', '', ...
    'target_vinf_mps', NaN, ...
    'target_tilt_deg', NaN, ...
    'front_guess_rpm', NaN, ...
    'rear_guess_rpm', NaN, ...
    'rear_fixed_rpm', NaN, ...
    'success', false, ...
    'acceptable', false, ...
    'classification', "not_scored", ...
    'score', inf, ...
    'max_normalized', inf, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'theta_deg', NaN, ...
    'goal_distance', inf, ...
    'prop_jump_from_hover', inf, ...
    'control_effort_deg', inf, ...
    'anchor_merit', inf, ...
    'termination_string', "");
end

function result = localAnchorResultTemplate()
result = struct( ...
    'spec', struct(), ...
    'trimCase', struct(), ...
    'trimResult', struct(), ...
    'scoreData', struct());
end

function linearizationPoint = localBuildAnchorLinearizationPoint(anchorResult, summaryRow)
linearizationPoint = struct();
trimResult = localGetField(anchorResult, 'trimResult', struct());
linearData = localGetField(trimResult, 'linear', struct());
if isempty(fieldnames(linearData))
    return;
end
linearizationPoint.name = char(summaryRow.name);
linearizationPoint.family = char(summaryRow.family);
linearizationPoint.success = summaryRow.success;
linearizationPoint.acceptable = summaryRow.acceptable;
linearizationPoint.classification = char(summaryRow.classification);
linearizationPoint.score = summaryRow.score;
linearizationPoint.max_normalized = summaryRow.max_normalized;
linearizationPoint.target_vinf_mps = summaryRow.target_vinf_mps;
linearizationPoint.target_tilt_deg = summaryRow.target_tilt_deg;
linearizationPoint.front_collective_rpm = summaryRow.front_collective_rpm;
linearizationPoint.rear_collective_rpm = summaryRow.rear_collective_rpm;
linearizationPoint.delta_f_deg = summaryRow.delta_f_deg;
linearizationPoint.delta_e_deg = summaryRow.delta_e_deg;
linearizationPoint.theta_deg = summaryRow.theta_deg;
linearizationPoint.trimCase = localGetField(anchorResult, 'trimCase', struct());
linearizationPoint.scheduling = localGetField(trimResult, 'scheduling', struct());
linearizationPoint.Att_Trim_deg = localGetField(trimResult, 'Att_Trim_deg', []);
linearizationPoint.Vel_B_BA_Trim = localGetField(trimResult, 'Vel_B_BA_Trim', []);
linearizationPoint.scoreData = localGetField(anchorResult, 'scoreData', struct());
linearizationPoint.linear = localExtractAnchorLinearData(linearData);
end

function linear = localExtractAnchorLinearData(linearData)
linear = struct();
linear.reduced_model_available = localGetField(linearData, 'reduced_model_available', false);
linear.A_full = localGetField(linearData, 'A_full', []);
linear.B_full = localGetField(linearData, 'B_full', []);
linear.C_full = localGetField(linearData, 'C_full', []);
linear.D_full = localGetField(linearData, 'D_full', []);
linear.B_front_collective = localGetField(linearData, 'B_front_collective', []);
linear.B_rear_collective = localGetField(linearData, 'B_rear_collective', []);
sys_full = localGetField(linearData, 'sys_full', []);
if ~isempty(sys_full)
    linear.full_state_names = string(sys_full.StateName(:));
    linear.full_input_names = string(sys_full.InputName(:));
    linear.full_output_names = string(sys_full.OutputName(:));
else
    linear.full_state_names = strings(0, 1);
    linear.full_input_names = strings(0, 1);
    linear.full_output_names = strings(0, 1);
end
sys9 = localGetField(linearData, 'sys_ss_9state', []);
if ~isempty(sys9)
    linear.A_9 = sys9.A;
    linear.B_9 = sys9.B;
    linear.C_9 = sys9.C;
    linear.D_9 = sys9.D;
    linear.state_names_9 = string(sys9.StateName(:));
    linear.input_names_9 = string(sys9.InputName(:));
    linear.output_names_9 = string(sys9.OutputName(:));
    linear.eigenvalues_9 = eig(sys9.A);
else
    linear.A_9 = [];
    linear.B_9 = [];
    linear.C_9 = [];
    linear.D_9 = [];
    linear.state_names_9 = strings(0, 1);
    linear.input_names_9 = strings(0, 1);
    linear.output_names_9 = strings(0, 1);
    linear.eigenvalues_9 = [];
end
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end
