function plan = TrimSearch_BuildPlan(userOptions, root_dir)
%BUILD_MIDBAND_GUIDEGRID_PLAN Build transition-search targets and seeds.
% This helper creates target points plus guide-curve seed previews. Profiles
% choose the target strategy; the trim engine then searches each target.

if nargin < 1 || ~isstruct(userOptions)
    userOptions = struct();
end
if nargin < 2 || isempty(root_dir)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        root_dir = fileparts(stack(1).file);
    else
        root_dir = pwd;
    end
end

config = localBuildConfig(userOptions, root_dir);
anchorTable = localLoadAnchorTables(config.anchor_history_csvs, config.anchor_max_vinf_mps);
anchorTable = localFilterAnchorTableByGuide(anchorTable, config);
anchorSeeds = localBuildAnchorSeeds(anchorTable);
targets = localBuildTargets(config, anchorTable);
seedPreview = localBuildSeedPreview(targets, config);

plan = struct();
plan.root_dir = root_dir;
plan.config = config;
plan.anchor_table = anchorTable;
plan.anchor_seeds = anchorSeeds;
plan.targets = targets;
plan.seed_preview = seedPreview;
end

function config = localBuildConfig(userOptions, root_dir)
dbPaths = TrimDB_Paths(root_dir);
config = struct();
config.root_dir = root_dir;
databaseName = string(localGetField(userOptions, 'database_name', ""));
if strlength(databaseName) > 0
    config.database_dir = fullfile(dbPaths.database_dir, char(databaseName));
else
    config.database_dir = localGetField(userOptions, 'database_dir', dbPaths.database_dir);
end
config.workspace_plots_dir = localGetField(userOptions, 'workspace_plots_dir', dbPaths.workspace_plots_dir);
config.output_prefix = localGetField(userOptions, 'output_prefix', 'transition_trim_midband_guidegrid_scored');
config.trim_formulation = string(localGetField(userOptions, 'trim_formulation', "legacy_vinf_tilt"));
config.target_strategy = string(localGetField(userOptions, 'target_strategy', "guide_grid"));
config.explicit_targets = localGetField(userOptions, 'explicit_targets', []);
config.checkpoint_every = localGetField(userOptions, 'checkpoint_every', 5);
config.write_debug_run_outputs = localGetField(userOptions, 'write_debug_run_outputs', false);
config.write_linearizations_to_db = localGetField(userOptions, 'write_linearizations_to_db', false);
config.linearization_root = localGetField(userOptions, 'linearization_root', ...
    fullfile(config.database_dir, 'transition_trim_linearizations'));
config.linearization_index_csv = localGetField(userOptions, 'linearization_index_csv', ...
    fullfile(config.database_dir, 'transition_trim_linearization_index.csv'));
config.linearization_index_mat = localGetField(userOptions, 'linearization_index_mat', ...
    fullfile(config.database_dir, 'transition_trim_linearization_index.mat'));
config.target_limit = localGetField(userOptions, 'target_limit', inf);
config.vinf_grid_mps = localGetField(userOptions, 'vinf_grid_mps', 20:2.5:50);
config.tilt_offsets_deg = localGetField(userOptions, 'tilt_offsets_deg', -7.5:2.5:7.5);
config.alpha_grid_deg = localGetField(userOptions, 'alpha_grid_deg', 0.0);
config.tilt_grid_deg = localGetField(userOptions, 'tilt_grid_deg', 0:5:90);
config.target_tilt_min_deg = localGetField(userOptions, 'target_tilt_min_deg', 0.0);
config.target_tilt_max_deg = localGetField(userOptions, 'target_tilt_max_deg', 95.0);
config.max_vinf_mps = localGetField(userOptions, 'max_vinf_mps', max(config.vinf_grid_mps));
config.base_vinf_ceiling_mps = localGetField(userOptions, 'base_vinf_ceiling_mps', 5.0);
config.vinf_per_tilt = localGetField(userOptions, 'vinf_per_tilt', 0.8);
config.skip_known_good_targets = localGetField(userOptions, 'skip_known_good_targets', false);
config.cruise_anchor = localGetField(userOptions, 'cruise_anchor', struct('tilt_deg', 90.0, 'vinf_mps', 70.0));
config.reference_grid_step_deg = localGetField(userOptions, 'reference_grid_step_deg', 2.5);
config.front_collective_min_rpm = localGetField(userOptions, 'front_collective_min_rpm', 600.0);
config.rear_collective_min_rpm = localGetField(userOptions, 'rear_collective_min_rpm', 100.0);
config.front_seed_offsets_rpm = localGetField(userOptions, 'front_seed_offsets_rpm', [-100 0 100]);
config.rear_seed_offsets_rpm = localGetField(userOptions, 'rear_seed_offsets_rpm', [-100 0 100]);
config.neighbor_seed_count = localGetField(userOptions, 'neighbor_seed_count', 1);
config.history_seed_count = localGetField(userOptions, 'history_seed_count', 1);
config.anchor_seed_front_window_rpm = localGetField(userOptions, 'anchor_seed_front_window_rpm', inf);
config.anchor_seed_rear_window_rpm = localGetField(userOptions, 'anchor_seed_rear_window_rpm', inf);
config.failed_seed_filter_enabled = localGetField(userOptions, 'failed_seed_filter_enabled', true);
config.failed_seed_vinf_radius_mps = localGetField(userOptions, 'failed_seed_vinf_radius_mps', 2.5);
config.failed_seed_tilt_radius_deg = localGetField(userOptions, 'failed_seed_tilt_radius_deg', 5.0);
config.failed_seed_front_radius_rpm = localGetField(userOptions, 'failed_seed_front_radius_rpm', 125.0);
config.failed_seed_rear_radius_rpm = localGetField(userOptions, 'failed_seed_rear_radius_rpm', 125.0);
config.failed_seed_density_threshold = localGetField(userOptions, 'failed_seed_density_threshold', 3);
config.family_names = localGetField(userOptions, 'family_names', {'front_rear_free_flap_elevator'});
config.enable_guide_grid_seeds = localGetField(userOptions, 'enable_guide_grid_seeds', true);
config.enable_low_speed_physics_seeds = localGetField(userOptions, 'enable_low_speed_physics_seeds', false);
config.enable_transition_force_balance_seeds = localGetField(userOptions, 'enable_transition_force_balance_seeds', false);
config.transition_force_balance_seed_count = localGetField(userOptions, 'transition_force_balance_seed_count', 3);
config.transition_force_balance_gamma_grid_deg = localGetField(userOptions, 'transition_force_balance_gamma_grid_deg', [-5 0 5 10 15]);
config.transition_force_balance_gamma_bounds_deg = localGetField(userOptions, 'transition_force_balance_gamma_bounds_deg', [-10 25]);
config.stop_after_exact = localGetField(userOptions, 'stop_after_exact', false);
config.use_vertical_speed_output_constraint = localGetField(userOptions, 'use_vertical_speed_output_constraint', false);
config.use_alpha_output_constraint = localGetField(userOptions, 'use_alpha_output_constraint', true);
config.alpha_constraint_min_vinf_mps = localGetField(userOptions, 'alpha_constraint_min_vinf_mps', 0.5);
config.update_master_attempt_db_on_checkpoint = localGetField(userOptions, 'update_master_attempt_db_on_checkpoint', true);
config.refresh_canonical_databases_on_checkpoint = localGetField(userOptions, 'refresh_canonical_databases_on_checkpoint', false);
config.seed_distance_weights = struct('tilt', 1 / 5, 'vinf', 1 / 5);
config.anchor_history_csv = localGetField(userOptions, 'anchor_history_csv', "");
config.anchor_history_csvs = localGetField(userOptions, 'anchor_history_csvs', ...
    {fullfile(config.database_dir, 'trim_attempts.csv')});
config.anchor_max_vinf_mps = localGetField(userOptions, 'anchor_max_vinf_mps', 25.0);
config.anchor_max_tilt_error_deg = localGetField(userOptions, 'anchor_max_tilt_error_deg', inf);
config.reference_knot_vinf_mps = localGetField(userOptions, 'reference_knot_vinf_mps', ...
    [0 2.5 5 10 20 30 40 50 60 70]);
config.reference_knot_tilt_deg = localGetField(userOptions, 'reference_knot_tilt_deg', ...
    [0 18 30 44 62 72 80 85 88 90]);
config.front_guide_knot_vinf_mps = localGetField(userOptions, 'front_guide_knot_vinf_mps', ...
    [0 10 20 30 40 50 60 70 75]);
config.front_guide_knot_rpm = localGetField(userOptions, 'front_guide_knot_rpm', ...
    [1880 1890 1885 1860 1760 1600 1380 1050 840]);
config.rear_guide_knot_vinf_mps = localGetField(userOptions, 'rear_guide_knot_vinf_mps', ...
    [0 10 20 30 40 50 60 70 75]);
config.rear_guide_knot_rpm = localGetField(userOptions, 'rear_guide_knot_rpm', ...
    [1780 1670 1500 1350 1220 1080 800 430 100]);
computeLinearization = logical(localGetField(userOptions, 'compute_linearization', ...
    localGetField(userOptions, 'computeLinearization', ...
    localGetField(userOptions, 'write_linearizations_to_db', false))));
config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false, ...
    'computeLinearization', computeLinearization);
config.score_options = localGetField(userOptions, 'score_options', struct( ...
    'profile', 'transition', ...
    'hold_horizon_s', 2.0));
end

function anchorTable = localLoadAnchorTables(filenames, anchorMaxVinf)
if ischar(filenames) || isstring(filenames)
    filenames = cellstr(string(filenames));
end
if isempty(filenames)
    anchorTable = table();
    return;
end

anchorTables = cell(0, 1);
for iFile = 1:numel(filenames)
    filename = char(filenames{iFile});
    tbl = localLoadSingleAnchorTable(filename, anchorMaxVinf);
    if ~isempty(tbl)
        anchorTables{end + 1, 1} = tbl; %#ok<AGROW>
    end
end

if isempty(anchorTables)
    anchorTable = table();
    return;
end

anchorTables = localAlignTablesForVertcat(anchorTables);
anchorTable = vertcat(anchorTables{:});
if ismember('score', anchorTable.Properties.VariableNames)
    [~, order] = sort(anchorTable.score, 'ascend');
    anchorTable = anchorTable(order, :);
end
end

function anchorTable = localLoadSingleAnchorTable(filename, anchorMaxVinf)
if exist(filename, 'file') ~= 2
    anchorTable = table();
    return;
end

anchorTable = readtable(filename, 'TextType', 'string');
if isempty(anchorTable)
    return;
end

successMask = localAsLogicalColumn(anchorTable, 'success');
acceptableMask = localAsLogicalColumn(anchorTable, 'acceptable');
keepMask = successMask | acceptableMask;
anchorTable = anchorTable(keepMask, :);
if isempty(anchorTable)
    return;
end

if nargin >= 2 && isfinite(anchorMaxVinf) && ismember('vinf_mps', anchorTable.Properties.VariableNames)
    anchorTable = anchorTable(anchorTable.vinf_mps <= anchorMaxVinf + 1e-9, :);
end
end

function tablesOut = localAlignTablesForVertcat(tablesIn)
tablesOut = tablesIn;
if isempty(tablesOut)
    return;
end

allNames = string.empty(1, 0);
for i = 1:numel(tablesOut)
    allNames = [allNames string(tablesOut{i}.Properties.VariableNames)]; %#ok<AGROW>
end
allNames = unique(allNames, 'stable');

for i = 1:numel(tablesOut)
    tbl = tablesOut{i};
    currentNames = string(tbl.Properties.VariableNames);
    missingNames = setdiff(allNames, currentNames, 'stable');
    for j = 1:numel(missingNames)
        varName = char(missingNames(j));
        prototype = localFindPrototypeColumn(tablesOut, varName);
        tbl.(varName) = localMakeMissingColumnLike(prototype, height(tbl));
    end
    tbl = tbl(:, cellstr(allNames));
    tablesOut{i} = tbl;
end
end

function prototype = localFindPrototypeColumn(tablesIn, varName)
prototype = [];
for i = 1:numel(tablesIn)
    tbl = tablesIn{i};
    if ismember(varName, tbl.Properties.VariableNames)
        prototype = tbl.(varName);
        return;
    end
end
end

function col = localMakeMissingColumnLike(prototype, nRows)
if isempty(prototype)
    col = strings(nRows, 1);
    col(:) = missing;
    return;
end

if isstring(prototype)
    col = strings(nRows, 1);
    col(:) = missing;
elseif isnumeric(prototype)
    col = nan(nRows, 1);
elseif islogical(prototype)
    col = false(nRows, 1);
elseif isdatetime(prototype)
    col = NaT(nRows, 1, 'TimeZone', prototype.TimeZone);
elseif isduration(prototype)
    col = seconds(nan(nRows, 1));
else
    col = strings(nRows, 1);
    col(:) = missing;
end
end

function anchorTable = localFilterAnchorTableByGuide(anchorTable, config)
if isempty(anchorTable)
    return;
end
if ~isfinite(config.anchor_max_tilt_error_deg)
    return;
end
if ~ismember('vinf_mps', anchorTable.Properties.VariableNames) || ...
        ~ismember('tilt_deg', anchorTable.Properties.VariableNames)
    return;
end

guideTilt = interp1(config.reference_knot_vinf_mps, config.reference_knot_tilt_deg, ...
    anchorTable.vinf_mps, 'pchip', 'extrap');
keepMask = abs(anchorTable.tilt_deg - guideTilt) <= config.anchor_max_tilt_error_deg + 1e-9;
anchorTable = anchorTable(keepMask, :);
end

function anchorSeeds = localBuildAnchorSeeds(anchorTable)
anchorSeeds = repmat(localSeedTemplate(), 0, 1);
if isempty(anchorTable)
    return;
end

for i = 1:height(anchorTable)
    seed = localSeedTemplate();
    seed.name = char(localTableValue(anchorTable, 'name', i, sprintf('anchor_%d', i)));
    seed.source = 'low_speed_anchor';
    seed.tilt_deg = localTableValue(anchorTable, 'tilt_deg', i, NaN);
    seed.vinf_mps = localTableValue(anchorTable, 'vinf_mps', i, NaN);
    seed.front_collective_guess_rpm = localTableValue(anchorTable, 'front_collective_rpm', i, NaN);
    seed.rear_collective_guess_rpm = localTableValue(anchorTable, 'rear_collective_rpm', i, NaN);
    seed.theta_guess_deg = localTableValue(anchorTable, 'theta_deg', i, 0.0);
    seed.alpha_guess_deg = localAlphaGuessFromTableRow(anchorTable, i);
    seed.delta_f_guess_deg = localTableValue(anchorTable, 'delta_f_deg', i, 0.0);
    seed.delta_a_guess_deg = localTableValue(anchorTable, 'delta_a_deg', i, 0.0);
    seed.delta_e_guess_deg = localTableValue(anchorTable, 'delta_e_deg', i, 0.0);
    seed.delta_r_guess_deg = localTableValue(anchorTable, 'delta_r_deg', i, 0.0);
    seed.success = localTableValue(anchorTable, 'success', i, false);
    seed.acceptable = localTableValue(anchorTable, 'acceptable', i, false);
    seed.classification = char(localTableValue(anchorTable, 'classification', i, ""));
    seed.score = localTableValue(anchorTable, 'score', i, inf);
    anchorSeeds(end + 1, 1) = seed; %#ok<AGROW>
end
end

function targets = localBuildTargets(config, anchorTable)
switch lower(string(config.target_strategy))
    case "guide_grid"
        targets = localBuildGuideGridTargets(config);
    case "low_speed_frontier"
        targets = localBuildLowSpeedFrontierTargets(config);
    case "bridge_frontier"
        targets = localBuildBridgeFrontierTargets(config, anchorTable);
    case "explicit_targets"
        targets = localBuildExplicitTargets(config);
    otherwise
        error('TrimSearch_BuildPlan:UnknownTargetStrategy', ...
            'Unknown target strategy: %s', config.target_strategy);
end
end

function targets = localBuildGuideGridTargets(config)
targets = repmat(localTargetTemplate(), 0, 1);
for iV = 1:numel(config.vinf_grid_mps)
    vinf_mps = config.vinf_grid_mps(iV);
    centerTilt = localInterpGuide(vinf_mps, config.reference_knot_vinf_mps, config.reference_knot_tilt_deg);
    frontGuide = localInterpGuide(vinf_mps, config.front_guide_knot_vinf_mps, config.front_guide_knot_rpm);
    rearGuide = localInterpGuide(vinf_mps, config.rear_guide_knot_vinf_mps, config.rear_guide_knot_rpm);
    for iOffset = 1:numel(config.tilt_offsets_deg)
        tilt_deg = config.reference_grid_step_deg * round( ...
            (centerTilt + config.tilt_offsets_deg(iOffset)) / config.reference_grid_step_deg);
        if tilt_deg < config.target_tilt_min_deg - 1e-9 || ...
                tilt_deg > config.target_tilt_max_deg + 1e-9
            continue;
        end
        for iAlpha = 1:numel(config.alpha_grid_deg)
            alpha_deg = config.alpha_grid_deg(iAlpha);
            target = localTargetTemplate();
            target.name = sprintf('MidbandGuide_Tilt%s_V%s_A%s', ...
                localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(alpha_deg));
            target.tilt_deg = tilt_deg;
            target.vinf_mps = vinf_mps;
            target.alpha_target_deg = alpha_deg;
            target.center_tilt_deg = centerTilt;
            target.front_guide_rpm = frontGuide;
            target.rear_guide_rpm = rearGuide;
            target.sort_key = vinf_mps + 0.1 * abs(tilt_deg - centerTilt) + 0.01 * abs(alpha_deg);
            targets(end + 1, 1) = target; %#ok<AGROW>
        end
    end
end

if ~isinf(config.target_limit)
    targets = targets(1:min(numel(targets), config.target_limit));
end

[~, order] = sort([targets.sort_key], 'ascend');
targets = targets(order);
end

function targets = localBuildLowSpeedFrontierTargets(config)
targets = repmat(localTargetTemplate(), 0, 1);
vinf_grid = config.vinf_grid_mps(:).';

for iTilt = 1:numel(config.tilt_grid_deg)
    tilt_deg = config.tilt_grid_deg(iTilt);
    if tilt_deg < config.target_tilt_min_deg - 1e-9 || ...
            tilt_deg > config.target_tilt_max_deg + 1e-9
        continue;
    end
    vinf_ceiling = min(config.max_vinf_mps, config.base_vinf_ceiling_mps + config.vinf_per_tilt * tilt_deg);
    vinf_values = vinf_grid(vinf_grid <= vinf_ceiling + 1e-9);
    if tilt_deg == 0
        vinf_values = unique([0.0, vinf_values]);
    else
        vinf_values = vinf_values(vinf_values > 0.0);
    end

    for vinf_mps = vinf_values
        for iAlpha = 1:numel(config.alpha_grid_deg)
            alpha_deg = config.alpha_grid_deg(iAlpha);
            target = localTargetTemplate();
            target.name = sprintf('LowSpeedFrontier_Tilt%s_V%s_A%s', ...
                localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(alpha_deg));
            target.tilt_deg = tilt_deg;
            target.vinf_mps = vinf_mps;
            target.alpha_target_deg = alpha_deg;
            target.center_tilt_deg = tilt_deg;
            target.front_guide_rpm = localInterpGuide(vinf_mps, config.front_guide_knot_vinf_mps, config.front_guide_knot_rpm);
            target.rear_guide_rpm = localInterpGuide(vinf_mps, config.rear_guide_knot_vinf_mps, config.rear_guide_knot_rpm);
            target.sort_key = vinf_mps + 0.75 * tilt_deg + 0.01 * abs(alpha_deg);
            targets(end + 1, 1) = target; %#ok<AGROW>
        end
    end
end

targets = localLimitAndSortTargets(targets, config);
end

function targets = localBuildBridgeFrontierTargets(config, anchorTable)
targets = repmat(localTargetTemplate(), 0, 1);
knownGoodPoints = localKnownGoodPointsFromTable(anchorTable);
knownGoodKeys = localKnownGoodKeyMap(knownGoodPoints);
vinf_grid = config.vinf_grid_mps(:).';

for iTilt = 1:numel(config.tilt_grid_deg)
    tilt_deg = config.tilt_grid_deg(iTilt);
    if tilt_deg < config.target_tilt_min_deg - 1e-9 || ...
            tilt_deg > config.target_tilt_max_deg + 1e-9
        continue;
    end
    vinf_values = vinf_grid(vinf_grid <= config.max_vinf_mps + 1e-9);
    for vinf_mps = vinf_values
        if config.skip_known_good_targets && isKey(knownGoodKeys, localKnownPointKey(tilt_deg, vinf_mps))
            continue;
        end

        for iAlpha = 1:numel(config.alpha_grid_deg)
            alpha_deg = config.alpha_grid_deg(iAlpha);
            target = localTargetTemplate();
            target.name = sprintf('BridgeFrontier_Tilt%s_V%s_A%s', ...
                localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(alpha_deg));
            target.tilt_deg = tilt_deg;
            target.vinf_mps = vinf_mps;
            target.alpha_target_deg = alpha_deg;
            target.center_tilt_deg = localInterpGuide(vinf_mps, config.reference_knot_vinf_mps, config.reference_knot_tilt_deg);
            target.front_guide_rpm = localInterpGuide(vinf_mps, config.front_guide_knot_vinf_mps, config.front_guide_knot_rpm);
            target.rear_guide_rpm = localInterpGuide(vinf_mps, config.rear_guide_knot_vinf_mps, config.rear_guide_knot_rpm);
            frontierDistance = localNearestKnownPointDistance(tilt_deg, vinf_mps, knownGoodPoints, config.seed_distance_weights);
            cruiseDistance = hypot((config.cruise_anchor.tilt_deg - tilt_deg) * config.seed_distance_weights.tilt, ...
                (config.cruise_anchor.vinf_mps - vinf_mps) * config.seed_distance_weights.vinf);
            target.sort_key = 100.0 * frontierDistance + cruiseDistance + 0.01 * abs(alpha_deg);
            targets(end + 1, 1) = target; %#ok<AGROW>
        end
    end
end

targets = localLimitAndSortTargets(targets, config);
end

function targets = localBuildExplicitTargets(config)
rawTargets = config.explicit_targets;
if isempty(rawTargets)
    targets = repmat(localTargetTemplate(), 0, 1);
    return;
end

if ischar(rawTargets) || (isstring(rawTargets) && isscalar(rawTargets))
    targetFile = char(rawTargets);
    if exist(targetFile, 'file') ~= 2
        error('TrimSearch_BuildPlan:ExplicitTargetsMissing', ...
            'Explicit target file does not exist: %s', targetFile);
    end
    rawTargets = readtable(targetFile, 'TextType', 'string');
end

targets = repmat(localTargetTemplate(), 0, 1);
nRows = localExplicitTargetCount(rawTargets);
for iRow = 1:nRows
    tilt_deg = localExplicitNumeric(rawTargets, 'tilt_deg', iRow, NaN);
    vinf_mps = localExplicitNumeric(rawTargets, 'vinf_mps', iRow, NaN);
    alpha_deg = localExplicitNumeric(rawTargets, 'alpha_target_deg', iRow, ...
        localExplicitNumeric(rawTargets, 'alpha_deg', iRow, 0.0));
    if ~isfinite(tilt_deg) || ~isfinite(vinf_mps) || ~isfinite(alpha_deg)
        continue;
    end
    if tilt_deg < config.target_tilt_min_deg - 1e-9 || ...
            tilt_deg > config.target_tilt_max_deg + 1e-9 || ...
            vinf_mps > config.max_vinf_mps + 1e-9
        continue;
    end

    target = localTargetTemplate();
    target.name = char(localExplicitString(rawTargets, 'name', iRow, ...
        sprintf('ExplicitTarget_Tilt%s_V%s_A%s', ...
        localValueLabel(tilt_deg), localValueLabel(vinf_mps), localValueLabel(alpha_deg))));
    target.tilt_deg = tilt_deg;
    target.vinf_mps = vinf_mps;
    target.alpha_target_deg = alpha_deg;
    target.center_tilt_deg = localExplicitNumeric(rawTargets, 'center_tilt_deg', iRow, ...
        localInterpGuide(vinf_mps, config.reference_knot_vinf_mps, config.reference_knot_tilt_deg));
    target.front_guide_rpm = localExplicitNumeric(rawTargets, 'front_guide_rpm', iRow, ...
        localInterpGuide(vinf_mps, config.front_guide_knot_vinf_mps, config.front_guide_knot_rpm));
    target.rear_guide_rpm = localExplicitNumeric(rawTargets, 'rear_guide_rpm', iRow, ...
        localInterpGuide(vinf_mps, config.rear_guide_knot_vinf_mps, config.rear_guide_knot_rpm));
    target.sort_key = localExplicitNumeric(rawTargets, 'sort_key', iRow, iRow);
    targets(end + 1, 1) = target; %#ok<AGROW>
end

targets = localLimitAndSortTargets(targets, config);
end

function nRows = localExplicitTargetCount(rawTargets)
if istable(rawTargets)
    nRows = height(rawTargets);
elseif isstruct(rawTargets)
    nRows = numel(rawTargets);
else
    error('TrimSearch_BuildPlan:BadExplicitTargets', ...
        'explicit_targets must be a table, struct array, or CSV filename.');
end
end

function value = localExplicitNumeric(rawTargets, fieldName, idx, defaultValue)
if ~localExplicitHasField(rawTargets, fieldName)
    value = defaultValue;
    return;
end
rawValue = localExplicitValue(rawTargets, fieldName, idx);
if isnumeric(rawValue) || islogical(rawValue)
    value = double(rawValue);
elseif isstring(rawValue) || ischar(rawValue)
    value = str2double(string(rawValue));
else
    value = defaultValue;
end
if isempty(value) || ~isfinite(value)
    value = defaultValue;
end
end

function value = localExplicitString(rawTargets, fieldName, idx, defaultValue)
if ~localExplicitHasField(rawTargets, fieldName)
    value = string(defaultValue);
    return;
end
rawValue = localExplicitValue(rawTargets, fieldName, idx);
if isempty(rawValue) || (isstring(rawValue) && any(ismissing(rawValue)))
    value = string(defaultValue);
else
    value = string(rawValue);
end
end

function tf = localExplicitHasField(rawTargets, fieldName)
if istable(rawTargets)
    tf = ismember(fieldName, rawTargets.Properties.VariableNames);
elseif isstruct(rawTargets)
    tf = isfield(rawTargets, fieldName);
else
    tf = false;
end
end

function value = localExplicitValue(rawTargets, fieldName, idx)
if istable(rawTargets)
    value = rawTargets.(fieldName)(idx);
else
    value = rawTargets(idx).(fieldName);
end
end

function targets = localLimitAndSortTargets(targets, config)
if isempty(targets)
    return;
end
if ~isinf(config.target_limit)
    targets = targets(1:min(numel(targets), config.target_limit));
end
[~, order] = sort([targets.sort_key], 'ascend');
targets = targets(order);
end

function knownGoodPoints = localKnownGoodPointsFromTable(anchorTable)
knownGoodPoints = zeros(0, 2);
if isempty(anchorTable) || ~all(ismember({'tilt_deg','vinf_mps'}, anchorTable.Properties.VariableNames))
    return;
end
for i = 1:height(anchorTable)
    knownGoodPoints(end + 1, :) = [anchorTable.tilt_deg(i), anchorTable.vinf_mps(i)]; %#ok<AGROW>
end
knownGoodPoints = unique(round(knownGoodPoints, 6), 'rows');
end

function keys = localKnownGoodKeyMap(knownGoodPoints)
keys = containers.Map('KeyType', 'char', 'ValueType', 'logical');
for i = 1:size(knownGoodPoints, 1)
    keys(localKnownPointKey(knownGoodPoints(i, 1), knownGoodPoints(i, 2))) = true;
end
end

function key = localKnownPointKey(tilt_deg, vinf_mps)
key = sprintf('T%.3f_V%.3f', round(tilt_deg, 3), round(vinf_mps, 3));
end

function distance = localNearestKnownPointDistance(tilt_deg, vinf_mps, knownGoodPoints, weights)
if isempty(knownGoodPoints)
    distance = 1.0;
    return;
end
dt = (knownGoodPoints(:, 1) - tilt_deg) * weights.tilt;
dv = (knownGoodPoints(:, 2) - vinf_mps) * weights.vinf;
distance = min(hypot(dt, dv));
end

function previewRows = localBuildSeedPreview(targets, config)
previewRows = repmat(localPreviewRowTemplate(), 0, 1);
if ~localGetField(config, 'enable_guide_grid_seeds', true)
    return;
end
for i = 1:numel(targets)
    target = targets(i);
    for iFront = 1:numel(config.front_seed_offsets_rpm)
        for iRear = 1:numel(config.rear_seed_offsets_rpm)
            row = localPreviewRowTemplate();
            row.name = target.name;
            row.tilt_deg = target.tilt_deg;
            row.vinf_mps = target.vinf_mps;
            row.alpha_target_deg = target.alpha_target_deg;
            row.center_tilt_deg = target.center_tilt_deg;
            row.front_center_rpm = target.front_guide_rpm;
            row.rear_center_rpm = target.rear_guide_rpm;
            row.front_guess_rpm = max(config.front_collective_min_rpm, ...
                target.front_guide_rpm + config.front_seed_offsets_rpm(iFront));
            row.rear_guess_rpm = max(config.rear_collective_min_rpm, ...
                target.rear_guide_rpm + config.rear_seed_offsets_rpm(iRear));
            previewRows(end + 1, 1) = row; %#ok<AGROW>
        end
    end
end
end

function value = localInterpGuide(x, xk, yk)
value = interp1(xk, yk, x, 'pchip', 'extrap');
end

function out = localAsLogicalColumn(tbl, varName)
if ~ismember(varName, tbl.Properties.VariableNames)
    out = false(height(tbl), 1);
    return;
end
column = tbl.(varName);
if islogical(column)
    out = column;
elseif isnumeric(column)
    out = column ~= 0;
else
    lowered = lower(strtrim(string(column)));
    out = lowered == "1" | lowered == "true" | lowered == "yes";
end
out = logical(out);
end

function value = localTableValue(tbl, varName, idx, defaultValue)
if ismember(varName, tbl.Properties.VariableNames)
    value = tbl.(varName)(idx);
else
    value = defaultValue;
end
end

function alpha_deg = localAlphaGuessFromTableRow(tbl, idx)
alpha_deg = localTableValue(tbl, 'alpha_deg', idx, NaN);
if isfinite(alpha_deg)
    return;
end

u_mps = localTableValue(tbl, 'u_mps', idx, NaN);
w_mps = localTableValue(tbl, 'w_mps', idx, NaN);
if isfinite(u_mps) && isfinite(w_mps)
    alpha_deg = atan2d(w_mps, u_mps);
    return;
end

theta_deg = localTableValue(tbl, 'theta_deg', idx, NaN);
if isfinite(theta_deg)
    alpha_deg = theta_deg;
else
    alpha_deg = 0.0;
end
end

function target = localTargetTemplate()
target = struct( ...
    'name', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'alpha_target_deg', NaN, ...
    'center_tilt_deg', NaN, ...
    'front_guide_rpm', NaN, ...
    'rear_guide_rpm', NaN, ...
    'sort_key', inf);
end

function row = localPreviewRowTemplate()
row = struct( ...
    'name', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'alpha_target_deg', NaN, ...
    'center_tilt_deg', NaN, ...
    'front_center_rpm', NaN, ...
    'rear_center_rpm', NaN, ...
    'front_guess_rpm', NaN, ...
    'rear_guess_rpm', NaN);
end

function seed = localSeedTemplate()
seed = struct( ...
    'name', '', ...
    'source', '', ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'front_collective_guess_rpm', NaN, ...
    'rear_collective_guess_rpm', NaN, ...
    'alpha_guess_deg', NaN, ...
    'theta_guess_deg', NaN, ...
    'delta_f_guess_deg', 0.0, ...
    'delta_a_guess_deg', 0.0, ...
    'delta_e_guess_deg', 0.0, ...
    'delta_r_guess_deg', 0.0, ...
    'success', false, ...
    'acceptable', false, ...
    'classification', '', ...
    'score', inf);
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end
