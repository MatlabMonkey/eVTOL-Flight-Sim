% Run_Trim_NearHover_Continuation.m
% Build a small continuation branch off the exact hover trim.
%
% Expected usage:
%   Init_EVTOL_Main
%   Run_Trim_NearHover_Continuation
%
% Optional configuration:
%   hoverContinuationOptions = struct( ...
%       'tilt_deg_list', 0:1:5, ...
%       'speed_mps_list', [0.0 0.5 1.0 2.0 3.0 4.0 5.0], ...
%       'max_seed_candidates', 3, ...
%       'emit_case_summary', true);
%
% This script keeps mixed controls pinned at zero and walks a small
% hover-side low-speed grid using only nearby trim results as seeds.

if ~exist('initData', 'var') || ~isstruct(initData)
    error(['initData is required. Run Init_EVTOL_Main first so the near-hover ', ...
           'continuation script has the aircraft constants and model names it needs.']);
end

if ~exist('hoverContinuationOptions', 'var') || isempty(hoverContinuationOptions)
    hoverContinuationOptions = struct();
end

config = localBuildConfig(hoverContinuationOptions);

nearHoverTrimResult = struct();
nearHoverTrimResult.meta = struct();
nearHoverTrimResult.meta.name = 'NearHoverContinuation';
nearHoverTrimResult.meta.created_on = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
nearHoverTrimResult.meta.config = config;
nearHoverTrimResult.meta.anchor_case = TrimCase_Hover();
nearHoverTrimResult.entries = repmat(localEntryTemplate(), 0, 1);
nearHoverTrimResult.summary = struct( ...
    'nTargets', 0, ...
    'nSuccess', 0, ...
    'nFailed', 0);

fprintf('Near-hover continuation\n');
fprintf('  tilt grid  = %s deg\n', mat2str(config.tilt_deg_list));
fprintf('  speed grid = %s m/s\n', mat2str(config.speed_mps_list));
fprintf('  max seed candidates = %d\n', config.max_seed_candidates);

fprintf('\n[anchor] exact hover\n');
[anchorResult, anchorSpec, anchorConsoleText] = localRunTrimQuiet(initData, nearHoverTrimResult.meta.anchor_case, config.trim_options); %#ok<NASGU>
anchorEntry = localMakeEntry( ...
    localMakeTarget(0.0, 0.0), ...
    nearHoverTrimResult.meta.anchor_case, ...
    anchorSpec, ...
    anchorResult, ...
    1, ...
    'anchor_hover', ...
    localMaxStateResidual(anchorResult.op_report));
nearHoverTrimResult.entries(end + 1, 1) = anchorEntry; %#ok<SAGROW>

fprintf('  success = %d | residual = %.6g | front = %.2f rpm | rear = %.2f rpm\n', ...
    anchorEntry.success, ...
    anchorEntry.max_state_residual, ...
    anchorEntry.front_collective_rpm, ...
    anchorEntry.rear_collective_rpm);

tilt_list = config.tilt_deg_list(:).';
tilt_list = tilt_list(abs(tilt_list) > 1e-9);

for iTilt = 1:numel(tilt_list)
    tilt_deg = tilt_list(iTilt);
    fprintf('\n=== Tilt %.1f deg ===\n', tilt_deg);

    for iSpeed = 1:numel(config.speed_mps_list)
        speed_mps = config.speed_mps_list(iSpeed);
        target = localMakeTarget(tilt_deg, speed_mps);

        [entry, attemptLog] = localSolveTarget(target, nearHoverTrimResult.entries, anchorEntry, initData, config);
        entry.attempts = attemptLog;
        nearHoverTrimResult.entries(end + 1, 1) = entry; %#ok<SAGROW>

        status_label = 'best';
        if entry.success
            status_label = 'exact';
        end

        fprintf('  V=%4.1f m/s | %s | residual=%.6g | seed=%s | front=%.2f | rear=%.2f | theta=%+.3f deg\n', ...
            speed_mps, ...
            status_label, ...
            entry.max_state_residual, ...
            entry.chosen_seed_name, ...
            entry.front_collective_rpm, ...
            entry.rear_collective_rpm, ...
            entry.theta_deg);
    end
end

nearHoverTrimResult.summary.nTargets = numel(nearHoverTrimResult.entries);
nearHoverTrimResult.summary.nSuccess = nnz([nearHoverTrimResult.entries.success]);
nearHoverTrimResult.summary.nFailed = nearHoverTrimResult.summary.nTargets - nearHoverTrimResult.summary.nSuccess;

fprintf('\nNear-hover continuation complete.\n');
fprintf('  success = %d / %d\n', ...
    nearHoverTrimResult.summary.nSuccess, ...
    nearHoverTrimResult.summary.nTargets);

function config = localBuildConfig(userOptions)
config = struct();
config.tilt_deg_list = localGetField(userOptions, 'tilt_deg_list', 0:1:5);
config.speed_mps_list = localGetField(userOptions, 'speed_mps_list', [0.0 0.5 1.0 2.0 3.0 4.0 5.0]);
config.max_seed_candidates = localGetField(userOptions, 'max_seed_candidates', 3);
config.emit_case_summary = localGetField(userOptions, 'emit_case_summary', true);

config.trim_options = struct( ...
    'verbose', false, ...
    'debug', false, ...
    'emitSummary', false, ...
    'emitLinearSummary', false);

config.seed_perturbations = [ ...
    localMakePerturbation('raw', 1.00, 1.00, 0.0); ...
    localMakePerturbation('theta_p0p5', 1.00, 1.00, 0.5); ...
    localMakePerturbation('theta_m0p5', 1.00, 1.00, -0.5); ...
    localMakePerturbation('front_p2pct', 1.02, 1.00, 0.0); ...
    localMakePerturbation('front_m2pct', 0.98, 1.00, 0.0); ...
    localMakePerturbation('rear_p2pct', 1.00, 1.02, 0.0); ...
    localMakePerturbation('rear_m2pct', 1.00, 0.98, 0.0)];
end

function target = localMakeTarget(tilt_deg, speed_mps)
target = struct();
target.tilt_deg = tilt_deg;
target.speed_mps = speed_mps;
target.name = sprintf('NearHover_tilt_%s__V_%s', ...
    localValueLabel(tilt_deg), localValueLabel(speed_mps));
end

function perturbation = localMakePerturbation(name, front_scale, rear_scale, theta_offset_deg)
perturbation = struct( ...
    'name', name, ...
    'front_scale', front_scale, ...
    'rear_scale', rear_scale, ...
    'theta_offset_deg', theta_offset_deg);
end

function [entry, attemptLog] = localSolveTarget(target, existingEntries, anchorEntry, initData, config)
seedCandidates = localBuildSeedCandidates(target, existingEntries, anchorEntry, config);
attemptLog = repmat(localAttemptTemplate(), 0, 1);

bestResult = struct();
bestSpec = struct();
bestCase = struct();
bestResidual = inf;
bestSeedName = '';
bestAttemptCount = 0;

attemptIndex = 0;
done = false;
for iSeed = 1:numel(seedCandidates)
    seed = seedCandidates(iSeed);
    perturbations = config.seed_perturbations;

    for iPerturb = 1:numel(perturbations)
        perturb = perturbations(iPerturb);
        case_i = localBuildNearHoverCase(target, seed, perturb);

        attemptIndex = attemptIndex + 1;
        attempt = localAttemptTemplate();
        attempt.index = attemptIndex;
        attempt.seed_name = seed.name;
        attempt.case_name = case_i.name;
        attempt.trim_case = case_i;

        try
            [result_i, spec_i, consoleText_i] = localRunTrimQuiet(initData, case_i, config.trim_options);
            residual_i = localMaxStateResidual(result_i.op_report);

            attempt.success = logical(result_i.success);
            attempt.termination_string = result_i.terminationString;
            attempt.max_state_residual = residual_i;
            attempt.summary = localCompactTrimSummary(result_i);
            attempt.console_text = consoleText_i;
            attemptLog(end + 1, 1) = attempt; %#ok<SAGROW>

            if result_i.success
                bestResult = result_i;
                bestSpec = spec_i;
                bestCase = case_i;
                bestResidual = residual_i;
                bestSeedName = seed.name;
                bestAttemptCount = attemptIndex;
                done = true;
                break;
            end

            if residual_i < bestResidual
                bestResult = result_i;
                bestSpec = spec_i;
                bestCase = case_i;
                bestResidual = residual_i;
                bestSeedName = seed.name;
                bestAttemptCount = attemptIndex;
            end
        catch ME
            attempt.success = false;
            attempt.termination_string = '';
            attempt.max_state_residual = inf;
            attempt.error_identifier = ME.identifier;
            attempt.error_message = ME.message;
            attemptLog(end + 1, 1) = attempt; %#ok<SAGROW>
        end
    end

    if done
        break;
    end
end

if isempty(fieldnames(bestResult))
    entry = localEntryTemplate();
    entry.name = target.name;
    entry.target_tilt_deg = target.tilt_deg;
    entry.target_vinf_mps = target.speed_mps;
    entry.chosen_seed_name = '';
    entry.max_state_residual = inf;
    entry.attempt_count = attemptIndex;
    return;
end

entry = localMakeEntry(target, bestCase, bestSpec, bestResult, bestAttemptCount, bestSeedName, bestResidual);
end

function seedCandidates = localBuildSeedCandidates(target, existingEntries, anchorEntry, config)
seedCandidates = repmat(localSeedTemplate(), 0, 1);

sameTiltExact = localFindNearestEntry(existingEntries, @(e) ...
    abs(e.target_tilt_deg - target.tilt_deg) < 1e-9 && ...
    e.target_vinf_mps < target.speed_mps && ...
    e.success, target);
sameTiltBest = localFindNearestEntry(existingEntries, @(e) ...
    abs(e.target_tilt_deg - target.tilt_deg) < 1e-9 && ...
    e.target_vinf_mps < target.speed_mps, target);
prevTiltExact = localFindNearestEntry(existingEntries, @(e) ...
    e.target_tilt_deg < target.tilt_deg && ...
    abs(e.target_vinf_mps - target.speed_mps) < 1e-9 && ...
    e.success, target);
prevTiltBest = localFindNearestEntry(existingEntries, @(e) ...
    e.target_tilt_deg < target.tilt_deg && ...
    abs(e.target_vinf_mps - target.speed_mps) < 1e-9, target);
nearestExact = localFindNearestEntry(existingEntries, @(e) e.success, target);

candidateEntries = {sameTiltExact, sameTiltBest, prevTiltExact, prevTiltBest, nearestExact, anchorEntry};
for i = 1:numel(candidateEntries)
    entry_i = candidateEntries{i};
    if isempty(entry_i)
        continue;
    end
    seedCandidates(end + 1, 1) = localSeedFromEntry(entry_i); %#ok<SAGROW>
end

seedCandidates = localUniqueSeeds(seedCandidates);
if numel(seedCandidates) > config.max_seed_candidates
    seedCandidates = seedCandidates(1:config.max_seed_candidates);
end
end

function entry = localFindNearestEntry(entries, predicateFcn, target)
entry = [];
if isempty(entries)
    return;
end

bestIdx = 0;
bestDistance = inf;
for i = 1:numel(entries)
    entry_i = entries(i);
    if ~predicateFcn(entry_i)
        continue;
    end

    distance_i = localEntryDistance(entry_i, target);
    if distance_i < bestDistance
        bestDistance = distance_i;
        bestIdx = i;
    end
end

if bestIdx > 0
    entry = entries(bestIdx);
end
end

function distance = localEntryDistance(entry, target)
distance = abs(entry.target_tilt_deg - target.tilt_deg) / 1.0 + ...
           abs(entry.target_vinf_mps - target.speed_mps) / 0.5;
if ~entry.success
    distance = distance + 0.25;
end
end

function case_i = localBuildNearHoverCase(target, seed, perturb)
case_i = TrimCase_Hover();
case_i.name = sprintf('%s__%s__%s', target.name, seed.name, perturb.name);
case_i.mode = 'transition';
case_i.Vinf_mps = target.speed_mps;
case_i.u_body_mps = target.speed_mps;
case_i.v_body_mps = 0.0;
case_i.w_body_mps = 0.0;
case_i.front_tilt_deg = target.tilt_deg;
case_i.front_tilt_cmd_deg = target.tilt_deg;
case_i.euler_known = [true; false; true];
case_i.euler_steady = [true; true; true];
case_i.body_velocity_known = [false; true; false];
case_i.body_velocity_steady = [true; true; true];
case_i.use_vinf_output_constraint = true;
case_i.use_vertical_speed_output_constraint = true;
case_i.mixed_control_known = [true; true; true; true];
case_i.delta_f_fixed_deg = 0.0;
case_i.delta_a_fixed_deg = 0.0;
case_i.delta_e_fixed_deg = 0.0;
case_i.delta_r_fixed_deg = 0.0;
case_i.validate_nonlinear_hold = false;

case_i.front_collective_guess_rpm = seed.front_collective_guess_rpm * perturb.front_scale;
case_i.rear_collective_guess_rpm = seed.rear_collective_guess_rpm * perturb.rear_scale;
case_i.rear_collective_trim_rpm = case_i.rear_collective_guess_rpm;
case_i.theta_guess_deg = seed.theta_guess_deg + perturb.theta_offset_deg;
end

function seed = localSeedFromEntry(entry)
seed = localSeedTemplate();
seed.name = ['entry_' entry.name];
seed.front_collective_guess_rpm = entry.front_collective_rpm;
seed.rear_collective_guess_rpm = entry.rear_collective_rpm;
seed.theta_guess_deg = entry.theta_deg;
seed.delta_f_guess_deg = entry.delta_f_deg;
seed.delta_e_guess_deg = entry.delta_e_deg;
end

function seedCandidates = localUniqueSeeds(seedCandidates)
if isempty(seedCandidates)
    return;
end

keys = strings(numel(seedCandidates), 1);
for i = 1:numel(seedCandidates)
    seed = seedCandidates(i);
    keys(i) = sprintf('%.6f|%.6f|%.6f|%.6f|%.6f', ...
        seed.front_collective_guess_rpm, ...
        seed.rear_collective_guess_rpm, ...
        seed.theta_guess_deg, ...
        seed.delta_f_guess_deg, ...
        seed.delta_e_guess_deg);
end

[~, keepIdx] = unique(keys, 'stable');
seedCandidates = seedCandidates(keepIdx);
end

function entry = localMakeEntry(target, case_i, spec_i, result_i, attempts, seed_name, maxResidual)
summary = localCompactTrimSummary(result_i);

entry = localEntryTemplate();
entry.name = target.name;
entry.target_tilt_deg = target.tilt_deg;
entry.target_vinf_mps = target.speed_mps;
entry.success = logical(result_i.success);
entry.termination_string = result_i.terminationString;
entry.attempt_count = attempts;
entry.chosen_seed_name = seed_name;
entry.max_state_residual = maxResidual;
entry.front_collective_rpm = summary.front_collective_rpm;
entry.rear_collective_rpm = summary.rear_collective_rpm;
entry.delta_f_deg = summary.delta_f_deg;
entry.delta_a_deg = summary.delta_a_deg;
entry.delta_e_deg = summary.delta_e_deg;
entry.delta_r_deg = summary.delta_r_deg;
entry.theta_deg = summary.theta_deg;
entry.u_mps = summary.u_mps;
entry.v_mps = summary.v_mps;
entry.w_mps = summary.w_mps;
entry.alpha_deg = summary.alpha_deg;
entry.beta_deg = summary.beta_deg;
entry.trim_case = case_i;
entry.trim_spec = spec_i;
entry.trim_summary = summary;
end

function summary = localCompactTrimSummary(result_i)
summary = struct();
summary.name = result_i.name;
summary.mode = result_i.mode;
summary.success = logical(result_i.success);
summary.termination_string = result_i.terminationString;
summary.front_tilt_deg = result_i.scheduling.front_tilt_deg;
summary.front_collective_rpm = result_i.scheduling.front_collective_rpm;
summary.rear_collective_rpm = result_i.scheduling.rear_collective_rpm;
summary.delta_f_deg = rad2deg(result_i.scheduling.delta_f_rad);
summary.delta_a_deg = rad2deg(result_i.scheduling.delta_a_rad);
summary.delta_e_deg = rad2deg(result_i.scheduling.delta_e_rad);
summary.delta_r_deg = rad2deg(result_i.scheduling.delta_r_rad);
summary.phi_deg = result_i.Att_Trim_deg(1);
summary.theta_deg = result_i.Att_Trim_deg(2);
summary.psi_deg = result_i.Att_Trim_deg(3);
summary.u_mps = result_i.Vel_B_BA_Trim(1);
summary.v_mps = result_i.Vel_B_BA_Trim(2);
summary.w_mps = result_i.Vel_B_BA_Trim(3);
summary.vinf_mps = result_i.scheduling.Vinf_mps;
summary.alpha_deg = rad2deg(result_i.scheduling.alpha_rad);
summary.beta_deg = rad2deg(result_i.scheduling.beta_rad);
end

function maxResidual = localMaxStateResidual(op_report)
maxResidual = inf;

try
    states = op_report.States;
catch
    return;
end

residuals = [];
for i = 1:numel(states)
    s = states(i);
    try
        dx = s.dx;
    catch
        dx = [];
    end
    try
        steadyMask = logical(s.SteadyState);
    catch
        steadyMask = true(size(dx));
    end

    if ~isempty(dx)
        dx = dx(:);
        if isempty(steadyMask)
            steadyMask = true(size(dx));
        else
            steadyMask = steadyMask(:);
            if numel(steadyMask) ~= numel(dx)
                steadyMask = true(size(dx));
            end
        end
        residuals = [residuals; abs(dx(steadyMask))]; %#ok<AGROW>
    end
end

if ~isempty(residuals)
    maxResidual = max(residuals);
end
end

function entry = localEntryTemplate()
entry = struct( ...
    'name', '', ...
    'target_tilt_deg', NaN, ...
    'target_vinf_mps', NaN, ...
    'success', false, ...
    'termination_string', '', ...
    'attempt_count', 0, ...
    'chosen_seed_name', '', ...
    'max_state_residual', inf, ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'u_mps', NaN, ...
    'v_mps', NaN, ...
    'w_mps', NaN, ...
    'alpha_deg', NaN, ...
    'beta_deg', NaN, ...
    'trim_case', struct(), ...
    'trim_spec', struct(), ...
    'trim_summary', struct(), ...
    'attempts', repmat(localAttemptTemplate(), 0, 1));
end

function attempt = localAttemptTemplate()
attempt = struct( ...
    'index', 0, ...
    'seed_name', '', ...
    'case_name', '', ...
    'success', false, ...
    'termination_string', '', ...
    'max_state_residual', inf, ...
    'error_identifier', '', ...
    'error_message', '', ...
    'trim_case', struct(), ...
    'summary', struct(), ...
    'console_text', '');
end

function seed = localSeedTemplate()
seed = struct( ...
    'name', '', ...
    'front_collective_guess_rpm', 0.0, ...
    'rear_collective_guess_rpm', 0.0, ...
    'theta_guess_deg', 0.0, ...
    'delta_f_guess_deg', 0.0, ...
    'delta_e_guess_deg', 0.0);
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function [trimResult, trimSpec, consoleText] = localRunTrimQuiet(initData, trimCase, trimOptions)
consoleText = evalc('[trimResult, trimSpec] = trim_evtol_case(initData, trimCase, trimOptions);');
end

function label = localValueLabel(value)
label = strrep(num2str(value, '%.4g'), '.', 'p');
label = strrep(label, '-', 'm');
end
