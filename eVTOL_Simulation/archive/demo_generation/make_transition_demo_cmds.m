function demo = make_transition_demo_cmds(varargin)
%MAKE_TRANSITION_DEMO_CMDS Build believable open-loop transition commands.
%
% This helper creates a transition command package without requiring a
% working controller. It uses the scored trim corridor data already saved
% in workspace_plots and turns those trim anchors into smooth open-loop
% command histories for the nonlinear plant.
%
% Preferred usage:
%   demo = make_transition_demo_cmds()
%   demo = make_transition_demo_cmds('direction', 'cruise_to_trim')
%   demo = make_transition_demo_cmds('target_vinf_mps', 50, 'assign_to_base', true)
%
% Outputs:
%   demo.cmds           Struct of From Workspace command matrices.
%   demo.anchor_table   Table of anchors used to synthesize the transition.
%   demo.meta           Summary of assumptions, source files, and coverage.
%   demo.runSpecHint    Suggested runSpec fields for a matching open-loop run.
%
% The returned cmds struct includes the fields already used elsewhere in the
% workflow:
%   airData_cmd, eul_cmd, gps_Pos_cmd, omega_cmd, accel_cmd,
%   motor_cmd, tilt_cmd, front_cmd, rear_cmd
%
% and the additional surface traces that are useful for demo runs:
%   surface_mixed_cmd, surface_local_cmd,
%   deltaLW_cmd, deltaRW_cmd, deltaLT_cmd, deltaRT_cmd

parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'direction', 'trim_to_cruise');
addParameter(parser, 'target_vinf_mps', []);
addParameter(parser, 'stop_time_s', 30.0);
addParameter(parser, 'hold_start_s', 2.0);
addParameter(parser, 'hold_end_s', 3.0);
addParameter(parser, 'step_time_s', []);
addParameter(parser, 'assign_to_base', false);
addParameter(parser, 'root_dir', '');
parse(parser, varargin{:});

opts = parser.Results;
direction = localCanonicalizeDirection(opts.direction);
root_dir = localResolveRootDir(opts.root_dir);
localEnsureCasePaths(root_dir);
context = localResolveContext();

step_time_s = opts.step_time_s;
if isempty(step_time_s)
    step_time_s = context.step_time_s;
end

hold_start_s = max(opts.hold_start_s, 0.0);
hold_end_s = max(opts.hold_end_s, 0.0);
stop_time_s = max(opts.stop_time_s, hold_start_s + hold_end_s + step_time_s);
transition_time_s = max(stop_time_s - hold_start_s - hold_end_s, step_time_s);

anchor_table = localBuildAnchorTable(root_dir);

if isempty(opts.target_vinf_mps)
    if strcmp(direction, 'hover_to_cruise')
        target_vinf_mps = 75.0;
    else
        target_vinf_mps = 0.0;
    end
else
    target_vinf_mps = opts.target_vinf_mps;
end

path_table = localSelectPathAnchors(anchor_table, direction, target_vinf_mps);

if height(path_table) < 2
    error('make_transition_demo_cmds:InsufficientAnchors', ...
        'Need at least two transition anchors, but only found %d.', height(path_table));
end

t = (0:step_time_s:stop_time_s).';
progress = zeros(size(t));
active_mask = t >= hold_start_s & t <= (stop_time_s - hold_end_s);
if any(active_mask)
    tau = (t(active_mask) - hold_start_s) / transition_time_s;
    progress(active_mask) = localSmootherStep(tau);
    progress(t > (stop_time_s - hold_end_s)) = 1.0;
end

if strcmp(direction, 'cruise_to_hover')
    path_progress = 1.0 - progress;
else
    path_progress = progress;
end

s_anchor = path_table.path_progress;
vinf_cmd = localInterpAnchorSignal(s_anchor, path_table.vinf_mps, path_progress, 'vinf_mps');
theta_cmd_deg = localInterpAnchorSignal(s_anchor, path_table.theta_deg, path_progress, 'theta_deg');
tilt_cmd_deg = localInterpAnchorSignal(s_anchor, path_table.tilt_deg, path_progress, 'tilt_deg');
front_collective_cmd = localInterpAnchorSignal(s_anchor, path_table.front_collective_rpm, path_progress, 'front_collective_rpm');
rear_collective_cmd = localInterpAnchorSignal(s_anchor, path_table.rear_collective_rpm, path_progress, 'rear_collective_rpm');
delta_f_deg = localInterpAnchorSignal(s_anchor, path_table.delta_f_deg, path_progress, 'delta_f_deg');
delta_a_deg = localInterpAnchorSignal(s_anchor, path_table.delta_a_deg, path_progress, 'delta_a_deg');
delta_e_deg = localInterpAnchorSignal(s_anchor, path_table.delta_e_deg, path_progress, 'delta_e_deg');
delta_r_deg = localInterpAnchorSignal(s_anchor, path_table.delta_r_deg, path_progress, 'delta_r_deg');
alpha_cmd_deg = localInterpAnchorSignal(s_anchor, path_table.alpha_deg, path_progress, 'alpha_deg');
beta_cmd_deg = localInterpAnchorSignal(s_anchor, path_table.beta_deg, path_progress, 'beta_deg');

surface_local_rad = localMixedToLocalRad( ...
    deg2rad(delta_f_deg), deg2rad(delta_a_deg), ...
    deg2rad(delta_e_deg), deg2rad(delta_r_deg));

motor_cmd = [ ...
    front_collective_cmd, front_collective_cmd, ...
    rear_collective_cmd, rear_collective_cmd];

cmds = struct();
cmds.airData_cmd = [t, vinf_cmd, deg2rad(alpha_cmd_deg), deg2rad(beta_cmd_deg)];
cmds.eul_cmd = [t, zeros(numel(t), 1), deg2rad(theta_cmd_deg), zeros(numel(t), 1)];
cmds.gps_Pos_cmd = [t, ...
    context.pos_init(1) * ones(numel(t), 1), ...
    context.pos_init(2) * ones(numel(t), 1), ...
    context.pos_init(3) * ones(numel(t), 1)];
cmds.omega_cmd = [t, zeros(numel(t), 3)];
cmds.accel_cmd = [t, zeros(numel(t), 3)];
cmds.motor_cmd = [t, motor_cmd];
cmds.tilt_cmd = [t, tilt_cmd_deg, tilt_cmd_deg];
cmds.front_cmd = [t, front_collective_cmd];
cmds.rear_cmd = [t, rear_collective_cmd];
cmds.surface_mixed_cmd = [t, ...
    deg2rad(delta_f_deg), deg2rad(delta_a_deg), ...
    deg2rad(delta_e_deg), deg2rad(delta_r_deg)];
cmds.surface_local_cmd = [t, surface_local_rad];
cmds.deltaLW_cmd = [t, surface_local_rad(:, 1)];
cmds.deltaRW_cmd = [t, surface_local_rad(:, 2)];
cmds.deltaLT_cmd = [t, surface_local_rad(:, 3)];
cmds.deltaRT_cmd = [t, surface_local_rad(:, 4)];
cmds.demo_progress = [t, path_progress];
cmds.vinf_schedule = [t, vinf_cmd];

runSpecHint = struct();
runSpecHint.startTime = 0.0;
runSpecHint.stopTime = stop_time_s;
runSpecHint.stepTime = step_time_s;
runSpecHint.useController = false;
runSpecHint.attemptSimulation = false;
runSpecHint.pos_init = context.pos_init;
runSpecHint.omega_init = [0; 0; 0];
runSpecHint.Motor_RPMs = motor_cmd(1, :).';
runSpecHint.Tilt_angles = [tilt_cmd_deg(1); tilt_cmd_deg(1)];
runSpecHint.front_collective = front_collective_cmd(1);
runSpecHint.rear_collective = rear_collective_cmd(1);
runSpecHint.surface_init = surface_local_rad(1, :).';
runSpecHint.eul_init = [0; deg2rad(theta_cmd_deg(1)); 0];
runSpecHint.V_init = [vinf_cmd(1); deg2rad(alpha_cmd_deg(1)); deg2rad(beta_cmd_deg(1))];
runSpecHint.airData_cmd = [vinf_cmd(1); deg2rad(alpha_cmd_deg(1)); deg2rad(beta_cmd_deg(1))];

demo = struct();
demo.name = sprintf('TransitionDemo_%s', direction);
demo.direction = direction;
demo.cmds = cmds;
demo.anchor_table = path_table;
demo.meta = localBuildMeta(root_dir, direction, target_vinf_mps, ...
    step_time_s, hold_start_s, hold_end_s, stop_time_s, anchor_table, path_table);
demo.runSpecHint = runSpecHint;

if opts.assign_to_base
    assignin('base', 'transitionDemo', demo);
    assignin('base', 'cmds', cmds);
end
end

function direction = localCanonicalizeDirection(raw_direction)
value = lower(strtrim(string(raw_direction)));
switch value
    case {"hover_to_cruise", "trim_to_cruise", "forward", "up"}
        direction = 'hover_to_cruise';
    case {"cruise_to_hover", "cruise_to_trim", "reverse", "down"}
        direction = 'cruise_to_hover';
    otherwise
        error('make_transition_demo_cmds:BadDirection', ...
            'Unsupported direction "%s".', raw_direction);
end
end

function root_dir = localResolveRootDir(root_dir_in)
if ~isempty(root_dir_in)
    root_dir = char(root_dir_in);
    return;
end

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end
end

function localEnsureCasePaths(root_dir)
trim_case_dir = fullfile(root_dir, 'cases', 'trim');
if exist(trim_case_dir, 'dir') == 7
    addpath(trim_case_dir);
end
end

function context = localResolveContext()
context = struct();
context.pos_init = [0; 0; 0];
context.motor_rpms = zeros(4, 1);
context.step_time_s = 0.01;

if evalin('base', 'exist(''initData'', ''var'')') %#ok<EVLC>
    initData = evalin('base', 'initData'); %#ok<EVLC>
    if isstruct(initData)
        if isfield(initData, 'initialState') && isstruct(initData.initialState) && ...
                isfield(initData.initialState, 'pos_init')
            context.pos_init = initData.initialState.pos_init(:);
        end
        if isfield(initData, 'timing') && isstruct(initData.timing) && ...
                isfield(initData.timing, 'stepTime')
            context.step_time_s = initData.timing.stepTime;
        end
        if isfield(initData, 'defaultRunSpec') && isstruct(initData.defaultRunSpec) && ...
                isfield(initData.defaultRunSpec, 'Motor_RPMs')
            context.motor_rpms = initData.defaultRunSpec.Motor_RPMs(:);
        end
    end
end

if evalin('base', 'exist(''trimResult'', ''var'')') %#ok<EVLC>
    trimResult = evalin('base', 'trimResult'); %#ok<EVLC>
    if isstruct(trimResult)
        if isfield(trimResult, 'U_trim_full') && numel(trimResult.U_trim_full) >= 4
            context.motor_rpms = trimResult.U_trim_full(1:4);
        end
        if isfield(trimResult, 'Pos_Trim') && numel(trimResult.Pos_Trim) >= 3
            context.pos_init = trimResult.Pos_Trim(1:3);
        end
    end
end
end

function anchor_table = localBuildAnchorTable(root_dir)
anchor_table = table();

source_specs = { ...
    struct('tag', "reference_line", 'file', fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv')), ...
    struct('tag', "path_scored",    'file', fullfile(root_dir, 'workspace_plots', 'transition_trim_path_scored_latest.csv')), ...
    struct('tag', "bridge_scored",  'file', fullfile(root_dir, 'workspace_plots', 'transition_trim_map_bridge_scored_latest.csv')), ...
    struct('tag', "low_speed",      'file', fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'))};

for idx = 1:numel(source_specs)
    source_spec = source_specs{idx};
    file_path = source_spec.file;
    if exist(file_path, 'file') ~= 2
        continue;
    end

    tbl = readtable(file_path, 'TextType', 'string');
    normalized = localNormalizeAnchorTable(tbl, source_spec.tag, file_path);
    if isempty(normalized)
        continue;
    end
    anchor_table = [anchor_table; normalized]; %#ok<AGROW>
end

hover_anchor = localCaseAnchor(TrimCase_Hover(), 'hover_case', true);
cruise_anchor = localCaseAnchor(TrimCase_Cruise75_FlapElevator(), 'cruise_case', false);
anchor_table = [anchor_table; hover_anchor; cruise_anchor];
end

function normalized = localNormalizeAnchorTable(tbl, source_tag, file_path)
required = {'tilt_deg', 'vinf_mps', 'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_a_deg', 'delta_e_deg', 'delta_r_deg', 'theta_deg'};
for idx = 1:numel(required)
    if ~ismember(required{idx}, tbl.Properties.VariableNames)
        normalized = table();
        return;
    end
end

tilt_deg = localNumericColumn(tbl, 'tilt_deg');
vinf_mps = localNumericColumn(tbl, 'vinf_mps');
front_collective_rpm = localNumericColumn(tbl, 'front_collective_rpm');
rear_collective_rpm = localNumericColumn(tbl, 'rear_collective_rpm');
delta_f_deg = localNumericColumn(tbl, 'delta_f_deg');
delta_a_deg = localNumericColumn(tbl, 'delta_a_deg');
delta_e_deg = localNumericColumn(tbl, 'delta_e_deg');
delta_r_deg = localNumericColumn(tbl, 'delta_r_deg');
theta_deg = localNumericColumn(tbl, 'theta_deg');

if ismember('u_mps', tbl.Properties.VariableNames)
    u_mps = localNumericColumn(tbl, 'u_mps');
else
    u_mps = vinf_mps;
end

if ismember('w_mps', tbl.Properties.VariableNames)
    w_mps = localNumericColumn(tbl, 'w_mps');
else
    w_mps = zeros(size(vinf_mps));
end

if ismember('alpha_deg', tbl.Properties.VariableNames)
    alpha_deg = localNumericColumn(tbl, 'alpha_deg');
else
    alpha_deg = rad2deg(atan2(w_mps, max(u_mps, 1e-6)));
end

if ismember('classification', tbl.Properties.VariableNames)
    classification = string(tbl.classification);
else
    classification = repmat("unknown", height(tbl), 1);
end

acceptable = classification == "exact_trim" | classification == "quasi_trim_usable";
finite_mask = isfinite(tilt_deg) & isfinite(vinf_mps) & ...
    isfinite(front_collective_rpm) & isfinite(rear_collective_rpm) & ...
    isfinite(delta_f_deg) & isfinite(delta_a_deg) & ...
    isfinite(delta_e_deg) & isfinite(delta_r_deg) & isfinite(theta_deg);
keep = acceptable & finite_mask;

normalized = table();
if ~any(keep)
    return;
end

names = localStringColumn(tbl, 'name', "unnamed");

normalized = table();
normalized.name = names(keep);
normalized.source_tag = repmat(string(source_tag), nnz(keep), 1);
normalized.source_file = repmat(string(file_path), nnz(keep), 1);
normalized.classification = classification(keep);
normalized.is_data_backed = true(nnz(keep), 1);
normalized.tilt_deg = tilt_deg(keep);
normalized.vinf_mps = vinf_mps(keep);
normalized.front_collective_rpm = front_collective_rpm(keep);
normalized.rear_collective_rpm = rear_collective_rpm(keep);
normalized.delta_f_deg = delta_f_deg(keep);
normalized.delta_a_deg = delta_a_deg(keep);
normalized.delta_e_deg = delta_e_deg(keep);
normalized.delta_r_deg = delta_r_deg(keep);
normalized.theta_deg = theta_deg(keep);
normalized.alpha_deg = alpha_deg(keep);
normalized.beta_deg = zeros(nnz(keep), 1);
end

function anchor = localCaseAnchor(trimCase, source_tag, is_hover)
anchor = table();
anchor.name = string(trimCase.name);
anchor.source_tag = string(source_tag);
anchor.source_file = "";
anchor.classification = "case_endpoint";
anchor.is_data_backed = false;
anchor.tilt_deg = trimCase.front_tilt_deg;
anchor.vinf_mps = trimCase.Vinf_mps;
anchor.front_collective_rpm = trimCase.front_collective_guess_rpm;

if isfield(trimCase, 'rear_collective_fixed_rpm')
    anchor.rear_collective_rpm = trimCase.rear_collective_fixed_rpm;
elseif isfield(trimCase, 'rear_collective_trim_rpm')
    anchor.rear_collective_rpm = trimCase.rear_collective_trim_rpm;
else
    anchor.rear_collective_rpm = trimCase.rear_collective_guess_rpm;
end

if is_hover
    anchor.delta_f_deg = 0.0;
    anchor.delta_a_deg = 0.0;
    anchor.delta_e_deg = 0.0;
    anchor.delta_r_deg = 0.0;
    anchor.theta_deg = 0.0;
    anchor.alpha_deg = 0.0;
else
    anchor.delta_f_deg = localGetField(trimCase, 'delta_f_guess_deg', 0.0);
    anchor.delta_a_deg = localGetField(trimCase, 'delta_a_fixed_deg', 0.0);
    anchor.delta_e_deg = localGetField(trimCase, 'delta_e_guess_deg', 0.0);
    anchor.delta_r_deg = localGetField(trimCase, 'delta_r_fixed_deg', 0.0);
    anchor.theta_deg = localGetField(trimCase, 'theta_guess_deg', 0.0);
    anchor.alpha_deg = 0.0;
end

anchor.beta_deg = 0.0;
end

function path_table = localSelectPathAnchors(anchor_table, direction, target_vinf_mps)
if isempty(anchor_table)
    path_table = anchor_table;
    return;
end

target_vinf_mps = max(min(target_vinf_mps, 75.0), 0.0);
path_table = localSelectPreferredCorridor(anchor_table);

path_table = localForceEndpoint(path_table, anchor_table, 0.0, "hover_case");
path_table = localForceEndpoint(path_table, anchor_table, 75.0, "cruise_case");
path_table = localKeepMonotoneTilt(path_table);

if strcmp(direction, 'hover_to_cruise')
    path_table = path_table(path_table.vinf_mps <= target_vinf_mps + 1e-9, :);
else
    path_table = path_table(path_table.vinf_mps >= target_vinf_mps - 1e-9, :);
end

path_table = localAddSyntheticCruiseBlend(path_table, target_vinf_mps, direction);
path_table = sortrows(path_table, 'vinf_mps', 'ascend');
path_table = localAttachPathProgress(path_table);
end

function path_table = localSelectPreferredCorridor(anchor_table)
reference_table = anchor_table(anchor_table.source_tag == "reference_line", :);
if height(reference_table) >= 3
    path_table = localSelectBestPerSpeed(reference_table);
    max_ref_v = max(path_table.vinf_mps);
    supplemental = anchor_table(anchor_table.is_data_backed & ...
        anchor_table.source_tag ~= "reference_line" & ...
        anchor_table.vinf_mps > max_ref_v + 1e-9, :);
    if ~isempty(supplemental)
        supplemental = localSelectBestPerSpeed(supplemental);
        path_table = [path_table; supplemental]; %#ok<AGROW>
        path_table = sortrows(path_table, 'vinf_mps', 'ascend');
    end
    return;
end

preferred_sources = ["path_scored", "bridge_scored", "low_speed", "hover_case", "cruise_case"];
subset = anchor_table(ismember(anchor_table.source_tag, preferred_sources), :);
path_table = localSelectBestPerSpeed(subset);
end

function path_table = localSelectBestPerSpeed(candidate_table)
speed_values = unique(round(candidate_table.vinf_mps * 2) / 2);
selected_rows = false(height(candidate_table), 1);

for idx = 1:numel(speed_values)
    speed_i = speed_values(idx);
    same_speed = abs(candidate_table.vinf_mps - speed_i) < 1e-9;
    subset = candidate_table(same_speed, :);
    [~, local_idx] = min(localSelectionScore(subset));
    subset_idx = find(same_speed);
    selected_rows(subset_idx(local_idx)) = true;
end

path_table = candidate_table(selected_rows, :);
path_table = sortrows(path_table, 'vinf_mps', 'ascend');
end

function scores = localSelectionScore(tbl)
ref_tilt = localReferenceTilt(tbl.vinf_mps);
tilt_penalty = abs(tbl.tilt_deg - ref_tilt) / 10.0;

source_penalty = zeros(height(tbl), 1);
for idx = 1:height(tbl)
    switch tbl.source_tag(idx)
        case "hover_case"
            source_penalty(idx) = -0.10;
        case "reference_line"
            source_penalty(idx) = 0.00;
        case "path_scored"
            source_penalty(idx) = 0.05;
        case "bridge_scored"
            source_penalty(idx) = 0.10;
        case "low_speed"
            source_penalty(idx) = 0.20;
        case "cruise_case"
            source_penalty(idx) = 0.25;
        otherwise
            source_penalty(idx) = 0.50;
    end
end

class_penalty = zeros(height(tbl), 1);
class_penalty(tbl.classification == "quasi_trim_usable") = 0.05;
class_penalty(tbl.classification == "case_endpoint") = 0.10;

scores = tilt_penalty + source_penalty + class_penalty;
end

function ref_tilt = localReferenceTilt(vinf_mps)
vinf_knot = [0 2.5 5 10 20 30 40 50 60 70];
tilt_knot = [0 18 30 44 62 72 80 85 88 90];
ref_tilt = interp1(vinf_knot, tilt_knot, vinf_mps, 'pchip', 'extrap');
ref_tilt = min(max(ref_tilt, 0.0), 90.0);
end

function path_table = localForceEndpoint(path_table, anchor_table, target_speed, source_tag)
path_table = path_table(abs(path_table.vinf_mps - target_speed) >= 1e-9, :);
mask = abs(anchor_table.vinf_mps - target_speed) < 1e-9 & anchor_table.source_tag == source_tag;
if any(mask)
    path_table = [path_table; anchor_table(find(mask, 1, 'first'), :)]; %#ok<AGROW>
end
end

function path_table = localKeepMonotoneTilt(path_table)
if isempty(path_table)
    return;
end

path_table = sortrows(path_table, 'vinf_mps', 'ascend');
keep = true(height(path_table), 1);
last_tilt = path_table.tilt_deg(1);
for idx = 2:height(path_table)
    if path_table.tilt_deg(idx) + 1.0 < last_tilt
        keep(idx) = false;
        continue;
    end
    last_tilt = max(last_tilt, path_table.tilt_deg(idx));
end

path_table = path_table(keep, :);
end

function path_table = localAddSyntheticCruiseBlend(path_table, target_vinf_mps, direction)
if strcmp(direction, 'cruise_to_hover')
    terminal_speed = max(path_table.vinf_mps);
else
    terminal_speed = target_vinf_mps;
end

if terminal_speed <= 0
    return;
end

cruise_row = path_table(abs(path_table.vinf_mps - 75.0) < 1e-9, :);
if isempty(cruise_row)
    return;
end

if strcmp(direction, 'hover_to_cruise')
    partial = path_table(path_table.vinf_mps <= terminal_speed + 1e-9, :);
else
    partial = path_table(path_table.vinf_mps >= terminal_speed - 1e-9, :);
end

data_backed = partial(partial.is_data_backed, :);
if isempty(data_backed)
    return;
end

last_backed = data_backed(end, :);
if strcmp(direction, 'cruise_to_hover')
    last_backed = data_backed(1, :);
end

if abs(cruise_row.vinf_mps - last_backed.vinf_mps) <= 12.5
    return;
end

blend_speeds = linspace(last_backed.vinf_mps, cruise_row.vinf_mps, 4);
blend_speeds = blend_speeds(2:3);

synthetic = table();
synthetic.name = strings(0, 1);
synthetic.source_tag = strings(0, 1);
synthetic.source_file = strings(0, 1);
synthetic.classification = strings(0, 1);
synthetic.is_data_backed = false(0, 1);
synthetic.tilt_deg = zeros(0, 1);
synthetic.vinf_mps = zeros(0, 1);
synthetic.front_collective_rpm = zeros(0, 1);
synthetic.rear_collective_rpm = zeros(0, 1);
synthetic.delta_f_deg = zeros(0, 1);
synthetic.delta_a_deg = zeros(0, 1);
synthetic.delta_e_deg = zeros(0, 1);
synthetic.delta_r_deg = zeros(0, 1);
synthetic.theta_deg = zeros(0, 1);
synthetic.alpha_deg = zeros(0, 1);
synthetic.beta_deg = zeros(0, 1);

for idx = 1:numel(blend_speeds)
    frac = (blend_speeds(idx) - last_backed.vinf_mps) / ...
        max(cruise_row.vinf_mps - last_backed.vinf_mps, eps);
    synthetic(end + 1, :) = localBlendRows(last_backed, cruise_row, frac, blend_speeds(idx)); %#ok<AGROW>
end

path_table = [path_table; synthetic];
end

function blended = localBlendRows(row_a, row_b, frac, target_speed)
frac = min(max(frac, 0.0), 1.0);
blended = row_a;
blended.name = "synthetic_cruise_blend";
blended.source_tag = "synthetic_blend";
blended.source_file = "";
blended.classification = "synthetic";
blended.is_data_backed = false;
blended.vinf_mps = target_speed;

numeric_fields = {'tilt_deg', 'front_collective_rpm', 'rear_collective_rpm', ...
    'delta_f_deg', 'delta_a_deg', 'delta_e_deg', 'delta_r_deg', ...
    'theta_deg', 'alpha_deg', 'beta_deg'};
for idx = 1:numel(numeric_fields)
    field_name = numeric_fields{idx};
    blended.(field_name) = row_a.(field_name) + frac * (row_b.(field_name) - row_a.(field_name));
end
end

function path_table = localAttachPathProgress(path_table)
if height(path_table) == 1
    path_table.path_progress = 0.0;
    return;
end

progress = zeros(height(path_table), 1);
for idx = 2:height(path_table)
    ds = (path_table.vinf_mps(idx) - path_table.vinf_mps(idx - 1)) / 75.0;
    dt = (path_table.tilt_deg(idx) - path_table.tilt_deg(idx - 1)) / 90.0;
    progress(idx) = progress(idx - 1) + hypot(ds, dt);
end

if progress(end) <= eps
    progress = linspace(0, 1, height(path_table)).';
else
    progress = progress / progress(end);
end

path_table.path_progress = progress;
end

function signal = localInterpAnchorSignal(anchor_progress, anchor_values, sample_progress, label)
anchor_progress = anchor_progress(:);
anchor_values = anchor_values(:);
sample_progress = sample_progress(:);

if numel(anchor_progress) ~= numel(anchor_values)
    error('make_transition_demo_cmds:AnchorSizeMismatch', ...
        'Anchor mismatch for %s.', label);
end

if numel(anchor_progress) == 1
    signal = anchor_values(1) * ones(size(sample_progress));
    return;
end

method = 'linear';
if numel(anchor_progress) >= 3
    method = 'pchip';
end
signal = interp1(anchor_progress, anchor_values, sample_progress, method);
end

function surface_local_rad = localMixedToLocalRad(delta_f_rad, delta_a_rad, delta_e_rad, delta_r_rad)
surface_local_rad = [ ...
    delta_f_rad + delta_a_rad, ...
    delta_f_rad - delta_a_rad, ...
    delta_e_rad - delta_r_rad, ...
    delta_e_rad + delta_r_rad];
end

function meta = localBuildMeta(root_dir, direction, target_vinf_mps, ...
    step_time_s, hold_start_s, hold_end_s, stop_time_s, anchor_table, path_table)
data_backed = path_table(path_table.is_data_backed, :);
if isempty(data_backed)
    data_backed_vinf_max = NaN;
else
    data_backed_vinf_max = max(data_backed.vinf_mps);
end

meta = struct();
meta.root_dir = root_dir;
meta.direction = direction;
meta.target_vinf_mps = target_vinf_mps;
meta.step_time_s = step_time_s;
meta.hold_start_s = hold_start_s;
meta.hold_end_s = hold_end_s;
meta.stop_time_s = stop_time_s;
meta.anchor_source_count = height(anchor_table);
meta.selected_anchor_count = height(path_table);
meta.data_backed_vinf_max = data_backed_vinf_max;
meta.uses_extrapolated_terminal_segment = target_vinf_mps > data_backed_vinf_max + 1e-9;
meta.selected_sources = unique(path_table.source_tag);
meta.selected_anchor_names = path_table.name;
end

function values = localNumericColumn(tbl, field_name)
raw = tbl.(field_name);
if isnumeric(raw)
    values = double(raw);
else
    values = str2double(string(raw));
end
values = values(:);
end

function values = localStringColumn(tbl, field_name, default_value)
if ismember(field_name, tbl.Properties.VariableNames)
    values = string(tbl.(field_name));
else
    values = repmat(string(default_value), height(tbl), 1);
end
values = values(:);
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function y = localSmootherStep(x)
x = min(max(x, 0.0), 1.0);
y = x.^3 .* (x .* (x * 6.0 - 15.0) + 10.0);
end
