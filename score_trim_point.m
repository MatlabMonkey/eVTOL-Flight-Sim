function scoreData = score_trim_point(trimResult, opts)
% trim score

if nargin < 2 || isempty(opts)
    opts = struct();
end

if ~isstruct(trimResult) || ~isfield(trimResult, 'op_report')
    error(['trimResult must be the struct returned by trim_evtol_case so ', ...
           'the scoring helper can inspect op_report residuals.']);
end

opts = localApplyDefaults(opts);
rawResiduals = localExtractResiduals(trimResult.op_report);
componentDefs = localBuildComponentDefs(opts);

components = repmat(localComponentTemplate(), 0, 1);
weighted_sq_sum = 0.0;
weight_sum = 0.0;
max_normalized = 0.0;

for i = 1:numel(componentDefs)
    def = componentDefs(i);
    residualInfo = rawResiduals.(def.field_name);
    if ~residualInfo.present
        continue;
    end

    comp = localComponentTemplate();
    comp.name = def.field_name;
    comp.label = def.label;
    comp.raw_residual = residualInfo.value;
    comp.raw_units = residualInfo.units;
    comp.weight = def.weight;
    comp.acceptable_limit = def.acceptable_limit;
    comp.acceptable_units = def.acceptable_units;
    comp.drift_estimate = localEstimateDrift(residualInfo.value, def.kind, opts.hold_horizon_s);
    comp.drift_units = def.acceptable_units;
    comp.normalized = abs(comp.drift_estimate) / max(def.acceptable_limit, eps);
    comp.weighted_contribution = def.weight * comp.normalized^2;

    weighted_sq_sum = weighted_sq_sum + comp.weighted_contribution;
    weight_sum = weight_sum + def.weight;
    max_normalized = max(max_normalized, comp.normalized);
    components(end + 1, 1) = comp; %#ok<SAGROW>
end

if isempty(components)
    error(['score_trim_point could not find any steady-state body-velocity or ', ...
           'body-rate residuals in trimResult.op_report.']);
end

[~, order] = sort([components.weighted_contribution], 'descend');
components = components(order);

score = sqrt(weighted_sq_sum / max(weight_sum, eps));
acceptable = max_normalized <= 1.0;

classification = 'not_usable';
if isfield(trimResult, 'success') && logical(trimResult.success)
    classification = 'exact_trim';
elseif max_normalized <= 1.0
    classification = 'quasi_trim_usable';
elseif max_normalized <= 2.0
    classification = 'near_trim_borderline';
end

scoreData = struct();
scoreData.name = localGetField(trimResult, 'name', 'TrimPoint');
scoreData.mode = localGetField(trimResult, 'mode', 'trim');
scoreData.profile = opts.profile;
scoreData.hold_horizon_s = opts.hold_horizon_s;
scoreData.score = score;
scoreData.max_normalized = max_normalized;
scoreData.acceptable = acceptable;
scoreData.classification = classification;
scoreData.success = localGetField(trimResult, 'success', false);
scoreData.termination_string = localGetField(trimResult, 'terminationString', '');
scoreData.components = components;
scoreData.raw_residuals = rawResiduals;

if ~isempty(components)
    scoreData.worst_component = components(1);
else
    scoreData.worst_component = localComponentTemplate();
end
end

function opts = localApplyDefaults(opts)
if ~isfield(opts, 'profile') || isempty(opts.profile)
    opts.profile = 'transition';
end
if ~isfield(opts, 'hold_horizon_s') || isempty(opts.hold_horizon_s)
    opts.hold_horizon_s = 2.0;
end
if ~isfield(opts, 'include_euler_rate_terms') || isempty(opts.include_euler_rate_terms)
    opts.include_euler_rate_terms = false;
end
end

function rawResiduals = localExtractResiduals(op_report)
rawResiduals = localResidualTemplate();

for i = 1:numel(op_report.States)
    state_i = op_report.States(i);
    block_name = strtrim(char(state_i.Block));
    dx = state_i.dx(:);
    steady = logical(state_i.SteadyState(:));
    known = logical(state_i.Known(:));

    if numel(steady) ~= numel(dx)
        steady = true(size(dx));
    end
    if numel(known) ~= numel(dx)
        known = false(size(dx));
    end

    if contains(block_name, 'body_velocity_state')
        labels = {'u_dot', 'v_dot', 'w_dot'};
        units = 'm/s^2';
    elseif contains(block_name, 'body_rates_state')
        labels = {'p_dot', 'q_dot', 'r_dot'};
        units = 'rad/s^2';
    elseif contains(block_name, 'euler_attitude_state')
        labels = {'phi_dot', 'theta_dot', 'psi_dot'};
        units = 'rad/s';
    else
        continue;
    end

    for j = 1:min(numel(dx), numel(labels))
        field_name = labels{j};
        rawResiduals.(field_name).present = logical(steady(j));
        rawResiduals.(field_name).value = dx(j);
        rawResiduals.(field_name).units = units;
        rawResiduals.(field_name).known = known(j);
        rawResiduals.(field_name).steady = steady(j);
        rawResiduals.(field_name).block = block_name;
    end
end
end

function defs = localBuildComponentDefs(opts)
switch lower(opts.profile)
    case 'transition'
        defs = [ ...
            localComponentDef('u_dot', 'Forward speed drift', 'linear_accel', 1.0, 'm/s', 0.50); ...
            localComponentDef('v_dot', 'Lateral speed drift', 'linear_accel', 0.5, 'm/s', 1.25); ...
            localComponentDef('w_dot', 'Vertical speed drift', 'linear_accel', 0.5, 'm/s', 1.50); ...
            localComponentDef('p_dot', 'Roll angle drift', 'angular_accel', 3.0, 'deg', 1.00); ...
            localComponentDef('q_dot', 'Pitch angle drift', 'angular_accel', 2.0, 'deg', 2.00); ...
            localComponentDef('r_dot', 'Yaw angle drift', 'angular_accel', 3.0, 'deg', 1.00)];
    case 'hold'
        defs = [ ...
            localComponentDef('u_dot', 'Forward speed drift', 'linear_accel', 0.5, 'm/s', 1.00); ...
            localComponentDef('v_dot', 'Lateral speed drift', 'linear_accel', 0.25, 'm/s', 1.50); ...
            localComponentDef('w_dot', 'Vertical speed drift', 'linear_accel', 0.25, 'm/s', 1.75); ...
            localComponentDef('p_dot', 'Roll angle drift', 'angular_accel', 2.0, 'deg', 1.25); ...
            localComponentDef('q_dot', 'Pitch angle drift', 'angular_accel', 1.5, 'deg', 2.50); ...
            localComponentDef('r_dot', 'Yaw angle drift', 'angular_accel', 2.0, 'deg', 1.25)];
    otherwise
        error('Unknown score_trim_point profile "%s".', opts.profile);
end

if opts.include_euler_rate_terms
    defs(end + 1) = localComponentDef('phi_dot', 'Roll angle rate drift', 'angle_rate', 1.0, 'deg', 0.25); %#ok<AGROW>
    defs(end + 1) = localComponentDef('theta_dot', 'Pitch angle rate drift', 'angle_rate', 1.0, 'deg', 0.35); %#ok<AGROW>
    defs(end + 1) = localComponentDef('psi_dot', 'Yaw angle rate drift', 'angle_rate', 1.5, 'deg', 0.25); %#ok<AGROW>
end
end

function def = localComponentDef(field_name, label, kind, acceptable_limit, acceptable_units, weight)
def = struct( ...
    'field_name', field_name, ...
    'label', label, ...
    'kind', kind, ...
    'acceptable_limit', acceptable_limit, ...
    'acceptable_units', acceptable_units, ...
    'weight', weight);
end

function drift = localEstimateDrift(raw_value, kind, hold_horizon_s)
switch kind
    case 'linear_accel'
        drift = raw_value * hold_horizon_s;
    case 'angular_accel'
        drift = rad2deg(0.5 * raw_value * hold_horizon_s^2);
    case 'angle_rate'
        drift = rad2deg(raw_value * hold_horizon_s);
    otherwise
        error('Unknown drift kind "%s".', kind);
end
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = default_value;
end
end

function comp = localComponentTemplate()
comp = struct( ...
    'name', '', ...
    'label', '', ...
    'raw_residual', NaN, ...
    'raw_units', '', ...
    'drift_estimate', NaN, ...
    'drift_units', '', ...
    'acceptable_limit', NaN, ...
    'acceptable_units', '', ...
    'weight', NaN, ...
    'normalized', NaN, ...
    'weighted_contribution', NaN);
end

function residuals = localResidualTemplate()
residuals = struct();
labels = {'u_dot', 'v_dot', 'w_dot', 'p_dot', 'q_dot', 'r_dot', ...
          'phi_dot', 'theta_dot', 'psi_dot'};
for i = 1:numel(labels)
    residuals.(labels{i}) = struct( ...
        'present', false, ...
        'value', NaN, ...
        'units', '', ...
        'known', false, ...
        'steady', false, ...
        'block', '');
end
end
