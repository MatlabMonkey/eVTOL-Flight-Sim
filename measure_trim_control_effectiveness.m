function report = measure_trim_control_effectiveness(trimResult, opts)
% trim control check

if nargin < 2 || isempty(opts)
    opts = struct();
end

if ~isfield(opts, 'delta_deg')
    opts.delta_deg = 1.0;
end
if ~isfield(opts, 'sim_stop_time')
    opts.sim_stop_time = 0.02;
end

delta_rad = deg2rad(opts.delta_deg);

if ~isstruct(trimResult)
    error('trimResult must be the struct returned by Trim_Main.');
end
if ~isfield(trimResult, 'op_trim') || isempty(trimResult.op_trim)
    error('trimResult.op_trim is required for the finite-difference test.');
end
if ~isfield(trimResult, 'linear') || ~isfield(trimResult.linear, 'sys_ss_9state')
    error('trimResult.linear.sys_ss_9state is required for reduced-model comparison.');
end

fprintf('\n=== Trim-Point Control Effectiveness Check ===\n');
fprintf('Trim point: %s (%s)\n', local_get_field(trimResult, 'name', 'unnamed_trim'), ...
    local_get_field(trimResult, 'mode', 'unknown_mode'));
fprintf('Perturbation: %.3f deg  |  sim stop time: %.3f s\n', opts.delta_deg, opts.sim_stop_time);

op_trim = trimResult.op_trim;
motor_trim = trimResult.U_trim_full(1:4);
tilt_trim = trimResult.U_trim_full(5:6);
front_collective_sched_rpm = trimResult.U_trim_full(7);
rear_collective_sched_rpm = trimResult.U_trim_full(8);
U_trim = trimResult.U_trim;

prop = evalin('base', 'prop');
wingL = evalin('base', 'wingL');
wingR = evalin('base', 'wingR');
tailL = evalin('base', 'tailL');
tailR = evalin('base', 'tailR');

load_system('Trim_Plant');
local_enable_named_signal_logging('Trim_Plant', {'body_force_sum', 'body_moment_sum'});

trim_state = local_build_trim_state(op_trim, prop, wingL, wingR, tailL, tailR);
trim_input = local_build_trim_input(trimResult, motor_trim, tilt_trim, ...
    front_collective_sched_rpm, rear_collective_sched_rpm);

cases = [ ...
    struct('label', "baseline", 'surface_delta', [0; 0; 0; 0], 'mixed_label', ""), ...
    struct('label', "+delta_f", 'surface_delta', [1; 1; 0; 0] * delta_rad, 'mixed_label', "delta_f"), ...
    struct('label', "+delta_a", 'surface_delta', [1; -1; 0; 0] * delta_rad, 'mixed_label', "delta_a"), ...
    struct('label', "+delta_e", 'surface_delta', [0; 0; 1; 1] * delta_rad, 'mixed_label', "delta_e"), ...
    struct('label', "+delta_r", 'surface_delta', [0; 0; -1; 1] * delta_rad, 'mixed_label', "delta_r") ...
];

results = repmat(struct( ...
    'label', "", ...
    'mixed_label', "", ...
    'surface_delta_rad', zeros(4, 1), ...
    'body_force_N', zeros(3, 1), ...
    'body_moment_Nm', zeros(3, 1), ...
    'dF_drad_N_per_rad', zeros(3, 1), ...
    'dM_drad_Nm_per_rad', zeros(3, 1), ...
    'B_column', zeros(9, 1)), numel(cases), 1);

for k = 1:numel(cases)
    case_input = trim_input;
    case_input.mixed_control_rad = trim_input.mixed_control_rad + cases(k).surface_delta;
    simOut = local_run_trim_point(case_input, trim_state, opts.sim_stop_time);

    results(k).label = cases(k).label;
    results(k).mixed_label = cases(k).mixed_label;
    results(k).surface_delta_rad = cases(k).surface_delta;
    results(k).body_force_N = local_read_logged_vector(simOut, 'body_force_sum');
    results(k).body_moment_Nm = local_read_logged_vector(simOut, 'body_moment_sum');
end

baseline_force = results(1).body_force_N;
baseline_moment = results(1).body_moment_Nm;

sys9 = trimResult.linear.sys_ss_9state;
input_names = string(sys9.InputName);

for k = 2:numel(results)
    results(k).dF_drad_N_per_rad = ...
        (results(k).body_force_N - baseline_force) / delta_rad;
    results(k).dM_drad_Nm_per_rad = ...
        (results(k).body_moment_Nm - baseline_moment) / delta_rad;

    if results(k).mixed_label ~= ""
        idx = find(input_names == results(k).mixed_label, 1);
        if ~isempty(idx)
            results(k).B_column = sys9.B(:, idx);
        end
    end
end

fprintf('\nBaseline body moment [N*m] = [% .4f, % .4f, % .4f]^T\n', baseline_moment.');
fprintf('Empirical finite-difference results:\n');
for k = 2:numel(results)
    fprintf('%s:\n', results(k).label);
    fprintf('  dF/d(delta) [N/rad]   = [% .4f, % .4f, % .4f]^T\n', results(k).dF_drad_N_per_rad.');
    fprintf('  dM/d(delta) [N*m/rad] = [% .4f, % .4f, % .4f]^T\n', results(k).dM_drad_Nm_per_rad.');
    fprintf('  B column [phi theta psi u v w P Q R]^T =\n');
    fprintf('    [% .4f, % .4f, % .4f, % .4f, % .4f, % .4f, % .4f, % .4f, % .4f]^T\n', ...
        results(k).B_column.');
end

report = struct();
report.trim_name = local_get_field(trimResult, 'name', 'unnamed_trim');
report.trim_mode = local_get_field(trimResult, 'mode', 'unknown_mode');
report.delta_deg = opts.delta_deg;
report.sim_stop_time = opts.sim_stop_time;
report.results = results;
report.input_names = input_names;

assignin('base', 'trim_point_control_effectiveness_report', report);
fprintf('\nSaved report to base workspace variable trim_point_control_effectiveness_report.\n');
end

function trim_state = local_build_trim_state(op_trim, prop_in, wingL_in, wingR_in, tailL_in, tailR_in)
trim_state = struct();
trim_state.prop = prop_in;
trim_state.wingL = wingL_in;
trim_state.wingR = wingR_in;
trim_state.tailL = tailL_in;
trim_state.tailR = tailR_in;

trim_state.V_init = local_find_op_state_vector(op_trim, 'body_velocity_state');
trim_state.eul_init = local_find_op_state_vector(op_trim, 'euler_attitude_state');
trim_state.omega_init = local_find_op_state_vector(op_trim, 'body_rates_state');
end

function trim_input = local_build_trim_input(trimResult, motor_trim_in, tilt_trim_in, front_coll_in, rear_coll_in)
trim_input = struct();
trim_input.motor_rpm_cmd = motor_trim_in(:);
trim_input.tilt_deg = tilt_trim_in(:);
trim_input.front_collective = front_coll_in;
trim_input.rear_collective = rear_coll_in;

if isfield(trimResult, 'trim') && isfield(trimResult.trim, 'direct_surface_trim') ...
        && isfield(trimResult.trim, 'mixed_control_trim')
    trim_input.direct_surface_rad = trimResult.trim.direct_surface_trim(:);
    trim_input.mixed_control_rad = trimResult.trim.mixed_control_trim(:);
else
    trim_input.direct_surface_rad = zeros(4, 1);
    trim_input.mixed_control_rad = trimResult.U_trim(2:5);
end
end

function simOut = local_run_trim_point(trim_input, trim_state, sim_stop_time)
t = [0; sim_stop_time];
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(repmat(trim_input.motor_rpm_cmd.', numel(t), 1), t), 'Motor_RPM_cmd');
ds = ds.addElement(timeseries(repmat(trim_input.tilt_deg.', numel(t), 1), t), 'Tilt_angles_cmd');
ds = ds.addElement(timeseries(repmat(trim_input.front_collective, numel(t), 1), t), 'Front_RPM_collective');
ds = ds.addElement(timeseries(repmat(trim_input.rear_collective, numel(t), 1), t), 'Rear_RPM_collective');
ds = ds.addElement(timeseries(repmat(trim_input.direct_surface_rad(1), numel(t), 1), t), 'delta_LW');
ds = ds.addElement(timeseries(repmat(trim_input.direct_surface_rad(2), numel(t), 1), t), 'delta_RW');
ds = ds.addElement(timeseries(repmat(trim_input.direct_surface_rad(3), numel(t), 1), t), 'delta_LT');
ds = ds.addElement(timeseries(repmat(trim_input.direct_surface_rad(4), numel(t), 1), t), 'delta_RT');
ds = ds.addElement(timeseries(repmat(trim_input.mixed_control_rad(1), numel(t), 1), t), 'delta_f');
ds = ds.addElement(timeseries(repmat(trim_input.mixed_control_rad(2), numel(t), 1), t), 'delta_a');
ds = ds.addElement(timeseries(repmat(trim_input.mixed_control_rad(3), numel(t), 1), t), 'delta_e');
ds = ds.addElement(timeseries(repmat(trim_input.mixed_control_rad(4), numel(t), 1), t), 'delta_r');

simIn = Simulink.SimulationInput('Trim_Plant');
simIn = simIn.setVariable('prop', trim_state.prop);
simIn = simIn.setVariable('wingL', trim_state.wingL);
simIn = simIn.setVariable('wingR', trim_state.wingR);
simIn = simIn.setVariable('tailL', trim_state.tailL);
simIn = simIn.setVariable('tailR', trim_state.tailR);
simIn = simIn.setVariable('V_init', trim_state.V_init);
simIn = simIn.setVariable('eul_init', trim_state.eul_init);
simIn = simIn.setVariable('omega_init', trim_state.omega_init);
simIn = simIn.setExternalInput(ds);
simIn = simIn.setModelParameter( ...
    'StopTime', num2str(sim_stop_time), ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout');

simOut = sim(simIn);
end

function vec = local_read_logged_vector(simOut, signal_name)
sig = simOut.logsout.get(signal_name);
if isempty(sig)
    error('Did not find logged signal "%s".', signal_name);
end

data = sig.Values.Data;
time_values = sig.Values.Time;
sz = size(data);
time_dim = find(sz == numel(time_values), 1, 'last');

if isempty(time_dim)
    slice = squeeze(data);
else
    indices = repmat({':'}, 1, ndims(data));
    indices{time_dim} = sz(time_dim);
    slice = squeeze(data(indices{:}));
end

vec = slice(:);
end

function value = local_find_op_state_value(op_trim, block_fragment)
idx = local_find_op_state_index(op_trim, block_fragment);
value = op_trim.States(idx).x(1);
end

function vec = local_find_op_state_vector(op_trim, block_fragment)
idx = local_find_op_state_index(op_trim, block_fragment);
vec = op_trim.States(idx).x(:);
end

function idx = local_find_op_state_index(op_trim, block_fragment)
idx = [];
for i = 1:numel(op_trim.States)
    blk = '';
    try
        blk = char(op_trim.States(i).Block);
    catch
    end
    if contains(blk, block_fragment)
        idx = i;
        return;
    end
end
error('Unable to find operating-point state containing "%s".', block_fragment);
end

function local_enable_named_signal_logging(model_name, signal_names)
lines = find_system(model_name, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(lines)
    line_name = get_param(lines(k), 'Name');
    if ~ismember(line_name, signal_names)
        continue;
    end
    try
        set_param(lines(k), 'DataLogging', 'on');
    catch
    end
    try
        set_param(lines(k), 'DataLoggingNameMode', 'Custom');
        set_param(lines(k), 'DataLoggingName', line_name);
    catch
    end
end
end

function value = local_get_field(s, field_name, fallback)
value = fallback;
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
end
end
