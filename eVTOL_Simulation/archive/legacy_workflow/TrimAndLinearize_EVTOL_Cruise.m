% TrimAndLinearize_EVTOL_Cruise.m
%
% Trim the active Trim_Plant EVTOL model to steady level cruise and linearize it.
% This version assumes the current named grouped-state Trim_Plant layout.

if ~exist('trim_model_name', 'var') || isempty(trim_model_name)
    trim_model_name = 'Trim_Plant';
end
if ~exist('trim_verbose', 'var')
    trim_verbose = false;
end
if ~exist('trim_run_label', 'var')
    trim_run_label = '';
end

load_system(trim_model_name);

use_supplied_opspec = exist('opspec', 'var') && ~isempty(opspec);

if ~use_supplied_opspec
    SetupForTrimAndLin_EVTOL_Cruise;

    if exist('trim_setup_defaults', 'var') && isstruct(trim_setup_defaults)
        trim_defaults_local = trim_setup_defaults;
    else
        trim_defaults_local = struct();
        trim_defaults_local.model_name = trim_model_name;
        trim_defaults_local.Vinf_trim_target = Vinf_trim_target;
        trim_defaults_local.front_tilt_trim_deg = front_tilt_trim_deg;
        trim_defaults_local.front_collective_guess_rpm = front_collective_guess_rpm;
        trim_defaults_local.rear_collective_guess_rpm = rear_collective_guess_rpm;
        trim_defaults_local.front_collective_max_rpm = front_collective_max_rpm;
        trim_defaults_local.rear_collective_max_rpm = rear_collective_max_rpm;
        trim_defaults_local.surface_limit_rad = surface_limit_rad;
        trim_defaults_local.pos_init = pos_init(:);
        trim_defaults_local.trim = trim;
    end

    opspec_request = struct();
    opspec_request.Vinf_mps = Vinf_trim_target;
    opspec_request.front_tilt_deg = front_tilt_trim_deg;
    opspec_request.front_collective_guess_rpm = front_collective_guess_rpm;
    opspec_request.rear_collective_guess_rpm = rear_collective_guess_rpm;

    [opspec, opspec_request_resolved] = make_evtol_cruise_opspec(opspec_request, trim_defaults_local);
    use_supplied_opspec = false;
    trim_model_name = trim_defaults_local.model_name;
else
    if exist('trim_setup_defaults', 'var') && isstruct(trim_setup_defaults)
        trim_defaults_local = trim_setup_defaults;
    else
        trim_defaults_local = struct();
    end
    opspec_request_resolved = struct(); %#ok<NASGU>
    use_supplied_opspec = true;
    if isfield(trim_defaults_local, 'model_name') && ~isempty(trim_defaults_local.model_name)
        trim_model_name = trim_defaults_local.model_name;
    end
end

if ~exist('trim_verbose', 'var') || isempty(trim_verbose)
    trim_verbose = false;
end
if ~exist('trim_run_label', 'var') || isempty(trim_run_label)
    trim_run_label = '';
end

%% Dump what MATLAB thinks the operating-point structure is
if trim_verbose
    fprintf('\n=== opspec.States (%d entries) ===\n', numel(opspec.States));
    for i = 1:numel(opspec.States)
        s = opspec.States(i);

        block_str = '';
        try
            block_str = char(s.Block);
        catch
        end

        fprintf('  [%2d] Nx=%d  Block=%s\n', i, s.Nx, block_str);

        try
            fprintf('       x0          = [%s]\n', num2str(s.x(:).', ' %.6g'));
        catch
        end
        try
            fprintf('       Known       = [%s]\n', num2str(s.Known(:).', ' %d'));
        catch
        end
        try
            fprintf('       SteadyState = [%s]\n', num2str(s.SteadyState(:).', ' %d'));
        catch
        end
    end
    fprintf('=== end opspec.States ===\n\n');

    if ~use_supplied_opspec
        fprintf('=== opspec.Inputs (%d entries) ===\n', numel(opspec.Inputs));
        for i = 1:numel(opspec.Inputs)
            inp = opspec.Inputs(i);
            block_str = '';
            try
                block_str = char(inp.Block);
            catch
            end
            fprintf('  [%2d] size=%d  Block=%s\n', i, numel(inp.u), block_str);
        end
        fprintf('=== end opspec.Inputs ===\n\n');
    end
end

%% Find trim
fprintf('Running trim solver ...\n');
trim_display_report = 'off';
if trim_verbose
    trim_display_report = 'iter';
end
trim_opts = findopOptions('DisplayReport', trim_display_report);
[op_trim, op_report] = findop(trim_model_name, opspec, trim_opts);
fprintf('Trim termination: %s\n', op_report.TerminationString);

%% Extract trim states
validate_trim_op_layout(op_trim);

idx_eul   = find_op_state_idx(op_trim, 'euler_attitude_state');
idx_rates = find_op_state_idx(op_trim, 'body_rates_state');
idx_vel   = find_op_state_idx(op_trim, 'body_velocity_state');
idx_pos   = find_op_state_idx(op_trim, 'ned_position_state');

idx_dLT = find_op_state_idx(op_trim, 'tailL_servo');
idx_dRT = find_op_state_idx(op_trim, 'tailR_servo');
idx_dLW = find_op_state_idx(op_trim, 'wingL_servo');
idx_dRW = find_op_state_idx(op_trim, 'wingR_servo');

Att_Trim      = op_trim.States(idx_eul).x(:);
Vel_B_BA_Trim = op_trim.States(idx_vel).x(:);
Rates_Trim    = op_trim.States(idx_rates).x(:);
Pos_Trim      = op_trim.States(idx_pos).x(:);

% Controller expects [dLW; dRW; dLT; dRT]
Act_Trim = [op_trim.States(idx_dLW).x; ...
            op_trim.States(idx_dRW).x; ...
            op_trim.States(idx_dLT).x; ...
            op_trim.States(idx_dRT).x];

Att_Trim_deg = Att_Trim * 180 / pi;

Vinf_trim  = norm(Vel_B_BA_Trim);
Vel_W_Trim = [Vinf_trim; ...
              atan2(Vel_B_BA_Trim(3), Vel_B_BA_Trim(1)); ...
              asin(Vel_B_BA_Trim(2) / max(Vinf_trim, 0.1))];

X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim; Act_Trim];

motor_trim = op_trim.Inputs(1).u(:);
tilt_trim = op_trim.Inputs(2).u(:);

front_collective_sched_rpm = op_trim.Inputs(3).u;
rear_collective_sched_rpm = op_trim.Inputs(4).u;
front_tilt_sched_deg = mean(tilt_trim);

U_trim = [front_collective_sched_rpm; ...
          op_trim.Inputs(5).u; ...
          op_trim.Inputs(6).u; ...
          op_trim.Inputs(7).u; ...
          op_trim.Inputs(8).u];

U_trim_full = [motor_trim; ...
               tilt_trim; ...
               op_trim.Inputs(3).u; ...
               op_trim.Inputs(4).u; ...
               op_trim.Inputs(5).u; ...
               op_trim.Inputs(6).u; ...
               op_trim.Inputs(7).u; ...
               op_trim.Inputs(8).u];

fprintf('\n--- Cruise Trim Results ---\n');
fprintf('Att_Trim (deg): phi=%.3f theta=%.3f psi=%.3f\n', ...
    Att_Trim_deg(1), Att_Trim_deg(2), Att_Trim_deg(3));
fprintf('Vel_B_BA (m/s): u=%.4f v=%.4f w=%.4f\n', ...
    Vel_B_BA_Trim(1), Vel_B_BA_Trim(2), Vel_B_BA_Trim(3));
fprintf('Rates (rad/s):  P=%.4f Q=%.4f R=%.4f\n', ...
    Rates_Trim(1), Rates_Trim(2), Rates_Trim(3));
fprintf('front_coll: %.1f rpm   rear_coll: %.1f rpm\n', ...
    front_collective_sched_rpm, rear_collective_sched_rpm);
fprintf('surfaces (deg): dLW=%+.3f dRW=%+.3f dLT=%+.3f dRT=%+.3f\n', ...
    U_trim(2)*180/pi, U_trim(3)*180/pi, U_trim(4)*180/pi, U_trim(5)*180/pi);

%% Linearize about trim
fprintf('Linearizing about trim ...\n');
sys_full = linearize(trim_model_name, op_trim);
[A_full, B_full, C_full, D_full] = ssdata(sys_full);

if trim_verbose
    fprintf('Full linearized system: %d states, %d inputs, %d outputs\n', ...
        size(A_full,1), size(B_full,2), size(C_full,1));

    fprintf('\n=== sys_full.StateName ===\n');
    for i = 1:numel(sys_full.StateName)
        fprintf('  [%2d] %s\n', i, sys_full.StateName{i});
    end

    fprintf('\n=== sys_full.InputName ===\n');
    for i = 1:numel(sys_full.InputName)
        fprintf('  [%2d] %s\n', i, sys_full.InputName{i});
    end
end

%% Reduced 13-state model for control design
idx_eul = find_state_name_indices(sys_full.StateName, 'euler_attitude_state', 3);
idx_rates = find_state_name_indices(sys_full.StateName, 'body_rates_state', 3);
idx_vel = find_state_name_indices(sys_full.StateName, 'body_velocity_state', 3);

idx_phi   = idx_eul(1);
idx_theta = idx_eul(2);
idx_psi   = idx_eul(3);

idx_P     = idx_rates(1);
idx_Q     = idx_rates(2);
idx_R     = idx_rates(3);

idx_u     = idx_vel(1);
idx_v     = idx_vel(2);
idx_w     = idx_vel(3);

idx_dLT = find_state_name_indices(sys_full.StateName, 'tailL_servo', 1);
idx_dRT = find_state_name_indices(sys_full.StateName, 'tailR_servo', 1);
idx_dLW = find_state_name_indices(sys_full.StateName, 'wingL_servo', 1);
idx_dRW = find_state_name_indices(sys_full.StateName, 'wingR_servo', 1);

% Put actuator states in the order expected by the controller:
% [dLW dRW dLT dRT]
idx13 = [idx_phi, idx_theta, idx_psi, ...
         idx_u,   idx_v,     idx_w, ...
         idx_P,   idx_Q,     idx_R, ...
         idx_dLW, idx_dRW,   idx_dLT, idx_dRT];

state_names_13 = {'phi','theta','psi','u','v','w','P','Q','R', ...
                  'dLW','dRW','dLT','dRT'};
ctrl_names     = {'front_coll_rpm','deltaLW','deltaRW','deltaLT','deltaRT'};
surface_input_idx = 5:8;

B_front_collective = B_full(idx13, 3);
B_ctrl = [B_front_collective, B_full(idx13, surface_input_idx)];

sys_ss_13state = ss(A_full(idx13, idx13), ...
                    B_ctrl, ...
                    eye(numel(idx13)), ...
                    zeros(numel(idx13), numel(ctrl_names)));

sys_ss_13state.StateName = state_names_13(:);
sys_ss_13state.InputName = ctrl_names(:);

fprintf('13-state system eigenvalues:\n');
lambda_13 = eig(sys_ss_13state.A);
disp(lambda_13);
print_eigenmode_summary(sys_ss_13state.A, state_names_13);

%% Save to struct for batch workflows
trim_linearize_result = struct();
trim_linearize_result.name = trim_run_label;
trim_linearize_result.success = true;

trim_linearize_result.meta = struct();
trim_linearize_result.meta.model_name = trim_model_name;
trim_linearize_result.meta.run_label = trim_run_label;
trim_linearize_result.meta.created_on = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));

trim_linearize_result.opspec = opspec;
trim_linearize_result.op_trim = op_trim;
trim_linearize_result.op_report = op_report;

trim_linearize_result.trim = struct();
trim_linearize_result.trim.Att_Trim = Att_Trim;
trim_linearize_result.trim.Att_Trim_deg = Att_Trim_deg;
trim_linearize_result.trim.Vel_B_BA_Trim = Vel_B_BA_Trim;
trim_linearize_result.trim.Vel_W_Trim = Vel_W_Trim;
trim_linearize_result.trim.Rates_Trim = Rates_Trim;
trim_linearize_result.trim.Pos_Trim = Pos_Trim;
trim_linearize_result.trim.Act_Trim = Act_Trim;
trim_linearize_result.trim.X_trim = X_trim;
trim_linearize_result.trim.U_trim = U_trim;
trim_linearize_result.trim.U_trim_full = U_trim_full;

trim_linearize_result.linear = struct();
trim_linearize_result.linear.sys_full = sys_full;
trim_linearize_result.linear.A_full = A_full;
trim_linearize_result.linear.B_full = B_full;
trim_linearize_result.linear.C_full = C_full;
trim_linearize_result.linear.D_full = D_full;
trim_linearize_result.linear.sys_ss_13state = sys_ss_13state;

trim_linearize_result.scheduling = struct();
trim_linearize_result.scheduling.Vinf_mps = Vel_W_Trim(1);
trim_linearize_result.scheduling.alpha_rad = Vel_W_Trim(2);
trim_linearize_result.scheduling.beta_rad = Vel_W_Trim(3);
trim_linearize_result.scheduling.phi_rad = Att_Trim(1);
trim_linearize_result.scheduling.theta_rad = Att_Trim(2);
trim_linearize_result.scheduling.psi_rad = Att_Trim(3);
trim_linearize_result.scheduling.front_tilt_deg = front_tilt_sched_deg;
trim_linearize_result.scheduling.front_collective_rpm = front_collective_sched_rpm;
trim_linearize_result.scheduling.rear_collective_rpm = rear_collective_sched_rpm;

function idx = find_op_state_idx(op_trim, block_fragment)
    idx = [];
    for i = 1:numel(op_trim.States)
        try
            blk = char(op_trim.States(i).Block);
        catch
            blk = '';
        end
        if contains(blk, block_fragment)
            idx = i;
            return;
        end
    end
    error('Unable to find op_trim state containing block fragment "%s".', block_fragment);
end

function idx = find_state_name_indices(state_names, fragment, expected_count)
    idx = find(contains(state_names, fragment));
    if numel(idx) ~= expected_count
        error('Expected %d linearized state(s) containing "%s", found %d.', ...
            expected_count, fragment, numel(idx));
    end
    idx = idx(:).';
end

function validate_trim_op_layout(op_trim)
    if numel(op_trim.States) ~= 14
        error('Expected op_trim to expose 14 grouped states, found %d.', numel(op_trim.States));
    end
    if numel(op_trim.Inputs) ~= 8
        error('Expected op_trim to expose 8 grouped inputs, found %d.', numel(op_trim.Inputs));
    end
end

function print_eigenmode_summary(A, state_names)
    [V, D] = eig(A);
    lambda = diag(D);

    fprintf('Dominant-state summary by eigenmode:\n');
    for i = 1:numel(lambda)
        weights = abs(V(:, i));
        if max(weights) > 0
            weights = weights / max(weights);
        end

        [~, order] = sort(weights, 'descend');
        top_count = min(3, numel(order));

        labels = cell(1, top_count);
        for k = 1:top_count
            idx = order(k);
            labels{k} = sprintf('%s (%.2f)', state_names{idx}, weights(idx));
        end

        fprintf('  mode %2d: %+.4f %+.4fi  dominant: %s\n', ...
            i, real(lambda(i)), imag(lambda(i)), strjoin(labels, ', '));
    end

    unstable_idx = find(real(lambda) > 1e-6);
    if isempty(unstable_idx)
        fprintf('  No unstable eigenvalues detected.\n');
    else
        fprintf('  Unstable modes: %s\n', mat2str(unstable_idx(:).'));
    end
end
