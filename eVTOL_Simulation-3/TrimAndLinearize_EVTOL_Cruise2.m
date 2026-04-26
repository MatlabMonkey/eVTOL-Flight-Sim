% TrimAndLinearize_EVTOL_Cruise.m
%
% Trim the grouped-state EVTOL plant to steady level cruise and linearize it.
% This version handles opspec.States entries that are vectors.

if ~exist('trim_model_name', 'var') || isempty(trim_model_name)
    trim_model_name = 'EVTOL_6DOF_Plant_EUL';
end
if ~exist('trim_verbose', 'var')
    trim_verbose = true;
end
if ~exist('trim_run_label', 'var')
    trim_run_label = '';
end

load_system(trim_model_name);

use_supplied_opspec = exist('opspec', 'var') && ~isempty(opspec);

if ~use_supplied_opspec
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
else
    opspec_request_resolved = struct();
end

%% Dump what MATLAB thinks the grouped operating-point structure is
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

if ~use_supplied_opspec
    %% Grouped state indices from your current opspec dump
    idx_dLT   = 1;
    idx_dLW   = 2;
    idx_dRT   = 3;
    idx_dRW   = 4;
    idx_eul   = 5;
    idx_rates = 6;
    idx_vel   = 7;
    idx_pos   = 8;

    %% State constraints

    % Surface actuator states: free, steady
    for k = [idx_dLT, idx_dLW, idx_dRT, idx_dRW]
        opspec.States(k).x           = 0.0;
        opspec.States(k).Known       = false;
        opspec.States(k).SteadyState = true;
    end

    % Euler angles = [phi; theta; psi]
    % Hold phi = 0 and psi = psi_trim_target, let theta float to trim.
    opspec.States(idx_eul).x           = [0.0; 0.0; 0.0];
    opspec.States(idx_eul).Known       = [true; false; true];
    opspec.States(idx_eul).SteadyState = [true; true; true];

    % Body velocity = [u; v; w]
    % Hold u = Vinf target, hold v = 0, hold w = 0.
    opspec.States(idx_vel).x           = [Vinf_trim_target; 0.0; 0.0];
    opspec.States(idx_vel).Known       = [true; true; true];
    opspec.States(idx_vel).SteadyState = [true; true; true];

    % Body rates = [P; Q; R] = 0
    opspec.States(idx_rates).x           = [0.0; 0.0; 0.0];
    opspec.States(idx_rates).Known       = [true; true; true];
    opspec.States(idx_rates).SteadyState = [true; true; true];

    % Position = [x; y; z]
    % Let x,y drift. Enforce z_dot = 0 for level flight.
    opspec.States(idx_pos).x           = pos_init(:);
    opspec.States(idx_pos).Known       = [false; false; false];
    opspec.States(idx_pos).SteadyState = [false; false; true];

    %% Input constraints
    % Based on your current input grouping:
    % 1 = Motor_RPM_cmd(12)
    % 2 = Tilt_angles_cmd(6)
    % 3 = Front_RPM_collective
    % 4 = Rear_RPM_collective
    % 5 = deltaLW
    % 6 = deltaRW
    % 7 = deltaLT
    % 8 = deltaRT

    opspec.Inputs(1).u     = zeros(12, 1);
    opspec.Inputs(1).Known = true(12, 1);

    opspec.Inputs(2).u     = front_tilt_trim_deg * ones(6, 1);
    opspec.Inputs(2).Known = true(6, 1);

    opspec.Inputs(3).u     = front_collective_guess_rpm;
    opspec.Inputs(3).Known = false;
    opspec.Inputs(3).Min   = 0.0;
    opspec.Inputs(3).Max   = front_collective_max_rpm;

    opspec.Inputs(4).u     = rear_collective_guess_rpm;
    opspec.Inputs(4).Known = false;
    opspec.Inputs(4).Min   = 0.0;
    opspec.Inputs(4).Max   = rear_collective_max_rpm;

    for i_in = 5:8
        opspec.Inputs(i_in).u     = 0.0;
        opspec.Inputs(i_in).Known = false;
        opspec.Inputs(i_in).Min   = -surface_limit_rad;
        opspec.Inputs(i_in).Max   =  surface_limit_rad;
    end
end

%% Find trim
fprintf('Running trim solver ...\n');
trim_opts = findopOptions('DisplayReport', 'iter');
[op_trim, op_report] = findop(trim_model_name, opspec, trim_opts);
fprintf('Trim termination: %s\n', op_report.TerminationString);

%% Extract trim states/inputs with layout awareness
n_state_groups = numel(op_trim.States);
n_input_groups = numel(op_trim.Inputs);

if n_state_groups >= 8
    op_state_layout = 'grouped';
elseif n_state_groups == 1
    op_state_layout = 'packed';
else
    error('Unexpected op_trim state layout. numel(op_trim.States) = %d', n_state_groups);
end

if n_input_groups >= 8
    op_input_layout = 'grouped';
elseif n_input_groups == 1
    op_input_layout = 'packed';
else
    error('Unexpected op_trim input layout. numel(op_trim.Inputs) = %d', n_input_groups);
end

switch op_state_layout
    case 'grouped'
        idx_dLT   = 1;
        idx_dLW   = 2;
        idx_dRT   = 3;
        idx_dRW   = 4;
        idx_eul   = 5;
        idx_rates = 6;
        idx_vel   = 7;
        idx_pos   = 8;

        Att_Trim      = op_trim.States(idx_eul).x(:);
        Vel_B_BA_Trim = op_trim.States(idx_vel).x(:);
        Rates_Trim    = op_trim.States(idx_rates).x(:);
        Pos_Trim      = op_trim.States(idx_pos).x(:);

        % Controller expects [dLW; dRW; dLT; dRT]
        Act_Trim = [op_trim.States(idx_dLW).x; ...
                    op_trim.States(idx_dRW).x; ...
                    op_trim.States(idx_dLT).x; ...
                    op_trim.States(idx_dRT).x];

    case 'packed'
        xop = op_trim.States(1).x(:);
        if numel(xop) < 16
            error('Packed op_trim state vector is shorter than 16 elements.');
        end

        % Packed state order:
        % 1 dLT, 2 dLW, 3 dRT, 4 dRW, 5 phi, 6 theta, 7 psi,
        % 8 P, 9 Q, 10 R, 11 u, 12 v, 13 w, 14 x, 15 y, 16 z

        Att_Trim      = xop(5:7);
        Rates_Trim    = xop(8:10);
        Vel_B_BA_Trim = xop(11:13);
        Pos_Trim      = xop(14:16);

        % Controller expects [dLW; dRW; dLT; dRT]
        Act_Trim = [xop(2); xop(4); xop(1); xop(3)];

    otherwise
        error('Unsupported op_trim state layout: %s', op_state_layout);
end

Att_Trim_deg = Att_Trim * 180 / pi;

Vinf_trim  = norm(Vel_B_BA_Trim);
Vel_W_Trim = [Vinf_trim; ...
              atan2(Vel_B_BA_Trim(3), Vel_B_BA_Trim(1)); ...
              asin(Vel_B_BA_Trim(2) / max(Vinf_trim, 0.1))];

X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim; Act_Trim];

switch op_input_layout
    case 'grouped'
        U_trim = [op_trim.Inputs(3).u; ...
                  op_trim.Inputs(5).u; ...
                  op_trim.Inputs(6).u; ...
                  op_trim.Inputs(7).u; ...
                  op_trim.Inputs(8).u];

        U_trim_full = zeros(24,1);
        U_trim_full(19) = op_trim.Inputs(3).u;
        U_trim_full(20) = op_trim.Inputs(4).u;
        U_trim_full(21) = op_trim.Inputs(5).u;
        U_trim_full(22) = op_trim.Inputs(6).u;
        U_trim_full(23) = op_trim.Inputs(7).u;
        U_trim_full(24) = op_trim.Inputs(8).u;

        front_tilt_sched_deg = op_trim.Inputs(2).u(1);
        front_collective_sched_rpm = op_trim.Inputs(3).u;
        rear_collective_sched_rpm = op_trim.Inputs(4).u;

    case 'packed'
        uop = op_trim.Inputs(1).u(:);
        if numel(uop) < 24
            error('Packed op_trim input vector is shorter than 24 elements.');
        end

        U_trim = [uop(19); ...
                  uop(21); ...
                  uop(22); ...
                  uop(23); ...
                  uop(24)];

        U_trim_full = uop(1:24);

        front_tilt_sched_deg = uop(13);
        front_collective_sched_rpm = uop(19);
        rear_collective_sched_rpm = uop(20);

    otherwise
        error('Unsupported op_trim input layout: %s', op_input_layout);
end

if trim_verbose
    fprintf('\n--- Cruise Trim Results ---\n');
    fprintf('Att_Trim (deg): phi=%.3f theta=%.3f psi=%.3f\n', ...
        Att_Trim_deg(1), Att_Trim_deg(2), Att_Trim_deg(3));
    fprintf('Vel_B_BA (m/s): u=%.4f v=%.4f w=%.4f\n', ...
        Vel_B_BA_Trim(1), Vel_B_BA_Trim(2), Vel_B_BA_Trim(3));
    fprintf('Rates (rad/s):  P=%.4f Q=%.4f R=%.4f\n', ...
        Rates_Trim(1), Rates_Trim(2), Rates_Trim(3));
    fprintf('front_coll: %.1f rpm\n', U_trim(1));
    fprintf('surfaces (deg): dLW=%+.3f dRW=%+.3f dLT=%+.3f dRT=%+.3f\n', ...
        U_trim(2)*180/pi, U_trim(3)*180/pi, U_trim(4)*180/pi, U_trim(5)*180/pi);
end

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
% Actual linearized state order:
% 1 dLT, 2 dLW, 3 dRT, 4 dRW, 5 phi, 6 theta, 7 psi,
% 8 P, 9 Q, 10 R, 11 u, 12 v, 13 w, 14 x, 15 y, 16 z

idx_phi   = 5;
idx_theta = 6;
idx_psi   = 7;

idx_P     = 8;
idx_Q     = 9;
idx_R     = 10;

idx_u     = 11;
idx_v     = 12;
idx_w     = 13;

idx_dLT   = 1;
idx_dLW   = 2;
idx_dRT   = 3;
idx_dRW   = 4;

% Put actuator states in the order expected by the controller:
% [dLW dRW dLT dRT]
idx13 = [idx_phi, idx_theta, idx_psi, ...
         idx_u,   idx_v,     idx_w, ...
         idx_P,   idx_Q,     idx_R, ...
         idx_dLW, idx_dRW,   idx_dLT, idx_dRT];

ctrl_idx = [19, 21, 22, 23, 24];

state_names_13 = {'phi','theta','psi','u','v','w','P','Q','R', ...
                  'dLW','dRW','dLT','dRT'};
ctrl_names     = {'front_coll_rpm','deltaLW','deltaRW','deltaLT','deltaRT'};

sys_ss_13state = ss(A_full(idx13, idx13), ...
                    B_full(idx13, ctrl_idx), ...
                    eye(numel(idx13)), ...
                    zeros(numel(idx13), numel(ctrl_idx)));

sys_ss_13state.StateName = state_names_13(:);
sys_ss_13state.InputName = ctrl_names(:);

fprintf('13-state eigenvalues:\n');
disp(eig(sys_ss_13state.A));

%% Save to struct
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