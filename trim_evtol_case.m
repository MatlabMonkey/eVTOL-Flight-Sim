function [trimResult, trimSpec] = trim_evtol_case(initData, trimCase, options)
%TRIM_EVTOL_CASE Solve one trim case and optionally linearize it.
%
% This is the shared trim implementation used by:
%   - Trim_Main.m
%   - TrimSearch_Run.m

    if nargin < 3 || isempty(options)
        options = struct();
    end

    if ~isstruct(initData)
        error('initData must be a struct. Run Init_Main first.');
    end
    if ~isstruct(trimCase)
        error('trimCase must be a struct.');
    end

    trim_verbose = localGetField(options, 'verbose', false);
    trim_debug = localGetField(options, 'debug', false);
    emit_summary = localGetField(options, 'emitSummary', true);
    emit_linear_summary = localGetField(options, 'emitLinearSummary', true);
    compute_linearization = localGetField(options, 'computeLinearization', true);

    trimSpec = localBuildTrimSpec(initData, trimCase);
    trim_model_name = trimSpec.modelName;

    trim = trimSpec.trim; %#ok<NASGU>
    trim_validation_stop_time = trimSpec.trim_validation_stop_time_s; %#ok<NASGU>
    surface_limit_deg = trimSpec.surface_limit_deg_scalar; %#ok<NASGU>
    surface_limit_rad = trimSpec.surface_limit_rad_scalar; %#ok<NASGU>

    if emit_summary
        fprintf('Trim case: %s (%s)\n', trimSpec.name, trimSpec.mode);
        fprintf('  model = %s\n', trim_model_name);
        fprintf('  Vinf  = %.1f m/s\n', trimSpec.Vinf_mps);
        fprintf('  tilt  = %.1f deg\n', trimSpec.front_tilt_deg);
        fprintf('  front collective guess = %.1f rpm\n', trimSpec.front_collective_guess_rpm);
        fprintf('  rear collective guess  = %.1f rpm\n', trimSpec.rear_collective_guess_rpm);
    end

    load_system(trim_model_name);
    localConfigureTrimModelInterface(trim_model_name);

    if trim_debug
        localPrintTrimEnvironmentFingerprint(trim_model_name);
    end

    opspec = localBuildCruiseOpspec(trimSpec);

    if trim_verbose
        localDumpOpspec(opspec);
    end

    if emit_summary
        fprintf('Running trim solver ...\n');
    end

    trim_display_report = 'off';
    if trim_verbose
        trim_display_report = 'iter';
    end
    trim_opts = findopOptions('DisplayReport', trim_display_report);
    warningStateRank = warning('off', 'MATLAB:rankDeficientMatrix');
    warningStateSing = warning('off', 'MATLAB:singularMatrix');
    warningStateNearSing = warning('off', 'MATLAB:nearlySingularMatrix');
    warningCleanup = onCleanup(@() localRestoreWarningStates( ...
        warningStateRank, warningStateSing, warningStateNearSing)); %#ok<NASGU>
    [op_trim, op_report] = findop(trim_model_name, opspec, trim_opts);

    if emit_summary
        fprintf('Trim termination: %s\n', op_report.TerminationString);
    end

    is_exact_trim = contains(op_report.TerminationString, 'successfully met', 'IgnoreCase', true);
    if trim_debug
        localPrintTrimReportSummary(op_report);
    end

    localValidateTrimOpLayout(op_trim);

    idx_eul   = localFindOpStateIdx(op_trim, 'euler_attitude_state');
    idx_rates = localFindOpStateIdx(op_trim, 'body_rates_state');
    idx_vel   = localFindOpStateIdx(op_trim, 'body_velocity_state');
    idx_pos   = localFindOpStateIdx(op_trim, 'ned_position_state');

    Att_Trim      = op_trim.States(idx_eul).x(:);
    Vel_B_BA_Trim = op_trim.States(idx_vel).x(:);
    Rates_Trim    = op_trim.States(idx_rates).x(:);
    Pos_Trim      = op_trim.States(idx_pos).x(:);

    Att_Trim_deg = Att_Trim * 180 / pi;

    Vinf_trim = norm(Vel_B_BA_Trim);
    beta_arg = 0.0;
    if Vinf_trim > 0.1
        beta_arg = Vel_B_BA_Trim(2) / Vinf_trim;
    end
    beta_arg = max(min(beta_arg, 1.0), -1.0);
    Vel_W_Trim = [Vinf_trim; ...
                  atan2(Vel_B_BA_Trim(3), Vel_B_BA_Trim(1)); ...
                  asin(beta_arg)];

    motor_trim = op_trim.Inputs(1).u(:);
    tilt_trim = op_trim.Inputs(2).u(:);
    front_collective_sched_rpm = op_trim.Inputs(3).u;
    rear_collective_sched_rpm = op_trim.Inputs(4).u;
    front_tilt_sched_deg = mean(tilt_trim);

    direct_surface_trim = [op_trim.Inputs(5).u; ...
                           op_trim.Inputs(6).u; ...
                           op_trim.Inputs(7).u; ...
                           op_trim.Inputs(8).u];
    mixed_control_trim = [op_trim.Inputs(9).u; ...
                          op_trim.Inputs(10).u; ...
                          op_trim.Inputs(11).u; ...
                          op_trim.Inputs(12).u];
    physical_surface_trim = direct_surface_trim + trimSpec.mixMatrix_faer * mixed_control_trim;
    Act_Trim = physical_surface_trim;

    X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim];

    U_trim = [front_collective_sched_rpm; mixed_control_trim];
    U_surface_trim = [front_collective_sched_rpm; physical_surface_trim];

    U_trim_full = [motor_trim; ...
                   tilt_trim; ...
                   op_trim.Inputs(3).u; ...
                   op_trim.Inputs(4).u; ...
                   physical_surface_trim];

    trim_plant_inputs = [motor_trim; ...
                         tilt_trim; ...
                         op_trim.Inputs(3).u; ...
                         op_trim.Inputs(4).u; ...
                         direct_surface_trim; ...
                         mixed_control_trim];

    if emit_summary && compute_linearization
        fprintf('\n--- Trim Results ---\n');
        fprintf('Att_Trim (deg): phi=%.3f theta=%.3f psi=%.3f\n', ...
            Att_Trim_deg(1), Att_Trim_deg(2), Att_Trim_deg(3));
        fprintf('Vel_B_BA (m/s): u=%.4f v=%.4f w=%.4f\n', ...
            Vel_B_BA_Trim(1), Vel_B_BA_Trim(2), Vel_B_BA_Trim(3));
        fprintf('Rates (rad/s):  P=%.4f Q=%.4f R=%.4f\n', ...
            Rates_Trim(1), Rates_Trim(2), Rates_Trim(3));
        fprintf('front_coll: %.1f rpm   rear_coll: %.1f rpm\n', ...
            front_collective_sched_rpm, rear_collective_sched_rpm);
        fprintf('mixed ctrls (deg): df=%+.3f da=%+.3f de=%+.3f dr=%+.3f\n', ...
            mixed_control_trim(1) * 180 / pi, mixed_control_trim(2) * 180 / pi, ...
            mixed_control_trim(3) * 180 / pi, mixed_control_trim(4) * 180 / pi);
        fprintf('surfaces (deg):    dLW=%+.3f dRW=%+.3f dLT=%+.3f dRT=%+.3f\n', ...
            physical_surface_trim(1) * 180 / pi, physical_surface_trim(2) * 180 / pi, ...
            physical_surface_trim(3) * 180 / pi, physical_surface_trim(4) * 180 / pi);
        fprintf('Linearizing about trim ...\n');
    end

    trimResult = struct();
    trimResult.name = trimSpec.name;
    trimResult.mode = trimSpec.mode;
    trimResult.modelName = trim_model_name;
    trimResult.success = is_exact_trim;
    trimResult.isExactTrim = is_exact_trim;
    trimResult.terminationString = op_report.TerminationString;
    trimResult.case = trimCase;
    trimResult.trimSpec = trimSpec;

    trimResult.Att_Trim = Att_Trim;
    trimResult.Att_Trim_deg = Att_Trim_deg;
    trimResult.Vel_B_BA_Trim = Vel_B_BA_Trim;
    trimResult.Vel_W_Trim = Vel_W_Trim;
    trimResult.Rates_Trim = Rates_Trim;
    trimResult.Pos_Trim = Pos_Trim;
    trimResult.Act_Trim = Act_Trim;
    trimResult.X_trim = X_trim;
    trimResult.U_trim = U_trim;
    trimResult.U_surface_trim = U_surface_trim;
    trimResult.U_trim_full = U_trim_full;
    trimResult.trimPlantInputs = trim_plant_inputs;
    trimResult.front_tilt_deg = front_tilt_sched_deg;

    trimResult.trim = struct();
    trimResult.trim.Att_Trim = Att_Trim;
    trimResult.trim.Att_Trim_deg = Att_Trim_deg;
    trimResult.trim.Vel_B_BA_Trim = Vel_B_BA_Trim;
    trimResult.trim.Vel_W_Trim = Vel_W_Trim;
    trimResult.trim.Rates_Trim = Rates_Trim;
    trimResult.trim.Pos_Trim = Pos_Trim;
    trimResult.trim.Act_Trim = Act_Trim;
    trimResult.trim.X_trim = X_trim;
    trimResult.trim.U_trim = U_trim;
    trimResult.trim.U_surface_trim = U_surface_trim;
    trimResult.trim.U_trim_full = U_trim_full;
    trimResult.trim.direct_surface_trim = direct_surface_trim;
    trimResult.trim.mixed_control_trim = mixed_control_trim;
    trimResult.trim.physical_surface_trim = physical_surface_trim;
    trimResult.trim.trimPlantInputs = trim_plant_inputs;

    trimResult.linear = struct();
    trimResult.linear.sys_full = [];
    trimResult.linear.A_full = [];
    trimResult.linear.B_full = [];
    trimResult.linear.C_full = [];
    trimResult.linear.D_full = [];
    trimResult.linear.sys_ss_9state = [];
    trimResult.linear.sys_ss_13state = [];
    trimResult.linear.B_front_collective = [];
    trimResult.linear.B_rear_collective = [];
    trimResult.linear.reduced_model_available = false;

    if compute_linearization
        sys_full = linearize(trim_model_name, op_trim);
        [A_full, B_full, C_full, D_full] = ssdata(sys_full);

        if trim_verbose
            fprintf('Full linearized system: %d states, %d inputs, %d outputs\n', ...
                size(A_full, 1), size(B_full, 2), size(C_full, 1));
            fprintf('\n=== sys_full.StateName ===\n');
            for i = 1:numel(sys_full.StateName)
                fprintf('  [%2d] %s\n', i, sys_full.StateName{i});
            end
            fprintf('\n=== sys_full.InputName ===\n');
            for i = 1:numel(sys_full.InputName)
                fprintf('  [%2d] %s\n', i, sys_full.InputName{i});
            end
        end

        idx_eul_lin = localFindStateNameIndices(sys_full.StateName, 'euler_attitude_state', 3);
        idx_rates_lin = localFindStateNameIndices(sys_full.StateName, 'body_rates_state', 3);
        idx_vel_lin = localFindStateNameIndices(sys_full.StateName, 'body_velocity_state', 3);

        idx_phi   = idx_eul_lin(1);
        idx_theta = idx_eul_lin(2);
        idx_psi   = idx_eul_lin(3);
        idx_P     = idx_rates_lin(1);
        idx_Q     = idx_rates_lin(2);
        idx_R     = idx_rates_lin(3);
        idx_u     = idx_vel_lin(1);
        idx_v     = idx_vel_lin(2);
        idx_w     = idx_vel_lin(3);

        ctrl_names = {'front_coll_rpm', 'rear_coll_rpm', 'delta_f', 'delta_a', 'delta_e', 'delta_r'};
        state_names_9 = {'phi', 'theta', 'psi', 'u', 'v', 'w', 'P', 'Q', 'R'};
        idx9 = [idx_phi, idx_theta, idx_psi, idx_u, idx_v, idx_w, idx_P, idx_Q, idx_R];

        % Input order from the current Trim_Plant is:
        %   1:4 Motor_RPM_cmd, 5:6 Tilt_angles_cmd, 7 Front_RPM_collective,
        %   8 Rear_RPM_collective, 9:12 mixed controls
        %   [delta_f delta_a delta_e delta_r].
        B_front_collective = B_full(idx9, 7);
        B_rear_collective = B_full(idx9, 8);
        B_ctrl = [B_front_collective, B_rear_collective, B_full(idx9, 9:12)];

        sys_ss_9state = ss(A_full(idx9, idx9), ...
                           B_ctrl, ...
                           eye(numel(idx9)), ...
                           zeros(numel(idx9), numel(ctrl_names)));

        sys_ss_9state.StateName = state_names_9(:);
        sys_ss_9state.InputName = ctrl_names(:);

        if emit_linear_summary
            fprintf('9-state rigid-body system eigenvalues:\n');
            lambda_9 = eig(sys_ss_9state.A);
            disp(lambda_9);
            localPrintEigenmodeSummary(sys_ss_9state.A, state_names_9);
        end

        trimResult.linear.sys_full = sys_full;
        trimResult.linear.A_full = A_full;
        trimResult.linear.B_full = B_full;
        trimResult.linear.C_full = C_full;
        trimResult.linear.D_full = D_full;
        trimResult.linear.sys_ss_9state = sys_ss_9state;
        trimResult.linear.B_front_collective = B_front_collective;
        trimResult.linear.B_rear_collective = B_rear_collective;
        trimResult.linear.reduced_model_available = true;
    end

    trimResult.scheduling = struct();
    trimResult.scheduling.Vinf_mps = Vel_W_Trim(1);
    trimResult.scheduling.alpha_rad = Vel_W_Trim(2);
    trimResult.scheduling.beta_rad = Vel_W_Trim(3);
    trimResult.scheduling.phi_rad = Att_Trim(1);
    trimResult.scheduling.theta_rad = Att_Trim(2);
    trimResult.scheduling.psi_rad = Att_Trim(3);
    trimResult.scheduling.front_tilt_deg = front_tilt_sched_deg;
    trimResult.scheduling.front_collective_rpm = front_collective_sched_rpm;
    trimResult.scheduling.rear_collective_rpm = rear_collective_sched_rpm;
    trimResult.scheduling.delta_f_rad = mixed_control_trim(1);
    trimResult.scheduling.delta_a_rad = mixed_control_trim(2);
    trimResult.scheduling.delta_e_rad = mixed_control_trim(3);
    trimResult.scheduling.delta_r_rad = mixed_control_trim(4);

    trimResult.opspec = opspec;
    trimResult.op_trim = op_trim;
    trimResult.op_report = op_report;
end

function trimSpec = localBuildTrimSpec(initData, trimCase)
    trimSpec = struct();
    trimSpec.name = localGetField(trimCase, 'name', 'TrimCase');
    trimSpec.mode = localGetField(trimCase, 'mode', 'trim');
    trimSpec.modelName = localGetField(trimCase, 'modelName', initData.modelNames.trim);

    trimSpec.Vinf_mps = localGetField(trimCase, 'Vinf_mps', 0.0);
    trimSpec.u_body_mps = localGetField(trimCase, 'u_body_mps', trimSpec.Vinf_mps);
    trimSpec.v_body_mps = localGetField(trimCase, 'v_body_mps', 0.0);
    trimSpec.w_body_mps = localGetField(trimCase, 'w_body_mps', 0.0);

    trimSpec.phi_deg = localGetField(trimCase, 'phi_deg', localGetField(trimCase, 'bank_deg', 0.0));
    trimSpec.theta_guess_deg = localGetField(trimCase, 'theta_guess_deg', 0.0);
    trimSpec.psi_deg = localGetField(trimCase, 'psi_deg', 0.0);
    trimSpec.pos_init = localGetField(trimCase, 'pos_init', initData.initialState.pos_init);

    trimSpec.front_tilt_deg = localGetField(trimCase, 'front_tilt_deg', 90.0);
    trimSpec.front_tilt_cmd_deg = localGetField(trimCase, 'front_tilt_cmd_deg', trimSpec.front_tilt_deg);

    if isfield(initData, 'aircraft') && isfield(initData.aircraft, 'MixMatrix')
        trimSpec.mixMatrix_faer = initData.aircraft.MixMatrix;
    else
        trimSpec.mixMatrix_faer = [1  1  0  0;
                                   1 -1  0  0;
                                   0  0  1 -1;
                                   0  0  1  1];
    end

    trimSpec.use_vinf_output_constraint = localGetField(trimCase, 'use_vinf_output_constraint', false);
    trimSpec.use_alpha_output_constraint = localGetField(trimCase, 'use_alpha_output_constraint', false);
    trimSpec.use_vertical_speed_output_constraint = localGetField(trimCase, ...
        'use_vertical_speed_output_constraint', false);
    trimSpec.alpha_target_rad = localResolveAlphaTarget(trimCase);

    trimSpec.motor_rpm_cmd = localGetField(trimCase, 'motor_rpm_cmd', zeros(4, 1));
    trimSpec.front_collective_guess_rpm = localGetField(trimCase, 'front_collective_guess_rpm', 1200.0);
    trimSpec.rear_collective_guess_rpm = localGetField(trimCase, ...
        'rear_collective_guess_rpm', localGetField(trimCase, 'rear_collective_trim_rpm', 0.0));

    trimSpec.front_collective_min_rpm = localGetField(trimCase, 'front_collective_min_rpm', 0.0);
    trimSpec.rear_collective_min_rpm = localGetField(trimCase, 'rear_collective_min_rpm', 0.0);
    trimSpec.front_collective_max_rpm = localGetField(trimCase, 'front_collective_max_rpm', 7000.0);
    trimSpec.rear_collective_max_rpm = localGetField(trimCase, 'rear_collective_max_rpm', 7000.0);
    trimSpec.front_collective_known = localGetField(trimCase, 'front_collective_known', false);
    trimSpec.rear_collective_known = localGetField(trimCase, 'rear_collective_known', false);
    trimSpec.front_collective_fixed_rpm = localGetField(trimCase, ...
        'front_collective_fixed_rpm', trimSpec.front_collective_guess_rpm);
    trimSpec.rear_collective_fixed_rpm = localGetField(trimCase, ...
        'rear_collective_fixed_rpm', trimSpec.rear_collective_guess_rpm);

    mixed_guess_deg = localGetField(trimCase, 'mixed_control_guess_deg', [0; 0; 0; 0]);
    mixed_guess_deg = localCanonicalize4(mixed_guess_deg);
    mixed_guess_deg(1) = localGetField(trimCase, 'delta_f_guess_deg', mixed_guess_deg(1));
    mixed_guess_deg(2) = localGetField(trimCase, 'delta_a_guess_deg', mixed_guess_deg(2));
    mixed_guess_deg(3) = localGetField(trimCase, 'delta_e_guess_deg', mixed_guess_deg(3));
    mixed_guess_deg(4) = localGetField(trimCase, 'delta_r_guess_deg', mixed_guess_deg(4));
    trimSpec.mixed_control_guess_rad = deg2rad(mixed_guess_deg);

    mixed_known_default = [false; false; false; false];
    trimSpec.mixed_control_known = localCanonicalizeLength( ...
        localGetField(trimCase, 'mixed_control_known', mixed_known_default), 4);

    mixed_fixed_deg = localGetField(trimCase, 'mixed_control_fixed_deg', mixed_guess_deg);
    mixed_fixed_deg = localCanonicalize4(mixed_fixed_deg);
    mixed_fixed_deg(1) = localGetField(trimCase, 'delta_f_fixed_deg', mixed_fixed_deg(1));
    mixed_fixed_deg(2) = localGetField(trimCase, 'delta_a_fixed_deg', mixed_fixed_deg(2));
    mixed_fixed_deg(3) = localGetField(trimCase, 'delta_e_fixed_deg', mixed_fixed_deg(3));
    mixed_fixed_deg(4) = localGetField(trimCase, 'delta_r_fixed_deg', mixed_fixed_deg(4));
    trimSpec.mixed_control_fixed_rad = deg2rad(mixed_fixed_deg);

    trimSpec.constraints = struct();
    trimSpec.constraints.euler_known = localCanonicalizeLength( ...
        localGetField(trimCase, 'euler_known', [true; false; true]), 3);
    trimSpec.constraints.euler_steady = localCanonicalizeLength( ...
        localGetField(trimCase, 'euler_steady', [true; true; true]), 3);

    trimSpec.constraints.body_rates_value = localCanonicalizeLength( ...
        localGetField(trimCase, 'body_rates_rad_s', [0.0; 0.0; 0.0]), 3);
    trimSpec.constraints.body_rates_known = localCanonicalizeLength( ...
        localGetField(trimCase, 'body_rates_known', [true; true; true]), 3);
    trimSpec.constraints.body_rates_steady = localCanonicalizeLength( ...
        localGetField(trimCase, 'body_rates_steady', [true; true; true]), 3);

    trimSpec.constraints.body_velocity_known = localCanonicalizeLength( ...
        localGetField(trimCase, 'body_velocity_known', [true; true; true]), 3);
    trimSpec.constraints.body_velocity_steady = localCanonicalizeLength( ...
        localGetField(trimCase, 'body_velocity_steady', [true; true; true]), 3);

    trimSpec.constraints.position_known = localCanonicalizeLength( ...
        localGetField(trimCase, 'position_known', [false; false; false]), 3);
    trimSpec.constraints.position_steady = localCanonicalizeLength( ...
        localGetField(trimCase, 'position_steady', [false; false; true]), 3);

    surface_limit_deg = localGetField(trimCase, 'surface_limit_deg', [25; 25; 25; 25]);
    surface_limit_deg = localCanonicalize4(surface_limit_deg);
    trimSpec.surface_limit_deg = surface_limit_deg;
    trimSpec.surface_limit_deg_scalar = max(surface_limit_deg);
    trimSpec.surface_limit_rad = deg2rad(surface_limit_deg);
    trimSpec.surface_limit_rad_scalar = max(trimSpec.surface_limit_rad);

    trimSpec.validate_nonlinear_hold = localGetField(trimCase, 'validate_nonlinear_hold', false);
    trimSpec.trim_validation_stop_time_s = localGetField(trimCase, 'trim_validation_stop_time_s', 5.0);

    trimStruct = struct();
    trimStruct.usePerfectAct = localGetField(trimCase, 'usePerfectAct', false);
    surface_init_deg = localGetField(trimCase, 'surface_init_deg', [0; 0; 0; 0]);
    surface_init_deg = localCanonicalize4(surface_init_deg);
    trimStruct.dLW = deg2rad(surface_init_deg(1));
    trimStruct.dRW = deg2rad(surface_init_deg(2));
    trimStruct.dLT = deg2rad(surface_init_deg(3));
    trimStruct.dRT = deg2rad(surface_init_deg(4));

    surface_tau_s = localGetField(trimCase, 'surface_tau_s', [1e-3; 1e-3; 1e-3; 1e-3]);
    surface_tau_s = localCanonicalize4(surface_tau_s);
    trimStruct.tauLW = surface_tau_s(1);
    trimStruct.tauRW = surface_tau_s(2);
    trimStruct.tauLT = surface_tau_s(3);
    trimStruct.tauRT = surface_tau_s(4);
    trimStruct.tauMin = localGetField(trimCase, 'surface_tau_min_s', 1e-3);

    rate_limit_deg_s = localGetField(trimCase, 'surface_rate_limit_deg_s', [250; 250; 250; 250]);
    rate_limit_deg_s = localCanonicalize4(rate_limit_deg_s);
    rate_limit_rad_s = deg2rad(rate_limit_deg_s);

    trimStruct.dLW_max = trimSpec.surface_limit_rad(1);
    trimStruct.dRW_max = trimSpec.surface_limit_rad(2);
    trimStruct.dLT_max = trimSpec.surface_limit_rad(3);
    trimStruct.dRT_max = trimSpec.surface_limit_rad(4);

    trimStruct.dLWdot_max = rate_limit_rad_s(1);
    trimStruct.dRWdot_max = rate_limit_rad_s(2);
    trimStruct.dLTdot_max = rate_limit_rad_s(3);
    trimStruct.dRTdot_max = rate_limit_rad_s(4);

    trimSpec.trim = trimStruct;
end

function localConfigureTrimModelInterface(model_name)
    load_system(model_name);
    set_param([model_name '/Motor_RPM_cmd'], 'PortDimensions', '4');
    set_param([model_name '/Tilt_angles_cmd'], 'PortDimensions', '2');
end

function opspec = localBuildCruiseOpspec(trimSpec)
    opspec = operspec(trimSpec.modelName);
    localValidateTrimOpspecLayout(opspec);

    n_motor = numel(opspec.Inputs(1).u);
    n_tilt = numel(opspec.Inputs(2).u);
    motor_rpm_cmd = localCanonicalizeLength(trimSpec.motor_rpm_cmd, n_motor);
    tilt_cmd_deg = localCanonicalizeLength(trimSpec.front_tilt_cmd_deg, n_tilt);
    pos_init = trimSpec.pos_init(:);

    idx_eul   = localFindOpspecStateIdx(opspec, 'euler_attitude_state');
    idx_rates = localFindOpspecStateIdx(opspec, 'body_rates_state');
    idx_vel   = localFindOpspecStateIdx(opspec, 'body_velocity_state');
    idx_pos   = localFindOpspecStateIdx(opspec, 'ned_position_state');

    opspec.States(idx_eul).x = [deg2rad(trimSpec.phi_deg); ...
                                deg2rad(trimSpec.theta_guess_deg); ...
                                deg2rad(trimSpec.psi_deg)];
    opspec.States(idx_eul).Known = trimSpec.constraints.euler_known;
    opspec.States(idx_eul).SteadyState = trimSpec.constraints.euler_steady;

    opspec.States(idx_rates).x = trimSpec.constraints.body_rates_value;
    opspec.States(idx_rates).Known = trimSpec.constraints.body_rates_known;
    opspec.States(idx_rates).SteadyState = trimSpec.constraints.body_rates_steady;

    opspec.States(idx_vel).x = [trimSpec.u_body_mps; ...
                                trimSpec.v_body_mps; ...
                                trimSpec.w_body_mps];
    opspec.States(idx_vel).Known = trimSpec.constraints.body_velocity_known;
    opspec.States(idx_vel).SteadyState = trimSpec.constraints.body_velocity_steady;

    opspec.States(idx_pos).x = pos_init;
    opspec.States(idx_pos).Known = trimSpec.constraints.position_known;
    opspec.States(idx_pos).SteadyState = trimSpec.constraints.position_steady;

    if trimSpec.use_vertical_speed_output_constraint
        opspec.Outputs(3).y = [0.0; 0.0; 0.0];
        opspec.Outputs(3).Known = [false; false; true];
    end

    if trimSpec.use_vinf_output_constraint
        opspec.Outputs(8).y = trimSpec.Vinf_mps;
        opspec.Outputs(8).Known = true;
    end

    if trimSpec.use_alpha_output_constraint
        opspec.Outputs(9).y = trimSpec.alpha_target_rad;
        opspec.Outputs(9).Known = true;
    end

    opspec.Inputs(1).u = motor_rpm_cmd;
    opspec.Inputs(1).Known = true(n_motor, 1);
    opspec.Inputs(1).Min = zeros(n_motor, 1);
    opspec.Inputs(1).Max = inf(n_motor, 1);

    opspec.Inputs(2).u = tilt_cmd_deg;
    opspec.Inputs(2).Known = true(n_tilt, 1);

    opspec.Inputs(3).u = trimSpec.front_collective_guess_rpm;
    opspec.Inputs(3).Known = trimSpec.front_collective_known;
    opspec.Inputs(3).Min = trimSpec.front_collective_min_rpm;
    opspec.Inputs(3).Max = trimSpec.front_collective_max_rpm;
    if trimSpec.front_collective_known
        opspec.Inputs(3).u = trimSpec.front_collective_fixed_rpm;
    end

    opspec.Inputs(4).u = trimSpec.rear_collective_guess_rpm;
    opspec.Inputs(4).Known = trimSpec.rear_collective_known;
    opspec.Inputs(4).Min = trimSpec.rear_collective_min_rpm;
    opspec.Inputs(4).Max = trimSpec.rear_collective_max_rpm;
    if trimSpec.rear_collective_known
        opspec.Inputs(4).u = trimSpec.rear_collective_fixed_rpm;
    end

    for i_in = 5:8
        limit_idx = i_in - 4;
        opspec.Inputs(i_in).u = 0.0;
        opspec.Inputs(i_in).Known = true;
        opspec.Inputs(i_in).Min = -trimSpec.surface_limit_rad(limit_idx);
        opspec.Inputs(i_in).Max = trimSpec.surface_limit_rad(limit_idx);
    end

    for i_in = 9:12
        limit_idx = i_in - 8;
        opspec.Inputs(i_in).u = trimSpec.mixed_control_guess_rad(limit_idx);
        opspec.Inputs(i_in).Known = trimSpec.mixed_control_known(limit_idx);
        if trimSpec.mixed_control_known(limit_idx)
            opspec.Inputs(i_in).u = trimSpec.mixed_control_fixed_rad(limit_idx);
        end
        opspec.Inputs(i_in).Min = -trimSpec.surface_limit_rad(limit_idx);
        opspec.Inputs(i_in).Max = trimSpec.surface_limit_rad(limit_idx);
    end
end

function localDumpOpspec(opspec)
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
            fprintf('       Known       = [%s]\n', num2str(s.Known(:).', ' %d'));
            fprintf('       SteadyState = [%s]\n', num2str(s.SteadyState(:).', ' %d'));
        catch
        end
    end
    fprintf('=== end opspec.States ===\n\n');

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

function value = localGetField(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    else
        value = default_value;
    end
end

function alpha_target_rad = localResolveAlphaTarget(trimCase)
    if isfield(trimCase, 'alpha_target_rad') && ~isempty(trimCase.alpha_target_rad)
        alpha_target_rad = trimCase.alpha_target_rad;
    elseif isfield(trimCase, 'alpha_target_deg') && ~isempty(trimCase.alpha_target_deg)
        alpha_target_rad = deg2rad(trimCase.alpha_target_deg);
    else
        alpha_target_rad = atan2(localGetField(trimCase, 'w_body_mps', 0.0), ...
            localGetField(trimCase, 'u_body_mps', localGetField(trimCase, 'Vinf_mps', 0.0)));
    end
end

function values = localCanonicalize4(values_in)
    values = values_in;
    if isscalar(values)
        values = repmat(values, 4, 1);
    else
        values = values(:);
    end
    if numel(values) ~= 4
        error('Expected a scalar or 4x1 value.');
    end
end

function values = localCanonicalizeLength(values_in, expected_count)
    values = values_in;
    if isscalar(values)
        values = repmat(values, expected_count, 1);
    else
        values = values(:);
    end
    if numel(values) ~= expected_count
        error('Expected a scalar or %dx1 value.', expected_count);
    end
end

function idx = localFindOpspecStateIdx(opspec, block_fragment)
    idx = [];
    for i = 1:numel(opspec.States)
        try
            blk = char(opspec.States(i).Block);
        catch
            blk = '';
        end
        if contains(blk, block_fragment)
            idx = i;
            return;
        end
    end
    error('Unable to find opspec state containing block fragment "%s".', block_fragment);
end

function idx = localFindOpStateIdx(op_trim, block_fragment)
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

function localValidateTrimOpspecLayout(opspec)
    if numel(opspec.States) ~= 4
        error('Expected Trim_Plant to expose 4 opspec states, found %d.', numel(opspec.States));
    end
    if numel(opspec.Inputs) ~= 12
        error('Expected Trim_Plant to expose 12 opspec inputs, found %d.', numel(opspec.Inputs));
    end

    required_blocks = { ...
        'euler_attitude_state', ...
        'body_rates_state', ...
        'body_velocity_state', ...
        'ned_position_state'};

    for i = 1:numel(required_blocks)
        localFindOpspecStateIdx(opspec, required_blocks{i});
    end
end

function localValidateTrimOpLayout(op_trim)
    if numel(op_trim.States) ~= 4
        error('Expected op_trim to expose 4 grouped states, found %d.', numel(op_trim.States));
    end
    if numel(op_trim.Inputs) ~= 12
        error('Expected op_trim to expose 12 grouped inputs, found %d.', numel(op_trim.Inputs));
    end
end

function idx = localFindStateNameIndices(state_names, fragment, expected_count)
    idx = find(contains(state_names, fragment));
    if numel(idx) ~= expected_count
        error('Expected %d linearized state(s) containing "%s", found %d.', ...
            expected_count, fragment, numel(idx));
    end
    idx = idx(:).';
end

function localPrintEigenmodeSummary(A, state_names)
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
        fprintf('  Unstable mode count: 0\n');
    else
        fprintf('  Unstable mode count: %d (indices %s)\n', ...
            numel(unstable_idx), mat2str(unstable_idx(:).'));
    end
end

function localPrintTrimEnvironmentFingerprint(model_name)
    fprintf('\n--- Trim Diagnostics: Environment ---\n');
    fprintf('Trim function path: %s\n', which('trim_evtol_case'));
    fprintf('aircraft_def path: %s\n', which('aircraft_def'));
    fprintf('Trim model path: %s\n', which(model_name));
    if bdIsLoaded(model_name)
        fprintf('Model dirty flag: %s\n', get_param(model_name, 'Dirty'));
    end
    if evalin('base', 'exist(''prop'', ''var'')')
        prop_local = evalin('base', 'prop');
        if isfield(prop_local, 'k_Thrust')
            fprintf('prop.k_Thrust = %.6g\n', prop_local.k_Thrust);
        end
    end
    if evalin('base', 'exist(''controls'', ''var'')')
        controls_local = evalin('base', 'controls');
        if isfield(controls_local, 'ruddervator') && isfield(controls_local.ruddervator, 'effectiveness_tau')
            fprintf('controls.ruddervator.effectiveness_tau = %.6g\n', ...
                controls_local.ruddervator.effectiveness_tau);
        end
        if isfield(controls_local, 'flaperon') && isfield(controls_local.flaperon, 'effectiveness_tau')
            fprintf('controls.flaperon.effectiveness_tau = %.6g\n', ...
                controls_local.flaperon.effectiveness_tau);
        end
    end
    fprintf('--- End Trim Diagnostics ---\n\n');
end

function localPrintTrimReportSummary(op_report)
    fprintf('\n--- Trim Diagnostics: Residual Summary ---\n');
    rows = struct('label', {}, 'x', {}, 'dx', {}, 'known', {}, 'steady', {});
    for i = 1:numel(op_report.States)
        s = op_report.States(i);
        label = strtrim(char(s.Block));
        x = s.x(:);
        dx = s.dx(:);
        known = s.Known(:);
        steady = s.SteadyState(:);
        for j = 1:numel(dx)
            if j == 1
                row_label = label;
            else
                row_label = sprintf('%s(%d)', label, j);
            end
            rows(end + 1) = struct( ... %#ok<AGROW>
                'label', row_label, ...
                'x', x(j), ...
                'dx', dx(j), ...
                'known', logical(known(j)), ...
                'steady', logical(steady(j)));
        end
    end

    if isempty(rows)
        fprintf('No state residuals available in op_report.\n');
    else
        residuals = abs([rows.dx]);
        [~, order] = sort(residuals, 'descend');
        top_count = min(10, numel(order));
        fprintf('Largest state-derivative residuals:\n');
        for n = 1:top_count
            r = rows(order(n));
            fprintf(['  %2d. |dx|=% .6g  dx=% .6g  x=% .6g  known=%d  steady=%d  %s\n'], ...
                n, abs(r.dx), r.dx, r.x, r.known, r.steady, r.label);
        end
    end

    fprintf('Input summary:\n');
    for i = 1:numel(op_report.Inputs)
        inp = op_report.Inputs(i);
        label = strtrim(char(inp.Block));
        u = inp.u(:).';
        known = inp.Known(:).';
        fprintf('  input %d: %s\n', i, label);
        fprintf('           u     = [%s]\n', num2str(u, ' %.6g'));
        fprintf('           known = [%s]\n', num2str(known, ' %d'));
    end
    fprintf('--- End Trim Diagnostics ---\n\n');
end

function localRestoreWarningStates(varargin)
for i = 1:nargin
    state = varargin{i};
    if isstruct(state) && isfield(state, 'identifier') && isfield(state, 'state')
        warning(state.state, state.identifier);
    end
end
end
