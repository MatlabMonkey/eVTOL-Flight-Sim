% TrimAndLinearize_EVTOL_Cruise_reworked.m
% Solve the cruise operating point, linearize the trim plant there, and
% package the results for downstream controller and run scripts.

if ~exist('trimSpec', 'var')
    SetupForTrimAndLin_EVTOL_Cruise;
end

load_system(trimSpec.modelName);

%% Linearization bookkeeping
% The operating-point specification and the linearized state ordering are
% not exactly the same in the copied trim plant. The operspec sees actuator
% states first, but the linearized A-matrix can drop them when
% trim.usePerfectAct is enabled. We therefore decide the reduced-order
% indices after calling linearize().
ctrl_idx = [19 21 22 23 24];
ctrl_names = {'front_collective_rpm', 'deltaLW', 'deltaRW', 'deltaLT', 'deltaRT'};

%% Define trim problem
opspec = operspec(trimSpec.modelName);

% Actuator states [dLT, dLW, dRT, dRW]
opspec.States(1).x = trim.dLT;
opspec.States(1).Known = false;
opspec.States(1).SteadyState = true;

opspec.States(2).x = trim.dLW;
opspec.States(2).Known = false;
opspec.States(2).SteadyState = true;

opspec.States(3).x = trim.dRT;
opspec.States(3).Known = false;
opspec.States(3).SteadyState = true;

opspec.States(4).x = trim.dRW;
opspec.States(4).Known = false;
opspec.States(4).SteadyState = true;

% Euler attitude [phi; theta; psi]
opspec.States(5).x = [0.0; 0.0; trimSpec.psi_trim_target];
opspec.States(5).Known = [true; false; true];
opspec.States(5).SteadyState = [true; true; true];

% Body rates [P; Q; R]
opspec.States(6).x = [0.0; 0.0; 0.0];
opspec.States(6).Known = [true; true; true];
opspec.States(6).SteadyState = [true; true; true];

% Body velocity [u; v; w]
opspec.States(7).x = [trimSpec.Vinf_trim_target; 0.0; 0.0];
opspec.States(7).Known = [false; true; false];
opspec.States(7).SteadyState = [true; true; true];

% Position should drift at steady cruise velocity.
opspec.States(8).Known = [false; false; false];
opspec.States(8).SteadyState = [false; false; false];

% Inputs:
%   1: Motor_RPM_cmd(12)      fixed at zero
%   2: Tilt_angles_cmd(6)     fixed at front_tilt_trim_deg
%   3: Front_RPM_collective   free
%   4: Rear_RPM_collective1   fixed at rear_collective_trim_rpm
%   5: deltaLW                free
%   6: deltaRW                free
%   7: deltaLT                free
%   8: deltaRT                free
opspec.Inputs(1).u = zeros(12, 1);
opspec.Inputs(1).Known = true(12, 1);

opspec.Inputs(2).u = trimSpec.front_tilt_trim_deg * ones(6, 1);
opspec.Inputs(2).Known = true(6, 1);
opspec.Inputs(2).Min = trimSpec.front_tilt_trim_deg * ones(6, 1);
opspec.Inputs(2).Max = trimSpec.front_tilt_trim_deg * ones(6, 1);

opspec.Inputs(3).u = trimSpec.front_collective_guess_rpm;
opspec.Inputs(3).Known = false;
opspec.Inputs(3).Min = 0.0;
opspec.Inputs(3).Max = 3500.0;

opspec.Inputs(4).u = trimSpec.rear_collective_trim_rpm;
opspec.Inputs(4).Known = true;
opspec.Inputs(4).Min = trimSpec.rear_collective_trim_rpm;
opspec.Inputs(4).Max = trimSpec.rear_collective_trim_rpm;

for i_input = 5:8
    opspec.Inputs(i_input).u = 0.0;
    opspec.Inputs(i_input).Known = false;
    opspec.Inputs(i_input).Min = -trimSpec.surface_limit_rad(i_input - 4);
    opspec.Inputs(i_input).Max = trimSpec.surface_limit_rad(i_input - 4);
end

% Output constraints:
%   V_E_truth(3) = 0 for level flight
%   vinf_truth   = Vinf_trim_target
opspec.Outputs(3).y = [0.0; 0.0; 0.0];
opspec.Outputs(3).Known = [false; false; true];
opspec.Outputs(8).y = trimSpec.Vinf_trim_target;
opspec.Outputs(8).Known = true;

%% Solve trim
trim_opts = findopOptions('DisplayReport', 'off');
[op_trim, op_report] = findop(trimSpec.modelName, opspec, trim_opts);

%% Extract trim values
Att_Trim = op_trim.States(5).x(:);
Rates_Trim = op_trim.States(6).x(:);
Vel_B_BA_Trim = op_trim.States(7).x(:);
Pos_Trim = op_trim.States(8).x(:);

Vel_W_Trim = [op_report.Outputs(8).y;
              op_report.Outputs(9).y;
              op_report.Outputs(10).y];

Trim_Output_Vel_E = op_report.Outputs(3).y(:);
Trim_Specific_Force = op_report.Outputs(11).y(:);
Trim_Termination = op_report.TerminationString;

X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim];
U_trim = [op_trim.Inputs(3).u;
          op_trim.Inputs(5).u;
          op_trim.Inputs(6).u;
          op_trim.Inputs(7).u;
          op_trim.Inputs(8).u];

U_trim_full = zeros(24, 1);
U_trim_full(19) = op_trim.Inputs(3).u;
U_trim_full(20) = op_trim.Inputs(4).u;
U_trim_full(21) = op_trim.Inputs(5).u;
U_trim_full(22) = op_trim.Inputs(6).u;
U_trim_full(23) = op_trim.Inputs(7).u;
U_trim_full(24) = op_trim.Inputs(8).u;

fprintf('\n--- Cruise Trim Results ---\n');
fprintf('Termination: %s\n', Trim_Termination);
fprintf('Att_Trim (deg): phi=%.3f  theta=%.3f  psi=%.3f\n', ...
    rad2deg(Att_Trim(1)), rad2deg(Att_Trim(2)), rad2deg(Att_Trim(3)));
fprintf('Vel_B_BA_Trim (m/s): u=%.4f  v=%.4f  w=%.4f\n', ...
    Vel_B_BA_Trim(1), Vel_B_BA_Trim(2), Vel_B_BA_Trim(3));
fprintf('Rates_Trim (rad/s): P=%.4f  Q=%.4f  R=%.4f\n', ...
    Rates_Trim(1), Rates_Trim(2), Rates_Trim(3));
fprintf('U_trim: front_collective=%.3f rpm  deltaLW=%.4f  deltaRW=%.4f  deltaLT=%.4f  deltaRT=%.4f\n', ...
    U_trim(1), U_trim(2), U_trim(3), U_trim(4), U_trim(5));
fprintf('Vel_W_Trim: Vinf=%.4f m/s  alpha=%.4f rad  beta=%.4f rad\n', ...
    Vel_W_Trim(1), Vel_W_Trim(2), Vel_W_Trim(3));
fprintf('Trim outputs: V_E,z=%.4f m/s  |specific force|=%.4f m/s^2\n\n', ...
    Trim_Output_Vel_E(3), norm(Trim_Specific_Force));

%% Linearize about trim
sys_struct = linearize(trimSpec.modelName, op_trim);
sys_ss = ss(sys_struct.a, sys_struct.b, sys_struct.c, sys_struct.d);

if size(sys_struct.a, 1) == 12
    % Linearized state order is [pos, vel, eul, rates].
    rb_idx = [7:9 4:6 10:12];
    state_names_rb = {'phi', 'theta', 'psi', 'u', 'v', 'w', 'P', 'Q', 'R'};
elseif size(sys_struct.a, 1) == 16
    % Linearized state order is [pos, vel, eul, rates, dLW, dRW, dLT, dRT].
    rb_idx = [7:9 4:6 10:12 13:16];
    state_names_rb = {'phi', 'theta', 'psi', 'u', 'v', 'w', 'P', 'Q', 'R', ...
        'dLW', 'dRW', 'dLT', 'dRT'};
else
    error('TrimAndLinearize:UnexpectedStateCount', ...
        'Expected 12 or 16 linearized states, got %d.', size(sys_struct.a, 1));
end

sys_ss_reduced = ss(sys_struct.a(rb_idx, rb_idx), ...
                    sys_struct.b(rb_idx, ctrl_idx), ...
                    eye(numel(rb_idx)), ...
                    zeros(numel(rb_idx), numel(ctrl_idx)));
sys_ss_reduced.StateName = state_names_rb(:);
sys_ss_reduced.InputName = ctrl_names(:);

% Keep both names alive. For perfect-actuator trim mode, the 13-state alias
% points at the same reduced system so downstream scripts do not break.
sys_ss_9state = sys_ss_reduced;
sys_ss_13state = sys_ss_reduced;

fprintf('Full linearized system size: %dx%d\n', size(sys_struct.a, 1), size(sys_struct.a, 2));
fprintf('Reduced-order system size: %dx%d\n', size(sys_ss_reduced.a, 1), size(sys_ss_reduced.a, 2));
disp(eig(sys_ss_reduced.a));

%% Nonlinear validation hold
Trim_Validation = struct( ...
    'executed', false, ...
    'pass', false, ...
    'message', 'Validation not run.');

if trimSpec.validateNonlinearHold
    t_val = [0.0; trimSpec.trim_validation_stop_time];

    trim_input_ds = Simulink.SimulationData.Dataset;
    trim_input_ds = addElement(trim_input_ds, timeseries(zeros(2, 12), t_val), 'Motor_RPM_cmd');
    trim_input_ds = addElement(trim_input_ds, timeseries(trimSpec.front_tilt_trim_deg * ones(2, 6), t_val), 'Tilt_angles_cmd');
    trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(19) * ones(2, 1), t_val), 'Front_RPM_collective');
    trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(20) * ones(2, 1), t_val), 'Rear_RPM_collective1');
    trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(21) * ones(2, 1), t_val), 'deltaLW');
    trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(22) * ones(2, 1), t_val), 'deltaRW');
    trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(23) * ones(2, 1), t_val), 'deltaLT');
    trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(24) * ones(2, 1), t_val), 'deltaRT');

    simIn = Simulink.SimulationInput(trimSpec.modelName);
    simIn = simIn.setExternalInput(trim_input_ds);
    simIn = simIn.setModelParameter('StopTime', num2str(trimSpec.trim_validation_stop_time), ...
                                    'SaveOutput', 'on', ...
                                    'OutputSaveName', 'yout');
    simIn = simIn.setVariable('pos_init', Pos_Trim);
    simIn = simIn.setVariable('V_init', Vel_B_BA_Trim);
    simIn = simIn.setVariable('eul_init', Att_Trim);
    simIn = simIn.setVariable('omega_init', Rates_Trim);

    try
        simOut_val = sim(simIn);
        yout = simOut_val.get('yout');

        vinf_val = squeeze(yout{8}.Values.Data);
        eul_val = squeeze(yout{4}.Values.Data);
        omega_val = squeeze(yout{5}.Values.Data);
        vE_val = squeeze(yout{3}.Values.Data);

        Trim_Validation = struct();
        Trim_Validation.executed = true;
        Trim_Validation.stopTime_s = trimSpec.trim_validation_stop_time;
        Trim_Validation.vinf_initial_mps = vinf_val(1);
        Trim_Validation.vinf_final_mps = vinf_val(end);
        Trim_Validation.vinf_error_mps = vinf_val(end) - trimSpec.Vinf_trim_target;
        Trim_Validation.theta_initial_deg = rad2deg(eul_val(1, 2));
        Trim_Validation.theta_final_deg = rad2deg(eul_val(end, 2));
        Trim_Validation.rate_norm_final_radps = norm(omega_val(end, :));
        Trim_Validation.vertical_speed_final_mps = vE_val(end, 3);
        Trim_Validation.front_collective_trim_rpm = U_trim(1);
        Trim_Validation.rear_collective_trim_rpm = trimSpec.rear_collective_trim_rpm;
        Trim_Validation.surface_symmetry_error_deg = max(abs(rad2deg([U_trim(2) - U_trim(3); U_trim(4) - U_trim(5)])));
        Trim_Validation.pass = ...
            abs(Trim_Validation.vinf_error_mps) <= 1.0 && ...
            abs(Trim_Validation.vertical_speed_final_mps) <= 0.5 && ...
            Trim_Validation.rate_norm_final_radps <= 0.05 && ...
            Trim_Validation.surface_symmetry_error_deg <= 1.0;
        Trim_Validation.message = 'Nonlinear trim-hold validation completed.';

        fprintf('Validation: Vinf final = %.4f m/s, theta final = %.4f deg, |rates| final = %.6f rad/s, V_E,z final = %.4f m/s\n', ...
            Trim_Validation.vinf_final_mps, Trim_Validation.theta_final_deg, ...
            Trim_Validation.rate_norm_final_radps, Trim_Validation.vertical_speed_final_mps);
        fprintf('Validation pass: %d\n\n', Trim_Validation.pass);
    catch ME
        Trim_Validation.executed = true;
        Trim_Validation.pass = false;
        Trim_Validation.message = sprintf('Validation run failed: %s', ME.message);
        warning('Trim validation failed: %s', ME.message);
    end
end

%% Structured trim result for downstream scripts
trimResult = struct();
trimResult.name = trimSpec.name;
trimResult.modelName = trimSpec.modelName;
trimResult.mode = trimSpec.mode;
trimResult.front_tilt_deg = trimSpec.front_tilt_trim_deg;
trimResult.op_trim = op_trim;
trimResult.op_report = op_report;
trimResult.X_trim = X_trim;
trimResult.U_trim = U_trim;
trimResult.U_trim_full = U_trim_full;
trimResult.Pos_Trim = Pos_Trim;
trimResult.Vel_B_BA_Trim = Vel_B_BA_Trim;
trimResult.Att_Trim = Att_Trim;
trimResult.Rates_Trim = Rates_Trim;
trimResult.Vel_W_Trim = Vel_W_Trim;
trimResult.Trim_Output_Vel_E = Trim_Output_Vel_E;
trimResult.Trim_Specific_Force = Trim_Specific_Force;
trimResult.Trim_Termination = Trim_Termination;
trimResult.sys_struct = sys_struct;
trimResult.sys_ss = sys_ss;
trimResult.sys_ss_reduced = sys_ss_reduced;
trimResult.sys_ss_13state = sys_ss_13state;
trimResult.sys_ss_9state = sys_ss_9state;
trimResult.ctrl_names = ctrl_names;
trimResult.state_names_rb = state_names_rb;
trimResult.rb_idx = rb_idx;
trimResult.ctrl_idx = ctrl_idx;
trimResult.validation = Trim_Validation;

fprintf('Stored trimResult with trim, linearization, and validation data.\n');
