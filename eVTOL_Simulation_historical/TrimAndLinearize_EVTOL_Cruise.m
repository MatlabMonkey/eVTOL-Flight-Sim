% TrimAndLinearize_EVTOL_Cruise.m
% Trims the eVTOL to steady level flight and linearizes about that point.
% Produces: X_trim, U_trim, sys_struct, Att_Trim, Vel_B_BA_Trim,
%           Rates_Trim, Vel_W_Trim, sys_ss_9state, Trim_Validation.

SetupForTrimAndLin_EVTOL_Cruise;

load_system('EVTOL_6DOF_Plant_EUL');

%% Get state names from the linearization model
[sizes, x0, names] = EVTOL_6DOF_Plant_EUL([], [], [], 'sizes');

state_names = cell(1, numel(names));
for i = 1:numel(names)
    n = max(strfind(names{i}, '/'));
    state_names{i} = names{i}(n+1:end);
end

%% Linearization state and input bookkeeping
idx_pos   = 1:3;
idx_vel   = 4:6;
idx_eul   = 7:9;
idx_rates = 10:12;
idx_cntr  = 13:16;

rb_idx = [idx_eul idx_vel idx_rates idx_cntr];
ctrl_idx = [19 21 22 23 24];

ctrl_names = {'front_collective_rpm', 'deltaLW', 'deltaRW', 'deltaLT', 'deltaRT'};
state_names_rb = {'phi', 'theta', 'psi', 'u', 'v', 'w', 'P', 'Q', 'R', 'dLW', 'dRW', 'dLT', 'dRT'};

%% Define trim problem
opspec = operspec('EVTOL_6DOF_Plant_EUL');

% Euler attitude [phi; theta; psi]
opspec.States(1).x = [0.0; 0.0; psi_trim_target];
opspec.States(1).Known = [true; false; true];
opspec.States(1).SteadyState = [true; true; true];

% Position state should drift at constant velocity in steady flight.
opspec.States(2).Known = [false; false; false];
opspec.States(2).SteadyState = [false; false; false];

% Body rates [P; Q; R]
opspec.States(3).x = [0.0; 0.0; 0.0];
opspec.States(3).Known = [true; true; true];
opspec.States(3).SteadyState = [true; true; true];

% Body velocity [u; v; w]
opspec.States(4).x = [Vinf_trim_target; 0.0; 0.0];
opspec.States(4).Known = [false; true; false];
opspec.States(4).SteadyState = [true; true; true];

% Inputs:
%   1: Motor_RPM_cmd(12)      fixed at zero
%   2: Tilt_angles_cmd(6)     fixed at zero degrees
%   3: Front_RPM_collective   free
%   4: Rear_RPM_collective1   fixed at zero
%   5: deltaLW                free
%   6: deltaRW                free
%   7: deltaLT                free
%   8: deltaRT                free
opspec.Inputs(1).u = zeros(12, 1);
opspec.Inputs(1).Known = true(12, 1);

opspec.Inputs(2).u = front_tilt_trim_deg * ones(6, 1);
opspec.Inputs(2).Known = true(6, 1);
opspec.Inputs(2).Min = front_tilt_trim_deg * ones(6, 1);
opspec.Inputs(2).Max = front_tilt_trim_deg * ones(6, 1);

opspec.Inputs(3).u = front_collective_guess_rpm;
opspec.Inputs(3).Known = false;
opspec.Inputs(3).Min = 0.0;
opspec.Inputs(3).Max = 3500.0;

opspec.Inputs(4).u = rear_collective_trim_rpm;
opspec.Inputs(4).Known = true;
opspec.Inputs(4).Min = rear_collective_trim_rpm;
opspec.Inputs(4).Max = rear_collective_trim_rpm;

for i_input = 5:8
    opspec.Inputs(i_input).u = 0.0;
    opspec.Inputs(i_input).Known = false;
    opspec.Inputs(i_input).Min = deg2rad(-25.0);
    opspec.Inputs(i_input).Max = deg2rad(25.0);
end

% Output constraints:
%   V_E_truth(3) = 0 for level flight
%   vinf_truth   = Vinf_trim_target
opspec.Outputs(3).y = [0.0; 0.0; 0.0];
opspec.Outputs(3).Known = [false; false; true];

opspec.Outputs(8).y = Vinf_trim_target;
opspec.Outputs(8).Known = true;

trim_opts = findopOptions('DisplayReport', 'off');
[op_trim, op_report] = findop('EVTOL_6DOF_Plant_EUL', opspec, trim_opts);

%% Extract trim states and controls
Att_Trim      = op_report.States(1).x(:);
Pos_Trim      = op_report.States(2).x(:);
Rates_Trim    = op_report.States(3).x(:);
Vel_B_BA_Trim = op_report.States(4).x(:);

Vel_W_Trim = [op_report.Outputs(8).y;
              op_report.Outputs(9).y;
              op_report.Outputs(10).y];

Trim_Output_Vel_E = op_report.Outputs(3).y(:);
Trim_Specific_Force = op_report.Outputs(11).y(:);
Trim_Termination = op_report.TerminationString;

X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim];
U_trim = [op_report.Inputs(3).u;
          op_report.Inputs(5).u;
          op_report.Inputs(6).u;
          op_report.Inputs(7).u;
          op_report.Inputs(8).u];

U_trim_full = zeros(24, 1);
U_trim_full(19) = op_report.Inputs(3).u;
U_trim_full(20) = op_report.Inputs(4).u;
U_trim_full(21) = op_report.Inputs(5).u;
U_trim_full(22) = op_report.Inputs(6).u;
U_trim_full(23) = op_report.Inputs(7).u;
U_trim_full(24) = op_report.Inputs(8).u;

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
sys_struct = linearize('EVTOL_6DOF_Plant_EUL', op_trim);
sys_ss_9state = ss(sys_struct.a(rb_idx, rb_idx), ...
                   sys_struct.b(rb_idx, ctrl_idx), ...
                   eye(numel(rb_idx)), ...
                   zeros(numel(rb_idx), numel(ctrl_idx)));

sys_ss_9state.StateName = state_names_rb(:);
sys_ss_9state.InputName = ctrl_names(:);

fprintf('9-state rigid-body eigenvalues:\n');
disp(eig(sys_ss_9state.a));

%% Nonlinear validation hold
t_val = [0.0; trim_validation_stop_time];

trim_input_ds = Simulink.SimulationData.Dataset;
trim_input_ds = addElement(trim_input_ds, timeseries(zeros(2, 12), t_val), 'Motor_RPM_cmd');
trim_input_ds = addElement(trim_input_ds, timeseries(front_tilt_trim_deg * ones(2, 6), t_val), 'Tilt_angles_cmd');
trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(19) * ones(2, 1), t_val), 'Front_RPM_collective');
trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(20) * ones(2, 1), t_val), 'Rear_RPM_collective1');
trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(21) * ones(2, 1), t_val), 'deltaLW');
trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(22) * ones(2, 1), t_val), 'deltaRW');
trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(23) * ones(2, 1), t_val), 'deltaLT');
trim_input_ds = addElement(trim_input_ds, timeseries(U_trim_full(24) * ones(2, 1), t_val), 'deltaRT');

simIn = Simulink.SimulationInput('EVTOL_6DOF_Plant_EUL');
simIn = simIn.setExternalInput(trim_input_ds);
simIn = simIn.setModelParameter('StopTime', num2str(trim_validation_stop_time), ...
                                'SaveOutput', 'on', ...
                                'OutputSaveName', 'yout');
simIn = simIn.setVariable('pos_init', Pos_Trim);
simIn = simIn.setVariable('V_init', Vel_B_BA_Trim);
simIn = simIn.setVariable('eul_init', Att_Trim);
simIn = simIn.setVariable('omega_init', Rates_Trim);

simOut = sim(simIn);
yout = simOut.get('yout');

vinf_val = squeeze(yout{8}.Values.Data);
eul_val = squeeze(yout{4}.Values.Data);
omega_val = squeeze(yout{5}.Values.Data);
vE_val = squeeze(yout{3}.Values.Data);

Trim_Validation = struct();
Trim_Validation.stopTime_s = trim_validation_stop_time;
Trim_Validation.vinf_initial_mps = vinf_val(1);
Trim_Validation.vinf_final_mps = vinf_val(end);
Trim_Validation.vinf_error_mps = vinf_val(end) - Vinf_trim_target;
Trim_Validation.theta_initial_deg = rad2deg(eul_val(1, 2));
Trim_Validation.theta_final_deg = rad2deg(eul_val(end, 2));
Trim_Validation.rate_norm_final_radps = norm(omega_val(end, :));
Trim_Validation.vertical_speed_final_mps = vE_val(end, 3);
Trim_Validation.front_collective_trim_rpm = U_trim(1);
Trim_Validation.rear_collective_trim_rpm = rear_collective_trim_rpm;
Trim_Validation.surface_symmetry_error_deg = max(abs(rad2deg([U_trim(2) - U_trim(3); U_trim(4) - U_trim(5)])));
Trim_Validation.pass = ...
    abs(Trim_Validation.vinf_error_mps) <= 1.0 && ...
    abs(Trim_Validation.vertical_speed_final_mps) <= 0.5 && ...
    Trim_Validation.rate_norm_final_radps <= 0.05 && ...
    Trim_Validation.surface_symmetry_error_deg <= 1.0;

fprintf('Validation: Vinf final = %.4f m/s, theta final = %.4f deg, |rates| final = %.6f rad/s, V_E,z final = %.4f m/s\n', ...
    Trim_Validation.vinf_final_mps, Trim_Validation.theta_final_deg, ...
    Trim_Validation.rate_norm_final_radps, Trim_Validation.vertical_speed_final_mps);
fprintf('Validation pass: %d\n\n', Trim_Validation.pass);
