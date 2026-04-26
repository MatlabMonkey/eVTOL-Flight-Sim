% TrimAndLinearize_ATT_wInputs_Aircraft6dof.m
% Trims the aircraft and linearizes about that trim point.
% Produces: X_trim, U_trim, sys_struct, Att_Trim, Vel_B_BA_Trim,
%           Rates_Trim, Vel_W_Trim, sys_ss_9state, and LQR gains.

SetupForTrimAndLin_Aircraft6dof;

%% Get state names from the linearization model
[sizes, x0, names] = Aircraft6dof_forTrimAndLin_ATT_wInputs([], [], [], 'sizes');

state_names = cell(1, numel(names));
for i = 1:numel(names)
    n = max(strfind(names{i}, '/'));
    state_names{i} = names{i}(n+1:end);
end

%% Define desired state ordering and trim constraints
desired_state_order = [{'Phi'} {'Theta'} {'Psi'} {'U'} {'V'} {'W'} ...
                       {'P'}   {'Q'}     {'R'}   {'dER'} {'dEL'} {'dE'} {'dR'} ...
                       {'hx'}  {'hy'}    {'hz'}];

fixed_states      = [{'Phi'} {'Psi'} {'V'} {'P'} {'Q'} {'R'} ...
                     {'hx'}  {'hy'}  {'hz'} {'dER'} {'dEL'} {'dE'} {'dR'}];
fixed_derivatives = [{'U'} {'W'} {'Q'}];
fixed_inputs      = [1; 2; 3; 4]; % Right Elevon, Left Elevon, Elevator, Rudder

u0 = [0; 0; 0; 0];

% Build index vectors
n_states    = [];
n_deriv     = [];
states_order = [];
for i = 1:length(fixed_states)
    n_states = [n_states, find(strcmp(fixed_states{i}, state_names))];
end
for i = 1:length(desired_state_order)
    states_order = [states_order, find(strcmp(desired_state_order{i}, state_names))];
end
for i = 1:length(fixed_derivatives)
    n_deriv = [n_deriv, find(strcmp(fixed_derivatives{i}, state_names))];
end

%% Trim
trimOptionsIn = [1; 1e-4; 1e-4; 1e-6; 0;0;0;0;0;0;0;0;0; 2000; 0;1e-8;0.1;0];

[X_trim, U_trim, Y_trim, DX, trimOptions] = trim( ...
    'Aircraft6dof_forTrimAndLin_ATT_wInputs', x0, u0, [], ...
    n_states, fixed_inputs, [], [], n_deriv, trimOptionsIn);

%% Extract trim values in desired state order
Att_Trim      = [X_trim(states_order(1)); X_trim(states_order(2)); X_trim(states_order(3))];
Att_Trim_deg  = Att_Trim * 180/pi;

Vel_B_BA_Trim = [X_trim(states_order(4)); X_trim(states_order(5)); X_trim(states_order(6))];
Rates_Trim    = [X_trim(states_order(7)); X_trim(states_order(8)); X_trim(states_order(9))];

% Actuator trim states
dER_trim = X_trim(states_order(10));
dEL_trim = X_trim(states_order(11));
dE_trim  = X_trim(states_order(12));
dR_trim  = X_trim(states_order(13));

% Wind-axes velocity [Vinf; alpha; beta]
Vel_W_Trim = [norm(Vel_B_BA_Trim);
              atan2(Vel_B_BA_Trim(3), Vel_B_BA_Trim(1));
              asin(Vel_B_BA_Trim(2) / norm(Vel_B_BA_Trim))];

C_BN = Att2Dcm_fcn(Att_Trim);
Vel_N_BA_Trim = C_BN' * Vel_B_BA_Trim;

fprintf('\n--- Trim Results ---\n');
fprintf('Att_Trim (deg): phi=%.3f  theta=%.3f  psi=%.3f\n', Att_Trim_deg(1), Att_Trim_deg(2), Att_Trim_deg(3));
fprintf('Vel_B_BA_Trim (m/s): u=%.4f  v=%.4f  w=%.4f\n', Vel_B_BA_Trim(1), Vel_B_BA_Trim(2), Vel_B_BA_Trim(3));
fprintf('Rates_Trim (rad/s): P=%.4f  Q=%.4f  R=%.4f\n', Rates_Trim(1), Rates_Trim(2), Rates_Trim(3));
fprintf('U_trim (rad): dER=%.4f  dEL=%.4f  dE=%.4f  dR=%.4f\n', U_trim(1), U_trim(2), U_trim(3), U_trim(4));
fprintf('Vinf_trim=%.4f m/s  alpha=%.4f rad  beta=%.4f rad\n\n', Vel_W_Trim(1), Vel_W_Trim(2), Vel_W_Trim(3));

%% Linearize about trim
% Pass both X_trim AND U_trim so linmod linearizes at the correct
% operating point. Omitting U_trim causes linmod to use u=0, giving wrong B.
sys_struct = linmod('Aircraft6dof_forTrimAndLin_ATT_wInputs', X_trim, U_trim);

%% Reorder states into desired order using states_order as an INDEX vector
% states_order(i) = index in sys_struct that corresponds to desired state i.
% Use it as an index vector (not a range) to correctly reorder A and B.
n_total = size(sys_struct.a, 1);
fprintf('Linearized system size: %dx%d\n', n_total, n_total);

% Full reordered system (all states in desired order)
A_ordered = sys_struct.a(states_order, states_order);
B_ordered = sys_struct.b(states_order, :);
C_ordered = sys_struct.c(:, states_order);

sys_ss = ss(sys_struct.a, sys_struct.b, sys_struct.c, sys_struct.d);

n_keep = 13; % [phi,theta,psi,u,v,w,P,Q,R,dER,dEL,dE,dR]
idx13  = states_order(1:n_keep);

sys_ss_13state = ss(sys_struct.a(idx13, idx13), ...
                    sys_struct.b(idx13, :), ...
                    sys_struct.c(:, idx13), ...
                    sys_struct.d);

fprintf('13-state system eigenvalues:\n');
disp(eig(sys_ss_13state.a));

%% Kalman estimator on 13-state system 
sys_ss_13state_wNoise = ss(sys_ss_13state.a, ...
    [sys_ss_13state.b, eye(size(sys_ss_13state.a,1))], ...
    sys_ss_13state.c, ...
    [sys_ss_13state.d, zeros(size(sys_ss_13state.c,1), size(sys_ss_13state.a,1))]);

[kest, L, P] = kalman(sys_ss_13state_wNoise, ...
    0.01*eye(size(sys_ss_13state.a,1)), ...
    0.01*eye(size(sys_ss_13state.c,1)));

%% LQR Design
Design_LQR_Controller;