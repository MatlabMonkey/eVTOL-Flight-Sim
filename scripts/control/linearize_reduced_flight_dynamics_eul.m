function data = linearize_reduced_flight_dynamics_eul(varargin)
%LINEARIZE_REDUCED_FLIGHT_DYNAMICS_EUL Trim, linearize, and extract textbook
% longitudinal and lateral small-disturbance models from Brown_6DOF_Plant_EUL.

p = inputParser;
p.addParameter('Model', 'Brown_6DOF_Plant_EUL', @(s) ischar(s) || isstring(s));
p.addParameter('CruiseSpeed', 70, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('DisplayReport', 'off', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});
opts = p.Results;

trimPoint = find_trim_point_eul_simple( ...
    'Model', opts.Model, ...
    'CruiseSpeed', opts.CruiseSpeed, ...
    'DisplayReport', opts.DisplayReport);

load_system(char(opts.Model));
sys = linearize(char(opts.Model), trimPoint.operatingPoint);

idx = struct();
idx.state.phi = 1;
idx.state.theta = 2;
idx.state.psi = 3;
idx.state.pos = 4:6;
idx.state.p = 7;
idx.state.q = 8;
idx.state.r = 9;
idx.state.u = 10;
idx.state.v = 11;
idx.state.w = 12;

idx.input.motor = 1:12;
idx.input.tilt = 13:18;
idx.input.front_collective = 19;
idx.input.rear_collective = 20;
idx.input.delta_f = 21;
idx.input.delta_a = 22;
idx.input.delta_e = 23;
idx.input.delta_r = 24;

idx.output.pos = 1:3;
idx.output.v_b = 4:6;
idx.output.v_e = 7:9;
idx.output.eul = 10:12;
idx.output.omega = 13:15;
idx.output.C_NB = 16:24;
idx.output.v_ba = 25:27;
idx.output.vinf = 28;
idx.output.alpha = 29;
idx.output.beta = 30;
idx.output.specific_force = 31:33;

ixLong = [idx.state.u idx.state.w idx.state.q idx.state.theta];
iuLong = [idx.input.front_collective idx.input.rear_collective idx.input.delta_e];
ixLat = [idx.state.v idx.state.p idx.state.r idx.state.phi];
iuLat = [idx.input.delta_a idx.input.delta_r];

data = struct();
data.trimPoint = trimPoint;
data.full = sys;
data.idx = idx;

data.long.state_idx = ixLong;
data.long.state_names = {'u','w','q','theta'};
data.long.input_idx = iuLong;
data.long.input_names = {'front_collective','rear_collective','delta_e'};
data.long.A = sys.A(ixLong, ixLong);
data.long.B = sys.B(ixLong, iuLong);
data.long.C = eye(numel(ixLong));
data.long.D = zeros(numel(ixLong), numel(iuLong));
data.long.sys = ss(data.long.A, data.long.B, data.long.C, data.long.D);
data.long.coupling_from_lateral = sys.A(ixLong, ixLat);

data.lat.state_idx = ixLat;
data.lat.state_names = {'v','p','r','phi'};
data.lat.input_idx = iuLat;
data.lat.input_names = {'delta_a','delta_r'};
data.lat.A = sys.A(ixLat, ixLat);
data.lat.B = sys.B(ixLat, iuLat);
data.lat.C = eye(numel(ixLat));
data.lat.D = zeros(numel(ixLat), numel(iuLat));
data.lat.sys = ss(data.lat.A, data.lat.B, data.lat.C, data.lat.D);
data.lat.coupling_from_longitudinal = sys.A(ixLat, ixLong);

data.open_loop_full_poles = eig(sys.A);
data.open_loop_long_poles = eig(data.long.A);
data.open_loop_lat_poles = eig(data.lat.A);

data.transfer = struct();
data.transfer.q_over_delta_e = minreal(ss(data.long.A, data.long.B(:, 3), [0 0 1 0], 0));
data.transfer.u_over_front_collective = minreal(ss(data.long.A, data.long.B(:, 1), [1 0 0 0], 0));
data.transfer.theta_over_delta_e = minreal(ss(data.long.A, data.long.B(:, 3), [0 0 0 1], 0));
data.transfer.p_over_delta_a = minreal(ss(data.lat.A, data.lat.B(:, 1), [0 1 0 0], 0));
data.transfer.r_over_delta_r = minreal(ss(data.lat.A, data.lat.B(:, 2), [0 0 1 0], 0));
end
