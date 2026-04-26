function u_cmd = controller_lqr_hold(x_meas, x_ref, trim_cmd, K_lqr)
%CONTROLLER_LQR_HOLD Full-state trim-hold LQR around one operating point.
% Inputs:
%   x_meas   9x1 = [phi; theta; psi; u; v; w; p; q; r]
%   x_ref    9x1 = same ordering as x_meas
%   trim_cmd 6x1 = [front_collective; rear_collective; delta_f; delta_a; delta_e; delta_r]
%   K_lqr    5x9 = output order [front_collective; delta_f; delta_a; delta_e; delta_r]
%
% Output:
%   u_cmd    6x1 = [front_collective; rear_collective; delta_f; delta_a; delta_e; delta_r]
%
% The dispatcher converts the mixed surface channels into the physical
% left/right wing and tail outputs used by the Wrapper model.

u_cmd = controller_hold_trim(trim_cmd);

x_err = x_meas - x_ref;
u_delta = -K_lqr * x_err;

u_cmd(1) = trim_cmd(1) + u_delta(1);
u_cmd(3) = trim_cmd(3) + u_delta(2);
u_cmd(4) = trim_cmd(4) + u_delta(3);
u_cmd(5) = trim_cmd(5) + u_delta(4);
u_cmd(6) = trim_cmd(6) + u_delta(5);
end
