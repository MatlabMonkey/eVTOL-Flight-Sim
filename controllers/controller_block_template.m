function [motor_rpm_cmd_out, tilt_angles_cmd_out, front_collective_rpm_out, rear_collective_rpm_out, deltaLW, deltaRW, deltaLT, deltaRT] = ...
    controller_block_template( ...
    airData_cmd, eul_cmd, gps_Pos_cmd, omega_cmd, accel_cmd, ...
    motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm, ...
    airData_meas, eul_meas, gps_Pos_meas, omega_meas, accel_meas)
%CONTROLLER_BLOCK_TEMPLATE Paste this body into the variant choice MATLAB Function block.
%
% Current Wrapper/Controller variant interface:
%   Inputs
%     airData_cmd, eul_cmd, gps_Pos_cmd, omega_cmd, accel_cmd,
%     motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm,
%     airData_meas, eul_meas, gps_Pos_meas, omega_meas, accel_meas
%   Outputs
%     motor_rpm_cmd_out, tilt_angles_cmd_out,
%     front_collective_rpm_out, rear_collective_rpm_out,
%     deltaLW, deltaRW, deltaLT, deltaRT
%
% Workspace parameters consumed by the dispatcher:
%   controller_id
%   controller_state_ref
%   controller_trim_cmd
%   controller_gain_lqr
%   controller_surface_limit_rad
%   front_collective_min_rpm / max_rpm
%   rear_collective_min_rpm  / max_rpm
%   MixMatrix
%
% The body can be pasted directly into the block and renamed to `fcn`.

%#codegen
[motor_rpm_cmd_out, tilt_angles_cmd_out, front_collective_rpm_out, rear_collective_rpm_out, deltaLW, deltaRW, deltaLT, deltaRT] = ...
    controller_dispatch( ...
    airData_cmd, eul_cmd, gps_Pos_cmd, omega_cmd, accel_cmd, ...
    motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm, ...
    airData_meas, eul_meas, gps_Pos_meas, omega_meas, accel_meas, ...
    controller_id, controller_state_ref, controller_trim_cmd, ...
    controller_gain_lqr, controller_surface_limit_rad, ...
    front_collective_min_rpm, front_collective_max_rpm, ...
    rear_collective_min_rpm, rear_collective_max_rpm, MixMatrix);
end
