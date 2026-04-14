function apply_trim_point_to_workspace(trimPoint)
%APPLY_TRIM_POINT_TO_WORKSPACE Push a trim-point struct into base variables.
%   apply_trim_point_to_workspace(trimPoint)
%
% This keeps the trim point as the named source of truth, while still
% populating the legacy workspace variables used by the plant and wrapper.

arguments
    trimPoint struct
end

assignin('base', 'pos_init', trimPoint.position(:));
assignin('base', 'eul_init', trimPoint.eul(:));
assignin('base', 'omega_init', trimPoint.omega(:));
assignin('base', 'V_init', trimPoint.velocity(:));

assignin('base', 'Motor_RPMs', [ ...
    trimPoint.inputs.front_collective_rpm * ones(6, 1); ...
    trimPoint.inputs.rear_collective_rpm * ones(6, 1)]);
assignin('base', 'Tilt_angles', trimPoint.inputs.tilt_angles_cmd(:));

assignin('base', 'controller_trim_motor_rpms', zeros(4, 1));
assignin('base', 'controller_trim_tilt_angles', trimPoint.inputs.tilt_angles_cmd(1:2));
assignin('base', 'controller_trim_motor_rpm_cmd', trimPoint.inputs.motor_rpm_cmd(:));
assignin('base', 'controller_trim_tilt_angles_cmd', trimPoint.inputs.tilt_angles_cmd(:));
assignin('base', 'controller_trim_delta_f', trimPoint.inputs.delta_f);
assignin('base', 'controller_trim_delta_a', trimPoint.inputs.delta_a);
assignin('base', 'controller_trim_delta_e', trimPoint.inputs.delta_e);
assignin('base', 'controller_trim_delta_r', trimPoint.inputs.delta_r);
assignin('base', 'controller_airspeed_cmd_base', trimPoint.vinf);
assignin('base', 'controller_bank_cmd_base', trimPoint.eul(1));
assignin('base', 'trim_front_collective_rpm', trimPoint.inputs.front_collective_rpm);
assignin('base', 'trim_rear_collective_rpm', trimPoint.inputs.rear_collective_rpm);
assignin('base', 'estimator_init_eul', trimPoint.eul(:));
end
