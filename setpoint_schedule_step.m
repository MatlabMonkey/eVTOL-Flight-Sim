function [airspeed_cmd, bank_cmd] = setpoint_schedule_step(t, airspeed_cmd_base, ...
    bank_cmd_base, step_time, airspeed_step, bank_step)
%SETPOINT_SCHEDULE_STEP Simple step-based setpoint generator.

airspeed_cmd = airspeed_cmd_base;
bank_cmd = bank_cmd_base;

if t >= step_time
    airspeed_cmd = airspeed_cmd + airspeed_step;
    bank_cmd = bank_cmd + bank_step;
end
end

