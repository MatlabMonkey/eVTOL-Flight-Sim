% Test_LowSpeed_Polar_Seed.m
% Quick validation harness for the LUT-based low-speed seed generator.

Init_EVTOL_Main

trimCase = TrimCase_Cruise75_FlapElevator();
trimCase.name = 'PolarSeedTest_20_45_800';
trimCase.mode = 'transition_low_speed';
trimCase.Vinf_mps = 45.0;
trimCase.u_body_mps = 45.0;
trimCase.v_body_mps = 0.0;
trimCase.w_body_mps = 0.0;
trimCase.front_tilt_deg = 20.0;
trimCase.front_tilt_cmd_deg = 20.0;
trimCase.rear_collective_known = true;
trimCase.rear_collective_fixed_rpm = 800.0;
trimCase.rear_collective_guess_rpm = 800.0;
trimCase.position_steady = [false; false; false];
trimCase.validate_nonlinear_hold = false;

[seed, details] = make_low_speed_seed_from_polars(initData, trimCase);

fprintf('Polar seed:\n');
disp(seed);
fprintf('Seed diagnostics: forward residual %.3f N | vertical residual %.3f N | pitch moment %.3f Nm\n', ...
    details.best.forward_residual_N, details.best.vertical_residual_N, details.best.net_pitch_moment_Nm);

bestResidual = inf;
bestLabel = '';
bestTrimResult = struct();

for frontScale = [0.95, 1.00, 1.05, 1.10, 1.15]
    for thetaOffsetDeg = [-1, 0, 1, 2]
        for deltaFOffsetDeg = [-2, 0, 2]
            for deltaEOffsetDeg = [-2, 0, 2]
                tc = trimCase;
                tc.front_collective_guess_rpm = seed.front_collective_guess_rpm * frontScale;
                tc.theta_guess_deg = seed.theta_guess_deg + thetaOffsetDeg;
                tc.delta_f_guess_deg = seed.delta_f_guess_deg + deltaFOffsetDeg;
                tc.delta_e_guess_deg = seed.delta_e_guess_deg + deltaEOffsetDeg;

                trimResult = trim_evtol_case(initData, tc, struct( ...
                    'verbose', false, ...
                    'debug', false, ...
                    'emitSummary', false, ...
                    'emitLinearSummary', false));

                maxResidual = localMaxResidual(trimResult.op_report);
                label = sprintf('frontScale=%.2f thetaOffset=%+.1f deltaFOffset=%+.1f deltaEOffset=%+.1f', ...
                    frontScale, thetaOffsetDeg, deltaFOffsetDeg, deltaEOffsetDeg);

                if trimResult.success
                    fprintf('Exact trim found with %s\n', label);
                    fprintf('  front = %.2f rpm | rear = %.2f rpm | theta = %.3f deg | df = %.3f deg | de = %.3f deg\n', ...
                        trimResult.scheduling.front_collective_rpm, ...
                        trimResult.scheduling.rear_collective_rpm, ...
                        trimResult.Att_Trim_deg(2), ...
                        rad2deg(trimResult.scheduling.delta_f_rad), ...
                        rad2deg(trimResult.scheduling.delta_e_rad));
                    lowSpeedPolarSeedResult = trimResult; %#ok<NASGU>
                    return;
                end

                if maxResidual < bestResidual
                    bestResidual = maxResidual;
                    bestLabel = label;
                    bestTrimResult = trimResult;
                end
            end
        end
    end
end

fprintf('No exact trim found. Best residual %.6g with %s\n', bestResidual, bestLabel);
if ~isempty(fieldnames(bestTrimResult))
    fprintf('  front = %.2f rpm | rear = %.2f rpm | theta = %.3f deg | df = %.3f deg | de = %.3f deg\n', ...
        bestTrimResult.scheduling.front_collective_rpm, ...
        bestTrimResult.scheduling.rear_collective_rpm, ...
        bestTrimResult.Att_Trim_deg(2), ...
        rad2deg(bestTrimResult.scheduling.delta_f_rad), ...
        rad2deg(bestTrimResult.scheduling.delta_e_rad));
end

function maxResidual = localMaxResidual(opReport)
maxResidual = inf;
try
    states = opReport.States;
catch
    return;
end

values = [];
for i = 1:numel(states)
    dx = states(i).dx(:);
    steadyMask = logical(states(i).SteadyState(:));
    if numel(steadyMask) ~= numel(dx)
        steadyMask = true(size(dx));
    end
    values = [values; abs(dx(steadyMask))]; %#ok<AGROW>
end

if ~isempty(values)
    maxResidual = max(values);
end
end
