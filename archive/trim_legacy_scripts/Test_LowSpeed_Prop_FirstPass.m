function results = Test_LowSpeed_Prop_FirstPass()
%TEST_LOWSPEED_PROP_FIRSTPASS Exercise the props-only first-pass seed.
%
% This harness keeps all aero controls fixed at zero and tests whether the
% props-only seed can land us in a trim basin at representative low-speed
% transition points. It reports both the direct seed result and the best
% local perturbation over front/rear/theta only.

    evalin('base', 'Init_EVTOL_Main;');
    initData = evalin('base', 'initData');

    test_points = [ ...
        15.0, 45.0; ...
        20.0, 35.0; ...
        20.0, 40.0; ...
        20.0, 45.0; ...
        25.0, 45.0; ...
        30.0, 45.0];

    front_scale_grid = [0.95, 1.00, 1.05, 1.10];
    rear_scale_grid = [0.90, 1.00, 1.10];
    theta_offset_grid_deg = [-2.0, 0.0, 2.0];

    base_options = struct( ...
        'verbose', false, ...
        'debug', false, ...
        'emitSummary', false, ...
        'emitLinearSummary', false);

    results = repmat(localEmptyResult(), size(test_points, 1), 1);

    fprintf('\n=== Props-Only First-Pass Seed Check ===\n');
    for i = 1:size(test_points, 1)
        tilt_deg = test_points(i, 1);
        vinf_mps = test_points(i, 2);

        trimCase = TrimCase_Cruise75_PropOnly();
        trimCase.name = sprintf('PropFirstPass_Tilt%s_V%s', ...
            localValueLabel(tilt_deg), localValueLabel(vinf_mps));
        trimCase.front_tilt_deg = tilt_deg;
        trimCase.Vinf_mps = vinf_mps;
        trimCase.u_body_mps = vinf_mps;
        trimCase.w_body_mps = 0.0;
        trimCase.position_steady = [false; false; false];
        trimCase.use_vertical_speed_output_constraint = true;
        trimCase.validate_nonlinear_hold = false;

        [seed, seedDetails] = make_low_speed_prop_first_pass_seed(initData, trimCase);

        trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
        trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
        trimCase.rear_collective_trim_rpm = seed.rear_collective_guess_rpm;
        trimCase.theta_guess_deg = seed.theta_guess_deg;

        rawText = evalc('[rawResult, ~] = trim_evtol_case(initData, trimCase, base_options);');
        rawResidual = localMaxResidual(rawResult);

        bestResult = rawResult;
        bestResidual = rawResidual;
        bestLabel = 'raw';

        if ~localIsExact(rawResult)
            for frontScale = front_scale_grid
                for rearScale = rear_scale_grid
                    for thetaOffset = theta_offset_grid_deg
                        trialCase = trimCase;
                        trialCase.front_collective_guess_rpm = seed.front_collective_guess_rpm * frontScale;
                        trialCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm * rearScale;
                        trialCase.rear_collective_trim_rpm = trialCase.rear_collective_guess_rpm;
                        trialCase.theta_guess_deg = seed.theta_guess_deg + thetaOffset;

                        trialText = evalc('[trialResult, ~] = trim_evtol_case(initData, trialCase, base_options);');
                        trialResidual = localMaxResidual(trialResult);

                        if localIsExact(trialResult)
                            bestResult = trialResult;
                            bestResidual = trialResidual;
                            bestLabel = sprintf('front x %.2f, rear x %.2f, theta %+g', ...
                                frontScale, rearScale, thetaOffset);
                            break;
                        end

                        if trialResidual < bestResidual
                            bestResult = trialResult;
                            bestResidual = trialResidual;
                            bestLabel = sprintf('front x %.2f, rear x %.2f, theta %+g', ...
                                frontScale, rearScale, thetaOffset);
                        end
                    end
                    if localIsExact(bestResult)
                        break;
                    end
                end
                if localIsExact(bestResult)
                    break;
                end
            end
        end

        results(i).tilt_deg = tilt_deg;
        results(i).vinf_mps = vinf_mps;
        results(i).seed = seed;
        results(i).seedDetails = seedDetails;
        results(i).rawResult = rawResult;
        results(i).rawConsoleText = rawText;
        results(i).rawResidual = rawResidual;
        results(i).bestResult = bestResult;
        results(i).bestResidual = bestResidual;
        results(i).bestLabel = bestLabel;
        if exist('trialText', 'var')
            results(i).lastTrialConsoleText = trialText;
        else
            results(i).lastTrialConsoleText = '';
        end
        results(i).success = localIsExact(bestResult);

        fprintf('tilt=%5.1f deg  V=%5.1f m/s  seed(front=%.1f rear=%.1f theta=%.2f)  ', ...
            tilt_deg, vinf_mps, seed.front_collective_guess_rpm, ...
            seed.rear_collective_guess_rpm, seed.theta_guess_deg);
        if results(i).success
            fprintf('EXACT via %s  -> front=%.2f rear=%.2f theta=%.3f\n', ...
                bestLabel, bestResult.scheduling.front_collective_rpm, ...
                bestResult.scheduling.rear_collective_rpm, bestResult.Att_Trim_deg(2));
        else
            fprintf('no exact trim, best residual=%.4g via %s\n', bestResidual, bestLabel);
        end
    end

    assignin('base', 'lowSpeedPropFirstPassResults', results);
end

function tf = localIsExact(trimResult)
    tf = isfield(trimResult, 'success') && logical(trimResult.success);
end

function value = localMaxResidual(trimResult)
    value = inf;
    if isfield(trimResult, 'diagnostics') && isfield(trimResult.diagnostics, 'stateResiduals') ...
            && ~isempty(trimResult.diagnostics.stateResiduals)
        residuals = abs(trimResult.diagnostics.stateResiduals(:));
        value = max(residuals);
    end
end

function result = localEmptyResult()
    result = struct( ...
        'tilt_deg', NaN, ...
        'vinf_mps', NaN, ...
        'seed', struct(), ...
        'seedDetails', struct(), ...
        'rawResult', struct(), ...
        'rawConsoleText', '', ...
        'rawResidual', inf, ...
        'bestResult', struct(), ...
        'bestResidual', inf, ...
        'bestLabel', '', ...
        'lastTrialConsoleText', '', ...
        'success', false);
end

function label = localValueLabel(value)
    label = strrep(num2str(value, '%.4g'), '.', 'p');
    label = strrep(label, '-', 'm');
end
