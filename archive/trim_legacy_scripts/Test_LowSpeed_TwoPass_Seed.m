function results = Test_LowSpeed_TwoPass_Seed()
%TEST_LOWSPEED_TWOPASS_SEED Check the second-pass low-speed seed helper.

    evalin('base', 'Init_EVTOL_Main;');
    initData = evalin('base', 'initData');

    tilts_deg = [20.0, 25.0, 30.0];
    vinf_mps = 45.0;

    theta_offset_grid_deg = [-1.0, 0.0, 1.0];
    delta_f_offset_grid_deg = [-2.0, 0.0, 2.0];
    delta_e_offset_grid_deg = [-2.0, 0.0, 2.0];

    base_options = struct( ...
        'verbose', false, ...
        'debug', false, ...
        'emitSummary', false, ...
        'emitLinearSummary', false);

    results = repmat(localEmptyResult(), numel(tilts_deg), 1);

    fprintf('\n=== Low-Speed Two-Pass Seed Check ===\n');
    for i = 1:numel(tilts_deg)
        tilt_deg = tilts_deg(i);

        trimCase = TrimCase_Cruise75_FlapElevator();
        trimCase.name = sprintf('TwoPass_Tilt%s_V%s', ...
            localValueLabel(tilt_deg), localValueLabel(vinf_mps));
        trimCase.Vinf_mps = vinf_mps;
        trimCase.u_body_mps = vinf_mps;
        trimCase.w_body_mps = 0.0;
        trimCase.front_tilt_deg = tilt_deg;
        trimCase.front_collective_known = true;
        trimCase.rear_collective_known = true;
        trimCase.position_steady = [false; false; false];
        trimCase.use_vertical_speed_output_constraint = true;
        trimCase.validate_nonlinear_hold = false;

        [seed, seedDetails] = make_low_speed_two_pass_seed(initData, trimCase);
        trimCase.front_collective_guess_rpm = seed.front_collective_guess_rpm;
        trimCase.front_collective_fixed_rpm = seed.front_collective_guess_rpm;
        trimCase.rear_collective_guess_rpm = seed.rear_collective_guess_rpm;
        trimCase.rear_collective_fixed_rpm = seed.rear_collective_guess_rpm;
        trimCase.rear_collective_trim_rpm = seed.rear_collective_guess_rpm;
        trimCase.theta_guess_deg = seed.theta_guess_deg;
        trimCase.delta_f_guess_deg = seed.delta_f_guess_deg;
        trimCase.delta_e_guess_deg = seed.delta_e_guess_deg;

        rawText = evalc('[rawResult, ~] = trim_evtol_case(initData, trimCase, base_options);');
        rawResidual = localMaxResidual(rawResult);

        bestResult = rawResult;
        bestResidual = rawResidual;
        bestLabel = 'raw';
        lastTrialText = '';

        if ~localIsExact(rawResult)
            for thetaOffset = theta_offset_grid_deg
                for deltaFOffset = delta_f_offset_grid_deg
                    for deltaEOffset = delta_e_offset_grid_deg
                        trialCase = trimCase;
                        trialCase.theta_guess_deg = seed.theta_guess_deg + thetaOffset;
                        trialCase.delta_f_guess_deg = seed.delta_f_guess_deg + deltaFOffset;
                        trialCase.delta_e_guess_deg = seed.delta_e_guess_deg + deltaEOffset;

                        lastTrialText = evalc('[trialResult, ~] = trim_evtol_case(initData, trialCase, base_options);');
                        trialResidual = localMaxResidual(trialResult);

                        if localIsExact(trialResult)
                            bestResult = trialResult;
                            bestResidual = trialResidual;
                            bestLabel = sprintf('theta %+g, df %+g, de %+g', ...
                                thetaOffset, deltaFOffset, deltaEOffset);
                            break;
                        end

                        if trialResidual < bestResidual
                            bestResult = trialResult;
                            bestResidual = trialResidual;
                            bestLabel = sprintf('theta %+g, df %+g, de %+g', ...
                                thetaOffset, deltaFOffset, deltaEOffset);
                        end
                    end
                    if localIsExact(bestResult), break; end
                end
                if localIsExact(bestResult), break; end
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
        results(i).lastTrialConsoleText = lastTrialText;
        results(i).success = localIsExact(bestResult);

        fprintf('tilt=%5.1f deg  V=%5.1f m/s  seed(front=%.1f rear=%.1f theta=%.2f df=%.2f de=%.2f)  ', ...
            tilt_deg, vinf_mps, seed.front_collective_guess_rpm, ...
            seed.rear_collective_guess_rpm, seed.theta_guess_deg, ...
            seed.delta_f_guess_deg, seed.delta_e_guess_deg);
        if results(i).success
            fprintf('EXACT via %s  -> front=%.2f rear=%.2f theta=%.3f df=%.3f de=%.3f\n', ...
                                    bestLabel, bestResult.scheduling.front_collective_rpm, ...
                bestResult.scheduling.rear_collective_rpm, bestResult.Att_Trim_deg(2), ...
                rad2deg(bestResult.trim.mixed_control_trim(1)), ...
                rad2deg(bestResult.trim.mixed_control_trim(3)));
        else
            fprintf('no exact trim, best residual=%.4g via %s\n', bestResidual, bestLabel);
        end
    end

    assignin('base', 'lowSpeedTwoPassResults', results);
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
