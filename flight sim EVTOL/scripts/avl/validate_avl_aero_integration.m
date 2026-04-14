% scripts/avl/validate_avl_aero_integration.m
% Repeatable validation checks for the compact AVL Simulink aero branch.

fprintf('== AVL aero integration validation ==\n');
fprintf('Timestamp: %s\n', datestr(now, 31));

thisFile = mfilename('fullpath');
repoRoot = fileparts(fileparts(fileparts(thisFile)));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

% -------------------------------------------------------------------------
% 1. Direct coefficient sanity checks against the stored AVL study
% -------------------------------------------------------------------------
avlAero = load_avl_aero_model(repoRoot);
V = [70*cosd(4); 0; 70*sind(4)];
[~, ~, C0] = avl_aircraft_aero_eval(V, 0, 0, 0, 0, 1.225, avlAero);
[~, ~, Cf] = avl_aircraft_aero_eval(V, 15, 15, 0, 0, 1.225, avlAero);
[~, ~, Ca] = avl_aircraft_aero_eval(V, 5, -5, 0, 0, 1.225, avlAero);
[~, ~, Ce] = avl_aircraft_aero_eval(V, 0, 0, 5, 5, 1.225, avlAero);
[~, ~, Cr] = avl_aircraft_aero_eval(V, 0, 0, -5, 5, 1.225, avlAero);

baselineExpected = [0.46835, 0.01170, 0.0, 0.0, -0.18689, 0.0];
baselineError = abs(C0(:)' - baselineExpected);
fprintf('[INFO] Baseline coefficient error = [%0.5f %0.5f %0.5f %0.5f %0.5f %0.5f]\n', baselineError);

if baselineError(1) < 0.02 && baselineError(2) < 0.005 && baselineError(5) < 0.03
    fprintf('[PASS] Compact AVL fit reproduces the stored baseline within tolerance\n');
else
    fprintf('[FAIL] Compact AVL fit baseline is outside tolerance\n');
    error('validate_avl_aero_integration:BaselineMismatch', ...
        'Compact AVL fit baseline is outside tolerance.');
end

if (Cf(1) - C0(1)) > 0
    fprintf('[PASS] Positive flap increases CL\n');
else
    fprintf('[FAIL] Positive flap did not increase CL\n');
    error('validate_avl_aero_integration:FlapSign', ...
        'Positive flap did not increase CL.');
end

if (Ca(4) - C0(4)) > 0
    fprintf('[PASS] Positive aileron produces positive Cl\n');
else
    fprintf('[FAIL] Positive aileron did not produce positive Cl\n');
    error('validate_avl_aero_integration:AileronSign', ...
        'Positive aileron did not produce positive Cl.');
end

if (Ce(5) - C0(5)) < 0
    fprintf('[PASS] Positive elevator produces more negative Cm\n');
else
    fprintf('[FAIL] Positive elevator did not produce more negative Cm\n');
    error('validate_avl_aero_integration:ElevatorSign', ...
        'Positive elevator did not produce more negative Cm.');
end

if (Cr(6) - C0(6)) > 0
    fprintf('[PASS] Positive rudder produces positive Cn\n');
else
    fprintf('[FAIL] Positive rudder did not produce positive Cn\n');
    error('validate_avl_aero_integration:RudderSign', ...
        'Positive rudder did not produce positive Cn.');
end

% -------------------------------------------------------------------------
% 2. Static trim-point spot checks
% -------------------------------------------------------------------------
trimTbl = readtable(fullfile(repoRoot, 'docs', 'avl_homework', 'tables', 'trim_cases.csv'));
for idx = 1:height(trimTbl)
    speed = trimTbl.speed_mps(idx);
    alphaDeg = trimTbl.alpha_trim_deg(idx);
    betaDeg = 0.0;
    Vtrim = [speed*cosd(alphaDeg)*cosd(betaDeg); ...
             speed*sind(betaDeg); ...
             speed*sind(alphaDeg)*cosd(betaDeg)];
    [~, ~, coeffs] = avl_aircraft_aero_eval( ...
        Vtrim, ...
        0.0 + trimTbl.aileron(idx), ...
        0.0 - trimTbl.aileron(idx), ...
        trimTbl.elevator(idx) - trimTbl.rudder(idx), ...
        trimTbl.elevator(idx) + trimTbl.rudder(idx), ...
        1.225, avlAero);

    if abs(coeffs(1) - trimTbl.CLtot(idx)) < 0.08 && abs(coeffs(2) - trimTbl.CDtot(idx)) < 0.02
        fprintf('[PASS] Trim spot check %s stayed close to stored AVL totals\n', trimTbl.case_name{idx});
    else
        fprintf('[WARN] Trim spot check %s drifted from stored AVL totals (CL %.4f vs %.4f, CD %.4f vs %.4f)\n', ...
            trimTbl.case_name{idx}, coeffs(1), trimTbl.CLtot(idx), coeffs(2), trimTbl.CDtot(idx));
    end
end

% -------------------------------------------------------------------------
% 3. Full-model branch-selection checks
% -------------------------------------------------------------------------
evalin('base', 'run(''Full_Sim_Init.m'');');
load_system('Brown_Full_Sim');
set_param('Brown_Full_Sim', 'StopTime', '0.05');

assignin('base', 'use_avl_aero', false);
assignin('base', 'controller_enable', false);
simOutAnalytic = sim('Brown_Full_Sim');
assertClose(simOutAnalytic.get('aero_force_selected'), simOutAnalytic.get('aero_force_analytic'), 1.0e-9, ...
    'Analytical selected force should equal analytical force log');
assertClose(simOutAnalytic.get('aero_moment_selected'), simOutAnalytic.get('aero_moment_analytic'), 1.0e-9, ...
    'Analytical selected moment should equal analytical moment log');
fprintf('[PASS] Analytical mode selects the original aero branch\n');

assignin('base', 'use_avl_aero', true);
assignin('base', 'controller_enable', false);
simOutAvlOff = sim('Brown_Full_Sim');
assertClose(simOutAvlOff.get('aero_force_selected'), simOutAvlOff.get('aero_force_avl'), 1.0e-9, ...
    'AVL selected force should equal AVL force log');
assertClose(simOutAvlOff.get('aero_moment_selected'), simOutAvlOff.get('aero_moment_avl'), 1.0e-9, ...
    'AVL selected moment should equal AVL moment log');
fprintf('[PASS] AVL mode selects the AVL aero branch (controller off)\n');

assignin('base', 'use_avl_aero', true);
assignin('base', 'controller_enable', true);
simOutAvlOn = sim('Brown_Full_Sim');
assertClose(simOutAvlOn.get('aero_force_selected'), simOutAvlOn.get('aero_force_avl'), 1.0e-9, ...
    'AVL selected force should equal AVL force log with controller enabled');
assertClose(simOutAvlOn.get('aero_moment_selected'), simOutAvlOn.get('aero_moment_avl'), 1.0e-9, ...
    'AVL selected moment should equal AVL moment log with controller enabled');
fprintf('[PASS] AVL mode selects the AVL aero branch (controller on)\n');

fprintf('AVL_AERO_VALIDATION: PASS\n');
close_system('Brown_Full_Sim', 0);

function assertClose(a, b, tol, message)
if max(abs(a(:) - b(:))) > tol
    error('validate_avl_aero_integration:Mismatch', '%s', message);
end
end
