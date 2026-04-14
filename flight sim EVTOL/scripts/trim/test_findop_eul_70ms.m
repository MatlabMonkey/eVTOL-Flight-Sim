repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

% Populate the base workspace because operating-point compilation evaluates
% block masks against base-workspace variables.
evalin('base', 'render_enable = false; run(''Full_Sim_Init.m''); use_avl_aero = false;');

model = 'Brown_6DOF_Plant_EUL';
load_system(model);

% These labels are great for debugging, but findop/operspec treat the
% ContinuousStateAttributes field as an evaluable parameter. Temporarily clear
% the custom labels so the model can compile for operating-point search.
stateBlocks = { ...
    [model '/Rotational Dynamics/Integrator3'], ...
    [model '/Translational Dynamics/Integrator3']};
oldAttrs = cell(size(stateBlocks));
for i = 1:numel(stateBlocks)
    oldAttrs{i} = get_param(stateBlocks{i}, 'ContinuousStateAttributes');
    set_param(stateBlocks{i}, 'ContinuousStateAttributes', '');
end
cleanupAttrs = onCleanup(@() restoreAttrs(stateBlocks, oldAttrs)); %#ok<NASGU>

set_param(model, 'SimulationCommand', 'update');

opspec = operspec(model);

% Inputs: 1:12 motor vector, 13:18 tilt, 19 front collective, 20 rear collective,
% 21 flap, 22 aileron, 23 elevator, 24 rudder.
opspec.Inputs(1).u = zeros(12,1);
opspec.Inputs(1).Known = true(12,1);

opspec.Inputs(2).u = 90 * ones(6,1);
opspec.Inputs(2).Known = true(6,1);

opspec.Inputs(3).u = 1181.43;
opspec.Inputs(3).Known = false;
opspec.Inputs(3).Min = 0;
opspec.Inputs(3).Max = 3000;

opspec.Inputs(4).u = 0;
opspec.Inputs(4).Known = false;
opspec.Inputs(4).Min = 0;
opspec.Inputs(4).Max = 3000;

opspec.Inputs(5).u = 0;
opspec.Inputs(5).Known = true;
opspec.Inputs(6).u = 0;
opspec.Inputs(6).Known = true;
opspec.Inputs(7).u = deg2rad(-15.06);
opspec.Inputs(7).Known = false;
opspec.Inputs(7).Min = deg2rad(-15);
opspec.Inputs(7).Max = deg2rad(15);
opspec.Inputs(8).u = 0;
opspec.Inputs(8).Known = true;

% States: EUL plant order is [eul(3); pos(3); omega(3); V_B(3)].
% Hold the intended cruise attitude and body velocity, require the dynamic
% states to be steady, and let only the trim actuators move.
opspec.States(1).x = [0; deg2rad(2.63); 0];
opspec.States(1).Known = [true; true; true];
opspec.States(1).SteadyState = [true; true; true];

opspec.States(2).x = [0; 0; -1000];
opspec.States(2).Known = [false; false; false];
opspec.States(2).SteadyState = [false; false; false];

opspec.States(3).x = [0; 0; 0];
opspec.States(3).Known = [true; true; true];
opspec.States(3).SteadyState = [true; true; true];

opspec.States(4).x = [70*cosd(2.63); 0; 70*sind(2.63)];
opspec.States(4).Known = [true; true; true];
opspec.States(4).SteadyState = [true; true; true];

opts = findopOptions('DisplayReport','iter');
[op, opreport] = findop(model, opspec, opts); %#ok<ASGLU>

fprintf('\nfindop result for %s\n', model);
fprintf('front collective rpm : %.6f\n', op.Inputs(3).u);
fprintf('rear collective rpm  : %.6f\n', op.Inputs(4).u);
fprintf('delta_e deg          : %.6f\n', rad2deg(op.Inputs(7).u));
fprintf('eul deg              : [%.6f %.6f %.6f]\n', rad2deg(op.States(1).x(1)), rad2deg(op.States(1).x(2)), rad2deg(op.States(1).x(3)));
fprintf('omega                : [%.6e %.6e %.6e]\n', op.States(3).x(1), op.States(3).x(2), op.States(3).x(3));
fprintf('V_B                  : [%.6f %.6f %.6f]\n', op.States(4).x(1), op.States(4).x(2), op.States(4).x(3));

assignin('base', 'findop_eul_70_result', op);
assignin('base', 'findop_eul_70_report', opreport);

function restoreAttrs(blocks, attrs)
for i = 1:numel(blocks)
    try
        if bdIsLoaded(bdroot(blocks{i}))
            set_param(blocks{i}, 'ContinuousStateAttributes', attrs{i});
        end
    catch
    end
end
end
