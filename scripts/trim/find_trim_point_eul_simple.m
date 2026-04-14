function trimPoint = find_trim_point_eul_simple(varargin)
%FIND_TRIM_POINT_EUL_SIMPLE Find a local cruise trim point with findop.
%   trimPoint = find_trim_point_eul_simple()
%   trimPoint = find_trim_point_eul_simple('CruiseSpeed', 70, 'Model', 'Brown_6DOF_Plant_EUL')
%
% This is the simplest reusable operating-point search for the Euler-angle
% plant. It keeps the search local to straight cruise, solves for
% front/rear collective, elevator, pitch, and the longitudinal body-velocity
% split, then returns a named trim-point struct plus the state/input vectors
% you can reuse for simulation or linearization.

p = inputParser;
p.addParameter('Model', 'Brown_6DOF_Plant_EUL', @(s) ischar(s) || isstring(s));
p.addParameter('CruiseSpeed', 70, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('TiltDeg', 90, @(x) isnumeric(x) && isscalar(x));
p.addParameter('AltitudeNED', -1000, @(x) isnumeric(x) && isscalar(x));
p.addParameter('DisplayReport', 'off', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});
opts = p.Results;

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupDir = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

evalin('base', 'render_enable = false;');
evalin('base', 'run(''Full_Sim_Init.m'');');
assignin('base', 'use_avl_aero', false);

model = char(opts.Model);
load_system(model);

[stateBlocks, oldAttrs] = localCaptureStateAttrs(model);
cleanupAttrs = onCleanup(@() localRestoreAttrs(stateBlocks, oldAttrs)); %#ok<NASGU>

set_param(model, 'SimulationCommand', 'update');
opspec = operspec(model);

alphaSeedDeg = 2.63;
thetaSeedDeg = alphaSeedDeg;
frontSeedRpm = 1181.43;
rearSeedRpm = 0;
deltaESeedDeg = -15.0602653566512;

opspec.Inputs(1).u = zeros(12, 1);
opspec.Inputs(1).Known = true(12, 1);

opspec.Inputs(2).u = opts.TiltDeg * ones(6, 1);
opspec.Inputs(2).Known = true(6, 1);

opspec.Inputs(3).u = frontSeedRpm;
opspec.Inputs(3).Known = false;
opspec.Inputs(3).Min = 0;
opspec.Inputs(3).Max = 3000;

opspec.Inputs(4).u = rearSeedRpm;
opspec.Inputs(4).Known = false;
opspec.Inputs(4).Min = 0;
opspec.Inputs(4).Max = 3000;

opspec.Inputs(5).u = 0;
opspec.Inputs(5).Known = true;
opspec.Inputs(6).u = 0;
opspec.Inputs(6).Known = true;

opspec.Inputs(7).u = deg2rad(deltaESeedDeg);
opspec.Inputs(7).Known = false;
opspec.Inputs(7).Min = deg2rad(-25);
opspec.Inputs(7).Max = deg2rad(5);

opspec.Inputs(8).u = 0;
opspec.Inputs(8).Known = true;

% State order in Brown_6DOF_Plant_EUL:
% 1 eul [phi theta psi]
% 2 position [N E D]
% 3 omega [p q r]
% 4 velocity [u v w]
opspec.States(1).x = [0; deg2rad(thetaSeedDeg); 0];
opspec.States(1).Known = [true; false; true];
opspec.States(1).SteadyState = [true; true; true];
opspec.States(1).Min = [0; deg2rad(-2); 0];
opspec.States(1).Max = [0; deg2rad(8); 0];

opspec.States(2).x = [0; 0; opts.AltitudeNED];
opspec.States(2).Known = [false; false; false];
opspec.States(2).SteadyState = [false; false; false];

opspec.States(3).x = [0; 0; 0];
opspec.States(3).Known = [true; true; true];
opspec.States(3).SteadyState = [true; true; true];

opspec.States(4).x = [opts.CruiseSpeed * cosd(alphaSeedDeg); 0; opts.CruiseSpeed * sind(alphaSeedDeg)];
opspec.States(4).Known = [false; true; false];
opspec.States(4).SteadyState = [true; true; true];
opspec.States(4).Min = [opts.CruiseSpeed - 5; 0; 0];
opspec.States(4).Max = [opts.CruiseSpeed + 5; 0; 6];

% Root output 8 is vinf_truth.
opspec.Outputs(8).y = opts.CruiseSpeed;
opspec.Outputs(8).Known = true;
opspec.Outputs(8).Min = opts.CruiseSpeed - 0.5;
opspec.Outputs(8).Max = opts.CruiseSpeed + 0.5;

opOpts = findopOptions('DisplayReport', char(opts.DisplayReport));
[op, opreport] = findop(model, opspec, opOpts); %#ok<ASGLU>

trimPoint = struct();
trimPoint.model = model;
trimPoint.cruise_speed_mps = opts.CruiseSpeed;
trimPoint.operatingPoint = op;
trimPoint.position = op.States(2).x(:);
trimPoint.eul = op.States(1).x(:);
trimPoint.omega = op.States(3).x(:);
trimPoint.velocity = op.States(4).x(:);
trimPoint.vinf = norm(trimPoint.velocity);

trimPoint.inputs = struct();
trimPoint.inputs.motor_rpm_cmd = zeros(12, 1);
trimPoint.inputs.tilt_angles_cmd = opts.TiltDeg * ones(6, 1);
trimPoint.inputs.front_collective_rpm = op.Inputs(3).u;
trimPoint.inputs.rear_collective_rpm = op.Inputs(4).u;
trimPoint.inputs.delta_f = op.Inputs(5).u;
trimPoint.inputs.delta_a = op.Inputs(6).u;
trimPoint.inputs.delta_e = op.Inputs(7).u;
trimPoint.inputs.delta_r = op.Inputs(8).u;

trimPoint.x0 = [trimPoint.eul; trimPoint.position; trimPoint.omega; trimPoint.velocity];
trimPoint.u0 = [ ...
    trimPoint.inputs.motor_rpm_cmd; ...
    trimPoint.inputs.tilt_angles_cmd; ...
    trimPoint.inputs.front_collective_rpm; ...
    trimPoint.inputs.rear_collective_rpm; ...
    trimPoint.inputs.delta_f; ...
    trimPoint.inputs.delta_a; ...
    trimPoint.inputs.delta_e; ...
    trimPoint.inputs.delta_r];

assignin('base', 'trimPoint', trimPoint);
assignin('base', 'trimPointReport', opreport);

close_system(model, 0);
end

function localRestoreAttrs(blocks, attrs)
for i = 1:numel(blocks)
    try
        if bdIsLoaded(bdroot(blocks{i}))
            set_param(blocks{i}, 'ContinuousStateAttributes', attrs{i});
        end
    catch
    end
end
end

function [blocks, attrs] = localCaptureStateAttrs(model)
blocks = {};
attrs = {};

allBlocks = find_system(model, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'Type', 'Block');

for i = 1:numel(allBlocks)
    blk = allBlocks{i};
    try
        attr = get_param(blk, 'ContinuousStateAttributes');
    catch
        continue;
    end

    if isempty(attr)
        continue;
    end

    if ischar(attr) || isstring(attr)
        attrText = strtrim(char(attr));
        if strcmp(attrText, '''''')
            continue;
        end
    end

    blocks{end+1} = blk; %#ok<AGROW>
    attrs{end+1} = attr; %#ok<AGROW>
    set_param(blk, 'ContinuousStateAttributes', '');
end
end
