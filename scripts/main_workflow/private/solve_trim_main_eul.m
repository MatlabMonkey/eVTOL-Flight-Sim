function trimPoint = solve_trim_main_eul(session, trimSpec)
%SOLVE_TRIM_MAIN_EUL Solve a fresh trim on the Euler-angle trim backend.

trimSpec = default_trim_spec_main(trimSpec);
assignin('base', 'use_avl_aero', logical(trimSpec.useAvlAero));

model = char(session.trimModel);
load_system(model);

[stateBlocks, oldAttrs] = localCaptureStateAttrs(model);
cleanupAttrs = onCleanup(@() localRestoreAttrs(stateBlocks, oldAttrs)); %#ok<NASGU>

set_param(model, 'SimulationCommand', 'update');
opspec = operspec(model);

seed = localBuildSeed(session, trimSpec);

opspec.Inputs(1).u = zeros(12, 1);
opspec.Inputs(1).Known = true(12, 1);

opspec.Inputs(2).u = localExpandTiltGrouped(trimSpec.tiltAnglesGrouped);
opspec.Inputs(2).Known = true(6, 1);

opspec.Inputs(3).u = seed.frontCollectiveRpm;
opspec.Inputs(3).Known = false;
opspec.Inputs(3).Min = 0;
opspec.Inputs(3).Max = 3000;

opspec.Inputs(4).u = seed.rearCollectiveRpm;
opspec.Inputs(4).Known = false;
opspec.Inputs(4).Min = 0;
opspec.Inputs(4).Max = 3000;

opspec.Inputs(5).u = seed.deltaF;
opspec.Inputs(5).Known = true;
opspec.Inputs(6).u = seed.deltaA;
opspec.Inputs(6).Known = true;

opspec.Inputs(7).u = seed.deltaE;
opspec.Inputs(7).Known = false;
opspec.Inputs(7).Min = deg2rad(-25);
opspec.Inputs(7).Max = deg2rad(5);

opspec.Inputs(8).u = seed.deltaR;
opspec.Inputs(8).Known = true;

% State order in Brown_6DOF_Plant_EUL:
% 1 eul [phi theta psi]
% 2 position [N E D]
% 3 omega [p q r]
% 4 velocity [u v w]
opspec.States(1).x = [deg2rad(trimSpec.bankDeg); seed.thetaRad; 0];
opspec.States(1).Known = [true; false; true];
opspec.States(1).SteadyState = [true; true; true];
opspec.States(1).Min = [deg2rad(trimSpec.bankDeg); deg2rad(-2); 0];
opspec.States(1).Max = [deg2rad(trimSpec.bankDeg); deg2rad(8); 0];

opspec.States(2).x = [0; 0; trimSpec.altitudeNED];
opspec.States(2).Known = [false; false; false];
opspec.States(2).SteadyState = [false; false; false];

opspec.States(3).x = [0; 0; 0];
opspec.States(3).Known = [true; true; true];
opspec.States(3).SteadyState = [true; true; true];

opspec.States(4).x = [ ...
    trimSpec.cruiseSpeedMps * cos(seed.alphaRad); ...
    0; ...
    trimSpec.cruiseSpeedMps * sin(seed.alphaRad)];
opspec.States(4).Known = [false; true; false];
opspec.States(4).SteadyState = [true; true; true];
opspec.States(4).Min = [trimSpec.cruiseSpeedMps - 5; 0; 0];
opspec.States(4).Max = [trimSpec.cruiseSpeedMps + 5; 0; 6];

opspec.Outputs(8).y = trimSpec.cruiseSpeedMps;
opspec.Outputs(8).Known = true;
opspec.Outputs(8).Min = trimSpec.cruiseSpeedMps - 0.5;
opspec.Outputs(8).Max = trimSpec.cruiseSpeedMps + 0.5;

opOpts = findopOptions('DisplayReport', char(trimSpec.displayReport));
[op, opreport] = findop(model, opspec, opOpts); %#ok<ASGLU>

trimPoint = struct();
trimPoint.model = model;
trimPoint.cruise_speed_mps = trimSpec.cruiseSpeedMps;
trimPoint.position = op.States(2).x(:);
trimPoint.eul = op.States(1).x(:);
trimPoint.omega = op.States(3).x(:);
trimPoint.velocity = op.States(4).x(:);
trimPoint.vinf = norm(trimPoint.velocity);
trimPoint.inputs = struct();
trimPoint.inputs.motor_rpm_cmd = zeros(12, 1);
trimPoint.inputs.tilt_angles_cmd = localExpandTiltGrouped(trimSpec.tiltAnglesGrouped);
trimPoint.inputs.front_collective_rpm = op.Inputs(3).u;
trimPoint.inputs.rear_collective_rpm = op.Inputs(4).u;
trimPoint.inputs.delta_f = op.Inputs(5).u;
trimPoint.inputs.delta_a = op.Inputs(6).u;
trimPoint.inputs.delta_e = op.Inputs(7).u;
trimPoint.inputs.delta_r = op.Inputs(8).u;
trimPoint.report = opreport;
trimPoint.seed = seed;
end

function seed = localBuildSeed(session, trimSpec)
seed = struct( ...
    'alphaRad', deg2rad(2.63), ...
    'thetaRad', deg2rad(2.63), ...
    'frontCollectiveRpm', 1181.43, ...
    'rearCollectiveRpm', 0.0, ...
    'deltaF', 0.0, ...
    'deltaA', 0.0, ...
    'deltaE', deg2rad(-15.0602653566512), ...
    'deltaR', 0.0);

try
    savedSeed = load_saved_trim_main(struct('repoRoot', session.repoRoot), trimSpec);
    seed.alphaRad = deg2rad(savedSeed.alpha_trim_deg);
    seed.thetaRad = deg2rad(savedSeed.pitch_trim_deg);
    seed.frontCollectiveRpm = savedSeed.front_collective_rpm;
    seed.rearCollectiveRpm = savedSeed.rear_collective_rpm;
    seed.deltaE = deg2rad(savedSeed.delta_e_deg);
    seed.deltaA = deg2rad(savedSeed.delta_a_deg);
    seed.deltaR = deg2rad(savedSeed.delta_r_deg);
catch
    % Keep the literal fallback seed when no matching saved row exists.
end
end

function expanded = localExpandTiltGrouped(groupedTilt)
groupedTilt = groupedTilt(:);
if numel(groupedTilt) < 2
    groupedTilt = groupedTilt(1) * ones(2, 1);
end
expanded = [groupedTilt(1) * ones(3, 1); groupedTilt(2) * ones(3, 1)];
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

    blocks{end + 1} = blk; %#ok<AGROW>
    attrs{end + 1} = attr; %#ok<AGROW>
    set_param(blk, 'ContinuousStateAttributes', '');
end
end
