function avlAero = load_avl_aero_model(repoRoot)
%LOAD_AVL_AERO_MODEL Build a compact AVL-based aircraft aero model.
%   avlAero = LOAD_AVL_AERO_MODEL(repoRoot) reads the existing AVL homework
%   study CSV outputs and returns a compact, fixed-size struct for the
%   top-level Simulink AVL aero branch.

if nargin < 1 || isempty(repoRoot)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        repoRoot = fileparts(stack(1).file);
    else
        repoRoot = pwd;
    end
end

tableRoot = fullfile(repoRoot, 'docs', 'avl_homework', 'tables');
requiredTables = { ...
    'alpha_sweep.csv', ...
    'beta_sweep.csv', ...
    'flap_sweep.csv', ...
    'aileron_sweep.csv', ...
    'elevator_sweep.csv', ...
    'rudder_sweep.csv', ...
    'trim_cases.csv'};

for idx = 1:numel(requiredTables)
    tablePath = fullfile(tableRoot, requiredTables{idx});
    if exist(tablePath, 'file') ~= 2
        error('load_avl_aero_model:MissingTable', ...
            'Missing AVL study table: %s', tablePath);
    end
end

alphaTbl = readtable(fullfile(tableRoot, 'alpha_sweep.csv'));
betaTbl = readtable(fullfile(tableRoot, 'beta_sweep.csv'));
flapTbl = readtable(fullfile(tableRoot, 'flap_sweep.csv'));
aileronTbl = readtable(fullfile(tableRoot, 'aileron_sweep.csv'));
elevatorTbl = readtable(fullfile(tableRoot, 'elevator_sweep.csv'));
rudderTbl = readtable(fullfile(tableRoot, 'rudder_sweep.csv'));

avlAero = struct();

% Reference geometry matches the current simplified AVL study.
avlAero.Sref = 18.75;
avlAero.Bref = 12.50;
avlAero.Cref = 1.50;

% Valid fit envelope is the exact range used in the AVL sweeps.
avlAero.alpha_limits_deg = [min(alphaTbl.alpha_deg), max(alphaTbl.alpha_deg)];
avlAero.beta_limits_deg = [min(betaTbl.beta_deg), max(betaTbl.beta_deg)];
avlAero.control_limits_deg = [min(flapTbl.flap_deg), max(flapTbl.flap_deg)];
avlAero.min_airspeed_mps = 0.1;

avlAero.alpha0_deg = 4.0;
avlAero.beta0_deg = 0.0;

% Baseline zero-control fits.
avlAero.fit.alpha.CL = polyfit(alphaTbl.alpha_deg, alphaTbl.CL_avl, 1);
avlAero.fit.alpha.CD = polyfit(alphaTbl.alpha_deg, alphaTbl.CD_avl, 2);
avlAero.fit.alpha.Cm = polyfit(alphaTbl.alpha_deg, alphaTbl.Cm_avl, 1);

avlAero.fit.beta.CY = fitThroughOrigin(betaTbl.beta_deg, betaTbl.CY_avl);
avlAero.fit.beta.Cl = fitThroughOrigin(betaTbl.beta_deg, betaTbl.Cl_avl);
avlAero.fit.beta.Cn = fitThroughOrigin(betaTbl.beta_deg, betaTbl.Cn_avl);

% Control increments are fit relative to the zero-deflection AVL baseline
% at alpha = 4 deg, beta = 0 deg. Inputs below stay in degrees.
avlAero.fit.flap.dCL = fitThroughOrigin(flapTbl.flap_deg, ...
    incrementFromZero(flapTbl.flap_deg, flapTbl.CL_avl));
avlAero.fit.flap.dCD = fitEvenQuadratic(flapTbl.flap_deg, ...
    incrementFromZero(flapTbl.flap_deg, flapTbl.CD_avl));
avlAero.fit.flap.dCm = fitThroughOrigin(flapTbl.flap_deg, ...
    incrementFromZero(flapTbl.flap_deg, flapTbl.Cm_avl));

avlAero.fit.aileron.dCY = fitThroughOrigin(aileronTbl.aileron_deg, ...
    incrementFromZero(aileronTbl.aileron_deg, aileronTbl.CY_avl));
avlAero.fit.aileron.dCl = fitThroughOrigin(aileronTbl.aileron_deg, ...
    incrementFromZero(aileronTbl.aileron_deg, aileronTbl.Cl_avl));
avlAero.fit.aileron.dCn = fitThroughOrigin(aileronTbl.aileron_deg, ...
    incrementFromZero(aileronTbl.aileron_deg, aileronTbl.Cn_avl));
avlAero.fit.aileron.dCD = fitEvenQuadratic(aileronTbl.aileron_deg, ...
    incrementFromZero(aileronTbl.aileron_deg, aileronTbl.CD_avl));

avlAero.fit.elevator.dCL = fitThroughOrigin(elevatorTbl.elevator_deg, ...
    incrementFromZero(elevatorTbl.elevator_deg, elevatorTbl.CL_avl));
avlAero.fit.elevator.dCD = fitEvenQuadratic(elevatorTbl.elevator_deg, ...
    incrementFromZero(elevatorTbl.elevator_deg, elevatorTbl.CD_avl));
avlAero.fit.elevator.dCm = fitThroughOrigin(elevatorTbl.elevator_deg, ...
    incrementFromZero(elevatorTbl.elevator_deg, elevatorTbl.Cm_avl));

avlAero.fit.rudder.dCY = fitThroughOrigin(rudderTbl.rudder_deg, ...
    incrementFromZero(rudderTbl.rudder_deg, rudderTbl.CY_avl));
avlAero.fit.rudder.dCl = fitThroughOrigin(rudderTbl.rudder_deg, ...
    incrementFromZero(rudderTbl.rudder_deg, rudderTbl.Cl_avl));
avlAero.fit.rudder.dCn = fitThroughOrigin(rudderTbl.rudder_deg, ...
    incrementFromZero(rudderTbl.rudder_deg, rudderTbl.Cn_avl));
avlAero.fit.rudder.dCD = fitEvenQuadratic(rudderTbl.rudder_deg, ...
    incrementFromZero(rudderTbl.rudder_deg, rudderTbl.CD_avl));

end

function slope = fitThroughOrigin(x, y)
x = x(:);
y = y(:);
den = x' * x;
if den < eps
    slope = 0.0;
else
    slope = (x' * y) / den;
end
end

function coeff = fitEvenQuadratic(x, y)
x2 = x(:) .^ 2;
y = y(:);
den = x2' * x2;
if den < eps
    coeff = 0.0;
else
    coeff = (x2' * y) / den;
end
end

function delta = incrementFromZero(x, y)
x = x(:);
y = y(:);
zeroIdx = find(abs(x) < 1.0e-9, 1, 'first');
if isempty(zeroIdx)
    error('load_avl_aero_model:MissingZeroRow', ...
        'Expected a zero-deflection row in AVL study table.');
end
delta = y(:) - y(zeroIdx);
end
