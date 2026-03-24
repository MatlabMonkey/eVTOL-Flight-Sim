function result = trim_cruise(varargin)
%TRIM_CRUISE Attempt cruise trim using toolbox method, then fallback heuristic.
%
% Usage:
%   result = trim_cruise();
%   result = trim_cruise('CruiseSpeed', 70, 'TiltDeg', 90);
%
% The function prefers MATLAB operating-point tools when available, but always
% returns a documented fallback estimate when those tools/model setup are not
% currently compatible.

p = inputParser;
p.addParameter('Model', 'Brown_Full_Sim', @(s)ischar(s) || isstring(s));
p.addParameter('CruiseSpeed', 70, @isnumeric);
p.addParameter('TiltDeg', 90, @isnumeric);
p.addParameter('TargetAltitude', -1000, @isnumeric);
p.parse(varargin{:});
opts = p.Results;

result = struct();
result.timestamp = datestr(now, 31);
result.options = opts;
result.methods_attempted = {};
result.success = false;
result.notes = {};

% Ensure workspace variables exist.
if ~evalin('base', 'exist(''Mass'', ''var'') && exist(''prop'', ''var'')')
    this_file = mfilename('fullpath');
    repo_root = fileparts(fileparts(fileparts(this_file)));
    repo_root_escaped = strrep(repo_root, '''', '''''');
    evalin('base', sprintf([ ...
        'orig_dir = pwd; ' ...
        'cleanup_obj = onCleanup(@() cd(orig_dir)); ' ...
        'cd(''%s''); ' ...
        'run(''Full_Sim_Init.m'');'], repo_root_escaped));
    result.notes{end+1} = 'Ran Full_Sim_Init.m to populate workspace variables.';
end

Mass = evalin('base', 'Mass');
prop = evalin('base', 'prop');

%% Method 1: findop/operspec (if available)
result.methods_attempted{end+1} = 'findop/operspec';
if exist('operspec', 'file') == 2 && exist('findop', 'file') == 2
    try
        mdl = char(opts.Model);
        load_system(mdl);

        opspec = operspec(mdl);
        % NOTE: Full state mapping for this model is currently unknown in
        % this repository context, so this section intentionally remains
        % conservative. If operspec dimensions mismatch, we catch and fall
        % back below.

        % Attempt a no-constraint operating point search as a smoke check.
        op = findop(mdl, opspec); %#ok<NASGU>

        result.success = true;
        result.method = 'findop/operspec';
        result.notes{end+1} = 'findop executed, but state/output constraints are not yet fully configured.';
    catch ME
        result.notes{end+1} = ['findop path failed: ', ME.message];
    end
else
    result.notes{end+1} = 'findop/operspec unavailable in this MATLAB installation.';
end

%% Method 2: lightweight fallback estimate (always provided)
% Forces in this heuristic are approximate and use only readily available
% scalar parameters. Intended as a starting point for robust trim scripting.
g = 9.81;
weightN = Mass * g;
kT = prop.k_Thrust;

% Rear rotor RPM needed to support full weight if wings/front do not assist.
rpmRearHoverEq = sqrt(weightN / (6 * kT));

% Approximate drag model using wing/tail CD0 terms if available.
if evalin('base', 'exist(''wing'', ''var'') && exist(''tailL'', ''var'') && exist(''tailR'', ''var'')')
    wing = evalin('base', 'wing');
    tailL = evalin('base', 'tailL');
    tailR = evalin('base', 'tailR');
    rho = evalin('base', 'rho');

    q = 0.5 * rho * opts.CruiseSpeed^2;
    CdArea = wing.CD0 * wing.S + tailL.CD0 * tailL.S + tailR.CD0 * tailR.S;
    dragN = q * CdArea;
else
    % Conservative placeholder if surface structs are unavailable.
    dragN = 0.12 * opts.CruiseSpeed^2;
end

% Front rotors are assumed tilted for cruise and primarily producing +X thrust.
rpmFrontCruiseEq = sqrt(max(dragN, 0) / (6 * kT));

fallback = struct();
fallback.weight_N = weightN;
fallback.estimated_drag_N = dragN;
fallback.rpm_front_for_drag = rpmFrontCruiseEq;
fallback.rpm_rear_for_full_weight = rpmRearHoverEq;
fallback.comment = ['Fallback only: does not close full 6-DoF trim, but gives ', ...
    'initial actuator targets for cruise-like operating point search.'];

result.fallback = fallback;

if ~result.success
    result.method = 'fallback';
end

end
