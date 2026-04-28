function result = Run_INDI_PerturbationRejection_Test(pathIndex, perturbation, opts)
%RUN_INDI_PERTURBATIONREJECTION_TEST Run INDI verification test 3.
%
% This uses the same single fixed trim point and fixed INDI schedule as
% Run_INDI_FrozenTrimHold_Test, but starts Wrapper slightly away from trim.
% The command/reference stays fixed at the unperturbed trim point.
%
% Usage:
%   result = Run_INDI_PerturbationRejection_Test
%   result = Run_INDI_PerturbationRejection_Test([], struct('u_mps', 2.0))
%   result = Run_INDI_PerturbationRejection_Test([], struct('theta_deg', 3.0))
%   result = Run_INDI_PerturbationRejection_Test(10, 'w')
%
% Named perturbation presets:
%   'u'      +2 m/s body-axis u offset
%   'w'      +0.75 m/s body-axis w offset
%   'theta'  +3 deg pitch offset
%   'q'      +2 deg/s pitch-rate offset
%   'all'    combined small offsets in u, w, theta, and q

if nargin < 1
    pathIndex = [];
end
if nargin < 2 || isempty(perturbation)
    perturbation = struct('u_mps', 2.0);
end
if nargin < 3 || isempty(opts)
    opts = struct();
end

perturbation = localNormalizePerturbation(perturbation);
opts.initial_condition_offset = perturbation;

result = Run_INDI_FrozenTrimHold_Test(pathIndex, opts);
assignin('base', 'indiPerturbationRejectionResult', result);
end

function perturbation = localNormalizePerturbation(perturbation)
if ischar(perturbation) || isstring(perturbation)
    name = lower(strtrim(char(perturbation)));
    switch name
        case 'u'
            perturbation = struct('u_mps', 2.0);
        case 'w'
            perturbation = struct('w_mps', 0.75);
        case 'theta'
            perturbation = struct('theta_deg', 3.0);
        case 'q'
            perturbation = struct('q_deg_s', 2.0);
        case {'all', 'combined'}
            perturbation = struct( ...
                'u_mps', 2.0, ...
                'w_mps', 0.5, ...
                'theta_deg', 2.0, ...
                'q_deg_s', 1.0);
        otherwise
            error('Run_INDI_PerturbationRejection_Test:BadPreset', ...
                'Unknown perturbation preset "%s". Use u, w, theta, q, or all.', name);
    end
elseif ~isstruct(perturbation)
    error('Run_INDI_PerturbationRejection_Test:BadPerturbation', ...
        'perturbation must be a struct or one of the presets: u, w, theta, q.');
end
end
