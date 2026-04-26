function summary = quick_simout_check(sim_data)
%QUICK_SIMOUT_CHECK Print a compact end-of-run sanity summary.
%
% Usage:
%   quick_simout_check
%   quick_simout_check(simOut)
%
% This helper looks for the key Wrapper outputs:
%   - V_B_truth
%   - eul_truth
%   - omega_truth
%   - specific_force_truth
%
% It accepts either:
%   - a Simulink.SimulationOutput
%   - a plain struct containing those fields
%   - no input, in which case it looks for simOut in the base workspace

if nargin < 1 || isempty(sim_data)
    if evalin('base', 'exist(''simOut'', ''var'')')
        sim_data = evalin('base', 'simOut');
    else
        error('quick_simout_check:MissingSimOut', ...
            'No input provided and no "simOut" variable was found in the base workspace.');
    end
end

summary = struct();
summary.V_B_truth = localReadFinalVector(sim_data, 'V_B_truth');
summary.eul_truth_rad = localReadFinalVector(sim_data, 'eul_truth');
summary.eul_truth_deg = rad2deg(summary.eul_truth_rad(:)).';
summary.omega_truth = localReadFinalVector(sim_data, 'omega_truth');
summary.specific_force_truth = localReadFinalVector(sim_data, 'specific_force_truth');

fprintf('\n=== simOut end-value check ===\n');
fprintf('V_B_truth [m/s]           = [% .4f  % .4f  % .4f]\n', summary.V_B_truth);
fprintf('eul_truth [deg]           = [% .4f  % .4f  % .4f]\n', summary.eul_truth_deg);
fprintf('omega_truth [rad/s]       = [% .4f  % .4f  % .4f]\n', summary.omega_truth);
fprintf('specific_force_truth [m/s^2] = [% .4f  % .4f  % .4f]\n', summary.specific_force_truth);

summary.V_mag = norm(summary.V_B_truth);
summary.omega_mag = norm(summary.omega_truth);
summary.specific_force_mag = norm(summary.specific_force_truth);

fprintf('speed magnitude [m/s]     = %.4f\n', summary.V_mag);
fprintf('rate magnitude [rad/s]    = %.4f\n', summary.omega_mag);
fprintf('specific force mag [m/s^2]= %.4f\n', summary.specific_force_mag);
end

function vec = localReadFinalVector(sim_data, signal_name)
value = localResolveSignal(sim_data, signal_name);

if isstruct(value) && isfield(value, 'signals') && isfield(value.signals, 'values')
    data = value.signals.values;
elseif isa(value, 'timeseries')
    data = value.Data;
else
    error('quick_simout_check:UnsupportedSignalType', ...
        'Signal "%s" is of unsupported class %s.', signal_name, class(value));
end

data = squeeze(data);
if isempty(data)
    error('quick_simout_check:EmptySignal', 'Signal "%s" is empty.', signal_name);
end

vec = localExtractFinal3Vector(data);
if isempty(vec)
    error('quick_simout_check:BadVectorSize', ...
        'Signal "%s" final sample has shape %s and could not be reduced to a 3-vector.', ...
        signal_name, mat2str(size(data)));
end
end

function value = localResolveSignal(sim_data, signal_name)
if isa(sim_data, 'Simulink.SimulationOutput')
    try
        value = sim_data.get(signal_name);
        return
    catch
    end
end

if isstruct(sim_data) && isfield(sim_data, signal_name)
    value = sim_data.(signal_name);
    return
end

error('quick_simout_check:SignalNotFound', ...
    'Could not find signal "%s" in the supplied sim data.', signal_name);
end

function vec = localExtractFinal3Vector(data)
vec = [];

if isvector(data)
    data = double(data(:));
    if numel(data) == 3
        vec = data(:).';
    end
    return
end

data = double(data);
sz = size(data);

if numel(sz) ~= 2
    data = reshape(data, sz(1), []);
    sz = size(data);
end

if sz(2) == 3
    vec = data(end, :);
    return
end

if sz(1) == 3
    vec = data(:, end).';
    return
end

if sz(2) == 1 && sz(1) == 3
    vec = data(:, 1).';
    return
end

if sz(1) == 1 && sz(2) == 3
    vec = data(1, :);
    return
end
end
