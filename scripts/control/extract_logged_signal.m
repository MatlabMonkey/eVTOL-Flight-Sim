function [t, y] = extract_logged_signal(simOut, varName, varargin)
%EXTRACT_LOGGED_SIGNAL Convert Simulink To Workspace data into NxM arrays.

p = inputParser;
p.addParameter('SampleTime', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
p.parse(varargin{:});

t = simOut.get('tout');
t = t(:);
raw = simOut.get(varName);

if isnumeric(raw)
    y = formatLoggedArray(raw);
else
    error('extract_logged_signal:UnsupportedType', ...
        'Unsupported logged type for %s: %s', varName, class(raw));
end

if size(y, 1) ~= numel(t) && size(y, 2) == numel(t)
    y = y.';
end

if size(y, 1) ~= numel(t)
    if ~isempty(p.Results.SampleTime)
        t = (0:size(y, 1) - 1).' * p.Results.SampleTime;
    elseif numel(t) >= 2
        t = linspace(t(1), t(end), size(y, 1)).';
    else
        t = (0:size(y, 1) - 1).';
    end
end
end

function y = formatLoggedArray(raw)
raw = double(raw);
if isvector(raw)
    y = raw(:);
    return;
end

if ndims(raw) == 3 && size(raw, 2) == 1
    y = squeeze(raw).';
    return;
end

if ndims(raw) == 3
    y = reshape(raw, size(raw, 1) * size(raw, 2), size(raw, 3)).';
    return;
end

if ismatrix(raw)
    y = raw;
    return;
end

error('extract_logged_signal:BadShape', ...
    'Unsupported logged array shape.');
end
