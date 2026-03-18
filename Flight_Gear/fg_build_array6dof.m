function array6dof = fg_build_array6dof(positionLLA, eulerAngles)
%FG_BUILD_ARRAY6DOF Convert logged LLA and Euler outputs into Array6DoF.

[llaTime, llaData] = localExtractSeries(positionLLA, "positionLLA");
[eulTime, eulData] = localExtractSeries(eulerAngles, "eul");

localValidateColumns(llaData, "positionLLA");
localValidateColumns(eulData, "eul");

if numel(llaTime) ~= numel(eulTime) || any(abs(llaTime - eulTime) > 1e-9)
    if isa(eulerAngles, "timeseries")
        eulerAngles = resample(eulerAngles, llaTime);
        [eulTime, eulData] = localExtractSeries(eulerAngles, "eul");
    else
        error("fg_build_array6dof:TimeMismatch", ...
            ["positionLLA and eul do not share the same sample times. " ...
             "Replay currently expects timeseries inputs or matching time vectors."]);
    end
end

array6dof = [ ...
    llaTime(:), ...
    llaData(:, 1), ...
    llaData(:, 2), ...
    llaData(:, 3), ...
    eulData(:, 1), ...
    eulData(:, 2), ...
    eulData(:, 3)];
end

function [time, data] = localExtractSeries(value, signalName)
if isa(value, "timeseries")
    time = double(value.Time(:));
    data = localEnsureMatrix(value.Data);
    return;
end

if isstruct(value) && isfield(value, "time") && isfield(value, "signals")
    time = double(value.time(:));
    data = localEnsureMatrix(value.signals.values);
    return;
end

if isnumeric(value) && size(value, 2) >= 4
    time = double(value(:, 1));
    data = double(value(:, 2:4));
    return;
end

error("fg_build_array6dof:UnsupportedInput", ...
    "Unsupported logged signal type for %s.", signalName);
end

function matrix = localEnsureMatrix(raw)
matrix = double(raw);

if ~ismatrix(matrix)
    matrix = squeeze(matrix);
end

if size(matrix, 1) == 3 && size(matrix, 2) ~= 3
    matrix = matrix.';
end
end

function localValidateColumns(data, signalName)
if size(data, 2) ~= 3
    error("fg_build_array6dof:BadShape", ...
        "%s must contain exactly three columns.", signalName);
end
end
