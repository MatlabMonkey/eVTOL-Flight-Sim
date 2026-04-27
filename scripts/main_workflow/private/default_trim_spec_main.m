function trimSpec = default_trim_spec_main(trimSpec)
%DEFAULT_TRIM_SPEC_MAIN Fill in missing trim-spec fields with readable defaults.

if nargin < 1 || isempty(trimSpec)
    trimSpec = struct();
end

trimSpec = localSetDefault(trimSpec, 'source', 'saved');
trimSpec = localSetDefault(trimSpec, 'cruiseSpeedMps', 70);
trimSpec = localSetDefault(trimSpec, 'bankDeg', 0);
trimSpec = localSetDefault(trimSpec, 'tiltAnglesGrouped', [90; 90]);
trimSpec = localSetDefault(trimSpec, 'altitudeNED', -1000);
trimSpec = localSetDefault(trimSpec, 'displayReport', 'off');
trimSpec = localSetDefault(trimSpec, 'providedTrim', struct());
trimSpec = localSetDefault(trimSpec, 'useAvlAero', false);
end

function s = localSetDefault(s, fieldName, defaultValue)
if ~isfield(s, fieldName) || isempty(s.(fieldName))
    s.(fieldName) = defaultValue;
end
end
