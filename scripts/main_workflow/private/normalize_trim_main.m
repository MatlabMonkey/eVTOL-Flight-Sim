function trimCtx = normalize_trim_main(rawTrim, trimSpec)
%NORMALIZE_TRIM_MAIN Convert any supported trim source into one schema.

trimSpec = default_trim_spec_main(trimSpec);

if isempty(rawTrim)
    error('normalize_trim_main:EmptyTrim', ...
        'The trim source returned an empty value.');
end

if isfield(rawTrim, 'speed_mps') && isfield(rawTrim, 'front_collective_rpm')
    trimCtx = localNormalizeSavedRow(rawTrim, trimSpec);
elseif isfield(rawTrim, 'inputs') && isfield(rawTrim, 'position')
    trimCtx = localNormalizeFreshTrim(rawTrim, trimSpec);
else
    trimCtx = localNormalizeProvided(rawTrim, trimSpec);
end
end

function trimCtx = localNormalizeSavedRow(rawTrim, trimSpec)
alpha = deg2rad(rawTrim.alpha_trim_deg);
beta = deg2rad(rawTrim.beta_trim_deg);

uBody = rawTrim.vinf_trim * cos(alpha) * cos(beta);
vBody = rawTrim.vinf_trim * sin(beta);
wBody = rawTrim.vinf_trim * sin(alpha) * cos(beta);

frontTiltDeg = localFieldOrDefault(rawTrim, 'front_tilt_deg_mean', ...
    mean(trimSpec.tiltAnglesGrouped));

trimCtx = struct();
trimCtx.pos = [0; 0; trimSpec.altitudeNED];
trimCtx.vel_body = [uBody; vBody; wBody];
trimCtx.euler = deg2rad([ ...
    rawTrim.roll_trim_deg; ...
    rawTrim.pitch_trim_deg; ...
    rawTrim.yaw_trim_deg]);
trimCtx.omega_body = [ ...
    localFieldOrDefault(rawTrim, 'p_trim', 0); ...
    localFieldOrDefault(rawTrim, 'q_trim', 0); ...
    localFieldOrDefault(rawTrim, 'r_trim', 0)];
trimCtx.vinf = rawTrim.vinf_trim;
trimCtx.collectives = struct( ...
    'front_rpm', rawTrim.front_collective_rpm, ...
    'rear_rpm', rawTrim.rear_collective_rpm);
trimCtx.motor_rpms_grouped = [ ...
    rawTrim.front_collective_rpm; ...
    rawTrim.front_collective_rpm; ...
    rawTrim.rear_collective_rpm; ...
    rawTrim.rear_collective_rpm];
trimCtx.tilt_angles_grouped = [frontTiltDeg; frontTiltDeg];
trimCtx.motor_rpm_cmd_grouped = zeros(4, 1);
trimCtx.surface_deflections = struct( ...
    'delta_f', deg2rad(rawTrim.delta_f_deg), ...
    'delta_a', deg2rad(rawTrim.delta_a_deg), ...
    'delta_e', deg2rad(rawTrim.delta_e_deg), ...
    'delta_r', deg2rad(rawTrim.delta_r_deg));
trimCtx.meta = struct( ...
    'source', 'saved', ...
    'case_name', localFieldOrDefault(rawTrim, 'case_name', 'saved_trim'), ...
    'raw', rawTrim);
end

function trimCtx = localNormalizeFreshTrim(rawTrim, trimSpec)
tiltGrouped = localNormalizeTiltGrouped(rawTrim.inputs.tilt_angles_cmd, trimSpec);
motorCmdGrouped = localCollapseMotorCommand(rawTrim.inputs.motor_rpm_cmd);

trimCtx = struct();
trimCtx.pos = rawTrim.position(:);
trimCtx.vel_body = rawTrim.velocity(:);
trimCtx.euler = rawTrim.eul(:);
trimCtx.omega_body = rawTrim.omega(:);
trimCtx.vinf = norm(trimCtx.vel_body);
trimCtx.collectives = struct( ...
    'front_rpm', rawTrim.inputs.front_collective_rpm, ...
    'rear_rpm', rawTrim.inputs.rear_collective_rpm);
trimCtx.motor_rpms_grouped = [ ...
    rawTrim.inputs.front_collective_rpm; ...
    rawTrim.inputs.front_collective_rpm; ...
    rawTrim.inputs.rear_collective_rpm; ...
    rawTrim.inputs.rear_collective_rpm];
trimCtx.tilt_angles_grouped = tiltGrouped;
trimCtx.motor_rpm_cmd_grouped = motorCmdGrouped;
trimCtx.surface_deflections = struct( ...
    'delta_f', rawTrim.inputs.delta_f, ...
    'delta_a', rawTrim.inputs.delta_a, ...
    'delta_e', rawTrim.inputs.delta_e, ...
    'delta_r', rawTrim.inputs.delta_r);
trimCtx.meta = struct( ...
    'source', 'fresh', ...
    'model', localFieldOrDefault(rawTrim, 'model', 'unknown'), ...
    'raw', rawTrim);
end

function trimCtx = localNormalizeProvided(rawTrim, trimSpec)
requiredFields = {'pos', 'vel_body', 'euler', 'omega_body', ...
    'motor_rpms_grouped', 'tilt_angles_grouped', 'surface_deflections'};
for i = 1:numel(requiredFields)
    if ~isfield(rawTrim, requiredFields{i})
        error('normalize_trim_main:MissingField', ...
            'Provided trim struct is missing ''%s''.', requiredFields{i});
    end
end

trimCtx = rawTrim;
trimCtx.pos = trimCtx.pos(:);
trimCtx.vel_body = trimCtx.vel_body(:);
trimCtx.euler = trimCtx.euler(:);
trimCtx.omega_body = trimCtx.omega_body(:);
trimCtx.motor_rpms_grouped = trimCtx.motor_rpms_grouped(:);
trimCtx.tilt_angles_grouped = localNormalizeTiltGrouped( ...
    trimCtx.tilt_angles_grouped, trimSpec);
trimCtx.vinf = norm(trimCtx.vel_body);

if ~isfield(trimCtx, 'collectives') || isempty(trimCtx.collectives)
    trimCtx.collectives = struct( ...
        'front_rpm', mean(trimCtx.motor_rpms_grouped(1:2)), ...
        'rear_rpm', mean(trimCtx.motor_rpms_grouped(3:4)));
end

if ~isfield(trimCtx, 'motor_rpm_cmd_grouped') || isempty(trimCtx.motor_rpm_cmd_grouped)
    trimCtx.motor_rpm_cmd_grouped = zeros(4, 1);
end
trimCtx.motor_rpm_cmd_grouped = trimCtx.motor_rpm_cmd_grouped(:);

if ~isfield(trimCtx, 'meta') || isempty(trimCtx.meta)
    trimCtx.meta = struct();
end
if ~isfield(trimCtx.meta, 'source')
    trimCtx.meta.source = 'provided';
end
end

function tiltGrouped = localNormalizeTiltGrouped(rawTilt, trimSpec)
rawTilt = rawTilt(:);
if isempty(rawTilt)
    tiltGrouped = trimSpec.tiltAnglesGrouped(:);
elseif numel(rawTilt) >= 6
    tiltGrouped = [mean(rawTilt(1:3)); mean(rawTilt(4:6))];
elseif numel(rawTilt) == 2
    tiltGrouped = rawTilt;
else
    tiltGrouped = rawTilt(1) * ones(2, 1);
end
end

function motorCmdGrouped = localCollapseMotorCommand(rawCmd)
rawCmd = rawCmd(:);
if isempty(rawCmd)
    motorCmdGrouped = zeros(4, 1);
elseif numel(rawCmd) >= 12
    motorCmdGrouped = [ ...
        mean(rawCmd(4:6)); ...
        mean(rawCmd(1:3)); ...
        mean(rawCmd(10:12)); ...
        mean(rawCmd(7:9))];
elseif numel(rawCmd) == 4
    motorCmdGrouped = rawCmd;
else
    motorCmdGrouped = rawCmd(1) * ones(4, 1);
end
end

function value = localFieldOrDefault(s, fieldName, defaultValue)
if ~isfield(s, fieldName) || isempty(s.(fieldName))
    value = defaultValue;
    return;
end

candidate = s.(fieldName);
if isnumeric(candidate) || islogical(candidate)
    if all(isfinite(candidate))
        value = candidate;
    else
        value = defaultValue;
    end
else
    value = candidate;
end
end
