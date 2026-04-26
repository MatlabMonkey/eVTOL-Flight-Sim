function trimResult = trim_result_from_controller_db_point(point)
%REBUILD_TRIM_RESULT_FROM_CONTROLLER_DB_POINT
% Reconstruct a run-ready trimResult from one controller_schedule point.

trimCase = localGetField(point, 'trimCase', struct());

Att_Trim_deg = localCanonicalizeLength(localGetField(point, 'Att_Trim_deg', [0; 0; 0]), 3);
Att_Trim = deg2rad(Att_Trim_deg(:));
Vel_B_BA_Trim = localCanonicalizeLength(localGetField(point, 'Vel_B_BA_Trim', [0; 0; 0]), 3);
Pos_Trim = localCanonicalizeLength(localGetField(trimCase, 'pos_init', zeros(3, 1)), 3);
Rates_Trim = localCanonicalizeLength(localGetField(point, 'x0', zeros(9, 1)), 9);
Rates_Trim = Rates_Trim(7:9);

vinf = localGetField(point, 'vinf_mps', norm(Vel_B_BA_Trim));
alphaRad = deg2rad(localGetField(point, 'alpha_deg', localAlphaFromBodyVelocity(Vel_B_BA_Trim)));
betaRad = localBetaFromBodyVelocity(Vel_B_BA_Trim);
Vel_W_Trim = [vinf; alphaRad; betaRad];

motorTrim = localCanonicalizeLength(localGetField(trimCase, 'motor_rpm_cmd', zeros(4, 1)), 4);
frontTiltDeg = localGetField(point, 'tilt_deg', localGetField(trimCase, 'front_tilt_deg', 90.0));
tiltTrim = repmat(frontTiltDeg, 2, 1);
trimCmdRad = localCanonicalizeLength(localGetField(point, 'trim_cmd_rad', zeros(6, 1)), 6);
frontCollective = localGetField(point, 'front_collective_rpm', trimCmdRad(1));
rearCollective = localGetField(point, 'rear_collective_rpm', trimCmdRad(2));
mixedControlTrim = trimCmdRad(3:6);

surfaceInitDeg = localCanonicalizeLength(localGetField(trimCase, 'surface_init_deg', zeros(4, 1)), 4);
directSurfaceTrim = deg2rad(surfaceInitDeg);
mixMatrix = localGetMixMatrix(trimCase);
mixedControlTrim = localResolveMixedControlTrim(point, trimCase, mixedControlTrim);
physicalSurfaceTrim = directSurfaceTrim + mixMatrix * mixedControlTrim;

trimResult = struct();
trimResult.name = char(string(localGetField(point, 'name', 'ControllerDbPoint')));
trimResult.mode = char(string(localGetField(trimCase, 'mode', 'cruise')));
trimResult.modelName = char(string(localGetField(trimCase, 'modelName', 'Trim_Plant')));
trimResult.success = strcmpi(string(localGetField(point, 'classification', "")), "exact_trim");
trimResult.isExactTrim = trimResult.success;
trimResult.terminationString = 'Loaded from controller schedule DB point.';
trimResult.case = trimCase;
trimResult.front_tilt_deg = frontTiltDeg;

trimResult.Att_Trim = Att_Trim;
trimResult.Att_Trim_deg = Att_Trim_deg(:);
trimResult.Vel_B_BA_Trim = Vel_B_BA_Trim;
trimResult.Vel_W_Trim = Vel_W_Trim;
trimResult.Rates_Trim = Rates_Trim;
trimResult.Pos_Trim = Pos_Trim;
trimResult.X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim];
trimResult.Act_Trim = physicalSurfaceTrim;
trimResult.U_trim = [frontCollective; mixedControlTrim];
trimResult.U_surface_trim = [frontCollective; physicalSurfaceTrim];
trimResult.U_trim_full = [motorTrim; tiltTrim; frontCollective; rearCollective; physicalSurfaceTrim];
trimResult.trimPlantInputs = [motorTrim; tiltTrim; frontCollective; rearCollective; directSurfaceTrim; mixedControlTrim];

trimResult.trim = struct();
trimResult.trim.Att_Trim = Att_Trim;
trimResult.trim.Att_Trim_deg = Att_Trim_deg(:);
trimResult.trim.Vel_B_BA_Trim = Vel_B_BA_Trim;
trimResult.trim.Vel_W_Trim = Vel_W_Trim;
trimResult.trim.Rates_Trim = Rates_Trim;
trimResult.trim.Pos_Trim = Pos_Trim;
trimResult.trim.Act_Trim = physicalSurfaceTrim;
trimResult.trim.X_trim = trimResult.X_trim;
trimResult.trim.U_trim = trimResult.U_trim;
trimResult.trim.U_surface_trim = trimResult.U_surface_trim;
trimResult.trim.U_trim_full = trimResult.U_trim_full;
trimResult.trim.direct_surface_trim = directSurfaceTrim;
trimResult.trim.mixed_control_trim = mixedControlTrim;
trimResult.trim.physical_surface_trim = physicalSurfaceTrim;
trimResult.trim.trimPlantInputs = trimResult.trimPlantInputs;

trimResult.linear = struct();
trimResult.linear.reduced_model_available = true;
trimResult.linear.sys_full = [];
trimResult.linear.A_full = localGetField(point, 'A_full', []);
trimResult.linear.B_full = localGetField(point, 'B_full', []);
trimResult.linear.C_full = localGetField(point, 'C_full', []);
trimResult.linear.D_full = localGetField(point, 'D_full', []);
trimResult.linear.sys_ss_13state = [];
trimResult.linear.B_front_collective = [];
trimResult.linear.B_rear_collective = [];
trimResult.linear.sys_ss_9state = ss(point.A_9, point.B_9, point.C_9, point.D_9);

if isfield(point, 'state_names_9') && ~isempty(point.state_names_9)
    trimResult.linear.sys_ss_9state.StateName = cellstr(point.state_names_9(:));
end
if isfield(point, 'input_names_9') && ~isempty(point.input_names_9)
    trimResult.linear.sys_ss_9state.InputName = cellstr(point.input_names_9(:));
end
if isfield(point, 'output_names_9') && ~isempty(point.output_names_9)
    trimResult.linear.sys_ss_9state.OutputName = cellstr(point.output_names_9(:));
end

trimResult.trimSpec = localBuildMinimalTrimSpec(trimCase, trimResult.modelName);
trimResult.scheduling = struct();
trimResult.scheduling.Vinf_mps = vinf;
trimResult.scheduling.alpha_rad = alphaRad;
trimResult.scheduling.beta_rad = betaRad;
trimResult.scheduling.phi_rad = Att_Trim(1);
trimResult.scheduling.theta_rad = Att_Trim(2);
trimResult.scheduling.psi_rad = Att_Trim(3);
trimResult.scheduling.front_tilt_deg = frontTiltDeg;
trimResult.scheduling.front_collective_rpm = frontCollective;
trimResult.scheduling.rear_collective_rpm = rearCollective;
trimResult.scheduling.delta_f_rad = mixedControlTrim(1);
trimResult.scheduling.delta_a_rad = mixedControlTrim(2);
trimResult.scheduling.delta_e_rad = mixedControlTrim(3);
trimResult.scheduling.delta_r_rad = mixedControlTrim(4);
end

function trimSpec = localBuildMinimalTrimSpec(trimCase, modelName)
surfaceLimitDeg = localCanonicalizeLength(localGetField(trimCase, 'surface_limit_deg', 25.0), 4);
trimSpec = struct();
trimSpec.name = localGetField(trimCase, 'name', 'ControllerDbPoint');
trimSpec.mode = localGetField(trimCase, 'mode', 'cruise');
trimSpec.modelName = modelName;
trimSpec.surface_limit_deg = surfaceLimitDeg;
trimSpec.surface_limit_deg_scalar = max(surfaceLimitDeg);
trimSpec.surface_limit_rad = deg2rad(surfaceLimitDeg);
trimSpec.surface_limit_rad_scalar = max(trimSpec.surface_limit_rad);
end

function mixMatrix = localGetMixMatrix(trimCase)
mixMatrix = [1 1 0 0; ...
             1 -1 0 0; ...
             0 0 1 -1; ...
             0 0 1 1];
if isstruct(trimCase) && isfield(trimCase, 'aircraft') && isstruct(trimCase.aircraft) && ...
        isfield(trimCase.aircraft, 'MixMatrix')
    candidate = trimCase.aircraft.MixMatrix;
    if isequal(size(candidate), [4 4])
        mixMatrix = candidate;
    end
end
end

function value = localCanonicalizeLength(value, n)
value = value(:);
if isempty(value)
    value = zeros(n, 1);
elseif numel(value) < n
    value(end + 1:n, 1) = 0;
elseif numel(value) > n
    value = value(1:n);
end
end

function mixedControlTrim = localResolveMixedControlTrim(point, trimCase, mixedControlTrim)
fieldNames = {'delta_f_deg', 'delta_a_deg', 'delta_e_deg', 'delta_r_deg'};
trimCaseDefaults = localTrimCaseMixedDefaults(trimCase);

for i = 1:4
    current = mixedControlTrim(i);
    if isfinite(current)
        continue;
    end

    degValue = localGetField(point, fieldNames{i}, NaN);
    if ~isfinite(degValue)
        degValue = trimCaseDefaults(i);
    end
    if ~isfinite(degValue)
        degValue = 0.0;
    end
    mixedControlTrim(i) = deg2rad(degValue);
end
end

function defaultsDeg = localTrimCaseMixedDefaults(trimCase)
defaultsDeg = zeros(4, 1);
defaultsDeg(1) = localGetFiniteField(trimCase, {'delta_f_fixed_deg', 'delta_f_guess_deg'}, 0.0);
defaultsDeg(2) = localGetFiniteField(trimCase, {'delta_a_fixed_deg', 'delta_a_guess_deg'}, 0.0);
defaultsDeg(3) = localGetFiniteField(trimCase, {'delta_e_fixed_deg', 'delta_e_guess_deg'}, 0.0);
defaultsDeg(4) = localGetFiniteField(trimCase, {'delta_r_fixed_deg', 'delta_r_guess_deg'}, 0.0);
end

function value = localGetFiniteField(s, fieldNames, fallback)
value = fallback;
if ~isstruct(s)
    return;
end
for i = 1:numel(fieldNames)
    name = fieldNames{i};
    if isfield(s, name) && ~isempty(s.(name)) && isfinite(s.(name))
        value = s.(name);
        return;
    end
end
end

function angleDeg = localAlphaFromBodyVelocity(velBody)
u = velBody(1);
w = velBody(3);
angleDeg = rad2deg(atan2(w, max(u, 1e-6)));
end

function betaRad = localBetaFromBodyVelocity(velBody)
vinf = norm(velBody);
if vinf <= 0.1
    betaRad = 0.0;
else
    betaArg = max(min(velBody(2) / vinf, 1.0), -1.0);
    betaRad = asin(betaArg);
end
end

function value = localGetField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end
