function [pathSpec, pathPoints, pathTable] = load_transition_jumpgap_path_points()
%LOAD_TRANSITION_JUMPGAP_PATH_POINTS Resolve the selected jump-gap path points.
%
% The selected points now come directly from controller_schedule_db.mat so
% every path point is already controller-ready and has matched linearized
% data. transition_trim_master_attempt_db.mat is used only for background
% visualization in the plotting script.

pathSpec = get_transition_jumpgap_path_spec();
controllerDb = localLoadControllerDb(pathSpec.controller_db_file);

selectedNames = string(pathSpec.selected_point_names(:));
nPts = numel(selectedNames);
pathPoints = repmat(localPointTemplate(), nPts, 1);

for i = 1:nPts
    point = localLoadControllerDbPoint(controllerDb, selectedNames(i));
    point.path_index = i;
    point.path_tilt_deg = point.tilt_deg;
    point.path_vinf_mps = point.vinf_mps;
    point.jump_after = (i == pathSpec.jump_after_index);
    pathPoints(i) = point;
end

pathTable = struct2table(pathPoints);
end

function point = localPointTemplate()
point = struct( ...
    'path_index', 0, ...
    'name', "", ...
    'key', "", ...
    'source_kind', "", ...
    'source_file', "", ...
    'selector', "", ...
    'classification', "", ...
    'success', false, ...
    'acceptable', false, ...
    'trimResult', struct(), ...
    'trimResult_source', "", ...
    'front_collective_rpm', NaN, ...
    'rear_collective_rpm', NaN, ...
    'delta_f_deg', NaN, ...
    'delta_a_deg', NaN, ...
    'delta_e_deg', NaN, ...
    'delta_r_deg', NaN, ...
    'theta_deg', NaN, ...
    'u_mps', NaN, ...
    'w_mps', NaN, ...
    'tilt_deg', NaN, ...
    'vinf_mps', NaN, ...
    'path_tilt_deg', NaN, ...
    'path_vinf_mps', NaN, ...
    'jump_after', false);
end

function controllerDb = localLoadControllerDb(controllerDbFile)
if exist(controllerDbFile, 'file') ~= 2
    error('Controller schedule DB not found: %s', controllerDbFile);
end

data = load(controllerDbFile, 'controllerScheduleDB');
if ~isfield(data, 'controllerScheduleDB') || ~isstruct(data.controllerScheduleDB)
    error('Controller schedule DB file does not contain controllerScheduleDB.');
end

controllerDb = data.controllerScheduleDB;
if ~isfield(controllerDb, 'points') || isempty(controllerDb.points)
    error('controllerScheduleDB.points is missing or empty.');
end
end

function point = localLoadControllerDbPoint(controllerDb, name)
dbPoints = controllerDb.points(:);
names = string({dbPoints.name});
idx = find(names == string(name), 1, 'first');
if isempty(idx)
    error('Could not find controller DB point "%s".', string(name));
end

entry = dbPoints(idx);
trimResult = localRebuildTrimResultFromControllerPoint(entry);

point = localPointTemplate();
point.name = string(entry.name);
point.key = string(localGetField(entry, 'key', ""));
point.source_kind = "controller_schedule_db";
point.source_file = string(localGetField(controllerDb.meta, 'workspace_plots_dir', "")) + filesep + "controller_schedule_db.mat";
point.selector = string(entry.name);
point.classification = string(localGetField(entry, 'classification', ""));
point.success = strcmpi(point.classification, "exact_trim");
point.acceptable = point.success;
point.trimResult = trimResult;
point.trimResult_source = "controller_schedule_db_point";
point.front_collective_rpm = localGetField(entry, 'front_collective_rpm', NaN);
point.rear_collective_rpm = localGetField(entry, 'rear_collective_rpm', NaN);
point.delta_f_deg = localGetField(entry, 'delta_f_deg', NaN);
point.delta_a_deg = localGetField(entry, 'delta_a_deg', NaN);
point.delta_e_deg = localGetField(entry, 'delta_e_deg', NaN);
point.delta_r_deg = localGetField(entry, 'delta_r_deg', NaN);
point.theta_deg = localGetField(entry, 'theta_deg', NaN);
point.u_mps = localGetField(entry, 'u_mps', NaN);
point.w_mps = localGetField(entry, 'w_mps', NaN);
point.tilt_deg = localGetField(entry, 'tilt_deg', NaN);
point.vinf_mps = localGetField(entry, 'vinf_mps', NaN);
end

function trimResult = localRebuildTrimResultFromControllerPoint(point)
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
physicalSurfaceTrim = directSurfaceTrim + mixMatrix * mixedControlTrim;

trimResult = struct();
trimResult.name = char(string(localGetField(point, 'name', 'ControllerDbPoint')));
trimResult.mode = char(string(localGetField(trimCase, 'mode', 'cruise')));
trimResult.modelName = char(string(localGetField(trimCase, 'modelName', 'Trim_Plant')));
trimResult.success = true;
trimResult.isExactTrim = true;
trimResult.terminationString = 'Loaded from controller schedule DB.';
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
mixMatrix = [1  1  0  0; ...
             1 -1  0  0; ...
             0  0  1 -1; ...
             0  0  1  1];
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
    value = nan(n, 1);
elseif numel(value) < n
    value(end + 1:n, 1) = 0;
elseif numel(value) > n
    value = value(1:n);
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
    return;
end
betaArg = max(min(velBody(2) / vinf, 1.0), -1.0);
betaRad = asin(betaArg);
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
end
end
