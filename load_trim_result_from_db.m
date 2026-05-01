function [trimResult, entry, meta] = load_trim_result_from_db(dbInput, selector, initData)
% load trim from DB

if nargin < 1 || isempty(dbInput)
    dbInput = localResolveDefaultDb();
end
if nargin < 2 || isempty(selector)
    selector = struct('group', 'hover', 'name', 'Hover');
end
if nargin < 3
    initData = struct();
end

[db, dbFile] = localLoadDb(dbInput);
entry = localSelectEntry(db, selector);
trimResult = localRebuildTrimResult(entry, initData);

meta = struct();
meta.db_file = dbFile;
meta.group = string(entry.group);
meta.name = string(entry.name);
meta.key = string(entry.key);
meta.source_map_label = string(localGetField(entry, 'source_map_label', ""));
end

function trimResult = localRebuildTrimResult(entry, initData)
trimCase = localGetField(entry, 'trim_case', struct());
trimSummary = localGetField(entry, 'trim_summary', struct());
linearData = localGetField(entry, 'linear', struct());

mixMatrix = localGetMixMatrix(initData);
posInit = localGetField(trimCase, 'pos_init', localGetNestedField(initData, {'initialState', 'pos_init'}, zeros(3, 1)));
posInit = posInit(:);

phiRad = deg2rad(localGetField(trimCase, 'phi_deg', localGetField(trimCase, 'bank_deg', 0.0)));
thetaRad = deg2rad(localGetField(trimSummary, 'theta_deg', localGetField(trimCase, 'theta_guess_deg', 0.0)));
psiRad = deg2rad(localGetField(trimCase, 'psi_deg', 0.0));
Att_Trim = [phiRad; thetaRad; psiRad];
Att_Trim_deg = rad2deg(Att_Trim);

uBody = localGetField(trimSummary, 'u_mps', localGetField(trimCase, 'u_body_mps', localGetField(trimCase, 'Vinf_mps', 0.0)));
vBody = localGetField(trimCase, 'v_body_mps', 0.0);
wBody = localGetField(trimSummary, 'w_mps', localGetField(trimCase, 'w_body_mps', 0.0));
Vel_B_BA_Trim = [uBody; vBody; wBody];

ratesDefault = localGetField(trimCase, 'body_rates_rad_s', zeros(3, 1));
Rates_Trim = localCanonicalizeLength(ratesDefault, 3);

vinf = localGetField(trimCase, 'Vinf_mps', norm(Vel_B_BA_Trim));
if ~isfinite(vinf) || vinf <= 0
    vinf = norm(Vel_B_BA_Trim);
end
if ~isfinite(vinf)
    vinf = 0.0;
end

alphaRad = deg2rad(localGetField(trimSummary, 'alpha_deg', localAlphaFromBodyVelocity(Vel_B_BA_Trim)));
betaRad = localBetaFromBodyVelocity(Vel_B_BA_Trim);
Vel_W_Trim = [vinf; alphaRad; betaRad];

motorTrim = localCanonicalizeLength(localGetField(trimCase, 'motor_rpm_cmd', zeros(4, 1)), 4);
tiltDeg = localGetField(trimSummary, 'front_tilt_deg', localGetField(trimCase, 'front_tilt_deg', 90.0));
tiltTrim = repmat(tiltDeg, 2, 1);
frontCollective = localGetField(trimSummary, 'front_collective_rpm', localGetField(trimCase, 'front_collective_guess_rpm', 0.0));
rearCollective = localGetField(trimSummary, 'rear_collective_rpm', localGetField(trimCase, 'rear_collective_guess_rpm', 0.0));

mixedControlTrim = deg2rad(localCanonicalizeLength([ ...
    localGetField(trimSummary, 'delta_f_deg', 0.0); ...
    localGetField(trimSummary, 'delta_a_deg', 0.0); ...
    localGetField(trimSummary, 'delta_e_deg', 0.0); ...
    localGetField(trimSummary, 'delta_r_deg', 0.0)], 4));

surfaceInitDeg = localCanonicalizeLength(localGetField(trimCase, 'surface_init_deg', zeros(4, 1)), 4);
directSurfaceTrim = deg2rad(surfaceInitDeg);
physicalSurfaceTrim = directSurfaceTrim + mixMatrix * mixedControlTrim;

Act_Trim = physicalSurfaceTrim;
Pos_Trim = posInit;
X_trim = [Pos_Trim; Vel_B_BA_Trim; Att_Trim; Rates_Trim];
U_trim = [frontCollective; mixedControlTrim];
U_surface_trim = [frontCollective; physicalSurfaceTrim];
U_trim_full = [motorTrim; tiltTrim; frontCollective; rearCollective; physicalSurfaceTrim];
trimPlantInputs = [motorTrim; tiltTrim; frontCollective; rearCollective; directSurfaceTrim; mixedControlTrim];

trimResult = struct();
trimResult.name = localGetField(entry, 'name', localGetField(trimCase, 'name', 'DBTrimPoint'));
trimResult.mode = localGetField(trimCase, 'mode', localGetField(trimSummary, 'mode', 'db_reconstructed'));
trimResult.modelName = localGetField(trimCase, 'modelName', localGetNestedField(initData, {'modelNames', 'trim'}, 'Trim_Plant'));
trimResult.success = logical(localGetField(linearData, 'replay_exact_trim', localGetField(entry, 'replay_success', localGetField(trimSummary, 'success', false))));
trimResult.isExactTrim = trimResult.success;
trimResult.terminationString = localGetField(entry, 'termination_string', localGetField(trimSummary, 'termination_string', 'Loaded from trim DB.'));
trimResult.case = trimCase;
trimResult.trimSpec = localBuildMinimalTrimSpec(trimCase, trimResult.modelName);
trimResult.db_source = struct('group', entry.group, 'name', entry.name, 'key', entry.key);

trimResult.Att_Trim = Att_Trim;
trimResult.Att_Trim_deg = Att_Trim_deg;
trimResult.Vel_B_BA_Trim = Vel_B_BA_Trim;
trimResult.Vel_W_Trim = Vel_W_Trim;
trimResult.Rates_Trim = Rates_Trim;
trimResult.Pos_Trim = Pos_Trim;
trimResult.Act_Trim = Act_Trim;
trimResult.X_trim = X_trim;
trimResult.U_trim = U_trim;
trimResult.U_surface_trim = U_surface_trim;
trimResult.U_trim_full = U_trim_full;
trimResult.trimPlantInputs = trimPlantInputs;
trimResult.front_tilt_deg = tiltDeg;

trimResult.trim = struct();
trimResult.trim.Att_Trim = Att_Trim;
trimResult.trim.Att_Trim_deg = Att_Trim_deg;
trimResult.trim.Vel_B_BA_Trim = Vel_B_BA_Trim;
trimResult.trim.Vel_W_Trim = Vel_W_Trim;
trimResult.trim.Rates_Trim = Rates_Trim;
trimResult.trim.Pos_Trim = Pos_Trim;
trimResult.trim.Act_Trim = Act_Trim;
trimResult.trim.X_trim = X_trim;
trimResult.trim.U_trim = U_trim;
trimResult.trim.U_surface_trim = U_surface_trim;
trimResult.trim.U_trim_full = U_trim_full;
trimResult.trim.direct_surface_trim = directSurfaceTrim;
trimResult.trim.mixed_control_trim = mixedControlTrim;
trimResult.trim.physical_surface_trim = physicalSurfaceTrim;
trimResult.trim.trimPlantInputs = trimPlantInputs;

trimResult.linear = localBuildLinearStruct(linearData);

trimResult.scheduling = struct();
trimResult.scheduling.Vinf_mps = Vel_W_Trim(1);
trimResult.scheduling.alpha_rad = Vel_W_Trim(2);
trimResult.scheduling.beta_rad = Vel_W_Trim(3);
trimResult.scheduling.phi_rad = Att_Trim(1);
trimResult.scheduling.theta_rad = Att_Trim(2);
trimResult.scheduling.psi_rad = Att_Trim(3);
trimResult.scheduling.front_tilt_deg = tiltDeg;
trimResult.scheduling.front_collective_rpm = frontCollective;
trimResult.scheduling.rear_collective_rpm = rearCollective;
trimResult.scheduling.delta_f_rad = mixedControlTrim(1);
trimResult.scheduling.delta_a_rad = mixedControlTrim(2);
trimResult.scheduling.delta_e_rad = mixedControlTrim(3);
trimResult.scheduling.delta_r_rad = mixedControlTrim(4);

trimResult.opspec = [];
trimResult.op_trim = [];
trimResult.op_report = struct('TerminationString', trimResult.terminationString);
end

function linear = localBuildLinearStruct(linearData)
linear = struct();
linear.reduced_model_available = logical(localGetField(linearData, 'reduced_model_available', false));
linear.sys_full = [];
linear.A_full = [];
linear.B_full = [];
linear.C_full = [];
linear.D_full = [];
linear.sys_ss_13state = [];
linear.B_front_collective = localGetField(linearData, 'front_collective_column_9', []);
linear.B_rear_collective = localGetField(linearData, 'rear_collective_column_9', []);

if linear.reduced_model_available
    A9 = localGetField(linearData, 'A_9', []);
    B9 = localGetField(linearData, 'B_9', []);
    C9 = localGetField(linearData, 'C_9', []);
    D9 = localGetField(linearData, 'D_9', []);
    linear.sys_ss_9state = ss(A9, B9, C9, D9);

    stateNames = localGetField(linearData, 'state_names_9', {});
    inputNames = localGetField(linearData, 'input_names_9', {});
    if ~isempty(stateNames)
        linear.sys_ss_9state.StateName = cellstr(stateNames(:));
    end
    if ~isempty(inputNames)
        linear.sys_ss_9state.InputName = cellstr(inputNames(:));
    end
else
    linear.sys_ss_9state = [];
end
end

function trimSpec = localBuildMinimalTrimSpec(trimCase, modelName)
surfaceLimitDeg = localCanonicalizeLength(localGetField(trimCase, 'surface_limit_deg', 25.0), 4);
trimSpec = struct();
trimSpec.name = localGetField(trimCase, 'name', 'DBTrimPoint');
trimSpec.mode = localGetField(trimCase, 'mode', 'db_reconstructed');
trimSpec.modelName = modelName;
trimSpec.surface_limit_deg = surfaceLimitDeg;
trimSpec.surface_limit_deg_scalar = max(surfaceLimitDeg);
trimSpec.surface_limit_rad = deg2rad(surfaceLimitDeg);
trimSpec.surface_limit_rad_scalar = max(trimSpec.surface_limit_rad);
end

function entry = localSelectEntry(db, selector)
groupNames = {'hover', 'cruise'};

if isnumeric(selector) && isscalar(selector)
    [entry, found] = localFindByGlobalIndex(db, double(selector), groupNames);
    if ~found
        error('Requested DB entry index %d is out of range.', double(selector));
    end
    return;
end

if ischar(selector) || isstring(selector)
    selector = struct('name', char(string(selector)));
end
if ~isstruct(selector)
    error('selector must be empty, numeric, char/string, or a struct.');
end

matches = {};
for iGroup = 1:numel(groupNames)
    groupName = groupNames{iGroup};
    if isfield(selector, 'group') && ~isempty(selector.group) && ~strcmpi(string(selector.group), groupName)
        continue;
    end
    if ~isfield(db, groupName) || ~isfield(db.(groupName), 'entries')
        continue;
    end

    entries = db.(groupName).entries(:);
    for i = 1:numel(entries)
        if localEntryMatches(entries(i), selector)
            matches{end + 1, 1} = entries(i); %#ok<AGROW>
        end
    end
end

if isempty(matches) && isfield(selector, 'name') && ~isempty(selector.name)
    fallbackSelector = rmfield(selector, intersect(fieldnames(selector), {'name'}));
    fallbackSelector.key = selector.name;
    for iGroup = 1:numel(groupNames)
        groupName = groupNames{iGroup};
        if isfield(fallbackSelector, 'group') && ~isempty(fallbackSelector.group) && ~strcmpi(string(fallbackSelector.group), groupName)
            continue;
        end
        if ~isfield(db, groupName) || ~isfield(db.(groupName), 'entries')
            continue;
        end

        entries = db.(groupName).entries(:);
        for i = 1:numel(entries)
            if localEntryMatches(entries(i), fallbackSelector)
                matches{end + 1, 1} = entries(i); %#ok<AGROW>
            end
        end
    end
end

if isempty(matches)
    error('Could not find a DB entry matching the requested selector.');
end

if isfield(selector, 'index') && ~isempty(selector.index)
    idx = double(selector.index);
    if idx < 1 || idx > numel(matches)
        error('Requested selector.index %d is out of range after filtering.', idx);
    end
    entry = matches{idx};
    return;
end

if numel(matches) > 1
    labels = strings(numel(matches), 1);
    for i = 1:numel(matches)
        candidate = matches{i};
        labels(i) = string(localGetField(candidate, 'group', "")) + ":" + string(localGetField(candidate, 'name', ""));
    end
    error('Selector matched multiple DB entries: %s', strjoin(cellstr(labels), ', '));
end

entry = matches{1};
end

function [db, dbFile] = localLoadDb(dbInput)
dbFile = char(string(dbInput));
if exist(dbFile, 'file') ~= 2
    error('DB file not found: %s', dbFile);
end
data = load(dbFile);
if isfield(data, 'trimLinearizationDB')
    db = data.trimLinearizationDB;
elseif isfield(data, 'trimLinearizationDb')
    db = data.trimLinearizationDb;
else
    error('Could not find trimLinearizationDB in %s.', dbFile);
end
end

function dbFile = localResolveDefaultDb()
stack = dbstack('-completenames');
if ~isempty(stack)
    rootDir = fileparts(stack(1).file);
else
    rootDir = pwd;
end

dbPaths = TrimDB_Paths(rootDir);
preferred = fullfile(dbPaths.database_dir, 'trim_linearization_db_latest.mat');
if exist(preferred, 'file') == 2
    dbFile = preferred;
    return;
end

fallback = fullfile(dbPaths.database_dir, 'trim_linearization_db.mat');
if exist(fallback, 'file') == 2
    dbFile = fallback;
    return;
end

dirInfo = dir(fullfile(dbPaths.database_dir, 'trim_linearization_db*'));
dirInfo = dirInfo([dirInfo.isdir]);
bestDatenum = -inf;
dbFile = '';
for i = 1:numel(dirInfo)
    candidate = fullfile(dirInfo(i).folder, dirInfo(i).name, 'trim_linearization_db.mat');
    if exist(candidate, 'file') ~= 2
        continue;
    end
    if dirInfo(i).datenum > bestDatenum
        bestDatenum = dirInfo(i).datenum;
        dbFile = candidate;
    end
end

if isempty(dbFile)
    error(['Could not find a trim linearization DB. Run Build_Trim_Linearization_DB ', ...
           'first, restore it under databases/, or pass the DB file path explicitly.']);
end
end

function mixMatrix = localGetMixMatrix(initData)
mixMatrix = [1  1  0  0; ...
             1 -1  0  0; ...
             0  0  1 -1; ...
             0  0  1  1];
if isstruct(initData) && isfield(initData, 'aircraft') && isstruct(initData.aircraft) && ...
        isfield(initData.aircraft, 'MixMatrix')
    candidate = initData.aircraft.MixMatrix;
    if isequal(size(candidate), [4 4])
        mixMatrix = candidate;
    end
end
end

function angleDeg = localAlphaFromBodyVelocity(velBody)
u = velBody(1);
w = velBody(3);
angleDeg = rad2deg(atan2(w, u));
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

function value = localGetNestedField(s, fieldPath, fallback)
value = fallback;
cursor = s;
for i = 1:numel(fieldPath)
    fieldName = fieldPath{i};
    if ~isstruct(cursor) || ~isfield(cursor, fieldName)
        return;
    end
    cursor = cursor.(fieldName);
end
value = cursor;
end

function value = localGetField(s, fieldName, fallback)
value = fallback;
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
end
end

function vec = localCanonicalizeLength(value, n)
vec = zeros(n, 1);
if isempty(value)
    return;
end
value = value(:);
count = min(numel(value), n);
vec(1:count) = value(1:count);
if count == 1 && n > 1
    vec(:) = value(1);
end
end

function entry = localEmptyEntry()
entry = struct('group', "", 'name', "", 'key', "");
end

function tf = localEntryMatches(entry, selector)
tf = true;
if isfield(selector, 'name') && ~isempty(selector.name)
    tf = tf && strcmp(string(localGetField(entry, 'name', "")), string(selector.name));
end
if isfield(selector, 'key') && ~isempty(selector.key)
    tf = tf && strcmp(string(localGetField(entry, 'key', "")), string(selector.key));
end
end

function [entry, found] = localFindByGlobalIndex(db, globalIndex, groupNames)
entry = localEmptyEntry();
found = false;
cursor = 0;
for iGroup = 1:numel(groupNames)
    groupName = groupNames{iGroup};
    if ~isfield(db, groupName) || ~isfield(db.(groupName), 'entries')
        continue;
    end
    entries = db.(groupName).entries(:);
    for i = 1:numel(entries)
        cursor = cursor + 1;
        if cursor == globalIndex
            entry = entries(i);
            found = true;
            return;
        end
    end
end
end
