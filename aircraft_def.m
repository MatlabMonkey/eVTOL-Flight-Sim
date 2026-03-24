function aircraft = aircraft_def(varargin)
%AIRCRAFT_DEF Define the Brown eVTOL aircraft and derive model structs.

% These literal defaults are intentionally easy to parse from tooling.
flight_mode = 0;
rho = 1.225;
g = -9.81; % Gravity sign convention used by the Simulink model

if nargin == 1 && isnumeric(varargin{1}) && isscalar(varargin{1})
    flight_mode = varargin{1};
elseif nargin ~= 0
    if mod(nargin, 2) ~= 0
        error('aircraft_def:BadInputs', ...
            'Use aircraft_def() or aircraft_def(''flight_mode'', value).');
    end

    for idx = 1:2:nargin
        name = varargin{idx};
        value = varargin{idx + 1};

        if ~(ischar(name) || (isstring(name) && isscalar(name)))
            error('aircraft_def:BadName', 'Parameter names must be text scalars.');
        end

        switch lower(char(name))
            case {'flight_mode', 'flightmode'}
                flight_mode = value;
            case {'rho', 'air_density'}
                rho = value;
            case {'g', 'gravity'}
                g = value;
            otherwise
                error('aircraft_def:UnknownParameter', ...
                    'Unknown parameter ''%s''.', char(name));
        end
    end
end

tilt_angle = 90 * flight_mode;

% --- Propeller Struct Constants ---
prop.hub_offset = 0.50;  % Standoff length from the pivot axis to the prop hub
prop.k_Thrust   = 1.2e-4;
prop.k_Torque   = 1.5e-5;  % Front masks
prop.k_torque   = 1.5e-5;  % Rear masks (matching lowercase 't')
prop.Lspin_dir  = [-1; 1; -1];
prop.Rspin_dir  = [1; -1; 1];

% =========================================================================
% COMPONENT DATA
% F-Rotor positions are defined as the PIVOT AXIS [1.95, y, -0.45]
% R-Rotor positions are the STATIC HUB positions [-2.20, y, -0.65]
% =========================================================================
compData = {
    % Body & Ballast
    % Fuselage mass intentionally lumps cabin, battery, systems, and landing gear.
    'Fuselage',   'fuselage', 1736, [7.50, 2.00, 1.60], [ 0.00,  0.00,  0.00], [0, 0, 0];
    'Ballast',    'box',      180,  [0.60, 0.60, 0.60], [ 3.00,  0.00,  0.00], [0, 0, 0];

    % Wings & Tails
    % Half-wing centers at y = +/-3.75 m preserve a small wing-root embed into the fuselage.
    'L Main Wing','box',      140,  [1.50,  6.25, 0.20], [-0.50, -3.75, -0.60], [0, 0, 0];
    'R Main Wing','box',      140,  [1.50,  6.25, 0.20], [-0.50,  3.75, -0.60], [0, 0, 0];
    'L V-Tail',   'box',      50,   [1.40,  2.00, 0.08], [-3.50, -0.71, -0.91], [ 45, 0, 0];
    'R V-Tail',   'box',      50,   [1.40,  2.00, 0.08], [-3.50,  0.71, -0.91], [-45, 0, 0];

    % Booms (ends at X = 2.05)
    'Boom L1',    'box',      55,   [4.50, 0.20, 0.20], [-0.20, -1.80, -0.45], [0, 0, 0];
    'Boom L2',    'box',      55,   [4.50, 0.20, 0.20], [-0.20, -4.20, -0.45], [0, 0, 0];
    'Boom L3',    'box',      55,   [4.50, 0.20, 0.20], [-0.20, -6.50, -0.45], [0, 0, 0];
    'Boom R1',    'box',      55,   [4.50, 0.20, 0.20], [-0.20,  1.80, -0.45], [0, 0, 0];
    'Boom R2',    'box',      55,   [4.50, 0.20, 0.20], [-0.20,  4.20, -0.45], [0, 0, 0];
    'Boom R3',    'box',      55,   [4.50, 0.20, 0.20], [-0.20,  6.50, -0.45], [0, 0, 0];

    % Front Arms (visual/inertia reference only; the block handles thrust shift)
    'F-Arm L1',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15, -1.80, -0.55], [0, tilt_angle, 0];
    'F-Arm L2',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15, -4.20, -0.55], [0, tilt_angle, 0];
    'F-Arm L3',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15, -6.50, -0.55], [0, tilt_angle, 0];
    'F-Arm R1',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15,  1.80, -0.55], [0, tilt_angle, 0];
    'F-Arm R2',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15,  4.20, -0.55], [0, tilt_angle, 0];
    'F-Arm R3',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15,  6.50, -0.55], [0, tilt_angle, 0];

    % Front Props (positions set to pivot axis: X = 1.95, Z = -0.45)
    'F-Rotor L1', 'crossprop',32,   [1.60, 0.15, 0.05], [ 1.95, -1.80, -0.45], [0, tilt_angle, 0];
    'F-Rotor L2', 'crossprop',32,   [1.60, 0.15, 0.05], [ 1.95, -4.20, -0.45], [0, tilt_angle, 0];
    'F-Rotor L3', 'crossprop',32,   [1.60, 0.15, 0.05], [ 1.95, -6.50, -0.45], [0, tilt_angle, 0];
    'F-Rotor R1', 'crossprop',32,   [1.60, 0.15, 0.05], [ 1.95,  1.80, -0.45], [0, tilt_angle, 0];
    'F-Rotor R2', 'crossprop',32,   [1.60, 0.15, 0.05], [ 1.95,  4.20, -0.45], [0, tilt_angle, 0];
    'F-Rotor R3', 'crossprop',32,   [1.60, 0.15, 0.05], [ 1.95,  6.50, -0.45], [0, tilt_angle, 0];

    % Rear Props (static hub positions)
    'R-Rotor L1', 'crossprop',32,   [1.60, 0.15, 0.05], [-2.20, -1.80, -0.65], [0, 0, 0];
    'R-Rotor L2', 'crossprop',32,   [1.60, 0.15, 0.05], [-2.20, -4.20, -0.65], [0, 0, 0];
    'R-Rotor L3', 'crossprop',32,   [1.60, 0.15, 0.05], [-2.20, -6.50, -0.65], [0, 0, 0];
    'R-Rotor R1', 'crossprop',32,   [1.60, 0.15, 0.05], [-2.20,  1.80, -0.65], [0, 0, 0];
    'R-Rotor R2', 'crossprop',32,   [1.60, 0.15, 0.05], [-2.20,  4.20, -0.65], [0, 0, 0];
    'R-Rotor R3', 'crossprop',32,   [1.60, 0.15, 0.05], [-2.20,  6.50, -0.65], [0, 0, 0];
};
N = size(compData, 1);

% Aero Properties
% Angle convention:
% - The aeroSurface block computes alpha in radians.
% - Therefore all angle quantities stored here must also use radians.
% - In practice that means i and a0 are radians, CLa and CMa are per rad,
%   and CDa is per rad^2 because it multiplies (alpha - a0)^2.
% - If you copy values from papers or plots reported in degrees, convert
%   them before entering them here.
% {name, normal_vec, CL0, e, i_rad, CD0, CDa_per_rad2, a0_rad, CM0, CMa_per_rad, CLa_per_rad}
aeroData = {
    % Rounded from NASA TM-20210017971 Table 6 and the finite-wing lift-slope/induced-drag formulas used by NDARC.
    'L Main Wing', [0; 0; -1],               0.35, 0.90, 0.00, 0.0085, 0.0424, -0.0733, -0.10, 0.00, 4.7671;
    'R Main Wing', [0; 0; -1],               0.35, 0.90, 0.00, 0.0085, 0.0424, -0.0733, -0.10, 0.00, 4.7671;
    'L V-Tail',  [0; sind(45); -cosd(45)],   0.00, 0.70, 0.00, 0.0200, 0.3183,  0.0000,  0.00, 0.00, 2.2028;
    'R V-Tail',  [0; -sind(45); -cosd(45)],  0.00, 0.70, 0.00, 0.0200, 0.3183,  0.0000,  0.00, 0.00, 2.2028;
};

% --- Mass, CG, and inertia ---
Mtot = 0;
CG_row = [0, 0, 0];
for i = 1:N
    m = compData{i, 3};
    pos = compData{i, 5};
    Mtot = Mtot + m;
    CG_row = CG_row + m * pos;
end
CG_row = CG_row / Mtot;

Mass = Mtot;
CG = CG_row';

I_total = zeros(3, 3);
for i = 1:N
    type = compData{i, 2};
    m = compData{i, 3};
    dim = compData{i, 4};
    pos = compData{i, 5};
    eul = compData{i, 6} * (pi / 180);

    if strcmp(type, 'crossprop')
        D = dim(1);
        W = dim(2);
        T = dim(3);
        Ixx1 = (1 / 12) * (m / 2) * (W^2 + T^2);
        Iyy1 = (1 / 12) * (m / 2) * (D^2 + T^2);
        Izz1 = (1 / 12) * (m / 2) * (D^2 + W^2);
        Ixx2 = (1 / 12) * (m / 2) * (D^2 + T^2);
        Iyy2 = (1 / 12) * (m / 2) * (W^2 + T^2);
        Izz2 = (1 / 12) * (m / 2) * (D^2 + W^2);
        I_local = diag([Ixx1 + Ixx2, Iyy1 + Iyy2, Izz1 + Izz2]);
    else
        xL = dim(1);
        yL = abs(dim(2));
        zL = dim(3);
        I_local = diag([
            (1 / 12) * m * (yL^2 + zL^2), ...
            (1 / 12) * m * (xL^2 + zL^2), ...
            (1 / 12) * m * (xL^2 + yL^2)]);
    end

    phi = eul(1);
    theta = eul(2);
    psi = eul(3);
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = Rz * Ry * Rx;

    I_rot = R * I_local * R';
    d = (pos - CG_row)';
    I_total = I_total + I_rot + m * (((d' * d) * eye(3)) - (d * d'));
end

J = I_total;

% --- Build aero structs ---
wingL = localBuildSurfaceStruct(compData, aeroData, 'L Main Wing', rho);
wingR = localBuildSurfaceStruct(compData, aeroData, 'R Main Wing', rho);
wing = localCombineWingHalves(wingL, wingR, rho);

tailL = localBuildSurfaceStruct(compData, aeroData, 'L V-Tail', rho);
tailR = localBuildSurfaceStruct(compData, aeroData, 'R V-Tail', rho);

% Archer Midnight uses wing flaperons and V-tail ruddervators, but the
% current aeroSurface block only accepts a static surface struct and has no
% live control-deflection input. Store control geometry/effectiveness data
% here now so the next Simulink patch has a single source of truth.
controls = localBuildControlSurfaceData(wing, wingL, wingR, tailL, tailR, CG);
wingL = localApplyControlFields(wingL, controls.flaperon, -0.35, 0.30);
wingR = localApplyControlFields(wingR, controls.flaperon, -0.35, 0.30);
tailL = localApplyControlFields(tailL, controls.ruddervator, 0.00, 0.10);
tailR = localApplyControlFields(tailR, controls.ruddervator, 0.00, 0.10);
wing = localCombineWingHalves(wingL, wingR, rho);

% --- Build propeller position matrices ---
idx_FR = find(contains(compData(:, 1), 'F-Rotor R'));
idx_FL = find(contains(compData(:, 1), 'F-Rotor L'));
idx_RR = find(contains(compData(:, 1), 'R-Rotor R'));
idx_RL = find(contains(compData(:, 1), 'R-Rotor L'));

prop.posFR = cell2mat(compData(idx_FR, 5));
prop.posFL = cell2mat(compData(idx_FL, 5));
prop.posRR = cell2mat(compData(idx_RR, 5));
prop.posRL = cell2mat(compData(idx_RL, 5));

aircraft = struct();
aircraft.flight_mode = flight_mode;
aircraft.tilt_angle = tilt_angle;
aircraft.rho = rho;
aircraft.g = g;
aircraft.prop = prop;
aircraft.compData = compData;
aircraft.aeroData = aeroData;
aircraft.Mass = Mass;
aircraft.CG = CG;
aircraft.J = J;
aircraft.wing = wing;
aircraft.wingL = wingL;
aircraft.wingR = wingR;
aircraft.tailL = tailL;
aircraft.tailR = tailR;
aircraft.controls = controls;
aircraft.render_surfaces = {wingL, wingR, tailL, tailR};
end

function surface = localBuildSurfaceStruct(compData, aeroData, surface_name, rho)
comp_idx = find(strcmp(compData(:, 1), surface_name), 1, 'first');
aero_idx = find(strcmp(aeroData(:, 1), surface_name), 1, 'first');

if isempty(comp_idx) || isempty(aero_idx)
    error('aircraft_def:MissingSurface', ...
        'Could not find matching compData/aeroData rows for ''%s''.', surface_name);
end

surface = struct();
surface.name = char(surface_name);
surface.c = compData{comp_idx, 4}(1);
surface.b = compData{comp_idx, 4}(2);
surface.S = surface.b * surface.c;
surface.half_rho_S = 0.5 * rho * surface.S;
surface.pos = (compData{comp_idx, 5})';
surface.n = aeroData{aero_idx, 2};
surface.CL0 = aeroData{aero_idx, 3};
surface.e = aeroData{aero_idx, 4};
surface.i = aeroData{aero_idx, 5};
surface.CD0 = aeroData{aero_idx, 6};
surface.CDa = aeroData{aero_idx, 7};
surface.a0 = aeroData{aero_idx, 8};
surface.CM0 = aeroData{aero_idx, 9};
surface.CMa = aeroData{aero_idx, 10};
surface.CLa = aeroData{aero_idx, 11};
surface.AR = surface.b / surface.c;
end

function wing = localCombineWingHalves(wingL, wingR, rho)
wing = wingL;
wing.name = 'Main Wing';
wing.b = wingL.b + wingR.b;
wing.S = wingL.S + wingR.S;
wing.half_rho_S = 0.5 * rho * wing.S;
wing.pos = ((wingL.S * wingL.pos) + (wingR.S * wingR.pos)) / wing.S;
wing.AR = wing.b / wing.c;
end

function surface = localApplyControlFields(surface, control_meta, CM_delta, CD_delta2)
surface.ctrl_tau = control_meta.effectiveness_tau;
surface.CM_delta = CM_delta;
surface.CD_delta2 = CD_delta2;
surface.delta_max = control_meta.max_deflection_rad;
end

function controls = localBuildControlSurfaceData(wing, wingL, wingR, tailL, tailR, CG)
controls = struct();
controls.units = struct( ...
    'angle', 'rad', ...
    'deflection', 'rad', ...
    'derivative', 'per rad');

controls.archer_midnight = struct( ...
    'has_vtail_ruddervators', true, ...
    'has_wing_flaperons', true, ...
    'sim_patch_status', 'metadata_and_visualization_only');

% Simplified V-tail model:
% - One ruddervator per tail half for this sim, even though Archer patents
%   show multiple segmented ruddervators per side.
% - Positive local deflection means trailing-edge down.
% - Standard mixed inputs follow fixed-wing convention:
%     delta_e > 0 => pitch-down command
%     delta_r > 0 => yaw-right command
%   with left/right local deflections:
%     delta_L = delta_e - delta_r
%     delta_R = delta_e + delta_r
vtail_half_angle = atan2(abs(tailL.n(2)), abs(tailL.n(3)));
vertical_proj = abs(tailL.n(3)) / norm(tailL.n);
lateral_proj = abs(tailL.n(2)) / norm(tailL.n);

ruddervator = struct();
ruddervator.surface_names = {'L V-Tail', 'R V-Tail'};
ruddervator.count_in_this_sim = 2;
ruddervator.count_in_archer_patent = 6;
ruddervator.control_chord_fraction = 0.30;
ruddervator.control_span_fraction = 1.00;
ruddervator.effectiveness_tau = 0.45;
ruddervator.max_deflection_rad = deg2rad(25);
ruddervator.vtail_half_angle_rad = vtail_half_angle;
ruddervator.local_deflection_sign = 'positive = trailing-edge down';
ruddervator.standard_command_sign = ...
    'delta_e positive = pitch-down, delta_r positive = yaw-right';
ruddervator.mix_from_standard = [1, -1; 1, 1];
ruddervator.mix_from_pilot = [-1, -1; -1, 1];
ruddervator.mix_labels = {'left tail row', 'right tail row'};
ruddervator.mix_input_labels_standard = {'delta_e', 'delta_r'};
ruddervator.mix_input_labels_pilot = {'pitch_cmd_nose_up', 'yaw_cmd_nose_right'};
ruddervator.current_block_hook = ...
    'Use i_eff = i0 + tau * delta_local as the first-pass Simulink implementation.';

local_CL_delta = tailL.CLa * ruddervator.effectiveness_tau;
tail_x = mean([tailL.pos(1), tailR.pos(1)]) - CG(1);
tail_z = mean([tailL.pos(3), tailR.pos(3)]) - CG(3);

ruddervator.expected = struct();
ruddervator.expected.local_tail_CL_delta_per_rad = local_CL_delta;
ruddervator.expected.CZ_delta_e_per_rad = ...
    -2 * (tailL.S / wing.S) * local_CL_delta * vertical_proj;
ruddervator.expected.CM_delta_e_per_rad = ...
    (-tail_x / wing.c) * ruddervator.expected.CZ_delta_e_per_rad;
ruddervator.expected.CY_delta_r_per_rad = ...
    -2 * (tailL.S / wing.S) * local_CL_delta * lateral_proj;
ruddervator.expected.CN_delta_r_per_rad = ...
    (tail_x / wing.b) * ruddervator.expected.CY_delta_r_per_rad;
ruddervator.expected.CL_delta_r_per_rad = ...
    -(tail_z / wing.b) * ruddervator.expected.CY_delta_r_per_rad;
ruddervator.expected.notes = { ...
    'Positive delta_e is standard fixed-wing trailing-edge-down elevator sign, so CM_delta_e should be negative.', ...
    'Positive delta_r is yaw-right rudder sign, so CN_delta_r should be positive.', ...
    'These derivatives are geometry-based first-pass estimates for the current simple aeroSurface model, not Archer-certified data.'};

flaperon = struct();
flaperon.count_in_archer_patent = 4;
flaperon.count_in_this_sim = 2;
flaperon.implemented_in_renderer = true;
flaperon.implemented_in_sim = false;
flaperon.control_functions = {'flap', 'aileron', 'spoiler'};
flaperon.surface_names = {'L Main Wing', 'R Main Wing'};
flaperon.control_chord_fraction = 0.28;
flaperon.control_span_fraction = 1.00;
flaperon.effectiveness_tau = 0.55;
flaperon.max_deflection_rad = deg2rad(25);
flaperon.local_deflection_sign = 'positive = trailing-edge down';
flaperon.standard_command_sign = ...
    'delta_f positive = flap-down, delta_a positive = roll-right';
flaperon.mix_from_standard = [1, 1; 1, -1];
flaperon.mix_labels = {'left wing row', 'right wing row'};
flaperon.mix_input_labels_standard = {'delta_f', 'delta_a'};
flaperon.current_block_hook = ...
    'Use i_eff = i0 + tau * delta_local for each wing half as the first-pass Simulink implementation.';
local_wing_CL_delta = wingL.CLa * flaperon.effectiveness_tau;
wing_y = wingR.pos(2);
flaperon.expected = struct();
flaperon.expected.local_wing_CL_delta_per_rad = local_wing_CL_delta;
flaperon.expected.CZ_delta_f_per_rad = ...
    -2 * (wingL.S / wing.S) * local_wing_CL_delta;
flaperon.expected.CL_delta_a_per_rad = ...
    2 * (wing_y / wing.b) * (wingL.S / wing.S) * local_wing_CL_delta;
flaperon.notes = { ...
    'Symmetric flap mode should primarily increase wing CL0 and CD0 and usually make CM0 more nose-down.', ...
    'Positive delta_a is defined here as a roll-right command, so the left flaperon goes trailing-edge down and the right flaperon goes trailing-edge up.', ...
    'This simplified renderer/scripting patch uses one flaperon per wing half to keep the control input count low.'};

controls.ruddervator = ruddervator;
controls.flaperon = flaperon;
end
