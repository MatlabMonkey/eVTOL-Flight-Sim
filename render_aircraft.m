function hFig = render_aircraft(varargin)
%RENDER_AIRCRAFT Render the Brown eVTOL geometry and vector overlays.
%
% Usage:
%   render_aircraft()
%   render_aircraft('hover')
%   render_aircraft('cruise')
%   render_aircraft('transition')
%   render_aircraft('transition', 45)
%   render_aircraft('Stable_6DOF')
%   render_aircraft('cruise', 'delta_e_deg', 5, 'delta_r_deg', 3)
%   render_aircraft('cruise', 'delta_f_deg', 10, 'delta_a_deg', 4)
%   render_aircraft(compData, CG, tilt_angle)
%   render_aircraft(compData, CG, tilt_angle, ...
%       'surfaces', {wing, tailL, tailR}, ...
%       'prop', prop, ...
%       'controls', controls, ...
%       'thrust_tilt_deg', Tilt_angles)

if nargin == 0
    aircraft = aircraft_def();
    scenario = scenario_def();
    hFig = render_aircraft(aircraft.compData, aircraft.CG, scenario.visual_tilt_deg, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', aircraft.prop, ...
        'controls', aircraft.controls, ...
        'thrust_tilt_deg', scenario.Tilt_angles, ...
        'title_text', sprintf('%s (Scenario: %s)', ...
            'Brown eVTOL Aircraft', scenario.name));
    return;
end

if ischar(varargin{1}) || (isstring(varargin{1}) && isscalar(varargin{1}))
    [tilt_angle, thrust_tilt_deg, title_text, extra_args] = ...
        localResolveRenderPreset(varargin);
    aircraft = aircraft_def('flight_mode', 0);
    hFig = render_aircraft(aircraft.compData, aircraft.CG, tilt_angle, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', aircraft.prop, ...
        'controls', aircraft.controls, ...
        'thrust_tilt_deg', thrust_tilt_deg, ...
        'title_text', title_text, ...
        extra_args{:});
    return;
end

compData = varargin{1};
CG = varargin{2};
tilt_angle = varargin{3};

p = inputParser;
p.addParameter('surfaces', {}, @(x) isempty(x) || iscell(x) || isstruct(x));
p.addParameter('prop', struct(), @isstruct);
p.addParameter('controls', struct(), @isstruct);
p.addParameter('thrust_tilt_deg', [], @isnumeric);
p.addParameter('delta_e', 0, @isnumeric);
p.addParameter('delta_r', 0, @isnumeric);
p.addParameter('delta_e_deg', 0, @isnumeric);
p.addParameter('delta_r_deg', 0, @isnumeric);
p.addParameter('delta_f', 0, @isnumeric);
p.addParameter('delta_a', 0, @isnumeric);
p.addParameter('delta_f_deg', 0, @isnumeric);
p.addParameter('delta_a_deg', 0, @isnumeric);
p.addParameter('show_vectors', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_labels', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_input_markers', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('normal_scale', 1.4, @isnumeric);
p.addParameter('thrust_scale', 1.1, @isnumeric);
p.addParameter('label_font_scale', 0.72, @isnumeric);
p.addParameter('title_text', '', @(x) ischar(x) || isstring(x));
p.parse(varargin{4:end});
opts = p.Results;

surfaces = localNormalizeSurfaces(opts.surfaces);
[delta_e_rad, delta_r_rad, delta_f_rad, delta_a_rad] = localResolveControlInputs(opts);
ruddervator_state = localRuddervatorState(opts.controls, delta_e_rad, delta_r_rad);
flaperon_state = localFlaperonState(opts.controls, delta_f_rad, delta_a_rad);
hub_offset = localHubOffset(opts.prop);
standoff_length = hub_offset;
standoff_offset = 0.5 * standoff_length;
frontSpecs = localFrontRotorSpecs(opts.prop, opts.thrust_tilt_deg, tilt_angle);
rearSpecs = localRearRotorSpecs(opts.prop);
label_sizes = localLabelFontSizes(opts.label_font_scale);

hFig = figure('Name', 'VTOL Aircraft Render', ...
    'Color', [0.1 0.1 0.1], ...
    'Position', [50 50 1400 900]);
hold on;
grid on;
axis equal;
set(gca, 'ZDir', 'reverse', ...
    'Color', [0.15 0.15 0.15], ...
    'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title_input = opts.title_text;
if isstring(title_input)
    title_input = char(title_input);
end

if ~isempty(title_input)
    title_text = sprintf('%s\nVisual front tilt: %.1f deg', ...
        title_input, tilt_angle);
else
    title_text = sprintf('Aircraft Render (Visual front tilt: %.1f deg)', tilt_angle);
end
title(title_text, 'Color', 'w', 'FontSize', 14);

N = size(compData, 1);
control_surface_specs = struct('name', {}, 'pos', {}, 'dir', {}, 'deflection_deg', {}, 'kind', {});
for i = 1:N
    name = compData{i, 1};
    type = compData{i, 2};
    dim = compData{i, 4};
    pos = compData{i, 5};
    eul = compData{i, 6} * (pi / 180);
    rotor_tilt = localFrontComponentTilt(name, tilt_angle, opts.thrust_tilt_deg);

    % Shift front arms and rotors to their actual rendered tilt angle.
    if contains(name, 'F-Arm')
        pivot = localFrontPivot(name, pos, opts.prop);
        n_thrust = localThrustDirection(rotor_tilt);
        dim = [0.20, 0.20, standoff_length];
        pos = pivot + (standoff_offset * n_thrust);
        eul = [0, -rotor_tilt, 0] * (pi / 180);
    elseif contains(name, 'F-Rotor')
        pivot = localFrontPivot(name, pos, opts.prop);
        n_thrust = localThrustDirection(rotor_tilt);
        pos = pivot + (hub_offset * n_thrust);
        eul = [0, -rotor_tilt, 0] * (pi / 180);
    end

    color = localComponentColor(name, type);

    if strcmp(type, 'box') && contains(name, 'Main Wing')
        deflection_rad = localWingDeflection(name, flaperon_state);
        [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
            localBuildWingWithFlaperon(dim, flaperon_state.chord_fraction, deflection_rad);
        R = localRotationMatrix(eul);
        V_fixed_trans = (R * V_fixed')' + pos;
        V_ctrl_trans = (R * V_ctrl')' + pos;
        patch('Vertices', V_fixed_trans, 'Faces', F_fixed, ...
            'FaceColor', color, 'FaceAlpha', 0.85, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        patch('Vertices', V_ctrl_trans, 'Faces', F_ctrl, ...
            'FaceColor', [0.85 1.00 0.25], 'FaceAlpha', 0.92, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        control_surface_specs(end + 1) = ... %#ok<AGROW>
            localBuildControlSurfaceSpec(name, eul, V_ctrl_trans, deflection_rad, 'flaperon');
        if opts.show_labels
            label_pos = mean(V_ctrl_trans, 1);
            text(label_pos(1), label_pos(2), label_pos(3), ...
                sprintf(' %s flap %.1f deg', localShortWingName(name), rad2deg(deflection_rad)), ...
                'Color', [0.90 1.00 0.65], 'FontSize', label_sizes.control, 'FontWeight', 'bold');
        end
        continue;
    end

    if strcmp(type, 'box') && contains(name, 'V-Tail')
        deflection_rad = localTailDeflection(name, ruddervator_state);
        [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
            localBuildTailWithRuddervator(dim, ruddervator_state.chord_fraction, deflection_rad);
        R = localRotationMatrix(eul);
        V_fixed_trans = (R * V_fixed')' + pos;
        V_ctrl_trans = (R * V_ctrl')' + pos;
        patch('Vertices', V_fixed_trans, 'Faces', F_fixed, ...
            'FaceColor', color, 'FaceAlpha', 0.85, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        patch('Vertices', V_ctrl_trans, 'Faces', F_ctrl, ...
            'FaceColor', [1.0 0.95 0.20], 'FaceAlpha', 0.92, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        control_surface_specs(end + 1) = ... %#ok<AGROW>
            localBuildControlSurfaceSpec(name, eul, V_ctrl_trans, deflection_rad, 'ruddervator');
        if opts.show_labels
            label_pos = mean(V_ctrl_trans, 1);
            text(label_pos(1), label_pos(2), label_pos(3), ...
                sprintf(' %s rv %.1f deg', localShortTailName(name), rad2deg(deflection_rad)), ...
                'Color', [1.0 1.0 0.6], 'FontSize', label_sizes.control, 'FontWeight', 'bold');
        end
        continue;
    end

    if strcmp(type, 'fuselage')
        [V, F] = genFacetedFuselage(dim(1), dim(2), dim(3));
    elseif strcmp(type, 'box')
        [V, F] = genCleanBox(dim(1), dim(2), dim(3));
    elseif strcmp(type, 'crossprop')
        [V, F] = genCrossProp(dim(1), dim(2), dim(3));
    else
        continue;
    end

    R = localRotationMatrix(eul);
    V_trans = (R * V')' + pos;

    patch('Vertices', V_trans, 'Faces', F, ...
        'FaceColor', color, 'FaceAlpha', 0.85, ...
        'EdgeColor', 'k', 'LineWidth', 1.0);
end

legend_handles = gobjects(0);
legend_labels = {};

% CG marker
hCG = plot3(CG(1), CG(2), CG(3), 'wp', ...
    'MarkerFaceColor', 'w', 'MarkerSize', 15, 'LineWidth', 2);
legend_handles(end + 1) = hCG; %#ok<AGROW>
legend_labels{end + 1} = 'Center of Gravity'; %#ok<AGROW>
text(CG(1), CG(2), CG(3) - 0.6, '  CG', ...
    'FontWeight', 'bold', 'FontSize', label_sizes.cg, 'Color', 'w');

if opts.show_input_markers
    [hAeroInput, hPropInput] = localPlotInputMarkers(surfaces, opts.prop);
    if ~isempty(hAeroInput) && isgraphics(hAeroInput)
        legend_handles(end + 1) = hAeroInput; %#ok<AGROW>
        legend_labels{end + 1} = 'Aero Reference Points'; %#ok<AGROW>
    end
    if ~isempty(hPropInput) && isgraphics(hPropInput)
        legend_handles(end + 1) = hPropInput; %#ok<AGROW>
        legend_labels{end + 1} = 'Prop Pivot/Hub Points'; %#ok<AGROW>
    end
end

if opts.show_vectors
    hAeroNormal = localPlotSurfaceNormals(surfaces, opts.normal_scale, opts.show_labels, label_sizes);
    hControlNormal = localPlotControlSurfaceNormals(control_surface_specs, opts.normal_scale, opts.show_labels, label_sizes);
    [hFrontThrust, hRearThrust] = localPlotThrustVectors( ...
        frontSpecs, rearSpecs, opts.thrust_scale, opts.show_labels, label_sizes);
    localPrintVectorSummary(surfaces, frontSpecs, rearSpecs);
    localPrintControlSummary(ruddervator_state, flaperon_state, control_surface_specs);
    if ~isempty(hControlNormal) && isgraphics(hControlNormal)
        legend_handles(end + 1) = hControlNormal; %#ok<AGROW>
        legend_labels{end + 1} = 'Control Surface Normals'; %#ok<AGROW>
    end
    if ~isempty(hAeroNormal) && isgraphics(hAeroNormal)
        legend_handles(end + 1) = hAeroNormal; %#ok<AGROW>
        legend_labels{end + 1} = 'Aero Normal Vectors'; %#ok<AGROW>
    end
    if ~isempty(hFrontThrust) && isgraphics(hFrontThrust)
        legend_handles(end + 1) = hFrontThrust; %#ok<AGROW>
        legend_labels{end + 1} = 'Front Thrust Vectors'; %#ok<AGROW>
    end
    if ~isempty(hRearThrust) && isgraphics(hRearThrust)
        legend_handles(end + 1) = hRearThrust; %#ok<AGROW>
        legend_labels{end + 1} = 'Rear Thrust Vectors'; %#ok<AGROW>
    end
end

if ~isempty(legend_handles)
    legend(legend_handles, legend_labels, ...
        'TextColor', 'w', 'Color', 'none', 'Location', 'best');
end

view(130, 25);
camlight right;
lighting flat;
hold off;
end

function [tilt_angle, thrust_tilt_deg, title_text, extra_args] = ...
    localResolveRenderPreset(args)
preset_name = char(string(args{1}));
extra_args = args(2:end);

switch lower(strtrim(preset_name))
    case 'hover'
        tilt_angle = 0;
        thrust_tilt_deg = zeros(6, 1);
        title_text = 'Brown eVTOL Aircraft (Preset: Hover)';

    case 'cruise'
        tilt_angle = 90;
        thrust_tilt_deg = 90 * ones(6, 1);
        title_text = 'Brown eVTOL Aircraft (Preset: Cruise)';

    case 'transition'
        tilt_angle = 45;
        if ~isempty(extra_args) && isnumeric(extra_args{1}) && isscalar(extra_args{1})
            tilt_angle = extra_args{1};
            extra_args = extra_args(2:end);
        end
        thrust_tilt_deg = tilt_angle * ones(6, 1);
        title_text = sprintf('Brown eVTOL Aircraft (Preset: Transition %.1f deg)', ...
            tilt_angle);

    otherwise
        scenario = scenario_def(preset_name);
        tilt_angle = scenario.visual_tilt_deg;
        thrust_tilt_deg = scenario.Tilt_angles;
        title_text = sprintf('Brown eVTOL Aircraft (Scenario: %s)', scenario.name);
end

if localHasOption(extra_args, 'title_text')
    title_text = '';
end
end

function tf = localHasOption(args, option_name)
tf = false;
if isempty(args)
    return;
end

for idx = 1:2:numel(args)
    if ~(ischar(args{idx}) || (isstring(args{idx}) && isscalar(args{idx})))
        continue;
    end

    if strcmpi(char(string(args{idx})), option_name)
        tf = true;
        return;
    end
end
end

function surfaces = localNormalizeSurfaces(surfaceInput)
if isempty(surfaceInput)
    surfaces = struct([]);
    return;
end

if isstruct(surfaceInput)
    surfaces = surfaceInput;
    return;
end

if ~iscell(surfaceInput)
    error('render_aircraft:BadSurfaces', ...
        'surfaces must be a struct or cell array of structs.');
end

if isempty(surfaceInput)
    surfaces = struct([]);
    return;
end

surfaces = surfaceInput{1};
for idx = 2:numel(surfaceInput)
    surfaces(end + 1) = surfaceInput{idx}; %#ok<AGROW>
end
end

function [delta_e_rad, delta_r_rad, delta_f_rad, delta_a_rad] = localResolveControlInputs(opts)
delta_e_rad = opts.delta_e + deg2rad(opts.delta_e_deg);
delta_r_rad = opts.delta_r + deg2rad(opts.delta_r_deg);
delta_f_rad = opts.delta_f + deg2rad(opts.delta_f_deg);
delta_a_rad = opts.delta_a + deg2rad(opts.delta_a_deg);
end

function state = localRuddervatorState(controls, delta_e_rad, delta_r_rad)
state = struct( ...
    'enabled', false, ...
    'chord_fraction', 0.30, ...
    'left_deflection_rad', 0, ...
    'right_deflection_rad', 0, ...
    'label', 'Ruddervators');

if ~isfield(controls, 'ruddervator') || isempty(controls.ruddervator)
    return;
end

state.enabled = true;
if isfield(controls.ruddervator, 'control_chord_fraction')
    state.chord_fraction = controls.ruddervator.control_chord_fraction;
end

mix = [1, -1; 1, 1];
if isfield(controls.ruddervator, 'mix_from_standard') && ...
        isequal(size(controls.ruddervator.mix_from_standard), [2, 2])
    mix = controls.ruddervator.mix_from_standard;
end

deflections = mix * [delta_e_rad; delta_r_rad];
if isfield(controls.ruddervator, 'max_deflection_rad') && ~isempty(controls.ruddervator.max_deflection_rad)
    deflections = max(min(deflections, controls.ruddervator.max_deflection_rad), ...
        -controls.ruddervator.max_deflection_rad);
end
state.left_deflection_rad = deflections(1);
state.right_deflection_rad = deflections(2);
end

function state = localFlaperonState(controls, delta_f_rad, delta_a_rad)
state = struct( ...
    'enabled', false, ...
    'chord_fraction', 0.28, ...
    'left_deflection_rad', 0, ...
    'right_deflection_rad', 0, ...
    'label', 'Flaperons');

if ~isfield(controls, 'flaperon') || isempty(controls.flaperon)
    return;
end

state.enabled = true;
if isfield(controls.flaperon, 'control_chord_fraction')
    state.chord_fraction = controls.flaperon.control_chord_fraction;
end

mix = [1, 1; 1, -1];
if isfield(controls.flaperon, 'mix_from_standard') && ...
        isequal(size(controls.flaperon.mix_from_standard), [2, 2])
    mix = controls.flaperon.mix_from_standard;
end

deflections = mix * [delta_f_rad; delta_a_rad];
if isfield(controls.flaperon, 'max_deflection_rad') && ~isempty(controls.flaperon.max_deflection_rad)
    deflections = max(min(deflections, controls.flaperon.max_deflection_rad), ...
        -controls.flaperon.max_deflection_rad);
end
state.left_deflection_rad = deflections(1);
state.right_deflection_rad = deflections(2);
end

function color = localComponentColor(name, type)
if strcmp(type, 'fuselage')
    color = [0.0 1.0 1.0];
elseif contains(name, 'Ballast')
    color = [1.0 1.0 0.0];
elseif contains(name, 'Wing')
    color = [1.0 0.0 1.0];
elseif contains(name, 'Tail')
    color = [1.0 0.5 0.0];
elseif contains(name, 'F-Rotor')
    color = [1.0 0.0 0.0];
elseif contains(name, 'R-Rotor')
    color = [0.0 0.5 1.0];
elseif strcmp(type, 'box')
    color = [0.0 1.0 0.0];
else
    color = [0.5 0.5 0.5];
end
end

function deflection_rad = localTailDeflection(name, state)
deflection_rad = 0;
if ~state.enabled
    return;
end

if contains(name, 'L V-Tail')
    deflection_rad = state.left_deflection_rad;
elseif contains(name, 'R V-Tail')
    deflection_rad = state.right_deflection_rad;
end
end

function deflection_rad = localWingDeflection(name, state)
deflection_rad = 0;
if ~state.enabled
    return;
end

if contains(name, 'L Main Wing')
    deflection_rad = state.left_deflection_rad;
elseif contains(name, 'R Main Wing')
    deflection_rad = state.right_deflection_rad;
end
end

function hub_offset = localHubOffset(prop)
hub_offset = 0.50;
if isfield(prop, 'hub_offset') && ~isempty(prop.hub_offset)
    hub_offset = prop.hub_offset;
end
end

function pivot = localFrontPivot(name, fallback_pos, prop)
pivot = fallback_pos;

tokens = regexp(name, 'F-(?:Arm|Rotor) ([LR])([123])', 'tokens', 'once');
if isempty(tokens)
    return;
end

side = tokens{1};
index = str2double(tokens{2});
field_name = 'posFL';
if side == 'R'
    field_name = 'posFR';
end

if isfield(prop, field_name) && size(prop.(field_name), 1) >= index
    pivot = prop.(field_name)(index, :);
    return;
end

pivot = [1.95, fallback_pos(2), -0.45];
end

function tilt_deg = localFrontComponentTilt(name, default_tilt_deg, thrust_tilt_deg)
tilt_deg = default_tilt_deg;

if isempty(thrust_tilt_deg) || numel(thrust_tilt_deg) ~= 6
    return;
end

tokens = regexp(name, 'F-(?:Arm|Rotor) ([LR])([123])', 'tokens', 'once');
if isempty(tokens)
    return;
end

side = tokens{1};
index = str2double(tokens{2});
if side == 'R'
    tilt_deg = thrust_tilt_deg(index);
else
    tilt_deg = thrust_tilt_deg(index + 3);
end
end

function dir_vec = localThrustDirection(tilt_deg)
dir_vec = [sind(tilt_deg), 0, -cosd(tilt_deg)];
end

function specs = localFrontRotorSpecs(prop, thrust_tilt_deg, default_tilt_deg)
specs = struct('name', {}, 'hub', {}, 'dir', {}, 'tilt_deg', {});
if ~isfield(prop, 'posFR') || ~isfield(prop, 'posFL') || ~isfield(prop, 'hub_offset')
    return;
end

right_tilts = default_tilt_deg * ones(3, 1);
left_tilts = default_tilt_deg * ones(3, 1);
if numel(thrust_tilt_deg) == 6
    right_tilts = thrust_tilt_deg(1:3);
    left_tilts = thrust_tilt_deg(4:6);
end

specs = [ ...
    localBuildFrontGroupSpecs(prop.posFR, prop.hub_offset, right_tilts, 'FR'); ...
    localBuildFrontGroupSpecs(prop.posFL, prop.hub_offset, left_tilts, 'FL')];
end

function specs = localBuildFrontGroupSpecs(pivots, hub_offset, tilts_deg, prefix)
specs = struct('name', {}, 'hub', {}, 'dir', {}, 'tilt_deg', {});
for idx = 1:size(pivots, 1)
    dir_vec = localThrustDirection(tilts_deg(idx));
    specs(idx).name = sprintf('%s%d', prefix, idx); %#ok<AGROW>
    specs(idx).hub = pivots(idx, :) + hub_offset * dir_vec;
    specs(idx).dir = dir_vec;
    specs(idx).tilt_deg = tilts_deg(idx);
end
end

function specs = localRearRotorSpecs(prop)
specs = struct('name', {}, 'hub', {}, 'dir', {}, 'tilt_deg', {});
if ~isfield(prop, 'posRR') || ~isfield(prop, 'posRL')
    return;
end

dir_vec = [0, 0, -1];
specs = [ ...
    localBuildRearGroupSpecs(prop.posRR, dir_vec, 'RR'); ...
    localBuildRearGroupSpecs(prop.posRL, dir_vec, 'RL')];
end

function specs = localBuildRearGroupSpecs(hubs, dir_vec, prefix)
specs = struct('name', {}, 'hub', {}, 'dir', {}, 'tilt_deg', {});
for idx = 1:size(hubs, 1)
    specs(idx).name = sprintf('%s%d', prefix, idx); %#ok<AGROW>
    specs(idx).hub = hubs(idx, :);
    specs(idx).dir = dir_vec;
    specs(idx).tilt_deg = 0;
end
end

function [hAeroInput, hPropInput] = localPlotInputMarkers(surfaces, prop)
hAeroInput = [];
hPropInput = [];

disp('Plotting verification markers for block inputs...');
for idx = 1:numel(surfaces)
    surf = surfaces(idx);
    h = plot3(surf.pos(1), surf.pos(2), surf.pos(3), ...
        'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    if isempty(hAeroInput)
        hAeroInput = h;
    end
end

prop_fields = {'posFR', 'posFL', 'posRR', 'posRL'};
for idx = 1:numel(prop_fields)
    field_name = prop_fields{idx};
    if ~isfield(prop, field_name) || isempty(prop.(field_name))
        continue;
    end

    h = plot3(prop.(field_name)(:,1), prop.(field_name)(:,2), prop.(field_name)(:,3), ...
        'co', 'MarkerSize', 8, 'LineWidth', 2);
    if isempty(hPropInput)
        hPropInput = h;
    end
end
end

function hFirst = localPlotSurfaceNormals(surfaces, scale, show_labels, label_sizes)
hFirst = [];
for idx = 1:numel(surfaces)
    surf = surfaces(idx);
    surf_name = localSurfaceName(surf, idx);
    dir_vec = surf.n(:)' / max(norm(surf.n), eps);
    h = quiver3(surf.pos(1), surf.pos(2), surf.pos(3), ...
        scale * dir_vec(1), scale * dir_vec(2), scale * dir_vec(3), 0, ...
        'Color', [1.0 0.4 1.0], 'LineWidth', 2, 'MaxHeadSize', 0.6);
    if isempty(hFirst)
        hFirst = h;
    end

    if show_labels
        label_pos = surf.pos(:)' + (scale + 0.15) * dir_vec;
        text(label_pos(1), label_pos(2), label_pos(3), ...
            sprintf(' %s n', surf_name), ...
            'Color', [1.0 0.8 1.0], 'FontWeight', 'bold', 'FontSize', label_sizes.aero);
    end
end
end

function [hFrontFirst, hRearFirst] = ...
    localPlotThrustVectors(frontSpecs, rearSpecs, scale, show_labels, label_sizes)
hFrontFirst = [];
hRearFirst = [];
for idx = 1:numel(frontSpecs)
    spec = frontSpecs(idx);
    h = quiver3(spec.hub(1), spec.hub(2), spec.hub(3), ...
        scale * spec.dir(1), scale * spec.dir(2), scale * spec.dir(3), 0, ...
        'Color', [1.0 0.7 0.0], 'LineWidth', 2, 'MaxHeadSize', 0.6);
    if isempty(hFrontFirst)
        hFrontFirst = h;
    end

    if show_labels && idx <= 2
        label_pos = spec.hub + (scale + 0.1) * spec.dir;
        text(label_pos(1), label_pos(2), label_pos(3), ...
            sprintf(' %s thrust', spec.name), ...
            'Color', [1.0 0.9 0.4], 'FontSize', label_sizes.thrust);
    end
end

for idx = 1:numel(rearSpecs)
    spec = rearSpecs(idx);
    h = quiver3(spec.hub(1), spec.hub(2), spec.hub(3), ...
        scale * spec.dir(1), scale * spec.dir(2), scale * spec.dir(3), 0, ...
        'Color', [0.2 0.9 1.0], 'LineWidth', 2, 'MaxHeadSize', 0.6);
    if isempty(hRearFirst)
        hRearFirst = h;
    end

    if show_labels && idx <= 2
        label_pos = spec.hub + (scale + 0.1) * spec.dir;
        text(label_pos(1), label_pos(2), label_pos(3), ...
            sprintf(' %s thrust', spec.name), ...
            'Color', [0.6 1.0 1.0], 'FontSize', label_sizes.thrust);
    end
end
end

function hFirst = localPlotControlSurfaceNormals(specs, scale, show_labels, label_sizes)
hFirst = [];
for idx = 1:numel(specs)
    spec = specs(idx);
    h = quiver3(spec.pos(1), spec.pos(2), spec.pos(3), ...
        scale * spec.dir(1), scale * spec.dir(2), scale * spec.dir(3), 0, ...
        'Color', [1.0 0.95 0.20], 'LineWidth', 2, 'MaxHeadSize', 0.6);
    if isempty(hFirst)
        hFirst = h;
    end

    if show_labels
        label_pos = spec.pos + (scale + 0.1) * spec.dir;
        text(label_pos(1), label_pos(2), label_pos(3), ...
            sprintf(' %s n', spec.name), ...
            'Color', [1.0 1.0 0.6], 'FontSize', label_sizes.control, 'FontWeight', 'bold');
    end
end
end

function label_sizes = localLabelFontSizes(scale)
scale = max(0.45, scale);
label_sizes = struct( ...
    'cg', max(8, 12 * scale), ...
    'aero', max(6, 10 * scale), ...
    'thrust', max(6, 9 * scale), ...
    'control', max(6, 9 * scale));
end

function localPrintVectorSummary(surfaces, frontSpecs, rearSpecs)
disp('Aero surface normal vectors:');
for idx = 1:numel(surfaces)
    surf = surfaces(idx);
    surf_name = localSurfaceName(surf, idx);
    fprintf('  %-10s pos = [%6.2f %6.2f %6.2f], n = [%5.2f %5.2f %5.2f]\n', ...
        surf_name, surf.pos(1), surf.pos(2), surf.pos(3), ...
        surf.n(1), surf.n(2), surf.n(3));
end

disp('Propeller thrust vectors:');
for idx = 1:numel(frontSpecs)
    spec = frontSpecs(idx);
    fprintf('  %-4s hub = [%6.2f %6.2f %6.2f], tilt = %5.1f deg, thrust = [%5.2f %5.2f %5.2f]\n', ...
        spec.name, spec.hub(1), spec.hub(2), spec.hub(3), spec.tilt_deg, ...
        spec.dir(1), spec.dir(2), spec.dir(3));
end
for idx = 1:numel(rearSpecs)
    spec = rearSpecs(idx);
    fprintf('  %-4s hub = [%6.2f %6.2f %6.2f], thrust = [%5.2f %5.2f %5.2f]\n', ...
        spec.name, spec.hub(1), spec.hub(2), spec.hub(3), ...
        spec.dir(1), spec.dir(2), spec.dir(3));
end
end

function localPrintControlSummary(ruddervator_state, flaperon_state, specs)
if ~ruddervator_state.enabled && ~flaperon_state.enabled
    return;
end

disp('Ruddervator deflections:');
fprintf('  L tail ruddervator = %6.2f deg\n', rad2deg(ruddervator_state.left_deflection_rad));
fprintf('  R tail ruddervator = %6.2f deg\n', rad2deg(ruddervator_state.right_deflection_rad));

disp('Flaperon deflections:');
fprintf('  L wing flaperon   = %6.2f deg\n', rad2deg(flaperon_state.left_deflection_rad));
fprintf('  R wing flaperon   = %6.2f deg\n', rad2deg(flaperon_state.right_deflection_rad));

if isempty(specs)
    return;
end

disp('Control-surface normal vectors:');
for idx = 1:numel(specs)
    spec = specs(idx);
    fprintf('  %-10s pos = [%6.2f %6.2f %6.2f], n = [%5.2f %5.2f %5.2f]\n', ...
        spec.name, spec.pos(1), spec.pos(2), spec.pos(3), ...
        spec.dir(1), spec.dir(2), spec.dir(3));
end
end

function surf_name = localSurfaceName(surf, idx)
if isfield(surf, 'name') && ~isempty(surf.name)
    surf_name = surf.name;
else
    surf_name = sprintf('Surface %d', idx);
end
end

function short_name = localShortTailName(name)
if contains(name, 'L V-Tail')
    short_name = 'L Tail';
elseif contains(name, 'R V-Tail')
    short_name = 'R Tail';
else
    short_name = char(name);
end
end

function short_name = localShortWingName(name)
if contains(name, 'L Main Wing')
    short_name = 'L Wing';
elseif contains(name, 'R Main Wing')
    short_name = 'R Wing';
else
    short_name = char(name);
end
end

function [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
    localBuildTailWithRuddervator(dim, control_chord_fraction, deflection_rad)
[V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
    localBuildTrailingEdgeSurface(dim, control_chord_fraction, deflection_rad);
end

function [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
    localBuildWingWithFlaperon(dim, control_chord_fraction, deflection_rad)
[V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
    localBuildTrailingEdgeSurface(dim, control_chord_fraction, deflection_rad);
end

function [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
    localBuildTrailingEdgeSurface(dim, control_chord_fraction, deflection_rad)
chord = dim(1);
span = dim(2);
thickness = dim(3);

control_chord_fraction = min(max(control_chord_fraction, 0.05), 0.95);
ctrl_chord = chord * control_chord_fraction;
fixed_chord = chord - ctrl_chord;

% Positive x is forward in this model, so the control surface belongs on
% the aft/trailing-edge side, which is the negative-x side.
[V_fixed, F_fixed] = genCleanBox(fixed_chord, span, thickness);
V_fixed(:, 1) = V_fixed(:, 1) + 0.5 * ctrl_chord;

[V_ctrl, F_ctrl] = genCleanBox(ctrl_chord, span, thickness);
V_ctrl(:, 1) = V_ctrl(:, 1) - 0.5 * fixed_chord;

hinge_local = [-0.5 * chord + ctrl_chord, 0, 0];
R_deflect = localRotationMatrix([0, deflection_rad, 0]);
V_ctrl = (R_deflect * (V_ctrl - hinge_local)')' + hinge_local;
end

function spec = localBuildControlSurfaceSpec(name, eul, V_ctrl_trans, deflection_rad, kind)
R_surface = localRotationMatrix(eul);
R_deflect = localRotationMatrix([0, deflection_rad, 0]);
dir_vec = R_surface * R_deflect * [0; 0; -1];
dir_vec = dir_vec / max(norm(dir_vec), eps);

spec = struct();
if strcmp(kind, 'flaperon')
    spec.name = localShortWingName(name);
else
    spec.name = localShortTailName(name);
end
spec.pos = mean(V_ctrl_trans, 1);
spec.dir = dir_vec(:)';
spec.deflection_deg = rad2deg(deflection_rad);
spec.kind = kind;
end

function R = localRotationMatrix(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rz * Ry * Rx;
end

function [V, F] = genCleanBox(L, W, H)
x = L / 2;
y = W / 2;
z = H / 2;
V = [-x -y -z;  x -y -z;  x  y -z; -x  y -z; ...
     -x -y  z;  x -y  z;  x  y  z; -x  y  z];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function [V, F] = genCrossProp(D, W, T)
[V1, F1] = genCleanBox(D, W, T);
[V2, F2] = genCleanBox(W, D, T);
V = [V1; V2];
F = [F1; F2 + 8];
end

function [V, F] = genFacetedFuselage(L, W, H)
x_norm = [0.5, 0.35, 0.05, -0.3, -0.5];
w_norm = [0.3, 0.8, 1.0, 0.6, 0.2];
z_top_norm = [-0.2, -0.7, -1.0, -0.6, -0.3];
z_bot_norm = [ 0.4,  0.8,  1.0,  0.5, -0.1];
n_sec = length(x_norm);
V = [];
for i = 1:n_sec
    x = x_norm(i) * L;
    y = w_norm(i) * W / 2;
    zT = z_top_norm(i) * H / 2;
    zB = z_bot_norm(i) * H / 2;
    V = [V; x, y, zT; x, -y, zT; x, -y, zB; x, y, zB]; %#ok<AGROW>
end

F = [];
for i = 1:(n_sec - 1)
    idx = (i - 1) * 4;
    F = [F; idx + 1, idx + 2, idx + 6, idx + 5]; %#ok<AGROW>
    F = [F; idx + 2, idx + 3, idx + 7, idx + 6]; %#ok<AGROW>
    F = [F; idx + 3, idx + 4, idx + 8, idx + 7]; %#ok<AGROW>
    F = [F; idx + 4, idx + 1, idx + 5, idx + 8]; %#ok<AGROW>
end

F = [F; 1, 2, 3, 4];
idx = (n_sec - 1) * 4;
F = [F; idx + 4, idx + 3, idx + 2, idx + 1];
end
