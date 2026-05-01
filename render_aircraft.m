function hFig = render_aircraft(varargin)
% render aircraft

if nargin == 0
    aircraft = aircraft_def();
    scenario = scenario_def();
    hFig = render_aircraft(aircraft.compData, aircraft.CG, scenario.visual_tilt_deg, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', aircraft.prop, ...
        'controls', aircraft.controls, ...
        'thrust_tilt_deg', scenario.Tilt_angles, ...
        'body_eul', scenario.eul_init, ...
        'title_text', sprintf('%s (Scenario: %s)', ...
            'Brown eVTOL Aircraft', scenario.name));
    return;
end

if ischar(varargin{1}) || (isstring(varargin{1}) && isscalar(varargin{1}))
    [request, extra_args] = localResolveRenderPreset(varargin);
    aircraft = aircraft_def('flight_mode', 0);
    hFig = render_aircraft(aircraft.compData, aircraft.CG, request.tilt_angle, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', aircraft.prop, ...
        'controls', aircraft.controls, ...
        'thrust_tilt_deg', request.thrust_tilt_deg, ...
        'body_eul', request.body_eul, ...
        'title_text', request.title_text, ...
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
p.addParameter('surface_deflections', [], @(x) isempty(x) || isnumeric(x) || isstruct(x));
p.addParameter('surface_deflections_deg', [], @(x) isempty(x) || isnumeric(x) || isstruct(x));
p.addParameter('body_eul', [], @isnumeric);
p.addParameter('body_eul_deg', [], @isnumeric);
p.addParameter('body_pos', [], @isnumeric);
p.addParameter('show_vectors', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_aero_normals', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_labels', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_input_markers', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('normal_scale', 1.4, @isnumeric);
p.addParameter('thrust_scale', 1.1, @isnumeric);
p.addParameter('label_font_scale', 0.72, @isnumeric);
p.addParameter('print_summary', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('title_text', '', @(x) ischar(x) || isstring(x));
p.addParameter('theme', 'dark', @(x) ischar(x) || isstring(x));
p.addParameter('figure_visible', 'on', @(x) ischar(x) || isstring(x) || islogical(x) || isnumeric(x));
p.addParameter('show_axes', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_title', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_legend', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('show_cg_label', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{4:end});
opts = p.Results;

CG = localAsColumn3(CG, 'render_aircraft:BadCG', 'CG must be a 3-element vector.');
surfaces = localNormalizeSurfaces(opts.surfaces);
surface_deflections = localResolveSurfaceDeflections(opts);
front_tilts_deg = localResolveFrontTiltVector(opts.thrust_tilt_deg, tilt_angle);
body_pose = localResolveBodyPose(opts, CG);
label_sizes = localLabelFontSizes(opts.label_font_scale);
theme = localResolveRenderTheme(opts.theme);

surface_normal_specs = localTransformSurfaceData(surfaces, body_pose, CG);
frontSpecs = localTransformRotorSpecs(localFrontRotorSpecs(opts.prop, front_tilts_deg), body_pose, CG);
rearSpecs = localTransformRotorSpecs(localRearRotorSpecs(opts.prop), body_pose, CG);

hFig = figure('Name', 'VTOL Aircraft Render', ...
    'Color', theme.figure_color, ...
    'Position', [50 50 1400 900], ...
    'Visible', localResolveVisibleMode(opts.figure_visible));
hold on;
axis equal;
ax = gca;
if opts.show_axes
    grid on;
    set(ax, 'ZDir', 'reverse', ...
        'Color', theme.axes_color, ...
        'XColor', theme.axis_color, 'YColor', theme.axis_color, 'ZColor', theme.axis_color, ...
        'GridColor', theme.grid_color, 'GridAlpha', theme.grid_alpha);
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
else
    axis off;
    set(ax, 'ZDir', 'reverse', ...
        'Color', theme.axes_color, ...
        'XColor', theme.axis_color, 'YColor', theme.axis_color, 'ZColor', theme.axis_color);
end

if opts.show_title
    title(localBuildTitleText(opts.title_text, mean(front_tilts_deg), body_pose.eul_deg), ...
        'Color', theme.text_color, 'FontSize', 14);
end

control_surface_specs = struct('name', {}, 'pos', {}, 'dir', {}, 'deflection_deg', {}, 'kind', {});
N = size(compData, 1);
for i = 1:N
    name = compData{i, 1};
    type = compData{i, 2};
    dim = compData{i, 4};
    pos_body = compData{i, 5};
    eul_body = deg2rad(compData{i, 6});
    rotor_tilt_deg = localFrontComponentTilt(name, front_tilts_deg, tilt_angle);

    if contains(name, 'F-Arm')
        pivot = localFrontPivot(name, pos_body, opts.prop);
        thrust_dir_body = localThrustDirection(rotor_tilt_deg);
        hub_offset = localHubOffset(opts.prop);
        dim = [hub_offset, 0.20, 0.20];
        pos_body = pivot + (0.5 * hub_offset * thrust_dir_body);
        eul_body = deg2rad([0, 90 - rotor_tilt_deg, 0]);
    elseif contains(name, 'F-Rotor')
        pivot = localFrontPivot(name, pos_body, opts.prop);
        thrust_dir_body = localThrustDirection(rotor_tilt_deg);
        pos_body = pivot + (localHubOffset(opts.prop) * thrust_dir_body);
        eul_body = deg2rad([0, -rotor_tilt_deg, 0]);
    end

    color = localComponentColor(name, type);
    R_comp = localRotationMatrix(eul_body);

    if strcmp(type, 'box') && contains(name, 'Main Wing')
        deflection_rad = localSurfaceDeflection(name, surface_deflections);
        control_chord_fraction = localWingControlChordFraction(opts.controls);
        [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
            localBuildWingWithFlaperon(dim, control_chord_fraction, deflection_rad);

        V_fixed_body = (R_comp * V_fixed')' + pos_body;
        V_ctrl_body = (R_comp * V_ctrl')' + pos_body;
        V_fixed_world = localApplyBodyPose(V_fixed_body, body_pose, CG);
        V_ctrl_world = localApplyBodyPose(V_ctrl_body, body_pose, CG);

        patch('Vertices', V_fixed_world, 'Faces', F_fixed, ...
            'FaceColor', color, 'FaceAlpha', 0.85, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        patch('Vertices', V_ctrl_world, 'Faces', F_ctrl, ...
            'FaceColor', [0.85 1.00 0.25], 'FaceAlpha', 0.92, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);

        control_surface_specs(end + 1) = ... %#ok<AGROW>
            localBuildControlSurfaceSpec(name, R_comp, V_ctrl_body, body_pose, CG, deflection_rad, 'flaperon');
        if opts.show_labels
            label_pos = mean(V_ctrl_world, 1);
            text(label_pos(1), label_pos(2), label_pos(3), ...
                sprintf(' %s %.1f deg', localShortWingName(name), rad2deg(deflection_rad)), ...
                'Color', [0.90 1.00 0.65], 'FontSize', label_sizes.control, 'FontWeight', 'bold');
        end
        continue;
    end

    if strcmp(type, 'box') && contains(name, 'V-Tail')
        deflection_rad = localSurfaceDeflection(name, surface_deflections);
        control_chord_fraction = localTailControlChordFraction(opts.controls);
        [V_fixed, F_fixed, V_ctrl, F_ctrl] = ...
            localBuildTailWithRuddervator(dim, control_chord_fraction, deflection_rad);

        V_fixed_body = (R_comp * V_fixed')' + pos_body;
        V_ctrl_body = (R_comp * V_ctrl')' + pos_body;
        V_fixed_world = localApplyBodyPose(V_fixed_body, body_pose, CG);
        V_ctrl_world = localApplyBodyPose(V_ctrl_body, body_pose, CG);

        patch('Vertices', V_fixed_world, 'Faces', F_fixed, ...
            'FaceColor', color, 'FaceAlpha', 0.85, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        patch('Vertices', V_ctrl_world, 'Faces', F_ctrl, ...
            'FaceColor', [1.0 0.95 0.20], 'FaceAlpha', 0.92, ...
            'EdgeColor', 'k', 'LineWidth', 1.0);

        control_surface_specs(end + 1) = ... %#ok<AGROW>
            localBuildControlSurfaceSpec(name, R_comp, V_ctrl_body, body_pose, CG, deflection_rad, 'ruddervator');
        if opts.show_labels
            label_pos = mean(V_ctrl_world, 1);
            text(label_pos(1), label_pos(2), label_pos(3), ...
                sprintf(' %s %.1f deg', localShortTailName(name), rad2deg(deflection_rad)), ...
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

    V_body = (R_comp * V')' + pos_body;
    V_world = localApplyBodyPose(V_body, body_pose, CG);

    patch('Vertices', V_world, 'Faces', F, ...
        'FaceColor', color, 'FaceAlpha', 0.85, ...
        'EdgeColor', 'k', 'LineWidth', 1.0);
end

legend_handles = gobjects(0);
legend_labels = {};

hCG = plot3(body_pose.position(1), body_pose.position(2), body_pose.position(3), 'o', ...
    'Color', theme.cg_edge_color, ...
    'MarkerFaceColor', theme.cg_face_color, ...
    'MarkerSize', 15, 'LineWidth', 2);
legend_handles(end + 1) = hCG; %#ok<AGROW>
legend_labels{end + 1} = 'Center of Gravity'; %#ok<AGROW>
text(body_pose.position(1), body_pose.position(2), body_pose.position(3) - 0.6, '  CG', ...
    'FontWeight', 'bold', 'FontSize', label_sizes.cg, 'Color', theme.text_color, ...
    'Visible', localOnOff(opts.show_cg_label));

if opts.show_input_markers
    [hAeroInput, hPropInput] = localPlotInputMarkers(surface_normal_specs, frontSpecs, rearSpecs);
    if ~isempty(hAeroInput) && isgraphics(hAeroInput)
        legend_handles(end + 1) = hAeroInput; %#ok<AGROW>
        legend_labels{end + 1} = 'Aero Reference Points'; %#ok<AGROW>
    end
    if ~isempty(hPropInput) && isgraphics(hPropInput)
        legend_handles(end + 1) = hPropInput; %#ok<AGROW>
        legend_labels{end + 1} = 'Rotor Hub Points'; %#ok<AGROW>
    end
end

if opts.show_vectors
    hAeroNormal = gobjects(0);
    hControlNormal = gobjects(0);
    if opts.show_aero_normals
        hAeroNormal = localPlotSurfaceNormals(surface_normal_specs, opts.normal_scale, opts.show_labels, label_sizes);
        hControlNormal = localPlotControlSurfaceNormals(control_surface_specs, opts.normal_scale, opts.show_labels, label_sizes);
    end
    [hFrontThrust, hRearThrust] = localPlotThrustVectors( ...
        frontSpecs, rearSpecs, opts.thrust_scale, opts.show_labels, label_sizes);
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

if opts.print_summary
    localPrintPoseSummary(body_pose, front_tilts_deg);
    localPrintVectorSummary(surface_normal_specs, frontSpecs, rearSpecs);
    localPrintSurfaceSummary(surface_deflections, control_surface_specs);
end

if opts.show_legend && ~isempty(legend_handles)
    legend(legend_handles, legend_labels, ...
        'TextColor', theme.text_color, 'Color', 'none', 'Location', 'best');
end

view(130, 25);
camlight right;
lighting flat;
hold off;
end

function [request, extra_args] = localResolveRenderPreset(args)
preset_name = char(string(args{1}));
extra_args = args(2:end);

request = struct( ...
    'tilt_angle', 90, ...
    'thrust_tilt_deg', 90 * ones(6, 1), ...
    'body_eul', zeros(3, 1), ...
    'title_text', '');

switch lower(strtrim(preset_name))
    case 'hover'
        request.tilt_angle = 0;
        request.thrust_tilt_deg = zeros(6, 1);
        request.title_text = 'Brown eVTOL Aircraft (Preset: Hover)';

    case 'cruise'
        request.tilt_angle = 90;
        request.thrust_tilt_deg = 90 * ones(6, 1);
        request.title_text = 'Brown eVTOL Aircraft (Preset: Cruise)';

    case 'transition'
        request.tilt_angle = 45;
        if ~isempty(extra_args) && isnumeric(extra_args{1}) && isscalar(extra_args{1})
            request.tilt_angle = extra_args{1};
            extra_args = extra_args(2:end);
        end
        request.thrust_tilt_deg = request.tilt_angle * ones(6, 1);
        request.title_text = sprintf('Brown eVTOL Aircraft (Preset: Transition %.1f deg)', ...
            request.tilt_angle);

    otherwise
        scenario = scenario_def(preset_name);
        request.tilt_angle = scenario.visual_tilt_deg;
        request.thrust_tilt_deg = scenario.Tilt_angles;
        request.body_eul = scenario.eul_init;
        request.title_text = sprintf('Brown eVTOL Aircraft (Scenario: %s)', scenario.name);
end

if localHasOption(extra_args, 'title_text')
    request.title_text = '';
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

function theme = localResolveRenderTheme(theme_name)
theme_name = lower(strtrim(char(string(theme_name))));

theme = struct( ...
    'figure_color', [0.10 0.10 0.10], ...
    'axes_color', [0.15 0.15 0.15], ...
    'axis_color', [1.0 1.0 1.0], ...
    'text_color', [1.0 1.0 1.0], ...
    'grid_color', [0.55 0.55 0.55], ...
    'grid_alpha', 0.22, ...
    'cg_face_color', [1.0 1.0 1.0], ...
    'cg_edge_color', [1.0 1.0 1.0]);

switch theme_name
    case {'report', 'white', 'light'}
        theme.figure_color = [1.0 1.0 1.0];
        theme.axes_color = [1.0 1.0 1.0];
        theme.axis_color = [0.10 0.10 0.10];
        theme.text_color = [0.10 0.10 0.10];
        theme.grid_color = [0.70 0.70 0.70];
        theme.grid_alpha = 0.28;
        theme.cg_face_color = [1.0 1.0 1.0];
        theme.cg_edge_color = [0.10 0.10 0.10];
end
end

function visible_mode = localResolveVisibleMode(value)
if islogical(value) || isnumeric(value)
    if value
        visible_mode = 'on';
    else
        visible_mode = 'off';
    end
    return;
end

visible_mode = lower(strtrim(char(string(value))));
if ~ismember(visible_mode, {'on', 'off'})
    visible_mode = 'on';
end
end

function value = localOnOff(tf)
if tf
    value = 'on';
else
    value = 'off';
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

function state = localResolveSurfaceDeflections(opts)
state = struct('deltaLW', 0, 'deltaRW', 0, 'deltaLT', 0, 'deltaRT', 0);

if ~isempty(opts.surface_deflections)
    state = localMergeSurfaceDeflections(state, opts.surface_deflections, false);
end
if ~isempty(opts.surface_deflections_deg)
    state = localMergeSurfaceDeflections(state, opts.surface_deflections_deg, true);
end
end

function state = localMergeSurfaceDeflections(state, deflections, use_degrees)
if isstruct(deflections)
    field_names = {'deltaLW', 'deltaRW', 'deltaLT', 'deltaRT'};
    for idx = 1:numel(field_names)
        field_name = field_names{idx};
        if isfield(deflections, field_name) && ~isempty(deflections.(field_name))
            value = deflections.(field_name);
            value = localAsScalar(value, 'render_aircraft:BadSurfaceDeflection', ...
                'Surface-deflection struct fields must be scalar values.');
            if use_degrees
                value = deg2rad(value);
            end
            state.(field_name) = value;
        end
    end
    return;
end

values = localAsColumn(deflections, 4, 'render_aircraft:BadSurfaceDeflection', ...
    'Surface deflections must be a 4-element vector in [deltaLW; deltaRW; deltaLT; deltaRT] order.');
if use_degrees
    values = deg2rad(values);
end

state.deltaLW = values(1);
state.deltaRW = values(2);
state.deltaLT = values(3);
state.deltaRT = values(4);
end

function front_tilts_deg = localResolveFrontTiltVector(thrust_tilt_deg, default_tilt_deg)
if isempty(thrust_tilt_deg)
    front_tilts_deg = default_tilt_deg * ones(6, 1);
    return;
end

if isscalar(thrust_tilt_deg)
    front_tilts_deg = thrust_tilt_deg * ones(6, 1);
    return;
end

values = thrust_tilt_deg(:);
switch numel(values)
    case 2
        front_tilts_deg = [ ...
            values(1) * ones(3, 1); ...
            values(2) * ones(3, 1)];
    case 6
        front_tilts_deg = values;
    otherwise
        error('render_aircraft:BadFrontTilt', ...
            ['thrust_tilt_deg must be a scalar, 2-element vector [FR; FL], ' ...
             'or 6-element vector [FR1; FR2; FR3; FL1; FL2; FL3].']);
end
end

function pose = localResolveBodyPose(opts, CG)
pose = struct();
pose.eul = zeros(3, 1);
if ~isempty(opts.body_eul)
    pose.eul = localAsColumn(opts.body_eul, 3, ...
        'render_aircraft:BadBodyEuler', ...
        'body_eul must be a 3-element [phi; theta; psi] vector in radians.');
end
if ~isempty(opts.body_eul_deg)
    pose.eul = deg2rad(localAsColumn(opts.body_eul_deg, 3, ...
        'render_aircraft:BadBodyEuler', ...
        'body_eul_deg must be a 3-element [phi; theta; psi] vector in degrees.'));
end

pose.position = CG;
if ~isempty(opts.body_pos)
    pose.position = localAsColumn(opts.body_pos, 3, ...
        'render_aircraft:BadBodyPosition', ...
        'body_pos must be a 3-element vector representing the CG position.');
end

pose.R = localRotationMatrix(pose.eul);
pose.eul_deg = rad2deg(pose.eul);
end

function txt = localBuildTitleText(title_input, front_tilt_deg, body_eul_deg)
if isstring(title_input)
    title_input = char(title_input);
end

if isempty(title_input)
    title_input = 'Brown eVTOL Aircraft';
end

txt = sprintf('%s\nBody Euler [deg]: [%.1f %.1f %.1f] | Front tilt: %.1f deg', ...
    title_input, body_eul_deg(1), body_eul_deg(2), body_eul_deg(3), front_tilt_deg);
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

function control_chord_fraction = localTailControlChordFraction(controls)
control_chord_fraction = 0.30;
if isfield(controls, 'ruddervator') && isfield(controls.ruddervator, 'control_chord_fraction')
    control_chord_fraction = controls.ruddervator.control_chord_fraction;
end
end

function control_chord_fraction = localWingControlChordFraction(controls)
control_chord_fraction = 0.28;
if isfield(controls, 'flaperon') && isfield(controls.flaperon, 'control_chord_fraction')
    control_chord_fraction = controls.flaperon.control_chord_fraction;
end
end

function deflection_rad = localSurfaceDeflection(name, state)
if contains(name, 'L Main Wing')
    deflection_rad = state.deltaLW;
elseif contains(name, 'R Main Wing')
    deflection_rad = state.deltaRW;
elseif contains(name, 'L V-Tail')
    deflection_rad = state.deltaLT;
elseif contains(name, 'R V-Tail')
    deflection_rad = state.deltaRT;
else
    deflection_rad = 0;
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

function tilt_deg = localFrontComponentTilt(name, front_tilts_deg, default_tilt_deg)
tilt_deg = default_tilt_deg;
tokens = regexp(name, 'F-(?:Arm|Rotor) ([LR])([123])', 'tokens', 'once');
if isempty(tokens)
    return;
end

side = tokens{1};
index = str2double(tokens{2});
if side == 'R'
    tilt_deg = front_tilts_deg(index);
else
    tilt_deg = front_tilts_deg(index + 3);
end
end

function dir_vec = localThrustDirection(tilt_deg)
dir_vec = [sind(tilt_deg), 0, -cosd(tilt_deg)];
end

function specs = localFrontRotorSpecs(prop, front_tilts_deg)
specs = struct('name', {}, 'hub', {}, 'dir', {}, 'tilt_deg', {});
if ~isfield(prop, 'posFR') || ~isfield(prop, 'posFL')
    return;
end

hub_offset = localHubOffset(prop);
right_tilts = front_tilts_deg(1:3);
left_tilts = front_tilts_deg(4:6);

specs = [ ...
    localBuildFrontGroupSpecs(prop.posFR, hub_offset, right_tilts, 'FR'); ...
    localBuildFrontGroupSpecs(prop.posFL, hub_offset, left_tilts, 'FL')];
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

function surfaces_world = localTransformSurfaceData(surfaces, body_pose, CG)
surfaces_world = surfaces;
for idx = 1:numel(surfaces)
    pos_world = localApplyBodyPose(surfaces(idx).pos(:)', body_pose, CG);
    n_world = localRotateDirection(surfaces(idx).n(:), body_pose);
    surfaces_world(idx).pos = pos_world(:)';
    surfaces_world(idx).n = n_world(:)';
end
end

function specs_world = localTransformRotorSpecs(specs, body_pose, CG)
specs_world = specs;
for idx = 1:numel(specs)
    specs_world(idx).hub = localApplyBodyPose(specs(idx).hub, body_pose, CG);
    specs_world(idx).dir = localRotateDirection(specs(idx).dir(:), body_pose).';
end
end

function points_world = localApplyBodyPose(points_body, body_pose, CG)
points_body = localAsPointRows(points_body);
CG_row = CG(:).';
points_world = (body_pose.R * (points_body - CG_row)')' + body_pose.position(:).';
end

function dir_world = localRotateDirection(dir_body, body_pose)
dir_world = body_pose.R * dir_body(:);
dir_world = dir_world / max(norm(dir_world), eps);
end

function [hAeroInput, hPropInput] = localPlotInputMarkers(surfaces, frontSpecs, rearSpecs)
hAeroInput = [];
hPropInput = [];

for idx = 1:numel(surfaces)
    surf = surfaces(idx);
    h = plot3(surf.pos(1), surf.pos(2), surf.pos(3), ...
        'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    if isempty(hAeroInput)
        hAeroInput = h;
    end
end

rotor_specs = [frontSpecs(:); rearSpecs(:)];
for idx = 1:numel(rotor_specs)
    spec = rotor_specs(idx);
    h = plot3(spec.hub(1), spec.hub(2), spec.hub(3), ...
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

function localPrintPoseSummary(body_pose, front_tilts_deg)
fprintf('Body Euler [deg] = [%6.2f %6.2f %6.2f]\n', ...
    body_pose.eul_deg(1), body_pose.eul_deg(2), body_pose.eul_deg(3));
fprintf('CG position       = [%6.2f %6.2f %6.2f]\n', ...
    body_pose.position(1), body_pose.position(2), body_pose.position(3));
fprintf('Front tilts [deg] = [%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f]\n', front_tilts_deg);
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

function localPrintSurfaceSummary(surface_deflections, specs)
disp('Local surface deflections:');
fprintf('  L wing = %6.2f deg\n', rad2deg(surface_deflections.deltaLW));
fprintf('  R wing = %6.2f deg\n', rad2deg(surface_deflections.deltaRW));
fprintf('  L tail = %6.2f deg\n', rad2deg(surface_deflections.deltaLT));
fprintf('  R tail = %6.2f deg\n', rad2deg(surface_deflections.deltaRT));

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

[V_fixed, F_fixed] = genCleanBox(fixed_chord, span, thickness);
V_fixed(:, 1) = V_fixed(:, 1) + 0.5 * ctrl_chord;

[V_ctrl, F_ctrl] = genCleanBox(ctrl_chord, span, thickness);
V_ctrl(:, 1) = V_ctrl(:, 1) - 0.5 * fixed_chord;

hinge_local = [-0.5 * chord + ctrl_chord, 0, 0];
R_deflect = localRotationMatrix([0, deflection_rad, 0]);
V_ctrl = (R_deflect * (V_ctrl - hinge_local)')' + hinge_local;
end

function spec = localBuildControlSurfaceSpec(name, R_comp, V_ctrl_body, body_pose, CG, deflection_rad, kind)
dir_body = R_comp * localRotationMatrix([0, deflection_rad, 0]) * [0; 0; -1];
dir_world = localRotateDirection(dir_body, body_pose);
pos_world = mean(localApplyBodyPose(V_ctrl_body, body_pose, CG), 1);

spec = struct();
if strcmp(kind, 'flaperon')
    spec.name = localShortWingName(name);
else
    spec.name = localShortTailName(name);
end
spec.pos = pos_world;
spec.dir = dir_world(:)';
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

function values = localAsColumn(value, expected_count, err_id, err_msg)
if ~isnumeric(value)
    error(err_id, err_msg);
end
values = value(:);
if numel(values) ~= expected_count
    error(err_id, err_msg);
end
end

function value = localAsScalar(value, err_id, err_msg)
if ~isnumeric(value) || numel(value) ~= 1
    error(err_id, err_msg);
end
value = double(value);
end

function value = localAsColumn3(value, err_id, err_msg)
value = localAsColumn(value, 3, err_id, err_msg);
end

function points = localAsPointRows(points)
if isempty(points)
    points = zeros(0, 3);
    return;
end
if ~isnumeric(points)
    error('render_aircraft:BadPointData', 'Point data must be numeric.');
end
if size(points, 2) == 3
    return;
end
if size(points, 1) == 3
    points = points.';
    return;
end
error('render_aircraft:BadPointData', 'Point data must have three columns.');
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
