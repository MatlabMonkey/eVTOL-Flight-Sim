function outputs = TESTER_external_forces_runner(varargin)
%TESTER_EXTERNAL_FORCES_RUNNER Run the classroom-style validation cases.
% This is the maintainable runnable counterpart to TESTER_external_forces.mlx.
% It uses aircraft_def.m and scenario_def.m as the source of truth for the
% aircraft configuration, mass properties, and final validation scenario.

p = inputParser;
p.addParameter('Cases', {}, @(x) isempty(x) || ischar(x) || isstring(x) || iscellstr(x));
p.addParameter('ShowRender', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('MakePlots', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('ReplayFlightGear', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('ReplayCase', 'Case 5 - Cruise Beta to Yaw', @(x) ischar(x) || isstring(x));
p.addParameter('PreviewOnly', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opts = p.Results;

root_dir = fileparts(mfilename('fullpath'));
addpath(root_dir);

required_files = {
    'aircraft_def.m'
    'scenario_def.m'
    'render_aircraft.m'
    'TESTER_Brown_Full_Sim.slx'
    'Brown_Full_Sim.slx'
};

for idx = 1:numel(required_files)
    required_path = fullfile(root_dir, required_files{idx});
    if ~isfile(required_path)
        error('TESTER_external_forces_runner:MissingFile', ...
            'Missing required file: %s', required_path);
    end
end

fprintf('Updated tester run using aircraft_def.m / scenario_def.m\n');
fprintf('Repository root: %s\n', root_dir);

aircraft = aircraft_def('flight_mode', 0);

if opts.ShowRender
    render_aircraft(aircraft.compData, aircraft.CG, aircraft.tilt_angle, ...
        'surfaces', aircraft.render_surfaces, ...
        'prop', aircraft.prop, ...
        'thrust_tilt_deg', zeros(6, 1), ...
        'title_text', 'Tester Aircraft Configuration');
end

cases = localBuildCases(aircraft);
cases = localFilterCases(cases, opts.Cases);
localDisplayCaseSummary(cases);

outputs = struct();
outputs.case_summary = localBuildSummaryTable(cases);

if opts.PreviewOnly
    return;
end

for idx = 1:numel(cases)
    case_def = cases{idx};
    field_name = matlab.lang.makeValidName(case_def.name);
    outputs.(field_name) = localRunFlightTest(case_def, aircraft, logical(opts.MakePlots));
end

if opts.ReplayFlightGear
    replay_field = matlab.lang.makeValidName(char(string(opts.ReplayCase)));
    if ~isfield(outputs, replay_field)
        error('TESTER_external_forces_runner:ReplayCaseMissing', ...
            'Replay case "%s" was not run in this invocation.', char(string(opts.ReplayCase)));
    end

    addpath(fullfile(root_dir, 'Flight_Gear'));
    cfg = fg_config();
    fg_replay_from_out(outputs.(replay_field).out, cfg);
end
end

function cases = localBuildCases(aircraft)
tester_model = 'TESTER_Brown_Full_Sim';
full_model = 'Brown_Full_Sim';

hover_rpms = zeros(12, 1);
hover_tilts = zeros(6, 1);
cruise_rpms = zeros(12, 1);
cruise_tilts = 90 * ones(6, 1);

weight_N = aircraft.Mass * 9.81;
cruise_u = 55;
cruise_w = 5;
beta_v = 5;
inertia_roll_rate = 0.20;
cross_rate = 0.02;
roll_probe_rate = 0.08;
pitch_moment_step = 3000;

TC1.name = 'Case 1 - Inertia Coupling';
TC1.model = tester_model;
TC1.t_stop = 10;
TC1.pos_init = [0; 0; -1000];
TC1.V_init = [0; 0; 0];
TC1.eul_init_deg = [0; 0; 0];
TC1.omega_init = [inertia_roll_rate; cross_rate; cross_rate];
TC1.g = 0;
TC1.Fext_B = [0; 0; 0];
TC1.Mext_B = [0; 0; 0];
TC1.Motor_RPMs = hover_rpms;
TC1.Tilt_angles = hover_tilts;
TC1.purpose = 'Rigid-body rotational coupling with gravity and external loads disabled.';
TC1.basis = 'Rates scaled to the current aircraft inertia so the case is energetic but still realistic.';
TC1.expected = 'Body rates should persist with mild cross-axis coupling and almost no translation.';
TC1.note = 'Checks that the updated inertia matrix produces sensible rigid-body coupling.';

TC2.name = 'Case 2 - 95% Weight Support';
TC2.model = tester_model;
TC2.t_stop = 10;
TC2.pos_init = [0; 0; -1000];
TC2.V_init = [0; 0; 0];
TC2.eul_init_deg = [0; 0; 0];
TC2.omega_init = [0; 0; 0];
TC2.g = 9.81;
TC2.Fext_B = [0; 0; -0.95 * weight_N];
TC2.Mext_B = [0; 0; 0];
TC2.Motor_RPMs = hover_rpms;
TC2.Tilt_angles = hover_tilts;
TC2.purpose = 'Vertical force-balance test using the current modeled aircraft weight.';
TC2.basis = sprintf('External upward force is set to 95%% of the current weight (%.0f N).', weight_N);
TC2.expected = 'Vehicle should descend slowly because external support is slightly smaller than weight.';
TC2.note = 'Checks that the tester model responds correctly to near-balanced external lift.';

TC3.name = 'Case 3 - Pitch Moment Step';
TC3.model = tester_model;
TC3.t_stop = 10;
TC3.pos_init = [0; 0; -1000];
TC3.V_init = [0; 0; 0];
TC3.eul_init_deg = [0; 0; 0];
TC3.omega_init = [0; 0; 0];
TC3.g = 0;
TC3.Fext_B = [0; 0; 0];
TC3.Mext_B = [0; pitch_moment_step; 0];
TC3.Motor_RPMs = hover_rpms;
TC3.Tilt_angles = hover_tilts;
TC3.purpose = 'Pure pitch-moment response using the current aircraft pitch inertia.';
TC3.basis = sprintf('Moment step is %g N-m, which gives an initial q-dot of about %.2f rad/s^2 with current Jyy.', ...
    pitch_moment_step, pitch_moment_step / aircraft.J(2, 2));
TC3.expected = 'Pitch rate should grow in the commanded direction with only small translation or roll/yaw coupling.';
TC3.note = 'Checks that the updated pitch inertia produces a believable angular-acceleration response.';

TC4.name = 'Case 4 - Roll Damping';
TC4.model = full_model;
TC4.t_stop = 10;
TC4.pos_init = [0; 0; -1000];
TC4.V_init = [cruise_u; 0; cruise_w];
TC4.eul_init_deg = [0; 0; 0];
TC4.omega_init = [roll_probe_rate; 0; 0];
TC4.g = 0;
TC4.Fext_B = [0; 0; 0];
TC4.Mext_B = [0; 0; 0];
TC4.Motor_RPMs = cruise_rpms;
TC4.Tilt_angles = cruise_tilts;
TC4.purpose = 'Aerodynamic roll-rate damping in a plausible cruise condition.';
TC4.basis = sprintf('Uses u = %g m/s and w = %g m/s, which corresponds to about %.1f deg alpha for the current wing.', ...
    cruise_u, cruise_w, rad2deg(atan2(cruise_w, cruise_u)));
TC4.expected = 'Roll rate should damp over time and may couple into yaw/pitch because of the V-tail and non-diagonal inertia.';
TC4.note = 'Replaces the old 250 m/s placeholder with a cruise-speed value that matches the current wing loading much better.';

TC5.name = 'Case 5 - Cruise Beta to Yaw';
TC5.model = full_model;
TC5.t_stop = 10;
TC5.pos_init = [0; 0; -1000];
TC5.V_init = [cruise_u; beta_v; cruise_w];
TC5.eul_init_deg = [0; 0; 0];
TC5.omega_init = [0; 0; 0];
TC5.g = 0;
TC5.Fext_B = [0; 0; 0];
TC5.Mext_B = [0; 0; 0];
TC5.Motor_RPMs = cruise_rpms;
TC5.Tilt_angles = cruise_tilts;
TC5.purpose = 'Sideslip-to-yaw/roll coupling check at a plausible cruise speed.';
TC5.basis = sprintf('Uses v = %g m/s at u = %g m/s, which corresponds to about %.1f deg beta.', ...
    beta_v, cruise_u, rad2deg(atan2(beta_v, cruise_u)));
TC5.expected = 'The aircraft should show a yawing and rolling response that tends to react to the initial sideslip.';
TC5.note = 'Checks whether the updated V-tail and wing aero data produce reasonable directional coupling.';

TC6.name = 'Case 6 - Unpowered Glide Response';
TC6.model = full_model;
TC6.t_stop = 10;
TC6.pos_init = [0; 0; -1000];
TC6.V_init = [cruise_u; 0; cruise_w];
TC6.eul_init_deg = [0; 0; 0];
TC6.omega_init = [0; 0; 0];
TC6.g = 9.81;
TC6.Fext_B = [0; 0; 0];
TC6.Mext_B = [0; 0; 0];
TC6.Motor_RPMs = cruise_rpms;
TC6.Tilt_angles = cruise_tilts;
TC6.purpose = 'Unpowered full-model response with gravity enabled.';
TC6.basis = 'Uses the same cruise-like initial condition as Cases 4-5, but with gravity turned on and motors off.';
TC6.expected = 'Aircraft should descend and adjust its attitude/velocity rather than hold level trim, because this is not a trimmed flight condition.';
TC6.note = 'Serves as a gravity-plus-aerodynamics sanity check without relying on the still-unvalidated propulsion model.';

cases = {TC1, TC2, TC3, TC4, TC5, TC6};
end

function cases = localFilterCases(all_cases, requested)
if isempty(requested)
    cases = all_cases;
    return;
end

if ischar(requested) || (isstring(requested) && isscalar(requested))
    requested = {char(string(requested))};
elseif isstring(requested)
    requested = cellstr(requested);
end

keep = false(size(all_cases));
for idx = 1:numel(all_cases)
    keep(idx) = any(strcmpi(all_cases{idx}.name, requested));
end

if ~any(keep)
    error('TESTER_external_forces_runner:NoCasesMatched', ...
        'No requested case names matched the available tester cases.');
end

cases = all_cases(keep);
end

function localDisplayCaseSummary(cases)
summary_table = localBuildSummaryTable(cases);
disp('Test case summary:');
disp(summary_table);
end

function summary_table = localBuildSummaryTable(cases)
summary_struct = repmat(struct( ...
    'Name', "", ...
    'Model', "", ...
    'Purpose', "", ...
    'Expected', "", ...
    'Basis', ""), numel(cases), 1);

for idx = 1:numel(cases)
    summary_struct(idx).Name = string(cases{idx}.name);
    summary_struct(idx).Model = string(cases{idx}.model);
    summary_struct(idx).Purpose = string(cases{idx}.purpose);
    summary_struct(idx).Expected = string(cases{idx}.expected);
    summary_struct(idx).Basis = string(cases{idx}.basis);
end

summary_table = struct2table(summary_struct);
end

function result = localRunFlightTest(case_def, aircraft, make_plots)
g_vec = localGravityVector(case_def.g);
case_tilt_deg = localRepresentativeFrontTilt(case_def.Tilt_angles);
case_flight_mode = double(case_tilt_deg >= 45);

disp('--------------------------------------------------');
disp(case_def.name);
disp('--------------------------------------------------');
fprintf('  Model:            %s\n', case_def.model);
fprintf('  Simulation Time:  %g s\n', case_def.t_stop);
fprintf('  Initial Position: [%g  %g  %g] m\n', case_def.pos_init);
fprintf('  Initial Velocity: [%g  %g  %g] m/s\n', case_def.V_init);
fprintf('  Initial Euler:    [%g  %g  %g] deg\n', case_def.eul_init_deg);
fprintf('  Initial Omega:    [%g  %g  %g] rad/s\n', case_def.omega_init);
fprintf('  Gravity:          [%g  %g  %g] m/s^2\n', g_vec);
fprintf('  Fext_B:           [%g  %g  %g] N\n', case_def.Fext_B);
fprintf('  Mext_B:           [%g  %g  %g] N-m\n', case_def.Mext_B);
fprintf('  Front Tilt:       %g deg\n', case_tilt_deg);
fprintf('  Purpose:          %s\n', case_def.purpose);
fprintf('  Expected:         %s\n', case_def.expected);
disp(' ');

assignin('base', 'Mass', aircraft.Mass);
assignin('base', 'm', aircraft.Mass);
assignin('base', 'CG', aircraft.CG);
assignin('base', 'J', aircraft.J);
assignin('base', 'rho', aircraft.rho);
assignin('base', 'wing', localModelSafeSurface(aircraft.wing));
assignin('base', 'wingL', localModelSafeSurface(aircraft.wingL));
assignin('base', 'wingR', localModelSafeSurface(aircraft.wingR));
assignin('base', 'tailL', localModelSafeSurface(aircraft.tailL));
assignin('base', 'tailR', localModelSafeSurface(aircraft.tailR));
assignin('base', 'prop', aircraft.prop);
assignin('base', 'compData', aircraft.compData);
assignin('base', 'aeroData', aircraft.aeroData);
assignin('base', 'flight_mode', case_flight_mode);
assignin('base', 'structural_tilt_angle', aircraft.tilt_angle);
assignin('base', 'tilt_angle', case_tilt_deg);
assignin('base', 'g', case_def.g);
assignin('base', 'pos_init', case_def.pos_init);
assignin('base', 'V_init', case_def.V_init);
assignin('base', 'eul_init', deg2rad(case_def.eul_init_deg(:)));
assignin('base', 'omega_init', case_def.omega_init);
assignin('base', 'Fext_B', case_def.Fext_B);
assignin('base', 'Mext_B', case_def.Mext_B);
assignin('base', 'Motor_RPMs', case_def.Motor_RPMs);
assignin('base', 'Tilt_angles', case_def.Tilt_angles);
assignin('base', 'controller_enable', false);
assignin('base', 'controller_base_rpm', mean(case_def.Motor_RPMs(:)));

out = sim(case_def.model, ...
    'StopTime', num2str(case_def.t_stop), ...
    'ReturnWorkspaceOutputs', 'on');

[omega, t_omega] = localGetTsData(out, 'omega');
[eul, t_eul] = localGetTsData(out, 'eul');
[v_b, t_v_b] = localGetTsData(out, 'v_b');
[position, t_pos] = localGetTsData(out, 'position');
[vinf, t_vinf] = localGetTsData(out, 'vinf');
[alpha, t_alpha] = localGetTsData(out, 'alpha');
[beta, t_beta] = localGetTsData(out, 'beta');

if ~isempty(eul)
    eul = rad2deg(eul);
end
if ~isempty(alpha)
    alpha = rad2deg(alpha);
end
if ~isempty(beta)
    beta = rad2deg(beta);
end

if make_plots
    figure('Name', case_def.name);
    sgtitle(case_def.name);

    subplot(2, 2, 1);
    plot(t_omega, omega, 'LineWidth', 1.5);
    grid on;
    title('Body Rates');
    ylabel('rad/s');
    legend('p', 'q', 'r', 'Location', 'best');

    subplot(2, 2, 2);
    plot(t_eul, eul, 'LineWidth', 1.5);
    grid on;
    title('Euler Angles');
    ylabel('deg');
    legend('\phi', '\theta', '\psi', 'Location', 'best');

    subplot(2, 2, 3);
    plot(t_v_b, v_b, 'LineWidth', 1.5);
    grid on;
    title('Body Velocity');
    ylabel('m/s');
    xlabel('Time (s)');
    legend('u', 'v', 'w', 'Location', 'best');

    subplot(2, 2, 4);
    plot(t_pos, position, 'LineWidth', 1.5);
    grid on;
    title('Position');
    ylabel('m');
    xlabel('Time (s)');
    legend('N', 'E', 'D', 'Location', 'best');

    if ~isempty(position)
        X = position(:, 1);
        Y = position(:, 2);
        Z = position(:, 3);

        figure('Name', [case_def.name, ' Trajectory']);
        plot3(X, Y, Z, 'b-', 'LineWidth', 2);
        grid on;
        hold on;
        set(gca, 'ZDir', 'reverse', 'YDir', 'reverse');
        plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
        plot3(X(end), Y(end), Z(end), 'rs', 'MarkerSize', 8, 'LineWidth', 2);
        xlabel('North [m]');
        ylabel('East [m]');
        zlabel('Down [m]');
        title([case_def.name, ' - Trajectory']);
        axis equal;
        xlim([min(X) - 100, max(X) + 100]);
        ylim([min(Y) - 100, max(Y) + 100]);
        zlim([min(Z) - 100, max(Z) + 100]);
    end

    if ~isempty(vinf) || ~isempty(alpha) || ~isempty(beta)
        figure('Name', [case_def.name, ' Air Data']);

        subplot(3, 1, 1);
        plot(t_vinf, vinf, 'LineWidth', 1.5);
        grid on;
        title('V_{inf}');
        ylabel('m/s');

        subplot(3, 1, 2);
        plot(t_alpha, alpha, 'LineWidth', 1.5);
        grid on;
        title('\alpha');
        ylabel('deg');

        subplot(3, 1, 3);
        plot(t_beta, beta, 'LineWidth', 1.5);
        grid on;
        title('\beta');
        ylabel('deg');
        xlabel('Time (s)');
    end
end

disp(case_def.note);

result = struct();
result.out = out;
result.omega = omega;
result.t_omega = t_omega;
result.eul_deg = eul;
result.t_eul = t_eul;
result.v_b = v_b;
result.t_v_b = t_v_b;
result.position = position;
result.t_pos = t_pos;
result.vinf = vinf;
result.t_vinf = t_vinf;
result.alpha_deg = alpha;
result.t_alpha = t_alpha;
result.beta_deg = beta;
result.t_beta = t_beta;
end

function g_vec = localGravityVector(g_value)
if isscalar(g_value)
    g_vec = [0; 0; g_value];
else
    g_vec = g_value(:);
end
end

function tilt_deg = localRepresentativeFrontTilt(Tilt_angles)
if isempty(Tilt_angles)
    tilt_deg = 0;
    return;
end

tilt_deg = mean(Tilt_angles(:));
end

function [data, t] = localGetTsData(out, var_name)
data = [];
t = [];

try
    ts = out.get(var_name);
catch
    return;
end

if isempty(ts)
    return;
end

t = ts.Time;
data = squeeze(ts.Data);

if isempty(data)
    return;
end

if isvector(data)
    data = data(:);
elseif size(data, 1) <= 3 && size(data, 2) > size(data, 1)
    data = data';
end
end

function surface = localModelSafeSurface(surface)
if isfield(surface, 'name')
    surface = rmfield(surface, 'name');
end
end
