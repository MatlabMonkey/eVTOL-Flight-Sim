function results = run_longitudinal_demo(varargin)
%RUN_LONGITUDINAL_DEMO Cruise-trim airspeed-hold step response demo.

p = inputParser;
p.addParameter('StopTime', 12, @(x) isnumeric(x) && isscalar(x));
p.addParameter('StepAirspeed', 5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('UseAVLAero', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('RebuildWrapper', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
ensure_control_wrapper_model('RebuildWrapper', p.Results.RebuildWrapper);
setup_cruise_trim_case('AirspeedStep', p.Results.StepAirspeed, 'BankStepDeg', 0);
apply_default_controller_gains();

assignin('base', 'controller_enable', true);
assignin('base', 'controller_mode', 3);
assignin('base', 'controller_bank_step', 0);
assignin('base', 'use_avl_aero', logical(p.Results.UseAVLAero));

simOut = sim('Brown_Control_Sim', 'StopTime', num2str(p.Results.StopTime));
[t, vinf] = extract_logged_signal(simOut, 'vinf');
[~, airspeed_cmd] = extract_logged_signal(simOut, 'airspeed_cmd');
[~, eul] = extract_logged_signal(simOut, 'eul');
[~, omega] = extract_logged_signal(simOut, 'omega');
controllerTs = evalin('base', 'controller_sample_time');
[t_ctrl, delta_e] = extract_logged_signal(simOut, 'delta_e_cmd', 'SampleTime', controllerTs);
[t_ctrl_motor, motor_cmd] = extract_logged_signal(simOut, 'motor_rpms_cmd', 'SampleTime', controllerTs);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 700]);
subplot(3,2,1); plot(t, vinf, t, airspeed_cmd, '--', 'LineWidth', 1.2); grid on; title('Airspeed');
xlabel('Time [s]'); ylabel('m/s'); legend('Measured', 'Command', 'Location', 'best');
subplot(3,2,2); plot(t, rad2deg(eul(:,2)), 'LineWidth', 1.2); grid on; title('Pitch Angle');
xlabel('Time [s]'); ylabel('deg');
subplot(3,2,3); plot(t, omega(:,2), 'LineWidth', 1.2); grid on; title('Pitch Rate q');
xlabel('Time [s]'); ylabel('rad/s');
subplot(3,2,4); plot(t_ctrl, rad2deg(delta_e), 'LineWidth', 1.2); grid on; title('Elevator Command');
xlabel('Time [s]'); ylabel('deg');
subplot(3,2,5); plot(t_ctrl_motor, motor_cmd(:,1:2), 'LineWidth', 1.2); grid on; title('Front Group Motor Commands');
xlabel('Time [s]'); ylabel('RPM');
subplot(3,2,6); plot(t_ctrl_motor, motor_cmd(:,3:4), 'LineWidth', 1.2); grid on; title('Rear Group Motor Commands');
xlabel('Time [s]'); ylabel('RPM');

outDir = localOutputDir();
figPath = fullfile(outDir, 'longitudinal_demo.png');
saveas(fig, figPath);
close(fig);

results = struct();
results.metrics = compute_step_metrics(t, vinf, airspeed_cmd);
results.figurePath = figPath;
results.t = t;
results.vinf = vinf;
results.airspeed_cmd = airspeed_cmd;
results.eul = eul;
results.omega = omega;
results.delta_e = delta_e;
results.motor_cmd = motor_cmd;
results.t_ctrl = t_ctrl;
results.t_ctrl_motor = t_ctrl_motor;
end

function outDir = localOutputDir()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'control_design');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
end
