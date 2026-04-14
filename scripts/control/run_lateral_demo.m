function results = run_lateral_demo(varargin)
%RUN_LATERAL_DEMO Cruise-trim bank-angle-hold step response demo.

p = inputParser;
p.addParameter('StopTime', 4, @(x) isnumeric(x) && isscalar(x));
p.addParameter('StepBankDeg', 5, @(x) isnumeric(x) && isscalar(x));
p.addParameter('UseAVLAero', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('EstimatorKAcc', 0.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('EstimatorKMag', 0.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('EstimatorKAtt', 5.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('RebuildWrapper', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
ensure_control_wrapper_model('RebuildWrapper', p.Results.RebuildWrapper);
setup_cruise_trim_case('AirspeedStep', 0, 'BankStepDeg', p.Results.StepBankDeg);
apply_default_controller_gains();

assignin('base', 'controller_enable', true);
assignin('base', 'controller_mode', 3);
assignin('base', 'controller_airspeed_step', 0);
assignin('base', 'use_avl_aero', logical(p.Results.UseAVLAero));
assignin('base', 'estimator_k_acc', p.Results.EstimatorKAcc);
assignin('base', 'estimator_k_mag', p.Results.EstimatorKMag);
assignin('base', 'estimator_k_att', p.Results.EstimatorKAtt);

simOut = sim('Brown_Control_Sim', 'StopTime', num2str(p.Results.StopTime));
[t_truth, bank_cmd_truth] = extract_logged_signal(simOut, 'bank_cmd');
[~, eul] = extract_logged_signal(simOut, 'eul');
[~, omega] = extract_logged_signal(simOut, 'omega');
controllerTs = evalin('base', 'controller_sample_time');
estimatorTs = evalin('base', 'estimator_sample_time');
[t_est, eul_hat] = extract_logged_signal(simOut, 'eul_hat', 'SampleTime', estimatorTs);
[t_ctrl, delta_a] = extract_logged_signal(simOut, 'delta_a_cmd', 'SampleTime', controllerTs);
[~, delta_r] = extract_logged_signal(simOut, 'delta_r_cmd', 'SampleTime', controllerTs);
[t_true, bank_true] = extract_logged_signal(simOut, 'eul');
[t_bank_true, bank_cmd_true_interp] = deal(t_truth, bank_cmd_truth);
bank_cmd_est = interp1(t_bank_true, bank_cmd_true_interp, t_est, 'previous', 'extrap');

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 700]);
subplot(3,2,1); plot(t_est, rad2deg(eul_hat(:,1)), t_est, rad2deg(bank_cmd_est), '--', 'LineWidth', 1.2); grid on; title('Estimated Bank Angle');
xlabel('Time [s]'); ylabel('deg'); legend('\phi_{hat}', 'Command', 'Location', 'best');
subplot(3,2,2); plot(t_true, rad2deg(eul(:,3)), 'LineWidth', 1.2); grid on; title('Heading');
xlabel('Time [s]'); ylabel('deg');
subplot(3,2,3); plot(t_true, omega(:,1), 'LineWidth', 1.2); grid on; title('Roll Rate p');
xlabel('Time [s]'); ylabel('rad/s');
subplot(3,2,4); plot(t_true, omega(:,3), 'LineWidth', 1.2); grid on; title('Yaw Rate r');
xlabel('Time [s]'); ylabel('rad/s');
subplot(3,2,5); plot(t_ctrl, rad2deg(delta_a), 'LineWidth', 1.2); grid on; title('Aileron Command');
xlabel('Time [s]'); ylabel('deg');
subplot(3,2,6); plot(t_ctrl, rad2deg(delta_r), t_true, rad2deg(eul(:,1)), 'LineWidth', 1.2); grid on; title('Rudder Command and True Bank');
xlabel('Time [s]'); ylabel('deg');
legend('\delta_r', '\phi', 'Location', 'best');

outDir = localOutputDir();
figPath = fullfile(outDir, 'lateral_demo.png');
saveas(fig, figPath);
close(fig);

results = struct();
results.metrics = compute_step_metrics(t_est, eul_hat(:,1), bank_cmd_est);
results.figurePath = figPath;
results.t = t_est;
results.eul_hat = eul_hat;
results.bank_cmd = bank_cmd_est;
results.eul = eul;
results.omega = omega;
results.delta_a = delta_a;
results.delta_r = delta_r;
results.true_bank = eul(:,1);
results.heading = eul(:,3);
results.p = omega(:,1);
results.r = omega(:,3);
results.t_true = t_true;
results.t_ctrl = t_ctrl;
end

function outDir = localOutputDir()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'control_design');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
end
