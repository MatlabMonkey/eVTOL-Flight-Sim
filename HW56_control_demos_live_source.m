%% Homework 5 and 6 Control Demos

clear; clc;
addpath(genpath(fullfile(pwd,'scripts')));

render_enable = false;
run('Full_Sim_Init.m');

[longDemo, latDemo] = localRunDemos();

%% Longitudinal
figure('Color','w','Position',[100 100 950 700]);
tiledlayout(3,1);

nexttile;
plot(longDemo.t, longDemo.vinf, 'LineWidth', 1.3); hold on;
plot(longDemo.t, longDemo.airspeed_cmd, '--', 'LineWidth', 1.2);
grid on;
ylabel('m/s');
title('Longitudinal Response');
legend('airspeed','command','Location','best');

nexttile;
plot(longDemo.t, rad2deg(longDemo.eul(:,2)), 'LineWidth', 1.3); hold on;
plot(longDemo.t, longDemo.omega(:,2), 'LineWidth', 1.1);
grid on;
ylabel('deg / rad s^{-1}');
legend('\theta','q','Location','best');

nexttile;
plot(longDemo.t_ctrl, rad2deg(longDemo.delta_e), 'LineWidth', 1.3); hold on;
plot(longDemo.t_ctrl_motor, longDemo.motor_cmd(:,1), 'LineWidth', 1.1);
plot(longDemo.t_ctrl_motor, longDemo.motor_cmd(:,3), 'LineWidth', 1.1);
grid on;
xlabel('Time [s]');
ylabel('cmd');
legend('\delta_e [deg]','front motor group','rear motor group','Location','best');

%% Lateral
figure('Color','w','Position',[100 100 950 750]);
tiledlayout(4,1);

nexttile;
plot(latDemo.t_true, rad2deg(latDemo.true_bank), 'LineWidth', 1.2); hold on;
plot(latDemo.t, rad2deg(latDemo.eul_hat(:,1)), 'LineWidth', 1.2);
plot(latDemo.t, rad2deg(latDemo.bank_cmd), '--', 'LineWidth', 1.2);
grid on;
ylabel('deg');
title('Lateral Response');
legend('\phi','\phi_{hat}','command','Location','best');

nexttile;
plot(latDemo.t_true, rad2deg(latDemo.heading), 'LineWidth', 1.2); hold on;
plot(latDemo.t_true, latDemo.p, 'LineWidth', 1.1);
grid on;
ylabel('deg / rad s^{-1}');
legend('\psi','p','Location','best');

nexttile;
plot(latDemo.t_true, latDemo.r, 'LineWidth', 1.2);
grid on;
ylabel('rad/s');
legend('r','Location','best');

nexttile;
plot(latDemo.t_ctrl, rad2deg(latDemo.delta_a), 'LineWidth', 1.2); hold on;
plot(latDemo.t_ctrl, rad2deg(latDemo.delta_r), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('deg');
legend('\delta_a','\delta_r','Location','best');

%% Metrics
metricNames = {'rise_time_s','overshoot_pct','settling_time_s'};
metricValues = [ ...
    longDemo.metrics.rise_time_s, longDemo.metrics.overshoot_pct, longDemo.metrics.settling_time_s; ...
    latDemo.metrics.rise_time_s,  latDemo.metrics.overshoot_pct,  latDemo.metrics.settling_time_s];

metricsTable = array2table(metricValues, ...
    'VariableNames', metricNames, ...
    'RowNames', {'longitudinal','lateral'});

metricsTable

function [longDemo, latDemo] = localRunDemos()
ensure_control_wrapper_model();

longDemo = run_longitudinal_demo( ...
    'StopTime', 2.5, ...
    'StepAirspeed', 8, ...
    'RebuildWrapper', false);

latDemo = run_lateral_demo( ...
    'StopTime', 4.0, ...
    'StepBankDeg', 5, ...
    'EstimatorKAcc', 0.0, ...
    'EstimatorKMag', 0.0, ...
    'EstimatorKAtt', 5.0, ...
    'RebuildWrapper', false);
end
