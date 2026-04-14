%% Homework 3 and 4 Sensors and Root Locus

clear; clc;
addpath(genpath(fullfile(pwd,'scripts')));

render_enable = false;
run('Full_Sim_Init.m');

[sensorCheck, linData, damperData, pdData, pidData, gains] = localRunHw34();

imgDir = fullfile(pwd, 'docs', 'control_design');
wrapperImg = fullfile(imgDir, 'Brown_Control_Sim.png');
localExportModelImage('Brown_Control_Sim', wrapperImg);

%% Wrapper
figure('Color','w','Position',[100 100 1200 520]);
localShowImage(wrapperImg);
title('Brown Control Sim');

%% Sensors
figure('Color','w','Position',[100 100 1000 720]);
tiledlayout(2,2);

nexttile;
plot(sensorCheck.t_truth, sensorCheck.truth.vinf, 'LineWidth', 1.2); hold on;
plot(sensorCheck.t_meas, sensorCheck.perfect.airspeed, '--', 'LineWidth', 1.2);
grid on;
title('Airspeed');
legend('truth','measurement','Location','best');

nexttile;
plot(sensorCheck.t_truth, rad2deg(sensorCheck.truth.alpha), 'LineWidth', 1.2); hold on;
plot(sensorCheck.t_meas, rad2deg(sensorCheck.perfect.alpha), '--', 'LineWidth', 1.2);
grid on;
title('Alpha');
legend('truth','measurement','Location','best');

nexttile;
plot(sensorCheck.t_truth, sensorCheck.truth.omega(:,1), 'LineWidth', 1.2); hold on;
plot(sensorCheck.t_meas, sensorCheck.perfect.gyro(:,1), '--', 'LineWidth', 1.2);
grid on;
title('Roll Rate Gyro');
legend('truth','measurement','Location','best');

nexttile;
plot(sensorCheck.t_truth, rad2deg(sensorCheck.truth.eul(:,2)), 'LineWidth', 1.2); hold on;
plot(sensorCheck.t_meas, rad2deg(sensorCheck.perfect.attitude(:,2)), '--', 'LineWidth', 1.2);
grid on;
title('Pitch Attitude');
legend('truth','measurement','Location','best');

figure('Color','w','Position',[100 100 1000 420]);
tiledlayout(1,2);

nexttile;
plot(sensorCheck.t_truth, sensorCheck.truth.vinf, 'LineWidth', 1.2); hold on;
plot(sensorCheck.t_meas, sensorCheck.perfect.airspeed, '--', 'LineWidth', 1.2);
plot(sensorCheck.t_meas, sensorCheck.biased.airspeed, ':', 'LineWidth', 1.4);
grid on;
title('Airspeed Bias Check');
legend('truth','perfect','bias on','Location','best');

nexttile;
plot(sensorCheck.t_truth, sensorCheck.truth.omega(:,1), 'LineWidth', 1.2); hold on;
plot(sensorCheck.t_meas, sensorCheck.perfect.gyro(:,1), '--', 'LineWidth', 1.2);
plot(sensorCheck.t_meas, sensorCheck.biased.gyro(:,1), ':', 'LineWidth', 1.4);
grid on;
title('Gyro Bias Check');
legend('truth','perfect','bias on','Location','best');

sensorTable = table( ...
    ["airspeed";"alpha";"beta";"gyros";"accelerometers";"GPS";"magnetometer";"attitude"], ...
    ["yes";"yes";"yes";"yes";"yes";"yes";"yes";"yes"], ...
    repmat("startup flag",8,1), ...
    'VariableNames', {'sensor','implemented','bias_control'});

sensorTable

%% Eigenvalues and root locus
eigTable = table(real(linData.openLoopEig), imag(linData.openLoopEig), ...
    'VariableNames', {'real_part','imag_part'});

eigTable

figure('Color','w','Position',[100 100 1200 900]);
localShowImage(damperData.figurePath);
title('Damper Root Locus');

figure('Color','w','Position',[100 100 1100 500]);
localShowImage(pdData.figurePath);
title('PD Root Locus');

figure('Color','w','Position',[100 100 1100 500]);
localShowImage(pidData.figurePath);
title('PI-D Root Locus');

gainTable = struct2table(gains);
gainTable

function localExportModelImage(modelName, outPath)
open_system(modelName);
set_param(modelName, 'ZoomFactor', 'FitSystem');
print(['-s' modelName], '-dpng', outPath);
close_system(modelName);
end

function localShowImage(imgPath)
img = imread(imgPath);
image(img);
axis image off;
end

function [sensorCheck, linData, damperData, pdData, pidData, gains] = localRunHw34()
ensure_control_wrapper_model();
ensure_control_analysis_helper();

sensorCheck = run_sensor_checkout('StopTime', 2.0, 'BiasAirspeedMps', 2.0, 'BiasRollRateRadS', 0.02, 'RebuildWrapper', false);
linData = linearize_cruise_plant('CruiseSpeed', 70);
damperData = design_dampers('CruiseSpeed', 70);
pdData = design_pd('CruiseSpeed', 70);
pidData = design_pid('CruiseSpeed', 70);
gains = apply_default_controller_gains();
end
