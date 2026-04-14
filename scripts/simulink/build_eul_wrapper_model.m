function build_eul_wrapper_model(varargin)
%BUILD_EUL_WRAPPER_MODEL Build a clean wrapper around Brown_6DOF_Plant_EUL.

p = inputParser;
p.addParameter('OuterModel', 'Brown_6DOF_Sim_Wrapper_EUL', @(s) ischar(s) || isstring(s));
p.addParameter('PlantModel', 'Brown_6DOF_Plant_EUL', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');

model = char(p.Results.OuterModel);
plantModel = char(p.Results.PlantModel);
modelFile = fullfile(repoRoot, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
if exist(modelFile, 'file') ~= 0
    delete(modelFile);
end

new_system(model);
load_system(plantModel);
set_param(model, 'StopTime', '10');

add_block('simulink/Sources/Clock', [model '/Clock'], ...
    'Position', [35 55 65 75]);
add_block('simulink/Sources/Constant', [model '/Controller Enable'], ...
    'Value', 'controller_enable', ...
    'Position', [35 105 105 135]);
add_block('simulink/Sources/Constant', [model '/Controller Mode'], ...
    'Value', 'controller_mode', ...
    'Position', [35 155 105 185]);
add_block('simulink/Sources/Constant', [model '/Sensor Bias Enable'], ...
    'Value', 'sensor_enable_bias', ...
    'Position', [35 205 120 235]);
add_block('simulink/Sources/Constant', [model '/Sensor Noise Enable'], ...
    'Value', 'sensor_enable_noise', ...
    'Position', [35 255 120 285]);
add_block('simulink/Sources/Constant', [model '/Estimator Enable'], ...
    'Value', 'estimator_enable', ...
    'Position', [35 305 120 335]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Setpoint Maker'], ...
    'Position', [150 60 360 195]);
configureSetpointSubsystem([model '/Setpoint Maker']);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Controller'], ...
    'Position', [415 60 675 330]);
set_param([model '/Controller'], ...
    'TreatAsAtomicUnit', 'on', ...
    'SystemSampleTime', 'controller_sample_time');
configureWrapperControllerSubsystem([model '/Controller']);

add_block('simulink/Signal Routing/Bus Selector', [model '/Plant Cmd Select'], ...
    'OutputSignals', 'motor_rpm_cmd,tilt_angles_cmd,front_collective_rpm,rear_collective_rpm,delta_f,delta_a,delta_e,delta_r', ...
    'Position', [720 85 815 305]);

add_block('simulink/Ports & Subsystems/Model', [model '/Plant'], ...
    'ModelName', plantModel, ...
    'Position', [860 70 1040 355]);

truthNames = {'pos_NED','V_B_truth','V_E_truth','eul_truth','omega_truth', ...
    'C_NB_truth','V_BA_truth','vinf_truth','alpha_truth','beta_truth', ...
    'specific_force_truth'};
for k = 1:numel(truthNames)
    y = 70 + (k - 1) * 25;
    add_block('simulink/Discrete/Memory', [model '/Truth Memory ' num2str(k)], ...
        'Position', [1055 y 1085 y + 20]);
end

add_block('simulink/Signal Routing/Bus Creator', [model '/Truth Bus'], ...
    'Inputs', '11', ...
    'Position', [1085 75 1110 350]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Sensor Suite'], ...
    'Position', [1185 60 1435 280]);
set_param([model '/Sensor Suite'], ...
    'TreatAsAtomicUnit', 'on', ...
    'SystemSampleTime', 'sensor_sample_time');
configureSensorSuiteBusSubsystem([model '/Sensor Suite']);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Attitude Estimator'], ...
    'Position', [1185 325 1435 470]);
set_param([model '/Attitude Estimator'], ...
    'TreatAsAtomicUnit', 'on', ...
    'SystemSampleTime', 'estimator_sample_time');
configureEstimatorBusSubsystem([model '/Attitude Estimator']);

add_block('simulink/Sinks/To Workspace', [model '/eul_hat_log'], ...
    'VariableName', 'wrapper_eul_hat', ...
    'SaveFormat', 'Array', ...
    'Position', [1465 350 1560 375]);

wireWrapper(model);

save_system(model, modelFile);
close_system(model);

fprintf('Built %s around %s.\n', model, plantModel);
end

function wireWrapper(model)
phClock = get_param([model '/Clock'], 'PortHandles');
phSet = get_param([model '/Setpoint Maker'], 'PortHandles');
phCtrl = get_param([model '/Controller'], 'PortHandles');
phCmdSel = get_param([model '/Plant Cmd Select'], 'PortHandles');
phPlant = get_param([model '/Plant'], 'PortHandles');
phTruthBus = get_param([model '/Truth Bus'], 'PortHandles');
phSens = get_param([model '/Sensor Suite'], 'PortHandles');
phEst = get_param([model '/Attitude Estimator'], 'PortHandles');
phEulHatLog = get_param([model '/eul_hat_log'], 'PortHandles');
truthNames = {'pos_NED','V_B_truth','V_E_truth','eul_truth','omega_truth', ...
    'C_NB_truth','V_BA_truth','vinf_truth','alpha_truth','beta_truth', ...
    'specific_force_truth'};

add_line(model, phClock.Outport(1), get_param([model '/Setpoint Maker'], 'PortHandles').Inport(1), 'autorouting', 'on');
add_line(model, phClock.Outport(1), get_param([model '/Controller'], 'PortHandles').Inport(1), 'autorouting', 'on');
add_line(model, phClock.Outport(1), get_param([model '/Attitude Estimator'], 'PortHandles').Inport(1), 'autorouting', 'on');

add_line(model, 'Controller Enable/1', 'Controller/2', 'autorouting', 'on');
add_line(model, 'Controller Mode/1', 'Controller/3', 'autorouting', 'on');
add_line(model, phSet.Outport(1), get_param([model '/Controller'], 'PortHandles').Inport(6), 'autorouting', 'on');

add_line(model, phCtrl.Outport(1), phCmdSel.Inport(1), 'autorouting', 'on');
for k = 1:8
    add_line(model, phCmdSel.Outport(k), phPlant.Inport(k), 'autorouting', 'on');
end

for k = 1:11
    phMem = get_param([model '/Truth Memory ' num2str(k)], 'PortHandles');
    lh = add_line(model, phPlant.Outport(k), phMem.Inport(1), 'autorouting', 'on');
    set_param(lh, 'Name', [truthNames{k} '_raw']);
    lh = add_line(model, phMem.Outport(1), phTruthBus.Inport(k), 'autorouting', 'on');
    set_param(lh, 'Name', truthNames{k});
end
add_line(model, phTruthBus.Outport(1), get_param([model '/Sensor Suite'], 'PortHandles').Inport(1), 'autorouting', 'on');

add_line(model, 'Sensor Bias Enable/1', 'Sensor Suite/2', 'autorouting', 'on');
add_line(model, 'Sensor Noise Enable/1', 'Sensor Suite/3', 'autorouting', 'on');
add_line(model, phSens.Outport(1), get_param([model '/Attitude Estimator'], 'PortHandles').Inport(3), 'autorouting', 'on');
add_line(model, phSens.Outport(1), get_param([model '/Controller'], 'PortHandles').Inport(4), 'autorouting', 'on');

add_line(model, 'Estimator Enable/1', 'Attitude Estimator/2', 'autorouting', 'on');
add_line(model, phEst.Outport(1), get_param([model '/Controller'], 'PortHandles').Inport(5), 'autorouting', 'on');
add_line(model, phEst.Outport(1), phEulHatLog.Inport(1), 'autorouting', 'on');
end

function configureSetpointSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

add_block('simulink/Sources/In1', [subsysPath '/t'], ...
    'Position', [25 58 55 72]);
constNames = {'controller_airspeed_cmd_base','controller_bank_cmd_base', ...
    'controller_step_time','controller_airspeed_step','controller_bank_step'};
for idx = 1:numel(constNames)
    y = 20 + (idx - 1) * 25;
    add_block('simulink/Sources/Constant', [subsysPath '/' constNames{idx}], ...
        'Value', constNames{idx}, ...
        'Position', [70 y 170 y + 20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Setpoints'], ...
    'Position', [210 45 380 120]);
setMatlabFunctionScript([subsysPath '/Setpoints'], sprintf([ ...
    'function [airspeed_cmd, bank_cmd] = fcn(t, airspeed_cmd_base, bank_cmd_base, step_time, airspeed_step, bank_step)\n' ...
    '%%#codegen\n' ...
    '[airspeed_cmd, bank_cmd] = setpoint_schedule_step(t, airspeed_cmd_base, bank_cmd_base, step_time, airspeed_step, bank_step);\n' ...
    'end\n']));

add_block('simulink/Signal Routing/Bus Creator', [subsysPath '/Bus'], ...
    'Inputs', '2', ...
    'Position', [440 50 465 100]);
add_block('simulink/Sinks/Out1', [subsysPath '/setpoint_bus'], ...
    'Position', [505 68 535 82]);

add_line(subsysPath, 't/1', 'Setpoints/1', 'autorouting', 'on');
for idx = 1:numel(constNames)
    add_line(subsysPath, [constNames{idx} '/1'], ['Setpoints/' num2str(idx + 1)], 'autorouting', 'on');
end
lh = add_line(subsysPath, 'Setpoints/1', 'Bus/1', 'autorouting', 'on');
set_param(lh, 'Name', 'airspeed_cmd');
lh = add_line(subsysPath, 'Setpoints/2', 'Bus/2', 'autorouting', 'on');
set_param(lh, 'Name', 'bank_cmd');
add_line(subsysPath, 'Bus/1', 'setpoint_bus/1', 'autorouting', 'on');
end

function configureSensorSuiteBusSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

add_block('simulink/Sources/In1', [subsysPath '/truth_bus'], ...
    'Position', [20 118 50 132]);
add_block('simulink/Sources/In1', [subsysPath '/enable_bias'], ...
    'Position', [20 158 50 172]);
add_block('simulink/Sources/In1', [subsysPath '/enable_noise'], ...
    'Position', [20 198 50 212]);

add_block('simulink/Signal Routing/Bus Selector', [subsysPath '/Truth Select'], ...
    'OutputSignals', 'vinf_truth,alpha_truth,beta_truth,omega_truth,specific_force_truth,pos_NED,V_E_truth,eul_truth,C_NB_truth', ...
    'Position', [85 70 165 230]);

constNames = { ...
    'sensor_airdata_bias', 'sensor_gyro_bias', 'sensor_accel_bias', ...
    'sensor_gps_pos_bias', 'sensor_gps_vel_bias', 'sensor_mag_bias', ...
    'sensor_attitude_bias', 'sensor_airdata_sigma', 'sensor_gyro_sigma', ...
    'sensor_accel_sigma', 'sensor_gps_pos_sigma', 'sensor_gps_vel_sigma', ...
    'sensor_mag_sigma', 'sensor_attitude_sigma', 'mag_ned'};
for idx = 1:numel(constNames)
    y = 20 + (idx - 1) * 18;
    add_block('simulink/Sources/Constant', [subsysPath '/' constNames{idx}], ...
        'Value', constNames{idx}, ...
        'Position', [190 y 285 y + 18]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Sensor Eval'], ...
    'Position', [320 55 560 245]);
setMatlabFunctionScript([subsysPath '/Sensor Eval'], sprintf([ ...
    'function [airspeed_meas, alpha_meas, beta_meas, gyro_meas, accel_meas, gps_pos_meas, gps_vel_meas, mag_body_meas, attitude_meas] = fcn(vinf_truth, alpha_truth, beta_truth, omega_truth, specific_force_truth, pos_truth, vel_truth, eul_truth, C_NB_truth, enable_bias, enable_noise, airdata_bias, gyro_bias, accel_bias, gps_pos_bias, gps_vel_bias, mag_bias, attitude_bias, airdata_sigma, gyro_sigma, accel_sigma, gps_pos_sigma, gps_vel_sigma, mag_sigma, attitude_sigma, mag_ned)\n' ...
    '%%#codegen\n' ...
    '[airspeed_meas, alpha_meas, beta_meas, gyro_meas, accel_meas, gps_pos_meas, gps_vel_meas, mag_body_meas, attitude_meas] = sensor_suite_eval(vinf_truth, alpha_truth, beta_truth, omega_truth, specific_force_truth, pos_truth, vel_truth, eul_truth, C_NB_truth, enable_bias, enable_noise, airdata_bias, gyro_bias, accel_bias, gps_pos_bias, gps_vel_bias, mag_bias, attitude_bias, airdata_sigma, gyro_sigma, accel_sigma, gps_pos_sigma, gps_vel_sigma, mag_sigma, attitude_sigma, mag_ned);\n' ...
    'end\n']));

outNames = {'airspeed_meas','alpha_meas','beta_meas','gyro_meas','accel_meas', ...
    'gps_pos_meas','gps_vel_meas','mag_body_meas','attitude_meas'};
add_block('simulink/Signal Routing/Bus Creator', [subsysPath '/Bus'], ...
    'Inputs', num2str(numel(outNames)), ...
    'Position', [610 55 635 255]);
add_block('simulink/Sinks/Out1', [subsysPath '/sensor_bus'], ...
    'Position', [670 148 700 162]);

add_line(subsysPath, 'truth_bus/1', 'Truth Select/1', 'autorouting', 'on');
for idx = 1:9
    add_line(subsysPath, ['Truth Select/' num2str(idx)], ['Sensor Eval/' num2str(idx)], 'autorouting', 'on');
end
add_line(subsysPath, 'enable_bias/1', 'Sensor Eval/10', 'autorouting', 'on');
add_line(subsysPath, 'enable_noise/1', 'Sensor Eval/11', 'autorouting', 'on');
for idx = 1:numel(constNames)
    add_line(subsysPath, [constNames{idx} '/1'], ['Sensor Eval/' num2str(11 + idx)], 'autorouting', 'on');
end
for idx = 1:numel(outNames)
    lh = add_line(subsysPath, ['Sensor Eval/' num2str(idx)], ['Bus/' num2str(idx)], 'autorouting', 'on');
    set_param(lh, 'Name', outNames{idx});
end
add_line(subsysPath, 'Bus/1', 'sensor_bus/1', 'autorouting', 'on');
end

function configureEstimatorBusSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

add_block('simulink/Sources/In1', [subsysPath '/t'], ...
    'Position', [20 48 50 62]);
add_block('simulink/Sources/In1', [subsysPath '/enable'], ...
    'Position', [20 88 50 102]);
add_block('simulink/Sources/In1', [subsysPath '/sensor_bus'], ...
    'Position', [20 128 50 142]);

add_block('simulink/Signal Routing/Bus Selector', [subsysPath '/Sensor Select'], ...
    'OutputSignals', 'gyro_meas,accel_meas,mag_body_meas,attitude_meas', ...
    'Position', [85 90 165 185]);

constNames = {'estimator_init_eul','estimator_k_acc','estimator_k_mag','estimator_k_att'};
for idx = 1:numel(constNames)
    y = 40 + (idx - 1) * 30;
    add_block('simulink/Sources/Constant', [subsysPath '/' constNames{idx}], ...
        'Value', constNames{idx}, ...
        'Position', [190 y 280 y + 20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Estimator'], ...
    'Position', [320 70 510 175]);
setMatlabFunctionScript([subsysPath '/Estimator'], sprintf([ ...
    'function eul_hat = fcn(t, enable, gyro_meas, accel_meas, mag_body_meas, attitude_meas, init_eul, k_acc, k_mag, k_att)\n' ...
    '%%#codegen\n' ...
    'eul_hat = attitude_complementary_step(t, enable, gyro_meas, accel_meas, mag_body_meas, attitude_meas, init_eul, k_acc, k_mag, k_att);\n' ...
    'end\n']));

add_block('simulink/Sinks/Out1', [subsysPath '/eul_hat'], ...
    'Position', [550 113 580 127]);

add_line(subsysPath, 't/1', 'Estimator/1', 'autorouting', 'on');
add_line(subsysPath, 'enable/1', 'Estimator/2', 'autorouting', 'on');
add_line(subsysPath, 'sensor_bus/1', 'Sensor Select/1', 'autorouting', 'on');
for idx = 1:4
    add_line(subsysPath, ['Sensor Select/' num2str(idx)], ['Estimator/' num2str(idx + 2)], 'autorouting', 'on');
end
for idx = 1:numel(constNames)
    add_line(subsysPath, [constNames{idx} '/1'], ['Estimator/' num2str(idx + 6)], 'autorouting', 'on');
end
add_line(subsysPath, 'Estimator/1', 'eul_hat/1', 'autorouting', 'on');
end

function configureWrapperControllerSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

inNames = {'t','enable','mode','sensor_bus','eul_hat','setpoint_bus'};
for idx = 1:numel(inNames)
    y = 30 + (idx - 1) * 28;
    add_block('simulink/Sources/In1', [subsysPath '/' inNames{idx}], ...
        'Position', [20 y 50 y + 14]);
end

add_block('simulink/Signal Routing/Bus Selector', [subsysPath '/Sensor Select'], ...
    'OutputSignals', 'airspeed_meas,gyro_meas', ...
    'Position', [85 95 165 150]);
add_block('simulink/Signal Routing/Bus Selector', [subsysPath '/Setpoint Select'], ...
    'OutputSignals', 'airspeed_cmd,bank_cmd', ...
    'Position', [85 180 165 235]);

constNames = { ...
    'controller_trim_motor_rpm_cmd', 'controller_trim_tilt_angles_cmd', ...
    'trim_front_collective_rpm', 'trim_rear_collective_rpm', ...
    'controller_trim_delta_f', 'controller_trim_delta_a', ...
    'controller_trim_delta_e', 'controller_trim_delta_r', ...
    'controller_k_p_damp', 'controller_k_q_damp', 'controller_k_r_damp', ...
    'controller_k_phi_p', 'controller_k_phi_i', 'controller_k_v_p', ...
    'controller_k_v_i', 'controller_surface_limit_rad', ...
    'controller_rpm_delta_limit', 'controller_integrator_limit'};
for idx = 1:numel(constNames)
    y = 20 + (idx - 1) * 16;
    add_block('simulink/Sources/Constant', [subsysPath '/' constNames{idx}], ...
        'Value', constNames{idx}, ...
        'Position', [190 y 300 y + 18]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Controller'], ...
    'Position', [335 70 615 250]);
setMatlabFunctionScript([subsysPath '/Controller'], sprintf([ ...
    'function [motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm, delta_f, delta_a, delta_e, delta_r] = fcn(t, enable, mode, airspeed_meas, gyro_meas, eul_hat, airspeed_cmd, bank_cmd, trim_motor_rpm_cmd, trim_tilt_angles_cmd, trim_front_collective_rpm, trim_rear_collective_rpm, trim_delta_f, trim_delta_a, trim_delta_e, trim_delta_r, k_p_damp, k_q_damp, k_r_damp, k_phi_p, k_phi_i, k_v_p, k_v_i, surface_limit_rad, rpm_delta_limit, integrator_limit)\n' ...
    '%%#codegen\n' ...
    '[motor_rpm_cmd, tilt_angles_cmd, front_collective_rpm, rear_collective_rpm, delta_f, delta_a, delta_e, delta_r] = wrapper_controller_actual_cmd_step(t, enable, mode, airspeed_meas, gyro_meas, eul_hat, airspeed_cmd, bank_cmd, trim_motor_rpm_cmd, trim_tilt_angles_cmd, trim_front_collective_rpm, trim_rear_collective_rpm, trim_delta_f, trim_delta_a, trim_delta_e, trim_delta_r, k_p_damp, k_q_damp, k_r_damp, k_phi_p, k_phi_i, k_v_p, k_v_i, surface_limit_rad, rpm_delta_limit, integrator_limit);\n' ...
    'end\n']));

outNames = {'motor_rpm_cmd','tilt_angles_cmd','front_collective_rpm','rear_collective_rpm','delta_f','delta_a','delta_e','delta_r'};
add_block('simulink/Signal Routing/Bus Creator', [subsysPath '/Bus'], ...
    'Inputs', num2str(numel(outNames)), ...
    'Position', [650 45 675 245]);
add_block('simulink/Sinks/Out1', [subsysPath '/plant_cmd_bus'], ...
    'Position', [710 138 740 152]);

add_line(subsysPath, 't/1', 'Controller/1', 'autorouting', 'on');
add_line(subsysPath, 'enable/1', 'Controller/2', 'autorouting', 'on');
add_line(subsysPath, 'mode/1', 'Controller/3', 'autorouting', 'on');
add_line(subsysPath, 'sensor_bus/1', 'Sensor Select/1', 'autorouting', 'on');
add_line(subsysPath, 'setpoint_bus/1', 'Setpoint Select/1', 'autorouting', 'on');
add_line(subsysPath, 'eul_hat/1', 'Controller/6', 'autorouting', 'on');
add_line(subsysPath, 'Sensor Select/1', 'Controller/4', 'autorouting', 'on');
add_line(subsysPath, 'Sensor Select/2', 'Controller/5', 'autorouting', 'on');
add_line(subsysPath, 'Setpoint Select/1', 'Controller/7', 'autorouting', 'on');
add_line(subsysPath, 'Setpoint Select/2', 'Controller/8', 'autorouting', 'on');
for idx = 1:numel(constNames)
    add_line(subsysPath, [constNames{idx} '/1'], ['Controller/' num2str(idx + 8)], 'autorouting', 'on');
end
for idx = 1:numel(outNames)
    lh = add_line(subsysPath, ['Controller/' num2str(idx)], ['Bus/' num2str(idx)], 'autorouting', 'on');
    set_param(lh, 'Name', outNames{idx});
end
add_line(subsysPath, 'Bus/1', 'plant_cmd_bus/1', 'autorouting', 'on');
end

function clearSubsystemContents(subsysPath)
contents = find_system(subsysPath, 'SearchDepth', 1, 'Type', 'Block');
for idx = 1:numel(contents)
    if strcmp(contents{idx}, subsysPath)
        continue;
    end
    delete_block(contents{idx});
end
end

function setMatlabFunctionScript(blockPath, scriptText)
sfRoot = sfroot;
chart = find(sfRoot, '-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('build_eul_wrapper_model:MissingChart', ...
        'Could not find MATLAB Function chart at %s', blockPath);
end
chart.Script = scriptText;
end
