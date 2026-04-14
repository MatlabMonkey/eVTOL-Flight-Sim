function build_model_reference_architecture(varargin)
%BUILD_MODEL_REFERENCE_ARCHITECTURE Split plant/reference and outer wrapper.
%   This script:
%   1) clones the current plant into Brown_6DOF_Plant.slx,
%   2) patches that model so it accepts mixed actuator commands and exposes
%      truth signals, and
%   3) rebuilds an outer wrapper containing sensors, estimator, controller,
%      commands, and logging.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

p = inputParser;
p.addParameter('RebuildPlant', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('OuterModel', 'Brown_Full_Sim', @(s) ischar(s) || isstring(s));
p.addParameter('PlantModel', 'Brown_6DOF_Plant', @(s) ischar(s) || isstring(s));
p.addParameter('PlantSourceModel', 'Brown_Full_Sim', @(s) ischar(s) || isstring(s));
p.addParameter('PlantSourceRef', 'HEAD', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});

outerModel = char(p.Results.OuterModel);
plantModel = char(p.Results.PlantModel);
plantSourceModel = char(p.Results.PlantSourceModel);
plantSourceRef = char(p.Results.PlantSourceRef);
plantFile = fullfile(repoRoot, [plantModel '.slx']);
sourceFile = fullfile(tempdir, [plantSourceModel '_source.slx']);

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');

needPlantBuild = logical(p.Results.RebuildPlant) || ...
    exist(plantFile, 'file') == 0 || ...
    plantLooksLikeWrapper(plantModel, plantFile);

if needPlantBuild
    exportTrackedModelSource(repoRoot, plantSourceModel, sourceFile, plantSourceRef);
    bdclose('all');
    copyfile(sourceFile, plantFile, 'f');
    patchPlantModel(plantModel);
    bdclose('all');
end

rebuildOuterWrapper(outerModel, plantModel);

fprintf('Built %s as the outer wrapper and %s as the referenced plant.\n', ...
    outerModel, plantModel);
end

function patchPlantModel(model)
load_system(model);

% Remove outer-only blocks.
deleteIfExists(model, ' controller_base_rpm');
deleteIfExists(model, ' controller_enable');
deleteIfExists(model, 'Simple PD Controller');
deleteIfExists(model, 'FlightGear Interface');
deleteIfExists(model, 'Mux1');
deleteIfExists(model, 'Flat Earth to LLA');
deleteIfExists(model, 'h_ref');
deleteIfExists(model, 'To Workspace');
deleteIfExists(model, 'To Workspace1');
deleteIfExists(model, 'To Workspace2');
deleteIfExists(model, 'To Workspace3');
deleteIfExists(model, 'To Workspace4');
deleteIfExists(model, 'To Workspace5');
deleteIfExists(model, 'To Workspace6');
deleteIfExists(model, 'To Workspace7');
deleteIfExists(model, 'To Workspace AVL Force');
deleteIfExists(model, 'To Workspace AVL Moment');
deleteIfExists(model, 'To Workspace Aero Mode');
deleteIfExists(model, 'To Workspace Analytic Force');
deleteIfExists(model, 'To Workspace Analytic Moment');
deleteIfExists(model, 'To Workspace Selected Force');
deleteIfExists(model, 'To Workspace Selected Moment');

replaceWithInport(model, ' Motor_RPMs', 'Motor_RPM_cmd');
replaceWithInport(model, ' Tilt Angles', 'Tilt_angles_cmd');
set_param([model '/Motor_RPM_cmd'], 'PortDimensions', '12');
set_param([model '/Tilt_angles_cmd'], 'PortDimensions', '6');
deleteIfExists(model, 'Front_RPM_collective');
deleteIfExists(model, 'Rear_RPM_collective1');
deleteIfExists(model, 'RPM Command Combine');
add_block('simulink/Sources/In1', [model '/Front_RPM_collective'], ...
    'Position', [-1210 -255 -1180 -241]);
add_block('simulink/Sources/In1', [model '/Rear_RPM_collective1'], ...
    'Position', [-1210 -225 -1180 -211]);
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/RPM Command Combine'], ...
    'Position', [-1125 -260 -975 -180]);
setMatlabFunctionScript([model '/RPM Command Combine'], sprintf([ ...
    'function motor_rpm_total = fcn(motor_rpm_cmd, front_collective, rear_collective)\n' ...
    '%%#codegen\n' ...
    'motor_rpm_total = motor_rpm_cmd(:);\n' ...
    'if numel(motor_rpm_total) ~= 12\n' ...
    '    motor_rpm_total = zeros(12, 1);\n' ...
    'end\n' ...
    'motor_rpm_total(1:6) = motor_rpm_total(1:6) + front_collective;\n' ...
    'motor_rpm_total(7:12) = motor_rpm_total(7:12) + rear_collective;\n' ...
    'end\n']));
ensureZeroWindConstant(model);

deleteIfExists(model, 'delta_f');
deleteIfExists(model, 'delta_a');
deleteIfExists(model, 'delta_e');
deleteIfExists(model, 'delta_r');
deleteIfExists(model, 'Control Mixing');
deleteIfExists(model, 'deltaLW rad2deg');
deleteIfExists(model, 'deltaRW rad2deg');
deleteIfExists(model, 'deltaLT rad2deg');
deleteIfExists(model, 'deltaRT rad2deg');
deleteIfExists(model, 'Specific Force Gain');
deleteIfExists(model, 'pos_NED');
deleteIfExists(model, 'V_B_truth');
deleteIfExists(model, 'V_E_truth');
deleteIfExists(model, 'eul_truth');
deleteIfExists(model, 'omega_truth');
deleteIfExists(model, 'C_NB_truth');
deleteIfExists(model, 'V_BA_truth');
deleteIfExists(model, 'vinf_truth');
deleteIfExists(model, 'alpha_truth');
deleteIfExists(model, 'beta_truth');
deleteIfExists(model, 'specific_force_truth');

add_block('simulink/Sources/In1', [model '/delta_f'], ...
    'Position', [-1210 -135 -1180 -121]);
add_block('simulink/Sources/In1', [model '/delta_a'], ...
    'Position', [-1210 -105 -1180 -91]);
add_block('simulink/Sources/In1', [model '/delta_e'], ...
    'Position', [-1210 -75 -1180 -61]);
add_block('simulink/Sources/In1', [model '/delta_r'], ...
    'Position', [-1210 -45 -1180 -31]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Control Mixing'], ...
    'Position', [-1125 -150 -930 -25]);
configureControlMixingSubsystem([model '/Control Mixing']);

add_block('simulink/Math Operations/Gain', [model '/deltaLW rad2deg'], ...
    'Gain', '180/pi', ...
    'Position', [-885 -140 -830 -120]);
add_block('simulink/Math Operations/Gain', [model '/deltaRW rad2deg'], ...
    'Gain', '180/pi', ...
    'Position', [-885 -105 -830 -85]);
add_block('simulink/Math Operations/Gain', [model '/deltaLT rad2deg'], ...
    'Gain', '180/pi', ...
    'Position', [-885 -70 -830 -50]);
add_block('simulink/Math Operations/Gain', [model '/deltaRT rad2deg'], ...
    'Gain', '180/pi', ...
    'Position', [-885 -35 -830 -15]);

phMix = get_param([model '/Control Mixing'], 'PortHandles');
phLWdeg = get_param([model '/deltaLW rad2deg'], 'PortHandles');
phRWdeg = get_param([model '/deltaRW rad2deg'], 'PortHandles');
phLTdeg = get_param([model '/deltaLT rad2deg'], 'PortHandles');
phRTdeg = get_param([model '/deltaRT rad2deg'], 'PortHandles');
phAVL = get_param([model '/AVL Aircraft Aero'], 'PortHandles');
phLW = get_param([model '/Left Wing'], 'PortHandles');
phRW = get_param([model '/Right Wing'], 'PortHandles');
phLT = get_param([model '/Left Tail'], 'PortHandles');
phRT = get_param([model '/Right Tail'], 'PortHandles');
phProp = get_param([model '/Propellers'], 'PortHandles');
phRPM = get_param([model '/Motor_RPM_cmd'], 'PortHandles');
phTilt = get_param([model '/Tilt_angles_cmd'], 'PortHandles');
phFrontCollective = get_param([model '/Front_RPM_collective'], 'PortHandles');
phRearCollective = get_param([model '/Rear_RPM_collective1'], 'PortHandles');
phRPMCombine = get_param([model '/RPM Command Combine'], 'PortHandles');

deleteInputLine(phLW.Inport(3));
deleteInputLine(phRW.Inport(3));
deleteInputLine(phLT.Inport(3));
deleteInputLine(phRT.Inport(3));
deleteInputLine(phAVL.Inport(2));
deleteInputLine(phAVL.Inport(3));
deleteInputLine(phAVL.Inport(4));
deleteInputLine(phAVL.Inport(5));
deleteInputLine(phProp.Inport(1));
deleteInputLine(phProp.Inport(2));
patchPropellerVectorWiring([model '/Propellers']);

add_line(model, 'delta_f/1', 'Control Mixing/1', 'autorouting', 'on');
add_line(model, 'delta_a/1', 'Control Mixing/2', 'autorouting', 'on');
add_line(model, 'delta_e/1', 'Control Mixing/3', 'autorouting', 'on');
add_line(model, 'delta_r/1', 'Control Mixing/4', 'autorouting', 'on');

add_line(model, phMix.Outport(1), phLW.Inport(3), 'autorouting', 'on');
add_line(model, phMix.Outport(2), phRW.Inport(3), 'autorouting', 'on');
add_line(model, phMix.Outport(3), phLT.Inport(3), 'autorouting', 'on');
add_line(model, phMix.Outport(4), phRT.Inport(3), 'autorouting', 'on');

add_line(model, phMix.Outport(1), phLWdeg.Inport(1), 'autorouting', 'on');
add_line(model, phMix.Outport(2), phRWdeg.Inport(1), 'autorouting', 'on');
add_line(model, phMix.Outport(3), phLTdeg.Inport(1), 'autorouting', 'on');
add_line(model, phMix.Outport(4), phRTdeg.Inport(1), 'autorouting', 'on');

add_line(model, phLWdeg.Outport(1), phAVL.Inport(2), 'autorouting', 'on');
add_line(model, phRWdeg.Outport(1), phAVL.Inport(3), 'autorouting', 'on');
add_line(model, phLTdeg.Outport(1), phAVL.Inport(4), 'autorouting', 'on');
add_line(model, phRTdeg.Outport(1), phAVL.Inport(5), 'autorouting', 'on');

add_line(model, phRPM.Outport(1), phRPMCombine.Inport(1), 'autorouting', 'on');
add_line(model, phFrontCollective.Outport(1), phRPMCombine.Inport(2), 'autorouting', 'on');
add_line(model, phRearCollective.Outport(1), phRPMCombine.Inport(3), 'autorouting', 'on');
add_line(model, phRPMCombine.Outport(1), phProp.Inport(1), 'autorouting', 'on');
add_line(model, phTilt.Outport(1), phProp.Inport(2), 'autorouting', 'on');

add_block('simulink/Math Operations/Gain', [model '/Specific Force Gain'], ...
    'Gain', '1/Mass', ...
    'Position', [210 -315 265 -285]);

phAdd1 = get_param([model '/Add1'], 'PortHandles');
phSpec = get_param([model '/Specific Force Gain'], 'PortHandles');
add_line(model, phAdd1.Outport(1), phSpec.Inport(1), 'autorouting', 'on');

addTruthOutport(model, 'pos_NED', [805 325 835 339], [model '/Integrator'], 1);
addTruthOutport(model, 'V_B_truth', [805 365 835 379], [model '/Translational Dynamics'], 1);
addTruthOutport(model, 'V_E_truth', [805 405 835 419], [model '/Translational Kinematics'], 1);
addTruthOutport(model, 'eul_truth', [805 445 835 459], [model '/Reshape'], 1);
addTruthOutport(model, 'omega_truth', [805 485 835 499], [model '/Rotational Dynamics'], 1);
addTruthOutport(model, 'C_NB_truth', [805 525 835 539], [model '/roational kinematics'], 3);
addTruthOutport(model, 'V_BA_truth', [805 565 835 579], [model '/Sum'], 1);
addTruthOutport(model, 'vinf_truth', [805 605 835 619], [model '/V_rel 2 Vinf Alpha Beta'], 1);
addTruthOutport(model, 'alpha_truth', [805 645 835 659], [model '/V_rel 2 Vinf Alpha Beta'], 2);
addTruthOutport(model, 'beta_truth', [805 685 835 699], [model '/V_rel 2 Vinf Alpha Beta'], 3);
addTruthOutport(model, 'specific_force_truth', [805 725 835 739], [model '/Specific Force Gain'], 1);

save_system(model);
close_system(model);
end

function rebuildOuterWrapper(model, plantModel)
if bdIsLoaded(model)
    close_system(model, 0);
end
if exist([model '.slx'], 'file') ~= 0
    bdclose(model);
    delete([model '.slx']);
end

new_system(model);
set_param(model, 'StopTime', '10');

add_block('simulink/Ports & Subsystems/Model', [model '/Plant'], ...
    'ModelName', plantModel, ...
    'Position', [555 137 710 398]);

add_block('simulink/Sources/Clock', [model '/Clock'], ...
    'Position', [35 55 65 75]);
add_block('simulink/Sources/Constant', [model '/Controller Enable'], ...
    'Value', 'controller_enable', ...
    'Position', [35 110 100 140]);
add_block('simulink/Sources/Constant', [model '/Controller Mode'], ...
    'Value', 'controller_mode', ...
    'Position', [35 165 100 195]);
add_block('simulink/Sources/Constant', [model '/Tilt Cmd'], ...
    'Value', 'controller_trim_tilt_angles', ...
    'Position', [465 90 535 120]);
add_block('simulink/Sources/Constant', [model '/Sensor Bias Enable'], ...
    'Value', 'sensor_enable_bias', ...
    'Position', [35 220 105 250]);
add_block('simulink/Sources/Constant', [model '/Sensor Noise Enable'], ...
    'Value', 'sensor_enable_noise', ...
    'Position', [35 255 105 285]);
add_block('simulink/Sources/Constant', [model '/Estimator Enable'], ...
    'Value', 'estimator_enable', ...
    'Position', [35 300 105 330]);
add_block('simulink/Sources/Constant', [model '/Trim Front Collective'], ...
    'Value', 'trim_front_collective_rpm', ...
    'Position', [465 240 535 270]);
add_block('simulink/Sources/Constant', [model '/Trim Rear Collective'], ...
    'Value', 'trim_rear_collective_rpm', ...
    'Position', [465 285 535 315]);

add_block('simulink/Sources/Constant', [model '/Airspeed Cmd Base'], ...
    'Value', 'controller_airspeed_cmd_base', ...
    'Position', [35 360 120 390]);
add_block('simulink/Sources/Step', [model '/Airspeed Cmd Step'], ...
    'Time', 'controller_step_time', ...
    'Before', '0', ...
    'After', 'controller_airspeed_step', ...
    'Position', [35 405 120 435]);
add_block('simulink/Math Operations/Sum', [model '/Airspeed Cmd Sum'], ...
    'Inputs', '++', ...
    'Position', [155 380 175 420]);

add_block('simulink/Sources/Constant', [model '/Bank Cmd Base'], ...
    'Value', 'controller_bank_cmd_base', ...
    'Position', [35 470 120 500]);
add_block('simulink/Sources/Step', [model '/Bank Cmd Step'], ...
    'Time', 'controller_step_time', ...
    'Before', '0', ...
    'After', 'controller_bank_step', ...
    'Position', [35 515 120 545]);
add_block('simulink/Math Operations/Sum', [model '/Bank Cmd Sum'], ...
    'Inputs', '++', ...
    'Position', [155 490 175 530]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Sensor Suite'], ...
    'Position', [220 115 425 420]);
set_param([model '/Sensor Suite'], ...
    'TreatAsAtomicUnit', 'on', ...
    'SystemSampleTime', 'sensor_sample_time');
configureSensorSuiteSubsystem([model '/Sensor Suite']);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Attitude Estimator'], ...
    'Position', [220 470 425 635]);
set_param([model '/Attitude Estimator'], ...
    'TreatAsAtomicUnit', 'on', ...
    'SystemSampleTime', 'estimator_sample_time');
configureEstimatorSubsystem([model '/Attitude Estimator']);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Outer Controller'], ...
    'Position', [220 680 430 915]);
set_param([model '/Outer Controller'], ...
    'TreatAsAtomicUnit', 'on', ...
    'SystemSampleTime', 'controller_sample_time');
configureControllerSubsystem([model '/Outer Controller']);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Prop Command Expand'], ...
    'Position', [465 115 535 220]);
configurePropCommandExpandSubsystem([model '/Prop Command Expand']);

add_block('simulink/Discrete/Memory', [model '/Motor Command Memory'], ...
    'Position', [470 690 510 715]);
add_block('simulink/Discrete/Memory', [model '/delta_f Memory'], ...
    'Position', [470 735 510 760]);
add_block('simulink/Discrete/Memory', [model '/delta_a Memory'], ...
    'Position', [470 780 510 805]);
add_block('simulink/Discrete/Memory', [model '/delta_e Memory'], ...
    'Position', [470 825 510 850]);
add_block('simulink/Discrete/Memory', [model '/delta_r Memory'], ...
    'Position', [470 870 510 895]);

wireOuterWrapper(model);
addOuterLogging(model);

save_system(model, [model '.slx']);
close_system(model);
end

function configureControlMixingSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

add_block('simulink/Sources/In1', [subsysPath '/delta_f'], ...
    'Position', [25 28 55 42]);
add_block('simulink/Sources/In1', [subsysPath '/delta_a'], ...
    'Position', [25 58 55 72]);
add_block('simulink/Sources/In1', [subsysPath '/delta_e'], ...
    'Position', [25 88 55 102]);
add_block('simulink/Sources/In1', [subsysPath '/delta_r'], ...
    'Position', [25 118 55 132]);

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Mix'], ...
    'Position', [95 35 220 125]);
setMatlabFunctionScript([subsysPath '/Mix'], sprintf([ ...
    'function [deltaLW, deltaRW, deltaLT, deltaRT] = fcn(delta_f, delta_a, delta_e, delta_r)\n' ...
    '%%#codegen\n' ...
    'deltaLW = delta_f + delta_a;\n' ...
    'deltaRW = delta_f - delta_a;\n' ...
    'deltaLT = delta_e - delta_r;\n' ...
    'deltaRT = delta_e + delta_r;\n' ...
    'end\n']));

add_block('simulink/Sinks/Out1', [subsysPath '/deltaLW'], ...
    'Position', [270 28 300 42]);
add_block('simulink/Sinks/Out1', [subsysPath '/deltaRW'], ...
    'Position', [270 58 300 72]);
add_block('simulink/Sinks/Out1', [subsysPath '/deltaLT'], ...
    'Position', [270 88 300 102]);
add_block('simulink/Sinks/Out1', [subsysPath '/deltaRT'], ...
    'Position', [270 118 300 132]);

add_line(subsysPath, 'delta_f/1', 'Mix/1', 'autorouting', 'on');
add_line(subsysPath, 'delta_a/1', 'Mix/2', 'autorouting', 'on');
add_line(subsysPath, 'delta_e/1', 'Mix/3', 'autorouting', 'on');
add_line(subsysPath, 'delta_r/1', 'Mix/4', 'autorouting', 'on');
add_line(subsysPath, 'Mix/1', 'deltaLW/1', 'autorouting', 'on');
add_line(subsysPath, 'Mix/2', 'deltaRW/1', 'autorouting', 'on');
add_line(subsysPath, 'Mix/3', 'deltaLT/1', 'autorouting', 'on');
add_line(subsysPath, 'Mix/4', 'deltaRT/1', 'autorouting', 'on');
end

function configureSensorSuiteSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

inNames = {'vinf_truth','alpha_truth','beta_truth','omega_truth', ...
    'specific_force_truth','pos_truth','vel_truth','eul_truth', ...
    'C_NB_truth','enable_bias','enable_noise'};
for idx = 1:numel(inNames)
    y = 30 + (idx - 1) * 30;
    add_block('simulink/Sources/In1', [subsysPath '/' inNames{idx}], ...
        'Position', [20 y 50 y + 14]);
end

constNames = { ...
    'sensor_airdata_bias', 'sensor_gyro_bias', 'sensor_accel_bias', ...
    'sensor_gps_pos_bias', 'sensor_gps_vel_bias', 'sensor_mag_bias', ...
    'sensor_attitude_bias', 'sensor_airdata_sigma', 'sensor_gyro_sigma', ...
    'sensor_accel_sigma', 'sensor_gps_pos_sigma', 'sensor_gps_vel_sigma', ...
    'sensor_mag_sigma', 'sensor_attitude_sigma', 'mag_ned'};
for idx = 1:numel(constNames)
    y = 30 + (idx - 1) * 30;
    add_block('simulink/Sources/Constant', [subsysPath '/' constNames{idx}], ...
        'Value', constNames{idx}, ...
        'Position', [65 y 135 y + 20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Sensor Eval'], ...
    'Position', [175 80 365 295]);
setMatlabFunctionScript([subsysPath '/Sensor Eval'], sprintf([ ...
    'function [airspeed_meas, alpha_meas, beta_meas, gyro_meas, accel_meas, gps_pos_meas, gps_vel_meas, mag_body_meas, attitude_meas] = fcn(vinf_truth, alpha_truth, beta_truth, omega_truth, specific_force_truth, pos_truth, vel_truth, eul_truth, C_NB_truth, enable_bias, enable_noise, airdata_bias, gyro_bias, accel_bias, gps_pos_bias, gps_vel_bias, mag_bias, attitude_bias, airdata_sigma, gyro_sigma, accel_sigma, gps_pos_sigma, gps_vel_sigma, mag_sigma, attitude_sigma, mag_ned)\n' ...
    '%%#codegen\n' ...
    '[airspeed_meas, alpha_meas, beta_meas, gyro_meas, accel_meas, gps_pos_meas, gps_vel_meas, mag_body_meas, attitude_meas] = sensor_suite_eval(vinf_truth, alpha_truth, beta_truth, omega_truth, specific_force_truth, pos_truth, vel_truth, eul_truth, C_NB_truth, enable_bias, enable_noise, airdata_bias, gyro_bias, accel_bias, gps_pos_bias, gps_vel_bias, mag_bias, attitude_bias, airdata_sigma, gyro_sigma, accel_sigma, gps_pos_sigma, gps_vel_sigma, mag_sigma, attitude_sigma, mag_ned);\n' ...
    'end\n']));

outNames = {'airspeed_meas','alpha_meas','beta_meas','gyro_meas', ...
    'accel_meas','gps_pos_meas','gps_vel_meas','mag_body_meas', ...
    'attitude_meas'};
for idx = 1:numel(outNames)
    y = 30 + (idx - 1) * 30;
    add_block('simulink/Sinks/Out1', [subsysPath '/' outNames{idx}], ...
        'Position', [410 y 440 y + 14]);
end

for idx = 1:numel(inNames)
    add_line(subsysPath, [inNames{idx} '/1'], ['Sensor Eval/' num2str(idx)], 'autorouting', 'on');
end
for idx = 1:numel(constNames)
    add_line(subsysPath, [constNames{idx} '/1'], ['Sensor Eval/' num2str(numel(inNames) + idx)], 'autorouting', 'on');
end
for idx = 1:numel(outNames)
    add_line(subsysPath, ['Sensor Eval/' num2str(idx)], [outNames{idx} '/1'], 'autorouting', 'on');
end
end

function configureEstimatorSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

inNames = {'t','enable','gyro_meas','accel_meas','mag_body_meas','attitude_meas'};
for idx = 1:numel(inNames)
    y = 45 + (idx - 1) * 30;
    add_block('simulink/Sources/In1', [subsysPath '/' inNames{idx}], ...
        'Position', [20 y 50 y + 14]);
end

add_block('simulink/Sources/Constant', [subsysPath '/estimator_init_eul'], ...
    'Value', 'estimator_init_eul', ...
    'Position', [70 70 145 90]);
add_block('simulink/Sources/Constant', [subsysPath '/estimator_k_acc'], ...
    'Value', 'estimator_k_acc', ...
    'Position', [70 110 145 130]);
add_block('simulink/Sources/Constant', [subsysPath '/estimator_k_mag'], ...
    'Value', 'estimator_k_mag', ...
    'Position', [70 150 145 170]);
add_block('simulink/Sources/Constant', [subsysPath '/estimator_k_att'], ...
    'Value', 'estimator_k_att', ...
    'Position', [70 190 145 210]);

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Estimator'], ...
    'Position', [185 80 365 210]);
setMatlabFunctionScript([subsysPath '/Estimator'], sprintf([ ...
    'function eul_hat = fcn(t, enable, gyro_meas, accel_meas, mag_body_meas, attitude_meas, init_eul, k_acc, k_mag, k_att)\n' ...
    '%%#codegen\n' ...
    'eul_hat = attitude_complementary_step(t, enable, gyro_meas, accel_meas, mag_body_meas, attitude_meas, init_eul, k_acc, k_mag, k_att);\n' ...
    'end\n']));

add_block('simulink/Sinks/Out1', [subsysPath '/eul_hat'], ...
    'Position', [405 118 435 132]);

for idx = 1:numel(inNames)
    add_line(subsysPath, [inNames{idx} '/1'], ['Estimator/' num2str(idx)], 'autorouting', 'on');
end
add_line(subsysPath, 'estimator_init_eul/1', 'Estimator/7', 'autorouting', 'on');
add_line(subsysPath, 'estimator_k_acc/1', 'Estimator/8', 'autorouting', 'on');
add_line(subsysPath, 'estimator_k_mag/1', 'Estimator/9', 'autorouting', 'on');
add_line(subsysPath, 'estimator_k_att/1', 'Estimator/10', 'autorouting', 'on');
add_line(subsysPath, 'Estimator/1', 'eul_hat/1', 'autorouting', 'on');
end

function configureControllerSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

inNames = {'t','enable','mode','airspeed_meas','gyro_meas','eul_hat', ...
    'airspeed_cmd','bank_cmd'};
for idx = 1:numel(inNames)
    y = 30 + (idx - 1) * 26;
    add_block('simulink/Sources/In1', [subsysPath '/' inNames{idx}], ...
        'Position', [20 y 50 y + 14]);
end

constNames = { ...
    'controller_trim_motor_rpms', 'controller_trim_delta_f', ...
    'controller_trim_delta_a', 'controller_trim_delta_e', ...
    'controller_trim_delta_r', 'controller_k_p_damp', ...
    'controller_k_q_damp', 'controller_k_r_damp', ...
    'controller_k_phi_p', 'controller_k_phi_i', ...
    'controller_k_v_p', 'controller_k_v_i', ...
    'controller_front_motor_mask', 'controller_surface_limit_rad', ...
    'controller_rpm_delta_limit', 'controller_integrator_limit'};
for idx = 1:numel(constNames)
    y = 30 + (idx - 1) * 26;
    add_block('simulink/Sources/Constant', [subsysPath '/' constNames{idx}], ...
        'Value', constNames{idx}, ...
        'Position', [65 y 170 y + 20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Controller'], ...
    'Position', [210 80 430 285]);
setMatlabFunctionScript([subsysPath '/Controller'], sprintf([ ...
    'function [motor_rpms, delta_f, delta_a, delta_e, delta_r] = fcn(t, enable, mode, airspeed_meas, gyro_meas, eul_hat, airspeed_cmd, bank_cmd, trim_motor_rpms, trim_delta_f, trim_delta_a, trim_delta_e, trim_delta_r, k_p_damp, k_q_damp, k_r_damp, k_phi_p, k_phi_i, k_v_p, k_v_i, front_motor_mask, surface_limit_rad, rpm_delta_limit, integrator_limit)\n' ...
    '%%#codegen\n' ...
    '[motor_rpms, delta_f, delta_a, delta_e, delta_r] = outer_controller_step(t, enable, mode, airspeed_meas, gyro_meas, eul_hat, airspeed_cmd, bank_cmd, trim_motor_rpms, trim_delta_f, trim_delta_a, trim_delta_e, trim_delta_r, k_p_damp, k_q_damp, k_r_damp, k_phi_p, k_phi_i, k_v_p, k_v_i, front_motor_mask, surface_limit_rad, rpm_delta_limit, integrator_limit);\n' ...
    'end\n']));

outNames = {'motor_rpms','delta_f','delta_a','delta_e','delta_r'};
for idx = 1:numel(outNames)
    y = 80 + (idx - 1) * 36;
    add_block('simulink/Sinks/Out1', [subsysPath '/' outNames{idx}], ...
        'Position', [470 y 500 y + 14]);
end

for idx = 1:numel(inNames)
    add_line(subsysPath, [inNames{idx} '/1'], ['Controller/' num2str(idx)], 'autorouting', 'on');
end
for idx = 1:numel(constNames)
    add_line(subsysPath, [constNames{idx} '/1'], ['Controller/' num2str(numel(inNames) + idx)], 'autorouting', 'on');
end
for idx = 1:numel(outNames)
    add_line(subsysPath, ['Controller/' num2str(idx)], [outNames{idx} '/1'], 'autorouting', 'on');
end
end

function configurePropCommandExpandSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

add_block('simulink/Sources/In1', [subsysPath '/motor_group_cmd'], ...
    'Position', [20 30 50 44]);
add_block('simulink/Sources/In1', [subsysPath '/tilt_group_cmd'], ...
    'Position', [20 80 50 94]);
add_block('simulink/User-Defined Functions/MATLAB Function', [subsysPath '/Expand'], ...
    'Position', [85 35 215 95]);
setMatlabFunctionScript([subsysPath '/Expand'], sprintf([ ...
    'function [motor_rpms_cmd, tilt_deg_cmd, front_collective_rpm, rear_collective_rpm] = fcn(motor_group_cmd, tilt_group_cmd)\n' ...
    '%%#codegen\n' ...
    'fr = motor_group_cmd(1);\n' ...
    'fl = motor_group_cmd(2);\n' ...
    'rr = motor_group_cmd(3);\n' ...
    'rl = motor_group_cmd(4);\n' ...
    'motor_rpms_cmd = [fr; fr; fr; fl; fl; fl; rr; rr; rr; rl; rl; rl];\n' ...
    'tilt_fr = tilt_group_cmd(1);\n' ...
    'tilt_fl = tilt_group_cmd(2);\n' ...
    'tilt_deg_cmd = [tilt_fr; tilt_fr; tilt_fr; tilt_fl; tilt_fl; tilt_fl];\n' ...
    'front_collective_rpm = 0.5 * (fr + fl);\n' ...
    'rear_collective_rpm = 0.5 * (rr + rl);\n' ...
    'end\n']));
add_block('simulink/Sinks/Out1', [subsysPath '/motor_rpms_cmd'], ...
    'Position', [255 40 285 54]);
add_block('simulink/Sinks/Out1', [subsysPath '/tilt_deg_cmd'], ...
    'Position', [255 80 285 94]);
add_block('simulink/Sinks/Out1', [subsysPath '/front_collective_rpm'], ...
    'Position', [255 120 285 134]);
add_block('simulink/Sinks/Out1', [subsysPath '/rear_collective_rpm'], ...
    'Position', [255 160 285 174]);

add_line(subsysPath, 'motor_group_cmd/1', 'Expand/1', 'autorouting', 'on');
add_line(subsysPath, 'tilt_group_cmd/1', 'Expand/2', 'autorouting', 'on');
add_line(subsysPath, 'Expand/1', 'motor_rpms_cmd/1', 'autorouting', 'on');
add_line(subsysPath, 'Expand/2', 'tilt_deg_cmd/1', 'autorouting', 'on');
add_line(subsysPath, 'Expand/3', 'front_collective_rpm/1', 'autorouting', 'on');
add_line(subsysPath, 'Expand/4', 'rear_collective_rpm/1', 'autorouting', 'on');
end

function wireOuterWrapper(model)
phPlant = get_param([model '/Plant'], 'PortHandles');
phSens = get_param([model '/Sensor Suite'], 'PortHandles');
phEst = get_param([model '/Attitude Estimator'], 'PortHandles');
phCtrl = get_param([model '/Outer Controller'], 'PortHandles');
phClock = get_param([model '/Clock'], 'PortHandles');
phExpand = get_param([model '/Prop Command Expand'], 'PortHandles');
phMemMotor = get_param([model '/Motor Command Memory'], 'PortHandles');
phMemDf = get_param([model '/delta_f Memory'], 'PortHandles');
phMemDa = get_param([model '/delta_a Memory'], 'PortHandles');
phMemDe = get_param([model '/delta_e Memory'], 'PortHandles');
phMemDr = get_param([model '/delta_r Memory'], 'PortHandles');

% Command generation.
add_line(model, 'Airspeed Cmd Base/1', 'Airspeed Cmd Sum/1', 'autorouting', 'on');
add_line(model, 'Airspeed Cmd Step/1', 'Airspeed Cmd Sum/2', 'autorouting', 'on');
add_line(model, 'Bank Cmd Base/1', 'Bank Cmd Sum/1', 'autorouting', 'on');
add_line(model, 'Bank Cmd Step/1', 'Bank Cmd Sum/2', 'autorouting', 'on');

% Plant inputs.
add_line(model, phCtrl.Outport(1), phMemMotor.Inport(1), 'autorouting', 'on');
add_line(model, phCtrl.Outport(2), phMemDf.Inport(1), 'autorouting', 'on');
add_line(model, phCtrl.Outport(3), phMemDa.Inport(1), 'autorouting', 'on');
add_line(model, phCtrl.Outport(4), phMemDe.Inport(1), 'autorouting', 'on');
add_line(model, phCtrl.Outport(5), phMemDr.Inport(1), 'autorouting', 'on');
add_line(model, phMemMotor.Outport(1), phExpand.Inport(1), 'autorouting', 'on');
add_line(model, 'Tilt Cmd/1', 'Prop Command Expand/2', 'autorouting', 'on');
add_line(model, phExpand.Outport(1), phPlant.Inport(1), 'autorouting', 'on');
add_line(model, phExpand.Outport(2), phPlant.Inport(2), 'autorouting', 'on');
if numel(phPlant.Inport) >= 8
    add_line(model, 'Trim Front Collective/1', 'Plant/3', 'autorouting', 'on');
    add_line(model, 'Trim Rear Collective/1', 'Plant/4', 'autorouting', 'on');
    add_line(model, phMemDf.Outport(1), phPlant.Inport(5), 'autorouting', 'on');
    add_line(model, phMemDa.Outport(1), phPlant.Inport(6), 'autorouting', 'on');
    add_line(model, phMemDe.Outport(1), phPlant.Inport(7), 'autorouting', 'on');
    add_line(model, phMemDr.Outport(1), phPlant.Inport(8), 'autorouting', 'on');
else
    add_line(model, phMemDf.Outport(1), phPlant.Inport(3), 'autorouting', 'on');
    add_line(model, phMemDa.Outport(1), phPlant.Inport(4), 'autorouting', 'on');
    add_line(model, phMemDe.Outport(1), phPlant.Inport(5), 'autorouting', 'on');
    add_line(model, phMemDr.Outport(1), phPlant.Inport(6), 'autorouting', 'on');
end

% Sensor suite truth inputs.
add_line(model, phPlant.Outport(8), phSens.Inport(1), 'autorouting', 'on');  % vinf
add_line(model, phPlant.Outport(9), phSens.Inport(2), 'autorouting', 'on');  % alpha
add_line(model, phPlant.Outport(10), phSens.Inport(3), 'autorouting', 'on'); % beta
add_line(model, phPlant.Outport(5), phSens.Inport(4), 'autorouting', 'on');  % omega
add_line(model, phPlant.Outport(11), phSens.Inport(5), 'autorouting', 'on'); % specific force
add_line(model, phPlant.Outport(1), phSens.Inport(6), 'autorouting', 'on');  % pos
add_line(model, phPlant.Outport(3), phSens.Inport(7), 'autorouting', 'on');  % vel_E
add_line(model, phPlant.Outport(4), phSens.Inport(8), 'autorouting', 'on');  % eul
add_line(model, phPlant.Outport(6), phSens.Inport(9), 'autorouting', 'on');  % C_NB
add_line(model, 'Sensor Bias Enable/1', 'Sensor Suite/10', 'autorouting', 'on');
add_line(model, 'Sensor Noise Enable/1', 'Sensor Suite/11', 'autorouting', 'on');

% Estimator inputs.
add_line(model, phClock.Outport(1), phEst.Inport(1), 'autorouting', 'on');
add_line(model, 'Estimator Enable/1', 'Attitude Estimator/2', 'autorouting', 'on');
add_line(model, phSens.Outport(4), phEst.Inport(3), 'autorouting', 'on');
add_line(model, phSens.Outport(5), phEst.Inport(4), 'autorouting', 'on');
add_line(model, phSens.Outport(8), phEst.Inport(5), 'autorouting', 'on');
add_line(model, phSens.Outport(9), phEst.Inport(6), 'autorouting', 'on');

% Controller inputs.
add_line(model, phClock.Outport(1), phCtrl.Inport(1), 'autorouting', 'on');
add_line(model, 'Controller Enable/1', 'Outer Controller/2', 'autorouting', 'on');
add_line(model, 'Controller Mode/1', 'Outer Controller/3', 'autorouting', 'on');
add_line(model, phSens.Outport(1), phCtrl.Inport(4), 'autorouting', 'on');
add_line(model, phSens.Outport(4), phCtrl.Inport(5), 'autorouting', 'on');
add_line(model, phEst.Outport(1), phCtrl.Inport(6), 'autorouting', 'on');
add_line(model, 'Airspeed Cmd Sum/1', 'Outer Controller/7', 'autorouting', 'on');
add_line(model, 'Bank Cmd Sum/1', 'Outer Controller/8', 'autorouting', 'on');
end

function addOuterLogging(model)
addToWorkspace(model, 'To Workspace position', [775 55 875 80], 'position');
addToWorkspace(model, 'To Workspace v_b', [775 95 875 120], 'v_b');
addToWorkspace(model, 'To Workspace v_e', [775 135 875 160], 'v_e');
addToWorkspace(model, 'To Workspace eul', [775 175 875 200], 'eul');
addToWorkspace(model, 'To Workspace omega', [775 215 875 240], 'omega');
addToWorkspace(model, 'To Workspace vinf', [775 255 875 280], 'vinf');
addToWorkspace(model, 'To Workspace alpha', [775 295 875 320], 'alpha');
addToWorkspace(model, 'To Workspace beta', [775 335 875 360], 'beta');
addToWorkspace(model, 'To Workspace specific_force', [775 375 875 400], 'specific_force_b');
addToWorkspace(model, 'To Workspace airspeed_meas', [775 445 875 470], 'airspeed_meas');
addToWorkspace(model, 'To Workspace alpha_meas', [775 475 875 500], 'alpha_meas');
addToWorkspace(model, 'To Workspace beta_meas', [775 505 875 530], 'beta_meas');
addToWorkspace(model, 'To Workspace gyro_meas', [775 535 875 560], 'gyro_meas');
addToWorkspace(model, 'To Workspace accel_meas', [775 565 875 590], 'accel_meas');
addToWorkspace(model, 'To Workspace gps_pos_meas', [775 595 875 620], 'gps_pos_meas');
addToWorkspace(model, 'To Workspace gps_vel_meas', [775 625 875 650], 'gps_vel_meas');
addToWorkspace(model, 'To Workspace mag_body_meas', [775 655 875 680], 'mag_body_meas');
addToWorkspace(model, 'To Workspace attitude_meas', [775 685 875 710], 'attitude_meas');
addToWorkspace(model, 'To Workspace eul_hat', [905 445 1005 470], 'eul_hat');
addToWorkspace(model, 'To Workspace motor_rpms_cmd', [905 475 1005 500], 'motor_rpms_cmd');
addToWorkspace(model, 'To Workspace delta_f_cmd', [905 505 1005 530], 'delta_f_cmd');
addToWorkspace(model, 'To Workspace delta_a_cmd', [905 535 1005 560], 'delta_a_cmd');
addToWorkspace(model, 'To Workspace delta_e_cmd', [905 565 1005 590], 'delta_e_cmd');
addToWorkspace(model, 'To Workspace delta_r_cmd', [905 595 1005 620], 'delta_r_cmd');
addToWorkspace(model, 'To Workspace bank_cmd', [905 625 1005 650], 'bank_cmd');
addToWorkspace(model, 'To Workspace airspeed_cmd', [905 655 1005 680], 'airspeed_cmd');

phPlant = get_param([model '/Plant'], 'PortHandles');
phSens = get_param([model '/Sensor Suite'], 'PortHandles');
phEst = get_param([model '/Attitude Estimator'], 'PortHandles');
phCtrl = get_param([model '/Outer Controller'], 'PortHandles');

wireLogging(model, phPlant.Outport(1), [model '/To Workspace position']);
wireLogging(model, phPlant.Outport(2), [model '/To Workspace v_b']);
wireLogging(model, phPlant.Outport(3), [model '/To Workspace v_e']);
wireLogging(model, phPlant.Outport(4), [model '/To Workspace eul']);
wireLogging(model, phPlant.Outport(5), [model '/To Workspace omega']);
wireLogging(model, phPlant.Outport(8), [model '/To Workspace vinf']);
wireLogging(model, phPlant.Outport(9), [model '/To Workspace alpha']);
wireLogging(model, phPlant.Outport(10), [model '/To Workspace beta']);
wireLogging(model, phPlant.Outport(11), [model '/To Workspace specific_force']);
wireLogging(model, phSens.Outport(1), [model '/To Workspace airspeed_meas']);
wireLogging(model, phSens.Outport(2), [model '/To Workspace alpha_meas']);
wireLogging(model, phSens.Outport(3), [model '/To Workspace beta_meas']);
wireLogging(model, phSens.Outport(4), [model '/To Workspace gyro_meas']);
wireLogging(model, phSens.Outport(5), [model '/To Workspace accel_meas']);
wireLogging(model, phSens.Outport(6), [model '/To Workspace gps_pos_meas']);
wireLogging(model, phSens.Outport(7), [model '/To Workspace gps_vel_meas']);
wireLogging(model, phSens.Outport(8), [model '/To Workspace mag_body_meas']);
wireLogging(model, phSens.Outport(9), [model '/To Workspace attitude_meas']);
wireLogging(model, phEst.Outport(1), [model '/To Workspace eul_hat']);
wireLogging(model, phCtrl.Outport(1), [model '/To Workspace motor_rpms_cmd']);
wireLogging(model, phCtrl.Outport(2), [model '/To Workspace delta_f_cmd']);
wireLogging(model, phCtrl.Outport(3), [model '/To Workspace delta_a_cmd']);
wireLogging(model, phCtrl.Outport(4), [model '/To Workspace delta_e_cmd']);
wireLogging(model, phCtrl.Outport(5), [model '/To Workspace delta_r_cmd']);
add_line(model, 'Bank Cmd Sum/1', 'To Workspace bank_cmd/1', 'autorouting', 'on');
add_line(model, 'Airspeed Cmd Sum/1', 'To Workspace airspeed_cmd/1', 'autorouting', 'on');
end

function addTruthOutport(model, outName, position, srcBlock, srcPort)
add_block('simulink/Sinks/Out1', [model '/' outName], ...
    'Position', position);
srcPH = get_param(srcBlock, 'PortHandles');
dstPH = get_param([model '/' outName], 'PortHandles');
add_line(model, srcPH.Outport(srcPort), dstPH.Inport(1), 'autorouting', 'on');
end

function replaceWithInport(model, oldName, newName)
blockPath = [model '/' oldName];
pos = get_param(blockPath, 'Position');
delete_block(blockPath);
add_block('simulink/Sources/In1', [model '/' newName], ...
    'Position', pos);
end

function ensureZeroWindConstant(model)
blockPath = [model '/Constant'];
if ~existBlock(blockPath)
    add_block('simulink/Sources/Constant', blockPath, ...
        'Value', '[0; 0; 0]', ...
        'Position', [-1035 360 -975 390]);
end
phConst = get_param(blockPath, 'PortHandles');
phMM = get_param([model '/MatrixMultiply'], 'PortHandles');
deleteInputLine(phMM.Inport(2));
add_line(model, phConst.Outport(1), phMM.Inport(2), 'autorouting', 'on');
end

function addToWorkspace(model, blockName, position, varName)
add_block('simulink/Sinks/To Workspace', [model '/' blockName], ...
    'VariableName', varName, ...
    'SaveFormat', 'Array', ...
    'Position', position);
end

function wireLogging(model, srcPort, dstBlock)
dstHandles = get_param(dstBlock, 'PortHandles');
add_line(model, srcPort, dstHandles.Inport(1), 'autorouting', 'on');
end

function setMatlabFunctionScript(blockPath, scriptText)
sfRoot = sfroot;
chart = find(sfRoot, '-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('build_model_reference_architecture:MissingChart', ...
        'Could not find MATLAB Function chart at %s', blockPath);
end
chart.Script = scriptText;
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

function deleteIfExists(model, blockName)
blockPath = [model '/' blockName];
if existBlock(blockPath)
    delete_block(blockPath);
end
end

function deleteInputLine(portHandle)
lineHandle = get_param(portHandle, 'Line');
if lineHandle ~= -1
    delete_line(lineHandle);
end
end

function patchPropellerVectorWiring(propPath)
phRPM = get_param([propPath '/Motor RPMs [12x1]'], 'PortHandles');
phTilt = get_param([propPath '/Tilt Angles [6x1]'], 'PortHandles');
phFR = get_param([propPath '/Front Right'], 'PortHandles');
phFL = get_param([propPath '/Front Left'], 'PortHandles');
phRR = get_param([propPath '/Rear Right'], 'PortHandles');
phRL = get_param([propPath '/Rear Left'], 'PortHandles');

if existBlock([propPath '/Demux'])
    delete_block([propPath '/Demux']);
end
if existBlock([propPath '/Demux1'])
    delete_block([propPath '/Demux1']);
end

deleteInputLine(phFR.Inport(2));
deleteInputLine(phFL.Inport(2));
deleteInputLine(phFR.Inport(1));
deleteInputLine(phFL.Inport(1));
deleteInputLine(phRR.Inport(1));
deleteInputLine(phRL.Inport(1));

add_block('simulink/Signal Routing/Demux', [propPath '/Demux'], ...
    'Outputs', '[3 3 3 3]', ...
    'Position', [90 70 95 240]);
add_block('simulink/Signal Routing/Demux', [propPath '/Demux1'], ...
    'Outputs', '[3 3]', ...
    'Position', [90 290 95 360]);

phRPMDemux = get_param([propPath '/Demux'], 'PortHandles');
phTiltDemux = get_param([propPath '/Demux1'], 'PortHandles');

add_line(propPath, phRPM.Outport(1), phRPMDemux.Inport(1), 'autorouting', 'on');
add_line(propPath, phRPMDemux.Outport(1), phFR.Inport(1), 'autorouting', 'on');
add_line(propPath, phRPMDemux.Outport(2), phFL.Inport(1), 'autorouting', 'on');
add_line(propPath, phRPMDemux.Outport(3), phRR.Inport(1), 'autorouting', 'on');
add_line(propPath, phRPMDemux.Outport(4), phRL.Inport(1), 'autorouting', 'on');

add_line(propPath, phTilt.Outport(1), phTiltDemux.Inport(1), 'autorouting', 'on');
add_line(propPath, phTiltDemux.Outport(1), phFR.Inport(2), 'autorouting', 'on');
add_line(propPath, phTiltDemux.Outport(2), phFL.Inport(2), 'autorouting', 'on');
end

function tf = plantLooksLikeWrapper(modelName, modelFile)
tf = false;
if exist(modelFile, 'file') == 0
    return;
end
wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelFile);
end
tf = existBlock([modelName '/Plant']);
if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function tf = existBlock(blockPath)
tf = false;
try
    get_param(blockPath, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function exportTrackedModelSource(repoRoot, modelName, sourceFile, sourceRef)
cmd = sprintf('cd ''%s'' && git show %s:%s.slx > ''%s''', ...
    escapeShellArg(repoRoot), escapeShellArg(sourceRef), modelName, escapeShellArg(sourceFile));
[status, msg] = system(cmd);
if status ~= 0
    error('build_model_reference_architecture:GitExportFailed', ...
        'Failed to export %s.slx at %s from git: %s', modelName, sourceRef, strtrim(msg));
end
end

function out = escapeShellArg(in)
out = strrep(in, '''', '''"''"''');
end
