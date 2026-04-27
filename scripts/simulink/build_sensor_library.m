function build_sensor_library(varargin)
%BUILD_SENSOR_LIBRARY Build a reusable Simulink library for sensor blocks.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

p = inputParser;
p.addParameter('LibraryName', 'Brown_Sensors_lib', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});

libName = char(p.Results.LibraryName);
libFile = fullfile(repoRoot, [libName '.slx']);

if bdIsLoaded(libName)
    close_system(libName, 0);
end
if exist(libFile, 'file') ~= 0
    delete(libFile);
end

new_system(libName, 'Library');
set_param(libName, 'Lock', 'off');

add_block('simulink/Ports & Subsystems/Subsystem', [libName '/Scalar Sensor'], ...
    'Position', [70 60 220 140]);
configureScalarSensor([libName '/Scalar Sensor']);

add_block('simulink/Ports & Subsystems/Subsystem', [libName '/3-Axis Sensor'], ...
    'Position', [70 190 220 300]);
configureVectorSensor([libName '/3-Axis Sensor']);

add_block('simulink/Ports & Subsystems/Subsystem', [libName '/GPS Sensor'], ...
    'Position', [70 340 220 460]);
configureGpsSensor([libName '/GPS Sensor']);

save_system(libName, libFile);
close_system(libName);

fprintf('Built %s at %s\n', libName, libFile);
end

function configureScalarSensor(blockPath)
clearSubsystemContents(blockPath);
set_param(blockPath, 'TreatAsAtomicUnit', 'on');

add_block('simulink/Sources/In1', [blockPath '/truth'], ...
    'Position', [25 43 55 57]);
add_block('simulink/Sources/Clock', [blockPath '/Clock'], ...
    'Position', [25 88 55 112]);
add_block('simulink/Signal Routing/Mux', [blockPath '/Mux'], ...
    'Inputs', '2', ...
    'Position', [75 52 85 103]);
add_block('simulink/User-Defined Functions/MATLAB Fcn', [blockPath '/Step'], ...
    'MATLABFcn', 'sensor_scalar_block_step(u(2), u(1), cfg)', ...
    'Position', [110 58 235 97]);
add_block('simulink/Sinks/Out1', [blockPath '/meas'], ...
    'Position', [275 83 305 97]);

add_line(blockPath, 'truth/1', 'Mux/1', 'autorouting', 'on');
add_line(blockPath, 'Clock/1', 'Mux/2', 'autorouting', 'on');
add_line(blockPath, 'Mux/1', 'Step/1', 'autorouting', 'on');
add_line(blockPath, 'Step/1', 'meas/1', 'autorouting', 'on');

applyMask(blockPath, 'cfg', 'default_sensor_scalar_cfg()', 'Scalar\nSensor', true);
end

function configureVectorSensor(blockPath)
clearSubsystemContents(blockPath);
set_param(blockPath, 'TreatAsAtomicUnit', 'on');

add_block('simulink/Sources/In1', [blockPath '/truth'], ...
    'Position', [25 58 55 72], ...
    'PortDimensions', '3');
add_block('simulink/Sources/Clock', [blockPath '/Clock'], ...
    'Position', [25 108 55 132]);
add_block('simulink/Signal Routing/Mux', [blockPath '/Mux'], ...
    'Inputs', '2', ...
    'Position', [75 72 85 123]);
add_block('simulink/User-Defined Functions/MATLAB Fcn', [blockPath '/Step'], ...
    'MATLABFcn', 'sensor_vector3_block_step(u(4), u(1:3), cfg)', ...
    'Position', [110 78 250 117]);
add_block('simulink/Sinks/Out1', [blockPath '/meas'], ...
    'Position', [285 98 315 112], ...
    'PortDimensions', '3');

add_line(blockPath, 'truth/1', 'Mux/1', 'autorouting', 'on');
add_line(blockPath, 'Clock/1', 'Mux/2', 'autorouting', 'on');
add_line(blockPath, 'Mux/1', 'Step/1', 'autorouting', 'on');
add_line(blockPath, 'Step/1', 'meas/1', 'autorouting', 'on');

applyMask(blockPath, 'cfg', 'default_sensor_vec3_cfg()', '3-Axis\nSensor', true);
end

function configureGpsSensor(blockPath)
clearSubsystemContents(blockPath);
set_param(blockPath, 'TreatAsAtomicUnit', 'on');

add_block('simulink/Sources/In1', [blockPath '/pos_truth'], ...
    'Position', [25 48 55 62], ...
    'PortDimensions', '3');
add_block('simulink/Sources/In1', [blockPath '/vel_truth'], ...
    'Position', [25 88 55 102], ...
    'PortDimensions', '3');
add_block('simulink/Sources/Clock', [blockPath '/Clock'], ...
    'Position', [25 132 55 156]);
add_block('simulink/Signal Routing/Mux', [blockPath '/Pos Mux'], ...
    'Inputs', '2', ...
    'Position', [75 52 85 103]);
add_block('simulink/Signal Routing/Mux', [blockPath '/Vel Mux'], ...
    'Inputs', '2', ...
    'Position', [75 112 85 163]);
add_block('simulink/User-Defined Functions/MATLAB Fcn', [blockPath '/Pos Step'], ...
    'MATLABFcn', 'sensor_vector3_block_step(u(4), u(1:3), cfg.pos_sensor)', ...
    'Position', [110 58 265 97]);
add_block('simulink/User-Defined Functions/MATLAB Fcn', [blockPath '/Vel Step'], ...
    'MATLABFcn', 'sensor_vector3_block_step(u(4), u(1:3), cfg.vel_sensor)', ...
    'Position', [110 118 265 157]);
add_block('simulink/Sinks/Out1', [blockPath '/pos_meas'], ...
    'Position', [305 88 335 102], ...
    'PortDimensions', '3');
add_block('simulink/Sinks/Out1', [blockPath '/vel_meas'], ...
    'Position', [305 128 335 142], ...
    'PortDimensions', '3');

add_line(blockPath, 'pos_truth/1', 'Pos Mux/1', 'autorouting', 'on');
add_line(blockPath, 'Clock/1', 'Pos Mux/2', 'autorouting', 'on');
add_line(blockPath, 'Pos Mux/1', 'Pos Step/1', 'autorouting', 'on');
add_line(blockPath, 'vel_truth/1', 'Vel Mux/1', 'autorouting', 'on');
add_line(blockPath, 'Clock/1', 'Vel Mux/2', 'autorouting', 'on');
add_line(blockPath, 'Vel Mux/1', 'Vel Step/1', 'autorouting', 'on');
add_line(blockPath, 'Pos Step/1', 'pos_meas/1', 'autorouting', 'on');
add_line(blockPath, 'Vel Step/1', 'vel_meas/1', 'autorouting', 'on');

applyMask(blockPath, 'cfg', 'default_sensor_gps_cfg()', 'GPS\nSensor', false);
end

function applyMask(blockPath, paramName, defaultValue, iconText, scalarIO)
maskObj = Simulink.Mask.create(blockPath);
maskObj.Description = 'Simple reusable sensor model block.';
maskObj.SelfModifiable = 'on';
maskObj.addParameter( ...
    'Type', 'edit', ...
    'Name', paramName, ...
    'Prompt', 'cfg', ...
    'Value', defaultValue);

if scalarIO
    maskObj.Display = sprintf([ ...
        'color(''black'');\n' ...
        'disp(''%s'');\n' ...
        'port_label(''input'',1,''truth'');\n' ...
        'port_label(''output'',1,''meas'');\n'], iconText);
else
    maskObj.Display = sprintf([ ...
        'color(''black'');\n' ...
        'disp(''%s'');\n' ...
        'port_label(''input'',1,''pos'');\n' ...
        'port_label(''input'',2,''vel'');\n' ...
        'port_label(''output'',1,''pos'');\n' ...
        'port_label(''output'',2,''vel'');\n'], iconText);
end
end

function clearSubsystemContents(subsysPath)
blocks = find_system(subsysPath, 'SearchDepth', 1, 'Type', 'Block');
for idx = 1:numel(blocks)
    if strcmp(blocks{idx}, subsysPath)
        continue;
    end
    delete_block(blocks{idx});
end
lines = find_system(subsysPath, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
delete(lines);
end
