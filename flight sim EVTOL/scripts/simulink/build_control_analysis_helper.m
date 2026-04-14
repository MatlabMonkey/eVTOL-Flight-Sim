function modelName = build_control_analysis_helper(varargin)
%BUILD_CONTROL_ANALYSIS_HELPER Build a scalar-input monolithic linmod helper.

p = inputParser;
p.addParameter('ModelName', 'Brown_Control_Analysis_Helper', @(s) ischar(s) || isstring(s));
p.addParameter('SourceModel', 'Brown_6DOF_Plant', @(s) ischar(s) || isstring(s));
p.parse(varargin{:});

modelName = char(p.Results.ModelName);
sourceModel = char(p.Results.SourceModel);
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
sourceFile = fullfile(repoRoot, [sourceModel '.slx']);
modelFile = fullfile(repoRoot, [modelName '.slx']);

if exist(sourceFile, 'file') == 0
    error('build_control_analysis_helper:MissingSourceModel', ...
        'Missing source model: %s', sourceFile);
end

bdclose(modelName);
copyfile(sourceFile, modelFile, 'f');
load_system(modelFile);
setMatlabFunctionScript([modelName '/RPM Command Combine'], sprintf([ ...
    'function motor_rpm_total = fcn(motor_rpm_cmd, front_collective, rear_collective)\n' ...
    '%%#codegen\n' ...
    'motor_rpm_total = motor_rpm_cmd(:);\n' ...
    'if numel(motor_rpm_total) ~= 12\n' ...
    '    motor_rpm_total = zeros(12, 1);\n' ...
    'end\n' ...
    'motor_rpm_total(1:6) = motor_rpm_total(1:6) + front_collective;\n' ...
    'motor_rpm_total(7:12) = motor_rpm_total(7:12) + rear_collective;\n' ...
    'end\n']));

replaceVectorInportWithScalars(modelName, 'Motor_RPM_cmd', ...
    {'motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6', ...
     'motor_7', 'motor_8', 'motor_9', 'motor_10', 'motor_11', 'motor_12'}, 12);
replaceVectorInportWithScalars(modelName, 'Tilt_angles_cmd', ...
    {'tilt_1', 'tilt_2', 'tilt_3', 'tilt_4', 'tilt_5', 'tilt_6'}, 6);

save_system(modelName, modelFile);
close_system(modelName);
end

function replaceVectorInportWithScalars(modelName, blockName, scalarNames, width)
blockPath = [modelName '/' blockName];
pos = get_param(blockPath, 'Position');
ph = get_param(blockPath, 'PortHandles');
lineHandle = get_param(ph.Outport(1), 'Line');
if lineHandle == -1
    error('build_control_analysis_helper:MissingLine', ...
        'Expected %s to drive a downstream line.', blockPath);
end
dstPorts = get_param(lineHandle, 'DstPortHandle');
if isempty(dstPorts)
    error('build_control_analysis_helper:MissingDestination', ...
        'Expected %s to have a destination port.', blockPath);
end
dstPort = dstPorts(1);
delete_line(lineHandle);
delete_block(blockPath);

muxName = [blockName '_mux'];
add_block('simulink/Signal Routing/Vector Concatenate', [modelName '/' muxName], ...
    'NumInputs', num2str(width), ...
    'Position', pos);

xLeft = pos(1) - 110;
for idx = 1:width
    yCenter = pos(2) + 20 * (idx - 0.5);
    add_block('simulink/Sources/In1', [modelName '/' scalarNames{idx}], ...
        'Position', [xLeft yCenter - 7 xLeft + 30 yCenter + 7]);
    add_line(modelName, [scalarNames{idx} '/1'], [muxName '/' num2str(idx)], 'autorouting', 'on');
end

muxPH = get_param([modelName '/' muxName], 'PortHandles');
add_line(modelName, muxPH.Outport(1), dstPort, 'autorouting', 'on');
end

function setMatlabFunctionScript(blockPath, scriptText)
sfRoot = sfroot;
chart = find(sfRoot, '-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('build_control_analysis_helper:MissingChart', ...
        'Could not find MATLAB Function chart at %s', blockPath);
end
chart.Script = scriptText;
end
