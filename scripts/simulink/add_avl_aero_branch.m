function add_avl_aero_branch()
%ADD_AVL_AERO_BRANCH Add a parallel AVL aero branch to Brown_Full_Sim.
%
% This is a repeatable model-patch script. It keeps the existing analytical
% surface-based branch, adds a compact aircraft-level AVL branch, and
% inserts a workspace-controlled switch (`use_avl_aero`) between them.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

model = 'Brown_Full_Sim';
load_system(model);

% Make sure the workspace variables used by the new branch exist.
evalin('base', 'run(''Full_Sim_Init.m'');');
load_system(model);

deleteIfExists(model, 'Analytic Aero Force Sum');
deleteIfExists(model, 'Analytic Aero Moment Sum');
deleteIfExists(model, 'AVL Aircraft Aero');
deleteIfExists(model, 'AVL Force Switch');
deleteIfExists(model, 'AVL Moment Switch');
deleteIfExists(model, 'AVL Aero Mode');
deleteIfExists(model, 'From AVL V_BA');
deleteIfExists(model, 'To Workspace AVL Force');
deleteIfExists(model, 'To Workspace AVL Moment');
deleteIfExists(model, 'To Workspace Analytic Force');
deleteIfExists(model, 'To Workspace Analytic Moment');
deleteIfExists(model, 'To Workspace Selected Force');
deleteIfExists(model, 'To Workspace Selected Moment');
deleteIfExists(model, 'To Workspace Aero Mode');

set_param([model '/Add'], 'Inputs', '++');
set_param([model '/Add1'], 'Inputs', '++');

add_block('simulink/Math Operations/Sum', [model '/Analytic Aero Force Sum'], ...
    'Inputs', '++++', ...
    'Position', [-5 -324 15 -236]);
add_block('simulink/Math Operations/Sum', [model '/Analytic Aero Moment Sum'], ...
    'Inputs', '++++', ...
    'Position', [-5 -519 15 -431]);

add_block('simulink/Signal Routing/Switch', [model '/AVL Force Switch'], ...
    'Criteria', 'u2 ~= 0', ...
    'Threshold', '0.5', ...
    'Position', [65 -318 105 -242]);
add_block('simulink/Signal Routing/Switch', [model '/AVL Moment Switch'], ...
    'Criteria', 'u2 ~= 0', ...
    'Threshold', '0.5', ...
    'Position', [65 -513 105 -437]);

add_block('simulink/Sources/Constant', [model '/AVL Aero Mode'], ...
    'Value', 'use_avl_aero', ...
    'Position', [10 -640 55 -620]);

add_block('simulink/Signal Routing/From', [model '/From AVL V_BA'], ...
    'GotoTag', 'V_BA', ...
    'Position', [-765 -294 -725 -276]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/AVL Aircraft Aero'], ...
    'Position', [-645 -445 -405 -175]);
configureAVLAeroSubsystem([model '/AVL Aircraft Aero']);

addLoggingBlocks(model);

% Existing top-level handles.
blkRightTail = [model '/Right Tail'];
blkLeftTail = [model '/Left Tail'];
blkRightWing = [model '/Right Wing'];
blkLeftWing = [model '/Left Wing'];
blkProp = [model '/Propellers'];
blkCtrl = [model '/Simple PD Controller'];

phRT = get_param(blkRightTail, 'PortHandles');
phLT = get_param(blkLeftTail, 'PortHandles');
phRW = get_param(blkRightWing, 'PortHandles');
phLW = get_param(blkLeftWing, 'PortHandles');
phProp = get_param(blkProp, 'PortHandles');
phCtrl = get_param(blkCtrl, 'PortHandles');
phAddForce = get_param([model '/Add1'], 'PortHandles');
phAddMoment = get_param([model '/Add'], 'PortHandles');
phAnaForce = get_param([model '/Analytic Aero Force Sum'], 'PortHandles');
phAnaMoment = get_param([model '/Analytic Aero Moment Sum'], 'PortHandles');
phForceSwitch = get_param([model '/AVL Force Switch'], 'PortHandles');
phMomentSwitch = get_param([model '/AVL Moment Switch'], 'PortHandles');
phMode = get_param([model '/AVL Aero Mode'], 'PortHandles');
phVB = get_param([model '/From AVL V_BA'], 'PortHandles');
phAVL = get_param([model '/AVL Aircraft Aero'], 'PortHandles');

% Branch surface outputs to the new analytical sums.
add_line(model, phRT.Outport(1), phAnaForce.Inport(1), 'autorouting', 'on');
add_line(model, phLT.Outport(1), phAnaForce.Inport(2), 'autorouting', 'on');
add_line(model, phRW.Outport(1), phAnaForce.Inport(3), 'autorouting', 'on');
add_line(model, phLW.Outport(1), phAnaForce.Inport(4), 'autorouting', 'on');

add_line(model, phRT.Outport(2), phAnaMoment.Inport(1), 'autorouting', 'on');
add_line(model, phLT.Outport(2), phAnaMoment.Inport(2), 'autorouting', 'on');
add_line(model, phRW.Outport(2), phAnaMoment.Inport(3), 'autorouting', 'on');
add_line(model, phLW.Outport(2), phAnaMoment.Inport(4), 'autorouting', 'on');

% Feed controller outputs and V_BA into the AVL branch.
add_line(model, phVB.Outport(1), phAVL.Inport(1), 'autorouting', 'on');
add_line(model, phCtrl.Outport(2), phAVL.Inport(2), 'autorouting', 'on');
add_line(model, phCtrl.Outport(3), phAVL.Inport(3), 'autorouting', 'on');
add_line(model, phCtrl.Outport(4), phAVL.Inport(4), 'autorouting', 'on');
add_line(model, phCtrl.Outport(5), phAVL.Inport(5), 'autorouting', 'on');

% Switch inputs: u1 = AVL, u2 = mode, u3 = analytical.
% For Simulink Switch blocks, a true condition selects u1 and false selects
% u3. That gives the desired behavior:
%   use_avl_aero = false => analytical
%   use_avl_aero = true  => AVL
add_line(model, phAVL.Outport(1), phForceSwitch.Inport(1), 'autorouting', 'on');
add_line(model, phMode.Outport(1), phForceSwitch.Inport(2), 'autorouting', 'on');
add_line(model, phAnaForce.Outport(1), phForceSwitch.Inport(3), 'autorouting', 'on');

add_line(model, phAVL.Outport(2), phMomentSwitch.Inport(1), 'autorouting', 'on');
add_line(model, phMode.Outport(1), phMomentSwitch.Inport(2), 'autorouting', 'on');
add_line(model, phAnaMoment.Outport(1), phMomentSwitch.Inport(3), 'autorouting', 'on');

% Remove the old direct analytical-to-dynamics wiring.
safeDeleteLine(model, blkRightTail, 1, [model '/Add1'], 1);
safeDeleteLine(model, blkLeftTail, 1, [model '/Add1'], 2);
safeDeleteLine(model, blkRightWing, 1, [model '/Add1'], 3);
safeDeleteLine(model, blkProp, 2, [model '/Add1'], 4);
safeDeleteLine(model, blkLeftWing, 1, [model '/Add1'], 5);

safeDeleteLine(model, blkRightTail, 2, [model '/Add'], 1);
safeDeleteLine(model, blkLeftTail, 2, [model '/Add'], 2);
safeDeleteLine(model, blkRightWing, 2, [model '/Add'], 3);
safeDeleteLine(model, blkProp, 1, [model '/Add'], 4);
safeDeleteLine(model, blkLeftWing, 2, [model '/Add'], 5);

% Final sums now only add selected aero and propeller totals.
add_line(model, phForceSwitch.Outport(1), phAddForce.Inport(1), 'autorouting', 'on');
add_line(model, phProp.Outport(2), phAddForce.Inport(2), 'autorouting', 'on');

add_line(model, phMomentSwitch.Outport(1), phAddMoment.Inport(1), 'autorouting', 'on');
add_line(model, phProp.Outport(1), phAddMoment.Inport(2), 'autorouting', 'on');

% Logging branches for comparison.
wireLogging(model, phAVL.Outport(1), [model '/To Workspace AVL Force']);
wireLogging(model, phAVL.Outport(2), [model '/To Workspace AVL Moment']);
wireLogging(model, phAnaForce.Outport(1), [model '/To Workspace Analytic Force']);
wireLogging(model, phAnaMoment.Outport(1), [model '/To Workspace Analytic Moment']);
wireLogging(model, phForceSwitch.Outport(1), [model '/To Workspace Selected Force']);
wireLogging(model, phMomentSwitch.Outport(1), [model '/To Workspace Selected Moment']);
wireLogging(model, phMode.Outport(1), [model '/To Workspace Aero Mode']);

save_system(model);
close_system(model);

fprintf('Added AVL aero branch to %s and saved the model.\n', model);

end

function configureAVLAeroSubsystem(subsysPath)
clearSubsystemContents(subsysPath);

add_block('simulink/Sources/In1', [subsysPath '/V_BA'], ...
    'Position', [30 38 60 52]);
add_block('simulink/Sources/In1', [subsysPath '/deltaLW_deg'], ...
    'Position', [30 83 60 97]);
add_block('simulink/Sources/In1', [subsysPath '/deltaRW_deg'], ...
    'Position', [30 128 60 142]);
add_block('simulink/Sources/In1', [subsysPath '/deltaLT_deg'], ...
    'Position', [30 173 60 187]);
add_block('simulink/Sources/In1', [subsysPath '/deltaRT_deg'], ...
    'Position', [30 218 60 232]);

add_block('simulink/Sources/Constant', [subsysPath '/rho'], ...
    'Value', 'rho', ...
    'Position', [30 263 70 287]);
add_block('simulink/Sources/Constant', [subsysPath '/avlAero'], ...
    'Value', 'avlAero', ...
    'OutDataTypeStr', 'Bus: avlAeroBus', ...
    'Position', [30 308 70 332]);

mfcnPath = [subsysPath '/Compute AVL Aero'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfcnPath, ...
    'Position', [130 92 300 248]);
setAVLMatlabFunctionScript(mfcnPath);

add_block('simulink/Sinks/Out1', [subsysPath '/F_aero_B'], ...
    'Position', [355 133 385 147]);
add_block('simulink/Sinks/Out1', [subsysPath '/M_aero_B'], ...
    'Position', [355 193 385 207]);

add_line(subsysPath, 'V_BA/1', 'Compute AVL Aero/1', 'autorouting', 'on');
add_line(subsysPath, 'deltaLW_deg/1', 'Compute AVL Aero/2', 'autorouting', 'on');
add_line(subsysPath, 'deltaRW_deg/1', 'Compute AVL Aero/3', 'autorouting', 'on');
add_line(subsysPath, 'deltaLT_deg/1', 'Compute AVL Aero/4', 'autorouting', 'on');
add_line(subsysPath, 'deltaRT_deg/1', 'Compute AVL Aero/5', 'autorouting', 'on');
add_line(subsysPath, 'rho/1', 'Compute AVL Aero/6', 'autorouting', 'on');
add_line(subsysPath, 'avlAero/1', 'Compute AVL Aero/7', 'autorouting', 'on');
add_line(subsysPath, 'Compute AVL Aero/1', 'F_aero_B/1', 'autorouting', 'on');
add_line(subsysPath, 'Compute AVL Aero/2', 'M_aero_B/1', 'autorouting', 'on');
end

function setAVLMatlabFunctionScript(blockPath)
sfRoot = sfroot;
chart = find(sfRoot, '-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('add_avl_aero_branch:MissingChart', ...
        'Could not find MATLAB Function chart at %s', blockPath);
end

chart.Script = sprintf([ ...
    'function [F_aero_B, M_aero_B] = fcn(V_BA, deltaLW_deg, deltaRW_deg, deltaLT_deg, deltaRT_deg, rho, avlAero)\n' ...
    '%%#codegen\n' ...
    '[F_aero_B, M_aero_B] = avl_aircraft_aero_eval(V_BA, deltaLW_deg, deltaRW_deg, deltaLT_deg, deltaRT_deg, rho, avlAero);\n' ...
    'end\n']);
end

function addLoggingBlocks(model)
addToWorkspace(model, 'To Workspace AVL Force', [430 -430 530 -400], 'aero_force_avl');
addToWorkspace(model, 'To Workspace AVL Moment', [430 -585 530 -555], 'aero_moment_avl');
addToWorkspace(model, 'To Workspace Analytic Force', [430 -390 530 -360], 'aero_force_analytic');
addToWorkspace(model, 'To Workspace Analytic Moment', [430 -545 530 -515], 'aero_moment_analytic');
addToWorkspace(model, 'To Workspace Selected Force', [430 -350 530 -320], 'aero_force_selected');
addToWorkspace(model, 'To Workspace Selected Moment', [430 -505 530 -475], 'aero_moment_selected');
addToWorkspace(model, 'To Workspace Aero Mode', [430 -650 530 -620], 'aero_mode_selected');
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
if exist_block(blockPath)
    delete_block(blockPath);
end
end

function tf = exist_block(blockPath)
tf = false;
try
    get_param(blockPath, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function safeDeleteLine(model, srcBlock, srcPort, dstBlock, dstPort)
try
    delete_line(model, [get_param(srcBlock, 'Name') '/' num2str(srcPort)], ...
        [get_param(dstBlock, 'Name') '/' num2str(dstPort)]);
catch
    % Line may already be absent; ignore.
end
end
