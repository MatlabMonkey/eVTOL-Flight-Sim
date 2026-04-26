function publish_main_workspace(runCtx)
%PUBLISH_MAIN_WORKSPACE Push a run context into the base workspace.

assignments = runCtx.workspaceAssignments;
fieldNames = fieldnames(assignments);
for i = 1:numel(fieldNames)
    assignin('base', fieldNames{i}, assignments.(fieldNames{i}));
end

prop = evalin('base', 'prop');
prop.rotor.init = runCtx.actuatorInitial.motor_rpms_grouped(:);
prop.tilt.init = runCtx.actuatorInitial.tilt_angles_grouped(:);
if numel(prop.rotor.init) >= 4
    prop.rotor.init_FR = prop.rotor.init(1);
    prop.rotor.init_FL = prop.rotor.init(2);
    prop.rotor.init_RR = prop.rotor.init(3);
    prop.rotor.init_RL = prop.rotor.init(4);
end
if numel(prop.tilt.init) >= 2
    prop.tilt.init_FR = prop.tilt.init(1);
    prop.tilt.init_FL = prop.tilt.init(2);
end

wingL = localAttachInit(evalin('base', 'wingL'), runCtx.surfaceInitial.wingL);
wingR = localAttachInit(evalin('base', 'wingR'), runCtx.surfaceInitial.wingR);
tailL = localAttachInit(evalin('base', 'tailL'), runCtx.surfaceInitial.tailL);
tailR = localAttachInit(evalin('base', 'tailR'), runCtx.surfaceInitial.tailR);

assignin('base', 'prop', prop);
assignin('base', 'wingL', wingL);
assignin('base', 'wingR', wingR);
assignin('base', 'tailL', tailL);
assignin('base', 'tailR', tailR);
assignin('base', 'main_workflow_last_runCtx', runCtx);
end

function surface = localAttachInit(surface, initValue)
surface.init = initValue;
end
