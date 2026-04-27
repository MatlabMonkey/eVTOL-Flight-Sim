function runSpec = default_run_spec_main(runSpec)
%DEFAULT_RUN_SPEC_MAIN Fill in missing run-spec fields with simple defaults.

if nargin < 1 || isempty(runSpec)
    runSpec = struct();
end

runSpec = localSetDefault(runSpec, 'name', 'trim hold');
runSpec = localSetDefault(runSpec, 'stopTime', 8.0);
runSpec = localSetDefault(runSpec, 'stepTime', 1.0);
runSpec = localSetDefault(runSpec, 'runSimulation', true);
runSpec = localSetDefault(runSpec, 'inputConvention', 'grouped');
runSpec = localSetDefault(runSpec, 'useAvlAero', false);
runSpec = localSetDefault(runSpec, 'motorRPMCmdStepGrouped', zeros(4, 1));
runSpec = localSetDefault(runSpec, 'tiltStepGrouped', zeros(2, 1));
runSpec = localSetDefault(runSpec, 'frontCollectiveStep', 0.0);
runSpec = localSetDefault(runSpec, 'rearCollectiveStep', 0.0);
runSpec = localSetDefault(runSpec, 'deltaFStep', 0.0);
runSpec = localSetDefault(runSpec, 'deltaAStep', 0.0);
runSpec = localSetDefault(runSpec, 'deltaEStep', 0.0);
runSpec = localSetDefault(runSpec, 'deltaRStep', 0.0);
end

function s = localSetDefault(s, fieldName, defaultValue)
if ~isfield(s, fieldName) || isempty(s.(fieldName))
    s.(fieldName) = defaultValue;
end
end
