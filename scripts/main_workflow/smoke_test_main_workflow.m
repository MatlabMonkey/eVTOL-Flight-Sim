function report = smoke_test_main_workflow(varargin)
%SMOKE_TEST_MAIN_WORKFLOW Exercise the new workflow without touching models.
%   report = SMOKE_TEST_MAIN_WORKFLOW()
%
% Purpose
%   Confirm that the new workflow package can initialize the workspace,
%   resolve a saved trim, optionally resolve a fresh trim, publish a run
%   context, and load/update the configured models. This is the fastest
%   sanity check to run before sharing the workflow with teammates.

p = inputParser;
p.addParameter('RunFreshTrim', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('RunModelUpdate', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opts = p.Results;

report = struct();
report.prepareSessionOk = false;
report.savedTrimOk = false;
report.freshTrimOk = false;
report.workspacePublishOk = false;
report.runModelLoadedOk = false;
report.runModelUpdateOk = false;
report.notes = {};

session = prepare_main_session('RenderEnable', false);
report.prepareSessionOk = true;
report.runModelLoadedOk = bdIsLoaded(session.runModel);

savedTrim = resolve_main_trim(session, struct('source', 'saved'));
report.savedTrimOk = true;

if opts.RunFreshTrim
    try
        freshTrim = resolve_main_trim(session, struct('source', 'fresh')); %#ok<NASGU>
        report.freshTrimOk = true;
    catch ME
        report.notes{end + 1} = sprintf( ...
            'Fresh-trim resolution failed: %s', ME.message); %#ok<AGROW>
    end
else
    report.notes{end + 1} = 'Fresh trim resolution skipped by request.'; %#ok<AGROW>
end

runCtx = adapt_trim_to_run_model(session, savedTrim, struct('runSimulation', false));
publish_main_workspace(runCtx);
report.workspacePublishOk = true;

if opts.RunModelUpdate
    try
        set_param(session.runModel, 'SimulationCommand', 'update');
        report.runModelUpdateOk = true;
    catch ME
        report.notes{end + 1} = sprintf( ...
            'Run-model update failed: %s', ME.message); %#ok<AGROW>
    end
else
    report.notes{end + 1} = 'Run-model update skipped by request.'; %#ok<AGROW>
end

disp(report);
end
