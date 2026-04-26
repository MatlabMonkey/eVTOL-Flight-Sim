function results = run_main_sweep(varargin)
%RUN_MAIN_SWEEP Run a trim-by-run sweep using the new main workflow.
%   results = RUN_MAIN_SWEEP()
%   results = RUN_MAIN_SWEEP('TrimCases', trimCases, 'RunCases', runCases)
%
% Purpose
%   Reuse one prepared session and one resolved trim per operating point,
%   then run a set of command cases at each trim.
%
% Example
%   trims = [struct('source','saved','cruiseSpeedMps',70), ...
%            struct('source','saved','cruiseSpeedMps',85)];
%   runs  = [struct('name','hold'), ...
%            struct('name','pitch step', 'deltaEStep', deg2rad(1))];
%   results = run_main_sweep('TrimCases', trims, 'RunCases', runs);

p = inputParser;
p.addParameter('Session', struct(), @isstruct);
p.addParameter('TrimCases', struct([]), @(x) isstruct(x) || iscell(x) || isempty(x));
p.addParameter('RunCases', struct([]), @(x) isstruct(x) || iscell(x) || isempty(x));
p.addParameter('ContinueOnError', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opts = p.Results;

if isempty(fieldnames(opts.Session))
    session = prepare_main_session();
else
    session = opts.Session;
end

trimCases = localToCaseCell(opts.TrimCases, default_trim_spec_main());
runCases = localToCaseCell(opts.RunCases, default_run_spec_main());

caseResults = cell(numel(trimCases) * numel(runCases), 1);
summaryRows = repmat(struct( ...
    'trimLabel', '', 'runLabel', '', 'status', '', ...
    'trimSource', '', 'cruiseSpeedMps', NaN, 'stopTime', NaN), ...
    numel(caseResults), 1);

idx = 1;
for i = 1:numel(trimCases)
    trimSpec = default_trim_spec_main(trimCases{i});
    trimCtx = resolve_main_trim(session, trimSpec);

    for j = 1:numel(runCases)
        runSpec = default_run_spec_main(runCases{j});
        try
            caseResult = run_main_case( ...
                'Session', session, ...
                'TrimCtx', trimCtx, ...
                'RunSpec', runSpec);
            status = 'ok';
        catch ME
            if ~opts.ContinueOnError
                rethrow(ME);
            end
            caseResult = struct( ...
                'simOut', [], ...
                'trimCtx', trimCtx, ...
                'runCtx', [], ...
                'summary', struct('simulationSucceeded', false, ...
                    'failureMessage', ME.message), ...
                'meta', struct('failureStage', 'sweep'));
            status = 'error';
        end

        caseResults{idx} = caseResult;
        summaryRows(idx).trimLabel = localLabelTrim(trimSpec);
        summaryRows(idx).runLabel = runSpec.name;
        summaryRows(idx).status = status;
        summaryRows(idx).trimSource = trimCtx.meta.source;
        summaryRows(idx).cruiseSpeedMps = trimCtx.vinf;
        summaryRows(idx).stopTime = runSpec.stopTime;
        idx = idx + 1;
    end
end

results = struct();
results.session = session;
results.cases = caseResults;
results.summaryTable = struct2table(summaryRows);
end

function label = localLabelTrim(trimSpec)
label = sprintf('%s_%gms_%gdeg', ...
    trimSpec.source, trimSpec.cruiseSpeedMps, trimSpec.bankDeg);
end

function caseCell = localToCaseCell(caseInput, defaultCase)
if nargin < 2
    defaultCase = struct();
end

if isempty(caseInput)
    caseCell = {defaultCase};
elseif iscell(caseInput)
    caseCell = caseInput(:);
elseif isstruct(caseInput)
    if isscalar(caseInput)
        caseCell = {caseInput};
    else
        caseCell = arrayfun(@(s) s, caseInput(:), 'UniformOutput', false);
    end
else
    error('run_main_sweep:BadCaseInput', ...
        'TrimCases and RunCases must be structs or cell arrays of structs.');
end
end
