function session = prepare_main_session(varargin)
%PREPARE_MAIN_SESSION Initialize the main-workflow MATLAB session once.
%   session = PREPARE_MAIN_SESSION()
%   session = PREPARE_MAIN_SESSION('RunModel', ...
%       'scripts/main_workflow/models/Brown_6DOF_Plant_main_main_workflow.slx')
%
% Purpose
%   This is the one supported initialization entrypoint for the new
%   scripts/main_workflow package. It runs Brown_6DOF_Init_main.m in the
%   base workspace, loads the trim and run models, and returns a readable
%   session struct that other workflow helpers can reuse.
%
% Inputs
%   'InitScript'   : Init script relative to repo root.
%   'TrimModel'    : Simulink model name or .slx path used for trim solving.
%   'RunModel'     : Simulink model name or .slx path used for actual runs.
%   'RenderEnable' : Whether the init script should render the aircraft.
%
% Outputs
%   session : Struct describing the active workflow session.
%
% Side effects
%   - Writes variables to the base workspace by running Brown_6DOF_Init_main.m
%   - Loads the configured Simulink models into memory
%
% Example
%   session = prepare_main_session('RenderEnable', false);

p = inputParser;
p.addParameter('InitScript', 'Brown_6DOF_Init_main.m', @(s) ischar(s) || isstring(s));
p.addParameter('TrimModel', ...
    fullfile('scripts', 'main_workflow', 'models', ...
    'Brown_6DOF_Plant_EUL_main_workflow.slx'), ...
    @(s) ischar(s) || isstring(s));
p.addParameter('RunModel', ...
    fullfile('scripts', 'main_workflow', 'models', ...
    'Brown_6DOF_Plant_main_main_workflow.slx'), ...
    @(s) ischar(s) || isstring(s));
p.addParameter('RenderEnable', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opts = p.Results;

workflowDir = fileparts(mfilename('fullpath'));
scriptsDir = fileparts(workflowDir);
repoRoot = fileparts(scriptsDir);

trimModelInfo = localResolveModel(repoRoot, opts.TrimModel);
runModelInfo = localResolveModel(repoRoot, opts.RunModel);
sessionKey = localSessionKey(repoRoot, opts, trimModelInfo, runModelInfo);

persistent sessionCache
if isempty(sessionCache)
    sessionCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

reused = false;
if isKey(sessionCache, sessionKey)
    cachedSession = sessionCache(sessionKey);
    if bdIsLoaded(cachedSession.trimModel) && bdIsLoaded(cachedSession.runModel)
        session = cachedSession;
        session.meta.preparedWithCachedSession = true;
        session.meta.preparedAt = char(datetime('now', 'TimeZone', 'local', ...
            'Format', 'yyyy-MM-dd HH:mm:ss Z'));
        sessionCache(sessionKey) = session;
        return;
    end
    reused = true;
end

initScriptPath = fullfile(repoRoot, char(opts.InitScript));
if exist(initScriptPath, 'file') ~= 2
    error('prepare_main_session:MissingInitScript', ...
        'Could not find init script: %s', initScriptPath);
end

assignin('base', 'render_enable', logical(opts.RenderEnable));
evalin('base', sprintf('run(''%s'');', initScriptPath));

load_system(trimModelInfo.loadTarget);
load_system(runModelInfo.loadTarget);

session = struct();
session.repoRoot = repoRoot;
session.workflowRoot = workflowDir;
session.initScript = char(opts.InitScript);
session.initScriptPath = initScriptPath;
session.trimModel = trimModelInfo.modelName;
session.trimModelPath = trimModelInfo.modelPath;
session.trimModelLoadTarget = trimModelInfo.loadTarget;
session.runModel = runModelInfo.modelName;
session.runModelPath = runModelInfo.modelPath;
session.runModelLoadTarget = runModelInfo.loadTarget;
session.renderEnable = logical(opts.RenderEnable);
session.sessionKey = sessionKey;
session.trimCache = struct( ...
    'mode', 'persistent function cache', ...
    'keyBuilder', 'make_trim_cache_key_main');
session.meta = struct( ...
    'preparedAt', char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd HH:mm:ss Z')), ...
    'preparedWithCachedSession', reused, ...
    'notes', 'Brown_6DOF_Init_main.m has already populated the base workspace.');

assignin('base', 'main_workflow_session', session);
sessionCache(sessionKey) = session;
end

function key = localSessionKey(repoRoot, opts, trimModelInfo, runModelInfo)
key = sprintf('%s|%s|%s|%s|render=%d', ...
    repoRoot, char(opts.InitScript), trimModelInfo.keyText, ...
    runModelInfo.keyText, logical(opts.RenderEnable));
end

function modelInfo = localResolveModel(repoRoot, modelSpec)
modelSpec = char(modelSpec);
looksLikePath = contains(modelSpec, filesep) || endsWith(lower(modelSpec), '.slx');

if looksLikePath
    if localIsAbsolutePath(modelSpec)
        modelPath = modelSpec;
    else
        modelPath = fullfile(repoRoot, modelSpec);
    end

        fileState = exist(modelPath, 'file');
        if ~(fileState == 2 || fileState == 4)
            error('prepare_main_session:MissingModel', ...
                'Could not find model file: %s', modelPath);
        end

    [~, modelName] = fileparts(modelPath);
    modelInfo = struct( ...
        'modelName', modelName, ...
        'modelPath', modelPath, ...
        'loadTarget', modelPath, ...
        'keyText', modelPath);
else
    modelInfo = struct( ...
        'modelName', modelSpec, ...
        'modelPath', '', ...
        'loadTarget', modelSpec, ...
        'keyText', modelSpec);
end
end

function tf = localIsAbsolutePath(pathText)
if ispc
    tf = ~isempty(regexp(pathText, '^[A-Za-z]:[\\/]', 'once'));
else
    tf = startsWith(pathText, filesep);
end
end
