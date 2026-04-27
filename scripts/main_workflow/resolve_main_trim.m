function trimCtx = resolve_main_trim(session, trimSpec)
%RESOLVE_MAIN_TRIM Resolve and normalize a trim point for the main workflow.
%   trimCtx = RESOLVE_MAIN_TRIM(session, trimSpec)
%
% Purpose
%   Select a trim source, normalize it into the grouped main-workflow
%   convention, and reuse a cached trim result when the exact same request
%   appears again in the current MATLAB session.
%
% Inputs
%   session  : Struct from prepare_main_session.
%   trimSpec : Struct describing the trim source and operating point.
%
% Outputs
%   trimCtx  : Canonical grouped trim struct with shallow, readable fields.
%
% Example
%   session = prepare_main_session();
%   trimCtx = resolve_main_trim(session, struct('source', 'saved'));

if nargin < 1 || ~isstruct(session)
    error('resolve_main_trim:BadSession', ...
        'Pass the session struct returned by prepare_main_session.');
end

if nargin < 2
    trimSpec = struct();
end
trimSpec = default_trim_spec_main(trimSpec);

cacheKey = make_trim_cache_key_main(session, trimSpec);

persistent trimCache
if isempty(trimCache)
    trimCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

if isKey(trimCache, cacheKey)
    trimCtx = trimCache(cacheKey);
    trimCtx.meta.cache_hit = true;
    trimCtx.meta.cache_key = cacheKey;
    assignin('base', 'main_workflow_last_trimCtx', trimCtx);
    return;
end

switch lower(trimSpec.source)
    case 'saved'
        rawTrim = load_saved_trim_main(session, trimSpec);
    case 'fresh'
        rawTrim = solve_trim_main_eul(session, trimSpec);
    case 'provided'
        rawTrim = trimSpec.providedTrim;
    otherwise
        error('resolve_main_trim:UnknownSource', ...
            'Unknown trimSpec.source ''%s''.', trimSpec.source);
end

trimCtx = normalize_trim_main(rawTrim, trimSpec);
trimCtx.meta.cache_hit = false;
trimCtx.meta.cache_key = cacheKey;
trimCtx.meta.trim_model = session.trimModel;
trimCtx.meta.run_model = session.runModel;

trimCache(cacheKey) = trimCtx;
assignin('base', 'main_workflow_last_trimCtx', trimCtx);
end
