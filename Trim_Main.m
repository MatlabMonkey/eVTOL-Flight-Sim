% one trim case

if ~exist('initData', 'var') || ~isstruct(initData)
    error(['initData is required. Run Init_Main first so the trim script ', ...
           'has the aircraft constants and model names it needs.']);
end

if ~exist('trimCase', 'var') || ~isstruct(trimCase)
    error(['trimCase is required. Example:\n', ...
           '  trimCase = struct(''name'',''Hover'',''mode'',''manual'',''Vinf_mps'',0,''front_tilt_deg'',0);\n', ...
           '  Trim_Main']);
end

if ~exist('trim_verbose', 'var') || isempty(trim_verbose)
    trim_verbose = false;
end

if ~exist('trim_debug', 'var') || isempty(trim_debug)
    trim_debug = false;
end

[trimResult, trimSpec] = trim_evtol_case(initData, trimCase, struct( ...
    'verbose', trim_verbose, ...
    'debug', trim_debug, ...
    'emitSummary', true, ...
    'emitLinearSummary', true));
