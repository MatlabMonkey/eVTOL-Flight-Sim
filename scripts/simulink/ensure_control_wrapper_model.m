function ensure_control_wrapper_model(varargin)
%ENSURE_CONTROL_WRAPPER_MODEL Build the wrapper only when needed, then load it.

p = inputParser;
p.addParameter('OuterModel', 'Brown_Control_Sim', @(s) ischar(s) || isstring(s));
p.addParameter('RebuildWrapper', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('RebuildPlant', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

outerModel = char(p.Results.OuterModel);
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
modelPath = fullfile(repoRoot, [outerModel '.slx']);

if p.Results.RebuildWrapper || exist(modelPath, 'file') == 0
    build_control_wrapper_model( ...
        'OuterModel', outerModel, ...
        'RebuildPlant', logical(p.Results.RebuildPlant));
end

load_system(outerModel);
end
