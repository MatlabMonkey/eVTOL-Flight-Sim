function ensure_control_analysis_helper(varargin)
%ENSURE_CONTROL_ANALYSIS_HELPER Build the analysis helper only when needed.

p = inputParser;
p.addParameter('ModelName', 'Brown_Control_Analysis_Helper', @(s) ischar(s) || isstring(s));
p.addParameter('Rebuild', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

modelName = char(p.Results.ModelName);
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
modelPath = fullfile(repoRoot, [modelName '.slx']);

if p.Results.Rebuild || exist(modelPath, 'file') == 0
    build_control_analysis_helper();
end

load_system(modelName);
end
