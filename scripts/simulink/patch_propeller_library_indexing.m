function patch_propeller_library_indexing()
%PATCH_PROPELLER_LIBRARY_INDEXING Fix prop position indexing in library blocks.
%
% The prop position arrays in aircraft_def are stored row-wise:
%   [x y z;
%    x y z;
%    x y z]
% but the original library code indexed them as if each motor location lived
% in a column. That corrupts the moment arms and produces incorrect pitch/yaw
% moments. This helper updates the library MATLAB Function blocks to read rows.

thisDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(thisDir));
libFile = fullfile(repoRoot, 'Brown_Flight_Controls_lib.slx');
if exist(libFile, 'file') == 0
    error('patch_propeller_library_indexing:MissingLibrary', ...
        'Could not find %s', libFile);
end

load_system(libFile);
set_param('Brown_Flight_Controls_lib', 'Lock', 'off');

patchChart( ...
    'Brown_Flight_Controls_lib/Front Propellers', ...
    'pivot_pos(:, i)', ...
    'pivot_pos(i, :).''');

patchChart( ...
    'Brown_Flight_Controls_lib/Rear Propellers', ...
    'prop_pos(:, i)', ...
    'prop_pos(i, :).''');

save_system('Brown_Flight_Controls_lib');
close_system('Brown_Flight_Controls_lib');
end

function patchChart(chartPath, oldText, newText)
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', chartPath);
if isempty(chart)
    error('patch_propeller_library_indexing:MissingChart', ...
        'Could not find chart %s', chartPath);
end

scriptText = chart.Script;
if contains(scriptText, newText)
    return;
end
if ~contains(scriptText, oldText)
    error('patch_propeller_library_indexing:MissingPattern', ...
        'Did not find expected text %s in %s', oldText, chartPath);
end

chart.Script = strrep(scriptText, oldText, newText);
end
