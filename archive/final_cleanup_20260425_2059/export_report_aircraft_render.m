function outputPath = export_report_aircraft_render(varargin)
%EXPORT_REPORT_AIRCRAFT_RENDER Save a white-background aircraft render for reports.
%
% Usage:
%   export_report_aircraft_render()
%   export_report_aircraft_render('preset', 'cruise')
%   export_report_aircraft_render('output_name', 'aircraft_render_transition45')
%
% Output:
%   Writes a PNG into eVTOL_Simulation/report_plots_final and returns the path.

p = inputParser;
p.addParameter('preset', 'cruise', @(x) ischar(x) || isstring(x));
p.addParameter('output_name', '', @(x) ischar(x) || isstring(x));
p.addParameter('resolution', 300, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.parse(varargin{:});
opts = p.Results;

baseDir = fileparts(mfilename('fullpath'));
outputDir = fullfile(baseDir, 'report_plots_final');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

preset = char(string(opts.preset));
outputName = char(string(opts.output_name));
if isempty(outputName)
    outputName = sprintf('aircraft_render_%s_white', lower(strtrim(preset)));
end

outputPath = fullfile(outputDir, [outputName '.png']);

hFig = render_aircraft(preset, ...
    'theme', 'report', ...
    'figure_visible', 'off', ...
    'show_axes', false, ...
    'show_vectors', false, ...
    'show_labels', false, ...
    'show_title', false, ...
    'show_legend', false, ...
    'show_cg_label', false, ...
    'title_text', '');

localTightenReportAxes(hFig);

exportgraphics(hFig, outputPath, 'Resolution', opts.resolution, 'BackgroundColor', 'white');
close(hFig);

fprintf('Saved report aircraft render to %s\n', outputPath);
end

function localTightenReportAxes(hFig)
ax = findobj(hFig, 'Type', 'axes');
if isempty(ax)
    return;
end
ax = ax(1);

patches = findobj(ax, 'Type', 'patch');
if isempty(patches)
    return;
end

allPts = [];
for idx = 1:numel(patches)
    verts = get(patches(idx), 'Vertices');
    if isnumeric(verts) && size(verts, 2) == 3
        allPts = [allPts; verts]; %#ok<AGROW>
    end
end

if isempty(allPts)
    return;
end

mins = min(allPts, [], 1);
maxs = max(allPts, [], 1);
span = max(maxs - mins, 1e-3);
pad = 0.08 * max(span);
center = 0.5 * (mins + maxs);
halfWidth = 0.5 * span + pad;

xlim(ax, center(1) + [-1 1] * halfWidth(1));
ylim(ax, center(2) + [-1 1] * halfWidth(2));
zlim(ax, center(3) + [-1 1] * halfWidth(3));
camtarget(ax, center);
camzoom(ax, 1.15);
end
