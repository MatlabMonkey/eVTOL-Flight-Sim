function results = design_pd(varargin)
%DESIGN_PD Root-locus plots for bank-angle and airspeed proportional loops.

linData = linearize_cruise_plant(varargin{:});
io = linData.io;
outDir = localOutputDir();

Gphi = ss(linData.A, linData.B(:, io.inputs.delta_a), ...
    linData.C(io.outputs.eul(1), :), linData.D(io.outputs.eul(1), io.inputs.delta_a));
GV = ss(linData.A, linData.B * linData.frontCollectiveVec, ...
    linData.C(io.outputs.vinf, :), linData.D(io.outputs.vinf, :) * linData.frontCollectiveVec);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 500]);
subplot(1,2,1); rlocus(Gphi); grid on; title('Bank-Angle Proportional Root Locus');
subplot(1,2,2); rlocus(GV); grid on; title('Airspeed Proportional Root Locus');
saveas(fig, fullfile(outDir, 'pd_root_locus.png'));
close(fig);

results = struct('Gphi', Gphi, 'GV', GV, ...
    'figurePath', fullfile(outDir, 'pd_root_locus.png'));
end

function outDir = localOutputDir()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'control_design');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
end
