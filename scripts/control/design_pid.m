function results = design_pid(varargin)
%DESIGN_PID Integral-augmented root loci for bank and airspeed loops.

linData = linearize_cruise_plant(varargin{:});
io = linData.io;
outDir = localOutputDir();
s = tf('s');

Gphi = ss(linData.A, linData.B(:, io.inputs.delta_a), ...
    linData.C(io.outputs.eul(1), :), linData.D(io.outputs.eul(1), io.inputs.delta_a));
GV = ss(linData.A, linData.B * linData.frontCollectiveVec, ...
    linData.C(io.outputs.vinf, :), linData.D(io.outputs.vinf, :) * linData.frontCollectiveVec);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 500]);
subplot(1,2,1); rlocus(minreal(Gphi / s)); grid on; title('Bank-Angle PI Root Locus');
subplot(1,2,2); rlocus(minreal(GV / s)); grid on; title('Airspeed PI Root Locus');
saveas(fig, fullfile(outDir, 'pid_root_locus.png'));
close(fig);

results = struct('GphiI', minreal(Gphi / s), 'GVI', minreal(GV / s), ...
    'figurePath', fullfile(outDir, 'pid_root_locus.png'));
end

function outDir = localOutputDir()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'control_design');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
end
