function results = design_dampers(varargin)
%DESIGN_DAMPERS Generate eigenvalue and rate-damper root-locus plots.

linData = linearize_cruise_plant(varargin{:});
io = linData.io;
outDir = localOutputDir();

Gp = ss(linData.A, linData.B(:, io.inputs.delta_a), ...
    linData.C(io.outputs.omega(1), :), linData.D(io.outputs.omega(1), io.inputs.delta_a));
Gq = ss(linData.A, linData.B(:, io.inputs.delta_e), ...
    linData.C(io.outputs.omega(2), :), linData.D(io.outputs.omega(2), io.inputs.delta_e));
Gr = ss(linData.A, linData.B(:, io.inputs.delta_r), ...
    linData.C(io.outputs.omega(3), :), linData.D(io.outputs.omega(3), io.inputs.delta_r));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 900]);
subplot(2,2,1);
plot(real(linData.openLoopEig), imag(linData.openLoopEig), 'x', 'LineWidth', 1.5);
grid on;
xlabel('Real');
ylabel('Imag');
title('Open-Loop Eigenvalues');

subplot(2,2,2); rlocus(Gp); grid on; title('Roll Damper Root Locus (p / \delta_a)');
subplot(2,2,3); rlocus(Gq); grid on; title('Pitch Damper Root Locus (q / \delta_e)');
subplot(2,2,4); rlocus(Gr); grid on; title('Yaw Damper Root Locus (r / \delta_r)');
saveas(fig, fullfile(outDir, 'damper_root_locus.png'));
close(fig);

results = struct('openLoopEig', linData.openLoopEig, 'Gp', Gp, 'Gq', Gq, 'Gr', Gr, ...
    'figurePath', fullfile(outDir, 'damper_root_locus.png'));
end

function outDir = localOutputDir()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'control_design');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
end
