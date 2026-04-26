%% Plot final wing and tail aerodynamic coefficient tables
% Report-ready white-background plots of the actual lookup-table sample points.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
builderScript = fullfile(repoRoot, 'scripts', 'avl', 'build_final_airfoil_polar_tables.m');
dataFile = fullfile(repoRoot, 'scripts', 'avl', 'generated', 'final_airfoil_polar_tables.mat');
outputDir = fullfile(repoRoot, 'documentation');
wingOutputPng = fullfile(outputDir, 'wing_airfoil_polar_tables.png');
tailOutputPng = fullfile(outputDir, 'tail_airfoil_polar_tables.png');

if exist(dataFile, 'file') ~= 2
    run(builderScript);
end

data = load(dataFile, 'wingPolar', 'tailPolar');
wingPolar = data.wingPolar;
tailPolar = data.tailPolar;

plotOneSurface(wingPolar, 'Wing Aerodynamic Coefficient Tables', wingOutputPng);
plotOneSurface(tailPolar, 'Tail Aerodynamic Coefficient Tables', tailOutputPng);

if exist(outputDir, 'dir') ~= 7
    mkdir(outputDir);
end

fprintf('Saved plot: %s\n', wingOutputPng);
fprintf('Saved plot: %s\n', tailOutputPng);

function plotOneSurface(polar, figTitle, outputPng)
figure('Color', 'w', 'Position', [140 140 1280 560]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

clColor = [0.00 0.45 0.74];
cdColor = [0.85 0.33 0.10];
cmColor = [0.47 0.67 0.19];
markerSize = 30;
lineWidth = 1.2;

nexttile;
hold on;
grid on;
box on;
scatter(polar.alpha_deg, polar.CL, markerSize, 'o', ...
    'MarkerEdgeColor', clColor, 'MarkerFaceColor', 'none', 'LineWidth', lineWidth);
scatter(polar.alpha_deg, polar.CD, markerSize, 's', ...
    'MarkerEdgeColor', cdColor, 'MarkerFaceColor', 'none', 'LineWidth', lineWidth);
scatter(polar.alpha_deg, polar.Cm, markerSize, '^', ...
    'MarkerEdgeColor', cmColor, 'MarkerFaceColor', 'none', 'LineWidth', lineWidth);
xlabel('\alpha (deg)');
ylabel('Coefficient Value');
title([figTitle ' - Full Range'], 'Color', 'k', 'FontWeight', 'bold');
xlim([-180 180]);
legend('C_L table points', 'C_D table points', 'C_m table points', ...
    'Location', 'best');
localStyleAxes(gca);

nexttile;
hold on;
grid on;
box on;
scatter(polar.alpha_deg, polar.CL, markerSize, 'o', ...
    'MarkerEdgeColor', clColor, 'MarkerFaceColor', 'none', 'LineWidth', lineWidth);
scatter(polar.alpha_deg, polar.CD, markerSize, 's', ...
    'MarkerEdgeColor', cdColor, 'MarkerFaceColor', 'none', 'LineWidth', lineWidth);
scatter(polar.alpha_deg, polar.Cm, markerSize, '^', ...
    'MarkerEdgeColor', cmColor, 'MarkerFaceColor', 'none', 'LineWidth', lineWidth);
xlabel('\alpha (deg)');
ylabel('Coefficient Value');
title([figTitle ' - Low-Angle Region'], 'Color', 'k', 'FontWeight', 'bold');
xlim([-20 20]);
legend('C_L table points', 'C_D table points', 'C_m table points', ...
    'Location', 'best');
localStyleAxes(gca);

exportgraphics(gcf, outputPng, 'Resolution', 300);
end

function localStyleAxes(ax)
set(ax, ...
    'Color', 'w', ...
    'XColor', 'k', ...
    'YColor', 'k', ...
    'GridColor', [0.75 0.75 0.75], ...
    'GridAlpha', 0.35, ...
    'LineWidth', 1.0, ...
    'FontSize', 12);
end
