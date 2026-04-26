%% Minimal LUT Aero init for manual Simulink run
% Loads the exact workspace variables and mask values needed for the LUT
% Aero subsystem, then opens the test model. Press Play manually after this.

origDir = pwd;
cleanupDir = onCleanup(@() cd(origDir)); %#ok<NASGU>

stack = dbstack('-completenames');
if ~isempty(stack)
    scriptDir = fileparts(stack(1).file);
else
    scriptDir = pwd;
end
repoRoot = fileparts(fileparts(scriptDir));
cd(repoRoot);

polarData = load(fullfile(repoRoot, 'scripts', 'avl', 'generated', 'final_airfoil_polar_tables.mat'), 'wingPolar');
aircraft = aircraft_def('flight_mode', 0);

wingR = aircraft.wingR; %#ok<NASGU>
if isfield(wingR, 'name')
    wingR = rmfield(wingR, 'name');
end
CG = aircraft.CG; %#ok<NASGU>
wingPolar = polarData.wingPolar; %#ok<NASGU>

assignin('base', 'wingR', wingR);
assignin('base', 'CG', CG);
assignin('base', 'wingPolar', wingPolar);

load_system('Brown_Flight_Controls_lib');
load_system('LUTaeroTest');

set_param('LUTaeroTest/LUT Aero', ...
    'surf', 'wingR', ...
    'CG', 'CG', ...
    'polar', 'wingPolar');

open_system('LUTaeroTest');

fprintf('\nInitialized LUTaeroTest with:\n');
fprintf('  surf  = wingR\n');
fprintf('  CG    = CG\n');
fprintf('  polar = wingPolar\n');
fprintf('Press Play in Simulink to run the model.\n\n');
