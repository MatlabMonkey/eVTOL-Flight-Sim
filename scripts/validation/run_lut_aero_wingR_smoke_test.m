%% Dead-simple LUT Aero smoke test
% Runs LUTaeroTest with a numeric-only wingR struct, then prints the final
% F_cg and M_cg values from the logged timeseries.

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

polarData = load(fullfile(repoRoot, 'scripts', 'avl', 'generated', ...
    'final_airfoil_polar_tables.mat'), 'wingPolar');
aircraft = aircraft_def('flight_mode', 0);

wingR_meta = aircraft.wingR; %#ok<NASGU>
wingR = aircraft.wingR;
if isfield(wingR, 'name')
    wingR = rmfield(wingR, 'name');
end
CG = aircraft.CG;
wingPolar = polarData.wingPolar;

assignin('base', 'wingR_meta', wingR_meta);
assignin('base', 'wingR', wingR);
assignin('base', 'CG', CG);
assignin('base', 'wingPolar', wingPolar);

t = [0; 2.0];
assignin('base', 'simin',  timeseries(repmat([70 0 0], 2, 1), t));
assignin('base', 'simin1', timeseries(repmat([0 0 0], 2, 1), t));
assignin('base', 'simin2', timeseries([0; 0], t));

load_system('Brown_Flight_Controls_lib');
load_system('LUTaeroTest');

set_param('LUTaeroTest/LUT Aero', ...
    'surf', 'wingR', ...
    'CG', 'CG', ...
    'polar', 'wingPolar');

simOut = sim('LUTaeroTest', 'StopTime', '2.0', 'ReturnWorkspaceOutputs', 'on');

F_ts = simOut.get('F_cg');
M_ts = simOut.get('M_cg');

F_cg_last = squeeze(F_ts.Data(:, :, end));
M_cg_last = squeeze(M_ts.Data(:, :, end));

fprintf('\nFinal LUT Aero values at t = %.3f s\n', F_ts.Time(end));
fprintf('F_cg = [% .6f, % .6f, % .6f]^T\n', F_cg_last);
fprintf('M_cg = [% .6f, % .6f, % .6f]^T\n', M_cg_last);

assignin('base', 'lutAeroSmoke_F_cg_last', F_cg_last);
assignin('base', 'lutAeroSmoke_M_cg_last', M_cg_last);
