repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_EUL';
load_system(model);

speed = 70;
alphaDeg = 2.63250658095872;
thetaDeg = alphaDeg;

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'V_init', [speed * cosd(alphaDeg); 0; speed * sind(alphaDeg)]);
assignin('base', 'eul_init', [0; deg2rad(thetaDeg); 0]);
assignin('base', 'omega_init', [0; 0; 0]);

ds = Simulink.SimulationData.Dataset;
t = [0; 0.001];
u = zeros(24, 2);
u(13:18, :) = 90;
u(19, :) = 1181.42797848053;
ds{1} = timeseries(u(1:12, :).', t);
ds{2} = timeseries(u(13:18, :).', t);
ds{3} = timeseries(u(19, :).', t);
ds{4} = timeseries(u(20, :).', t);
ds{5} = timeseries(u(21, :).', t);
ds{6} = timeseries(u(22, :).', t);
ds{7} = timeseries(u(23, :).', t);
ds{8} = timeseries(u(24, :).', t);

simIn = Simulink.SimulationInput(model);
simIn = simIn.setExternalInput(ds);
simIn = simIn.setModelParameter( ...
    'StopTime', '0.001', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'ReturnWorkspaceOutputs', 'on');
simOut = sim(simIn);

yout = simOut.yout;
vb = lastVec(yout{2}.Values.Data);
vba = lastVec(yout{7}.Values.Data);

fprintf('\nV_B check with zero wind\n');
fprintf('V_B   = [%.6f %.6f %.6f]\n', vb(1), vb(2), vb(3));
fprintf('V_BA  = [%.6f %.6f %.6f]\n', vba(1), vba(2), vba(3));
fprintf('diff  = [%.6e %.6e %.6e]\n', vba(1)-vb(1), vba(2)-vb(2), vba(3)-vb(3));
fprintf('max abs diff = %.6e\n', max(abs(vba - vb)));

function vec = lastVec(arr)
if ismatrix(arr) && size(arr, 2) >= 3
    vec = arr(end, end-2:end).';
else
    vec = arr(:);
end
end
