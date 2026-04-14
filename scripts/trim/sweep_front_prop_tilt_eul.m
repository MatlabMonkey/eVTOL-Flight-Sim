repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_EUL';
load_system(model);
propBlkName = findPropBlockName(model);

speed = 70;
alphaDeg = 2.63250658095872;
thetaDeg = alphaDeg;
frontRPM = 1181.42797848053;

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'V_init', [speed * cosd(alphaDeg); 0; speed * sind(alphaDeg)]);
assignin('base', 'eul_init', [0; deg2rad(thetaDeg); 0]);
assignin('base', 'omega_init', [0; 0; 0]);

tempBlock = '';
cleanupModel = onCleanup(@() cleanupTempBlock(model, tempBlock)); %#ok<NASGU>
tempBlock = addLogBlock(model, propBlkName, 1, 'tmp_propF');

tilts = [0 45 90];
forces = zeros(3, numel(tilts));

for k = 1:numel(tilts)
    t = [0; 0.001];
    ds = Simulink.SimulationData.Dataset;
    ds{1} = timeseries(repmat(zeros(12, 1), 1, 2).', t);
    ds{2} = timeseries(repmat(tilts(k) * ones(6, 1), 1, 2).', t);
    ds{3} = timeseries(repmat(frontRPM, 2, 1), t);
    ds{4} = timeseries(repmat(0, 2, 1), t);
    ds{5} = timeseries(repmat(0, 2, 1), t);
    ds{6} = timeseries(repmat(0, 2, 1), t);
    ds{7} = timeseries(repmat(0, 2, 1), t);
    ds{8} = timeseries(repmat(0, 2, 1), t);

    simIn = Simulink.SimulationInput(model);
    simIn = simIn.setExternalInput(ds);
    simIn = simIn.setModelParameter('StopTime', '0.001', 'ReturnWorkspaceOutputs', 'on');
    simOut = sim(simIn);
    forces(:, k) = lastVec(simOut.get('tmp_propF'));
end

fprintf('\nFront prop tilt sweep on %s\n', model);
for k = 1:numel(tilts)
    fprintf('  tilt %5.1f deg -> F = [%9.3f %9.3f %9.3f] N\n', ...
        tilts(k), forces(1, k), forces(2, k), forces(3, k));
end

close_system(model, 0);

function blk = addLogBlock(model, srcName, outIdx, varName)
src = [model '/' srcName];
dst = [model '/' varName];
pos = get_param(src, 'Position');
x = pos(3) + 120 + 110 * (outIdx - 1);
y = pos(2) + 30 * outIdx;
add_block('simulink/Sinks/To Workspace', dst, ...
    'VariableName', varName, ...
    'SaveFormat', 'Array', ...
    'Position', [x y x + 100 y + 20]);
add_line(model, [srcName '/' num2str(outIdx)], [varName '/1'], 'autorouting', 'on');
blk = dst;
end

function name = findPropBlockName(model)
candidates = {'Propellers','Propellers1'};
for k = 1:numel(candidates)
    if ~isempty(find_system(model,'SearchDepth',1,'Name',candidates{k}))
        name = candidates{k};
        return;
    end
end
error('Could not find top-level propeller subsystem in %s', model);
end

function cleanupTempBlock(model, blk)
if bdIsLoaded(model) && ~isempty(blk)
    if ~isempty(find_system(model, 'SearchDepth', 1, 'Name', get_param(blk, 'Name')))
        delete_block(blk);
    end
    set_param(model, 'Dirty', 'off');
end
evalin('base', 'if exist(''tmp_propF'',''var''), clear tmp_propF; end');
end

function vec = lastVec(arr)
sz = size(arr);
if ndims(arr) == 3 && sz(1) == 3
    vec = squeeze(arr(:, 1, end));
elseif ismatrix(arr) && size(arr, 2) >= 3
    vec = arr(end, end-2:end).';
else
    vec = arr(:);
end
end
