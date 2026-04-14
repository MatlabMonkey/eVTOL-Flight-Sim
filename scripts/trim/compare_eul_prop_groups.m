repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
assignin('base', 'use_avl_aero', false);

model = 'Brown_6DOF_Plant_EUL';
load_system(model);
propBlk = findPropBlockName(model);

speed = 70;
alphaDeg = 2.63250658095872;
frontRPM = 1181.42797848053;

assignin('base', 'pos_init', [0; 0; -1000]);
assignin('base', 'V_init', [speed * cosd(alphaDeg); 0; speed * sind(alphaDeg)]);
assignin('base', 'eul_init', [0; deg2rad(alphaDeg); 0]);
assignin('base', 'omega_init', [0; 0; 0]);

tempBlocks = {};
cleanupModel = onCleanup(@() cleanupTempBlocks(propBlk, tempBlocks)); %#ok<NASGU>

names = {'Front Right','Front Left','Rear Right','Rear Left'};
vars = {'tmp_frF','tmp_frM','tmp_flF','tmp_flM','tmp_rrF','tmp_rrM','tmp_rlF','tmp_rlM'};
idx = 1;
for k = 1:numel(names)
    tempBlocks{end+1} = addLogBlock(propBlk, names{k}, 1, vars{idx}); idx = idx + 1;
    tempBlocks{end+1} = addLogBlock(propBlk, names{k}, 2, vars{idx}); idx = idx + 1;
end

t = [0; 0.001];
ds = Simulink.SimulationData.Dataset;
ds{1} = timeseries(repmat(zeros(12, 1), 1, 2).', t);
ds{2} = timeseries(repmat(90 * ones(6, 1), 1, 2).', t);
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

labels = {'FR','FL','RR','RL'};
for k = 1:numel(labels)
    F = lastVec(simOut.get(vars{2*k-1}));
    M = lastVec(simOut.get(vars{2*k}));
    fprintf('%s force  = [%9.3f %9.3f %9.3f] N\n', labels{k}, F(1), F(2), F(3));
    fprintf('%s moment = [%9.3f %9.3f %9.3f] N-m\n', labels{k}, M(1), M(2), M(3));
end

close_system(model, 0);

function name = findPropBlockName(model)
candidates = {'Propellers','Propellers1'};
for k = 1:numel(candidates)
    if ~isempty(find_system(model, 'SearchDepth', 1, 'Name', candidates{k}))
        name = [model '/' candidates{k}];
        return;
    end
end
error('Could not find top-level propeller subsystem in %s', model);
end

function blk = addLogBlock(subsys, srcName, outIdx, varName)
src = [subsys '/' srcName];
dst = [subsys '/' varName];
pos = get_param(src, 'Position');
x = pos(3) + 80 + 90 * (outIdx - 1);
y = pos(2) + 20 * outIdx;
add_block('simulink/Sinks/To Workspace', dst, ...
    'VariableName', varName, ...
    'SaveFormat', 'Array', ...
    'Position', [x y x + 80 y + 20]);
add_line(subsys, [srcName '/' num2str(outIdx)], [varName '/1'], 'autorouting', 'on');
blk = dst;
end

function cleanupTempBlocks(subsys, blocks)
modelName = regexp(subsys, '^[^/]+', 'match', 'once');
if isempty(modelName) || ~bdIsLoaded(modelName)
    return;
end
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', regexp(subsys, '[^/]+$', 'match', 'once')))
    return;
end
for k = 1:numel(blocks)
    if isempty(blocks{k})
        continue;
    end
    blkName = regexp(blocks{k}, '[^/]+$', 'match', 'once');
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', blkName))
        delete_block(blocks{k});
    end
end
set_param(modelName, 'Dirty', 'off');
vars = {'tmp_frF','tmp_frM','tmp_flF','tmp_flM','tmp_rrF','tmp_rrM','tmp_rlF','tmp_rlM'};
for k = 1:numel(vars)
    evalin('base', sprintf('if exist(''%s'',''var''), clear(''%s''); end', vars{k}, vars{k}));
end
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
