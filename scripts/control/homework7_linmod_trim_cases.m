function linResults = homework7_linmod_trim_cases(varargin)
%HOMEWORK7_LINMOD_TRIM_CASES Linearize the Homework 7 trim cases with linmod().

p = inputParser;
p.addParameter('TrimResultsFile', '', @(s) ischar(s) || isstring(s));
p.addParameter('RerunTrimIfMissing', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outDir = fullfile(repoRoot, 'docs', 'homework7_trim');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
addpath(genpath(fullfile(repoRoot, 'scripts')));

trimFile = char(p.Results.TrimResultsFile);
if isempty(trimFile)
    trimFile = fullfile(outDir, 'homework7_trim_results.mat');
end

if ~exist(trimFile, 'file')
    if p.Results.RerunTrimIfMissing
        homework7_trim_cases();
    else
        error('homework7_linmod_trim_cases:MissingTrimResults', ...
            'Could not find trim results file: %s', trimFile);
    end
end

trimData = load(trimFile, 'results', 'meta');
trimResults = trimData.results;
trimMeta = trimData.meta;

evalin('base', 'render_enable = false; run(''Full_Sim_Init.m'');');
assignin('base', 'use_avl_aero', logical(trimMeta.use_avl_aero));
build_control_analysis_helper();
load_system('Brown_6DOF_Plant');
load_system('Brown_Control_Analysis_Helper');

linResults = repmat(emptyLinStruct(), numel(trimResults), 1);
eigRows = [];
for k = 1:numel(trimResults)
    trimCase = trimResults(k);
    [A, B, C, D] = linmod('Brown_Control_Analysis_Helper', trimCase.x_trim, trimCase.u_trim);
    eigVals = eig(A);

    linResults(k).case_name = trimCase.case_name;
    linResults(k).speed_mps = trimCase.speed_mps;
    linResults(k).bank_deg = trimCase.bank_deg;
    linResults(k).converged_trim = trimCase.converged;
    linResults(k).A = A;
    linResults(k).B = B;
    linResults(k).C = C;
    linResults(k).D = D;
    linResults(k).eig = eigVals;
    linResults(k).state_count = size(A, 1);
    linResults(k).input_count = size(B, 2);
    linResults(k).output_count = size(C, 1);
    linResults(k).max_real_eig = max(real(eigVals));
    linResults(k).min_real_eig = min(real(eigVals));

    for i = 1:numel(eigVals)
        eigRows = [eigRows; {string(trimCase.case_name), trimCase.speed_mps, trimCase.bank_deg, i, real(eigVals(i)), imag(eigVals(i))}]; %#ok<AGROW>
    end
end

summaryTbl = struct2table(arrayfun(@summarizeLinResult, linResults));
eigTbl = cell2table(eigRows, 'VariableNames', ...
    {'case_name', 'speed_mps', 'bank_deg', 'eig_index', 'eig_real', 'eig_imag'});

summaryCsv = fullfile(outDir, 'homework7_linmod_summary.csv');
eigCsv = fullfile(outDir, 'homework7_linmod_eigs.csv');
writetable(summaryTbl, summaryCsv);
writetable(eigTbl, eigCsv);

resultsMat = fullfile(outDir, 'homework7_linmod_results.mat');
save(resultsMat, 'linResults', 'summaryTbl', 'eigTbl', 'trimFile');

fprintf('Homework 7 linmod results saved to:\n  %s\n  %s\n  %s\n', ...
    resultsMat, summaryCsv, eigCsv);

close_system('Brown_6DOF_Plant', 0);
close_system('Brown_Control_Analysis_Helper', 0);
end

function out = summarizeLinResult(in)
out = struct();
out.case_name = string(in.case_name);
out.speed_mps = in.speed_mps;
out.bank_deg = in.bank_deg;
out.converged_trim = in.converged_trim;
out.state_count = in.state_count;
out.input_count = in.input_count;
out.output_count = in.output_count;
out.max_real_eig = in.max_real_eig;
out.min_real_eig = in.min_real_eig;
end

function s = emptyLinStruct()
s = struct( ...
    'case_name', '', ...
    'speed_mps', nan, ...
    'bank_deg', nan, ...
    'converged_trim', false, ...
    'A', [], ...
    'B', [], ...
    'C', [], ...
    'D', [], ...
    'eig', [], ...
    'state_count', nan, ...
    'input_count', nan, ...
    'output_count', nan, ...
    'max_real_eig', nan, ...
    'min_real_eig', nan);
end
