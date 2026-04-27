function trimCase = load_homework_trim_case(varargin)
%LOAD_HOMEWORK_TRIM_CASE Load a converged Homework 7 trim row.

p = inputParser;
p.addParameter('CruiseSpeed', 70, @(x) isnumeric(x) && isscalar(x));
p.addParameter('BankDeg', 0, @(x) isnumeric(x) && isscalar(x));
p.parse(varargin{:});

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
summaryPath = fullfile(repoRoot, 'docs', 'homework7_trim', 'homework7_trim_summary.csv');
if exist(summaryPath, 'file') ~= 2
    error('load_homework_trim_case:MissingTrimSummary', ...
        'Missing Homework 7 trim summary: %s', summaryPath);
end

tbl = readtable(summaryPath);
tbl = tbl(tbl.converged ~= 0, :);
if isempty(tbl)
    error('load_homework_trim_case:NoConvergedRows', ...
        'No converged trim rows were found in %s.', summaryPath);
end

match = abs(tbl.speed_mps - p.Results.CruiseSpeed) < 1e-6 & ...
    abs(tbl.bank_deg - p.Results.BankDeg) < 1e-6;
if any(match)
    row = tbl(find(match, 1, 'first'), :);
else
    levelRows = tbl(abs(tbl.bank_deg - p.Results.BankDeg) < 1e-6, :);
    if isempty(levelRows)
        levelRows = tbl;
    end
    [~, idx] = min(abs(levelRows.speed_mps - p.Results.CruiseSpeed));
    row = levelRows(idx, :);
end

trimCase = table2struct(row, 'ToScalar', true);
end
