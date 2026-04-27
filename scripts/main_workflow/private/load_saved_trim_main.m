function trimRow = load_saved_trim_main(session, trimSpec)
%LOAD_SAVED_TRIM_MAIN Read one converged Homework 7 trim row directly.

summaryPath = fullfile(session.repoRoot, 'docs', 'homework7_trim', ...
    'homework7_trim_summary.csv');
if exist(summaryPath, 'file') ~= 2
    error('load_saved_trim_main:MissingTrimSummary', ...
        'Missing Homework 7 trim summary: %s', summaryPath);
end

tbl = readtable(summaryPath);
tbl = tbl(tbl.converged ~= 0, :);
if isempty(tbl)
    error('load_saved_trim_main:NoConvergedRows', ...
        'No converged trim rows found in %s.', summaryPath);
end

match = abs(tbl.speed_mps - trimSpec.cruiseSpeedMps) < 1e-6 & ...
    abs(tbl.bank_deg - trimSpec.bankDeg) < 1e-6;
if any(match)
    row = tbl(find(match, 1, 'first'), :); %#ok<FNDSB>
else
    bankRows = tbl(abs(tbl.bank_deg - trimSpec.bankDeg) < 1e-6, :);
    if isempty(bankRows)
        bankRows = tbl;
    end
    [~, idx] = min(abs(bankRows.speed_mps - trimSpec.cruiseSpeedMps));
    row = bankRows(idx, :);
end

trimRow = table2struct(row, 'ToScalar', true);
trimRow.meta = struct( ...
    'source', 'saved', ...
    'summaryPath', summaryPath);
end
