function quick_trim_test
% Quick trim test runner
% Runs the current trim path one or more times and prints a short pass/fail
% summary. Keep this lightweight so it is easy to rerun while debugging
% model-link or mask issues.

num_runs = 1;
stop_on_error = true;

orig_dir = pwd;
cleanup_obj = onCleanup(@() cd(orig_dir)); %#ok<NASGU>

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);

fprintf('Quick trim test in %s\n', script_dir);
fprintf('Requested runs: %d\n\n', num_runs);

results = repmat(struct( ...
    'run_idx', 0, ...
    'passed', false, ...
    'message', "", ...
    'trim_result_exists', false), num_runs, 1);

for run_idx = 1:num_runs
    fprintf('=== Run %d / %d ===\n', run_idx, num_runs);
    results(run_idx).run_idx = run_idx;

    try
        bdclose('all');

        evalin('base', 'Init_EVTOL_Main;');
        evalin('base', 'trimCase = TrimCase_Cruise75();');
        evalin('base', 'Trim_EVTOL_Main;');

        results(run_idx).passed = true;
        results(run_idx).message = "OK";
        results(run_idx).trim_result_exists = evalin('base', 'exist(''trimResult'', ''var'') == 1');

        fprintf('PASS\n');
        if results(run_idx).trim_result_exists
            fprintf('trimResult exists in base workspace.\n');
        end
    catch ME
        results(run_idx).message = string(ME.message);

        fprintf('FAIL\n');
        fprintf('%s\n', ME.message);
        fprintf('\nFull report:\n%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));

        if stop_on_error
            break;
        end
    end

    fprintf('\n');
end

assignin('base', 'quick_trim_test_results', results);
fprintf('Saved summary to base workspace variable quick_trim_test_results.\n');
end
