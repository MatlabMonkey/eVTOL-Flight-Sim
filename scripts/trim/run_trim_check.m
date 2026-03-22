% scripts/trim/run_trim_check.m
% Wrapper to execute trim_cruise and persist a machine-readable artifact.

fprintf('== run_trim_check ==\n');
fprintf('Timestamp: %s\n', datestr(now, 31));

try
    result = trim_cruise('CruiseSpeed', 70, 'TiltDeg', 90);

    if isfield(result, 'method')
        fprintf('Method: %s\n', result.method);
    end
    if isfield(result, 'fallback')
        fb = result.fallback;
        fprintf('Fallback estimate: front rpm %.1f, rear rpm %.1f\n', ...
            fb.rpm_front_for_drag, fb.rpm_rear_for_full_weight);
    end

    outDir = fullfile('docs', 'evidence');
    if ~exist(outDir, 'dir'); mkdir(outDir); end

    save(fullfile(outDir, 'trim_check_latest.mat'), 'result');

    % JSON export when available (R2016b+)
    try
        jsonText = jsonencode(result);
        fid = fopen(fullfile(outDir, 'trim_check_latest.json'), 'w');
        fprintf(fid, '%s\n', jsonText);
        fclose(fid);
    catch
        % If jsonencode unavailable, skip JSON export.
    end

    fprintf('TRIM_CHECK: PASS (script executed)\n');
catch ME
    fprintf('TRIM_CHECK: FAIL\n');
    fprintf('Error: %s\n', ME.message);
    rethrow(ME);
end
