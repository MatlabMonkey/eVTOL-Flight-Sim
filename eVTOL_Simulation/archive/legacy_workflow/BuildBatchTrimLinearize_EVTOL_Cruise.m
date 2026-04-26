function evtol_trim_db = BuildBatchTrimLinearize_EVTOL_Cruise(batch_cfg)
% BuildBatchTrimLinearize_EVTOL_Cruise
%
% Batch trim and linearization database builder for the eVTOL cruise plant.
% This function keeps the master database in function scope and uses the
% base workspace only as a scratch area for the legacy setup/trim scripts.
%
% Example:
%   evtol_trim_db = BuildBatchTrimLinearize_EVTOL_Cruise;
%
%   batch_cfg = struct();
%   batch_cfg.sweep.Vinf_mps = 60:5:80;
%   batch_cfg.sweep.front_tilt_deg = [85 90];
%   evtol_trim_db = BuildBatchTrimLinearize_EVTOL_Cruise(batch_cfg);

    if nargin < 1
        batch_cfg = struct();
    end

    %% User settings
    if ~isfield(batch_cfg, 'sweep') || isempty(batch_cfg.sweep)
        batch_cfg.sweep = struct();
    end

    % Edit these sweep vectors directly for now. This is the simplest setup.
    if ~isfield(batch_cfg.sweep, 'Vinf_mps')
        batch_cfg.sweep.Vinf_mps = 65:5:85;
    end
    if ~isfield(batch_cfg.sweep, 'front_tilt_deg')
        batch_cfg.sweep.front_tilt_deg = 90;
    end
    if ~isfield(batch_cfg.sweep, 'front_collective_guess_rpm')
        batch_cfg.sweep.front_collective_guess_rpm = 1180;
    end
    if ~isfield(batch_cfg.sweep, 'rear_collective_guess_rpm')
        batch_cfg.sweep.rear_collective_guess_rpm = 0;
    end

    if ~isfield(batch_cfg, 'model_name') || isempty(batch_cfg.model_name)
        batch_cfg.model_name = 'Trim_Plant';
    end
    if ~isfield(batch_cfg, 'workspace_var_name') || isempty(batch_cfg.workspace_var_name)
        batch_cfg.workspace_var_name = 'evtol_trim_db';
    end
    if ~isfield(batch_cfg, 'mat_filename') || isempty(batch_cfg.mat_filename)
        batch_cfg.mat_filename = 'evtol_trim_db.mat';
    end
    if ~isfield(batch_cfg, 'save_mat_file')
        batch_cfg.save_mat_file = true;
    end
    if ~isfield(batch_cfg, 'assign_to_base')
        batch_cfg.assign_to_base = true;
    end
    if ~isfield(batch_cfg, 'trim_verbose')
        batch_cfg.trim_verbose = false;
    end
    if ~isfield(batch_cfg, 'stop_on_error')
        batch_cfg.stop_on_error = false;
    end

    %% Build the sweep grid
    sweep_fields = fieldnames(batch_cfg.sweep);
    n_sweep = numel(sweep_fields);

    if n_sweep == 0
        error('batch_cfg.sweep must contain at least one sweep field.');
    end

    sweep_cells = cell(1, n_sweep);
    for iField = 1:n_sweep
        values = batch_cfg.sweep.(sweep_fields{iField});
        validateattributes(values, {'numeric'}, {'vector', 'nonempty'}, ...
            mfilename, ['batch_cfg.sweep.' sweep_fields{iField}]);
        sweep_cells{iField} = reshape(values, 1, []);
    end

    grid_cells = cell(1, n_sweep);
    [grid_cells{:}] = ndgrid(sweep_cells{:});
    n_scenarios = numel(grid_cells{1});

    %% Preallocate the database
    scenario_template = struct( ...
        'name', '', ...
        'index', 0, ...
        'sweep_values', struct(), ...
        'opspec', [], ...
        'op_trim', [], ...
        'op_report', [], ...
        'trim', struct(), ...
        'linear', struct(), ...
        'success', false, ...
        'diagnostic', struct(), ...
        'metadata', struct());

    evtol_trim_db = struct();
    evtol_trim_db.meta = struct();
    evtol_trim_db.meta.created_on = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    evtol_trim_db.meta.model_name = batch_cfg.model_name;
    evtol_trim_db.meta.setup_script = 'SetupForTrimAndLin_EVTOL_Cruise.m';
    evtol_trim_db.meta.trim_script = 'TrimAndLinearize_EVTOL_Cruise.m';
    evtol_trim_db.meta.opspec_helper = 'make_evtol_cruise_opspec.m';
    evtol_trim_db.meta.batch_cfg = batch_cfg;
    evtol_trim_db.meta.sweep_fields = sweep_fields;

    evtol_trim_db.summary = struct();
    evtol_trim_db.summary.n_scenarios = n_scenarios;
    evtol_trim_db.summary.n_success = 0;
    evtol_trim_db.summary.n_failed = 0;
    evtol_trim_db.summary.success_mask = false(n_scenarios, 1);

    evtol_trim_db.scenarios = repmat(scenario_template, n_scenarios, 1);
    evtol_trim_db.schedule_table = struct([]);

    %% Run the sweep
    for iScenario = 1:n_scenarios
        request = struct();
        for iField = 1:n_sweep
            request.(sweep_fields{iField}) = grid_cells{iField}(iScenario);
        end

        scenario_name = local_make_scenario_name(iScenario, request, sweep_fields);

        fprintf('\n[%d/%d] %s\n', iScenario, n_scenarios, scenario_name);

        scenario = scenario_template;
        scenario.name = scenario_name;
        scenario.index = iScenario;
        scenario.sweep_values = request;

        diagnostic = struct();
        diagnostic.had_exception = false;
        diagnostic.error_identifier = '';
        diagnostic.error_message = '';
        diagnostic.termination_string = '';
        diagnostic.trim_stage = '';

        try
            % Clear only the scratch variables used by the legacy scripts.
            evalin('base', ...
                ['clear(''', ...
                'opspec'', ''op_trim'', ''op_report'', ''trim_linearize_result'', ', ...
                '''trim_model_name'', ''trim_verbose'', ''trim_run_label'', ', ...
                '''trim_setup_defaults'')']);

            % Run existing setup in the base workspace.
            evalin('base', 'SetupForTrimAndLin_EVTOL_Cruise;');

            % Pull the packed setup defaults back into this function.
            trim_setup_defaults = evalin('base', 'trim_setup_defaults;');
            trim_setup_defaults.model_name = batch_cfg.model_name;

            % Build a scenario-specific opspec.
            [opspec, resolved_request] = make_evtol_cruise_opspec(request, trim_setup_defaults);

            % Push the opspec and run metadata into the base workspace.
            assignin('base', 'opspec', opspec);
            assignin('base', 'trim_model_name', batch_cfg.model_name);
            assignin('base', 'trim_verbose', batch_cfg.trim_verbose);
            assignin('base', 'trim_run_label', scenario_name);

            % Run the existing trim/linearize script.
            evalin('base', 'TrimAndLinearize_EVTOL_Cruise;');

            % Pull the packed result back out.
            result = evalin('base', 'trim_linearize_result;');

            scenario.sweep_values = resolved_request;
            scenario.opspec = opspec;
            scenario.op_trim = result.op_trim;
            scenario.op_report = result.op_report;
            scenario.trim = result.trim;
            scenario.linear = result.linear;
            scenario.metadata = struct();
            scenario.metadata.model_name = batch_cfg.model_name;
            scenario.metadata.run_label = scenario_name;
            scenario.metadata.scheduling = result.scheduling;

            scenario.success = local_result_success(result);
            diagnostic.termination_string = local_get_termination_string(result);

            fprintf('    success = %d\n', scenario.success);
            if ~isempty(diagnostic.termination_string)
                fprintf('    termination = %s\n', diagnostic.termination_string);
            end

        catch ME
            diagnostic.had_exception = true;
            diagnostic.error_identifier = ME.identifier;
            diagnostic.error_message = ME.message;

            if ~isempty(ME.stack)
                diagnostic.error_file = ME.stack(1).file;
                diagnostic.error_line = ME.stack(1).line;
            else
                diagnostic.error_file = '';
                diagnostic.error_line = NaN;
            end
            
            diagnostic.trim_stage = 'opspec_or_trim_or_linearize';

            scenario.success = false;
            scenario.diagnostic = diagnostic;

            fprintf('    FAILED: %s\n', ME.message);

            if batch_cfg.stop_on_error
                rethrow(ME);
            end
        end

        if isempty(fieldnames(scenario.diagnostic))
            scenario.diagnostic = diagnostic;
        else
            % Keep any existing diagnostic fields and add the exception info.
            scenario.diagnostic.had_exception = diagnostic.had_exception;
            scenario.diagnostic.error_identifier = diagnostic.error_identifier;
            scenario.diagnostic.error_message = diagnostic.error_message;
            scenario.diagnostic.termination_string = diagnostic.termination_string;
            scenario.diagnostic.trim_stage = diagnostic.trim_stage;
        end

        evtol_trim_db.scenarios(iScenario) = scenario;
        evtol_trim_db.summary.success_mask(iScenario) = scenario.success;
        evtol_trim_db.summary.n_success = nnz(evtol_trim_db.summary.success_mask);
        evtol_trim_db.summary.n_failed = iScenario - evtol_trim_db.summary.n_success;

        % Save partial progress so one later failure does not wipe the run.
        if batch_cfg.assign_to_base
            assignin('base', batch_cfg.workspace_var_name, evtol_trim_db);
        end
        if batch_cfg.save_mat_file
            save(batch_cfg.mat_filename, 'evtol_trim_db', '-v7.3');
        end
    end

    %% Build a lightweight scheduling table at the end
    evtol_trim_db.schedule_table = local_build_schedule_table(evtol_trim_db.scenarios);

    if batch_cfg.assign_to_base
        assignin('base', batch_cfg.workspace_var_name, evtol_trim_db);
    end
    if batch_cfg.save_mat_file
        save(batch_cfg.mat_filename, 'evtol_trim_db', '-v7.3');
    end

    fprintf('\nBatch trim complete.\n');
    fprintf('  Success: %d / %d\n', evtol_trim_db.summary.n_success, evtol_trim_db.summary.n_scenarios);
    fprintf('  Saved variable: %s\n', batch_cfg.workspace_var_name);
    if batch_cfg.save_mat_file
        fprintf('  Saved file: %s\n', batch_cfg.mat_filename);
    end
end

function name = local_make_scenario_name(iScenario, request, sweep_fields)
    parts = {sprintf('scn_%03d', iScenario)};

    for iField = 1:numel(sweep_fields)
        field_name = sweep_fields{iField};
        value = request.(field_name);

        if isscalar(value)
            value_str = strtrim(num2str(value, '%.4g'));
            value_str = strrep(value_str, '.', 'p');
            value_str = strrep(value_str, '-', 'm');
        else
            value_str = 'vec';
        end

        parts{end+1} = [field_name '_' value_str]; 
    end

    name = matlab.lang.makeValidName(strjoin(parts, '__'));
end

function tf = local_result_success(result)
    tf = false;

    if ~isstruct(result)
        return;
    end
    if ~isfield(result, 'trim') || ~isfield(result, 'linear')
        return;
    end
    if ~isfield(result.trim, 'X_trim') || ~isfield(result.trim, 'U_trim')
        return;
    end
    if ~isfield(result.linear, 'A_full') || ~isfield(result.linear, 'B_full')
        return;
    end

    tf = all(isfinite(result.trim.X_trim(:))) && ...
         all(isfinite(result.trim.U_trim(:))) && ...
         all(isfinite(result.linear.A_full(:))) && ...
         all(isfinite(result.linear.B_full(:)));
end

function term_str = local_get_termination_string(result)
    term_str = '';

    try
        term_str = result.op_report.TerminationString;
    catch
    end
end

function schedule_table = local_build_schedule_table(scenarios)
    template = struct( ...
        'index', 0, ...
        'name', '', ...
        'success', false, ...
        'Vinf_mps', nan, ...
        'alpha_rad', nan, ...
        'beta_rad', nan, ...
        'phi_rad', nan, ...
        'theta_rad', nan, ...
        'psi_rad', nan, ...
        'front_tilt_deg', nan, ...
        'front_collective_rpm', nan, ...
        'rear_collective_rpm', nan);

    n = numel(scenarios);
    schedule_table = repmat(template, n, 1);

    for i = 1:n
        schedule_table(i).index = scenarios(i).index;
        schedule_table(i).name = scenarios(i).name;
        schedule_table(i).success = scenarios(i).success;

        if scenarios(i).success && isfield(scenarios(i), 'metadata') && ...
                isfield(scenarios(i).metadata, 'scheduling')

            s = scenarios(i).metadata.scheduling;

            schedule_table(i).Vinf_mps = s.Vinf_mps;
            schedule_table(i).alpha_rad = s.alpha_rad;
            schedule_table(i).beta_rad = s.beta_rad;
            schedule_table(i).phi_rad = s.phi_rad;
            schedule_table(i).theta_rad = s.theta_rad;
            schedule_table(i).psi_rad = s.psi_rad;
            schedule_table(i).front_tilt_deg = s.front_tilt_deg;
            schedule_table(i).front_collective_rpm = s.front_collective_rpm;
            schedule_table(i).rear_collective_rpm = s.rear_collective_rpm;
        end
    end
end
