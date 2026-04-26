function evtol_trim_db = AddLQRToBatchTrimDB_EVTOL_Cruise(db_in, lqr_cfg)
% AddLQRToBatchTrimDB_EVTOL_Cruise
%
% Batch-runs Design_LQR_Controller_EVTOL_Cruise.m for every successful
% trim/linearization scenario in evtol_trim_db and stores the controller
% results back into the same struct.
%
% You can pass either:
%   1) a struct already in memory
%   2) a .mat filename that contains evtol_trim_db
%
% Example:
%   evtol_trim_db = AddLQRToBatchTrimDB_EVTOL_Cruise('evtol_trim_db.mat');
%
%   evtol_trim_db = AddLQRToBatchTrimDB_EVTOL_Cruise('evtol_trim_db.mat', ...
%       struct('save_mat_file', true, 'assign_to_base', true));
%
%   evtol_trim_db = AddLQRToBatchTrimDB_EVTOL_Cruise(evtol_trim_db);

    if nargin < 1 || isempty(db_in)
        db_in = 'evtol_trim_db.mat';
    end

    if nargin < 2
        lqr_cfg = struct();
    end

    %% Config defaults
    if ~isfield(lqr_cfg, 'mat_filename') || isempty(lqr_cfg.mat_filename)
        if ischar(db_in) || isstring(db_in)
            lqr_cfg.mat_filename = char(db_in);
        else
            lqr_cfg.mat_filename = 'evtol_trim_db.mat';
        end
    end

    if ~isfield(lqr_cfg, 'workspace_var_name') || isempty(lqr_cfg.workspace_var_name)
        lqr_cfg.workspace_var_name = 'evtol_trim_db';
    end

    if ~isfield(lqr_cfg, 'assign_to_base')
        lqr_cfg.assign_to_base = true;
    end

    if ~isfield(lqr_cfg, 'save_mat_file')
        lqr_cfg.save_mat_file = true;
    end

    if ~isfield(lqr_cfg, 'stop_on_error')
        lqr_cfg.stop_on_error = false;
    end

    if ~isfield(lqr_cfg, 'overwrite_existing')
        lqr_cfg.overwrite_existing = false;
    end

    if ~isfield(lqr_cfg, 'verbose')
        lqr_cfg.verbose = true;
    end

    %% Load database
    if isstruct(db_in)
        evtol_trim_db = db_in;
    elseif ischar(db_in) || isstring(db_in)
        S = load(char(db_in), 'evtol_trim_db');
        if ~isfield(S, 'evtol_trim_db')
            error('The MAT file does not contain variable "evtol_trim_db".');
        end
        evtol_trim_db = S.evtol_trim_db;
    else
        error('db_in must be either a struct or a MAT filename.');
    end

    if ~isfield(evtol_trim_db, 'scenarios') || isempty(evtol_trim_db.scenarios)
        error('evtol_trim_db.scenarios is missing or empty.');
    end
    
    %% Normalize scenario struct fields so later assignments do not fail
    if ~isfield(evtol_trim_db.scenarios, 'controller')
        empty_controller = struct( ...
            'design_name', '', ...
            'success', false, ...
            'meta', struct(), ...
            'K_lqr_cruise', [], ...
            'x_trim_lqr', [], ...
            'U_trim_lqr', [], ...
            'use_full_9state', [], ...
            'cl_eigs', [], ...
            'S_lqr', [], ...
            'Q_use', [], ...
            'R_use', [], ...
            'keep_idx', [], ...
            'schedule_point', struct(), ...
            'diagnostic', struct());
    
        for k = 1:numel(evtol_trim_db.scenarios)
            evtol_trim_db.scenarios(k).controller = empty_controller;
        end
    end
    %% Set up shared constants from your existing setup script
    % This keeps surface_limit_deg consistent with the rest of the project.
    evalin('base', 'SetupForTrimAndLin_EVTOL_Cruise;');
    surface_limit_deg_local = evalin('base', 'surface_limit_deg;');

    %% Prepare controller summary fields
    if ~isfield(evtol_trim_db, 'controller_summary') || isempty(evtol_trim_db.controller_summary)
        evtol_trim_db.controller_summary = struct();
    end

    evtol_trim_db.controller_summary.design_script = 'Design_LQR_Controller_EVTOL_Cruise.m';
    evtol_trim_db.controller_summary.created_on = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    evtol_trim_db.controller_summary.n_attempted = 0;
    evtol_trim_db.controller_summary.n_success = 0;
    evtol_trim_db.controller_summary.n_failed = 0;

    %% Loop over scenarios
    n_scenarios = numel(evtol_trim_db.scenarios);

    for iScenario = 1:n_scenarios
        scn = evtol_trim_db.scenarios(iScenario);

        if lqr_cfg.verbose
            fprintf('\n[LQR %d/%d] %s\n', iScenario, n_scenarios, scn.name);
        end

        % Make sure the scenario has a controller field
        if ~isfield(scn, 'controller') || isempty(scn.controller)
            scn.controller = struct();
        end

        scn.controller.design_name = 'LQR_Cruise';
        scn.controller.success = false;
        scn.controller.diagnostic = struct();

        % Skip failed trim points
        if ~isfield(scn, 'success') || ~scn.success
            scn.controller.diagnostic.message = 'Skipped because trim/linearization failed.';
            evtol_trim_db.scenarios(iScenario).controller = scn.controller;

            if lqr_cfg.verbose
                fprintf('    skipped: trim scenario was not successful\n');
            end
            continue;
        end

        % Skip already-designed points unless overwrite is requested
        if ~lqr_cfg.overwrite_existing && ...
           isfield(scn.controller, 'success') && scn.controller.success && ...
           isfield(scn.controller, 'design_name') && strcmp(scn.controller.design_name, 'LQR_Cruise')

            if lqr_cfg.verbose
                fprintf('    skipped: controller already exists\n');
            end
            evtol_trim_db.scenarios(iScenario).controller = scn.controller;
            continue;
        end

        evtol_trim_db.controller_summary.n_attempted = evtol_trim_db.controller_summary.n_attempted + 1;

        try
            % Clear only the scratch variables used by the design script
            evalin('base', ['clear(''', ...
                'sys_ss_13state'', ''Att_Trim'', ''Vel_B_BA_Trim'', ''Rates_Trim'', ', ...
                '''U_trim'', ''surface_limit_deg'', ''K_lqr_cruise'', ''K_lqr'', ', ...
                '''x_trim_lqr'', ''U_trim_lqr'', ''S_lqr'', ''cl_eigs'', ', ...
                '''use_full_9state'', ''keep_idx'', ''A_lqr'', ''B_lqr'', ', ...
                '''A_use'', ''B_use'', ''Q_use'', ''R_use'')']);

            % Push scenario data into base workspace so the existing script
            % can run unchanged.
            assignin('base', 'sys_ss_13state', scn.linear.sys_ss_13state);
            assignin('base', 'Att_Trim', scn.trim.Att_Trim);
            assignin('base', 'Vel_B_BA_Trim', scn.trim.Vel_B_BA_Trim);
            assignin('base', 'Rates_Trim', scn.trim.Rates_Trim);
            assignin('base', 'U_trim', scn.trim.U_trim);
            assignin('base', 'surface_limit_deg', surface_limit_deg_local);

            % Run the existing project script
            evalin('base', 'Design_LQR_Controller_EVTOL_Cruise;');

            % Pull results back out
            controller = struct();
            controller.design_name = 'LQR_Cruise';
            controller.success = true;

            controller.meta = struct();
            controller.meta.created_on = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
            controller.meta.source_script = 'Design_LQR_Controller_EVTOL_Cruise.m';
            controller.meta.scenario_name = scn.name;
            controller.meta.scenario_index = scn.index;

            controller.K_lqr_cruise = evalin('base', 'K_lqr_cruise;');
            controller.x_trim_lqr = evalin('base', 'x_trim_lqr;');
            controller.U_trim_lqr = evalin('base', 'U_trim_lqr;');

            controller.use_full_9state = evalin('base', 'use_full_9state;');
            controller.cl_eigs = evalin('base', 'cl_eigs;');
            controller.S_lqr = evalin('base', 'S_lqr;');

            controller.Q_use = evalin('base', 'Q_use;');
            controller.R_use = evalin('base', 'R_use;');
            controller.keep_idx = evalin('base', 'keep_idx;');

            controller.diagnostic = struct();
            controller.diagnostic.message = '';

            % Handy scheduling bundle for later gain scheduling logic
            controller.schedule_point = struct();
            if isfield(scn, 'metadata') && isfield(scn.metadata, 'scheduling')
                controller.schedule_point = scn.metadata.scheduling;
            end

            scn.controller = controller;
            evtol_trim_db.scenarios(iScenario).controller = scn.controller;
            evtol_trim_db.controller_summary.n_success = evtol_trim_db.controller_summary.n_success + 1;

            if lqr_cfg.verbose
                fprintf('    success = 1\n');
            end

        catch ME
            controller = struct();
            controller.design_name = 'LQR_Cruise';
            controller.success = false;

            controller.diagnostic = struct();
            controller.diagnostic.error_identifier = ME.identifier;
            controller.diagnostic.error_message = ME.message;

            if ~isempty(ME.stack)
                controller.diagnostic.error_file = ME.stack(1).file;
                controller.diagnostic.error_line = ME.stack(1).line;
            else
                controller.diagnostic.error_file = '';
                controller.diagnostic.error_line = NaN;
            end

            scn.controller = controller;
            evtol_trim_db.scenarios(iScenario).controller = scn.controller;
            evtol_trim_db.controller_summary.n_failed = evtol_trim_db.controller_summary.n_failed + 1;

            if lqr_cfg.verbose
                fprintf('    FAILED: %s\n', ME.message);
            end

            if lqr_cfg.stop_on_error
                rethrow(ME);
            end
        end

        % Save partial progress after every scenario
        if lqr_cfg.assign_to_base
            assignin('base', lqr_cfg.workspace_var_name, evtol_trim_db);
        end

        if lqr_cfg.save_mat_file
            save(lqr_cfg.mat_filename, 'evtol_trim_db', '-v7.3');
        end
    end

    %% Build a lightweight controller schedule table
    evtol_trim_db.controller_schedule_table = local_build_controller_schedule_table(evtol_trim_db.scenarios);

    if lqr_cfg.assign_to_base
        assignin('base', lqr_cfg.workspace_var_name, evtol_trim_db);
    end

    if lqr_cfg.save_mat_file
        save(lqr_cfg.mat_filename, 'evtol_trim_db', '-v7.3');
    end

    fprintf('\nBatch LQR design complete.\n');
    fprintf('  Attempted: %d\n', evtol_trim_db.controller_summary.n_attempted);
    fprintf('  Success  : %d\n', evtol_trim_db.controller_summary.n_success);
    fprintf('  Failed   : %d\n', evtol_trim_db.controller_summary.n_failed);
end

function controller_schedule_table = local_build_controller_schedule_table(scenarios)
    template = struct( ...
        'index', 0, ...
        'name', '', ...
        'trim_success', false, ...
        'controller_success', false, ...
        'Vinf_mps', nan, ...
        'alpha_rad', nan, ...
        'beta_rad', nan, ...
        'phi_rad', nan, ...
        'theta_rad', nan, ...
        'psi_rad', nan, ...
        'front_tilt_deg', nan, ...
        'front_collective_rpm', nan, ...
        'rear_collective_rpm', nan, ...
        'K_lqr_cruise', [] );

    n = numel(scenarios);
    controller_schedule_table = repmat(template, n, 1);

    for i = 1:n
        scn = scenarios(i);

        controller_schedule_table(i).index = scn.index;
        controller_schedule_table(i).name = scn.name;
        controller_schedule_table(i).trim_success = isfield(scn, 'success') && scn.success;

        if isfield(scn, 'metadata') && isfield(scn.metadata, 'scheduling')
            s = scn.metadata.scheduling;
            controller_schedule_table(i).Vinf_mps = s.Vinf_mps;
            controller_schedule_table(i).alpha_rad = s.alpha_rad;
            controller_schedule_table(i).beta_rad = s.beta_rad;
            controller_schedule_table(i).phi_rad = s.phi_rad;
            controller_schedule_table(i).theta_rad = s.theta_rad;
            controller_schedule_table(i).psi_rad = s.psi_rad;
            controller_schedule_table(i).front_tilt_deg = s.front_tilt_deg;
            controller_schedule_table(i).front_collective_rpm = s.front_collective_rpm;
            controller_schedule_table(i).rear_collective_rpm = s.rear_collective_rpm;
        end

        if isfield(scn, 'controller') && isfield(scn.controller, 'success') && scn.controller.success
            controller_schedule_table(i).controller_success = true;
            controller_schedule_table(i).K_lqr_cruise = scn.controller.K_lqr_cruise;
        end
    end
end