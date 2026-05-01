%% Best INDI full-transition run

repoRoot = fileparts(mfilename('fullpath'));
cd(repoRoot);

builderOpts = struct();

builderOpts.path = struct();
builderOpts.path.point_weight_surface = 3.0;
builderOpts.path.max_abs_surface_deg = 18.0;

builderOpts.outer_loop = struct();
builderOpts.outer_loop.kq = 2.20;
builderOpts.outer_loop.ktheta = 2.10;
builderOpts.outer_loop.ktheta_high_speed = 2.60;

builderOpts.allocation = struct();
builderOpts.allocation.virtual_error_weights = [0.30; 1.00; 2.0];
builderOpts.allocation.control_regularization = [1.2e-6; 1.2e-6; 10.0; 10.0];
builderOpts.allocation.delta_eta_limits = [1.5e6; 1.5e6; deg2rad(5.0); deg2rad(5.0)];
builderOpts.allocation.rotor_trim_feedforward_blend = 0.0;
builderOpts.allocation.surface_trim_feedforward_blend = 0.0;

builderOpts.runtime_g = struct();
builderOpts.runtime_g.enabled = true;
builderOpts.runtime_g.entry_name = 'stall_boundary_delta_alpha_dense';

runOutDir = fullfile(repoRoot, 'workspace_plots', ...
    'indi_full_hover_to_cruise_best_reg10_q2_300s');

result = Run_INDI_PathSegment_Test([], struct( ...
    'full_path', true, ...
    'stopTime_s', 300, ...
    'builder_opts', builderOpts, ...
    'output_dir', runOutDir));

if strcmp(result.runResult.status, 'simulated')
    plot_indi_transition_debug(out, ...
        fullfile(runOutDir, 'INDI_FullHoverToCruise_best_reg10_q2_300s'), ...
        'INDI Full Hover To Cruise');
end

plot_indi_transition_trim_path_map(result.controllerData, struct( ...
    'show_popup', true, ...
    'output_dir', fullfile(runOutDir, 'path_map'), ...
    'alpha_zlim_deg', [-5 12]));
