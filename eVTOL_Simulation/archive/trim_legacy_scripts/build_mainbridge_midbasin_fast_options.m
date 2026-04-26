function options = build_mainbridge_midbasin_fast_options(root_dir)
%BUILD_MAINBRIDGE_MIDBASIN_FAST_OPTIONS Local patch for the 30-37.5 m/s mid basin.
% This pass targets the front-mid / rear-mid basin around the user's
% circled region near 35 m/s. It is tighter than the broad 30-40 runs and
% uses the most recent blue-circle dense results as the first anchor bank.

if nargin < 1 || isempty(root_dir)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        root_dir = fileparts(stack(1).file);
    else
        root_dir = pwd;
    end
end

options = struct();
options.anchor_history_csvs = { ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_bluecircle_dense_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_bluecircle_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_handoff_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_rescue_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_leftbridge_fast_latest.csv') };
options.anchor_max_vinf_mps = 40.0;
options.anchor_max_tilt_error_deg = 15.0;

options.vinf_grid_mps = 30:2.5:37.5;
options.tilt_offsets_deg = [-7.5 -5 -2.5 0 2.5 5 7.5];

options.front_seed_offsets_rpm = [-150 -75 0 75];
options.rear_seed_offsets_rpm = [-150 -75 0 75];
options.history_seed_count = 3;
options.neighbor_seed_count = 3;
options.anchor_seed_front_window_rpm = 250;
options.anchor_seed_rear_window_rpm = 300;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;
options.failed_seed_filter_enabled = true;
options.failed_seed_vinf_radius_mps = 2.5;
options.failed_seed_tilt_radius_deg = 5.0;
options.failed_seed_front_radius_rpm = 125.0;
options.failed_seed_rear_radius_rpm = 125.0;
options.failed_seed_density_threshold = 2;

options.reference_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5];
options.reference_knot_tilt_deg = [0 30 45 55 62 67 68 70 72.5 75 77.5];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2235 2225 2000 1700 1600 1500 1400];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1070 780 700 1050 975 875 775];
