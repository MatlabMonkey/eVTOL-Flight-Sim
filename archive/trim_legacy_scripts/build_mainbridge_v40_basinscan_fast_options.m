function options = build_mainbridge_v40_basinscan_fast_options(root_dir)
%BUILD_MAINBRIDGE_V40_BASINSCAN_FAST_OPTIONS
% Single-airspeed basin scan at V=40 m/s, centered on the user's requested
% front-collective band (~1400-1600 rpm) and a broad low-to-mid rear band.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_downstream_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_midbasin_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_bluecircle_dense_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_midband_scored_latest.csv') };
options.anchor_max_vinf_mps = 42.5;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 40;
options.tilt_offsets_deg = [-7.5 -5.0 -2.5 0 2.5 5.0];

options.front_seed_offsets_rpm = [-100 0 100];
options.rear_seed_offsets_rpm = [-450 -300 -150 0 150 300 450];
options.history_seed_count = 4;
options.neighbor_seed_count = 4;
options.anchor_seed_front_window_rpm = 300;
options.anchor_seed_rear_window_rpm = 500;
options.failed_seed_filter_enabled = true;
options.failed_seed_vinf_radius_mps = 1.0;
options.failed_seed_tilt_radius_deg = 2.5;
options.failed_seed_front_radius_rpm = 75.0;
options.failed_seed_rear_radius_rpm = 100.0;
options.failed_seed_density_threshold = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45];
options.reference_knot_tilt_deg = [0 45 62 67.5 65 67.5 72.5 76.5 78];

options.front_guide_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45];
options.front_guide_knot_rpm = [1865.76 2104 2235 2050 1900 1889 1500 1500 1450];

options.rear_guide_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45];
options.rear_guide_knot_rpm = [1756.38 1517 1070 700 700 625 550 600 700];
