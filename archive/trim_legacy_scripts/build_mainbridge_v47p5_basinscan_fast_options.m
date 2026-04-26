function options = build_mainbridge_v47p5_basinscan_fast_options(root_dir)
%BUILD_MAINBRIDGE_V47P5_BASINSCAN_FAST_OPTIONS
% Single-airspeed basin scan at V=47.5 m/s, targeted at the lower portion
% of the already-working rear-on basin so the 45 m/s exact can hand off
% cleanly into the existing 47.5 m/s exacts.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_v45_basinscan_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_v42p5_basinscan_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_highbridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_rearon_forever_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_rearon_connector_forever_latest.csv') };
options.anchor_max_vinf_mps = 50.0;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 47.5;
options.tilt_offsets_deg = [-7.5 -5.0 -2.5 0 2.5 5.0];

options.front_seed_offsets_rpm = [-125 0 125];
options.rear_seed_offsets_rpm = [-200 -100 0 100 200];
options.history_seed_count = 4;
options.neighbor_seed_count = 4;
options.anchor_seed_front_window_rpm = 300;
options.anchor_seed_rear_window_rpm = 350;
options.failed_seed_filter_enabled = true;
options.failed_seed_vinf_radius_mps = 1.0;
options.failed_seed_tilt_radius_deg = 2.5;
options.failed_seed_front_radius_rpm = 100.0;
options.failed_seed_rear_radius_rpm = 125.0;
options.failed_seed_density_threshold = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45 47.5 50];
options.reference_knot_tilt_deg = [0 45 62 67.5 65 67.5 67.5 72.5 70 75 77.5];

options.front_guide_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45 47.5 50];
options.front_guide_knot_rpm = [1865.76 2104 2235 2050 1900 1889 1665 1641 1261 1100 950];

options.rear_guide_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45 47.5 50];
options.rear_guide_knot_rpm = [1756.38 1517 1070 700 700 625 1485 1472 1197 1050 950];
