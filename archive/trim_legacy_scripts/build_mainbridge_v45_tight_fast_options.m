function options = build_mainbridge_v45_tight_fast_options(root_dir)
%BUILD_MAINBRIDGE_V45_TIGHT_FAST_OPTIONS
% Tight follow-up at V = 45 m/s to try converting the existing borderline
% 72.5-80 deg points into exact trims within the already-working basin.

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
options.anchor_max_vinf_mps = 47.5;
options.anchor_max_tilt_error_deg = 7.5;

options.vinf_grid_mps = 45;
options.tilt_offsets_deg = [0 2.5 5.0 7.5];

options.front_seed_offsets_rpm = [-75 0 75];
options.rear_seed_offsets_rpm = [-100 0 100];
options.history_seed_count = 4;
options.neighbor_seed_count = 4;
options.anchor_seed_front_window_rpm = 200;
options.anchor_seed_rear_window_rpm = 200;
options.failed_seed_filter_enabled = true;
options.failed_seed_vinf_radius_mps = 1.0;
options.failed_seed_tilt_radius_deg = 1.0;
options.failed_seed_front_radius_rpm = 60.0;
options.failed_seed_rear_radius_rpm = 80.0;
options.failed_seed_density_threshold = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 10 20 30 35 40 42.5 45 47.5];
options.reference_knot_tilt_deg = [0 45 62 67.5 65 67.5 72.5 75 77.5];

options.front_guide_knot_vinf_mps = [0 10 20 30 35 40 42.5 45 47.5];
options.front_guide_knot_rpm = [1865.76 2104 2235 2050 1900 1665 1641 1100 950];

options.rear_guide_knot_vinf_mps = [0 10 20 30 35 40 42.5 45 47.5];
options.rear_guide_knot_rpm = [1756.38 1517 1070 700 700 1485 1472 1050 975];
