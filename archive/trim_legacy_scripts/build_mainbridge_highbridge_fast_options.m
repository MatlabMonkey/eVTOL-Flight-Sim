function options = build_mainbridge_highbridge_fast_options(root_dir)
%BUILD_MAINBRIDGE_HIGHBRIDGE_FAST_OPTIONS
% Narrow bridge patch from the new 40/75 exact toward the existing
% 47.5 m/s high-speed exacts in the 77.5-82.5 deg band.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_midband_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_rearon_forever_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_rearon_connector_forever_latest.csv') };
options.anchor_max_vinf_mps = 50.0;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 40:2.5:47.5;
options.tilt_offsets_deg = [-2.5 0 2.5 5.0];

options.front_seed_offsets_rpm = [-125 0 125];
options.rear_seed_offsets_rpm = [-150 0 150];
options.history_seed_count = 4;
options.neighbor_seed_count = 4;
options.anchor_seed_front_window_rpm = 350;
options.anchor_seed_rear_window_rpm = 350;
options.failed_seed_filter_enabled = true;
options.failed_seed_vinf_radius_mps = 2.5;
options.failed_seed_tilt_radius_deg = 5.0;
options.failed_seed_front_radius_rpm = 150.0;
options.failed_seed_rear_radius_rpm = 150.0;
options.failed_seed_density_threshold = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45 47.5 50];
options.reference_knot_tilt_deg = [0 45 62 67.5 65 67.5 75 76 77.5 80 82.5];

options.front_guide_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45 47.5 50];
options.front_guide_knot_rpm = [1865.76 2104 2235 2050 1900 1889 1961 1750 1500 1200 1000];

options.rear_guide_knot_vinf_mps = [0 10 20 30 35 37.5 40 42.5 45 47.5 50];
options.rear_guide_knot_rpm = [1756.38 1517 1070 700 700 625 314 500 700 950 950];
