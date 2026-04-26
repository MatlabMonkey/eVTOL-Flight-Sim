function options = build_mainbridge_downstream_fast_options(root_dir)
%BUILD_MAINBRIDGE_DOWNSTREAM_FAST_OPTIONS Focused downstream patch from 35/65.
% This pass follows the strongest foothold currently in the canonical DB:
%   - 35 m/s, 65 deg exact trim
% and pushes diagonally toward the higher-speed successful region without
% reusing the failed 30/70 basin.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_midband_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_bluecircle_dense_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_midbasin_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_leftbridge_fast_latest.csv') };
options.anchor_max_vinf_mps = 45.0;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 35:2.5:42.5;
options.tilt_offsets_deg = [-2.5 0 2.5 5.0];

options.front_seed_offsets_rpm = [-100 0 100];
options.rear_seed_offsets_rpm = [-125 0 125];
options.history_seed_count = 3;
options.neighbor_seed_count = 3;
options.anchor_seed_front_window_rpm = 250;
options.anchor_seed_rear_window_rpm = 250;
options.failed_seed_filter_enabled = true;
options.failed_seed_vinf_radius_mps = 2.5;
options.failed_seed_tilt_radius_deg = 5.0;
options.failed_seed_front_radius_rpm = 125.0;
options.failed_seed_rear_radius_rpm = 125.0;
options.failed_seed_density_threshold = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 10 20 25 30 32.5 35 37.5 40 42.5 45];
options.reference_knot_tilt_deg = [0 45 62 67 67.5 65 65 67.5 70 72.5 75];

options.front_guide_knot_vinf_mps = [0 10 20 25 30 32.5 35 37.5 40 42.5 45];
options.front_guide_knot_rpm = [1865.76 2104 2235 2225 2050 2000 1900 1800 1650 1500 1350];

options.rear_guide_knot_vinf_mps = [0 10 20 25 30 32.5 35 37.5 40 42.5 45];
options.rear_guide_knot_rpm = [1756.38 1517 1070 780 700 650 700 625 575 500 450];
