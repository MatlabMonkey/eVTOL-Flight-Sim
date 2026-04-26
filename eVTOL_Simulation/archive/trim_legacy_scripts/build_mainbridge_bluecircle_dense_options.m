function options = build_mainbridge_bluecircle_dense_options(root_dir)
%BUILD_MAINBRIDGE_BLUECIRCLE_DENSE_OPTIONS Denser search of the blue-circle basin.
% This is a deliberately denser version of the blue-circle run. It searches
% more front/rear combinations inside the front-low / rear-high basin and
% widens the local tilt band slightly.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_bluecircle_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_handoff_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_rescue_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_leftbridge_fast_latest.csv') };
options.anchor_max_vinf_mps = 42.5;
options.anchor_max_tilt_error_deg = 15.0;

options.vinf_grid_mps = 30:2.5:40;
options.tilt_offsets_deg = [-7.5 -5 -2.5 0 2.5 5 7.5];

options.front_seed_offsets_rpm = [-150 -75 0 75 150];
options.rear_seed_offsets_rpm = [-175 -90 0 90 175];
options.history_seed_count = 3;
options.neighbor_seed_count = 3;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5 40];
options.reference_knot_tilt_deg = [0 30 45 55 62 67 68 70 72.5 75 78 80];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5 40];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2235 2225 2200 1850 1750 1600 1450 1300];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5 40];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1070 780 620 1125 1025 925 850 775];
end
