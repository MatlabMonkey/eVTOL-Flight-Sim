function options = build_mainbridge_rescue_fast_options(root_dir)
%BUILD_MAINBRIDGE_RESCUE_FAST_OPTIONS Narrow rescue search for 30-32.5 m/s.
% This pass is intentionally local. It re-establishes continuity at the
% left edge of the 30-40 m/s gap before we try to extend farther right.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_leftbridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv') };
options.anchor_max_vinf_mps = 35.0;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 30:2.5:32.5;
options.tilt_offsets_deg = [-5 -2.5 0 2.5 5];

options.front_seed_offsets_rpm = [-100 0 100];
options.rear_seed_offsets_rpm = [-150 0 150];
options.history_seed_count = 2;
options.neighbor_seed_count = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5];
options.reference_knot_tilt_deg = [0 30 45 55 62 67 68 70 72.5];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2235 2225 2210 2100 2000];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1070 780 620 950 850];
end
