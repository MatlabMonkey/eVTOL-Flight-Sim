function options = build_mainbridge_handoff_fast_options(root_dir)
%BUILD_MAINBRIDGE_HANDOFF_FAST_OPTIONS Narrow handoff search for 27.5-32.5 m/s.
% This pass is intentionally narrow. It starts from the only region with
% actual continuity (25 -> 27.5 -> 30 m/s) and searches a small basin
% around the observed front/rear RPM trend instead of following the broader
% 30-40 m/s guide curve.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_leftbridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv') };
options.anchor_max_vinf_mps = 35.0;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 27.5:2.5:32.5;
options.tilt_offsets_deg = [-2.5 0 2.5 5];

options.front_seed_offsets_rpm = [-60 0 60];
options.rear_seed_offsets_rpm = [-120 0 120];
options.history_seed_count = 2;
options.neighbor_seed_count = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5];
options.reference_knot_tilt_deg = [0 30 45 55 62 67 67.5 68 70];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2235 2250 2155 2105 2050];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1070 823 540 400 350];
end
