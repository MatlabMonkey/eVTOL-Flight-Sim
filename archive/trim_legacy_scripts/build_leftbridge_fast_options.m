function options = build_leftbridge_fast_options(root_dir)
%BUILD_LEFTBRIDGE_FAST_OPTIONS Focused fast bridge search for 20-27.5 m/s.
% This is the next left-side iterative patch runner. It starts from the
% lower-mid good points and searches the 20-27.5 m/s band near the current
% successful tilt frontier with flatter front collective and falling rear
% collective seeds.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv') };
options.anchor_max_vinf_mps = 35.0;
options.anchor_max_tilt_error_deg = 10.0;

options.vinf_grid_mps = 20:2.5:27.5;
options.tilt_offsets_deg = [-5 -2.5 0 2.5 5];

options.front_seed_offsets_rpm = [-60 0 60];
options.rear_seed_offsets_rpm = [-100 0 100];
options.history_seed_count = 2;
options.neighbor_seed_count = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 5 10 15 20 22.5 25 27.5];
options.reference_knot_tilt_deg = [0 30 45 55 62 64 67 69];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 22.5 25 27.5];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2215 2210 2205 2195];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 22.5 25 27.5];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1100 950 780 560];
end
