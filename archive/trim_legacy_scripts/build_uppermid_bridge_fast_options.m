function options = build_uppermid_bridge_fast_options(root_dir)
%BUILD_UPPERMID_BRIDGE_FAST_OPTIONS Focused fast bridge search for 25-35 m/s.
% Uses the current lower-mid fast results as the main anchor bank, but seeds
% a new guide family that stays flatter in front collective and drops the
% rear collective more aggressively through the 25-35 m/s gap.

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
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv') };
options.anchor_max_vinf_mps = 40.0;
options.anchor_max_tilt_error_deg = 10.0;

% Focus only on the upper end of the lower-mid gap.
options.vinf_grid_mps = 25:2.5:35;
options.tilt_offsets_deg = [-5 -2.5 0 2.5 5];

% Small guide-grid search around a flatter front-collective path and a
% steeper rear-collective drop, following the trend visible in the found
% points rather than the older hand-drawn guide.
options.front_seed_offsets_rpm = [-60 0 60];
options.rear_seed_offsets_rpm = [-120 0 120];
options.history_seed_count = 2;
options.neighbor_seed_count = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35];
options.reference_knot_tilt_deg = [0 30 45 55 62 70 68 70 73 76];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2235 2225 2210 2200 2190 2165];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1070 580 500 400 250 125];
end
