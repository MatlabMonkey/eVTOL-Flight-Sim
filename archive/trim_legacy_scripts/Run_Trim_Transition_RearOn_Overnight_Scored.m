% Run_Trim_Transition_RearOn_Overnight_Scored.m
% Rear-on overnight transition sweep.
%
% This wrapper drives the scored reference-line search, but it excludes the
% zero-rear cruise basin and biases all seeds toward the hover-side prop
% split and the known-good rear-on cruise anchor.
%
% Goals:
%   - keep rear collective on throughout the transition search
%   - search densely around the hover-to-cruise reference corridor
%   - spend extra effort in the 20-50 m/s midband gap
%   - use smoother prop interpolation instead of large jumps

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

transitionPathScoredOptions = struct();
transitionPathScoredOptions.output_prefix = 'transition_trim_rearon_overnight_scored';
transitionPathScoredOptions.latest_prefix = 'transition_trim_rearon_overnight_scored';

% Let this run for a long time without needing babysitting.
transitionPathScoredOptions.max_attempts = 5000;
transitionPathScoredOptions.checkpoint_every = 1;

% Search a dense band around the intended transition corridor, with extra
% density through the troublesome midband.
transitionPathScoredOptions.min_vinf_mps = 0.0;
transitionPathScoredOptions.max_vinf_mps = 75.0;
transitionPathScoredOptions.min_tilt_deg = 0.0;
transitionPathScoredOptions.max_tilt_deg = 90.0;
transitionPathScoredOptions.reference_vinf_samples = ...
    unique([0:2.5:20, 20:1.25:50, 50:2.5:75]);
transitionPathScoredOptions.reference_knot_vinf_mps = ...
    [0 2.5 5 10 15 20 25 30 35 40 45 50 60 70 75];
transitionPathScoredOptions.reference_knot_tilt_deg = ...
    [0 5 10 20 30 40 50 60 67 73 78 82 86 90 90];
transitionPathScoredOptions.reference_tilt_offsets_deg = ...
    [-10 -7.5 -5 -2.5 0 2.5 5 7.5 10];
transitionPathScoredOptions.reference_grid_step_deg = 2.5;

% Use fewer but more relevant seed families for overnight speed.
transitionPathScoredOptions.enabled_families = { ...
    'front_rear_free_flap_elevator', ...
    'rear_fixed_flap_elevator', ...
    'hover_zero_surface'};
transitionPathScoredOptions.neighbor_seed_count = 2;
transitionPathScoredOptions.history_exact_seed_count = 2;
transitionPathScoredOptions.history_scored_seed_count = 2;
transitionPathScoredOptions.max_prop_nudge_fraction = 0.08;

% Force rear-on behavior everywhere in the search.
transitionPathScoredOptions.front_collective_min_rpm = 0.0;
transitionPathScoredOptions.rear_collective_min_rpm = 100.0;
transitionPathScoredOptions.history_min_rear_collective_rpm = 100.0;
transitionPathScoredOptions.hover_anchor_case_name = 'TrimCase_Hover';
transitionPathScoredOptions.cruise_anchor_case_name = 'TrimCase_Cruise75_FlapElevator_Rear500';

% Reuse the broad exact history, but take scored guidance from the current
% reference-line sweep instead of the older low-speed-only runner.
transitionPathScoredOptions.history_exact_csv = ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv');
transitionPathScoredOptions.history_scored_csv = ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv');

% Extra interpolation help in the bad 20-50 m/s band.
transitionPathScoredOptions.midband_vinf_range_mps = [20.0 50.0];
transitionPathScoredOptions.midband_interp_tilt_window_deg = 22.5;
transitionPathScoredOptions.midband_interp_fracs = [0.15 0.25 0.35 0.5 0.65 0.75 0.85];

Run_Trim_Transition_Reference_Line_Scored
