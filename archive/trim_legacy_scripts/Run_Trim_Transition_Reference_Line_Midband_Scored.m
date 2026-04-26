% Run_Trim_Transition_Reference_Line_Midband_Scored.m
% Focused reference-line scored transition sweep for the red-circled
% mid-band gap. This searches only the 20-50 m/s region around a tighter
% reference segment and uses the current reference-line run as scored
% history so prop guesses stay closer to the already-found line trend.

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

transitionPathScoredOptions = struct();
transitionPathScoredOptions.output_prefix = 'transition_trim_reference_line_midband_scored';
transitionPathScoredOptions.latest_prefix = 'transition_trim_reference_line_midband_scored';
transitionPathScoredOptions.max_attempts = 200;
transitionPathScoredOptions.checkpoint_every = 2;

% Only search the problematic middle band.
transitionPathScoredOptions.min_vinf_mps = 20.0;
transitionPathScoredOptions.max_vinf_mps = 50.0;
transitionPathScoredOptions.min_tilt_deg = 55.0;
transitionPathScoredOptions.max_tilt_deg = 90.0;
transitionPathScoredOptions.reference_vinf_samples = 20:2.5:50;
transitionPathScoredOptions.reference_knot_vinf_mps = [20 25 30 35 40 45 50];
transitionPathScoredOptions.reference_knot_tilt_deg = [60 65 70 74 78 82 85];
transitionPathScoredOptions.reference_tilt_offsets_deg = [-10 -5 0 5 10];
transitionPathScoredOptions.reference_grid_step_deg = 5.0;

% Bias toward smoother prop interpolation across the current line trend.
transitionPathScoredOptions.neighbor_seed_count = 2;
transitionPathScoredOptions.history_exact_seed_count = 2;
transitionPathScoredOptions.history_scored_seed_count = 3;
transitionPathScoredOptions.midband_interp_fracs = [0.2 0.35 0.5 0.65 0.8];
transitionPathScoredOptions.midband_interp_tilt_window_deg = 18.0;
transitionPathScoredOptions.max_prop_nudge_fraction = 0.10;

% Use the current reference-line run as the scored seed bank so we
% interpolate around the line we are already finding, not the older
% low-speed-only sweep.
transitionPathScoredOptions.history_scored_csv = ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv');
transitionPathScoredOptions.history_exact_csv = ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_merged_latest.csv');

Run_Trim_Transition_Reference_Line_Scored
