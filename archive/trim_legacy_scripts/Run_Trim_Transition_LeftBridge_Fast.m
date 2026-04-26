% Run_Trim_Transition_LeftBridge_Fast.m
% Focused fast bridge run for the 20-27.5 m/s region.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_leftbridge_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_leftbridge_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_leftbridge_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
