% Run_Trim_Transition_MainBridge_Fast.m
% Focused fast main bridge run for the 30-40 m/s region.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
