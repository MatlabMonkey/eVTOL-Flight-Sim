% Run_Trim_Transition_MainBridge_HighBridge_Fast.m
% Focused bridge run from the new 40/75 exact toward the 47.5 m/s exacts.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_highbridge_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_highbridge_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_highbridge_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
