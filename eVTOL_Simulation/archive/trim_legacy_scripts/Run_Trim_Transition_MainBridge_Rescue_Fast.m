% Run_Trim_Transition_MainBridge_Rescue_Fast.m
% Narrow rescue run for the 30-32.5 m/s left edge of the main bridge gap.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_rescue_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_rescue_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_rescue_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
