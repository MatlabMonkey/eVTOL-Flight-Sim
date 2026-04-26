% Run_Trim_Transition_MainBridge_MiddleGap_Fast.m
% Focused middle-gap bridge patch between 40/75 and the rear-on 45-47.5 branch.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_middlegap_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_middlegap_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_middlegap_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
