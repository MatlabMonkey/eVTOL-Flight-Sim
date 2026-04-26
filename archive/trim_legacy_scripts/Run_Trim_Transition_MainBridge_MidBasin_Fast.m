% Run_Trim_Transition_MainBridge_MidBasin_Fast.m
% Target the 30-37.5 m/s mid-basin around the user-selected blue-circle region.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_midbasin_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_midbasin_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_midbasin_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
