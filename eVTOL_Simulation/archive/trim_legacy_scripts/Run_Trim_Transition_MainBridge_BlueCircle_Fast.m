% Run_Trim_Transition_MainBridge_BlueCircle_Fast.m
% Direct search of the blue-circle basin in 30-40 m/s.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_bluecircle_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_bluecircle_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_bluecircle_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
