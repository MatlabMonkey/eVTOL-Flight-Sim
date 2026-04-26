% Run_Trim_Transition_MainBridge_BlueCircle_Dense.m
% Denser direct search of the 30-40 m/s blue-circle basin.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_bluecircle_dense_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_bluecircle_dense';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_bluecircle_dense';

Run_Trim_Transition_Midband_GuideGrid_Scored
