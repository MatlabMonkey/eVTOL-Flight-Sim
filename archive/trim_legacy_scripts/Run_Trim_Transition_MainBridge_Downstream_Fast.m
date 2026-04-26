% Run_Trim_Transition_MainBridge_Downstream_Fast.m
% Focused downstream bridge run from the 35/65 foothold toward 45/75.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_downstream_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_downstream_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_downstream_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
