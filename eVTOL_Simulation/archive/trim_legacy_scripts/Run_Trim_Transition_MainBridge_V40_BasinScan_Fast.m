% Run_Trim_Transition_MainBridge_V40_BasinScan_Fast.m
% Focused single-airspeed basin scan at V=40 m/s.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_v40_basinscan_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_v40_basinscan_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_v40_basinscan_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
