% Run_Trim_Transition_MainBridge_V45_BasinScan_Fast.m
% Focused single-airspeed basin scan at V=45 m/s.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_v45_basinscan_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_v45_basinscan_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_v45_basinscan_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
