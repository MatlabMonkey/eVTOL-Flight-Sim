% Run_Trim_Transition_MainBridge_V42p5_Tilt70_Fast.m
% Tight basin follow-up at V=42.5 m/s around 70 deg tilt.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_v42p5_tilt70_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_v42p5_tilt70_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_v42p5_tilt70_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
