% Run_Trim_Transition_MainBridge_Handoff_Fast.m
% Narrow 27.5-32.5 m/s handoff run built from the working 25-30 m/s basin.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_handoff_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_handoff_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_handoff_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
