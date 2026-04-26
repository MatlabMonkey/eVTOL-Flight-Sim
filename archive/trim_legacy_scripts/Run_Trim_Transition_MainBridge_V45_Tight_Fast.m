% Run_Trim_Transition_MainBridge_V45_Tight_Fast.m
% Tight follow-up at V=45 m/s around the existing borderline basin.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_mainbridge_v45_tight_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_mainbridge_v45_tight_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_mainbridge_v45_tight_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
