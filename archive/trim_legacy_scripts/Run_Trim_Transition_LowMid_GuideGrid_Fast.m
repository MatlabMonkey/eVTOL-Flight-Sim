% Run_Trim_Transition_LowMid_GuideGrid_Fast.m
% Faster lower-mid guide-grid sweep that builds directly off the current
% lower-mid results and searches a narrower band with fewer seeds.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_lowmid_fast_guide_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_lowmid_guidegrid_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_lowmid_guidegrid_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
