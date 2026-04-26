% Plot_Transition_LowMid_GuideGrid_Fast_Preview.m
% Preview the faster lower-mid guide-grid sweep before launching it.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridPreviewOptions = build_lowmid_fast_guide_options(root_dir);

Plot_Transition_Midband_GuideGrid_Preview
