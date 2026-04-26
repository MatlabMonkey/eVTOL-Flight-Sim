% Plot_Transition_LeftBridge_Fast_Preview.m
% Preview the focused 20-27.5 m/s bridge search before launching it.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridPreviewOptions = build_leftbridge_fast_options(root_dir);

Plot_Transition_Midband_GuideGrid_Preview
