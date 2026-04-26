% Plot_Transition_MainBridge_BlueCircle_Fast_Map.m
% Plot the explicit blue-circle basin search in 30-40 m/s.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_bluecircle_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_bluecircle_fast_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
