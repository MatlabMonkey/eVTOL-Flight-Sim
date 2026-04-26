% Plot_Transition_MainBridge_Fast_Map.m
% Plot the focused 30-40 m/s main bridge run while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_fast_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
