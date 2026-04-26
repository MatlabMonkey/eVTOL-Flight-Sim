% Plot_Transition_LeftBridge_Fast_Map.m
% Plot the focused 20-27.5 m/s bridge run while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_leftbridge_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_leftbridge_fast_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
