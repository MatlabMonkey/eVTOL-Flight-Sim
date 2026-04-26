% Plot_Transition_UpperMid_Bridge_Fast_Map.m
% Plot the focused 25-35 m/s bridge run while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_uppermid_bridge_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_uppermid_bridge_fast_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
