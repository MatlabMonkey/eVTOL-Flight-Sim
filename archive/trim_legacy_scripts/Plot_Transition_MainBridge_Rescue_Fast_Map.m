% Plot_Transition_MainBridge_Rescue_Fast_Map.m
% Plot the narrow 30-32.5 m/s rescue run while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_rescue_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_rescue_fast_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
