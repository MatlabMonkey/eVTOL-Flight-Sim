% Plot_Transition_MainBridge_Handoff_Fast_Map.m
% Plot the narrow 27.5-32.5 m/s handoff run while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_handoff_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_handoff_fast_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
