% Plot_Transition_MainBridge_HighBridge_Fast_Map.m
% Plot the high bridge 40-47.5 m/s patch on top of the full map.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_highbridge_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_highbridge_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
