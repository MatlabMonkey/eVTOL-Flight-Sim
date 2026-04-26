% Plot_Transition_MainBridge_MidBasin_Fast_Map.m
% Plot the 30-37.5 m/s mid-basin patch run.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_midbasin_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_midbasin_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
