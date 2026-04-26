% Plot_Transition_MainBridge_V45_BasinScan_Fast_Map.m
% Plot the V=45 basin scan over the full transition map.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_v45_basinscan_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_v45_basinscan_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
