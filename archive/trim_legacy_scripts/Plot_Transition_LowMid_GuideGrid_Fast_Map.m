% Plot_Transition_LowMid_GuideGrid_Fast_Map.m
% Plot the faster lower-mid guide-grid sweep while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_lowmid_guidegrid_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_lowmid_fast_guide_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
