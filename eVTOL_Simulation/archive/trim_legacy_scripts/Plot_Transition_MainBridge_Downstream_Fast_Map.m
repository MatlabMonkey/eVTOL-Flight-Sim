% Plot_Transition_MainBridge_Downstream_Fast_Map.m
% Plot the downstream 35-42.5 m/s bridge patch run.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_downstream_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_downstream_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
