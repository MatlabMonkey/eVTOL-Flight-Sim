% Plot_Transition_MainBridge_V45_Tight_Fast_Map.m
% Plot the tight V=45 follow-up over the full transition map.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_v45_tight_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_v45_tight_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
