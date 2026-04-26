% Plot_Transition_MainBridge_MiddleGap_Fast_Map.m
% Plot the focused middle-gap patch over the full transition map.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_middlegap_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_middlegap_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
