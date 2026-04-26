% Plot_Transition_MainBridge_V42p5_Tilt70_Fast_Map.m
% Plot the tight V=42.5 / tilt~70 follow-up over the full map.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_v42p5_tilt70_fast_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_v42p5_tilt70_fast_options(root_dir);
transitionMidbandGuideGridPlotOptions = struct('show_popup', true, 'show_full_background', true);
Plot_Transition_Midband_GuideGrid_Scored_Map
