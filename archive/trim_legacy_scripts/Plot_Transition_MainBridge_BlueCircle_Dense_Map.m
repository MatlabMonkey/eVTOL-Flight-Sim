% Plot_Transition_MainBridge_BlueCircle_Dense_Map.m
% Plot the denser 30-40 m/s blue-circle basin search.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_mainbridge_bluecircle_dense_latest.csv');
transitionMidbandGuideGridPlotPlanOptions = build_mainbridge_bluecircle_dense_options(root_dir);
Plot_Transition_Midband_GuideGrid_Scored_Map
