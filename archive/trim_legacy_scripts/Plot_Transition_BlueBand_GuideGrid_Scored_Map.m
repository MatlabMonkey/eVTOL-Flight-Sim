% Plot_Transition_BlueBand_GuideGrid_Scored_Map.m
% Plot the focused blue-band guide-grid sweep while it is running.

root_dir = fileparts(mfilename('fullpath'));
transitionMidbandGuideGridPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_blueband_guidegrid_scored_latest.csv');
Plot_Transition_Midband_GuideGrid_Scored_Map
