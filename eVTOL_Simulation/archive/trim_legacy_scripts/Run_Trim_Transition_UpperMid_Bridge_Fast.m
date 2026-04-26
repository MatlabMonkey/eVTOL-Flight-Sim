% Run_Trim_Transition_UpperMid_Bridge_Fast.m
% Focused fast bridge run for the 25-35 m/s region, using the latest
% lower-mid fast results as anchors and a flatter/steeper guide trend.

root_dir = fileparts(mfilename('fullpath'));

transitionMidbandGuideGridOptions = build_uppermid_bridge_fast_options(root_dir);
transitionMidbandGuideGridOptions.output_prefix = 'transition_trim_uppermid_bridge_fast';
transitionMidbandGuideGridOptions.latest_prefix = 'transition_trim_uppermid_bridge_fast';

Run_Trim_Transition_Midband_GuideGrid_Scored
