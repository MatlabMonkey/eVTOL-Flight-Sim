% Plot_Transition_RearOn_Connector_Forever_Map.m
% Convenience wrapper for plotting the rear-on connector forever run.

stack = dbstack('-completenames');
if ~isempty(stack)
    root_dir = fileparts(stack(1).file);
else
    root_dir = pwd;
end

transitionReferenceLineScoredPlotInputCsv = fullfile(root_dir, 'workspace_plots', ...
    'transition_trim_rearon_connector_forever_latest.csv'); %#ok<NASGU>
Plot_Transition_Reference_Line_Scored_Map
