%% Updated External-Forces Tester
% This live-script source wraps the shared tester runner so the validation
% cases always use the current aircraft and scenario definitions.
%
% It is the maintainable source for TESTER_external_forces_updated.mlx.

clear; clc;

script_file = localGetThisFile();
if ~isempty(script_file)
    addpath(fileparts(script_file));
end

%% Options
% Leave requestedCases empty to run all six cases.
requestedCases = {};

% Set these flags before running the script.
showRender = true;
makePlots = true;
replayFlightGear = false;
replayCase = 'Case 5 - Cruise Beta to Yaw';

%% Preview Test Matrix
% The runner prints this summary too, but it is helpful to keep it visible in
% the live script before the simulation output starts.
previewOutputs = TESTER_external_forces_runner( ...
    'Cases', requestedCases, ...
    'ShowRender', false, ...
    'MakePlots', false, ...
    'PreviewOnly', true);
caseSummary = previewOutputs.case_summary

%% Run Validation Cases
outputs = TESTER_external_forces_runner( ...
    'Cases', requestedCases, ...
    'ShowRender', showRender, ...
    'MakePlots', makePlots, ...
    'ReplayFlightGear', replayFlightGear, ...
    'ReplayCase', replayCase);

%% Output Summary
disp('Completed tester cases:');
disp(setdiff(fieldnames(outputs), {'case_summary'}));

%% Notes
% This script intentionally delegates the actual setup/sim logic to
% TESTER_external_forces_runner.m so the aircraft definition only lives in
% one place.

function script_file = localGetThisFile()
script_file = mfilename('fullpath');
if ~isempty(script_file)
    return;
end

script_file = '';
try
    script_file = matlab.desktop.editor.getActiveFilename;
catch
    % Fall back to the current folder if the editor API is unavailable.
end
end
