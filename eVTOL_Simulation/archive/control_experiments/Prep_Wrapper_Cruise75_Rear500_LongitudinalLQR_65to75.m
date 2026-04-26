% Prep_Wrapper_Cruise75_Rear500_LongitudinalLQR_65to75.m
% Stage the Wrapper workspace for the known-good rear-500 cruise recovery.
%
% Usage:
%   Prep_Wrapper_Cruise75_Rear500_LongitudinalLQR_65to75
%
% This script does not run the model. It initializes the workflow
% workspace, loads a known-good cruise trim plus longitudinal LQR from a
% cache file, prepares the Wrapper inputs, and opens Wrapper so the user
% can press Run manually.

Init_EVTOL_Main

root_dir = fileparts(mfilename('fullpath'));
cache_file = fullfile(root_dir, '.workflow_cache', ...
    'Cruise75_FlapElevator_Rear500_LongitudinalLQR.mat');

if exist(cache_file, 'file') ~= 2
    error(['Missing cache file: %s\n', ...
           'Rebuild the rear-500 cruise trim/controller cache before using this prep script.'], ...
        cache_file);
end

cache_data = load(cache_file, 'trimResult', 'controllerData');
trimResult = cache_data.trimResult;
controllerData = cache_data.controllerData;

runCase = RunCase_CruiseRecover65to75();
runCase.attemptSimulation = false;
Run_EVTOL_Main

open_system('Wrapper')

fprintf(['Wrapper is prepared for the rear-500 cruise recovery test.\n', ...
         'Initial speed = 65 m/s, commanded cruise = 75 m/s.\n', ...
         'Press Run in Wrapper, then use:\n', ...
         '  plot_outputs(''important'')\n']);
