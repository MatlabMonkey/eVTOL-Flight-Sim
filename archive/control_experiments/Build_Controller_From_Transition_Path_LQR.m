% Build_Controller_From_Transition_Path_LQR.m
% Build a scheduled LQR controller along the selected hover-to-cruise path.
%
% Usage:
%   Build_Controller_From_Transition_Path_LQR
%
% Optional workspace overrides:
%   transitionPathControllerOptions = struct( ...
%       'db_file', '<path to trim_linearization_db.mat>', ...
%       'controller_opts', struct(...));

dbFile = [];
controllerOpts = struct();
if exist('transitionPathControllerOptions', 'var') && isstruct(transitionPathControllerOptions)
    if isfield(transitionPathControllerOptions, 'db_file')
        dbFile = transitionPathControllerOptions.db_file;
    end
    if isfield(transitionPathControllerOptions, 'controller_opts') && ...
            isstruct(transitionPathControllerOptions.controller_opts)
        controllerOpts = transitionPathControllerOptions.controller_opts;
    end
end

controllerData = build_transition_path_lqr_controller(dbFile, controllerOpts);
K_lqr_cruise = controllerData.K_lqr_cruise;
x_trim_lqr = controllerData.x_trim_lqr;
U_trim_lqr = controllerData.U_trim_lqr;
