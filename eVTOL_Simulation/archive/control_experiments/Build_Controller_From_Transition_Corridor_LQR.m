% Build_Controller_From_Transition_Corridor_LQR.m
% Build a scheduled LQR controller from the selected controller-DB corridor.
%
% Optional workspace overrides:
%   transitionCorridorControllerOptions = struct( ...
%       'controller_opts', struct(...));

controllerOpts = struct();
if exist('transitionCorridorControllerOptions', 'var') && isstruct(transitionCorridorControllerOptions)
    if isfield(transitionCorridorControllerOptions, 'controller_opts') && ...
            isstruct(transitionCorridorControllerOptions.controller_opts)
        controllerOpts = transitionCorridorControllerOptions.controller_opts;
    end
end

controllerData = build_transition_corridor_lqr_controller(controllerOpts);
K_lqr_cruise = controllerData.K_lqr_cruise;
x_trim_lqr = controllerData.x_trim_lqr;
U_trim_lqr = controllerData.U_trim_lqr;
