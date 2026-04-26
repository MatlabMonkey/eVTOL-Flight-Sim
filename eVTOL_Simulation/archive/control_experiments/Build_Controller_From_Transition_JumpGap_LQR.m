% Build_Controller_From_Transition_JumpGap_LQR.m
% Build the scheduled LQR controller along the hand-selected jump-gap path.

controllerOpts = struct();
if exist('transitionJumpGapControllerOptions', 'var') && isstruct(transitionJumpGapControllerOptions)
    controllerOpts = transitionJumpGapControllerOptions;
end

controllerData = build_transition_jumpgap_lqr_controller(controllerOpts);
K_lqr_cruise = controllerData.K_lqr_cruise;
x_trim_lqr = controllerData.x_trim_lqr;
U_trim_lqr = controllerData.U_trim_lqr;
