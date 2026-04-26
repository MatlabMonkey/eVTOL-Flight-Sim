% Build_Controller_From_TrimPoint_LQR.m
% Preferred trim-point LQR entrypoint.
%
% Usage:
%   Init_EVTOL_Main
%   trimCase = <any trim case that produces an exact trim>
%   Trim_EVTOL_Main
%   Build_Controller_From_TrimPoint_LQR
%
% Outputs in base workspace:
%   - controllerData
%   - K_lqr_cruise (legacy compatibility view)
%   - x_trim_lqr (legacy compatibility view)
%   - U_trim_lqr (legacy compatibility view)

controllerData = build_trim_point_lqr_controller(trimResult);
K_lqr_cruise = controllerData.K_lqr_cruise;
x_trim_lqr = controllerData.x_trim_lqr;
U_trim_lqr = controllerData.U_trim_lqr;
