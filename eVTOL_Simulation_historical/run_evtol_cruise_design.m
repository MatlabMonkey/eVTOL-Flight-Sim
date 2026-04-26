% run_evtol_cruise_design.m
% Main runner for steady level-flight cruise trim, linearization, and LQR design.

SetupForTrimAndLin_EVTOL_Cruise;
TrimAndLinearize_EVTOL_Cruise;
Design_LQR_Controller_EVTOL_Cruise;
