% run_evtol_cruise_design.m
% Legacy top-level runner kept for familiarity.
%
% The preferred workflow is now:
%   1) Init_EVTOL_Main
%   2) Trim_EVTOL_Main
%   3) Build_Controller_EVTOL_Cruise
%
% This file keeps the old one-shot entrypoint alive by chaining those
% scripts in the new order.

Init_EVTOL_Main;
Trim_EVTOL_Main;
Build_Controller_EVTOL_Cruise;
