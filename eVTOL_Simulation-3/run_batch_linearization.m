% run_batch_linearization.m
% You make the cfg struct here and then send it to the batch runner
% function
% Example:
%   batch_cfg = struct();
%   batch_cfg.sweep.Vinf_mps = 60:5:80;
%   batch_cfg.sweep.front_tilt_deg = [85 90];
%   evtol_trim_db = BuildBatchTrimLinearize_EVTOL_Cruise(batch_cfg);

%% Build the batch_cfg
batch_cfg = struct();
batch_cfg.sweep.Vinf_mps = 30:5:100;
batch_cfg.sweep.front_tilt_deg = 75:5:90;

%% Run the batch linearization
%evtol_trim_db = BuildBatchTrimLinearize_EVTOL_Cruise(batch_cfg);

%% Output a summary
evtol_trim_db.summary

%% Run the LQR runner on the struct we just made
evtol_trim_db = AddLQRToBatchTrimDB_EVTOL_Cruise('evtol_trim_db.mat');