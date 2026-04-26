function options = build_mainbridge_bluecircle_fast_options(root_dir)
%BUILD_MAINBRIDGE_BLUECIRCLE_FAST_OPTIONS Search the explicit blue-circle basin.
% This run targets the front-low / rear-high RPM basin visible in the prop
% plots around 30-40 m/s. It does not try to follow the old guide curve.

if nargin < 1 || isempty(root_dir)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        root_dir = fileparts(stack(1).file);
    else
        root_dir = pwd;
    end
end

options = struct();
options.anchor_history_csvs = { ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_handoff_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_mainbridge_rescue_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_uppermid_bridge_fast_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_leftbridge_fast_latest.csv') };
options.anchor_max_vinf_mps = 42.5;
options.anchor_max_tilt_error_deg = 12.5;

options.vinf_grid_mps = 30:2.5:40;
options.tilt_offsets_deg = [-5 -2.5 0 2.5 5];

options.front_seed_offsets_rpm = [-100 0 100];
options.rear_seed_offsets_rpm = [-125 0 125];
options.history_seed_count = 2;
options.neighbor_seed_count = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

options.reference_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5 40];
options.reference_knot_tilt_deg = [0 30 45 55 62 67 68 70 72.5 75 78 80];

options.front_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5 40];
options.front_guide_knot_rpm = [1865.76 1988 2104 2180 2235 2225 2200 1800 1675 1550 1425 1300];

options.rear_guide_knot_vinf_mps = [0 5 10 15 20 25 27.5 30 32.5 35 37.5 40];
options.rear_guide_knot_rpm = [1756.38 1665 1517 1380 1070 780 620 1100 1025 950 875 800];
end
