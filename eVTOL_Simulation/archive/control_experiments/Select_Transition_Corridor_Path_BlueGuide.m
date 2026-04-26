% Select_Transition_Corridor_Path_BlueGuide.m
% Build one ordered transition corridor path from controller_schedule_db.mat
% using a hand-picked blue-guide corridor in (Vinf, front tilt) space.
%
% Outputs are left in the base workspace:
%   selectedTransitionPathSpec
%   selectedTransitionPathPoints
%   selectedTransitionPathTable
%   selectedTransitionPathDebug

opts = struct();
opts.guide_vinf_mps = [0 2.5 5 10 15 20 25 30 40 50 60 70];
opts.guide_tilt_deg = [0 10 22 45 55 62 68 72 78 83 87 90];
opts.allowed_classifications = ["exact_trim", "quasi_trim_usable"];
opts.require_rear_on = true;
opts.min_rear_collective_rpm = 1.0;
opts.waypoint_search_vinf_mps = 7.5;
opts.waypoint_search_tilt_deg = 7.5;
opts.max_candidates_per_waypoint = 12;
opts.monotonic_vinf = true;
opts.monotonic_tilt = true;
opts.plot_result = true;
opts.show_popup = true;
opts.save_plot = false;

[selectedTransitionPathSpec, selectedTransitionPathPoints, ...
    selectedTransitionPathTable, selectedTransitionPathDebug] = ...
    select_transition_corridor_path_from_controller_db([], opts);

disp(selectedTransitionPathTable(:, {'path_index', 'guide_vinf_mps', 'guide_tilt_deg', ...
    'selected_vinf_mps', 'selected_tilt_deg', 'name'}));
