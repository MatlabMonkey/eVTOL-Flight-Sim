% TrimSearch_Run.m
% User-facing entrypoint for transition trim searches.
%
% Preferred usage:
%   Init_Main
%   transitionTrimSearchOptions = struct('target_limit', 20);
%   TrimSearch_Run
%   TrimDB_Build
%
% This script owns the clean workflow:
%   search -> databases/trim_attempts
%          -> databases/transition_trim_linearizations
%
% The current engine is the guide-grid scored search. Search profiles select
% target generators, guide curves, seed families, and optional physics seeds
% without reintroducing one-off campaign scripts.

if ~exist('transitionTrimSearchOptions', 'var') || ~isstruct(transitionTrimSearchOptions)
    transitionTrimSearchOptions = struct();
end

searchOptions = localApplyDefaults(transitionTrimSearchOptions);
transitionMidbandGuideGridOptions = localBuildGuideGridOptions(searchOptions);

fprintf('=== TrimSearch_Run ===\n');
fprintf('Profile: %s\n', searchOptions.profile);
fprintf('Target strategy: %s\n', searchOptions.target_strategy);
fprintf('Targets: V=[%.1f..%.1f] m/s, %d V-points, %d tilt offsets\n', ...
    min(searchOptions.vinf_grid_mps), max(searchOptions.vinf_grid_mps), ...
    numel(searchOptions.vinf_grid_mps), numel(searchOptions.tilt_offsets_deg));
fprintf('Writes: master DB + canonical linearization artifacts\n');
fprintf('Debug run outputs: %d\n\n', logical(searchOptions.write_debug_run_outputs));

TrimSearch_Engine

if exist('transitionTrimMidbandGuideGridSummary', 'var')
    transitionTrimSearchSummary = transitionTrimMidbandGuideGridSummary;
end
if exist('transitionTrimMidbandGuideGridMap', 'var')
    transitionTrimSearchMap = transitionTrimMidbandGuideGridMap;
end
if exist('transitionMidbandGuideGridOptions', 'var')
    transitionTrimSearchOptionsUsed = transitionMidbandGuideGridOptions;
end

function opts = localApplyDefaults(opts)
requestedOptions = opts;
profile = string(localGetField(requestedOptions, 'profile', "guide_grid"));
opts = localOverlayStruct(localProfileDefaults(profile), requestedOptions);
opts.profile = string(localGetField(opts, 'profile', "guide_grid"));
opts.output_prefix = localGetField(opts, 'output_prefix', 'transition_trim_search');
opts.target_strategy = string(localGetField(opts, 'target_strategy', "guide_grid"));
opts.target_limit = localGetField(opts, 'target_limit', inf);
opts.vinf_grid_mps = localGetField(opts, 'vinf_grid_mps', 20:2.5:50);
opts.tilt_offsets_deg = localGetField(opts, 'tilt_offsets_deg', -7.5:2.5:7.5);
opts.tilt_grid_deg = localGetField(opts, 'tilt_grid_deg', 0:5:90);
opts.max_vinf_mps = localGetField(opts, 'max_vinf_mps', max(opts.vinf_grid_mps));
opts.base_vinf_ceiling_mps = localGetField(opts, 'base_vinf_ceiling_mps', 5.0);
opts.vinf_per_tilt = localGetField(opts, 'vinf_per_tilt', 0.8);
opts.skip_known_good_targets = localGetField(opts, 'skip_known_good_targets', false);
opts.cruise_anchor = localGetField(opts, 'cruise_anchor', struct('tilt_deg', 90.0, 'vinf_mps', 70.0));
opts.reference_grid_step_deg = localGetField(opts, 'reference_grid_step_deg', 2.5);
opts.reference_knot_vinf_mps = localGetField(opts, 'reference_knot_vinf_mps', ...
    [0 2.5 5 10 20 30 40 50 60 70]);
opts.reference_knot_tilt_deg = localGetField(opts, 'reference_knot_tilt_deg', ...
    [0 18 30 44 62 72 80 85 88 90]);
opts.front_guide_knot_vinf_mps = localGetField(opts, 'front_guide_knot_vinf_mps', ...
    [0 10 20 30 40 50 60 70 75]);
opts.front_guide_knot_rpm = localGetField(opts, 'front_guide_knot_rpm', ...
    [1880 1890 1885 1860 1760 1600 1380 1050 840]);
opts.rear_guide_knot_vinf_mps = localGetField(opts, 'rear_guide_knot_vinf_mps', ...
    [0 10 20 30 40 50 60 70 75]);
opts.rear_guide_knot_rpm = localGetField(opts, 'rear_guide_knot_rpm', ...
    [1780 1670 1500 1350 1220 1080 800 430 100]);
opts.front_seed_offsets_rpm = localGetField(opts, 'front_seed_offsets_rpm', [-150 -75 0 75 150]);
opts.rear_seed_offsets_rpm = localGetField(opts, 'rear_seed_offsets_rpm', [-150 -75 0 75 150]);
opts.front_collective_min_rpm = localGetField(opts, 'front_collective_min_rpm', 600.0);
opts.rear_collective_min_rpm = localGetField(opts, 'rear_collective_min_rpm', 100.0);
opts.enable_low_speed_physics_seeds = localGetField(opts, 'enable_low_speed_physics_seeds', false);
opts.family_names = localGetField(opts, 'family_names', {'front_rear_free_flap_elevator', 'rear_fixed_flap_elevator'});
opts.stop_after_exact = localGetField(opts, 'stop_after_exact', false);
opts.checkpoint_every = localGetField(opts, 'checkpoint_every', 5);
opts.write_debug_run_outputs = localGetField(opts, 'write_debug_run_outputs', false);
opts.write_linearizations_to_db = localGetField(opts, 'write_linearizations_to_db', true);
opts.refresh_canonical_databases_on_checkpoint = localGetField(opts, 'refresh_canonical_databases_on_checkpoint', false);
opts.score_options = localGetField(opts, 'score_options', struct('profile', 'transition', 'hold_horizon_s', 2.0));
end

function defaults = localProfileDefaults(profile)
profile = lower(string(profile));
defaults = struct('profile', profile);
switch profile
    case "guide_grid"
        defaults.output_prefix = 'transition_trim_search';
        defaults.target_strategy = "guide_grid";

    case "low_speed"
        defaults.output_prefix = 'transition_trim_low_speed_search';
        defaults.target_strategy = "low_speed_frontier";
        defaults.tilt_grid_deg = 0:5:30;
        defaults.vinf_grid_mps = 0:2.5:25;
        defaults.max_vinf_mps = 25.0;
        defaults.base_vinf_ceiling_mps = 5.0;
        defaults.vinf_per_tilt = 0.8;
        defaults.enable_low_speed_physics_seeds = true;
        defaults.neighbor_seed_count = 3;
        defaults.history_seed_count = 3;
        defaults.family_names = {'hover_zero_surface', 'front_rear_free_flap_elevator', ...
            'rear_fixed_flap_elevator', 'prop_fixed_flap_elevator'};
        defaults.front_seed_offsets_rpm = [-100 0 100];
        defaults.rear_seed_offsets_rpm = [-100 0 100];

    case "bridge"
        defaults.output_prefix = 'transition_trim_bridge_search';
        defaults.target_strategy = "bridge_frontier";
        defaults.tilt_grid_deg = 15:5:90;
        defaults.vinf_grid_mps = 20:2.5:50;
        defaults.max_vinf_mps = 50.0;
        defaults.skip_known_good_targets = true;
        defaults.neighbor_seed_count = 5;
        defaults.history_seed_count = 5;
        defaults.family_names = {'front_rear_free_flap_elevator', 'rear_fixed_flap_elevator', ...
            'prop_fixed_flap_elevator'};

    case "blueband"
        defaults.output_prefix = 'transition_trim_blueband_search';
        defaults.target_strategy = "guide_grid";
        defaults.anchor_max_vinf_mps = 35.0;
        defaults.anchor_max_tilt_error_deg = 8.0;
        defaults.vinf_grid_mps = 17.5:2.5:30;
        defaults.tilt_offsets_deg = [-10 -7.5 -5 -2.5 0 2.5];
        defaults.front_guide_knot_vinf_mps = [0 10 20 30 40 50 60 70 75];
        defaults.front_guide_knot_rpm = [1865.76 1860 1840 1800 1700 1550 1380 1100 1000];
        defaults.rear_guide_knot_vinf_mps = [0 10 20 30 40 50 60 70 75];
        defaults.rear_guide_knot_rpm = [1756.38 1660 1540 1425 1300 1120 850 420 100];
        defaults.neighbor_seed_count = 3;
        defaults.history_seed_count = 3;

    otherwise
        error('TrimSearch_Run:UnknownProfile', ...
            'Unknown transition trim search profile: %s', profile);
end
end

function out = localOverlayStruct(defaults, overrides)
out = defaults;
if ~isstruct(overrides)
    return;
end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    fieldName = fields{i};
    out.(fieldName) = overrides.(fieldName);
end
end

function engineOptions = localBuildGuideGridOptions(opts)
engineOptions = struct();
fields = fieldnames(opts);
for i = 1:numel(fields)
    fieldName = fields{i};
    engineOptions.(fieldName) = opts.(fieldName);
end
end

function value = localGetField(s, fieldName, fallback)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = fallback;
end
end
