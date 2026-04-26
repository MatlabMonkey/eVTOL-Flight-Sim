function options = build_lowmid_fast_guide_options(root_dir)
%BUILD_LOWMID_FAST_GUIDE_OPTIONS Build a data-driven fast lower-mid guide setup.
% This helper fits the fast guide curves from the current lower-mid results
% so the next sweep follows the natural trend already found by the slower run.

if nargin < 1 || isempty(root_dir)
    stack = dbstack('-completenames');
    if ~isempty(stack)
        root_dir = fileparts(stack(1).file);
    else
        root_dir = pwd;
    end
end

csvFile = fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_scored_latest.csv');

options = struct();
options.anchor_history_csvs = { ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_lowmid_guidegrid_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_map_low_speed_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_reference_line_scored_latest.csv'), ...
    fullfile(root_dir, 'workspace_plots', 'transition_trim_path_scored_latest.csv') };
options.anchor_max_vinf_mps = 40.0;
options.anchor_max_tilt_error_deg = 7.5;
options.vinf_grid_mps = 12.5:2.5:35;
options.tilt_offsets_deg = [-5 -2.5 0 2.5 5];
options.front_seed_offsets_rpm = [-60 0 60];
options.rear_seed_offsets_rpm = [-80 0 80];
options.history_seed_count = 2;
options.neighbor_seed_count = 2;
options.family_names = {'front_rear_free_flap_elevator'};
options.stop_after_exact = true;

fallback = localFallbackGuide();
if exist(csvFile, 'file') ~= 2
    options.reference_knot_vinf_mps = fallback.reference_vinf;
    options.reference_knot_tilt_deg = fallback.reference_tilt;
    options.front_guide_knot_vinf_mps = fallback.front_vinf;
    options.front_guide_knot_rpm = fallback.front_rpm;
    options.rear_guide_knot_vinf_mps = fallback.rear_vinf;
    options.rear_guide_knot_rpm = fallback.rear_rpm;
    return;
end

tbl = readtable(csvFile, 'TextType', 'string');
if isempty(tbl)
    options.reference_knot_vinf_mps = fallback.reference_vinf;
    options.reference_knot_tilt_deg = fallback.reference_tilt;
    options.front_guide_knot_vinf_mps = fallback.front_vinf;
    options.front_guide_knot_rpm = fallback.front_rpm;
    options.rear_guide_knot_vinf_mps = fallback.rear_vinf;
    options.rear_guide_knot_rpm = fallback.rear_rpm;
    return;
end

successMask = localAsLogical(tbl.success);
acceptableMask = localAsLogical(tbl.acceptable);
classMask = tbl.classification == "near_trim_borderline";
keepMask = successMask | acceptableMask | classMask;
tbl = tbl(keepMask, :);
if isempty(tbl)
    options.reference_knot_vinf_mps = fallback.reference_vinf;
    options.reference_knot_tilt_deg = fallback.reference_tilt;
    options.front_guide_knot_vinf_mps = fallback.front_vinf;
    options.front_guide_knot_rpm = fallback.front_rpm;
    options.rear_guide_knot_vinf_mps = fallback.rear_vinf;
    options.rear_guide_knot_rpm = fallback.rear_rpm;
    return;
end

vinfValues = unique(tbl.vinf_mps);
vinfValues = sort(vinfValues);
vinfValues = vinfValues(vinfValues >= 5.0 & vinfValues <= 12.5);
if isempty(vinfValues)
    options.reference_knot_vinf_mps = fallback.reference_vinf;
    options.reference_knot_tilt_deg = fallback.reference_tilt;
    options.front_guide_knot_vinf_mps = fallback.front_vinf;
    options.front_guide_knot_rpm = fallback.front_rpm;
    options.rear_guide_knot_vinf_mps = fallback.rear_vinf;
    options.rear_guide_knot_rpm = fallback.rear_rpm;
    return;
end

obsTilt = zeros(size(vinfValues));
obsFront = zeros(size(vinfValues));
obsRear = zeros(size(vinfValues));
for i = 1:numel(vinfValues)
    mask = abs(tbl.vinf_mps - vinfValues(i)) <= 1e-9;
    obsTilt(i) = mean(tbl.tilt_deg(mask));
    obsFront(i) = mean(tbl.front_collective_rpm(mask));
    obsRear(i) = mean(tbl.rear_collective_rpm(mask));
end

referenceVinf = [0 5 vinfValues(:)' 15 20 25 30 35];
referenceTilt = [0 30 obsTilt(:)' 55 62 68 72 76];

frontPeakV = max(vinfValues(end) + 5, 17.5);
frontPeak = max(obsFront) + 110.0;
frontVinf = [0 vinfValues(:)' frontPeakV 20 25 30 35];
frontRpm = [1865.76 obsFront(:)' frontPeak 2235 2100 1930 1700];

rearVinf = [0 vinfValues(:)' 15 20 25 30 35];
rearRpm = [1756.38 obsRear(:)' 1380 1300 1200 1100 1000];

[referenceVinf, referenceTilt] = localUniqueKnotPairs(referenceVinf, referenceTilt);
[frontVinf, frontRpm] = localUniqueKnotPairs(frontVinf, frontRpm);
[rearVinf, rearRpm] = localUniqueKnotPairs(rearVinf, rearRpm);

options.reference_knot_vinf_mps = referenceVinf;
options.reference_knot_tilt_deg = referenceTilt;
options.front_guide_knot_vinf_mps = frontVinf;
options.front_guide_knot_rpm = frontRpm;
options.rear_guide_knot_vinf_mps = rearVinf;
options.rear_guide_knot_rpm = rearRpm;
end

function out = localAsLogical(col)
if islogical(col)
    out = col;
elseif isnumeric(col)
    out = col ~= 0;
else
    lowered = lower(strtrim(string(col)));
    out = lowered == "1" | lowered == "true" | lowered == "yes";
end
out = logical(out);
end

function fallback = localFallbackGuide()
fallback = struct();
fallback.reference_vinf = [0 5 7.5 10 12.5 15 20 25 30 35];
fallback.reference_tilt = [0 30 38 45 49 55 62 68 72 76];
fallback.front_vinf = [0 5 7.5 10 12.5 15 20 25 30 35];
fallback.front_rpm = [1865.76 1980 2030 2100 2155 2120 2050 1950 1830 1700];
fallback.rear_vinf = [0 5 7.5 10 12.5 15 20 25 30 35];
fallback.rear_rpm = [1756.38 1675 1600 1500 1435 1380 1300 1200 1100 1000];
end

function [xOut, yOut] = localUniqueKnotPairs(xIn, yIn)
xIn = xIn(:);
yIn = yIn(:);
[xSorted, order] = sort(xIn);
ySorted = yIn(order);

[xUnique, ~, groupIdx] = unique(xSorted, 'stable');
yUnique = accumarray(groupIdx, ySorted, [], @mean);

xOut = xUnique.';
yOut = yUnique.';
end
