# INDI Control Work Done

Last reviewed: 2026-04-28 02:57 PDT

Status update: this note is now partially superseded by the first implemented
scheduled INDI controller in `controllers/controller_indi_transition.m` and
the builder in `controllers/builders/build_indi_transition_controller.m`. The
readable math/code walkthrough is
`docs/INDI_TRANSITION_CONTROLLER_CODE_WALKTHROUGH.html`.

This document summarizes the INDI control work completed so far. It is a
handoff note for continuing the controller implementation, not a final report.

## Full Hover-To-Cruise INDI Transition Update

The current best full-transition result is the deduped 4+3+5 setup. The intent
was to reduce actuator limit hits using three changes:

```text
4. soften the pitch/qdot demand
3. regularize the control surfaces harder in the allocator
5. penalize trim-path points with large steady surface deflections
```

The first two changes still reached cruise, but the elevator continued to hit
the `-25 deg` surface limit. The third change initially looked like it avoided
limits, but it exposed a path-builder bug: the dynamic-programming path could
select the same trim point for consecutive guide waypoints. Because path
progress is inferred from unique increasing progress values, duplicate
consecutive path points made the controller truncate the schedule early and
stop around the mid-transition point.

The implemented fix is in:

```text
controllers/builders/build_indi_transition_controller.m
```

After path selection, the builder now drops consecutive duplicate kinematic path
rows before packing the controller schedule. The duplicate test checks:

```text
Vinf, tilt, alpha, theta
```

This preserves repeated nearby regions if the actual kinematic point changes,
but prevents identical rows from creating zero-progress schedule entries.

### Successful 4+3+5 Tuning Case

The successful case used:

```matlab
builderOpts.path.point_weight_surface = 3.0;
builderOpts.path.max_abs_surface_deg = 18.0;

builderOpts.outer_loop.kq = 2.20;
builderOpts.outer_loop.ktheta = 2.10;
builderOpts.outer_loop.ktheta_high_speed = 2.60;

builderOpts.allocation.virtual_error_weights = [0.30; 1.00; 4.0];
builderOpts.allocation.control_regularization = [1.2e-6; 1.2e-6; 3.0; 3.0];
builderOpts.allocation.delta_eta_limits = [1.5e6; 1.5e6; deg2rad(5.0); deg2rad(5.0)];
builderOpts.allocation.rotor_trim_feedforward_blend = 0.0;
builderOpts.allocation.surface_trim_feedforward_blend = 0.0;

builderOpts.runtime_g.enabled = true;
builderOpts.runtime_g.entry_name = 'stall_boundary_delta_alpha_dense';
```

The selected path has 7 points:

```text
idx  Vinf  tilt  alpha  theta    front_rpm  rear_rpm  flap    elevator  class
1    0     0     0      0.000    1865.7     1756.4    0.000   0.000     exact
2    5     15    0      8.159    1900.2     1727.4   -1.689   1.068     exact
3    20    25    5      12.909   1859.2     1521.7   -0.136   2.985     exact
4    30    37.5  5      16.552   1744.0     1130.6    5.882  -1.048     quasi
5    40    52.5  2.5    17.651   1695.9      873.0    7.184  -5.772     quasi
6    60    67.5  2.5     8.191   1150.7      778.2    0.316 -10.708     exact
7    70    90    0       3.389   1002.5      979.4    1.792  -8.052     exact
```

The 300 s run reached the cruise target while staying off the surface limits:

```text
final Vinf             = 70.000 m/s
final alpha            = ~0 deg
final theta error      = ~0 deg
final u error          = ~0 m/s
max abs flap actual    = 21.10 deg
max abs elevator actual = 21.78 deg
surface limit hit      = false
```

The same setup was also run for 150 s for report plots. The 150 s run uses the
same path and controller settings, with a shorter stop time.

Saved artifacts:

```text
workspace_plots/indi_full_hover_to_cruise_limit_tuning/
  03b_pitch_qdot_soft_surface_path_penalty_deduped/
  03b_pitch_qdot_soft_surface_path_penalty_deduped_150s/
```

Useful plot commands after a run:

```matlab
plot_report_responses(out, [], ...
    fullfile(runOutDir, 'report_output', 'INDI_FullHoverToCruise_03b_150s_report'), ...
    'INDI Full Hover To Cruise 03b 150s');

plot_indi_transition_debug(out, ...
    fullfile(runOutDir, 'INDI_FullHoverToCruise_03b_deduped_150s'), ...
    'INDI Full Hover To Cruise 03b Deduped 150s');

plot_indi_transition_trim_path_map(result.controllerData, ...
    struct('show_popup', true, ...
           'output_dir', fullfile(runOutDir, 'path_map'), ...
           'alpha_zlim_deg', [-5 12]));
```

Validation run:

```text
checkcode controllers/builders/build_indi_transition_controller.m: PASS
```

### Refined Surface-Regularized Tune

After the deduped 4+3+5 case was working, a second tuning pass varied the
surface allocator regularization and qdot virtual-axis weighting. The useful
trend was clear: path-cost changes had little effect on the dynamic surface
peaks, but increasing the surface regularization in the allocator strongly
reduced flap/elevator excursions while still reaching the final cruise point.

The best 150 s refinement was:

```matlab
builderOpts.allocation.virtual_error_weights = [0.30; 1.00; 2.0];
builderOpts.allocation.control_regularization = [1.2e-6; 1.2e-6; 10.0; 10.0];
```

with the same path, gating, outer-loop gains, runtime G-map, and no trim
feedforward:

```matlab
builderOpts.outer_loop.kq = 2.20;
builderOpts.outer_loop.ktheta = 2.10;
builderOpts.outer_loop.ktheta_high_speed = 2.60;
builderOpts.allocation.rotor_trim_feedforward_blend = 0.0;
builderOpts.allocation.surface_trim_feedforward_blend = 0.0;
```

The 150 s sweep ranking for the best cases was:

```text
case          reaches target  max abs flap  max abs elevator
reg10_q2      yes             8.43 deg      8.38 deg
reg8_q3       yes            10.10 deg      9.65 deg
reg8_q2p5     yes            10.31 deg      9.63 deg
reg6_q3       yes            13.62 deg     11.87 deg
03b baseline  yes            21.10 deg     21.78 deg
```

The best `reg10_q2` case was then rerun for 300 s:

```text
final Vinf              = 70.000 m/s
final alpha             = ~0 deg
final theta             = 3.3888 deg
final u error           = ~0 m/s
final theta error       = ~0 deg
max abs flap actual     = 8.426 deg
max abs elevator actual = 8.380 deg
surface limit hit       = false
reaches target          = true
```

Saved artifacts:

```text
workspace_plots/indi_full_hover_to_cruise_tuning_sweep_150s/
workspace_plots/indi_full_hover_to_cruise_tuning_refine_150s/
workspace_plots/indi_full_hover_to_cruise_best_reg10_q2_300s/
```

## Follow-Up INDI Tuning Sweep

After the `reg10_q2` validation, a narrower sweep was run around the same
controller structure:

```matlab
builderOpts.allocation.virtual_error_weights = [0.30; 1.00; 2.0];
builderOpts.allocation.control_regularization = [1.2e-6; 1.2e-6; surfaceReg; surfaceReg];
```

The goal was to reduce peak flap/elevator motion without losing the complete
hover-to-cruise transition. With `qdot` weight fixed at `2.0`, increasing the
surface regularization produced the following successful 150 s cases:

```text
case        reaches target  max abs flap  max abs elevator  min rear actual
reg11_q2    yes             8.30 deg      7.70 deg          326 rpm
reg16_q2    yes             8.28 deg      7.56 deg          ~0 rpm
reg18_q2    yes             8.09 deg      8.01 deg          ~0 rpm
reg20_q2    yes             7.81 deg      8.58 deg          ~0 rpm
```

The surface-only optimum is misleading: higher surface regularization unloads
the rear rotor too aggressively. An asymmetric rotor-regularization branch was
also tested to protect rear RPM. It preserved rear RPM, but it pushed elevator
usage into the `11-25 deg` range and was rejected.

The best balanced follow-up candidate is therefore `reg11_q2`. It was rerun for
300 s:

```text
final Vinf              = 70.000 m/s
final alpha             = ~0 deg
final theta             = 3.3888 deg
final u error           = ~0 m/s
final theta error       = ~0 deg
max abs flap actual     = 8.302 deg
max abs elevator actual = 7.700 deg
min rear actual RPM     = 326 rpm
surface limit hit       = false
reaches target          = true
```

Current recommendation:

```text
reg10_q2 = safer default, slightly higher surfaces, better rear-RPM margin
reg11_q2 = lower peak surfaces, but rear RPM dips to ~326 rpm
```

Saved artifacts:

```text
workspace_plots/indi_full_hover_to_cruise_tuning_refine3_150s/
workspace_plots/indi_full_hover_to_cruise_tuning_refine4_150s/
workspace_plots/indi_full_hover_to_cruise_candidate_reg11_q2_300s/
```

## Current Idea

The next controller direction is a longitudinal INDI controller using:

```text
measured acceleration + actual actuator states + control effectiveness map
```

The proposed INDI control variable is:

```text
eta = [front_rpm^2; rear_rpm^2; delta_f; delta_e]
```

Using `RPM^2` internally is useful because the current propeller model is
approximately linear in `RPM^2`. Surface effectiveness is mapped over:

```text
Vinf, alpha, surface deflection
```

Tilt should remain a slow scheduled variable for the first implementation.

## Files Added

- `controllers/benchmark_trim_plant_surface_sampling.m`
- `controllers/builders/build_indi_surface_effectiveness_map.m`
- `plotting/plot_indi_surface_map_grid_preview.m`
- `docs/INDI_CONTROL_ARCHITECTURE_AND_SURFACE_MAP_PLAN.md`
- `docs/INDI_CONTROL_WORK_DONE_2026-04-26.md`

## Generated Artifacts

No durable INDI surface-effectiveness map is currently kept. The first generated
map used an invalid coefficient-based fallback polar path and was removed.
Regenerate it only after `Init_Main` loads:

```text
databases/aero_polars/final_airfoil_polar_tables.mat
```

When regenerated, the default output paths are:

```text
databases/indi_surface_effectiveness_map_coarse.mat
databases/indi_surface_effectiveness_map_coarse.md
```

Planning, preview, and smoke-test artifacts are temporary and should live in
`workspace_plots/`.

## Invalid Coarse Map Run

The first coarse surface-effectiveness map was generated through `Trim_Plant`,
but it should be treated as invalid because it used the removed fallback polar
path.

Command:

```matlab
map = build_indi_surface_effectiveness_map();
```

Grid:

```text
Vinf_mps  = [2.5 5 10 15 20 30 40 50 60 70 80]
alpha_deg = [-10 -5 0 5 10 15 20 25 30 35 40 45]
delta_deg = [-25 -20 -15 -10 -5 0 5 10 15 20 25]
```

Finite-difference setup:

```text
surface perturbation = 0.5 deg
Trim_Plant stop time = 0.02 s
calls per grid point = 4
total calls          = 5808
```

Runtime:

```text
elapsed        = 2063.6 s
elapsed        = 34.4 min
seconds/call   = 0.3553
```

Stored derivative fields:

```text
map.flap.dF_drad_N_per_rad       [11 12 11 3]
map.flap.dM_drad_Nm_per_rad      [11 12 11 3]
map.elevator.dF_drad_N_per_rad   [11 12 11 3]
map.elevator.dM_drad_Nm_per_rad  [11 12 11 3]
```

Quick integrity check:

```text
calls completed      = 5808
flap force range     = [-187179, 138830] N/rad
elevator moment range = [-67170.1, 0] N*m/rad
```

## Accuracy Notes

The map was sampled through `Trim_Plant`, not from a standalone copied aero
function. For each sample, the surface servo initial state and the commanded
surface value were both set to the sampled deflection. That means the map
represents actual actuator state rather than a command transient.

Prop inputs were zeroed so the logged forces and moments are aerodynamic
surface loads only.

Important caveat: that run happened before the restored AVL polar tables were
back in the repo-root `scripts/` folder. The coefficient-based fallback has now
been removed. `build_indi_surface_effectiveness_map` requires generated AVL
polar tables from `Init_Main` and errors if they are missing.

## Next Steps

1. Build a small plot/inspection helper for the coarse map.
   This should show flap/elevator `dFz/delta` and `dMy/delta` slices over
   `Vinf-alpha` at representative surface deflections.

2. Build the longitudinal INDI allocator.
   It should use `map`, measured acceleration, measured or estimated `q_dot`,
   actual actuator states, and scheduled tilt.

3. Verify actuator-state access in the controller block.
   INDI should use actual rotor speeds and actual surface deflections. If the
   controller only receives commands, expose actual actuator states before
   trusting the controller.

4. Start with trim-hold tests.
   Test one point first, then a short transition segment, then the full
   corridor.

## Fixed-Trim INDI Verification Tests

These notes capture the current debugging pass on `controller_id = 6` before
returning to full path scheduling. The goal was to check whether the INDI loop
works locally at one trim point when the path scheduler is removed from the
problem.

Files used:

```text
Run_INDI_FrozenTrimHold_Test.m
Run_INDI_PerturbationRejection_Test.m
plotting/plot_indi_transition_debug.m
controllers/controller_indi_transition.m
controllers/builders/build_indi_transition_controller.m
```

The tests run through `Wrapper`, not `Trim_Plant`. `Trim_Plant` is only used
offline for trim and effectiveness-map generation.

### Test 1: Frozen Trim Hold

Command pattern:

```matlab
result = Run_INDI_FrozenTrimHold_Test([], ...
    struct('stopTime_s', 60, 'plot_outputs', false));
reportData = plot_indi_transition_debug(out, 'save', ...
    'INDI frozen trim hold');
```

Observed result at the selected cruise trim point: the controller holds the
trim point cleanly. State errors remain small, actuator increments replay near
zero, and measured body specific force matches the trim-specific target
`[g*sin(theta_trim); -g*cos(theta_trim)]`.

### Test 3: Single-State Perturbation Rejection

Command pattern:

```matlab
result = Run_INDI_PerturbationRejection_Test([], 'u', ...
    struct('stopTime_s', 60, 'plot_outputs', false));
result = Run_INDI_PerturbationRejection_Test([], 'w', ...
    struct('stopTime_s', 60, 'plot_outputs', false));
result = Run_INDI_PerturbationRejection_Test([], 'theta', ...
    struct('stopTime_s', 60, 'plot_outputs', false));
result = Run_INDI_PerturbationRejection_Test([], 'q', ...
    struct('stopTime_s', 60, 'plot_outputs', false));
reportData = plot_indi_transition_debug(out, 'save', ...
    'INDI perturbation rejection');
```

Preset perturbations:

```text
u      : +2.0 m/s
w      : +0.75 m/s
theta  : +3.0 deg
q      : +2.0 deg/s
```

Observed result: all four perturbations are rejected without actuator runaway.
The `u` and `w` cases return toward trim slowly but cleanly. The `q` case kills
pitch rate quickly, but the pitch/`qdot` channel is lightly damped and does not
settle as fast as desired. The `theta` case shows the strongest coupling into
`w`, which is expected for longitudinal dynamics but indicates weak damping.

### Combined Perturbation Test

Added a combined preset for faster tuning iteration:

```matlab
result = Run_INDI_PerturbationRejection_Test([], 'all', ...
    struct('stopTime_s', 60, 'plot_outputs', false));
reportData = plot_indi_transition_debug(out, 'save', ...
    'INDI combined perturbation rejection');
```

The `all` preset applies:

```text
u_mps     = +2.0
w_mps     = +0.5
theta_deg = +2.0
q_deg_s   = +1.0
```

A 10 s smoke run reduced `[u, w, theta, q]` absolute errors from
`[2.0, 0.5, 2.0, 1.0]` to approximately
`[0.63, 0.025, 0.74, 0.18]`, using units
`[m/s, m/s, deg, deg/s]`.

### Plotter Update

`plot_indi_transition_debug` now reconstructs INDI debug data if the model does
not log a `scheduler_debug` signal. It replays `controller_indi_transition`
over the logged run data to populate `nu_err` and `delta_eta`, so the
`\Delta eta` allocator panel works for frozen-trim and perturbation tests.

### Current Tuning Read

The local INDI loop appears stable near the cruise trim point. The current
weakness is not the `u`/`w` channels; it is the pitch acceleration channel:
`qdot_cmd - qdot_meas` decays slowly and the pitch response is underdamped.

Current default outer-loop values from
`controllers/builders/build_indi_transition_controller.m` are:

```text
ku      = 0.08
kw      = 0.20
kq      = 0.35
ktheta  = 0.45
```

For the next tuning pass, change the pitch channel first. Prefer increasing
`kq` before increasing `ktheta`, because the observed problem is insufficient
pitch-rate damping rather than insufficient pitch-angle authority. A reasonable
next override is:

```matlab
opts = struct();
opts.outer_loop = struct();
opts.outer_loop.ku = 0.08;
opts.outer_loop.kw = 0.20;
opts.outer_loop.kq = 0.70;
opts.outer_loop.ktheta = 0.35;
opts.outer_loop.accel_error_clip = 2.0;

result = Run_INDI_PerturbationRejection_Test([], 'all', ...
    struct('stopTime_s', 60, ...
           'plot_outputs', false, ...
           'builder_opts', opts));
reportData = plot_indi_transition_debug(out, 'save', ...
    'INDI combined perturbation kq70 ktheta35');
```

If `q` damps faster but `theta` returns too slowly, raise `ktheta` back toward
`0.45`. If the actuator increments become noisy or hit limits, back `kq` down
or increase pitch-axis allocation regularization.

### Airdata Debug And Regularized Aggressive Test

`plotting/plot_indi_transition_debug.m` now creates a second airdata figure
with measured/reference/error traces for:

```text
Vinf, alpha, beta
```

This is useful for checking whether an INDI test is accidentally moving into a
high-alpha region while the body-state plots still look acceptable. The debug
replay fallback was also corrected so alpha comes from
`controllerData.schedule_alpha_deg` instead of stale `controller_state_ref`
rows that now contain settle/ramp settings.

The aggressive combined perturbation test improved response speed, but the
allocator replay warned that the weighted least-squares matrix was nearly
singular. The next test should keep the aggressive outer-loop gains but add
moderate allocator regularization:

```matlab
opts = struct();

opts.outer_loop = struct();
opts.outer_loop.ku = 0.80;
opts.outer_loop.ku_high_speed = 0.80;
opts.outer_loop.kw = 1.20;
opts.outer_loop.kw_high_speed = 1.20;
opts.outer_loop.kq = 3.00;
opts.outer_loop.ktheta = 3.00;
opts.outer_loop.ktheta_high_speed = 3.00;
opts.outer_loop.accel_error_clip = 8.0;

opts.allocation = struct();
opts.allocation.virtual_error_weights = [0.35; 1.0; 8.0];
opts.allocation.control_regularization = [5.0e-7; 5.0e-7; 0.8; 0.8];
opts.allocation.delta_eta_limits = [2.0e6; 2.0e6; deg2rad(8.0); deg2rad(8.0)];

result = Run_INDI_PerturbationRejection_Test([], 'all', ...
    struct('stopTime_s', 30, 'plot_outputs', false, 'builder_opts', opts));

reportData = plot_indi_transition_debug(out, 'save', ...
    'INDI combined aggressive outer loop regularized');
```
