# Transition Corridor Controller Build Log

This note is a handoff / report document for the work done on `2026-04-21`
to move from the consolidated trim databases toward a runnable scheduled
transition controller in `eVTOL_Simulation`.

It focuses on:

- how the trim databases were used for control design
- how the corridor path was selected
- how the scheduled longitudinal controller was assembled
- what scheduler variants were tried
- what bugs were found and fixed
- what still needs attention

This document is intentionally practical. It is not a polished paper-style
derivation; it is a repo-grounded record of what was built and why.

## Context

The aircraft is being controlled in longitudinal motion only for this phase.
The working assumption in this workflow is:

- active controlled channels:
  - front collective
  - rear collective
  - `delta_f`
  - `delta_e`
- ignored / held-fixed lateral channels:
  - `delta_a`
  - `delta_r`

The controller architecture reuses the existing MATLAB-function dispatch path
already present in the `Wrapper` model, specifically the scheduled-path LQR
branch (`controller_id = 4`).

Relevant earlier notes:

- [CONTROL_FORMULATION_AND_WORKING_DEMOS_2026-04-21.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/docs/CONTROL_FORMULATION_AND_WORKING_DEMOS_2026-04-21.md)
- [TRIM_DATABASES.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/TRIM_DATABASES.md)
- [TRIM_DATABASE_BUILDER_SPEC.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/TRIM_DATABASE_BUILDER_SPEC.md)
- [TRANSITION_TRIM_DATABASE_AND_CONTROL_REPORT.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/docs/TRANSITION_TRIM_DATABASE_AND_CONTROL_REPORT.md)

## Starting Point

The consolidated trim DB state used for this work was:

- `transition_trim_master_attempt_db.mat`
- `controller_schedule_db.mat`

The important interpretation was:

- `transition_trim_master_attempt_db` is the search / audit cloud
- `controller_schedule_db` is the controller-facing candidate set

Important discovery during inspection:

- `controller_schedule_db` is **not** single-valued over `(Vinf, tilt)`
- it contains multiple exact-trim candidates at many scheduling coordinates
- therefore it should **not** be treated as a clean 2D interpolation surface
  without additional branch selection

At the time of inspection:

- controller DB points: `723`
- duplicated `(tilt_deg, vinf_mps)` cells: `103`

So the practical conclusion was:

- use one selected corridor path first
- interpolate locally along that ordered path
- do **not** perform naive 2D interpolation across the full DB cloud

## Trim Database Maps

Representative DB-native map used during this work:

![Transition Trim DB Map](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_db_visuals_20260421_145503/transition_trim_db_master_plus_controller.png)

Representative DB metric maps:

![Transition Trim DB Metrics](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_db_visuals_20260421_145503/transition_trim_db_metric_maps.png)

Representative front/rear collective visualization:

![Transition Trim DB Prop RPM](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_db_visuals_20260421_145503/transition_trim_db_prop_rpm.png)

Saved auto-selected corridor path visualization:

![Selected Corridor Path](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/selected_transition_corridor_path/selected_transition_corridor_path.png)

## Core Design Decision

The main design decision was:

- do not select “the best point” independently at each `(Vinf, tilt)` cell
- instead, select one smooth ordered branch through the controller DB

The path-selection logic was kept intentionally simple:

1. define a hand-picked blue guide corridor in `(Vinf, tilt)` space
2. find controller-ready candidates near each guide waypoint
3. solve a small dynamic-programming path problem
4. minimize:
   - distance to the guide waypoint
   - jumps in front RPM
   - jumps in rear RPM
   - jumps in `delta_f`
   - jumps in `delta_e`
   - jumps in `theta`

This produced a usable ordered path without trying to solve a full surface
selection problem over the whole map.

## Files Added For Corridor Selection

### 1. Path selector

- [select_transition_corridor_path_from_controller_db.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/select_transition_corridor_path_from_controller_db.m)

Purpose:

- load `controller_schedule_db`
- define or accept a guide corridor
- select one ordered path through controller-ready points
- optionally save a plot of the selected path

### 2. Blue-guide path selector launcher

- [Select_Transition_Corridor_Path_BlueGuide.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Select_Transition_Corridor_Path_BlueGuide.m)

Purpose:

- easy one-command execution of the selector using the default blue guide

## Controller Construction

The scheduled controller was built by reusing the existing scheduled-path
runtime branch in the repo rather than inventing a new Simulink integration.

The runtime branch still used is:

- [controllers/controller_dispatch.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_dispatch.m)
- [controllers/controller_lqr_path_schedule.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_lqr_path_schedule.m)

The local controller at each path point is still:

- longitudinal trim-point LQR
- state subset:
  - `[theta, u, w, Q]`
- active inputs:
  - `[front_collective, rear_collective, delta_f, delta_e]`

The lateral channels `delta_a` and `delta_r` are **not** part of the active
LQR. They are treated as fixed finite trim/default channels.

### Files Added For Controller Build

#### 1. Rebuild trimResult from controller DB point

- [rebuild_trim_result_from_controller_db_point.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/rebuild_trim_result_from_controller_db_point.m)

Purpose:

- reconstruct a run-ready `trimResult` from `controllerScheduleDB.points(i)`
- avoid rerunning trim just to build a controller or stage the wrapper

#### 2. Corridor-path controller builder

- [build_transition_corridor_lqr_controller.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/build_transition_corridor_lqr_controller.m)

Purpose:

- call the corridor selector
- rebuild `trimResult` at each selected point
- build a trim-point longitudinal LQR at each point
- package:
  - `controller_state_ref`
  - `controller_trim_cmd`
  - `controller_gain_lqr`
  in the same format the existing scheduled runtime expects

#### 3. Simple builder launcher

- [Build_Controller_From_Transition_Corridor_LQR.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Build_Controller_From_Transition_Corridor_LQR.m)

Purpose:

- easy one-command build path similar to the earlier transition-path helpers

## Wrapper Prep Scripts

### Main corridor prep script

- [Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR.m)

Purpose:

- build the corridor controller
- build command traces
- stage `Wrapper`
- set:
  - initial state
  - controller package
  - command traces
  - stop time

### Simple launcher

- [Prep_Demo_Transition_Corridor_BlueGuide.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_BlueGuide.m)

Purpose:

- convenience wrapper for the default corridor prep

## Scheduler Variants Built

Three schedule styles were built or tested.

### 1. Global smooth ramp

File:

- [make_transition_path_schedule_cmds.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/make_transition_path_schedule_cmds.m)

Purpose:

- one continuous ramp in progress `s`
- command traces interpolated continuously across the path

Important later improvement:

- added `command_anchor_source`
  - `'selected_path'`
  - `'guide'`

This matters because using the selected trim points directly as command
anchors can create staircase-like command behavior if the selected path has
flat / repeated scheduling coordinates.

Using `'guide'` makes the commanded `(tilt, Vinf)` pair follow the smooth
blue guide instead.

### 2. Segmented slow point-to-point scheduler

File:

- [make_transition_path_schedule_cmds_segmented.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/make_transition_path_schedule_cmds_segmented.m)

Purpose:

- hold at one point
- ramp to the next point
- hold again
- continue along the selected path

This was built specifically to test the hypothesis that the original
transition failed because the scheduled reference moved too quickly.

### 3. Preset wrapper/demo scripts

Added:

- [Prep_Demo_Transition_Corridor_BlueGuide_Slow.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_BlueGuide_Slow.m)
- [Prep_Demo_Transition_Corridor_BlueGuide_Smooth20s.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_BlueGuide_Smooth20s.m)
- [Prep_Demo_Transition_Corridor_BlueGuide_Smooth60s.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_BlueGuide_Smooth60s.m)
- [Prep_Demo_Transition_Corridor_FirstSegment_Smooth20s.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_FirstSegment_Smooth20s.m)

Purpose:

- `Slow`:
  - very slow segmented schedule
  - used for first-pass corridor debugging
- `Smooth20s`:
  - full corridor
  - smooth guide-based commands
  - 20-second run
- `Smooth60s`:
  - full corridor
  - smooth guide-based commands
  - 60-second run
- `FirstSegment_Smooth20s`:
  - only point 1 to point 2
  - isolate the earliest transition slice

## Bugs Found And Fixed

### 1. NaN actuator initial conditions

Observed symptom:

- Simulink integrator initial condition error
- `tailL_servo/Integrator` had `InitialCondition = NaN`

Root cause:

- some controller DB points had missing `delta_a` / `delta_r`
- trim reconstruction passed those through as `NaN`
- surface mapping produced `surface_init = [NaN NaN NaN NaN]`

Fix:

- sanitize missing mixed-control channels in
  [rebuild_trim_result_from_controller_db_point.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/rebuild_trim_result_from_controller_db_point.m)
- fall back to:
  - point values if finite
  - trim-case fixed/guess values
  - otherwise zero

Post-fix result:

- selected path points no longer contain NaNs in reconstructed plant inputs
- `surface_init` is finite when the wrapper is staged

### 2. Prep script missing `trimResult` handoff

Observed symptom:

- `Run_EVTOL_Main` error:
  - `trimResult is required`

Root cause:

- the corridor prep script computed `initialTrim` but did not also provide a
  variable literally named `trimResult` for `Run_EVTOL_Main`

Fix:

- explicitly bind `trimResult = initialTrim` in
  [Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR.m)

### 3. Slow preset still stopping at 20 seconds

Observed symptom:

- command trace was long
- wrapper/model still stopped at 20 s

Root cause:

- `RunCase_PrepOnly()` defaults `stopTime_s = 20`
- prep logic still inherited that run length

Fix:

- make the prep script force the run stop time from the schedule
- optionally extend command traces to the final run stop time by holding the
  last command value

Result:

- `cmd_end_t`, `stopTime`, and `EVTOL.stopTime` now match for the prepared run

### 4. Stair-step commanded tilt / airspeed path

Observed symptom:

- commanded `tilt` was effectively staircase-like
- that can yank the scheduled gains and trim reference too aggressively

Fix:

- allow command anchors to come from the smooth guide instead of the selected
  trim points

Result:

- `Smooth20s` and `Smooth60s` now use the blue guide as the commanded path

## Verified Presets

At the time of this note, the following were verified at the wrapper-prep
stage:

### `Prep_Demo_Transition_Corridor_BlueGuide_Smooth20s`

- stop time: `20 s`
- command end time: `20 s`
- tilt command: `0 -> 90 deg`
- `Vinf` command: `0 -> 70 m/s`

### `Prep_Demo_Transition_Corridor_BlueGuide_Smooth60s`

- stop time: `60 s`
- command end time: `60 s`
- tilt command: `0 -> 90 deg`
- `Vinf` command: `0 -> 70 m/s`

### `Prep_Demo_Transition_Corridor_FirstSegment_Smooth20s`

- stop time: `20 s`
- command end time: `20 s`
- tilt command: `0 -> 10 deg`
- `Vinf` command: `0 -> 2.5 m/s`

### `Prep_Demo_Transition_Corridor_BlueGuide_Slow`

- long slow segmented debug run
- command trace extended to the full requested stop time

## Current Assessment

The current corridor-controller workflow is now good enough to support the
next debugging phase.

The repo now has:

- a clean way to select a controller-ready corridor from the DB
- a clean way to rebuild run-ready trim points from controller DB entries
- a scheduled longitudinal LQR builder along that corridor
- multiple scheduler styles for debugging
- preset wrapper launchers for full-path and first-segment tests

The main unresolved question is no longer “how do we stage and run the
controller?” The main unresolved question is:

- whether the selected corridor and trim allocation are actually good enough
  dynamically in the low / mid transition region

If the first-segment smooth test fails, that localizes the problem to the
very first hover-to-low-speed transition slice.

If the full smooth 60 s run fails but the first-segment test works, that
pushes attention toward:

- the mid-transition corridor points
- rear-thrust allocation dropping too early
- local controller authority around the `45-65 deg` tilt band

## Suggested Next Steps

1. Run [Prep_Demo_Transition_Corridor_FirstSegment_Smooth20s.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_FirstSegment_Smooth20s.m)
   - verify the earliest path slice can be tracked

2. Run [Prep_Demo_Transition_Corridor_BlueGuide_Smooth60s.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_BlueGuide_Smooth60s.m)
   - evaluate whether a smoother full transition schedule is enough

3. If failure persists, inspect:
   - rear rotor schedule through the mid-transition points
   - trim allocation at the selected path points
   - whether the selected path itself should be changed before retuning the
     LQR

4. If needed later, add:
   - state-gated progress rather than purely time-based progress
   - but only after the smoother guide-based time schedule is understood

## Later Iterations: Path Quality, Gated Scheduling, And Tuning

The earlier sections describe the first working corridor-controller build.
The work after that point shifted from "make the architecture run" to
"identify why the selected path fails dynamically and make the path/control
combination more realistic."

The main findings from that second phase were:

- the original scheduled-path LQR was too willing to lean on surfaces
- several selected path points were mathematically valid trims but poor
  control waypoints
- the biggest failures were caused by branch changes in trim allocation,
  not just by aggressive schedule timing
- a time-only scheduler was not sufficient once the path points stopped being
  locally easy to reach

### Report Plotting Split

During debugging, the base report plotting script was heavily modified to
show path overlays, switch times, target segments, and the scheduled command
state. That was useful for debugging but not desirable as the default report
path.

The final arrangement is:

- restored base report plotter:
  - [plot_report_responses.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/plot_report_responses.m)
- transition/debug report plotter:
  - [plot_report_responses_transition_debug.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/plot_report_responses_transition_debug.m)

The new debug plotter now contains the transition-specific diagnostics:

- selected path table pop-out
- switch-time overlays for gated scheduling
- target segment lines
- scheduled-target figure with:
  - scheduled airdata
  - scheduled Euler commands
  - scheduled tilt
  - scheduled front/rear rotor commands
  - scheduled local surfaces
  - scheduled progress

This keeps the default report workflow simple while preserving the debugging
tooling that was needed to understand the path/controller interaction.

### Why The First Path Failed

Once the path points were plotted directly against the state and actuator
responses, the failure pattern became clearer:

- the controller was not simply "bad everywhere"
- it would reach several early points, then fail to settle around a specific
  mid-transition waypoint
- that waypoint would often lie on a branch where the trim allocation and
  local linearization suddenly became surface-dominant

In practical terms, the controller would:

- approach the selected point
- demand large `delta_f` / `delta_e`
- hit surface limits
- fail the settle condition
- or move on before truly stabilizing if using a pure time scheduler

The important conclusion was:

- some selected points were exact trims, but they were not good
  **controller waypoints**

This distinction mattered a great deal. The scheduling problem was not just
"pick exact trims." It was "pick exact trims that produce a controllable and
smooth branch for the chosen longitudinal controller."

### LQR Tuning Progression

The original trim-point longitudinal LQR builder used one fixed `Q`/`R`
design across the full corridor. The active state/input sets stayed:

- states:
  - `[theta, u, w, Q]`
- inputs:
  - `[front_collective, rear_collective, delta_f, delta_e]`

The baseline tuning was:

```matlab
Q_long = diag([3.4, 0.45, 0.35, 0.95]);
R_long = diag([6.5, 20.0, 100.0, 100.0]);
```

That tuning was hand-set and behaved too conservatively for transition.
Several stronger prop-biased presets were then added to force the controller
to rely more on rotor collective and less on surfaces.

Representative presets added:

- moderate prop bias
- strong prop bias
- extreme prop bias
- ultra prop bias

The current strongest preset used in the route-comparison work is:

```matlab
Q_long = diag([12.0, 1.20, 3.20, 5.00]);
R_long = diag([0.45, 0.70, 3200.0, 2600.0]);
```

That tuning materially improved behavior. It did not fix bad path points by
itself, but it moved the failure farther downstream and made it clear that
the selector/path quality had become the dominant issue.

### Gated Scheduler

The original scheduled-path runtime advanced on time. That was too brittle
for corridor points that required actual stabilization before moving on.

To address that, a gated scheduler was added:

- [controllers/controller_lqr_path_schedule_gated.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_lqr_path_schedule_gated.m)
- [controllers/controller_dispatch.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_dispatch.m)

The gated controller now:

- holds the current point until the measured longitudinal state is close
  enough
- then ramps toward the next point
- keeps transitioning once a segment has started, rather than repeatedly
  restarting the same ramp

The gating configuration is now packed into the scheduled controller data
inside:

- [build_transition_corridor_lqr_controller.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/build_transition_corridor_lqr_controller.m)

This avoided adding new Simulink ports.

The stricter gating used in later tests was approximately:

```matlab
settle_theta_deg   = 2.5;
settle_u_mps       = 0.75;
settle_w_mps       = 0.75;
settle_q_deg_s     = 2.5;
settle_time_s      = 1.5;
segment_ramp_time_s = 6.0;
```

This was a necessary improvement, but not a sufficient one. If the next path
point was structurally poor, the controller still failed; it just failed more
honestly.

### Path Selector Refinements

The selector started as a smoothness-based dynamic program over a guide
corridor. That was useful as a first approximation, but the later failures
showed two additional issues:

1. soft penalties alone were not enough
2. branch changes in rear-thrust allocation were especially damaging

The selector was therefore extended to include:

- hard limits on:
  - `max_abs_delta_f_deg`
  - `max_abs_delta_e_deg`
  - `max_abs_theta_deg`
- hard per-step limits on:
  - `max_delta_vinf_per_step`
  - `max_delta_tilt_per_step`
- a regional rear-RPM floor:
  - before `V < 45` and `tilt < 75`, require rear collective to stay above a
    minimum threshold
- rear-RPM drop penalties and a hard rear-drop limit per step

These changes were made in:

- [select_transition_corridor_path_from_controller_db.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/select_transition_corridor_path_from_controller_db.m)

The key motivation was to avoid paths where:

- the selected trims looked smooth in `(V, tilt)` space
- but the rear-prop contribution collapsed abruptly
- leaving the surfaces to do nearly all of the longitudinal work

### Important Selector Caveat

Even after the selector refinements, one important limitation remained:

- the selector still truncates each guide waypoint to the top local
  candidates before dynamic programming

That means a globally good branch can still be hidden if it does not rank
well under the local point score. This mattered in the bridge-route work,
where the only viable bridge candidate around `(50,70)` was being dropped
until the candidate count was increased.

### Route Comparison

To make the path issue explicit, a family of route presets was added through:

- [build_transition_corridor_test_preset.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/build_transition_corridor_test_preset.m)

These presets compare:

- low route
- mid route
- high route
- high-bridge route

with corresponding demo launchers:

- [Prep_Demo_Transition_Corridor_LowRoute_Smooth120s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_LowRoute_Smooth120s_Tune_RpmBiasUltra.m)
- [Prep_Demo_Transition_Corridor_MidRoute_Smooth120s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_MidRoute_Smooth120s_Tune_RpmBiasUltra.m)
- [Prep_Demo_Transition_Corridor_HighRoute_Smooth120s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_HighRoute_Smooth120s_Tune_RpmBiasUltra.m)
- [Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth120s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth120s_Tune_RpmBiasUltra.m)

The route-comparison work showed:

- `low` and `mid` still drove the selector toward branches with poor
  mid-transition allocations
- the original `high` route moved the failure farther downstream, but still
  fell onto a branch where rear thrust collapsed too early
- the `highbridge` route was introduced specifically to skip the unreachable
  `25/70 -> 30/75` guide region while keeping rear support alive longer

### Current Best Candidate Path

After adding:

- stricter step-size limits
- rear-floor logic
- rear-drop penalties
- increased local candidate count for the bridge route

the current best path selected by the dry-build process is approximately:

1. `LowSpeedScored_Tilt0_V0`
2. `LowSpeedScored_Tilt10_V2p5`
3. `MidbandGuide_Tilt25_V5`
4. `MidbandGuide_Tilt32p5_V5`
5. `MidbandGuide_Tilt47p5_V15`
6. `MidbandGuide_Tilt57p5_V25`
7. `ReferenceLineScored_Tilt65_V35`
8. `FE_tilt_70__V_50__rear_500`
9. `FE_tilt_85__V_60__rear_600`
10. `FE_tilt_85__V_65__rear_600`
11. `FE_tilt_90__V_70__rear_600`

This is the path used by:

- [Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth120s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth120s_Tune_RpmBiasUltra.m)

and the longer-duration variant:

- [Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth240s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth240s_Tune_RpmBiasUltra.m)

### Why The High-Bridge Path Looks Better

The improvement is not arbitrary. It comes from preserving a better rear-thrust
branch for longer.

The problematic earlier branch had a step where:

- tilt stayed nearly constant
- speed increased only slightly
- rear collective dropped dramatically

That forced the surfaces to carry too much of the local correction.

The bridge-route updates instead keep rear collective supported through the
mid-transition region and only transition toward the FE branch later, when the
guide and the available trim points make that transition more defensible.

### Remaining Weaknesses

The work so far improved the path/controller pairing substantially, but it is
not "solved" in a formal sense. The main remaining weaknesses are:

- the selector still does not use `B`-matrix effectiveness explicitly
- local candidate truncation can still hide better global branches
- several path points still approach surface limits
- the controller is still a longitudinal-only scheduled LQR, not a more
  general allocator or trajectory-tracking scheme

So the current result should be interpreted as:

- a pragmatic, runnable corridor-controller workflow
- with a path that is meaningfully better than the earlier ones
- but still dependent on careful route selection and tuning

## Current Recommendation

The current best next test for the repo is:

- [Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth240s_Tune_RpmBiasUltra.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth240s_Tune_RpmBiasUltra.m)

Reason:

- it uses the best current route logic
- it uses the strongest prop-biased tuning tested so far
- it uses stricter gated progression
- and it gives the path more time to settle between larger trim changes

If that run still fails in the same place, the next technical step should not
be more blind LQR retuning. The next step should be one of:

1. add `B`-effectiveness or saturation-margin terms to the selector cost
2. increase local candidate retention further for difficult bridge waypoints
3. explicitly blacklist path points that repeatedly produce impossible local
   demands

Those would attack the remaining problem at the correct layer: path quality,
not just controller aggressiveness.
