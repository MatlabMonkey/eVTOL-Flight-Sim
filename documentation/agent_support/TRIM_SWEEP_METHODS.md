# Trim Sweep Methods

This note is the practical handoff for how the trim sweeps in this repo have
been run, what the scripts are doing, and what assumptions are currently
working or failing.

It is not a theory note. It is an operations note for another agent.

## High-Level Goal

We are trying to populate a transition trim map over:
- airspeed `Vinf`
- front tilt angle

The immediate control-design goal is not just "find any trim." The useful
points are:
- rear-on points by default
- exact trims first
- acceptable / quasi-usable near-trims second
- points with reasonable continuity in front/rear collective and control
  surface usage

Zero-rear cruise-like points were explored earlier, but the user does not
want those to dominate controller-facing datasets or plots.

## Core Trim Engine

All of these sweep scripts eventually call:
- [trim_evtol_case.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/trim_evtol_case.m)

That uses:
- `findop`
- the trim model:
  - `Trim_Plant`

Typical sweep pattern:
1. choose a target `(tilt_deg, vinf_mps)`
2. generate a set of seed guesses
3. try one or more trim "families"
4. score the result with:
   - [score_trim_point.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/score_trim_point.m)
5. save the best result for that target

## Main Search Families

The most successful family in the later fast patch runs has been:
- `front_rear_free_flap_elevator`

Meaning:
- front collective free
- rear collective free
- flap free
- elevator free
- other direct surfaces fixed

Earlier runs also tried families like:
- `rear_fixed_flap_elevator`
- fixed-prop cleanup variants
- hover-like or zero-surface variants

But for the fast local patch runs, the current pragmatic default is:
- one family only:
  - `front_rear_free_flap_elevator`

That keeps runtime down and avoids fighting multiple basins at once.

## Seeding Strategy History

Several seeding methods were tried.

### 1. Legacy coefficient / analytical seeds

These used simple coefficient-based force-balance ideas. They were useful as
early experiments but do not match the current plant truth well enough.

Those simple scalar aero coefficients are not the real active aero model
anymore. The real model uses lookup tables / effective angle-of-attack based
polars.

### 2. Props-first analytical seeds

We tried:
- front thrust
- rear thrust
- theta / alpha

with surfaces fixed first, then a cleanup pass.

This was physically sensible, but it still did not reliably land in the
correct `findop` basin by itself.

### 3. Scored corridor sweeps

We then moved to:
- multiple seeds
- one or more trim families
- score all returned points
- keep exact or usable near-trims

This became the basis of:
- reference-line searches
- mid-band corridor searches
- later fast patch runs

### 4. Fast local patch runs

This is the current practical method.

Instead of sweeping large areas, define a tight region:
- a small airspeed band
- a small tilt band
- a local front collective guide
- a local rear collective guide
- small offsets around those centers

Then:
- use nearby successful points as anchors
- use the smallest useful seed grid
- stop as soon as an exact trim is found for a target

This is how the later files named `*_fast` were built.

## Current Useful Runner Pattern

The reusable shared engine is:
- [Run_Trim_Transition_Midband_GuideGrid_Scored.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_Midband_GuideGrid_Scored.m)

Most later patch runners are thin wrappers around it:
- they define guide/target options in `build_*_options.m`
- they set:
  - `output_prefix`
  - `latest_prefix`
- then they call the shared runner

Example wrappers:
- [Run_Trim_Transition_LowMid_GuideGrid_Fast.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_LowMid_GuideGrid_Fast.m)
- [Run_Trim_Transition_LeftBridge_Fast.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_LeftBridge_Fast.m)
- [Run_Trim_Transition_UpperMid_Bridge_Fast.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_UpperMid_Bridge_Fast.m)
- [Run_Trim_Transition_MainBridge_BlueCircle_Fast.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_MainBridge_BlueCircle_Fast.m)
- [Run_Trim_Transition_MainBridge_BlueCircle_Dense.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_MainBridge_BlueCircle_Dense.m)
- [Run_Trim_Transition_MainBridge_MidBasin_Fast.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_Trim_Transition_MainBridge_MidBasin_Fast.m)

## What an Options Builder Usually Defines

Typical builder fields:
- `anchor_history_csvs`
- `anchor_max_vinf_mps`
- `anchor_max_tilt_error_deg`
- `vinf_grid_mps`
- `tilt_offsets_deg`
- `front_seed_offsets_rpm`
- `rear_seed_offsets_rpm`
- `history_seed_count`
- `neighbor_seed_count`
- `family_names`
- `stop_after_exact`
- `reference_knot_vinf_mps`
- `reference_knot_tilt_deg`
- `front_guide_knot_vinf_mps`
- `front_guide_knot_rpm`
- `rear_guide_knot_vinf_mps`
- `rear_guide_knot_rpm`

That is the current knob set for targeted search design.

## How Seeds Are Chosen

The shared runner builds seeds from:
1. prior run CSVs listed in `anchor_history_csvs`
2. points already found in the current run
3. guide-center RPM guesses at the active `Vinf`
4. local front/rear offset grids around those guide centers
5. sometimes blended or interpolated seeds

Important: `findop` is basin-sensitive.

That means:
- if the target basin is not explicitly seeded, it is common to get repeated
  failures even though nearby exact trims exist elsewhere
- this is why local patch runs matter

## Scoring

The fast ranking/filtering step is:
- [score_trim_point.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/score_trim_point.m)

It uses short-horizon drift logic based on the operating-point residuals.

Important classifications:
- `exact_trim`
- `quasi_trim_usable`
- `near_trim_borderline`
- `not_usable`

Interpretation:
- exact trims are preferred
- acceptable / quasi-usable points may still be useful for path finding or
  temporary controller candidates
- borderline and failed points are mostly search diagnostics

## Global Attempt DB

The shared runner now uses a global attempt cache:
- [transition_trim_global_attempt_db_latest.csv](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_global_attempt_db_latest.csv)

Purpose:
- avoid repeating already-solved targets
- merge attempt history across patch runs

Important limitation:
- this file is a search cache, not the clean controller dataset
- if it becomes corrupted or schema-misaligned, runners can fail during
  checkpoint/update

## Output Conventions

Each runner usually writes:
- `workspace_plots/<prefix>_latest.csv`
- `workspace_plots/<prefix>_latest.mat`
- `workspace_plots/<prefix>_latest.md`
- `workspace_plots/<prefix>_<timestamp>/`

This means each patch run is self-contained, but it also means the repo now
contains many separate CSV summaries.

For data organization, see:
- [TRIM_DATABASES.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/TRIM_DATABASES.md)
- [TRIM_DATABASE_BUILDER_SPEC.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/TRIM_DATABASE_BUILDER_SPEC.md)

## Plotting

Useful plots:
- merged map:
  - [Plot_All_Transition_Trim_Points_Map.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Plot_All_Transition_Trim_Points_Map.m)
- run-local plots:
  - `Plot_Transition_*_Map.m` wrappers

The merged plotter now:
- excludes zero-rear rows by default
- includes the custom patch CSVs by default

## What Has Been Working Best

The most productive method has been:
1. run a broad or medium corridor search once
2. inspect the propeller-value plots
3. identify a specific basin or gap
4. make a very local fast patch run
5. reuse its successful points as anchors
6. iterate

This has worked better than:
- large blind sweeps
- trying to force the entire map through one global guide curve
- expecting `findop` to jump between widely separated propeller basins

## Common Failure Modes

1. **Wrong basin**
- Guide curves are centered in the wrong front/rear RPM region.
- Result: all attempts fail even though nearby exact trims exist elsewhere.

2. **Too-broad band**
- Airspeed or tilt range is too wide.
- Result: runtime explodes and the run spends time on obviously bad targets.

3. **Anchor mismatch**
- Search starts too far from the nearest successful patch.
- Result: no continuity, especially around `30–40 m/s`.

4. **Global DB checkpoint errors**
- The shared cache writer can fail if string/missing values are not handled
  robustly.
- This can kill a run after it has already solved some points.

5. **Plot confusion**
- Some custom patch CSVs were initially not included in the merged plotter.
- A run could succeed or fail, but not show up in the all-points view.

## Practical Guidance For New Patch Runs

If another agent is making a new patch run:
1. start from the nearest working patch, not the global map
2. define a narrow airspeed window
3. define a narrow tilt band
4. center front/rear guide curves in the actual basin seen in the prop plots
5. use only one trim family first:
   - `front_rear_free_flap_elevator`
6. keep seed grids small unless there is a clear basin-search reason to go
   wider
7. give the run a unique `output_prefix` and `latest_prefix`
8. make sure the output CSV name is added to:
   - the merged plotter defaults if needed
   - the database-builder spec include list if the run matters long-term

## Current Design Direction

The long-term data model should be:
1. `master_attempt_db`
2. `controller_schedule_db`

The builder spec for that is:
- [TRIM_DATABASE_BUILDER_SPEC.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/TRIM_DATABASE_BUILDER_SPEC.md)

That builder should replace the current ad hoc "many CSVs everywhere"
workflow for controller design.*** End Patch


