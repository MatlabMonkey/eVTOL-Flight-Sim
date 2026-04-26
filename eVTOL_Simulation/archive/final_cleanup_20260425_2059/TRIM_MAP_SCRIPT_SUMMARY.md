# Trim Map Script Summary

Last reviewed: 2026-04-25 17:03 PDT

Status: historical reference. The scripts described here are old exploration
campaigns and are not the current search architecture. Use
`TrimSearch_Run.m` plus profiles instead.

This note summarizes the two large transition-trim exploration scripts in
`eVTOL_Simulation` and the files they produce.

## Main Scripts

### `Run_Trim_Transition_Map.m`

Purpose:
- Build a broad transition trim map between the current hover anchor and the
  cruise-side exact trim family.
- Use continuation-style seeding instead of a blind brute-force sweep.
- Reuse known exact trims to seed nearby targets.

What it explores:
- Hover anchor from `TrimCase_Hover()`
- Cruise anchors from `TrimCase_Cruise75_FlapElevator()`
- Rear-fixed flap/elevator families at selected tilt and speed values
- A broad transition corridor
- A hover-side zero-surface family

Current behavior:
- Leaves results in the base workspace when run as a script:
  - `transitionTrimMap`
  - `transitionTrimSummary`
  - `transitionTrimOutputDir`
  - `transitionTrimCheckpointFile`
  - `transitionTrimLogFile`

Main outputs:
- `workspace_plots/transition_trim_map_<timestamp>/`
  - `transition_trim_map.mat`
  - `transition_trim_map_summary.csv`
  - `transition_trim_map_summary.md`
  - `transition_trim_map.log`

Convenience latest copies:
- `workspace_plots/transition_trim_map_latest.mat`
- `workspace_plots/transition_trim_map_latest.csv`
- `workspace_plots/transition_trim_map_latest.md`

Current result from the completed overnight run:
- `991` targets attempted
- `582` exact successes
- strongest exact family: `cruise_flap_elevator_rear_fixed`

Best use:
- broad first-pass map generation
- finding strong exact corridors in tilt / speed / rear-fixed space

### `Run_Trim_Transition_Map_LowSpeed.m`

Purpose:
- Extend the map into the low-speed / low-tilt region without deleting or
  replacing the original map.
- Use the existing exact map as a seed bank.
- Use a simple force-balance-style propeller seed to guess front thrust and
  rear-fixed values more intelligently.

Key differences from the main script:
- Focuses on low-speed bridge regions instead of the whole map
- Loads exact entries from `transition_trim_map_latest.mat`
- Builds rear-fixed targets around a physics-informed rear schedule
- Uses the flap/elevator family for low-speed transition search
- Removes the duplicate NED-down steady-state hold and relies on the
  vertical-speed output constraint for level trim

Current behavior:
- Leaves results in the base workspace when run as a script:
  - `transitionTrimLowSpeedMap`
  - `transitionTrimLowSpeedSummary`
  - `transitionTrimLowSpeedOutputDir`
  - `transitionTrimLowSpeedCheckpointFile`
  - `transitionTrimLowSpeedLogFile`
  - `transitionTrimMergedSummary`

Main outputs:
- `workspace_plots/transition_trim_map_low_speed_<timestamp>/`
  - `transition_trim_map_low_speed.mat`
  - `transition_trim_map_low_speed_summary.csv`
  - `transition_trim_map_low_speed_summary.md`
  - `transition_trim_map_low_speed.log`

Convenience latest copies:
- `workspace_plots/transition_trim_map_low_speed_latest.mat`
- `workspace_plots/transition_trim_map_low_speed_latest.csv`
- `workspace_plots/transition_trim_map_low_speed_latest.md`
- `workspace_plots/transition_trim_map_merged_latest.csv`
- `workspace_plots/transition_trim_map_merged_latest.md`

Current status:
- this script was launched after the first overnight run
- it already found additional exact low-speed points near:
  - `tilt = 45 deg, V = 50 m/s, rear = 600 rpm`
  - `tilt = 45 deg, V = 47.5 m/s, rear = 600 rpm`
  - `tilt = 45 deg, V = 47.5 m/s, rear = 750 rpm`

Best use:
- filling the low-speed bridge between hover and the stronger mid-transition
  corridor
- testing whether better prop seeding recovers exact trims below the original
  map's successful speed band

## Recommended Workflow

1. Run the broad script first:
   - `Run_Trim_Transition_Map`
2. Inspect:
   - `workspace_plots/transition_trim_map_latest.csv`
   - `workspace_plots/transition_trim_map_latest.md`
3. Then run the low-speed script:
   - `Run_Trim_Transition_Map_LowSpeed`
4. Inspect:
   - `workspace_plots/transition_trim_map_low_speed_latest.csv`
   - `workspace_plots/transition_trim_map_low_speed_latest.md`
   - `workspace_plots/transition_trim_map_merged_latest.csv`

## What To Look For In The Output

Most useful columns in the CSV summaries:
- `name`
- `phase`
- `family`
- `tilt_deg`
- `vinf_mps`
- `rear_fixed_rpm`
- `success`
- `max_state_residual`
- `front_collective_rpm`
- `rear_collective_rpm`
- `delta_f_deg`
- `delta_e_deg`
- `theta_deg`
- `u_mps`
- `w_mps`
- `alpha_deg`

Interpretation tips:
- `success = 1` means the trim met the exact operating-point constraints.
- Low residual but `success = 0` can still be useful as a seed neighborhood.
- Large `delta_f` / `delta_e` values usually mean the point is being carried
  more by aero surfaces than by thrust vectoring alone.

## Files To Check First

If you only want the shortest path to the current results, open:
- [transition_trim_map_latest.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_map_latest.md)
- [transition_trim_map_low_speed_latest.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_map_low_speed_latest.md)
- [transition_trim_map_merged_latest.csv](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/workspace_plots/transition_trim_map_merged_latest.csv)
