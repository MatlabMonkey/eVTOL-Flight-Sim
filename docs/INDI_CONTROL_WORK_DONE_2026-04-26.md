# INDI Control Work Done

Last reviewed: 2026-04-27 00:00 PDT

Status update: this note is now partially superseded by the first implemented
scheduled INDI controller in `controllers/controller_indi_transition.m` and
the builder in `controllers/builders/build_indi_transition_controller.m`. The
readable math/code walkthrough is
`docs/INDI_TRANSITION_CONTROLLER_CODE_WALKTHROUGH.html`.

This document summarizes the INDI control work completed so far. It is a
handoff note for continuing the controller implementation, not a final report.

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
