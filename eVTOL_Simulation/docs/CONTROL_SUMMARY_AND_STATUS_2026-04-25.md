# Control Summary And Status

Last reviewed: 2026-04-25 20:48 PDT

This is a concise status note for the controller side of the project.

## Current Control Path

```text
databases/controller_schedule.mat
  -> select_corridor_path_from_db
  -> build_trim_lqr_controller
  -> build_corridor_lqr_controller
  -> controllerData
  -> Run_Main
```

The active controller is a scheduled longitudinal LQR:

```text
u = u*(s) - K(s)(x - x*(s))
```

The scheduler interpolates trim references, feedforward commands, and gains
along a selected path through the controller DB.

## Active Files

- `controllers/builders/build_corridor_lqr_controller.m`
- `controllers/builders/build_trim_lqr_controller.m`
- `planning/select_corridor_path_from_db.m`
- `controllers/controller_lqr_path_schedule_gated.m`
- `controllers/controller_dispatch.m`
- `controllers/presets/build_corridor_test_preset.m`

## Control Inputs

Main longitudinal channels:

- front collective
- rear collective
- `delta_f`
- `delta_e`

Held/secondary channels:

- `delta_a`
- `delta_r`

## What Works

- Local trim-point longitudinal LQR can be built from DB points.
- Scheduled LQR can interpolate along a corridor path.
- Gated scheduling is the best current runtime strategy.
- The infrastructure now runs from the controller DB, not one-off scripts.

## Main Remaining Risk

The biggest control issue is path quality, not basic wiring.

Some late-transition trim points can lie on poor allocation branches. If the
selected path jumps onto a bad branch, the nonlinear vehicle may remain bounded
but fail to settle cleanly to the scheduled trim command.

So the next useful controller work is:

1. improve controller DB point quality
2. improve corridor path selection
3. retune scheduled LQR only after the path is reasonable

## How To Rebuild Controller Data

```matlab
Init_Main
TrimDB_Build
controllerData = build_corridor_lqr_controller();
```

Then prepare/run with:

```matlab
Run_Main
```

`Run_Main` expects `trimResult`; use
`controllers/builders/trim_result_from_controller_db_point.m` when rebuilding a
run-ready trim point from the controller DB.

## Historical Material

Older two-point demos, route experiments, and controller prep scripts are in:

- `archive/control_experiments/`

They are reference material only. Do not treat them as the active workflow.
