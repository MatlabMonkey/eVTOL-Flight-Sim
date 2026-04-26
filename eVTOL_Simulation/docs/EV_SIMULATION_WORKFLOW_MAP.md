# eVTOL_Simulation Workflow Map

Last reviewed: 2026-04-25 20:48 PDT

This is the simple architecture map for the active workflow. For file-level
dependencies and old-to-new filenames, see `docs/EV_SIMULATION_DEPENDENCY_MAP.md`.

## Active Workflow

```text
Init_Main
  -> Full_Sim_Init
  -> initData

Single trim:
  trimCase
  -> Trim_Main
  -> trim_evtol_case
  -> trimResult

Transition search:
  TrimSearch_Run
  -> TrimSearch_Engine
  -> trim_evtol_case
  -> score_trim_point
  -> TrimDB_UpdateMaster
  -> databases/trim_attempts
  -> databases/transition_trim_linearizations

Database build:
  TrimDB_Build
  -> databases/trim_attempts
  -> preserves existing inlined controller rows when raw linearization files are absent
  -> databases/controller_schedule

Controller build:
  build_corridor_lqr_controller
  -> controllerData

Runtime:
  Run_Main
  -> Wrapper.slx
```

## What Each Layer Owns

`Init_Main`

- Adds paths.
- Loads aircraft/model defaults.
- Creates `initData`.

`Trim_Main`

- Solves one explicit `trimCase`.
- Produces `trimResult`.
- Uses `trim_evtol_case`.

`TrimSearch_Run`

- Runs transition trim searches.
- Uses profiles such as `low_speed`, `bridge`, and `guide_grid`.
- Writes to the master attempt DB and linked linearization files.

`TrimDB_Build`

- Curates search results into `controller_schedule`.
- Loads linked linearizations.
- Inlines controller matrices and trim vectors.

`build_corridor_lqr_controller`

- Selects a path through `controller_schedule`.
- Builds scheduled longitudinal LQR data.
- Produces `controllerData`.

`Run_Main`

- Converts `trimResult`, `controllerData`, and `runCase` into workspace values.
- Optionally runs `Wrapper.slx`.

## The Database Boundary

The clean architecture has one important boundary:

```text
search results -> master DB -> controller DB -> controller builder
```

Durable DB assets live in `databases/`. Generated plots and optional debug
run folders live in `workspace_plots/`.

Search scripts should not write controller-ready data directly. Controller
builders should not read one-off search folders directly.

## Current Models

- `Trim_Plant.slx`: trim model.
- `Wrapper.slx`: main runtime model.
- `Plant.slx`: plant model.
- `eVTOL_lib.slx`: shared library.

## What Is Legacy

Do not use these as current workflow:

- `cases/*`
- `Run_EVTOL_Suite.m`
- `Trim_EVTOL_Sweep.m`
- old `Run_Trim_Transition_*` campaign wrappers
- per-run `*_latest.csv/.mat/.md` as source-of-truth files

Archived scripts can be useful as historical reference, but active work should
go through the current workflow above.
