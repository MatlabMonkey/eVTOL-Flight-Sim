# eVTOL_Simulation

Last reviewed: 2026-04-25 20:48 PDT

This is the active Six-DoF eVTOL simulation workspace. Use this folder, not
the legacy material outside it.

## Start Here

The current workflow is:

```text
Init_Main
-> Trim_Main or TrimSearch_Run
-> TrimDB_Build
-> build_corridor_lqr_controller
-> Run_Main
```

Most users only need these files:

- `Init_Main.m`: initialize the workspace.
- `Trim_Main.m`: solve one explicit trim case.
- `TrimSearch_Run.m`: search for transition trim points.
- `TrimDB_Build.m`: rebuild the master/controller trim databases.
- `TrimDB_Plot.m`: plot trim database coverage.
- `Run_Main.m`: prepare and optionally run `Wrapper.slx`.

## Common Tasks

### Initialize

```matlab
cd('/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/eVTOL_Simulation')
Init_Main
```

### Run A Transition Trim Search

```matlab
Init_Main
transitionTrimSearchOptions = struct('profile', 'low_speed', 'target_limit', 20);
TrimSearch_Run
TrimDB_Build
```

Useful profiles:

- `guide_grid`: default transition search.
- `low_speed`: near-hover / low-speed search with physics-based seeds.
- `bridge`: fills gaps between known-good points.
- `blueband`: focused bridge-region search.

### Build/Use Controller Data

The controller workflow expects `databases/controller_schedule.*`.

```matlab
TrimDB_Build
controllerData = build_corridor_lqr_controller();
Run_Main
```

Durable trim/controller data lives in `databases/`. The clean handoff keeps
`trim_attempts.*` and `controller_schedule.*` there. Generated plots, previews,
and optional debug run folders live in `workspace_plots/`.

If `databases/controller_schedule.mat` is missing, restore the database
files or regenerate them with `TrimSearch_Run` and `TrimDB_Build`.

## Models

The active Simulink source models are:

- `Plant.slx`
- `Trim_Plant.slx`
- `Wrapper.slx`
- `eVTOL_lib.slx`

## Documentation

Read in this order:

1. `README.md`: quick start and current file names.
2. `TRIM_DATABASES.md`: how trim/search/controller databases work.
3. `docs/EV_SIMULATION_WORKFLOW_MAP.md`: workflow-level architecture.
4. `docs/EV_SIMULATION_DEPENDENCY_MAP.md`: file dependencies and rename table.
5. `docs/CONTROL_SUMMARY_AND_STATUS_2026-04-25.md`: current control status.

Historical or deep-reference docs are marked as such. Do not start with them.

## Do Not Use As Current Workflow

- `cases/*`
- `Run_EVTOL_Suite.m`
- `Trim_EVTOL_Sweep.m`
- `Build_Controller_EVTOL_Cruise.m`
- per-run `*_latest.csv/.mat/.md` files as source-of-truth handoffs

Those paths are legacy/historical. The current workflow is database-centered.
