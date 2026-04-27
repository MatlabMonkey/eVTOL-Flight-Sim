# eVTOL Flight Sim

Last reviewed: 2026-04-26 17:48 PDT

This is the active Six-DoF eVTOL simulation workspace. The current working
folder is the repo root:

```text
/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim
```

Use this folder, not the old nested `eVTOL_Simulation/` layout.

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
cd('/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim')
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

### Plot Aero Polars

```matlab
Init_Main
plot_aero_polars
```

To also save PNGs under `workspace_plots/`:

```matlab
plot_aero_polars(struct('saveFigures', true))
```

## Support Assets

Generated aero data lives with the other durable databases:

```text
databases/aero_polars/final_airfoil_polar_tables.mat
```

If that file is missing, `Init_Main` will warn and LUT aero blocks will not have
the generated AVL polar table data. INDI surface-map tools treat this file as
required and will error instead of generating coefficient-based fallback polars.

`scripts/` contains restored support tooling for rebuilding or validating those
polars, but the runtime workflow should not depend on keeping every old script.

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
