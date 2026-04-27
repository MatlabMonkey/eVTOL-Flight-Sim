# Agent Notes For eVTOL Flight Sim

Last reviewed: 2026-04-26 17:48 PDT

This is the active MATLAB/Simulink eVTOL workspace. The active folder is the
repo root, not a nested `eVTOL_Simulation/` subfolder. Treat `archive/` as
historical unless the user explicitly asks for it.

## Read First

1. `README.md`
2. `TRIM_DATABASES.md`
3. `docs/EV_SIMULATION_WORKFLOW_MAP.md`
4. `docs/EV_SIMULATION_DEPENDENCY_MAP.md`
5. the specific file the user asked about

## Current Workflow

```text
Init_Main
-> Trim_Main or TrimSearch_Run
-> TrimDB_Build
-> build_corridor_lqr_controller
-> Run_Main
```

The active models are:

- `Plant.slx`
- `Trim_Plant.slx`
- `Wrapper.slx`
- `eVTOL_lib.slx`

Treat `.slx` files as source, not generated junk.

Generated aero polars are durable data, not scratch output:
`databases/aero_polars/final_airfoil_polar_tables.mat` is loaded directly by
`Init_Main`. INDI surface-map tools require those generated AVL polar tables
and do not fall back to coefficient-generated aircraft polars. `scripts/` is
restored support tooling for rebuilding or validating those polars, but the
runtime workflow should not depend on keeping every old script.

## Trim/Search Rules

- Use `TrimSearch_Run.m` for transition searches.
- Add search ideas as profiles, target strategies, or seed helpers.
- Do not restore one-off `Run_Trim_Transition_*` campaign wrappers unless the user explicitly asks for temporary scratch work.
- New searches write to `databases/trim_attempts.*` and, when enabled, linked linearization files.
- `TrimDB_Build.m` derives `controller_schedule.*`.

## Database Rules

Durable database assets live in `databases/`:

- `trim_attempts.*`
- `controller_schedule.*`

New trim searches may also create `transition_trim_linearizations/` and
`transition_trim_linearization_index.*`. Those are generated support artifacts,
not required for the current handoff if `controller_schedule.mat` already has
the inlined controller points.

Generated plots, previews, and optional debug run folders live in
`workspace_plots/`.

Per-run `*_latest.csv/.mat/.md` files are not the current source-of-truth path.

## Editing Rules

- Prefer small, direct changes.
- Preserve the clean current workflow over legacy compatibility.
- Do not rename Simulink models unless explicitly requested.
- Do not rename base-workspace variables unless explicitly requested.
- If a doc is edited, update its `Last reviewed` line.
- If a doc is historical, say so near the top.

## Validation

Use the smallest relevant check:

- docs-only change: timestamp/reference/manifest checks
- MATLAB helper change: `checkcode` and a narrow smoke test
- trim/search change: target-limited or zero-target smoke first
- controller change: builder discovery or small prep smoke first

If a check cannot run because databases/toolboxes/files are missing, say that.
