# eVTOL Flight Sim Keep/Drop Audit

Last reviewed: 2026-04-26 17:48 PDT

Scope frozen on 2026-04-25 after the first active-root cleanup pass.

This audit covers the active repo-root tree and excludes:

- `archive/`
- `.simcache/`
- `.simcode/`
- `.workflow_cache/`

The goal is aggressive local cleanup. The bias is:

- keep the current initialize/trim/controller/database/run path
- keep the minimum generic trim-search and plotting engines needed for future work
- move campaign residue and stale docs into `archive-only`
- treat generated plots/videos as `remove-local`

## Coverage Summary

The matching manifest is:

- `planning/ev_simulation_keep_drop_manifest.csv`

This audit document is the classification source. Cleanup actions are performed
separately from the manifest.

Final cleanup moved the root-level `archive-only` set into
`archive/final_cleanup_20260425_2059/` and removed generated database/report
outputs from the active tree.

## What Stays

### Core Runtime And Models

Keep the active runtime chain:

- `Init_Main.m`
- `Trim_Main.m`
- `trim_evtol_case.m`
- `Run_Main.m`
- `Plant.slx`
- `Trim_Plant.slx`
- `Wrapper.slx`
- `eVTOL_lib.slx`
- `aircraft_def.m`
- `load_trim_result_from_db.m`

These are the files that preserve the current initialize -> trim -> run workflow.

### Databases And Controller Path

Keep the current controller/database toolchain:

- `TrimDB_UpdateMaster.m`
- `TrimDB_Build.m`
- `score_trim_point.m`
- `planning/select_corridor_path_from_db.m`
- the active `controllers/` runtime/build subtree

Within `controllers/`, keep the whole current subtree. It is small, coherent, and contains the live runtime dispatch, the corridor builder, the trim-point LQR builder, and the schedule builders. `controllers/README.md` has been reviewed and now describes the builder-chain plus `Run_Main.m` workflow.

Preferred trim DB workflow:

1. generic searches append directly into `trim_attempts`
2. generic searches save linearization artifacts directly into `databases/transition_trim_linearizations`
3. generic searches link those artifacts from the master DB using `linearization_latest_file`
4. `TrimDB_Build.m` rebuilds `controller_schedule` from the canonical master DB plus the linked linearization files
5. legacy CSV import through the builder manifest remains available only for recovery or one-time migration

`TrimDB_BackfillLinearizations.m` is now archive-only for the clean workflow. It is still useful historical/recovery code if old rows lack saved linearizations, but new trim searches should not depend on replay MATs or backfill.

### Canonical User-Facing Scripts

The active root should feel small. These are the top-level scripts that still make sense as user-facing entrypoints:

- `Init_Main.m`
- `Trim_Main.m`
- `Run_Main.m`
- `TrimSearch_Run.m`
- `TrimDB_Build.m`
- `TrimDB_Plot.m`

Everything else should be treated as one of:

- a reusable helper behind those entrypoints
- an optional manual diagnostic
- historical residue that should move out of the active root

### Non-Canonical But Legitimate Kept Utilities

These are still reasonable to keep, but they are not the preferred top-level workflow:

- `TrimSearch_Preview.m`
- `measure_trim_control_effectiveness.m`
- `plot_outputs.m`
- `plotting/plot_report_responses*.m`

The rule is:

- keep them because they add reusable capability
- do not treat them as the main path unless a workflow explicitly needs them

### Generic Trim Search

Keep the surviving generic trim-search path:

- `TrimSearch_Run.m`
- `TrimSearch_Engine.m`
- `TrimSearch_BuildPlan.m`
- `make_low_speed_prop_seed.m`
- `make_low_speed_surface_seed.m`
- `TrimSearch_Preview.m`

This is the right level to preserve. `TrimSearch_Run.m` is the user-facing entrypoint; the midband guide-grid script is the current engine behind it.

The generic search runner now supports reusable profiles instead of one-off campaign scripts:

- `guide_grid`: default guide-curve search
- `low_speed`: low-speed frontier targets plus physics-based prop/two-pass seeds
- `bridge`: bridge/frontier target ordering around existing known-good points
- `blueband`: focused version of the old blue-band guide-grid campaign

The generic search runner writes search memory directly into the canonical master attempt DB and writes linearization artifacts directly when `trim_evtol_case` returns a reduced model. That is the future-facing path. Its run-local checkpoint files are optional debug outputs, not canonical database sources.

### Active Plotting And Visualization

Keep the current generic plotting and visualization helpers:

- `TrimDB_Plot.m`
- `plot_outputs.m`
- `plotting/plot_report_responses.m`
- `plotting/plot_transition_debug.m`
- `render_aircraft.m`
- `scenario_def.m`
- `measure_trim_control_effectiveness.m`

`databases/` is the canonical directory for durable trim/controller/aero data.

Within `databases/`, the durable source-of-truth assets are:

- `trim_attempts.*`
- `controller_schedule.*`
- `aero_polars/final_airfoil_polar_tables.mat`
- `transition_trim_linearization_index.*` and `transition_trim_linearizations/` when created by new searches

`workspace_plots/` stays as the generated-output directory for plots, previews,
and optional run-local debug folders.

Per-run checkpoint folders should be treated as short-lived debug artifacts unless a specific run needs to be preserved for review. New clean runs should not create `*_latest.csv/.mat/.md` handoff files.

Historical backfill/rearon connector linearization folders are no longer part of
the active handoff. The current controller DB already inlines the controller
linearizations, and `TrimDB_Build.m` preserves those inlined rows if historical
raw files are absent.

`report_plots_final/` stays as an output directory only. Its generated contents do not stay.

### Current Docs Worth Keeping

Keep these docs:

- `AGENTS.md`
- `README.md`
- `docs/EV_SIMULATION_DEPENDENCY_MAP.md`
- `docs/EV_SIMULATION_KEEP_DROP_AUDIT.md`
- `docs/EV_SIMULATION_WORKFLOW_MAP.md`
- `TRIM_DATABASES.md`
- `TRIM_DATABASE_BUILDER_SPEC.md`
- `WORKFLOW_ARCHITECTURE_SPEC.md`

These still describe the active database or architecture path and are useful to another engineer or agent.

## What Moves To Archive-Only

### Stale Or Historical Docs

Archive these because they are useful context but not safe as current instructions:

- `GROUP_WORKFLOW_GUIDE.html`
- `TRIM_EXPLORATION_NOTES.md`
- `TRIM_MAP_SCRIPT_SUMMARY.md`

Main reason:

- they point at removed `cases/*` folders, removed trim-map wrappers, or older helper names

### Already Removed From The Active Root

These were removed from the live tree during the cleanup pass because they were not part of the healthy active chain:

- `Plot_All_Transition_Trim_Points_Map.m`
- `Plot_Transition_Midband_GuideGrid_Scored_Map.m`
- `Run_EVTOL_Suite.m`
- `Trim_EVTOL_Sweep.m`
- `run_batch_linearization.m`
- `control_effectiveness_fd_test.m`
- `plot_evtol_workspace_outputs.m`
- `plot_responses_Jake.m`

Main reason:

- they are either superseded
- or they depend on missing `RunCase_*`, `TrimCase_*`, or `TrimSweep_*` helpers
- or they are compatibility aliases rather than core functionality

The active replacements are:

- `TrimSearch_Run.m` for transition trim search
- `trim_evtol_case.m` and `Trim_Main.m` for single-case trim
- `TrimDB_Plot.m` for trim/controller DB plotting
- `plot_outputs.m` and `plotting/plot_report_responses*.m` for wrapper output plots

### Demo And Report Helpers

Archive these because they are nice-to-have but not core:

- `Good Transitions/`
- `make_evtol_video.m`
- `export_report_aircraft_render.m`

This keeps the active runtime/controller/database path clean while still allowing recovery from the zip/archive if demo generation matters later.

## What Is Safe To Remove Locally

Immediate `remove-local` items:

- `.DS_Store`
- `report_plots_final/*`

Reason:

- these are generated outputs or OS metadata, not source-of-truth assets

`report_plots_final/` itself should remain, because kept plotting/report helpers still write there.

## Sensitive Notes

`README.md` was rewritten on 2026-04-25 and is now a current top-level handoff guide.

`Run_EVTOL_Suite.m`, `Trim_EVTOL_Sweep.m`, and `run_batch_linearization.m` were removed from the active root because the helper families they expected are gone from the clean workflow.

`Plot_All_Transition_Trim_Points_Map.m` and `Plot_Transition_Midband_GuideGrid_Scored_Map.m` were removed from the active root. The DB-native plotting path is `TrimDB_Plot.m`.

`Front_Propeller_Loads.m` has no plain-text inbound refs, but it looks like model-support code. It should be treated as `keep` unless the model blocks are checked directly.

`measure_trim_control_effectiveness.m` is kept even though it is manual-only, because it is the reusable generic diagnostic. The old one-off wrapper was removed.

## Workflow Reduction Findings From The Second Pass

### Already Improved

The biggest cleanup win is already in place:

- the generic transition search runner now appends directly into `trim_attempts` through `TrimDB_UpdateMaster.m`
- the generic transition search runner now writes solved/acceptable linearizations directly into `databases/transition_trim_linearizations`
- `TrimDB_Build.m` now treats the canonical master DB as the default source, honors direct `linearization_latest_file` links, and only uses explicit CSV import as rebuild/recovery mode

That removes the old mandatory path:

- `search -> per-run CSV -> importer -> replay MAT/backfill -> controller DB`

and replaces it with:

- `search -> canonical master DB + linked linearizationPoint MAT -> controller DB`

### Remaining Non-Canonical Paths

There are still a few paths in the tree that are useful only outside the clean workflow:

1. `TrimSearch_Engine.m` can still write run-local checkpoint MAT/CSV/MD/log files when `write_debug_run_outputs=true`.

   Those are disabled by default and are for debugging/crash review only.

2. `TrimDB_BackfillLinearizations.m` is no longer part of the normal workflow. Keep it only in archive/recovery context for old rows that lack saved linearization artifacts.

### What “Ideal” Looks Like

For the active tree, the ideal steady-state workflow is:

1. search runner finds trim points
2. search runner writes canonical trim memory directly into `trim_attempts`
3. search runner writes linearization artifacts directly and links them from the master DB
4. `TrimDB_Build.m` rebuilds `controller_schedule`
5. controller builders and plotters read from the canonical DBs only

That means any remaining file/output path that exists only to shuttle data between those steps should be treated as a cleanup target.

## Safe Deletion Set

If the goal is to cut the active tree hard while preserving the current workflow:

1. Delete all `remove-local` paths now.
2. Move all `archive-only` paths to the zip or `archive/`.
3. Leave the `keep` set untouched.

That yields a smaller active tree centered on:

- current runtime/models
- current trim path
- current DB builder path
- current controller runtime/build path
- the surviving generic trim-search engine
- the DB-native plotting path

## Next Cleanup Pass

After this audit, the next high-value cleanup would be:

1. move the remaining `archive-only` set out of the active root when the team is comfortable.
2. leave `workspace_plots/` and `report_plots_final/` as directories, but not as storage for long-term source assets.
3. keep `Last reviewed` timestamps current when docs are edited.
