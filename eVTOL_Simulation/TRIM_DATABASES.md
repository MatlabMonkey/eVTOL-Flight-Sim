# Trim Databases

Last reviewed: 2026-04-25 20:48 PDT

This is the practical guide for trim/search/controller database use.

## The Two Databases

`trim_attempts`

- Search memory.
- Contains all trim attempts: exact, near-trim, borderline, and failed.
- Stores scores, residual summaries, source metadata, and links to saved linearizations.
- New trim searches write here directly.

`controller_schedule`

- Controller-ready subset.
- Derived from the master DB by `TrimDB_Build.m`.
- Keeps controller-eligible points and inlines `A/B/C/D/x0/u0`.
- Controller builders should read this, not raw search outputs.

## Normal Workflow

```matlab
Init_Main
transitionTrimSearchOptions = struct('profile', 'low_speed', 'target_limit', 20);
TrimSearch_Run
TrimDB_Build
```

Data flow:

```text
TrimSearch_Run
  -> databases/trim_attempts.*
  -> databases/transition_trim_linearizations/*.mat
  -> databases/transition_trim_linearization_index.*

TrimDB_Build
  -> reads databases/trim_attempts.*
  -> preserves existing inlined controller points when raw linearization files are absent
  -> databases/controller_schedule.*
```

## Final Workflow Files

The normal handoff workflow needs these files in `databases/`:

- `trim_attempts.mat`
- `trim_attempts.csv`
- `trim_attempts.md`
- `controller_schedule.mat`
- `controller_schedule.csv`
- `controller_schedule.md`

`controller_schedule.mat` is the key controller-facing file. It already inlines
the reduced linear models and trim data needed by `build_corridor_lqr_controller`.

## Optional Generated Artifacts

These are not required to run the wrapper or build the current controller from
`controller_schedule.mat`. They should not be included in the clean handoff
unless somebody is actively debugging/rebuilding linearizations.

New searches may create:

- `transition_trim_linearizations/`
- `transition_trim_linearization_index.csv`
- `transition_trim_linearization_index.mat`

Explicit diagnostics may create:

- `controller_schedule_missing_linearizations.csv`
- `controller_schedule_build_log.md`

Historical backfill/rearon connector raw artifacts are archive-only. Do not put
them back into the clean handoff unless somebody intentionally wants to recreate
old campaign rebuilds.

Generated plots, preview images, and optional run-local debug folders live in
`workspace_plots/`. A database file in `workspace_plots/` should be treated as
an old/misplaced copy, not the current source of truth.

The clean workflow keeps the inlined controller data in `controller_schedule.mat`.
`TrimDB_Build.m` preserves those inlined rows if historical raw files are absent.

## Search Profiles

Use `TrimSearch_Run.m` with `transitionTrimSearchOptions.profile`:

- `guide_grid`: default transition guide-curve search.
- `low_speed`: near-hover / low-speed search with physics-based seeds.
- `bridge`: fills gaps between existing known-good points.
- `blueband`: focused bridge-region search.

Do not add new one-off `Run_Trim_Transition_*` campaign scripts. Add a profile,
target strategy, or seed helper instead.

## Controller DB Contents

Controller points include:

- trim summary fields
- `x0`
- `u0`
- reduced matrices `A_9`, `B_9`, `C_9`, `D_9`
- state/input names
- trim commands and scheduling fields

Reduced state order:

```text
[phi; theta; psi; u; v; w; P; Q; R]
```

Reduced input order:

```text
[front_coll_rpm; rear_coll_rpm; delta_f; delta_a; delta_e; delta_r]
```

`x0` uses radians for attitude states. `u0` stores RPM and surface deflections
in degrees. `trim_cmd_rad` is also included for controller code.

## Recovery / Legacy Import

The archived backfill script and legacy CSV import path are recovery tools
only. They are not part of the normal clean workflow.

Use them only if old master DB rows are missing linked linearization files.

## Related Docs

- `docs/EV_SIMULATION_WORKFLOW_MAP.md`: workflow architecture.
- `docs/EV_SIMULATION_DEPENDENCY_MAP.md`: file dependency map.
- `TRIM_DATABASE_BUILDER_SPEC.md`: detailed builder/schema spec.
