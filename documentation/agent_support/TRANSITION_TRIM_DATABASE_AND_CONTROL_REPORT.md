# Transition Trim Database And Control Report

This note explains:

- what the consolidated transition trim databases are
- where the point cloud came from
- how the controller-ready points were assembled
- how missing linearizations were backfilled
- how the current controller workflow uses the resulting data

This is meant as a handoff document for other agents and teammates working in
`eVTOL_Simulation`.

## Scope

The canonical outputs live in:

- `workspace_plots/transition_trim_master_attempt_db.mat`
- `workspace_plots/transition_trim_master_attempt_db.csv`
- `workspace_plots/controller_schedule_db.mat`
- `workspace_plots/controller_schedule_db.csv`

The main builder scripts are:

- `Build_Transition_Trim_Databases.m`
- `Build_Transition_Trim_Linearization_Backfill.m`

The formal source-list and normalization spec is:

- `TRIM_DATABASE_BUILDER_SPEC.md`

The shorter practical usage note is:

- `TRIM_DATABASES.md`

## What These Databases Mean

There are two different products on purpose.

### 1. `transition_trim_master_attempt_db`

This is the audit and search database.

It contains:

- exact trims
- acceptable / quasi-trim points
- borderline points
- failed points
- normalized metadata across old and new CSV schemas
- source-run identity
- flags showing whether a matched linearization file exists

This database is for:

- plotting the full explored trim region
- avoiding repeated search work
- checking which search families covered which parts of the envelope
- diagnosing where the search still fails

### 2. `controller_schedule_db`

This is the controller-facing design dataset.

Default controller filter:

- `classification == exact_trim`
- `rear_collective_rpm > 1`
- matched linearization file required
- smoke sources excluded by default

This database is for:

- selecting a transition corridor path
- building scheduled trim commands
- building scheduled gain sets
- reconstructing run-ready trim/linearization points without rerunning trim

In `controller_schedule_db.mat`, the important fields are:

- `controllerScheduleDB.summary_table`
- `controllerScheduleDB.points`

`summary_table` is the flat lookup table.

`points` is the rich struct payload and includes:

- trim summary fields
- `trimCase`
- `Att_Trim_deg`
- `Vel_B_BA_Trim`
- `x0`
- `u0`
- `trim_cmd_rad`
- reduced model matrices `A_9/B_9/C_9/D_9`
- reduced model state/input/output names
- full-order matrices if available

## Current Snapshot

Snapshot below is from the latest completed rebuild on `2026-04-21`.

From `controller_schedule_db_build_log.md`:

- Master all rows: `4023`
- Master best unique points: `2031`
- Controller candidate rows kept: `723`
- Controller candidates missing linearization: `2`
- Rows with matched linearization reference: `1317`
- Rows with existing linearization file: `1317`

From the latest backfill summary:

- Backfill rows saved in the full run: `701`
- Of those, `561` were produced by replaying stored `trimCase`
- Of those, `140` were produced directly from stored `trimResult.linear`
- Backfill index currently contains `705` rows total because it also includes a few earlier smoke-test/manual rows written during development

So the current state is:

- the controller DB is now dense enough to support real corridor selection work
- the remaining missing controller-ready rows are small in number
- the main bottleneck is no longer “we have points but no linearizations”

## Where The Points Came From

The point cloud was not produced by one monolithic sweep. It came from a
stack of search families, all writing their own `*_latest.csv` and
`*_latest.mat` outputs into `workspace_plots`.

The consolidated builder intentionally uses an explicit source manifest rather
than recursively crawling the folder.

Current source categories in `Build_Transition_Trim_Databases.m`:

### Broad map / legacy map sources

- `transition_trim_map_latest.csv`
- `transition_trim_map_low_speed_latest.csv`
- `transition_trim_map_low_speed_scored_latest.csv`
- `transition_trim_map_bridge_scored_latest.csv`
- `transition_trim_map_merged_latest.csv`

These provide the broad envelope coverage and much of the historical search
record.

### Reference-line and corridor searches

- `transition_trim_reference_line_scored_latest.csv`
- `transition_trim_reference_line_midband_scored_latest.csv`

These are corridor-focused searches used to create more controller-usable
paths through the feasible region.

### Lower-mid / bridge patch searches

- `transition_trim_lowmid_guidegrid_scored_latest.csv`
- `transition_trim_lowmid_guidegrid_fast_latest.csv`
- `transition_trim_leftbridge_fast_latest.csv`
- `transition_trim_uppermid_bridge_fast_latest.csv`

These filled holes in the lower-to-mid transition corridor.

### Main-bridge patch searches

- `transition_trim_mainbridge_fast_latest.csv`
- `transition_trim_mainbridge_rescue_fast_latest.csv`
- `transition_trim_mainbridge_handoff_fast_latest.csv`
- `transition_trim_mainbridge_bluecircle_fast_latest.csv`
- `transition_trim_mainbridge_bluecircle_dense_latest.csv`
- `transition_trim_mainbridge_midbasin_fast_latest.csv`
- `transition_trim_mainbridge_downstream_fast_latest.csv`
- `transition_trim_mainbridge_middlegap_fast_latest.csv`
- `transition_trim_mainbridge_v40_basinscan_fast_latest.csv`
- `transition_trim_mainbridge_v42p5_basinscan_fast_latest.csv`
- `transition_trim_mainbridge_highbridge_fast_latest.csv`

These are the local rescue / handoff / basin scans used to densify the
central transition corridor.

### Path and branch searches

- `transition_trim_path_scored_latest.csv`
- `transition_trim_path_scored_branch30v20_latest.csv`
- `transition_trim_path_scored_branch30v25_latest.csv`

These were targeted path-following or branch-following searches rather than
broad gridded exploration.

### Rear-on and cruise-anchor searches

- `transition_trim_rearon_overnight_scored_latest.csv`
- `transition_trim_rearon_forever_scored_latest.csv`
- `transition_trim_rearon_connector_forever_latest.csv`
- `rear_on_cruise_anchor_latest.csv`

These are especially important for controller work because they contain the
rear-on exact trims used to bridge toward cruise and provide matched
linearizations.

## How The Master DB Is Built

`Build_Transition_Trim_Databases.m` performs four main steps.

### Step 1. Read and normalize the explicit CSV sources

The builder handles mixed schemas:

- newer scored/fast CSVs with a native `key`
- scored/fast CSVs without a native `key`
- older map-summary CSVs
- the cruise-anchor summary CSV

If a source row does not already have a key, the builder synthesizes one from
tilt, airspeed, family, source run, and optionally seed identity.

### Step 2. Build the full audit table

All normalized rows are concatenated into:

- `master_attempt_db_all_rows`

This table preserves duplicates across runs because duplicates are useful for
audit and provenance.

### Step 3. Build the best-unique table

The builder then groups rows by a unique-point identity:

- tilt
- airspeed
- family
- rear-fixed setting

Within each group it chooses the best representative using:

- classification rank
- score
- max normalized residual
- stable original order

That produces:

- `master_attempt_db_best_unique_points`

### Step 4. Attach linearization matches

The builder reads the configured linearization index CSVs and attaches a
matched linearization reference to master rows.

Matching logic:

- keyed sources try key-based matching first
- otherwise the builder falls back to tolerance-based matching on:
  - tilt
  - airspeed
  - family
  - front RPM
  - rear RPM

The master DB keeps zero-rear rows. It only flags them:

- `rear_on_ok = rear_collective_rpm > 1`

That distinction matters because zero-rear rows are still useful for search
history and plotting even though they are excluded from the default
controller-ready set.

## Why The Controller DB Was Sparse Before

The first consolidated database build showed the underlying issue clearly:

- there were many exact trim points already in the master DB
- but only a small subset had matched linearization files

In other words, the point cloud existed, but the controller-ready subset did
not because the linearization index only covered part of the explored
envelope.

This was not a trim-search problem. It was a linearization-coverage problem.

## How Missing Linearizations Were Backfilled

`Build_Transition_Trim_Linearization_Backfill.m` fixes that.

### Backfill strategy

The backfill script does not treat the consolidated CSV as the replay source.

Instead it:

1. starts from `transition_trim_master_attempt_db.mat`
2. identifies controller-relevant exact rows still missing a linearization
3. resolves each row back to its original source `*_latest.mat`
4. reuses the stored trim data from that MAT source

There are two supported backfill paths:

### A. Direct save from stored `trimResult.linear`

Some newer MAT-backed sources already preserve:

- `trimCase`
- `trimResult`
- `trimResult.linear`

For those rows, the backfill just packages the existing linearization into a
new `linearizationPoint` file and index row. No new trim solve is required.

### B. Replay from stored `trimCase`

Some older map-backed sources only preserve:

- `trim_case` / `trimCase`
- trim summary metadata

For those rows, the backfill script reruns:

- `trim_evtol_case(initData, trimCase, ...)`

with nonlinear-hold validation disabled by default.

This is still valid because the original `trimCase` preserves the seed and
constraint setup that produced the original solved point.

### Backfill outputs

The script writes:

- `workspace_plots/transition_trim_linearization_backfill_index.csv`
- `workspace_plots/transition_trim_linearization_backfill_index.mat`
- `workspace_plots/transition_trim_linearization_backfill_linearizations/*.mat`
- `workspace_plots/transition_trim_linearization_backfill_summary_latest.csv`
- `workspace_plots/transition_trim_linearization_backfill_summary_latest.md`

By default, after the backfill pass finishes it also reruns
`Build_Transition_Trim_Databases` with the backfill index/root added as extra
linearization sources.

## What A `linearizationPoint` File Contains

The backfill writes the same kind of payload the existing linearization
scripts were already using.

Important contents:

- trim metadata: name, family, key, classification, score
- schedule quantities: tilt, airspeed, RPMs, control deflections
- `trimCase`
- `Att_Trim_deg`
- `Vel_B_BA_Trim`
- `Rates_Trim`
- `linear.A_9/B_9/C_9/D_9`
- reduced state/input/output names
- full-order linear data when available

This is what lets the controller DB inline controller-ready state and model
data instead of pointing at a loose file list.

## How The Current Controller Workflow Uses The DB

There are two different controller-era workflows in the repo right now.

### Current DB-backed corridor/path workflow

These scripts directly use `controller_schedule_db.mat`:

- `select_transition_corridor_path_from_controller_db.m`
- `get_transition_jumpgap_path_spec.m`
- `load_transition_jumpgap_path_points.m`
- `rebuild_trim_result_from_controller_db_point.m`

This is the cleaner workflow.

It assumes:

- the selected path points already live in `controller_schedule_db`
- each point already has matched reduced linearization data
- a run-ready `trimResult` can be reconstructed directly from the controller DB point

That means later wrapper prep or plotting code does not need to rerun trim
just to recover hover/cruise/transition operating points.

### Older scheduled LQR builder path

`Build_Controller_From_Transition_Path_LQR.m` and
`build_transition_path_lqr_controller.m` still default to the older
`trim_linearization_db*.mat` flow.

That is a separate path and has not been fully migrated to the consolidated
controller DB yet.

So the practical guidance is:

- use `controller_schedule_db` for current corridor/path selection work
- treat the older `trim_linearization_db` path as legacy compatibility unless
  and until it is explicitly migrated

## Controller-Side Data Conventions

Inside `controllerScheduleDB.points`:

- `Att_Trim_deg` is kept in degrees for readability
- `x0` stores attitude states in radians because that matches the reduced
  linear model basis
- `u0` is stored as:

```text
[front_collective_rpm;
 rear_collective_rpm;
 delta_f_deg;
 delta_a_deg;
 delta_e_deg;
 delta_r_deg]
```

- `trim_cmd_rad` mirrors that command vector with surface deflections in
  radians

Reduced-state order:

```text
[phi; theta; psi; u; v; w; P; Q; R]
```

Reduced-input order:

```text
[front_coll_rpm; rear_coll_rpm; delta_f; delta_a; delta_e; delta_r]
```

## How To Rebuild The Whole Stack

From MATLAB, in `eVTOL_Simulation`:

### 1. Backfill missing controller-relevant linearizations

```matlab
clear transitionTrimLinearizationBackfillOptions
Build_Transition_Trim_Linearization_Backfill
```

Default behavior:

- best-unique rows only
- exact trims only
- rear-on only
- skip rows that already have a backfill entry
- prefer stored `trimResult.linear`
- replay `trimCase` when needed
- rebuild the consolidated DBs at the end

### 2. Rebuild only the consolidated DBs

```matlab
clear transitionTrimDatabaseOptions
Build_Transition_Trim_Databases
```

### 3. Rebuild DBs with additional sources

```matlab
transitionTrimDatabaseOptions = struct( ...
    'extra_trim_sources', {'transition_trim_new_patch_latest.csv'}, ...
    'extra_linearization_index_sources', {'transition_trim_new_patch_linearization_index.csv'}, ...
    'extra_linearization_roots', {fullfile(pwd, 'workspace_plots', 'transition_trim_new_patch_linearizations')});

Build_Transition_Trim_Databases
```

## Practical Guidance For Other Agents

If you are adding a new search script:

- write to an existing canonical `*_latest.csv` if the new search is part of
  an existing family
- otherwise add the new filename explicitly through
  `transitionTrimDatabaseOptions.extra_trim_sources`
- if the new search generates linearizations, also add:
  - `extra_linearization_index_sources`
  - `extra_linearization_roots`

Do not expect the database builder to discover new files by recursively
scanning `workspace_plots`. The manifest is explicit by design.

If you are doing controller work:

- prefer points from `controller_schedule_db.mat`
- use `controllerScheduleDB.points` when you need inline matrices and initial
  conditions
- use `rebuild_trim_result_from_controller_db_point.m` when you need a
  run-ready `trimResult` from a controller DB point
- use `transition_trim_master_attempt_db.mat` for background plotting and
  provenance, not as the primary controller schedule source

## Bottom Line

The repo now has a defensible separation between:

- search provenance
- controller-ready schedule data

The critical change that made the controller DB usable was the linearization
backfill step. Before that, the exact trims existed but many did not have
matched linearizations. After backfill and rebuild, the controller DB became
dense enough to support actual path selection and scheduled-control work.
