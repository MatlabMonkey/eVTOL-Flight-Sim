# Trim Database Builder Spec

Last reviewed: 2026-04-25 20:48 PDT

Status: historical implementation spec. The current user-facing workflow is in
`TRIM_DATABASES.md`; this file remains as deeper background for the builder
schema and old source normalization rules.

This document is the build spec for the script that consolidates trim
search results into clean databases under `eVTOL_Simulation/databases/`
for:

1. search / audit work
2. controller design work

This is written as a handoff for another agent. The goal is to remove the
current ambiguity about which CSVs matter, which ones are junk, and where the
linearizations actually live.

## Why This Exists

The current repo state is good for exploration but not good for control
design. Right now:

- every runner writes its own `*_latest.csv`, `*_latest.mat`, and
  `*_latest.md`
- there is not one master trim database
- there is not one controller schedule database
- the shared global attempt DB exists, but it is not yet the clean source of
  truth for controller design
- linearizations exist, but they are not assembled into one controller-ready
  database

We need one builder script that reads the known sources, normalizes them, and
exports two clean products:

1. `master_attempt_db`
2. `controller_schedule`

## Required Outputs

The builder must produce these files in:

- `/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/eVTOL_Simulation/databases/`

Required outputs:

1. `trim_attempts.mat`
2. `trim_attempts.csv`
3. `trim_attempts.md`

4. `controller_schedule.mat`
5. `controller_schedule.csv`
6. `controller_schedule.md`

Optional diagnostic outputs, only when explicitly requested:

7. `controller_schedule_missing_linearizations.csv`
8. `controller_schedule_build_log.md`

## Database Definitions

### 1. Master Attempt DB

This is the full audit trail of attempts.

It should include:
- exact trims
- acceptable / quasi-usable trims
- borderline trims
- failed trims

It should keep the source run identity.

This DB is for:
- avoiding repeated search work
- plotting all attempted regions
- debugging where search basins are failing

It is **not** the controller-ready schedule.

### 2. Controller Schedule DB

This is the clean design dataset.

It should contain only points that are candidates for controller scheduling.

Default filter:
- keep `classification == exact_trim`
- optionally also keep `acceptable == 1`
- exclude rows with `rear_collective_rpm <= 1`

This DB is for:
- LQR schedule generation
- LQG schedule generation
- gain interpolation / blending
- trim command scheduling

For controller work, this DB should inline the linearization data in the MAT
output so the controller code does not need to chase separate files point by
point.

## Explicit Source List

Use an explicit list. Do **not** do blind recursive crawling over
`workspace_plots/`.

`workspace_plots/` is only a legacy/import/debug source location here. It is
not the durable database output location.

### Include: trim-result CSV sources

These should be read into the builder.

#### Broad map / legacy map sources

- `transition_trim_map_latest.csv`
- `transition_trim_map_low_speed_latest.csv`
- `transition_trim_map_low_speed_scored_latest.csv`
- `transition_trim_map_bridge_scored_latest.csv`
- `transition_trim_map_merged_latest.csv`

#### Reference-line and corridor searches

- `transition_trim_reference_line_scored_latest.csv`
- `transition_trim_reference_line_midband_scored_latest.csv`

#### Lower-mid / left / upper-mid patch searches

- `transition_trim_lowmid_guidegrid_scored_latest.csv`
- `transition_trim_lowmid_guidegrid_fast_latest.csv`
- `transition_trim_leftbridge_fast_latest.csv`
- `transition_trim_uppermid_bridge_fast_latest.csv`

#### Main-bridge patch searches

- `transition_trim_mainbridge_fast_latest.csv`
- `transition_trim_mainbridge_rescue_fast_latest.csv`
- `transition_trim_mainbridge_handoff_fast_latest.csv`
- `transition_trim_mainbridge_bluecircle_fast_latest.csv`
- `transition_trim_mainbridge_bluecircle_dense_latest.csv`
- `transition_trim_mainbridge_midbasin_fast_latest.csv`

#### Path and branch searches

- `transition_trim_path_scored_latest.csv`
- `transition_trim_path_scored_branch30v20_latest.csv`
- `transition_trim_path_scored_branch30v25_latest.csv`

#### Rear-on search summaries

- `transition_trim_rearon_overnight_scored_latest.csv`
- `transition_trim_rearon_forever_scored_latest.csv`
- `transition_trim_rearon_connector_forever_latest.csv`

#### Cruise-anchor search summary

- `rear_on_cruise_anchor_latest.csv`

### Include: linearization index sources

These should be read to attach actual linearized models to controller
candidates.

Primary source:
- `transition_trim_rearon_connector_forever_linearization_index.csv`

Optional secondary sources, if they are present and clean:
- `transition_trim_rearon_connector_forever_smoke_linearization_index.csv`
- `transition_trim_rearon_connector_forever_smoke2_linearization_index.csv`
- `transition_trim_rearon_connector_forever_smoke3_linearization_index.csv`

These smoke sources are optional and should default to **excluded** unless
the builder has an explicit flag to include them.

### Include: linearization file roots

Primary linearization directory:
- `transition_trim_rearon_connector_forever_linearizations/`

Optional smoke linearization roots, default excluded:
- `transition_trim_rearon_connector_forever_smoke_linearizations/`
- `transition_trim_rearon_connector_forever_smoke2_linearizations/`
- `transition_trim_rearon_connector_forever_smoke3_linearizations/`

### Exclude: non-source files and folders

Do **not** treat these as trim databases:

- any `*_visuals_*` folders
- any plot image folders
- any `*_launcher.log`
- any `*_live.log`
- any `*_nohup.out`
- `transition_trim_global_attempt_db_latest.csv`
- `transition_trim_global_attempt_db_latest.mat`
- `transition_trim_global_attempt_db_latest.md`
- `trim_map_linearizations_latest.csv` as a primary controller source
- `trim_map_linearizations_latest.mat` as a primary controller source

Reason:
- visuals and logs are not raw trim data
- the global attempt DB is currently a search cache, not a clean source of
  truth
- `trim_map_linearizations_latest.*` is useful summary material, but it is
  not the best direct source for a controller schedule because the rear-on
  connector linearization index already points to actual `.mat` files

### Exclude by content

The builder should support content-level filters.

Default controller-schedule exclusions:
- `rear_collective_rpm <= 1`
- obvious smoke-test rows if they come only from smoke sources
- rows missing both trim-state info and linearization references

The master attempt DB should **not** remove zero-rear rows. It should flag
them instead.

Suggested flag:
- `rear_on_ok = rear_collective_rpm > 1`

## Source Schema Reality

The builder must normalize mixed schemas.

There are at least three real classes of source CSVs:

### A. Newer scored / fast files with `key`

Example fields:
- `key`
- `name`
- `tilt_deg`
- `vinf_mps`
- `family`
- `seed_name`
- `success`
- `acceptable`
- `classification`
- `score`
- `max_normalized`
- `worst_component`
- `worst_component_normalized`
- `front_collective_rpm`
- `rear_collective_rpm`
- `delta_f_deg`
- `delta_a_deg`
- `delta_e_deg`
- `delta_r_deg`
- `theta_deg`
- `u_mps`
- `w_mps`
- `alpha_deg`
- `termination_string`

### B. Scored / fast files without `key`

Same as above, but no `key` column.

The builder must synthesize a canonical key, for example:
- `tilt_<tilt>__vinf_<vinf>__family_<family>__source_<source>`

or, if a run-level attempt key is needed:
- `tilt_<tilt>__vinf_<vinf>__family_<family>__seed_<seed_name>__run_<source_run>`

### C. Older map summary files

Example fields:
- `name`
- `phase`
- `family`
- `tilt_deg`
- `vinf_mps`
- `rear_fixed_rpm`
- `success`
- `termination`
- `max_state_residual`
- `attempt_count`
- `front_collective_rpm`
- `rear_collective_rpm`
- `delta_f_deg`
- `delta_e_deg`
- `theta_deg`
- `u_mps`
- `w_mps`
- `alpha_deg`

These lack `acceptable`, `classification`, `score`, and `seed_name`.
The builder must fill missing normalized fields with defaults.

## Required Normalized Master Schema

The master attempt DB should normalize every source row into one schema.

Required columns:
- `source_file`
- `source_kind`
- `source_run_prefix`
- `name`
- `key`
- `tilt_deg`
- `vinf_mps`
- `family`
- `seed_name`
- `phase`
- `rear_fixed_rpm`
- `success`
- `acceptable`
- `classification`
- `score`
- `max_normalized`
- `worst_component`
- `worst_component_normalized`
- `front_collective_rpm`
- `rear_collective_rpm`
- `delta_f_deg`
- `delta_a_deg`
- `delta_e_deg`
- `delta_r_deg`
- `theta_deg`
- `u_mps`
- `w_mps`
- `alpha_deg`
- `termination_string`
- `max_state_residual`
- `attempt_count`
- `rear_on_ok`
- `linearization_available`
- `linearization_index_source`
- `linearization_latest_file`

If a field does not exist in the source, populate it with:
- `NaN` for numeric
- `""` for text
- `false` for logical

## Deduping Rules

The builder must support two views:

1. `master_attempt_db_all_rows`
2. `master_attempt_db_best_unique_points`

The all-rows version keeps every source row.

The best-unique version groups by at least:
- `tilt_deg`
- `vinf_mps`
- `family`
- optionally `rear_fixed_rpm` when applicable

Ranking order:
1. exact trim
2. acceptable / quasi usable
3. near trim borderline
4. failed / not usable
5. lower `score`
6. lower `max_normalized`

Do **not** collapse unlike families into one row for the full audit table.

## Controller Schedule DB Requirements

The controller schedule DB should be built from the normalized master DB, but
with strict filtering.

Default inclusion:
- `classification == exact_trim`
- OR `acceptable == true` if a config flag allows quasi trims
- `rear_on_ok == true`

Recommended default:
- exact trims only
- plus a separate optional switch for acceptable points

Each controller point should include:
- `key`
- `name`
- `tilt_deg`
- `vinf_mps`
- `family`
- `score`
- `classification`
- `front_collective_rpm`
- `rear_collective_rpm`
- `delta_f_deg`
- `delta_a_deg`
- `delta_e_deg`
- `delta_r_deg`
- `theta_deg`
- `u_mps`
- `w_mps`
- `alpha_deg`
- `linearization_available`
- `linearization_latest_file`

And in the MAT output, if a linearization exists, inline:
- `A_9`
- `B_9`
- `C_9`
- `D_9`
- `state_names_9`
- `input_names_9`
- `output_names_9`
- optionally `A_full`
- optionally `B_full`
- optionally `C_full`
- optionally `D_full`
- `full_state_names`
- `full_input_names`
- `full_output_names`

Also include trim operating point data:
- `Att_Trim_deg`
- `Vel_B_BA_Trim`
- `trimCase` if available
- `x0`
- `u0`

### Required `x0`

For the reduced 9-state controller schedule, build:
- `x0 = [phi; theta; psi; u; v; w; P; Q; R]`

Use:
- `Att_Trim_deg`
- `Vel_B_BA_Trim`
- steady rates = zero unless explicitly available

### Required `u0`

Build:
- `u0 = [front_collective_rpm; rear_collective_rpm; delta_f_deg; delta_a_deg; delta_e_deg; delta_r_deg]`

Use the same input ordering as:
- `input_names_9 = ["front_coll_rpm"; "rear_coll_rpm"; "delta_f"; "delta_a"; "delta_e"; "delta_r"]`

## Linearization Source Details

The main saved per-point linearization file currently contains:
- `linearizationPoint`

Inside `linearizationPoint.linear`, the useful saved fields are:
- `A_full`
- `B_full`
- `C_full`
- `D_full`
- `full_state_names`
- `full_input_names`
- `full_output_names`
- `A_9`
- `B_9`
- `C_9`
- `D_9`
- `state_names_9`
- `input_names_9`
- `output_names_9`

Observed current reduced model:
- `A_9` is `9x9`
- `B_9` is `9x6`
- `C_9` is identity
- `D_9` is zero
- states:
  - `phi, theta, psi, u, v, w, P, Q, R`
- inputs:
  - `front_coll_rpm, rear_coll_rpm, delta_f, delta_a, delta_e, delta_r`

## Builder Behavior Requirements

The builder must:

1. read only the explicit source lists above
2. normalize the schemas
3. add source metadata
4. create the all-rows master DB
5. create the best-unique master DB
6. merge available linearization indices
7. attach linearization file references to matching points
8. inline linearization matrices into the controller MAT DB when available
9. mark rows missing linearizations
10. write a missing-linearization report

## Matching Rules For Linearizations

Primary match key:
- `key` when both sides have it

Fallback match:
- `tilt_deg`
- `vinf_mps`
- `family`
- `front_collective_rpm`
- `rear_collective_rpm`

Use tolerances for numeric fallback matching:
- tilt tolerance: `1e-6` or exact text-normalized decimal label
- vinf tolerance: `1e-6`
- RPM tolerance: small, e.g. `1e-3`

If no linearization matches:
- keep the controller candidate row only if the builder is configured to
  allow trim-only candidates
- otherwise send it to `controller_schedule_missing_linearizations.csv`

## Zero-Rear Policy

This is important.

For the controller schedule DB:
- default to excluding rows with `rear_collective_rpm <= 1`

For the master attempt DB:
- keep them
- add `rear_on_ok = false`

## Do Not Include These By Default

Default-exclude these files:

- `rear_on_cruise_anchor_smoke_latest.*`
- `rear_on_cruise_anchor_smoke2_latest.*`
- `rear_on_cruise_anchor_smoke3_latest.*`
- `transition_trim_rearon_connector_forever_smoke*_latest.*`
- `transition_trim_reference_line_scored_smoketest*_latest.*`
- any `*_visuals_*` folder outputs
- any plot PNGs
- any launcher/live/nohup logs

The builder may support an `include_smoke_sources` option, but it should
default to `false`.

## Recommended Script Name

Use a single top-level builder script:

- `/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/eVTOL_Simulation/TrimDB_Build.m`

Recommended helper structure:
- `build_transition_trim_source_manifest.m`
- `normalize_transition_trim_table.m`
- `merge_transition_trim_linearizations.m`
- `TrimDB_Build.m`

One script is fine, but those separations are the right internal tasks.

## Acceptance Criteria

The build is complete when:

1. `trim_attempts.csv` exists and contains all intended
   source rows
2. `controller_schedule.mat` exists and contains only controller
   candidates
3. controller rows with available linearizations have inline `A/B/C/D`
4. controller rows without linearizations are explicitly reported
5. zero-rear rows are excluded from the controller DB by default
6. the build log clearly lists:
   - included sources
   - excluded sources
   - total rows by source
   - total controller candidates
   - total linearized controller candidates

## Recommended First Validation Checks

After building, validate:

1. count rows from each source file
2. verify no smoke-test files were imported unless explicitly enabled
3. verify at least one known point loads correctly from:
   - `transition_trim_rearon_connector_forever_linearization_index.csv`
4. verify `controller_schedule.mat` contains:
   - one point with `A_9`, `B_9`, `C_9`, `D_9`
   - `x0`
   - `u0`
5. verify the controller DB has no rows with `rear_collective_rpm <= 1`
