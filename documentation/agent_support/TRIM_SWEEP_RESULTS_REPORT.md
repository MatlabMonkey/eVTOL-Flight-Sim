# Transition Trim Sweep Results Report

## Snapshot

- Project: `eVTOL-Flight-Sim`
- Workspace: `eVTOL_Simulation/`
- Report date: `2026-04-21`
- Primary objective:
  - build usable transition trim coverage from hover-side points toward the high-speed / cruise-side branch
  - preserve rear-on solutions where possible
  - generate controller-usable trim points and linearizations

This report is a handoff summary of what the trim sweeps actually achieved, where the search succeeded, where it failed, and what data products now exist.

## Executive Summary

The trim sweep campaign produced a strong low-speed bank, a nearly complete high-speed bank, and a narrower but usable rear-on bridge branch through the difficult `40–47.5 m/s` region.

The hardest part of the map was the middle transition basin:

- `30–40 m/s`
- `60–80 deg` front tilt
- front collective roughly between `1400–2000 rpm`
- rear collective roughly between `500–1500 rpm`

That region repeatedly split into competing basins:

- a high-front / low-rear branch
- a low-front / high-rear branch
- a partial rear-on branch that was the most useful for controller work

The main technical outcome is that we now have a concrete rear-on bridge sequence:

| Vinf (m/s) | Tilt (deg) | Classification | Front coll. (rpm) | Rear coll. (rpm) | Source run |
| --- | ---: | --- | ---: | ---: | --- |
| 40.0 | 67.5 | exact_trim | 1664.67 | 1485.19 | `transition_trim_mainbridge_v40_basinscan_fast` |
| 42.5 | 67.5 | quasi_trim_usable | 1635.00 | 1350.00 | `transition_trim_mainbridge_v42p5_tilt70_fast` |
| 42.5 | 72.5 | exact_trim | 1641.27 | 1472.35 | `transition_trim_mainbridge_v42p5_basinscan_fast` |
| 45.0 | 70.0 | exact_trim | 1260.99 | 1196.76 | `transition_trim_mainbridge_v45_basinscan_fast` |
| 45.0 | 77.5 | quasi_trim_usable | 1175.00 | 1050.00 | `transition_trim_mainbridge_v45_tight_fast` |
| 47.5 | 67.5 | exact_trim | 885.42 | 820.66 | `transition_trim_mainbridge_v47p5_basinscan_fast` |
| 47.5 | 70.0 | exact_trim | 908.36 | 1033.77 | `transition_trim_mainbridge_v47p5_basinscan_fast` |
| 47.5 | 72.5 | exact_trim | 918.62 | 1021.97 | `transition_trim_mainbridge_v47p5_basinscan_fast` |
| 47.5 | 75.0 | exact_trim | 926.38 | 1004.40 | `transition_trim_mainbridge_v47p5_basinscan_fast` |
| 47.5 | 77.5 | exact_trim | 921.42 | 974.33 | `transition_trim_mainbridge_highbridge_fast` |
| 47.5 | 80.0 | exact_trim | 933.27 | 962.77 | `transition_trim_mainbridge_highbridge_fast` |
| 47.5 | 82.5 | exact_trim | 948.70 | 943.98 | `transition_trim_mainbridge_highbridge_fast` |

That branch is the most important outcome of the later focused patch runs.

## Canonical Database Snapshot

### Master attempt database

Canonical file:

- `workspace_plots/transition_trim_master_attempt_db.csv`

Current contents:

| Metric | Value |
| --- | ---: |
| Total attempt rows | 4062 |
| `exact_trim` rows | 1517 |
| `quasi_trim_usable` rows | 49 |
| `near_trim_borderline` rows | 44 |
| `not_usable` rows | 2452 |
| Rear-on rows (`rear_collective_rpm > 1`) | 3793 |
| Rows with linearization available | 1326 |

Percentages over all attempt rows:

| Class | Count | Percent |
| --- | ---: | ---: |
| `exact_trim` | 1517 | 37.3% |
| `quasi_trim_usable` | 49 | 1.2% |
| `near_trim_borderline` | 44 | 1.1% |
| `not_usable` | 2452 | 60.4% |

Interpretation:

- the campaign attempted a large number of points
- most attempted rows are failures, which is expected for dense local patch searches
- the usable subset is meaningful but concentrated in certain basins

### Best-unique point set

For map coverage, the more meaningful view is the best row at each trim-point key. That collapses duplicates from repeated runs and keeps the best available classification per point.

Best-unique totals:

| Metric | Value |
| --- | ---: |
| Best-unique rows | 449 |
| Acceptable-or-better rows | 298 |
| Rear-on acceptable-or-better rows | 209 |
| Zero-rear acceptable-or-better rows | 89 |

Best-unique class counts:

| Class | Count | Percent |
| --- | ---: | ---: |
| `exact_trim` | 266 | 59.2% |
| `quasi_trim_usable` | 32 | 7.1% |
| `near_trim_borderline` | 23 | 5.1% |
| `not_usable` | 128 | 28.5% |

Interpretation:

- after collapsing repeats, the map is much healthier than the raw attempt totals suggest
- the controller-relevant subset is the `298` acceptable-or-better best-unique rows
- of those, `209` are rear-on and `89` are effectively zero-rear

### Controller schedule database

Canonical file:

- `workspace_plots/controller_schedule_db.csv`

Current contents:

| Metric | Value |
| --- | ---: |
| Total rows | 725 |
| `exact_trim` rows | 725 |
| Rows with linearization available | 725 |

Interpretation:

- the controller schedule database is stricter than the master attempt DB
- it is the curated exact-trim / controller-facing dataset
- it is already fully linearized from the builder’s point of view

## Linearization Inventory

The cleanest explicit per-point linearization inventory currently on disk is the rear-on connector set:

- `workspace_plots/transition_trim_rearon_connector_forever_linearization_index.csv`
- `workspace_plots/transition_trim_rearon_connector_forever_linearizations/`

Current inventory:

| Asset | Count |
| --- | ---: |
| Rear-on connector linearization index rows | 28 |
| Rear-on connector `.mat` linearization files | 28 |
| `trim_map_linearizations_latest.csv` rows | 2 |

Interpretation:

- the controller database can point at many linearized points
- the explicitly tracked rear-on connector folder contains `28` saved per-point linearization files
- the rest of the controller-ready linearization information is being managed through the canonical builder outputs

## Coverage by Airspeed Band

Best-unique coverage by airspeed band:

| Airspeed band (m/s) | Best-unique rows | Exact | Quasi | Borderline | Failed | Acceptable-or-better |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `0–10` | 43 | 36 | 4 | 1 | 2 | 40 |
| `10–20` | 69 | 46 | 6 | 6 | 11 | 52 |
| `20–30` | 87 | 36 | 16 | 5 | 30 | 52 |
| `30–40` | 64 | 6 | 5 | 2 | 51 | 11 |
| `40–50` | 66 | 28 | 0 | 9 | 29 | 28 |
| `50–60` | 44 | 43 | 1 | 0 | 0 | 44 |
| `60–80` | 75 | 70 | 0 | 0 | 5 | 70 |

Key conclusion:

- `30–40 m/s` was by far the hardest region
- `40–50 m/s` improved significantly after the focused basin scans
- `50–80 m/s` is now comparatively strong

## What the Sweep Methods Actually Were

The trim campaign did not use one single brute-force sweep. It evolved through several search styles:

1. Merged trim-map sweeps
- broad initial transition map generation
- useful for overall coverage but not basin-sensitive enough in the hard middle region

2. Low-speed scored sweeps
- built the hover-side and lower-speed bank
- these provided the best early anchors

3. Reference-line scored sweeps
- sampled around a hand-defined hover-to-cruise line
- useful for broad corridor exploration
- not good enough by itself once the search hit competing propeller basins

4. Guide-grid patch sweeps
- local patch runs around a narrow `(Vinf, tilt)` region
- small front/rear collective seed grids around guide centers
- used for iterative map repair

5. Single-airspeed basin scans
- fixed `Vinf`, scanned a tight tilt band
- much better for finding footholds in the hard mid-speed region

The most effective late-stage method was:

- single-airspeed basin scan
- centered in a basin suggested by nearby successful points
- canonical skip logic from the master attempt DB
- canonical good anchors from the controller schedule DB

## Important Runs and Outcomes

### Early-to-middle useful runs

| Run prefix | Total rows | Exact | Quasi | Borderline | Failed |
| --- | ---: | ---: | ---: | ---: | ---: |
| `transition_trim_lowmid_guidegrid_fast` | 50 | 20 | 5 | 3 | 22 |
| `transition_trim_reference_line_midband_scored` | 35 | 3 | 4 | 3 | 25 |
| `transition_trim_rearon_forever_scored` | 55 | 21 | 5 | 7 | 22 |
| `transition_trim_rearon_connector_forever` | 48 | 20 | 3 | 5 | 20 |

Interpretation:

- the lower-mid guide-grid run was productive
- the rear-on searches were useful, but not enough by themselves to solve the `30–45 m/s` gap cleanly

### Focused patch runs in the hard middle region

| Run prefix | Total rows | Exact | Quasi | Borderline | Failed | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `transition_trim_mainbridge_bluecircle_fast` | 25 | 0 | 0 | 0 | 25 | Full miss in the first blue-circle basin attempt |
| `transition_trim_mainbridge_bluecircle_dense` | 15 | 1 | 1 | 0 | 13 | First foothold near `30 / 62.5` |
| `transition_trim_mainbridge_midbasin_fast` | 25 | 0 | 0 | 0 | 25 | Full miss; wrong basin / repeated failures |
| `transition_trim_mainbridge_downstream_fast` | 16 | 2 | 1 | 2 | 11 | Found `37.5 / 65 exact` and `40 / 75 exact` but rear collapsed low |
| `transition_trim_mainbridge_v40_basinscan_fast` | 5 | 1 | 0 | 0 | 4 | Critical rear-on foothold at `40 / 67.5 exact` |
| `transition_trim_mainbridge_v42p5_basinscan_fast` | 5 | 1 | 0 | 3 | 1 | Critical rear-on foothold at `42.5 / 72.5 exact` |
| `transition_trim_mainbridge_v42p5_tilt70_fast` | 2 | 0 | 1 | 0 | 1 | Useful `42.5 / 67.5 quasi` |
| `transition_trim_mainbridge_v45_basinscan_fast` | 6 | 1 | 0 | 4 | 1 | Critical rear-on foothold at `45 / 70 exact` |
| `transition_trim_mainbridge_v45_tight_fast` | 4 | 0 | 1 | 2 | 1 | Tightened `45` branch around `77.5 deg` |
| `transition_trim_mainbridge_middlegap_fast` | 8 | 1 | 0 | 2 | 5 | Partial middle-gap foothold, but still inconsistent |
| `transition_trim_mainbridge_highbridge_fast` | 15 | 4 | 0 | 1 | 10 | Strong `47.5` rear-on branch |
| `transition_trim_mainbridge_v47p5_basinscan_fast` | 4 | 4 | 0 | 0 | 0 | Clean `47.5` lower-basin rear-on scan; strongest late-stage patch run |

## The Most Important Achievements

### 1. Strong low-speed bank

The low-speed side is no longer the problem. It produced dense exact coverage through the hover-side bank and into the early transition region.

Representative examples:

| Vinf (m/s) | Tilt (deg) | Classification | Front rpm | Rear rpm | Source |
| --- | ---: | --- | ---: | ---: | --- |
| 30.0 | 20.0 | exact_trim | 1627.03 | 1231.59 | `transition_trim_map_bridge_scored` |
| 35.0 | 65.0 | exact_trim | 1896.18 | 686.20 | `transition_trim_reference_line_midband_scored` |

### 2. Identification of the hard middle transition basin

The hard region was clearly:

- `30–40 m/s`
- `60–80 deg`

Problems in that region:

- repeated failures around neighboring points
- multiple propeller basins
- low-rear collapse if seeded poorly
- inability of global reference curves to stay in the right basin

### 3. Rear-on bridge branch through `40–47.5 m/s`

This is the main campaign success.

Bridge sequence:

| Vinf (m/s) | Tilt (deg) | Class | Front rpm | Rear rpm |
| --- | ---: | --- | ---: | ---: |
| 40.0 | 67.5 | exact_trim | 1664.67 | 1485.19 |
| 42.5 | 67.5 | quasi_trim_usable | 1635.00 | 1350.00 |
| 42.5 | 72.5 | exact_trim | 1641.27 | 1472.35 |
| 45.0 | 70.0 | exact_trim | 1260.99 | 1196.76 |
| 45.0 | 77.5 | quasi_trim_usable | 1175.00 | 1050.00 |
| 47.5 | 67.5 | exact_trim | 885.42 | 820.66 |
| 47.5 | 70.0 | exact_trim | 908.36 | 1033.77 |
| 47.5 | 72.5 | exact_trim | 918.62 | 1021.97 |
| 47.5 | 75.0 | exact_trim | 926.38 | 1004.40 |
| 47.5 | 77.5 | exact_trim | 921.42 | 974.33 |
| 47.5 | 80.0 | exact_trim | 933.27 | 962.77 |
| 47.5 | 82.5 | exact_trim | 948.70 | 943.98 |

Interpretation:

- this is the first genuinely coherent rear-on bridge branch discovered by the patch scans
- front collective drops substantially across the branch
- rear collective stays meaningfully on
- this branch is a credible controller-scheduling backbone through the difficult region

### 4. The low-rear branch was found, but is not the preferred controller branch

Examples:

| Vinf (m/s) | Tilt (deg) | Class | Front rpm | Rear rpm |
| --- | ---: | --- | ---: | ---: |
| 37.5 | 65.0 | exact_trim | 1889.14 | 103.66 |
| 40.0 | 75.0 | exact_trim | 1961.10 | 314.12 |

These were useful for proving the map was not empty, but they are weaker if the design goal is rear-on continuity.

## What Failed Repeatedly

The sweeps did not fail randomly. There were clear repeated failure patterns:

1. Repeated local failures in the same `(Vinf, tilt)` neighborhoods
- especially in `30–40 m/s`
- especially when the guide centers remained too close to previously failed basins

2. High-front / low-rear seed centers
- these often produced failures or low-rear exacts
- they were useful diagnostically, but not ideal for controller continuity

3. Broad guide curves
- these were too coarse in the difficult middle region
- they pushed searches into the wrong basin

4. Overly aggressive downstream jumps
- trying to jump directly from `35–37.5` up toward `45+` often skipped the actual bridge basin

## Data Products Now Available

### Canonical databases

| Purpose | File |
| --- | --- |
| Master attempt history | `workspace_plots/transition_trim_master_attempt_db.csv` |
| Controller-facing exact-trim set | `workspace_plots/controller_schedule_db.csv` |

### Primary linearization assets

| Purpose | File / folder |
| --- | --- |
| Rear-on linearization index | `workspace_plots/transition_trim_rearon_connector_forever_linearization_index.csv` |
| Rear-on per-point linearizations | `workspace_plots/transition_trim_rearon_connector_forever_linearizations/` |

### Reference docs

| Document | Purpose |
| --- | --- |
| `TRIM_DATABASES.md` | explains current trim database organization |
| `TRIM_DATABASE_BUILDER_SPEC.md` | spec for building / rebuilding canonical databases |
| `TRIM_SWEEP_METHODS.md` | explains sweep methods, script families, and search logic |

## Data Organization Appendix

The current trim data is no longer a single-folder collection of ad hoc CSVs. The intended organization is:

```text
eVTOL_Simulation/
  TRIM_DATABASES.md
  TRIM_DATABASE_BUILDER_SPEC.md
  TRIM_SWEEP_METHODS.md
  TRIM_SWEEP_RESULTS_REPORT.md
  workspace_plots/
    transition_trim_master_attempt_db.csv
    transition_trim_master_attempt_db.mat
    controller_schedule_db.csv
    controller_schedule_db.mat
    transition_trim_*_latest.csv
    transition_trim_*_latest.mat
    transition_trim_*_latest.md
    transition_trim_*_<timestamp>/
    transition_trim_rearon_connector_forever_linearization_index.csv
    transition_trim_rearon_connector_forever_linearizations/
```

Practical meaning:

- `transition_trim_*_latest.csv` files are the run-local raw results.
- `transition_trim_master_attempt_db.csv` is the canonical merged attempt history.
- `controller_schedule_db.csv` is the canonical controller-facing exact-trim set.
- `transition_trim_rearon_connector_forever_linearizations/` holds the clearest explicitly tracked per-point `.mat` linearizations.

The canonical databases are rebuilt from the run-local files. The run-local files remain the raw evidence of what each sweep did.

## Source Inventory Notes

The canonical attempt database currently includes a mix of broad sweeps and focused patch runs. The largest source prefixes by row count are:

| Source run prefix | Rows |
| --- | ---: |
| `transition_trim_map_merged` | 1671 |
| `transition_trim_map` | 991 |
| `transition_trim_map_low_speed` | 680 |
| `rear_on_cruise_anchor` | 96 |
| `transition_trim_lowmid_guidegrid_scored` | 89 |
| `transition_trim_reference_line_scored` | 64 |
| `transition_trim_rearon_forever_scored` | 55 |
| `transition_trim_lowmid_guidegrid_fast` | 50 |
| `transition_trim_rearon_connector_forever` | 48 |
| `transition_trim_map_low_speed_scored` | 45 |

This matters for interpretation:

- the canonical database is dominated numerically by the early broad sweeps
- the technically important bridge results came from much smaller late-stage patch runs
- therefore, row count alone is not a measure of usefulness

## What the Report Writer Should Emphasize

1. The low-speed and high-speed regions are not the story.
- Those are relatively well-covered.

2. The real work was in the middle transition basin.
- That is where most failures occurred.
- That is where the search methodology had to become local and basin-aware.

3. The important achievement is not “we found one trim point.”
- The important achievement is that a rear-on bridge branch was found across multiple neighboring airspeeds and tilts.

4. The bridge branch is narrow.
- It exists, but it is not yet a broad robust surface.
- That is the correct tone for the report: meaningful success, but not complete closure.

## Recommended Next Steps

1. Rebuild the canonical databases after the latest runs if needed.
- `Build_Transition_Trim_Databases`

2. Run linearization backfill for any newly accepted controller-relevant points.
- `Build_Transition_Trim_Linearization_Backfill`

3. For controller design, prioritize the rear-on branch:
- `40 / 67.5 exact`
- `42.5 / 72.5 exact`
- `45 / 70 exact`
- `47.5 / 67.5–82.5 exact`

4. Treat the low-rear exacts as secondary.
- useful for map understanding
- less useful for a rear-on transition controller

5. If more trim work is done, use local basin scans instead of broad corridor sweeps in the hard middle region.

## Bottom Line

The trim sweep campaign did not produce a uniformly filled transition map. It did produce:

- a large canonical attempt database (`4062` attempted rows)
- a meaningful best-unique map (`449` best-unique points)
- a controller schedule database (`725` exact linearized rows)
- and most importantly, a concrete rear-on bridge branch through the hardest part of the map

The strongest new controller-relevant bridge points are:

- `40 / 67.5 exact`
- `42.5 / 72.5 exact`
- `45 / 70 exact`
- `47.5 / 67.5`, `70`, `72.5`, `75`, `77.5`, `80`, `82.5 exact`

That is the main technical achievement of the sweep effort.
