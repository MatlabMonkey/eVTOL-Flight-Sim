# Changelog

All notable changes to this repository are documented in this file.

## [Unreleased]

## [2026-03-18] - Phase 1 collaboration/hygiene cleanup

### Added
- Root `README.md` with:
  - project overview
  - MATLAB/toolbox assumptions
  - quick start workflow
  - smoke test usage
  - repository structure summary
- `scripts/smoke_test.m` headless-friendly sanity script with explicit PASS/FAIL markers.
- `CHANGELOG.md` baseline and cleanup tracking.

### Changed
- Expanded `.gitignore` with macOS and common MATLAB/Simulink generated artifacts.

### Removed
- Tracked `.DS_Store` from version control.

## Baseline

Repository baseline includes core eVTOL simulation assets:
- Simulink models (`Brown_Full_Sim*.slx`, controls library)
- initialization script (`Full_Sim_Init.m`)
- FlightGear integration directory (`Flight_Gear/`)
- tester/reference artifacts (`TESTER_*`, PDF/MLX)
