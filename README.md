# eVTOL-Flight-Sim

Simulink-based eVTOL flight simulation workspace containing full-system models, initialization scripts, and FlightGear integration assets.

## Project overview

This repository provides a Brown eVTOL simulation stack centered around Simulink models:

- full vehicle simulation models (`Brown_Full_Sim*.slx`)
- controls library model (`Brown_Flight_Controls_lib.slx`)
- initialization/configuration script (`Full_Sim_Init.m`)
- FlightGear support content (`Flight_Gear/`)
- tester/reference artifacts (`TESTER_*` and documentation PDFs/MLX)

The goal is to enable repeatable simulation setup and quick validation of model health before deeper controls or dynamics work.

## MATLAB/toolbox assumptions

Tested/expected environment assumptions:

- MATLAB with Simulink installed
- Ability to open/compile `.slx` models
- Script execution permissions for `Full_Sim_Init.m`

Commonly required for model execution (depending on local setup):

- Simulink
- Aerospace/controls-related toolboxes used by the model
- FlightGear (optional; only for external visualization workflows)

If your installation is missing a required product, MATLAB will report the missing dependency at load/update/compile time.

## Quick start

1. Clone and enter the repository:
   ```bash
   git clone <repo-url>
   cd eVTOL-Flight-Sim
   ```
2. Open MATLAB in this repository root.
3. Run initialization:
   ```matlab
   Full_Sim_Init
   ```
4. Open your target model (for example):
   ```matlab
   open_system('Brown_Full_Sim')
   ```

## Smoke test (headless-friendly)

A minimal non-GUI sanity check is provided:

```matlab
run('scripts/smoke_test.m')
```

The smoke test validates:

- expected key files are present
- `Full_Sim_Init.m` executes
- a representative model (`Brown_Full_Sim`) can be loaded and updated/compiled headlessly

It prints explicit markers:

- `SMOKE_TEST: PASS`
- `SMOKE_TEST: FAIL`

Use this as a pre-commit or pre-PR confidence check.

## Repository structure

```text
.
├── README.md
├── CHANGELOG.md
├── Full_Sim_Init.m
├── Brown_Full_Sim.slx
├── Brown_Full_Sim_FlightGear.slx
├── Brown_Flight_Controls_lib.slx
├── TESTER_Brown_Full_Sim.slx
├── TESTER_external_forces.mlx
├── BROWN_TESTER_external_forces.pdf
├── render_aircraft.m
├── scripts/
│   └── smoke_test.m
└── Flight_Gear/
    └── ...
```

## Notes

- Prefer running the smoke test before modifying controls/dynamics content.
- Keep generated artifacts out of git (see `.gitignore`).
