# AME 532 Progress Report Facts

Date: 2026-03-24

## Proposal goals from Progress Report 1

| Original proposal goal | Current status on 2026-03-24 |
| --- | --- |
| Produce an accurate 6DOF simulation of a VTOL aircraft using contemporary VTOL aircraft designs. | A running 6-DoF Simulink model now exists for a lift+cruise eVTOL based on an Archer Midnight-like configuration. The aircraft geometry, mass properties, aerodynamic surfaces, rotor layout, and visualization have all been updated. |
| Create different scenarios for which a flight transition can be analyzed and evaluated. | `scenario_def.m` now centralizes baseline scenario setup, and the external-force validation workflow was updated with six cases tied to the current aircraft properties. |
| Create a framework for comparing the effectiveness of different flight transition control methods or aircraft configurations for their performance in the transition region. | A transition-metrics scaffold exists, a simple PD controller scaffold is now integrated into `Brown_Full_Sim.slx`, and a trim helper now produces first-pass actuator estimates. The framework exists, but the controller is still untuned and a fully converged trim point has not yet been found. |

## Verified current aircraft properties

Source: `aircraft_def.m` evaluated in MATLAB on 2026-03-24.

| Property | Value |
| --- | --- |
| Vehicle type | Lift+cruise eVTOL, Archer Midnight-inspired |
| Total modeled mass | 3040 kg |
| CG | `[0.0002, 0.0000, -0.2089] m` |
| Inertia matrix | `Jxx = 2.184e4`, `Jyy = 1.418e4`, `Jzz = 3.474e4`, `Jxz = -5.03e2 kg·m^2` |
| Fuselage dimensions | `7.50 x 2.00 x 1.60 m` |
| Wing layout | Two half-wings, each `1.50 x 6.25 x 0.20 m` |
| Total modeled wing area | 18.75 `m^2` |
| Tail layout | Two V-tail halves, each `1.40 x 2.00 x 0.08 m` at `±45 deg` |
| Total modeled tail area | 5.60 `m^2` |
| Rotor layout | 12 total rotors: 6 front tilting, 6 rear fixed |
| Propeller coefficients | `kT = 1.2e-4`, `kQ = 1.5e-5` |
| Wing lift-curve slope | `CLa = 4.7671 / rad` |
| Tail lift-curve slope | `CLa = 2.2028 / rad` |

## Control surfaces now represented in the repo

| Surface | Current simplified implementation |
| --- | --- |
| Ruddervators | One ruddervator per V-tail half in the sim and renderer |
| Flaperons | One flaperon per wing half in the sim metadata and renderer |
| Maximum deflection | `±25 deg` for both simplified surface types |
| Ruddervator effectiveness | `tau = 0.45` |
| Flaperon effectiveness | `tau = 0.55` |

## Verified control-surface expectations

Source basis: `docs/CONTROL_SURFACE_RESEARCH_2026-03-23.md`

| Quantity | First-pass estimate |
| --- | --- |
| `CM_delta_e` | `-0.487 / rad` |
| `CY_delta_r` | `-0.209 / rad` |
| `CN_delta_r` | `+0.059 / rad` |
| `CL_delta_r` | `-0.012 / rad` |

## Model and script changes completed

- `Full_Sim_Init.m` was converted into a wrapper around `aircraft_def.m` and `scenario_def.m`.
- `render_aircraft.m` was expanded to show:
  - thrust vectors,
  - aerodynamic surface normals,
  - control-surface normals,
  - hover / transition / cruise presets,
  - ruddervator and flaperon deflection states.
- `Brown_Full_Sim.slx` and `TESTER_Brown_Full_Sim.slx` were patched so the gravity input is a vector `[0; 0; g]` instead of a scalar.
- `Brown_Full_Sim.slx` now contains a gated `Simple PD Controller` subsystem.
- `TESTER_external_forces_runner.m` was updated to use the new aircraft data and corrected gravity convention.
- `scripts/trim/trim_cruise.m` was patched so it can bootstrap from the repo root correctly.

## Verification completed on 2026-03-24

| Check | Result |
| --- | --- |
| `scripts/smoke_test.m` | PASS |
| `Full_Sim_Init.m` + `sim('Brown_Full_Sim')` | PASS |
| `scripts/trim/run_trim_check.m` | PASS as executable workflow; method returned `fallback` |
| `scripts/metrics/example_metrics_run.m` | PASS |
| `TESTER_external_forces_runner` | PASS on updated cases after gravity/workspace fixes |

## Quantitative results worth citing

### Trim helper

From `scripts/trim/run_trim_check.m` at `CruiseSpeed = 70 m/s`, `TiltDeg = 90 deg`:

- Method returned: `fallback`
- Estimated front RPM for cruise drag: `1063.6`
- Estimated rear RPM for full-weight support: `6435.8`

Interpretation:
- The trim workflow now runs cleanly.
- These are first-pass actuator estimates, not a fully converged 6-DoF trim solution.

### Transition metrics scaffold

From `scripts/metrics/example_metrics_run.m`:

- `transition_time_s = 9.60`
- `settling_time_s = 9.60`
- `overshoot_mps = 0.00`
- `jerk_rms = 1.155 m/s^3`
- `control_smoothness = 0.273`

Recommended metrics to report going forward:

- `transition_time_s`
- `jerk_rms`

### Updated cruise-response test outputs

From the updated tester runs:

| Case | Final result at 10 s |
| --- | --- |
| Case 4 - Roll Damping | `omega = [0.0009, 0.0655, 0.0006] rad/s`, `eul = [2.63, 31.71, 1.58] deg` |
| Case 5 - Cruise Beta to Yaw | `omega = [0.0094, 0.0658, -0.0481] rad/s`, `eul = [1.14, 31.78, 5.87] deg` |
| Case 6 - Unpowered Glide Response | `omega = [-0.0000, 0.1358, -0.0000] rad/s`, `eul = [-180.00, 84.60, -180.00] deg` |

Interpretation:

- The cruise-response cases are now tied to plausible aircraft-scale speeds instead of the older placeholder values.
- The vehicle responds consistently to the updated aerodynamic and mass model.
- These cases are still validation/sanity checks, not evidence of a tuned transition controller.

## Controller status to state honestly

- A simple PD controller scaffold is now integrated into `Brown_Full_Sim.slx`.
- It is enabled through the workspace variable `controller_enable`.
- It uses Euler-angle error and body-rate feedback to generate:
  - left/right wing local flaperon commands,
  - left/right tail local ruddervator commands,
  - a motor RPM command vector.
- The controller is still a placeholder and is not yet tuned for trim or transition performance.
- A controller-enabled simulation now executes, but it does not hold a trimmed condition over the 10 s run.
