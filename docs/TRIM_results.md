# Cruise Trim Work Results

Date: 2026-03-21

## Artifacts added

- MATLAB trim scaffold:
  - `scripts/trim/trim_cruise.m`
  - `scripts/trim/run_trim_check.m`
- Python fallback estimator (executable in this environment):
  - `scripts/trim/trim_cruise_fallback.py`
- Evidence:
  - `docs/evidence/trim_fallback_command_output.txt`
  - `docs/evidence/trim_fallback_40mps.json`
  - `docs/evidence/trim_fallback_60mps.json`
  - `docs/evidence/trim_fallback_70mps.json`
  - `docs/evidence/trim_fallback_80mps.json`
  - `docs/evidence/trim_fallback_sweep_summary.csv`

## Attempted methods

### 1) Native MATLAB/Simulink trim path (findop/trim)
Status: **Not executed in this run environment**

Reason:
- `matlab` executable unavailable (`command not found`).

Mitigation implemented:
- `trim_cruise.m` includes a `findop/operspec` attempt path plus fallback estimates.
- `run_trim_check.m` wraps execution and writes artifacts when MATLAB is available.

### 2) Fallback quasi-steady numerical search (executed)
Status: **Executed**

Modeling basis for fallback:
- Aero force/moment equations from library chart `aeroSurface`.
- Propeller thrust direction and geometry from front/rear prop chart logic.
- Component geometry and coefficients parsed from `Full_Sim_Init.m`.
- Solves nonnegative least-squares at each candidate angle of attack for:
  - `Fx ≈ 0`
  - `Fz ≈ 0`
  - `My ≈ 0`

## Quantitative results (best candidate per speed)

| Speed (m/s) | alpha (deg) | Front RPM | Rear RPM | Residual norm* |
|---:|---:|---:|---:|---:|
| 40 | 16.0 | 0.0 | 2759.4 | 25036.3 |
| 60 | 16.0 | 1680.6 | 2779.9 | 22636.0 |
| 70 | 16.0 | 2229.9 | 2788.9 | 21077.4 |
| 80 | 16.0 | 2729.6 | 2799.2 | 19279.0 |

\* Residual norm uses `[Fx, Fz, My]` components (N, N, N·m).

Result interpretation:
- No candidate met the “feasible” threshold (`residual < 500`, RPM <= 8000).
- Residuals remain very large, so a physically consistent cruise trim was **not found** under current assumptions/parameters.

## Primary blockers identified

1. **Likely actuator/parameter mismatch**
   - Validation pass shows very low thrust-to-weight at nominal 1500 RPM under current `kT` and mass.
2. **Spin direction variables missing in init**
   - `prop.Lspin_dir` and `prop.Rspin_dir` are required by model masks but not set in `Full_Sim_Init.m`.
3. **Potential units ambiguity**
   - Prop formulas use `T = kT * rpm^2`; verify whether intended variable is RPM or rad/s.
4. **No full closed-loop controller visible in model inventory**
   - Current setup appears mostly open-loop for propulsion commands, making robust trim convergence unlikely without additional constraints/control structure.

## Next concrete trim steps

1. Define spin-direction vectors in init script and verify ordering (L1..L3 / R1..R3).
2. Confirm propulsion units (`kT`, `kQ`, RPM vs rad/s) and recalibrate coefficients.
3. Add realistic RPM and tilt constraints/saturations before trim solving.
4. When MATLAB is available:
   - run `scripts/trim/run_trim_check.m`
   - wire a true operating-point problem (`operspec`) with explicit state/output constraints (`u_dot=0`, `w_dot=0`, `q_dot=0`, `q/r/p` near zero, target airspeed/altitude).
