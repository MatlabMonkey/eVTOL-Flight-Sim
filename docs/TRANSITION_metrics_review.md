# Transition Metrics Review (Hover ↔ Cruise)

Date: 2026-03-21

## Artifacts added

- MATLAB scaffold:
  - `scripts/metrics/compute_transition_metrics.m`
  - `scripts/metrics/example_metrics_run.m`
- Python executable helper (used for this run's evidence):
  - `scripts/metrics/compute_transition_metrics.py`
  - `scripts/metrics/example_metrics_run.py`
- Evidence:
  - `docs/evidence/metrics_example_command_output.txt`
  - `docs/evidence/metrics_example_latest.json`
  - `docs/evidence/metrics_example_latest.csv`

## Candidate metric families evaluated

1. **Energy usage / loss proxy**
   - Implemented as `∫ sum(abs(control)) dt`.
   - Pro: available from command logs even without detailed powertrain model.
   - Con: not physical power unless mapped to actuator/power model.

2. **Ride quality: acceleration / jerk**
   - `accel_rms`, `jerk_rms` from translational acceleration/jerk.
   - Pro: directly reflects passenger comfort and structural excitation.
   - Con: sensitive to sensor noise; should be filtered in real logs.

3. **Control smoothness / aggressiveness**
   - `∫ ||du/dt||^2 dt` over actuator command vector.
   - Pro: discourages chattering/high-frequency control action.
   - Con: requires consistent actuator scaling.

4. **Transition timing + tracking quality**
   - start time, transition time, settling time, overshoot (speed target based).
   - Pro: directly tied to mission usability and handling quality.
   - Con: requires robust target/band definitions.

## Sample output (synthetic example run)

From `python3 scripts/metrics/example_metrics_run.py`:

- `energy_proxy` = **191.999**
- `accel_rms` = **3.500 m/s²**
- `jerk_rms` = **1.155 m/s³**
- `control_smoothness` = **0.273**
- `transition_start_s` = **5.65 s**
- `transition_time_s` = **9.60 s**
- `overshoot_mps` = **0.00 m/s**
- `settling_time_s` = **9.60 s**

## Recommended metric set for this project

For near-term iteration, use this **minimum decision set**:

1. `transition_time_s` (primary mission KPI)
2. `settling_time_s` and `overshoot_mps` (stability/handling)
3. `jerk_rms` (comfort + structural smoothness)
4. `control_smoothness` (actuator friendliness)
5. `energy_proxy` (efficiency trend, until true power model is connected)

Recommended aggregate score (optional):

\[
J = w_t T_{trans} + w_s T_{settle} + w_o O_{speed} + w_j J_{rms} + w_u U_{smooth} + w_e E_{proxy}
\]

with normalization by baseline-case values before weighting.

## Next implementation upgrades

1. Replace `energy_proxy` with physically grounded electric power estimate:
   - per motor: `P_i = Q_i * ω_i` or ESC/battery model output.
2. Add mode-aware transition boundaries:
   - detect start/end from tilt command + speed + climb-rate conditions.
3. Add separate hover→cruise and cruise→hover scorecards.
4. Persist metrics per simulation run to CSV for trend plots/regression checks.
