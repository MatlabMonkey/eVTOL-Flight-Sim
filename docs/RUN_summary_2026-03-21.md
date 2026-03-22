# Run Summary — 2026-03-21

## Completed vs partial

### ✅ Completed

1. **Current-state repo report**
   - Created: `docs/REPORT_state_of_sim_2026-03-21.md`
   - Includes architecture, assumptions, limitations, and evidence references.

2. **Physical validity/accuracy pass (static)**
   - Added checker: `scripts/validation/check_physical_consistency.py`
   - Added plan/status doc: `docs/VALIDATION_plan.md`
   - Produced evidence outputs under `docs/evidence/validation_*`.

3. **Trim workflow scaffolding + executed fallback search**
   - Added MATLAB scaffold:
     - `scripts/trim/trim_cruise.m`
     - `scripts/trim/run_trim_check.m`
   - Added executable fallback solver:
     - `scripts/trim/trim_cruise_fallback.py`
   - Documented outcomes: `docs/TRIM_results.md`

4. **Transition metrics scaffolding + sample quantitative run**
   - Added MATLAB scaffold:
     - `scripts/metrics/compute_transition_metrics.m`
     - `scripts/metrics/example_metrics_run.m`
   - Added Python executable helper and sample run:
     - `scripts/metrics/compute_transition_metrics.py`
     - `scripts/metrics/example_metrics_run.py`
   - Review/recommendation doc: `docs/TRANSITION_metrics_review.md`

5. **Reproducibility logging**
   - Created `docs/progress_log.md`
   - Captured command outputs in `docs/evidence/`.

### ⚠️ Partial / blocked

1. **MATLAB-dependent simulation execution** (smoke + native trim)
   - Blocked in this run environment: `matlab` command unavailable.
   - Smoke test command captured failure (`command not found`).

2. **Cruise trim convergence (full 6-DoF consistency)**
   - Fallback search executed, but no low-residual feasible point found with current assumptions/parameters.

## Quantitative results produced

### Physical validation
- Mass: **3040 kg**
- CG: **[-0.0474, 0, -0.2513] m**
- Principal inertia moments: **[14400.1, 23041.2, 36228.5] kg·m²**
- Check summary: **PASS 10 / FAIL 2 / WARN 2**
- Failures:
  - `prop.Lspin_dir` undefined in init
  - `prop.Rspin_dir` undefined in init
- Thrust-to-weight diagnostic at 1500 RPM:
  - 12-motor thrust: **3240 N**
  - Weight: **29822 N**
  - T/W ≈ **0.109**

### Trim fallback sweep (best per speed)
- 40 m/s: alpha 16°, front 0 rpm, rear 2759 rpm, residual 25036
- 60 m/s: alpha 16°, front 1681 rpm, rear 2780 rpm, residual 22636
- 70 m/s: alpha 16°, front 2230 rpm, rear 2789 rpm, residual 21077
- 80 m/s: alpha 16°, front 2730 rpm, rear 2799 rpm, residual 19279

(Residual combines `[Fx, Fz, My]` mismatch.)

### Transition metrics sample
- energy_proxy: **191.999**
- accel_rms: **3.500 m/s²**
- jerk_rms: **1.155 m/s³**
- control_smoothness: **0.273**
- transition_time: **9.6 s**
- settling_time: **9.6 s**
- overshoot: **0 m/s**

## Key blockers

1. MATLAB not present in execution environment, preventing direct Simulink runs.
2. Propeller spin-direction variables required by model masks are not initialized.
3. Current propulsion/weight balance and trim residuals suggest parameter/control structure mismatch.
4. No explicit PID/saturation blocks detected in scanned model XML.

## Next concrete steps for next run

1. In `Full_Sim_Init.m`, define and document rotor spin vectors:
   - `prop.Lspin_dir`, `prop.Rspin_dir`.
2. Verify thrust/torque coefficient units and scaling (RPM vs rad/s) and recalibrate.
3. Add actuator saturations/limits (RPM and tilt) explicitly.
4. Run MATLAB smoke test and native trim scripts when MATLAB environment is available:
   - `run('scripts/smoke_test.m')`
   - `run('scripts/trim/run_trim_check.m')`
5. Promote transition metrics from synthetic data to real simulation logs and establish baseline score thresholds.
