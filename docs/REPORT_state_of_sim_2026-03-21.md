# eVTOL Flight Sim — Current State Report

Date: 2026-03-21

## 1) Executive snapshot

Repository is a Simulink-centric eVTOL simulation workspace with:
- one main full-vehicle model (`Brown_Full_Sim.slx`),
- one tester variant (`TESTER_Brown_Full_Sim.slx`),
- one controls/physics library (`Brown_Flight_Controls_lib.slx`),
- one monolithic initialization script (`Full_Sim_Init.m`) that computes mass properties, builds aero/prop structs, sets test cases, and renders geometry.

This run added reproducible static-analysis tooling and documentation for:
- physical consistency checks,
- cruise trim progress (including fallback numerical search),
- transition metrics scaffold and recommended KPI set.

## 2) Repository structure (current)

Top-level key assets:
- `Full_Sim_Init.m` — parameter setup + mass/inertia + test-case manager + visualization
- `Brown_Full_Sim.slx` — main 6-DoF integrated model
- `TESTER_Brown_Full_Sim.slx` — tester variant
- `Brown_Flight_Controls_lib.slx` — reusable dynamics/kinematics/aero/propeller blocks
- `scripts/smoke_test.m` — smoke sanity script
- `Flight_Gear/` — FlightGear integration utilities/artifacts

Newly added in this run:
- `scripts/common/evtol_model_parser.py`
- `scripts/validation/check_physical_consistency.py`
- `scripts/trim/trim_cruise.m`
- `scripts/trim/run_trim_check.m`
- `scripts/trim/trim_cruise_fallback.py`
- `scripts/metrics/compute_transition_metrics.m`
- `scripts/metrics/example_metrics_run.m`
- `scripts/metrics/compute_transition_metrics.py`
- `scripts/metrics/example_metrics_run.py`
- `docs/*.md` (report/validation/trim/metrics/progress/summary)

(Full snapshot: `docs/evidence/repo_inventory_snapshot.txt`)

## 3) Model architecture and blocks discovered

### Main model (`Brown_Full_Sim.slx`)
Top-level references/subsystems include:
- `Translational Dynamics`
- `Rotational Dynamics`
- `Translational Kinematics`
- `roational kinematics` (typo in block name is present in model)
- `V_rel 2 Vinf Alpha Beta`
- `Wing`, `Left Tail`, `Right Tail` (`aeroSurface` references)
- `Propellers` subsystem (front/rear groups)
- `wind model (dryden)`
- `FlightGear Interface`

### Library model (`Brown_Flight_Controls_lib.slx`)
Contains subsystems for:
- translational/rotational dynamics,
- quaternion/DCM/euler conversions,
- aerodynamic surface force model,
- front/rear propeller group models,
- relative wind angle computation.

### Equation-level highlights (from extracted Stateflow EML)
- Aero angles: `V_inf, alpha, beta` from `v_rel`
- Aero surface model: linear `CL(alpha)`, parabolic `CD(alpha)`, moment coefficient `CM(alpha)`
- Propeller thrust/torque: `T = kT * rpm^2`, `Q = kQ * rpm^2 * spin_dir`
- Front propeller hub position: `pivot_pos + hub_offset * n(tilt)`
- Rigid-body kinematics: quaternion/euler/DCM conversion + `Cdot = W*C`

(Inventory evidence: `docs/evidence/model_inventory_snapshot.txt`, `docs/evidence/key_stateflow_equations.txt`)

## 4) Existing controllers and control strategy status

Observed current behavior:
- No explicit PID-type blocks found in scanned model XML.
- No explicit Saturation blocks found in scanned model XML.
- `Full_Sim_Init.m` test manager sets open-loop vectors:
  - `Motor_RPMs` (12x1)
  - `Tilt_angles` (6x1)

Interpretation:
- Present repo state appears **primarily open-loop / feedforward** for actuator commands in the baseline flow.
- Closed-loop flight control law (if intended) is not apparent in currently inspected blocks.

Evidence: `docs/evidence/controller_search_output.txt`

## 5) Physical assumptions currently encoded

From `Full_Sim_Init.m` + library equations:
- Rigid-body mass assembled from component primitives (box/crossprop), with parallel-axis theorem.
- Gravity scalar `g = -9.81` used in model wiring.
- Aerodynamics:
  - surfaces use fixed normals and static coefficients,
  - `CL = CL0 + CLa*alpha`,
  - `CD = CD0 + CDa*(alpha-a0)^2`,
  - no explicit dynamic stall/downwash/inflow corrections in extracted equations.
- Propulsion:
  - thrust and torque proportional to squared RPM,
  - front props tilt with nacelles, rear props fixed vertical.
- Wind:
  - Dryden turbulence block present (Aerospace library).

## 6) What has been validated in this run

### Confirmed by executed checks
- Mass/inertia bookkeeping is internally consistent:
  - Mass = **3040 kg**
  - CG = **[-0.0474, 0, -0.2513] m**
  - inertia matrix symmetric and positive definite
- Rotor inventory count (front=6, rear=6) matches expectations.
- Positive `kT`, `kQ` detected.

### Failures / warnings found by check pass
- **FAIL:** model expects `prop.Lspin_dir`, `prop.Rspin_dir` but init script does not define them.
- **WARN:** static thrust margin at 1500 RPM is very low vs weight (`T/W≈0.109` for all 12 motors under current coefficients/assumptions).
- **WARN:** no explicit saturation blocks found in scanned systems.

Evidence: `docs/evidence/validation_latest.md`

## 7) Known limitations and blockers (current state)

1. **Toolchain availability blocker in this execution environment**
   - MATLAB unavailable (`matlab: command not found`), so dynamic Simulink runs were not executable here.
2. **Init/model mask mismatch**
   - Missing spin-direction variables likely break Propeller subsystem execution.
3. **Potential units/scale risk on propulsion constants**
   - RPM-based thrust formula and current coefficients produce low thrust margin at nominal command.
4. **No explicit control law/saturation discovered**
   - likely contributes to instability/non-trimmable behavior.
5. **`Full_Sim_Init.m` side effects**
   - `clear/close/clc` and figure rendering are always executed, complicating headless automation.

## 8) What is missing for full confidence

Not yet completed in this environment:
- end-to-end smoke simulation pass (`scripts/smoke_test.m`) with MATLAB,
- verified trim point from actual Simulink operating-point solve (`findop`/`trim`),
- dynamic stability margins and mode-transition performance from real simulation logs,
- regression baseline plots/CI checks for flight quality metrics.

## 9) Evidence section (commands run + outputs)

Executed command set (non-exhaustive, key items):

1. Repo audit
```bash
ls -la
find . -maxdepth 3 -type f | sort
```
Output captured in:
- `docs/evidence/repo_inventory_snapshot.txt`

2. Model inventory extraction (zip/xml parse)
```bash
python3 <inline script parsing system_root.xml + stateflow chart signatures>
```
Output:
- `docs/evidence/model_inventory_snapshot.txt`

3. Key equation extraction
```bash
python3 <inline script extracting chart_110/119/130/147 etc scripts>
```
Output:
- `docs/evidence/key_stateflow_equations.txt`

4. Physical consistency checks
```bash
python3 scripts/validation/check_physical_consistency.py --repo-root .
```
Outputs:
- `docs/evidence/validation_command_output.txt`
- `docs/evidence/validation_latest.md`
- `docs/evidence/validation_latest.json`

5. Propeller mask variable cross-check
```bash
unzip -p Brown_Full_Sim.slx simulink/systems/system_90.xml | rg "spin_dir|kT|kQ|pivot_pos|prop_pos"
rg "spin_dir|Lspin|Rspin" Full_Sim_Init.m
```
Output:
- `docs/evidence/prop_mask_vs_init_vars.txt`

6. Trim fallback sweep
```bash
python3 scripts/trim/trim_cruise_fallback.py --speed 40/60/70/80
```
Outputs:
- `docs/evidence/trim_fallback_command_output.txt`
- `docs/evidence/trim_fallback_*mps.json`
- `docs/evidence/trim_fallback_sweep_summary.csv`

7. Transition metrics sample run
```bash
python3 scripts/metrics/example_metrics_run.py
```
Outputs:
- `docs/evidence/metrics_example_command_output.txt`
- `docs/evidence/metrics_example_latest.json`

8. Smoke test attempt (blocked)
```bash
matlab -batch "run('scripts/smoke_test.m')"
```
Output:
- `docs/evidence/smoke_test_command_output.txt`

9. Controller/saturation scan
```bash
python3 <inline script scanning model XML for PID/Saturation block types>
```
Output:
- `docs/evidence/controller_search_output.txt`

---

For step-by-step timing and status, see `docs/progress_log.md`.
