# Physical Validation Plan (Current Pass)

Date: 2026-03-21

## Scope of this pass

This pass focuses on **static physical consistency** and model wiring sanity that can be validated without launching Simulink in this environment.

Primary sources inspected:
- `Full_Sim_Init.m`
- `Brown_Full_Sim.slx` (XML inside `.slx` archive)
- `Brown_Flight_Controls_lib.slx` (XML + Stateflow chart scripts)

Automated checker added:
- `scripts/validation/check_physical_consistency.py`

Latest run artifacts:
- `docs/evidence/validation_latest.md`
- `docs/evidence/validation_latest.json`

## Checklist and status

### A. Mass / inertia sanity
- [x] Total mass > 0
- [x] Mass within broad plausibility range (heuristic 500..10000 kg)
- [x] CG lies inside component-position bounding box
- [x] Inertia matrix symmetric
- [x] Inertia positive definite
- [x] Principal-moment triangle inequalities hold

Current computed values (from script):
- Mass = **3040 kg**
- CG = **[-0.0474, 0, -0.2513] m**
- Principal moments = **[14400.1, 23041.2, 36228.5] kg·m²**

### B. Geometry / actuator inventory sanity
- [x] Front rotor count = 6
- [x] Rear rotor count = 6
- [x] `kT` and `kQ` positive
- [ ] Spin-direction mask variables are defined in init script (**FAIL**)  
  Required by model: `prop.Lspin_dir`, `prop.Rspin_dir`  
  Observed in `Full_Sim_Init.m`: not defined.

### C. Command/constraint sanity
- [ ] Explicit actuator saturation blocks present (**WARN**)  
  XML scan did not find `Saturation` blocks in scanned systems.

### D. Force/weight magnitude sanity
- [ ] Baseline commanded thrust plausibility at 1500 RPM (**WARN**)  
  Estimated with current coefficients:  
  - Thrust per motor @1500 RPM ≈ **270 N**  
  - 12-motor total ≈ **3240 N**  
  - Weight (3040 kg) ≈ **29822 N**  
  - T/W ≈ **0.109** (rear-only 0.054)

## Interpretation

What passes:
- Rigid-body mass/inertia bookkeeping in `Full_Sim_Init.m` is internally consistent.
- Component/rotor geometry extraction is coherent enough for downstream scripts.

What currently fails or is risky:
1. **Model-wiring mismatch**: prop spin direction parameters expected by the Propellers subsystem are not initialized.
2. **Actuator envelope mismatch**: current nominal RPM command (1500) appears far below weight-support requirement given present `kT` and mass.
3. **No visible saturation blocks**: command clipping/limits are not explicit in scanned model XML.

## Next validation actions (recommended order)

1. **Fix hard fail first**: define `prop.Lspin_dir` and `prop.Rspin_dir` in `Full_Sim_Init.m` with explicit sign conventions and rotor ordering documentation.
2. **Add actuator limits to model or upstream command generation** (RPM and tilt rate/angle limits).
3. **Unit audit** (critical): verify whether `kT`,`kQ` are intended for RPM or rad/s usage. If intended for rad/s, convert formulas accordingly.
4. **Dynamic validation once MATLAB is available**:
   - run `scripts/smoke_test.m`
   - compare logged force balance vs expected static hover/cruise points
   - check translational/rotational accelerations under zero-input and trim input conditions.

## Repro command

```bash
python3 scripts/validation/check_physical_consistency.py --repo-root .
```
