# Aero and Mass Parameter Update Notes

Date: 2026-03-23

## Scope

This update replaces the original placeholder aerodynamic coefficients and several visibly arbitrary component masses in `aircraft_def.m` with values guided by NASA lift-plus-cruise reference vehicle data and the same finite-wing formulas used in NDARC.

The goal was not to create a fully validated vehicle model, but to replace obviously nonphysical values with a more defensible first-pass set while preserving:
- total modeled mass of **3040 kg**
- a **forward CG** relative to the main-wing aerodynamic center
- compatibility with the current simple `aeroSurface` model

## Source Basis

Primary references used:

1. NASA/TM-20210017971
   - Table 6: aerodynamic inputs for NASA tiltwing / updated lift-plus-cruise reference analyses
   - Table 11 / Table 12: updated lift-plus-cruise dimensions and weight comparisons
2. The current `aeroSurface` block implementation in `Brown_Flight_Controls_lib.slx`
   - `CL = CL0 + CLa * alpha`
   - `CD = CD0 + CDa * (alpha - a0)^2`
   - `CM = CM0 + CMa * alpha`

## Key Decisions

- The wing coefficients were based on NASA’s cambered `GA(W)-1 Mod` wing inputs.
- The V-tail coefficients were based on a symmetric-tail assumption, closer to NASA’s `NACA 0020` vertical-tail treatment than to a cambered lifting tail.
- Total aircraft mass was kept at **3040 kg** because it is very close to NASA’s updated lift-plus-cruise **empty weight** scale, even though the current Simulink model does not explicitly separate payload/battery/fuel-state bookkeeping.
- The forward ballast mass was reduced, but not removed. After the mass rebalance, the CG remains forward of the wing quarter-chord by about **8.3% MAC**.

## Aerodynamic Coefficients

The old coefficients were mostly placeholder values. In particular, `CLa = 0.01` for both wing and tail surfaces was far too small to represent a full-scale fixed wing.

### Wing halves (`L Main Wing`, `R Main Wing`)

| Parameter | Old | New | Notes |
|---|---:|---:|---|
| `CL0` | 0.05 | 0.35 | Rounded from finite-wing lift slope and `alpha0 = -4.2 deg` |
| `e` | 0.90 | 0.90 | Retained; matches NASA span-efficiency input |
| `i` (rad) | 0.05 | 0.00 | Removed built-in incidence to avoid double-counting camber lift |
| `CD0` | 0.01 | 0.0085 | NASA Table 6 main-wing parasite-drag input |
| `CDa` | 1.00 | 0.0424 | Replaced with `1 / (pi e AR)` using current wing AR |
| `a0` (rad) | 0.05 | -0.0733 | `-4.2 deg` zero-lift AoA from NASA Table 6 |
| `CM0` | -0.05 | -0.10 | NASA pitching-moment input for cambered main wing |
| `CMa` | 0.00 | 0.00 | Left at zero; current simple model already gets major stability from force arm about CG |
| `CLa` (per rad) | 0.01 | 4.7671 | Finite-wing slope from NASA `cl_alpha = 6.10`, `tau = 0.20`, current AR |

### V-tail halves (`L V-Tail`, `R V-Tail`)

| Parameter | Old | New | Notes |
|---|---:|---:|---|
| `CL0` | 0.00 | 0.00 | Symmetric-tail assumption |
| `e` | 0.80 | 0.70 | Chosen to match NASA tail span-efficiency input order of magnitude |
| `i` (rad) | 0.00 | 0.00 | No built-in incidence |
| `CD0` | 0.01 | 0.0200 | Closer to NASA symmetric-tail / vertical-tail drag input |
| `CDa` | 1.00 | 0.3183 | Replaced with `1 / (pi e AR)` using current V-tail panel AR |
| `a0` (rad) | 0.00 | 0.00 | Symmetric-tail assumption |
| `CM0` | 0.00 | 0.00 | Symmetric-tail assumption |
| `CMa` | 0.00 | 0.00 | Left at zero in current simplified model |
| `CLa` (per rad) | 0.01 | 2.2028 | Finite-wing slope from NASA `cl_alpha = 5.70`, `tau = 0.25`, current AR |

## Mass Updates

Not every component has a clean public one-to-one reference value, so the mass update strategy was:

- directly correct the wing and rotor masses toward NASA lift-plus-cruise reference scales
- reduce the oversized tail masses
- keep booms/arms unchanged where public analog data were weak
- rebalance the remaining mass into the fuselage lump
- keep a smaller but still meaningful forward ballast mass for CG placement

| Component group | Old modeled mass | New modeled mass | Notes |
|---|---:|---:|---|
| Fuselage lump | 1500 kg | 1736 kg | Now explicitly carries the remainder of cabin, battery, systems, and landing gear mass |
| Forward ballast | 250 kg | 180 kg | Reduced, but preserved for forward CG control |
| Main wings total | 450 kg | 280 kg | Brought closer to NASA updated L+C main-wing weight scale |
| V-tail total | 180 kg | 100 kg | Reduced from hobby-aircraft-like overestimate |
| 12 rotors total | 300 kg | 384 kg | Brought closer to NASA rotor-group weight scale |
| Booms total | 330 kg | 330 kg | Retained pending better structural sizing data |
| Front arms total | 30 kg | 30 kg | Retained pending better nacelle/support sizing data |
| Total aircraft mass | 3040 kg | 3040 kg | Preserved intentionally |

## Resulting CG / Stability Check

With the updated masses:

- Total mass = **3040 kg**
- CG = **[0.0002, 0.0000, -0.2089] m**
- Main-wing quarter-chord x-location = **-0.125 m**
- CG forward of wing aerodynamic center by **0.1252 m**
- Static margin relative to the wing MAC = **8.34% MAC**

This is a much more comfortable result than simply leaving the old ballast untouched after the mass rebalance, and it satisfies the project requirement that the CG remain ahead of the main aerodynamic center.

## Important Limitations

- These updates make the present model **more defensible**, not fully validated.
- The simple `aeroSurface` block still omits major real effects:
  - stall / post-stall behavior
  - propeller slipstream effects on wing/tail lift curve
  - wing-body interference
  - downwash / tail effectiveness changes
  - control-surface derivatives
- The V-tail is still being modeled as two simple lifting panels with fixed normals, not as a full empennage/control-derivative model.
- Booms and arm masses remain heuristic because no clean public component-weight analog was available for those exact members.

## Next Best Follow-Up

If the simulation is going to be used for anything more than first-pass trim / transition prototyping, the next highest-value updates would be:

1. add explicit fuselage / hub parasite drag rather than forcing wing/tail `CD0` to absorb it
2. split the wing into `wingL` and `wingR` inside the Simulink aero model, not just in visualization/data structures
3. add a simple tail-effectiveness / downwash correction
4. revisit propeller `kT` and `kQ`, because propulsion is still the dominant mismatch in the current repo
