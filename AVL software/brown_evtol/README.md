# Brown eVTOL AVL Study

This folder contains a simplified fixed-aero AVL model for the current Brown
eVTOL sim. It is intentionally limited to the aerodynamic lifting surfaces:

- one symmetric wing
- one symmetric V-tail surface pair
- flap, aileron, elevator, and rudder controls

It omits:

- rotors and propwash
- booms and fuselage body effects
- viscous/profile drag modeling beyond AVL's inviscid outputs

The purpose of this model is:

1. complete the homework requirements for aircraft build, sweeps, and trim
2. generate aircraft-level coefficient data for comparison against the
   current `aeroSurface` linear-superposition model

The source-of-truth geometry and mass properties come from:

- `aircraft_def.m`
- the current progress-report fact sheet values for mass, CG, and inertia

Coordinate conversion from the sim to AVL is:

- `x_avl = -x_sim`
- `y_avl =  y_sim`
- `z_avl = -z_sim`

To rerun the full study and regenerate the HTML report, tables, and plots:

```bash
python3 scripts/avl/run_brown_evtol_study.py
```
