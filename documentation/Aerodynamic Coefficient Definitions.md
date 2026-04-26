# Final Wing, Tail, and Moment Polar Source

This note documents the **final version** of the wing and tail aerodynamic polar sources used for the updated aerodynamic model.

## Final Decision

For the final wing and tail lift/drag polars, we used the **Airfoil360 wind-tunnel dataset** directly, not the old analytical coefficient formulas and not the low-alpha AVL fit.

For the final local section pitching-moment coefficient, we used:

- low alpha: `XFOIL` section `Cm(alpha)` data
- high alpha: a smooth analytical continuation based on a flat-plate-style local moment assumption
- final runtime form: one final `Cm(alpha)` table per surface

## Airfoils Used

- Wing airfoil: `NACA 2412`
- Tail airfoil: `NACA 0012`

These match the airfoils used in the Brown AVL geometry file:

- [brown_evtol.avl](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/AVL%20software/brown_evtol/brown_evtol.avl)

## Data Source

The source file is:

- [Airfoil360_wind_tunnel_data_v2022.xlsx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Airfoil360_wind_tunnel_data_v2022.xlsx)

We used the **Re = 100k** sheets:

- `NACA 2412 Re=100K`
- `NACA 0012 Re=100K`

## What Was Taken From the Workbook

For each airfoil, we used:

- `AOA`
- `CL`
- `CD`

So the final aerodynamic inputs are:

- Wing: `CL_wing(alpha)`, `CD_wing(alpha)`
- Tail: `CL_tail(alpha)`, `CD_tail(alpha)`

## How the Polar Curves Were Built

The workbook stores angle of attack from `0 deg` to `360 deg`.  
To make the curves usable in the simulation, the angle axis was converted to the standard signed form:

- `0 deg` to `180 deg` stays unchanged
- `180 deg` to `360 deg` is remapped to `-180 deg` to `0 deg`

After that:

1. The angle data was sorted onto a signed `[-180, 180] deg` axis.
2. Duplicate endpoints were removed if `0 deg` and `360 deg` mapped to the same signed angle.
3. The final `CL(alpha)` and `CD(alpha)` curves were used as direct lookup-table data.

## Final Modeling Choice

The final choice for lift and drag was:

- **no polynomial fit**
- **no low-alpha analytical `CL0 + CLa * alpha` model**
- **no AVL-derived lift curve used as the final source**

Instead, the final curves are just the wind-tunnel airfoil tables used directly as lookup-table data.

At runtime, the model should do:

- look up `CL` from the appropriate airfoil table using effective angle of attack
- look up `CD` from the appropriate airfoil table using effective angle of attack
- then compute aerodynamic forces from those coefficients

## Final Pitching-Moment Approach

The final local section moment coefficient is handled separately from `CL` and `CD`.

### Low-Alpha Source

For low alpha, we used `XFOIL` section `Cm(alpha)` data for:

- `NACA 2412` at `Re = 100k`
- `NACA 0012` at `Re = 100k`

This gives a physically reasonable local airfoil-section pitching moment in the attached-flow regime.

### High-Alpha Source

For high alpha, we used an analytical continuation based on a flat-plate-style local moment model.

The important modeling choice is:

- this is a **local section moment** model
- it is **not** the CG moment from the wing or tail force arm

That means it does **not** replace the separate `cross(r_arm, F)` force-arm moment already computed by the main aerodynamic force/moment code.

### Why This Avoids Double Counting

The force model still works as:

- `CL(alpha)` and `CD(alpha)` define aerodynamic force
- the code computes the force vector
- the code separately applies the force at the surface location relative to the CG

The `Cm(alpha)` table only supplies the additional **local section moment about the surface reference point**, which is the standard role of `Cm`.

### Final Runtime Form

At runtime, the model should use:

- wing `Cm(alpha)` lookup table
- tail `Cm(alpha)` lookup table

So the final aerodynamic lookup structure is:

- Wing: `CL(alpha)`, `CD(alpha)`, `Cm(alpha)`
- Tail: `CL(alpha)`, `CD(alpha)`, `Cm(alpha)`

## Important Scope Note

These are **2D airfoil section polars**, not full-aircraft coefficients.

That means:

- they are appropriate as the **local wing and tail section lift/drag source**
- they do **not** include the aircraft CG force arm
- they do **not** replace the aircraft-level moment calculation

For `Cm`, the same local-section logic applies:

- the table is for the **local airfoil section moment coefficient**
- the aircraft CG moment from the wing/tail force arm is still handled separately in the main force/moment calculation

## Useful Comparison Scripts

The main quick-look comparison script used while checking the final lift/drag source was:

- [plot_airfoil360_vs_avl.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/avl/plot_airfoil360_vs_avl.m)

That script was only used for visual comparison and sanity checks.  
The final `CL/CD` source still comes directly from the Airfoil360 workbook.

The quick-look comparison script used while checking the final `Cm` approach was:

- [plot_xfoil_vs_airfoil360_cm.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/avl/plot_xfoil_vs_airfoil360_cm.m)

That script was used to compare:

- XFOIL low-alpha `Cm`
- the analytical continuation
- the final blended one-table `Cm` approach
