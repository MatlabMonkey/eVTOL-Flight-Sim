# Validation check output

Generated: 2026-03-21T17:51:44.529556

## Mass properties
- Mass: **3040.000 kg**
- CG: **[-0.04736842105263159, 0.0, -0.25125] m**
- Inertia J (kg·m^2): `[23,083.856, -0.000, -749.220; -0.000, 14,400.124, 0.000; -749.220, 0.000, 36,185.752]`
- Principal moments: **[14400.12394736842, 23041.151727700562, 36228.45671966784]**

## Derived thrust/weight
- Weight: **29822.4 N**
- Thrust per motor at 1500 RPM: **270.0 N**
- Total thrust (12 motors): **3240.0 N** (T/W=0.109)
- Rear-only thrust (6 motors): **1620.0 N** (T/W=0.054)

## Checks
- [PASS] **Mass positive** — mass=3040.000 kg
- [PASS] **Mass plausible range** — Expected 500..10000 kg for this class (heuristic).
- [PASS] **CG within component bounding box** — CG=[-0.04736842105263159, 0.0, -0.25125], bbox_min=[-3.5, -6.5, -0.91], bbox_max=[3.0, 6.5, 0.0]
- [PASS] **Inertia matrix symmetry** — J-J^T max abs=0.000e+00
- [PASS] **Inertia positive definite** — principal moments=[14400.12394736842, 23041.151727700562, 36228.45671966784]
- [PASS] **Principal moments satisfy triangle inequality** — I=[14400.12394736842, 23041.151727700562, 36228.45671966784]
- [PASS] **Front rotor count == 6** — count=6
- [PASS] **Rear rotor count == 6** — count=6
- [PASS] **Thrust coefficient positive** — kT=0.00012
- [PASS] **Torque coefficient positive** — kQ=1.5e-05
- [WARN] **Static thrust margin at 1500 RPM** — T_motor=270.0 N, T_12=3240.0 N, T_rear6=1620.0 N, Weight=29822.4 N
- [FAIL] **Init defines prop.Lspin_dir** — Required by Propellers subsystem mask.
- [FAIL] **Init defines prop.Rspin_dir** — Required by Propellers subsystem mask.
- [WARN] **Propeller command saturation blocks present** — No explicit Saturation block found in Propellers subsystem (system_90.xml).
