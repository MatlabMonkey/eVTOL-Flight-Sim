# INDI Control Architecture And Surface Map Plan

Last reviewed: 2026-04-27 00:00 PDT

Status update: the first scheduled longitudinal INDI controller has now been
implemented in `controllers/controller_indi_transition.m`, with offline
schedule/path construction in
`controllers/builders/build_indi_transition_controller.m`. This document remains
the architecture rationale and planning background.

This document captures the current control idea for replacing or augmenting
the scheduled longitudinal LQR with an incremental nonlinear dynamic inversion
(INDI) controller.

## Summary

The current corridor LQR work showed that trim-scheduled feedforward is useful,
but the controller is sensitive to the selected trim branch and to actuator
allocation. INDI is a strong next step because the plant can provide
acceleration measurements in simulation. Those measurements let the controller
avoid modeling the full nonlinear drift term and instead solve an incremental
control-effectiveness problem around the current measured state.

Recommended high-level architecture:

```text
trim/corridor schedule
  -> slow reference and nominal tilt path
  -> feedforward nominal actuator bias

measured state + measured acceleration + actual actuator states
  -> INDI longitudinal acceleration controller
  -> constrained allocator over RPM^2, flap, elevator
  -> actuator commands with limits/rate limits
```

The trim corridor should still exist, but it should not be responsible for
perfect point-to-point settling. It should provide a reasonable path, nominal
tilt, and nominal actuator bias. The INDI loop should close the fast
longitudinal acceleration/pitch dynamics around the actual vehicle response.

## INDI Formulation

Use a longitudinal acceleration vector such as:

```text
nu_dot = [a_x_body; a_z_body; q_dot]
```

or, if the available accelerometer signal is specific force, use the specific
force consistently in both measurement and command.

The incremental control law is:

```text
delta_nu_dot = nu_dot_cmd - nu_dot_meas
delta_nu_dot ≈ G(x, u_actual) * delta_eta
```

where the proposed control variable is:

```text
eta = [front_rpm^2; rear_rpm^2; delta_f; delta_e]
```

Then solve:

```text
delta_eta = argmin ||W_nu * (G * delta_eta - delta_nu_dot)||^2
                  + ||W_eta * delta_eta||^2
```

subject to actuator position and rate limits. After solving, convert the RPM
channels back to physical commands:

```text
front_rpm_cmd = sqrt(max(front_rpm_actual^2 + delta_front_rpm_sq, 0))
rear_rpm_cmd  = sqrt(max(rear_rpm_actual^2  + delta_rear_rpm_sq,  0))
```

Using `RPM^2` internally is preferred because the current prop model is
approximately linear in `RPM^2`, so the allocator sees cleaner columns in
the effectiveness matrix. Physical actuator limits should still be enforced in
RPM and RPM/s after the conversion.

## Control Effectiveness Matrix

The controller needs an effectiveness matrix:

```text
G = d[a_x_body; a_z_body; q_dot] / d[front_rpm^2; rear_rpm^2; delta_f; delta_e]
```

Propeller columns can be computed analytically from the current prop model:

```text
T = kT * RPM^2
```

Front prop direction depends on scheduled/actual front tilt. Rear prop
direction is fixed. Moment effects come from the rotor lever arms and reaction
torque terms.

Surface columns should be mapped or sampled:

```text
G_f(Vinf, alpha, delta_f) = d[Fx; Fz; My] / d(delta_f)
G_e(Vinf, alpha, delta_e) = d[Fx; Fz; My] / d(delta_e)
```

For slow transition runs, ignoring pitch-rate dependence is acceptable for a
first pass. The active aero block does include local velocity from body rate,
but if the transition is intentionally slow, the local `cross(omega, r_arm)`
term should be a second-order correction. Surface deflection dependence is more
important because the failed LQR cases often drove flaps/elevator near large
deflections or saturation.

Convert force/moment effectiveness to acceleration effectiveness with:

```text
dax/delta ≈ dFx/delta / mass
daz/delta ≈ dFz/delta / mass
dqdot/delta ≈ dMy/delta / Iyy
```

If the final implementation uses exact rigid-body equations, include the
appropriate inertia coupling terms. For the first longitudinal controller,
the simple conversion is probably adequate.

## Surface Map Sampling Plan

Use `Trim_Plant` to generate the surface maps if time permits. That is more
defensible in a presentation because it samples the same Simulink plant path
used by trim and simulation, including actuator initial states and aero block
wiring.

Sampling method:

```text
for each Vinf, alpha, delta_f:
  set V_init = [Vinf*cos(alpha); 0; Vinf*sin(alpha)]
  set omega_init = [0; 0; 0]
  set wing surface actual states to delta_f
  zero prop inputs
  run Trim_Plant briefly
  log aero_force_sum and aero_moment_sum
  finite-difference around delta_f for dF/d(delta_f), dM/d(delta_f)

repeat similarly for delta_e using tail surfaces
```

Recommended first map grid:

```text
Vinf    = [2.5 5 7.5 10 15:5:80] m/s
alpha   = -10:2.5:45 deg
delta   = -25:5:25 deg
```

That is about 4,500 to 5,000 grid points per surface before finite-difference
doubling. A smaller first-pass grid should be used for implementation:

```text
Vinf    = [2.5 5 10 15 20 30 40 50 60 70 80] m/s
alpha   = -10:5:45 deg
delta   = -25:5:25 deg
```

That is `11 * 12 * 11 = 1452` points per surface. With central differences,
both flap and elevator maps would require roughly `2 * 2 * 1452 = 5808`
Trim_Plant calls if no results are reused. That is feasible but not instant.

## Benchmark Result

A reusable benchmark helper was added:

```matlab
result = benchmark_trim_plant_surface_sampling(100);
```

Benchmark result from 2026-04-26:

```text
samples          : 100
Trim_Plant stop  : 0.0200 s
elapsed          : 36.169 s
seconds/sample   : 0.3617 s
samples/second   : 2.76
```

The original benchmark result was saved to:

```text
workspace_plots/indi_surface_sampling_benchmark.mat
```

Important caveat: that first benchmark was run before the restored AVL polar
tables were back in the repo-root `scripts/` folder. The coefficient-based
fallback path has now been removed. The benchmark helper now requires
`databases/aero_polars/final_airfoil_polar_tables.mat` through `Init_Main` and
errors if generated AVL polar tables are unavailable.

Based on the benchmark, a 5,000-call serial map would take roughly 30 minutes
after compile overhead. A 10,000-call map would take roughly 60 minutes. That is
acceptable for an overnight or final-build artifact, but too slow for repeated
interactive tuning. The practical workflow should be:

```text
small grid -> implement INDI -> validate behavior -> densify only if needed
```

## Coarse Map Status

The first coarse map was generated on 2026-04-26 with:

```matlab
map = build_indi_surface_effectiveness_map();
```

Output files:

```text
databases/indi_surface_effectiveness_map_coarse.mat
databases/indi_surface_effectiveness_map_coarse.md
```

Completed run:

```text
Vinf_mps        = [2.5 5 10 15 20 30 40 50 60 70 80]
alpha_deg       = [-10 -5 0 5 10 15 20 25 30 35 40 45]
delta_deg       = [-25 -20 -15 -10 -5 0 5 10 15 20 25]
calls           = 5808
elapsed         = 2063.6 s = 34.4 min
seconds/call    = 0.3553
```

That run used the same invalid coefficient fallback path, so the local generated
artifact was removed and should not be used for control. Rebuild it only after
`Init_Main` confirms the generated AVL polar tables are loaded. The regenerated
artifact will store flap and elevator force/moment derivatives with shape
`[nV nAlpha nDelta 3]`.

## Simulink Sampling Versus Copied Aero Function

Using `Trim_Plant` is more presentation-valid because it demonstrates that the
effectiveness map comes from the same model used for trim and nonlinear
simulation. It also catches wiring, sign, surface mixing, and unit mistakes.

Using a copied MATLAB aero function would be much faster and easier to grid
dense, but it carries a synchronization risk: if the Simulink aero block and
the copied function drift apart, the INDI map becomes less trustworthy.

Recommended compromise:

```text
1. Build the first map from Trim_Plant on a moderate grid.
2. Build a copied MATLAB evaluator only if runtime becomes painful.
3. Validate any copied evaluator against sparse Trim_Plant samples.
```

## Implementation Steps

1. Build `build_indi_surface_effectiveness_map.m`.
   It should generate flap/elevator force and moment derivative tables over
   `Vinf`, `alpha`, and surface deflection.

2. Build `controller_indi_longitudinal.m`.
   It should use measured acceleration, measured angular acceleration or
   filtered/differenced pitch rate, actual actuator states, and the current
   scheduled tilt.

3. Add a new `controller_id` path in `controllers/controller_dispatch.m`.
   The dispatcher already receives measured acceleration and actuator commands,
   but we need to verify it has access to actual actuator states. If not, the
   controller block interface should be extended or the actual actuator states
   should be exposed through existing measurement buses.

4. Keep tilt scheduled and slow.
   Do not include tilt in the first fast INDI inversion. Tilt changes geometry
   and is rate-limited, so it should remain a transition-scheduler variable.

5. Test first as trim hold.
   Before trying hover-to-cruise, hold one mid-transition point and command
   small acceleration/pitch-rate changes. If the signs and allocator are right,
   then connect it to the corridor schedule.

## Open Questions

The biggest implementation question is whether the controller block currently
receives actual actuator states or only commanded actuator signals. INDI should
use actual actuator states. If only commands are available, the next model
change should expose actual rotor speeds, actual tilt, and actual surface
deflections to the controller.

The second question is angular acceleration. If `q_dot` is directly available
from the plant, use it. If not, estimate it from measured pitch rate with a
simple filtered derivative for simulation. Since this work currently assumes
noise-free measurements, that derivative can be simple at first.
