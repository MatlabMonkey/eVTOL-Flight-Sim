# Control Formulation And Working Demos

Date: 2026-04-21

This note captures two things:

1. the ideal control formulation once a larger trim/linearization map exists
2. what is already working in the current demo scripts

The intended audience is anyone working in `eVTOL_Simulation` who needs a clear description of the current scheduled-controller path.

## Ideal Control Formulation

Once a usable map of trim points and local linearizations exists over transition, the right controller structure is:

\[
u = u^\star(\rho) - K(\rho)\bigl(x - x^\star(\rho)\bigr)
\]

where:

- \(x^\star(\rho)\) is the trimmed state at the scheduled operating point
- \(u^\star(\rho)\) is the trimmed command at the scheduled operating point
- \(K(\rho)\) is the local feedback gain designed from the linearization at that point
- \(\rho\) is the scheduling variable

In this project, there are two practical choices for \(\rho\):

- path progress \(s \in [0,1]\) along a feasible transition corridor
- direct scheduling on \((\text{tilt}, V_\infty)\) once the trim map is dense enough

For the current project stage, path-based scheduling is the better formulation.

## Recommended Architecture

The recommended architecture is hierarchical:

1. `transition manager`
   chooses where the vehicle should be in the transition corridor
2. `trim scheduler`
   looks up \(x^\star\), \(u^\star\), and \(K\)
3. `inner-loop feedback`
   stabilizes the aircraft around the scheduled operating point

The practical implementation is:

\[
u = u^\star(s) - K(s)\bigl(x - x^\star(s)\bigr)
\]

with smooth interpolation in \(s\).

This is better than switching directly between endpoint controllers, because the scheduled commands and gains vary continuously instead of jumping when the aircraft reaches a new point.

## What Should Be Stored In The Map

At each scheduled point, store:

- trim state \(x^\star_i\)
- trim command \(u^\star_i\)
- linear model \(A_i, B_i\)
- local feedback gain \(K_i\)

Then interpolate these quantities online.

For this project, the best near-term version is:

- choose a feasible path through the trim map
- sample a sequence of valid trim points along that path
- build a local controller at each point
- interpolate \(x^\star\), \(u^\star\), and \(K\) along the path

This is the right compromise between rigor and implementation effort.

## Working Demo Controllers

Two demo aliases currently work through the scheduled path-LQR flow:

- [Prep_Demo1_45to90_Rear500.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo1_45to90_Rear500.m)
- [Prep_Demo2_Bridge30V60_to_Cruise90V75_Rear500.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Demo2_Bridge30V60_to_Cruise90V75_Rear500.m)

These alias scripts call:

- [Prep_Wrapper_Demo_Mid45_to_Cruise90_Rear500_ScheduledLQR.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Wrapper_Demo_Mid45_to_Cruise90_Rear500_ScheduledLQR.m)
- [Prep_Wrapper_Demo_Bridge30V60_to_Cruise90V75_Rear500_ScheduledLQR.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Prep_Wrapper_Demo_Bridge30V60_to_Cruise90V75_Rear500_ScheduledLQR.m)

These prep scripts do not contain the control law themselves. They:

1. load cached trim points and cached `controllerData`
2. build smooth command traces with [make_transition_path_schedule_cmds.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/make_transition_path_schedule_cmds.m)
3. call [Run_EVTOL_Main.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/Run_EVTOL_Main.m), which publishes the controller package into the wrapper workspace

## What Controller They Use

The active runtime controller is:

- `controller_id = 4`

inside:

- [controllers/controller_dispatch.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_dispatch.m)
- [controllers/controller_lqr_path_schedule.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_lqr_path_schedule.m)

That scheduled controller is built from:

- [build_two_point_transition_lqr_controller.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/build_two_point_transition_lqr_controller.m)

and each endpoint controller is built by:

- [build_trim_point_longitudinal_lqr_controller.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/build_trim_point_longitudinal_lqr_controller.m)

## Local Endpoint Controller

Each endpoint controller is a longitudinal LQR, not a full 6-DOF controller.

Its active state set is:

\[
x_{\text{long}} = [\theta,\ u,\ w,\ Q]
\]

Its active input set is:

\[
u_{\text{long}} = [\text{front\_coll},\ \text{rear\_coll},\ \delta_f,\ \delta_e]
\]

The full state convention used by the dispatcher remains:

\[
x = [\phi,\ \theta,\ \psi,\ u,\ v,\ w,\ P,\ Q,\ R]
\]

The full mixed-control command convention is:

\[
u_{\text{mixed}} =
\begin{bmatrix}
\text{front\_collective} \\
\text{rear\_collective} \\
\delta_f \\
\delta_a \\
\delta_e \\
\delta_r
\end{bmatrix}
\]

The local longitudinal LQR computes:

\[
\Delta u = -K_{\text{long}} \Delta x
\]

and embeds that into the full 6-channel mixed-control vector, leaving the lateral channels at trim.

## Two-Point Scheduled Formulation

The two-point scheduled builder stores:

- `controller_state_ref`
- `controller_trim_cmd`
- `controller_gain_lqr`

for two endpoints only.

The schedule arrays contain:

- state reference columns for point A and point B
- trim command columns for point A and point B
- gain pages for point A and point B
- metadata rows for tilt, airspeed, and normalized progress

At runtime, the scheduled controller:

1. reads commanded `tilt` and commanded `Vinf`
2. projects that command pair onto the scheduled path in `(tilt, Vinf)` space
3. computes the corresponding path progress
4. finds the two bracketing schedule points
5. linearly interpolates:
   - \(x^\star\)
   - \(u^\star\)
   - \(K\)
6. applies the LQR hold law around the interpolated operating point

That is implemented in [controllers/controller_lqr_path_schedule.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_lqr_path_schedule.m).

## Runtime Control Law

For the working demos, the runtime law is:

\[
x^\star(\lambda) = (1-\lambda)x^\star_A + \lambda x^\star_B
\]

\[
u^\star(\lambda) = (1-\lambda)u^\star_A + \lambda u^\star_B
\]

\[
K(\lambda) = (1-\lambda)K_A + \lambda K_B
\]

then

\[
u = u^\star(\lambda) - K(\lambda)\bigl(x - x^\star(\lambda)\bigr)
\]

This is the actual formulation in the demos. It is not a hard switch from one equilibrium controller to the next.

## Command Scheduling Versus Gain Scheduling

There are two separate interpolations in the working demo path:

1. `command scheduling`
   [make_transition_path_schedule_cmds.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/make_transition_path_schedule_cmds.m) builds smooth time histories of:
   - `Vinf`
   - `alpha`
   - `beta`
   - Euler commands
   - tilt
   - collective commands
   - mixed/local surface commands

2. `controller scheduling`
   [controllers/controller_lqr_path_schedule.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_lqr_path_schedule.m) interpolates:
   - `x_ref`
   - `trim_cmd`
   - `K`

The demos work because these two schedules are aligned. The command path and the controller path are the same two-point transition branch.

## Output Channels Used By The Plant

After the mixed-control command is computed, the dispatcher converts the surface channels into physical surfaces using `MixMatrix`, then sends:

- `front_collective_rpm`
- `rear_collective_rpm`
- `deltaLW`
- `deltaRW`
- `deltaLT`
- `deltaRT`

to the wrapper outputs.

This conversion happens inside [controllers/controller_dispatch.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/controllers/controller_dispatch.m).

## Practical Interpretation

What is working so far is not a general 2D scheduled controller over the whole trim map. It is a narrower, more controlled design:

- choose two exact trim points
- build one longitudinal LQR at each point
- interpolate references, trim commands, and gains between those two points
- drive the interpolation with the commanded `(tilt, Vinf)` pair

That is a reasonable first working formulation because:

- it is simple
- it uses only exact endpoint points
- it avoids a hard controller switch
- it keeps the transition path coherent

The natural extension is to replace the 2-point schedule with a denser path extracted from the larger trim map, while keeping the same runtime law.
