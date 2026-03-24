# VTOL Flight Transition Simulation Progress Report

Zacharias Brown  
AME 532  
March 24, 2026

## 1. Original Project Specifications and Scope Update

The original proposal for this project had three main objectives. The first objective was to produce an accurate 6-DoF simulation of a VTOL aircraft using a contemporary vehicle configuration. The second objective was to create different scenarios in which the transition region could be analyzed and evaluated. The third objective was to create a framework for comparing different transition-control methods or aircraft configurations.

At this point, the project scope is more focused and much better defined than it was in the proposal. Instead of a generic VTOL concept, the aircraft model is now based on a lift+cruise configuration inspired by the Archer Midnight. The simulation includes a full rigid-body model, aerodynamic surfaces, a 12-rotor propulsion layout, simplified control surfaces, and a set of updated validation cases. The comparison framework is still in progress, but it now has the main pieces needed to move in that direction, including a trim workflow, a simple controller scaffold, and a transition-metrics script.

Table 1 summarizes the original proposal goals and the current status.

| Original proposal goal | Current status |
| --- | --- |
| Build a 6-DoF VTOL simulation using a contemporary configuration | Completed at a first-pass research level. The main Simulink model now runs with updated aircraft geometry, mass properties, and aerodynamic coefficients. |
| Create scenarios for transition analysis | Partially completed. Scenario initialization is now separated into `scenario_def.m`, and the updated tester includes six validation cases tied to the current aircraft. |
| Build a framework for comparing control methods or configurations | In progress. Transition metrics are defined, a trim workflow exists, and a simple PD controller scaffold has been integrated, but the controller is not yet tuned and a fully converged trim point has not been obtained. |

## 2. Aircraft Model Development

The largest technical step in this reporting period was replacing the original placeholder aircraft setup with a more defensible aircraft definition. The main model now uses a simplified Archer Midnight-like lift+cruise configuration with two half-wings, a V-tail, six front tilting rotors, and six rear fixed rotors. The aircraft definition was separated from scenario initialization so that the physical vehicle can be edited independently from initial conditions and test cases.

[Insert Figure 1 here: rendered aircraft model with thrust vectors and aerodynamic/control-surface normals]

The current modeled aircraft properties are listed in Table 2.

| Property | Current value |
| --- | --- |
| Total modeled mass | 3040 kg |
| CG | `[0.0002, 0.0000, -0.2089] m` |
| Inertia matrix | `Jxx = 2.184e4`, `Jyy = 1.418e4`, `Jzz = 3.474e4`, `Jxz = -5.03e2 kg·m^2` |
| Fuselage dimensions | `7.50 x 2.00 x 1.60 m` |
| Wing geometry | Two half-wings, each `1.50 x 6.25 x 0.20 m` |
| Total wing area | 18.75 `m^2` |
| Tail geometry | Two V-tail halves, each `1.40 x 2.00 x 0.08 m` at `±45 deg` |
| Rotor layout | 12 total rotors: 6 front tilting, 6 rear fixed |
| Wing `CLa` | `4.7671 / rad` |
| Tail `CLa` | `2.2028 / rad` |

These values are not intended to be treated as final validated aircraft data. The goal of this update was to replace the original obviously arbitrary values with a more defensible first-pass set based on contemporary lift+cruise reference data and finite-wing estimates. In particular, the wing and tail aerodynamic coefficients were updated, the major component masses were rebalanced, and the CG was checked to make sure it remained ahead of the main aerodynamic center.

## 3. Control Surfaces

Another important update was the addition of control surfaces to the aircraft definition and visualization. The current simplified control-surface model includes one ruddervator per V-tail half and one flaperon per wing half. This is not the full segmented control layout of the actual Archer platform, but it is a reasonable simplification for the current stage of the simulation.

[Insert Figure 2 here: rendered aircraft with visible ruddervators and flaperons deflected]

The renderer now shows the physical surface deflections as well as the corresponding control-surface normal vectors. This made it much easier to verify the sign conventions and geometry. Positive flap deflection is trailing-edge down. Positive ruddervator deflection is also trailing-edge down. The current metadata also includes first-pass control-effect estimates for the V-tail. For example, the expected ruddervator pitching-moment derivative is about `CM_delta_e = -0.487 / rad`, and the estimated yaw derivative is `CN_delta_r = +0.059 / rad`.

The aerodynamic surface blocks were also updated so they can accept local control-deflection inputs. That means the model now has a direct path from control commands to aerodynamic force and moment changes on each wing half and each V-tail half.

## 4. Controller and Trim Progress

A simple PD controller scaffold has now been integrated into `Brown_Full_Sim.slx`. The controller is currently structured as a gated placeholder block that uses Euler-angle error and body-rate feedback to generate left and right flaperon commands, left and right ruddervator commands, and a motor RPM command vector. The controller is disabled by default through the workspace variable `controller_enable` so that the baseline validation cases can still be run without changing their meaning.

This is an integration milestone, not a final control result. The controller-enabled simulation now executes successfully, so the command path from vehicle states to aerodynamic control surfaces and propulsion commands exists in the model. However, the controller is not yet tuned well enough to hold a trimmed flight condition over the 10-second run. I would describe this as proof that the control architecture is now connected, not proof that the transition control problem is solved.

The trim workflow also improved during this reporting period. The trim helper script now runs correctly from the repo and returns first-pass actuator estimates. For a 70 m/s cruise target with 90 degree front-rotor tilt, the current fallback estimate was:

- front RPM for cruise drag balance: `1063.6`
- rear RPM for full-weight support: `6435.8`

This is useful progress because it gives a starting point for later operating-point work, but it is still not a fully converged 6-DoF trim solution. That distinction matters. At this stage, it is accurate to say that preliminary trim conditions were estimated and a trim workflow now exists, but a true closed-loop trimmed operating point has not yet been found.

## 5. Updated Validation and Transition Metrics

The old classroom-style test cases were also updated so that their values make sense for the current aircraft instead of the original placeholder setup. The updated tester now includes inertia-coupling, near-weight-support, pitch-moment, roll-damping, sideslip-to-yaw, and unpowered-glide cases. These are now tied to the current mass properties and cruise-like velocities.

The baseline validation flow currently passes the main smoke check, the full model simulates successfully, and the updated cruise-response cases produce consistent outputs. For example, in the updated roll-damping case the final body rates at 10 s are approximately `[0.0009, 0.0655, 0.0006] rad/s`. In the cruise beta-to-yaw case the final yaw rate is about `-0.0481 rad/s`, which at least shows that the updated lateral-directional coupling is active in the model.

For transition evaluation, I settled on two metrics that make the most sense to carry forward immediately:

1. `transition_time_s`
2. `jerk_rms`

The first metric captures whether the vehicle can complete the maneuver in a useful amount of time, and the second metric captures how aggressive or uncomfortable the maneuver is. A synthetic example run of the metrics script gave `transition_time_s = 9.60 s` and `jerk_rms = 1.155 m/s^3`, which confirms that the metric framework is implemented and ready to be used on actual transition simulations once the controller is improved.

## 6. Current State of the Project

Overall, I think the project is in a much stronger state than it was at the proposal stage. The aircraft model is now substantially more realistic, the geometry and control surfaces are much clearer, the validation cases are tied to the actual vehicle data, the trim workflow runs, and the model now contains a real controller scaffold instead of only open-loop commands. At the same time, the most important remaining limitation is still the control problem itself. The controller is present, but it is not yet tuned for transition, and the propulsion model still makes it difficult to obtain a believable trimmed condition.

So the most honest summary is this: the plant model and evaluation framework have advanced substantially during this reporting period, and the project is now set up for controller-tuning and transition-performance work, but it is not yet at the stage where I would claim a successful controlled transition.
