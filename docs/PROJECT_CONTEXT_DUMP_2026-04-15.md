# Project Context Dump

This document is meant to let a brand-new chat, teammate, or future version of me pick up this project without having to reconstruct the repo history from scratch.

It is intentionally broader than a recent status update. It explains:

- what this project is
- how the main models fit together
- what work has already been done
- what is currently working
- what is still rough
- how to run the current control-demo workflow

Project root:

`/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim`

## 1. What This Project Is

This repository is a Simulink-based full-flight simulation workspace for a Brown eVTOL aircraft. It is being used for flight controls coursework and controller-design homework, not just for one-off trim calculations.

At a high level, the repo contains:

- a vehicle definition and initialization pipeline
- multiple 6DOF plant models
- full-system simulation models
- control and sensor libraries
- trim, linearization, and validation scripts
- AVL integration work
- live scripts for homework/design documentation

The main use case is:

1. initialize the aircraft/scenario/workspace
2. trim the vehicle
3. linearize the plant
4. design simple controllers
5. run closed-loop Simulink demos
6. document the results in MATLAB live scripts

## 2. Big-Picture Model Architecture

There are a few layers in this repo.

### Aircraft and scenario definition

These define the physical vehicle and the starting scenario:

- [Full_Sim_Init.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Full_Sim_Init.m)
- [aircraft_def.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/aircraft_def.m)
- [scenario_def.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scenario_def.m)

`aircraft_def.m` defines:

- the component geometry and masses
- CG and inertia
- the wing and V-tail aerodynamic surfaces
- the front and rear propeller geometry
- control-surface metadata

Important physical picture from [aircraft_def.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/aircraft_def.m):

- 6 front tilting props
- 6 rear fixed props
- main wing split into left/right halves
- V-tail split into left/right halves

`Full_Sim_Init.m` exports the legacy workspace variables that the Simulink models expect, including:

- `CG`, `J`, `Mass`
- `wing`, `wingL`, `wingR`, `tailL`, `tailR`
- `prop`
- `pos_init`, `V_init`, `eul_init`, `omega_init`
- controller defaults
- sensor defaults
- AVL aero selection and bus object

### Plant models

The plant models represent the aircraft rigid-body + aero + propulsion dynamics.

Main ones in the root:

- [Brown_6DOF_Plant_EUL.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Plant_EUL.slx)
- [Brown_6DOF_Plant_zach.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Plant_zach.slx)
- [Brown_6DOF_Plant.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Plant.slx)

Current control-design and trim work has mostly centered on:

- [Brown_6DOF_Plant_EUL.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Plant_EUL.slx)

This is the Euler-angle plant used for:

- trim finding
- reduced-model linearization
- wrapper-based control demos

### Wrapper / control demo models

These wrap the plant with setpoints, sensors, and controller logic.

Important files:

- [Brown_6DOF_Sim_Wrapper.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Sim_Wrapper.slx)
- [Brown_6DOF_Sim_Wrapper_EUL.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Sim_Wrapper_EUL.slx)
- [Brown_Control_Sim.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_Control_Sim.slx)
- [Brown_Control_Analysis_Helper.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_Control_Analysis_Helper.slx)

The wrapper currently in active use for the homework/control demos is:

- [Brown_6DOF_Sim_Wrapper.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Sim_Wrapper.slx)

That wrapper now contains:

- a setpoint source path
- a command generator / setpoint combiner path
- a controller subsystem
- a sensor suite
- the plant
- logging outputs

### Libraries

Core reusable blocks live in:

- [Brown_Flight_Controls_lib.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_Flight_Controls_lib.slx)
- [Brown_Sensors_lib.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_Sensors_lib.slx)

These libraries matter because several model-level bugs we traced were actually inside library blocks, not only in the top-level wrapper.

### Full-system / legacy models

There are also full-system models that predate the more focused wrapper workflow:

- [Brown_Full_Sim.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_Full_Sim.slx)
- [Brown_Full_Sim_FlightGear.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_Full_Sim_FlightGear.slx)
- [TESTER_Brown_Full_Sim.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/TESTER_Brown_Full_Sim.slx)

Those are still useful reference points, but the recent controller homework workflow has moved toward:

- `Brown_6DOF_Plant_EUL`
- `Brown_6DOF_Sim_Wrapper`

## 3. Current Aerodynamic / Propulsion Modeling Picture

There are two aero branches in the repo:

1. analytical surface-based aero
2. AVL-based compact fitted aero

The switch is controlled by:

- `use_avl_aero`

In the current control-design and instability-debug work, the active path has been:

- analytical aero
- `use_avl_aero = false`

That matters a lot: most of the trim and controller work below is based on the analytical model path, not the AVL fit.

## 4. Most Important Debugging / Validation Progress So Far

This project started with a serious mismatch between:

- the analytical trim/stability picture
- and the Simulink plant behavior

The big question was: if the analytical model says the vehicle should be trim-stable at 70 m/s, why did the plant drift or trim poorly?

### Major issues that were found

#### A. Propeller geometry indexing bug

The front propeller group blocks were reading the prop-position matrices with the wrong row/column convention.

The core issue was:

- positions were stored row-wise in `aircraft_def.m`
- some library logic treated them as column-wise

This corrupted the thrust moment arms.

Related script:

- [scripts/simulink/patch_propeller_library_indexing.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/simulink/patch_propeller_library_indexing.m)

Supporting diagnostics:

- [scripts/trim/compare_eul_prop_groups.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/trim/compare_eul_prop_groups.m)
- [scripts/trim/compare_eul_seed_force_moment.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/trim/compare_eul_seed_force_moment.m)

#### B. Aero force/moment routing swap

At one point, aerodynamic force and moment outputs were routed into the wrong sums in the model:

- force into the moment path
- moment into the force path

That was a major plant-implementation bug and absolutely could make trim and linearization nonsense.

The user later found and fixed this inside the model wiring.

#### C. Gravity sign convention bug

This was the most important plant-level sign bug.

The repo used to have confusing gravity-sign usage:

- negative `g` in some places
- signed vectors built inconsistently in the plant

The fix was to make gravity a positive magnitude in:

- [aircraft_def.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/aircraft_def.m)

and then use explicit signed vectors in the model.

Current convention in [aircraft_def.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/aircraft_def.m):

- `g = 9.81`

This gravity fix was the key step that made the trim point and plant loads line up.

#### D. `findop` / trim feasibility cleanup

Once the major plant bugs were fixed, `findop` started behaving sensibly.

The useful scripts around that work are:

- [scripts/trim/test_findop_eul_70_localcruise.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/trim/test_findop_eul_70_localcruise.m)
- [scripts/trim/find_trim_point_eul_simple.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/trim/find_trim_point_eul_simple.m)

### Bottom-line trim result

The repo now has a consistent local cruise trim around:

- `vinf = 70 m/s`
- `theta ≈ 2.63 deg`
- `front collective ≈ 1181.4 rpm`
- `rear collective ≈ 0`
- `delta_e ≈ -15.06 deg`

This trim point is the basis for:

- the wrapper demos
- the reduced linear models
- the live-script homework outputs

## 5. Current Reduced-Order Flight-Mechanics Workflow

To support the coursework in a more textbook way, the project now has a reduced linearization path around the Euler plant:

- [scripts/control/linearize_reduced_flight_dynamics_eul.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/control/linearize_reduced_flight_dynamics_eul.m)

This script:

1. trims `Brown_6DOF_Plant_EUL`
2. linearizes the full plant
3. extracts textbook-style reduced models

The reduced states are:

- longitudinal: `[u, w, q, theta]`
- lateral: `[v, p, r, phi]`

The grouped control inputs are:

- longitudinal: `front_collective`, `rear_collective`, `delta_e`
- lateral: `delta_a`, `delta_r`

This is the model used in:

- [HW4_submission_root_locus.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_submission_root_locus.mlx)
- [HW4_submission_root_locus_live_source.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_submission_root_locus_live_source.m)

and the more detailed design notebooks:

- [HW4_root_locus_walkthrough.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_root_locus_walkthrough.mlx)
- [HW4_reduced_flight_dynamics_walkthrough.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_reduced_flight_dynamics_walkthrough.mlx)

## 6. Current Controller / Wrapper Demo Workflow

The current controller-demo workflow lives around:

- [scripts/control/setup_eul_controller_demo.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/control/setup_eul_controller_demo.m)
- [Brown_6DOF_Sim_Wrapper.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Sim_Wrapper.slx)
- [HW356_simple_demos.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW356_simple_demos.mlx)
- [HW356_simple_demos_live_source.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW356_simple_demos_live_source.m)

### What the setup script does

[setup_eul_controller_demo.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/control/setup_eul_controller_demo.m) now:

1. runs `Full_Sim_Init`
2. computes a fresh trim point
3. pushes trim values into the legacy workspace variables
4. defines grouped controller gains
5. defines sensor settings
6. defines base setpoint and command-delta structs
7. defines command profile defaults
8. defines bus objects
9. defines `truth_ic` for truth-memory initialization
10. defines `log_idx` so bus columns are decoded consistently

### Current default gains in the setup script

In the setup script itself, the current defaults are:

- `gains.V.Kp = 100`
- `gains.V.Ki = 5`
- `gains.theta.Kp = 0.10`
- `gains.theta.Kd = 0.10`
- `gains.phi.Kp = 2`
- `gains.phi.Kd = -2`

These are baseline values, not final “perfect” tuning.

### Current gain overrides in the live script

[HW356_simple_demos_live_source.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW356_simple_demos_live_source.m) now has an explicit tuning section near the top where a user can edit gains before running:

- `gains.V.Kp = 300`
- `gains.V.Ki = 5`
- `gains.theta.Kp = 0.10`
- `gains.theta.Kd = 0.10`
- `gains.phi.Kp = 2`
- `gains.phi.Kd = -2`

The live script is intentionally arranged so the gains are visible and easy to tweak for homework screenshots.

### Current control structure

The controller structure in the wrapper is now approximately:

#### Longitudinal

- airspeed PI -> front collective
- pitch-rate damper (`q`) -> elevator
- pitch-angle correction (`theta_cmd - theta`) -> elevator

Form intended:

```text
delta_e = delta_e_trim + Kd*q - Kp*(theta_cmd - theta)
```

#### Lateral

- bank-angle P -> aileron
- roll-rate damper (`p`) -> aileron

Important sign result:

- `gains.phi.Kd` must be negative in the current wrapper implementation

That sign mistake was found and corrected during debugging.

## 7. Current Sensor / Bus / Logging Structure

### Sensor config

The current sensor settings are defined in:

- [scripts/control/setup_eul_controller_demo.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/control/setup_eul_controller_demo.m)

with fields:

- `sensors.airspeed`
- `sensors.alpha`
- `sensors.beta`
- `sensors.omega`
- `sensors.accel`
- `sensors.pos`
- `sensors.vel`
- `sensors.eul`

For now:

- bias is zero
- small noise is used
- sensor sample time is `0.001`

### Logged buses

The wrapper logs at least:

- `sensor_bus`
- `plant_cmd_bus`
- truth signals such as `vinf_truth`, `eul_truth`, `omega_truth`

The log decoding is centralized through `log_idx` defined in the setup script.

Important `plant_cmd_bus` mapping:

- `1:12` = `motor_rpm_cmd`
- `13:18` = `tilt_angles_cmd`
- `19` = `front_collective_rpm`
- `20` = `rear_collective_rpm`
- `21` = `delta_f_cmd`
- `22` = `delta_a_cmd`
- `23` = `delta_e_cmd`
- `24` = `delta_r_cmd`

The live scripts now decode the buses through `log_idx`, not hard-coded magic numbers.

## 8. Truth Memory Initialization

One source of early plot ugliness was that the truth-memory blocks in the wrapper started from zeros instead of trim-like values, which caused a first-sample artifact.

To support fixing that, the setup script now creates:

- `truth_ic`

Use these names directly in the wrapper memory blocks:

- `truth_ic.pos_NED`
- `truth_ic.V_B_truth`
- `truth_ic.V_E_truth`
- `truth_ic.eul_truth`
- `truth_ic.omega_truth`
- `truth_ic.C_NB_truth`
- `truth_ic.V_BA_truth`
- `truth_ic.vinf_truth`
- `truth_ic.alpha_truth`
- `truth_ic.beta_truth`
- `truth_ic.specific_force_truth`

Current values are trim-based. For example:

- `truth_ic.vinf_truth = 70`
- `truth_ic.eul_truth = [0; 0.045946; 0]`
- `truth_ic.omega_truth = [0; 0; 0]`
- `truth_ic.V_B_truth = [69.9261; 0; 3.21508]`
- `truth_ic.alpha_truth ≈ 0.045946 rad`

## 9. What Is Working Right Now

### Working

- `Full_Sim_Init` runs
- `Brown_6DOF_Plant_EUL` trims and linearizes
- the reduced-model workflow works
- the wrapper sim runs
- the lateral controller behaves reasonably after the roll-damper sign fix
- the Homework 3 / 5 / 6 live script path runs
- setpoint and truth bus logging are working in the current wrapper flow

### Mostly working but still imperfect

- the longitudinal loop tracks a speed command, but not cleanly
- the airplane does not settle exactly at the commanded `75 m/s` after a `+5 m/s` step
- pitch angle drifts upward during the speed-hold maneuver

This is not just a plotting bug. It appears to be a real closed-loop behavior caused by longitudinal coupling.

## 10. Current Longitudinal Control Problem

The current problem is:

- command `vinf` from 70 to 75 m/s
- actual airspeed rises, but settles more like `71–73 m/s`
- `theta` slowly increases during the maneuver

Main interpretation so far:

- front collective is not acting like a pure throttle
- it also shifts pitch trim / vehicle attitude
- the speed loop is coupled to pitch
- the PI speed loop and the pitch loop are not yet balanced well enough

What was tested:

- reducing `Ki` helped a lot
- negative `Ki` clearly pushed speed the wrong way
- so the sign is probably correct and the problem is mainly tuning/coupling

Observed trend:

- larger positive `Ki` gives stronger speed tracking but worse drift
- smaller positive `Ki` reduces the drift
- larger `V.Kp` helps some, but too much also increases pitch drift

Current interpretation:

- this is a tuning / coupling problem, not a simple sign error

## 11. Live Scripts / Coursework Artifacts

Important notebooks and their roles:

- [HW4_submission_root_locus.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_submission_root_locus.mlx)
  - reduced-model root-locus notebook
- [HW4_root_locus_walkthrough.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_root_locus_walkthrough.mlx)
  - more detailed root-locus walkthrough
- [HW4_reduced_flight_dynamics_walkthrough.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_reduced_flight_dynamics_walkthrough.mlx)
  - reduced long/lat derivation workflow
- [HW356_simple_demos.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW356_simple_demos.mlx)
  - sensor checkout, longitudinal demo, lateral demo
- [HOMEWORK7_trim_linmod.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HOMEWORK7_trim_linmod.mlx)
  - trim/linearization workflow from earlier homework

Important supporting HTML/docs:

- [docs/PLANT_INSTABILITY_INVESTIGATION_2026-04-08.html](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/docs/PLANT_INSTABILITY_INVESTIGATION_2026-04-08.html)
- [docs/HW34_HW56_trace_2026-04-07.html](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/docs/HW34_HW56_trace_2026-04-07.html)
- [docs/TRIM_results.md](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/docs/TRIM_results.md)

## 12. Recommended Run Commands

### A. Basic initialization

```matlab
cd('/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim')
run('Full_Sim_Init.m')
```

### B. Current wrapper/control setup

```matlab
cd('/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim')
run('scripts/control/setup_eul_controller_demo.m')
open_system('Brown_6DOF_Sim_Wrapper')
```

### C. Run the current wrapper directly

```matlab
simOut = sim('Brown_6DOF_Sim_Wrapper', 'StopTime', '30', 'ReturnWorkspaceOutputs', 'on');
```

### D. Run the current demo notebook

Open and run:

- [HW356_simple_demos.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW356_simple_demos.mlx)

### E. Run the reduced linear-model notebook

Open and run:

- [HW4_submission_root_locus.mlx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW4_submission_root_locus.mlx)

## 13. Important Gotchas

### A. Bus typing in Simulink

Several Constant blocks and MATLAB Function outputs rely on explicit bus objects.

The setup script creates:

- `BaseSetPointBus`
- `CmdProfileBus`
- `CommandDeltaBus`
- `TruthBus`

If a model opens with bus-selector / Constant-block complaints, rerun:

- [scripts/control/setup_eul_controller_demo.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/control/setup_eul_controller_demo.m)

before debugging the model.

### B. Current wrapper is using the root-level files

There is also a nested folder:

- `flight sim EVTOL/`

That folder contains older parallel work and some related scripts, but the active work described here is in the repo root and root-level `scripts/` directory.

Be careful not to confuse:

- root-level files
- with similarly named files under `flight sim EVTOL/`

### C. The repo currently has extra local files

Current `git status` shows:

- modified:
  - `HW356_simple_demos.mlx`
  - `HW356_simple_demos_live_source.m`
- untracked:
  - `flight sim EVTOL 2.zip`
  - `flight sim EVTOL/Brown_Control_Sim.slx`
  - `flight sim EVTOL/Full_Sim_Init.m`

So the working tree is not pristine.

### D. There was a GitHub push failure caused by a too-large installer file

This file exists in history:

- `AVL software/XQuartz-2.8.5.pkg`

GitHub rejected a push because it is over 100 MB. That must be removed from local commit history before a normal push will succeed.

## 14. Suggested Starting Point For the Next Chat

If a new chat takes over, the most useful starting questions are probably:

1. Do we want to continue tuning the longitudinal controller in the wrapper?
2. Do we want to initialize the wrapper truth memories from `truth_ic` now?
3. Do we want to keep using analytical aero for control design, or switch any workflow back to AVL?
4. Do we want to clean up the repo state and push, including removing the oversized `.pkg` from history?

If the next task is controller tuning, the cleanest place to start is:

- [HW356_simple_demos_live_source.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/HW356_simple_demos_live_source.m)
- [scripts/control/setup_eul_controller_demo.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/control/setup_eul_controller_demo.m)
- [Brown_6DOF_Sim_Wrapper.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Sim_Wrapper.slx)

If the next task is plant/trimming investigation, start from:

- [scripts/trim/find_trim_point_eul_simple.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/trim/find_trim_point_eul_simple.m)
- [scripts/trim/compare_eul_total_loads_trimpoint.m](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/scripts/trim/compare_eul_total_loads_trimpoint.m)
- [Brown_6DOF_Plant_EUL.slx](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/Brown_6DOF_Plant_EUL.slx)

## 15. One-Sentence Summary

This repo is a Brown eVTOL Simulink flight-sim workspace that now has a working trim/linearization/controller-demo path centered on `Brown_6DOF_Plant_EUL` and `Brown_6DOF_Sim_Wrapper`, with major plant bugs already fixed, reduced-model control notebooks in place, and one main open issue remaining: longitudinal closed-loop tuning/coupling is still not good enough to hit the commanded steady-state airspeed cleanly.
