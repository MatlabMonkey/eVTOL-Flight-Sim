# eVTOL Simulation Workflow Architecture

Last reviewed: 2026-04-26 17:30 PDT

Status: design-intent reference. For the current implemented workflow, read
`README.md` and `docs/EV_SIMULATION_WORKFLOW_MAP.md` first.

This note captures the intended workflow architecture we discussed for the active
repo-root workspace. It is meant for teammates and future agents, and it
describes the **target design**, not necessarily every detail of the current
implementation.

## Summary

The intended workflow is a **clean, explicit four-step flow**:

```text
Init
-> Trim
-> Build_Controller
-> Run_Main
```

and then optionally:

```text
Run_Suite
```

for batch runs.

The important part is not just the names, but the **design rules** behind them.

## Core Design Rules

1. Scripts talk to each other with **structs**.
2. Simulink models get **plain workspace variables / From Workspace signals**.
3. No hidden compatibility wrappers just to paper over mismatches.
4. No abstract scenario-builder magic.
5. Cases should store **explicit numeric values**, not just symbolic labels.
6. If a helper is only used by one script, it should usually become a **local function inside that script**.

So the intended split is:

- **script-to-script**: structs
- **script-to-model**: plain arrays, scalars, timeseries, workspace vars

## Top-Level Script Roles

Each top-level script should own one job only.

### 1. `Init`

Purpose:

- load aircraft constants
- set model-wide defaults
- set neutral/default workspace values so models compile and run
- define which trim model and run model are being used

It should own things like:

- mass, inertia, geometry
- wing/tail/prop structs
- default actuator parameters
- default sensor parameters
- default placeholder command traces if the wrapper needs them

It should **not** own:

- specific trim operating points
- specific test scenarios
- controller design choices for a particular case

So `Init` is the “make the environment ready” step.

### 2. `Trim`

Purpose:

- pick an operating point
- solve trim
- optionally linearize there

It should consume:

- `initData`
- `trimCase`

It should produce:

- `trimResult`

Trim should be defined by **explicit values**, not a hidden translator.

So instead of this being the only thing that matters:

```matlab
trimCase.mode = 'cruise';
```

we want something more like:

```matlab
trimCase.name = 'Cruise75';
trimCase.mode = 'cruise';          % human-readable label only
trimCase.Vinf_mps = 75;
trimCase.frontTilt_deg = 90;       % or whatever the model convention is
trimCase.bank_deg = 0;
trimCase.psi_deg = 0;
```

So:

- `mode` is just a label
- the real work is driven by the numeric fields

This matters because the group did not like the older “case name gets translated somewhere else” pattern.

We also talked about:

- **trim model** and **run model** being allowed to be different
- trim model can be simpler
- run model is the real plant

So the intended architecture is:

- trim on a trim-friendly plant, probably Euler-based
- run on the real nonlinear plant

### 3. `Build_Controller`

Purpose:

- take a trim result
- build the controller around that operating point

It should consume:

- `trimResult`
- maybe a `controllerCase` later

It should produce:

- `controllerData`

Examples:

- LQR gains
- trim reference vectors
- scheduling metadata

Important idea:

- controller build should **use** trim
- it should not secretly re-trim on its own
- `Run_Main` should not redesign the controller every time unless that is explicitly intended

So the dependency is:

```text
trimResult -> controllerData
trimResult -> Run_Main initialization
```

### 4. `Run_Main`

Purpose:

- define the actual experiment
- initialize the nonlinear plant at trim
- apply controller + commands
- run the simulation

It should consume:

- `initData`
- `trimResult`
- optional `controllerData`
- `runCase`

It should produce:

- `runResult`

This is where the **scenario** should live.

That means:

- `Trim` owns the operating point
- `Run_Main` owns the experiment

So `runCase` is where things like these belong:

```matlab
runCase.name = 'PitchStep';
runCase.stopTime_s = 20;
runCase.useController = true;
runCase.controllerType = 'lqr';
runCase.eulerStepTime = 2.0;
runCase.eulerStepDeg = [0; 5; 0];
runCase.disturbance = 'none';
```

So:

- `Trim` answers: “what equilibrium are we around?”
- `Run_Main` answers: “what are we doing around that equilibrium?”

### 5. `Run_Suite`

Purpose:

- run many `runCase`s, usually at one trim
- or later, many trim cases and many run cases

This is optional and later.

The intended reuse pattern is:

```text
Init once
Trim once per operating point
Build_Controller once per operating point if needed
Run_Main many times
```

or later:

```text
for each trimCase
    Trim
    Build_Controller
    for each runCase
        Run_Main
```

## How Cases Should Be Stored

We explicitly said we did **not** want to start with one giant all-in-one `caseDef` for everything. That is something that might make sense later, but not at the beginning.

For now, the intended split is:

```text
cases/
  trim/
    TrimCase_Cruise75.m
    TrimCase_Hover.m

  run/
    RunCase_PrepOnly.m
    RunCase_OpenLoopMainAttempt.m
```

Each file just returns a struct.

Example trim case:

```matlab
function trimCase = TrimCase_Cruise75()
trimCase.name = 'Cruise75';
trimCase.mode = 'cruise';
trimCase.Vinf_mps = 75;
trimCase.frontTilt_deg = 90;
trimCase.bank_deg = 0;
trimCase.psi_deg = 0;
end
```

Example run case:

```matlab
function runCase = RunCase_PitchStep()
runCase.name = 'PitchStep';
runCase.stopTime_s = 20;
runCase.useController = true;
runCase.eulerStepTime = 2.0;
runCase.eulerStepDeg = [0; 5; 0];
end
```

So the storage idea is:

- **one file per trim case**
- **one file per run case**
- maybe later **controller cases** too, if needed

This keeps iteration easy:

- change trim cases independently
- change run cases independently
- don’t bundle everything too early

## The Intended Handoff Structs

We talked about a small set of standard structs.

### `initData`

Holds:

- aircraft constants
- model names
- default settings
- default initial conditions/placeholders

### `trimCase`

Holds:

- explicit operating-point values
- speed
- tilt
- bank
- heading
- maybe altitude and climb rate later

### `trimResult`

Holds:

- trim state
- trim inputs
- linear model if produced
- metadata about the trim point

We specifically discussed fields like:

- attitude
- body velocity
- rates
- grouped motor values
- grouped tilt values
- surface deflections

### `controllerData`

Holds:

- gains
- trim references used by the controller
- controller type / metadata

### `runCase`

Holds:

- stop time
- step commands
- disturbances
- controller on/off
- controller selection
- run label

### `runResult`

Holds:

- sim output
- metadata
- summary metrics if desired

So the clean handoff chain is:

```text
Init        -> initData
Trim        -> trimResult
Build_Controller -> controllerData
Run_Main    -> runResult
```

with `trimCase` and `runCase` injected explicitly by the user.

## How Models Should Be Fed

The intended rule is:

- **Scripts** should pass around structs.
- **Simulink** should still get plain old workspace variables and `From Workspace` data.

So for the model side, the script can publish things like:

- `cmds`
- `EVTOL`
- `controller_mode`
- `K_lqr_cruise`
- `x_trim_lqr`
- `U_trim_lqr`
- `V_init`
- `eul_init`
- `omega_init`

That is acceptable.

What we did **not** want was script A secretly depending on many loose workspace variables from script B with no structure at all.

So the intended design is:

- loose workspace vars are acceptable at the **final model boundary**
- not as the main scripting architecture

## What We Explicitly Wanted To Avoid

We talked pretty clearly about avoiding these:

- hidden compatibility wrappers
- old scenario-builder machinery
- case-name translation logic hidden in helper scripts
- giant helper jungles where ownership is unclear
- scripts silently inventing operating points or controller assumptions
- making everything “robust” to bad inputs if that just makes the system harder to understand

The preference was:

- better to be simple and explicit
- better to break loudly if someone gives the wrong input
- this is a simulation environment, not a consumer web app

So the target architecture is intentionally **not** over-defensive.

## Convenience Wrappers

We did say it makes sense to have **one convenience script later**, but not as the core logic.

Something like:

```text
Run_All
```

that would do:

```text
Init
Trim
Build_Controller
Run_Main
```

But that should sit **on top of** the clean four-script architecture, not replace it.

And we also said:

- `Run_Suite` is the place for repeated experiments
- not `Init`
- not `Trim`

## Hover vs Cruise

Hover and cruise are really **trim definitions**, not separate architectures.

We said:

- hover/cruise should be represented by explicit trim values, especially tilt angle
- `mode` can exist as a label, but it should not be the only thing the scripts rely on

Examples:

```matlab
trimCase.mode = 'hover';
trimCase.frontTilt_deg = 90;
trimCase.Vinf_mps = 0;
```

or

```matlab
trimCase.mode = 'cruise';
trimCase.frontTilt_deg = 0;   % or 90, depending your final convention
trimCase.Vinf_mps = 75;
```

Again:

- explicit values first
- labels second

## Trim Plant vs Run Plant

We also settled on this idea:

- **Trim / linearization plant**
  - can be simpler
  - usually Euler states
  - can omit things that make trimming harder

- **Run plant**
  - the real nonlinear plant
  - can include more realism, actuator dynamics, etc.

So the architecture is intentionally:

```text
one init
two plants if useful
one trim script
one controller build script
one run script
```

Not “one model must do everything.”

## Intended Folder-Level Shape

At the cleanest level, this is roughly what we had in mind:

```text
eVTOL-Flight-Sim/
  Init_Main.m
  Trim_Main.m
  Build_Controller_EVTOL_Cruise.m
  Run_Main.m
  Run_EVTOL_Suite.m

  cases/
    trim/
      TrimCase_Cruise75.m
      TrimCase_Hover.m
    run/
      RunCase_PrepOnly.m
      RunCase_PitchStep.m
```

And then only a small number of real reusable helpers outside that, such as:

- `aircraft_def.m`
- `render_aircraft.m`

Everything else should either:

- be local to one top-level script, or
- only exist if it’s a genuinely reusable domain helper

## One-Sentence Summary

Use a simple four-step workflow (`Init`, `Trim`, `Build_Controller`, `Run_Main`), store trim cases and run cases as explicit struct-returning functions, pass structs between scripts, and only convert to plain workspace variables at the Simulink model boundary.
