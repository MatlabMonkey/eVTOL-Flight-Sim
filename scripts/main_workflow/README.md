# Main Workflow

This folder is the new clean workflow for running the **main plant model** and trimming with a separate **Euler-angle trim backend**.

The goal is to give the project one readable, portable path that your group can follow without having to understand the older wrapper/helper chain.

## Design

- Init script: `Brown_6DOF_Init_main.m`
- Trim backend copy: `models/Brown_6DOF_Plant_EUL_main_workflow.slx`
- Run backend copy: `models/Brown_6DOF_Plant_main_main_workflow.slx`
- Canonical command/state convention:
  - `Motor_RPMs_grouped = [FR; FL; RR; RL]`
  - `Tilt_angles_grouped = [FR; FL]`

These are copies of the repo-root Simulink plants, placed here so the main workflow bundle has its own local model files.

The trim model and the run model are intentionally different:

- The trim backend is simpler and uses Euler states.
- The run backend is the real plant you want to study going forward.
- The workflow adapts trim results onto the run model by seeding:
  - plant initial conditions
  - rotor and tilt actuator initial states
  - physical wing/tail surface actuator initial states

## Run Tree

```text
prepare_main_session
|
|-- Brown_6DOF_Init_main.m
|-- load trim model copy (Brown_6DOF_Plant_EUL_main_workflow)
`-- load run model copy  (Brown_6DOF_Plant_main_main_workflow)

resolve_main_trim
|
|-- source = saved    -> read Homework 7 trim CSV
|-- source = fresh    -> run findop on Brown_6DOF_Plant_EUL_main_workflow
`-- source = provided -> normalize caller-supplied trim struct

adapt_trim_to_run_model
|
|-- convert grouped trim into run-model initial conditions
|-- seed rotor/tilt actuator states
`-- seed physical wing/tail actuator states

publish_main_workspace
|
`-- write the trimmed run context into the base workspace

run_main_case
|
|-- build root-inport command signals
|-- update Brown_6DOF_Plant_main_main_workflow
`-- simulate if the main model is currently runnable
```

## Public Entry Points

- `prepare_main_session.m`
  - Run this once to initialize the workspace and load both models.
- `resolve_main_trim.m`
  - Use this when you want a saved trim, a fresh trim, or to normalize a trim struct.
- `run_main_case.m`
  - One-call single-case runner.
- `run_main_sweep.m`
  - Reuses trim points across multiple run cases.
- `example_main_case.m`
  - Small example your group can copy.
- `smoke_test_main_workflow.m`
  - Quick readiness check before larger studies.

## One-Case Example

```matlab
session = prepare_main_session('RenderEnable', false);

trimSpec = struct( ...
    'source', 'saved', ...
    'cruiseSpeedMps', 70, ...
    'bankDeg', 0);

runSpec = struct( ...
    'name', 'small elevator step', ...
    'stopTime', 8.0, ...
    'stepTime', 1.0, ...
    'deltaEStep', deg2rad(1.0));

result = run_main_case( ...
    'Session', session, ...
    'TrimSpec', trimSpec, ...
    'RunSpec', runSpec);
```

If `Brown_6DOF_Plant_main_main_workflow` still has unresolved Simulink issues, `run_main_case` will stop with a clear diagnostic **after** it has already prepared the trimmed workspace context. The last published run context is left in the base workspace as:

- `main_workflow_last_runCtx`
- `main_workflow_last_trimCtx`
- `main_workflow_last_result`

## Sweep Example

Use cell arrays when cases do not all share the same fields.

```matlab
trimCases = {
    struct('source', 'saved', 'cruiseSpeedMps', 70)
    struct('source', 'saved', 'cruiseSpeedMps', 85)
};

runCases = {
    struct('name', 'trim hold', 'runSimulation', false)
    struct('name', 'pitch step', 'runSimulation', false, ...
        'deltaEStep', deg2rad(1.0))
};

results = run_main_sweep( ...
    'TrimCases', trimCases, ...
    'RunCases', runCases);
```

## Smoke Test

```matlab
report = smoke_test_main_workflow;
```

The smoke test checks:

- session preparation
- saved-trim normalization
- fresh-trim solving on `Brown_6DOF_Plant_EUL_main_workflow`
- workspace publishing for the main plant
- load/update readiness of `Brown_6DOF_Plant_main_main_workflow`

## Important Notes

- `Brown_6DOF_Init_main.m` is the only init script used by this workflow.
- The old `scripts/control/*` and `scripts/trim/*` helpers remain in the repo as references, but this folder is the authoritative path for future work.
- The current run model may still fail to update because of unresolved Simulink mask issues. The workflow is written to make that failure easy to diagnose rather than hiding it.
