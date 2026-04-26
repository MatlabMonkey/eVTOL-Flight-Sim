# AGENTS.md

## Repository purpose

This repository is a MATLAB/Simulink eVTOL flight simulation workspace with:
- top-level initialization in `Full_Sim_Init.m`
- primary full-model entry in `Brown_Full_Sim.slx`
- alternate plant/wrapper paths in:
  - `Brown_6DOF_Plant.slx`
  - `Brown_6DOF_Plant_EUL.slx`
  - `Brown_6DOF_Plant_zach.slx`
  - `Brown_Control_Sim.slx`
  - `Brown_6DOF_Sim_Wrapper.slx`
- trim, control, sensor, validation, and Simulink-builder scripts under `scripts/`

Treat `.slx` files as source artifacts, not disposable generated files.

## Primary working path

Unless the user says otherwise, assume this workflow:

1. Read `Full_Sim_Init.m`
2. Identify which model/script path the task touches
3. Prefer the smallest change that solves the task
4. Validate with the lightest relevant check
5. Summarize exactly what changed, what was verified, and what remains uncertain

Default first-run validation order:
1. `Full_Sim_Init.m`
2. `scripts/smoke_test.m`
3. task-specific script or model
4. only then broader demos, trim, or control-design scripts

## Think before coding

Do not guess silently.

Before implementing:
- State assumptions explicitly.
- If something is unclear, say exactly what is unclear.
- If there are multiple plausible interpretations, present them instead of choosing silently.
- If a simpler approach exists, say so.
- Push back on unnecessary complexity.
- If the task is ambiguous in a way that materially affects correctness, ask.

For nontrivial tasks, give a brief plan in this format:

1. [Step] -> verify: [check]
2. [Step] -> verify: [check]
3. [Step] -> verify: [check]

## Simplicity first

Write the minimum code that solves the requested problem.

Rules:
- No speculative features.
- No abstractions for one-off code.
- No configurability unless requested.
- No defensive code for impossible scenarios.
- No large framework-style rewrites for small tasks.
- If 50 lines can do the job, do not write 200.

Always ask:
- Is this the simplest thing that works?
- Would a senior engineer say this is overcomplicated?

If yes, simplify.

## Surgical changes

Touch only what is necessary for the user's request.

When editing existing code:
- Do not refactor unrelated code.
- Do not restyle unrelated code.
- Do not rename unrelated variables or files.
- Match the existing local style, even if it is not ideal.
- If unrelated dead code is noticed, mention it in the summary but do not remove it unless asked.

Clean up only things made unused by your own change:
- imports
- local helper functions
- local variables
- small comments that became wrong because of your change

Every changed line should trace directly to the task.

## Goal-driven execution

Convert vague requests into verifiable outcomes.

Examples:
- "Fix the bug" -> reproduce it, change code, verify the failure is gone
- "Add validation" -> add or run checks that demonstrate invalid cases are handled as intended
- "Refactor X" -> preserve behavior before and after, and say how you checked

Do not stop at "code written".
Stop at "behavior checked" when possible.

## MATLAB / Simulink repo rules

This repo is stateful and workspace-driven. Be careful.

- Read the relevant initialization path before editing.
- Expect hidden dependencies through the MATLAB base workspace.
- Do not rename workspace variables unless explicitly requested.
- Do not rename Simulink models unless explicitly requested.
- Do not convert script-based flows to classes/config systems unless explicitly requested.
- Preserve model names and file locations when possible.
- Be cautious with `assignin`, `evalin`, `load_system`, `set_param`, and programmatic model generation.
- Be explicit about any required current-directory assumptions.
- Call out missing files, toolboxes, generated artifacts, or external executables immediately.

## Simulink editing rules

When touching `.slx`-related logic:
- Prefer changing the MATLAB script that builds/configures the model if that is clearly the intended source of truth.
- If editing generated or derived model artifacts, say so explicitly.
- Do not silently assume one model variant is canonical if multiple variants exist.
- If a task depends on choosing between `Brown_Full_Sim`, `Brown_Control_Sim`, `Brown_6DOF_Sim_Wrapper`, `Brown_6DOF_Plant_EUL`, or `Brown_6DOF_Plant_zach`, state which one you are using and why.

## High-risk areas

Flag these carefully in your summary if touched:
- sign conventions
- control mixing
- propeller indexing
- trim assumptions
- inertial properties
- force/moment calculations
- sensor frame conventions
- estimator conventions
- grouped vs expanded actuator commands
- AVL integration and optional aero branches

## Validation expectations

Use the lightest relevant validation that matches the task.

Examples:
- small MATLAB helper change -> run the narrowest script that exercises it
- trim logic change -> run the affected trim script and inspect residuals/output reasonableness
- controller change -> run the smallest relevant demo or analysis script
- parser/docs-only change -> no simulation required; say that explicitly

If you could not validate, say exactly why:
- missing MATLAB
- missing toolbox
- missing local asset
- missing generated file
- ambiguous canonical model path

## Output expectations

When you finish a task, provide:
1. what you changed
2. why you changed it
3. what you verified
4. what you did not verify
5. risks or assumptions

If there were multiple plausible approaches, mention the rejected alternatives briefly.

## What not to do

- Do not silently invent missing files or variables.
- Do not hide uncertainty.
- Do not broaden scope.
- Do not rewrite large sections unless asked.
- Do not "clean up" unrelated technical debt.
- Do not assume a task requires extra architecture.

## Practical repo landmarks

Read these first for most tasks:
- `Full_Sim_Init.m`
- `scripts/smoke_test.m`
- `scripts/control/setup_eul_controller_demo.m`
- `scripts/trim/find_trim_point_eul_simple.m`
- `scripts/control/linearize_cruise_plant.m`
- `wrapper_controller_actual_cmd_step.m`

Important model/library files:
- `Brown_Flight_Controls_lib.slx`
- `Brown_Full_Sim.slx`
- `Brown_6DOF_Plant.slx`
- `Brown_6DOF_Plant_EUL.slx`
- `Brown_6DOF_Plant_zach.slx`
- `Brown_Control_Sim.slx`
- `Brown_6DOF_Sim_Wrapper.slx`

## Default stance

Be skeptical, precise, and conservative.
Prefer understanding first, then minimal change, then verification.