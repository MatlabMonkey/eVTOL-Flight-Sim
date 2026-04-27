# Controllers

Last reviewed: 2026-04-27 00:00 PDT

This folder contains the active controller runtime and builder helpers.

## Current Controller Flow

```text
controller_schedule or trim_attempts + INDI map
-> controller builder
-> controllerData
-> Run_Main
```

The default controller schedule file is
`../databases/controller_schedule.mat`.

## Main Files

- `builders/build_corridor_lqr_controller.m`: builds the scheduled corridor controller.
- `builders/build_trim_lqr_controller.m`: builds one local longitudinal LQR.
- `builders/build_indi_transition_controller.m`: builds the transition path and compact INDI schedules.
- `../planning/select_corridor_path_from_db.m`: selects a path through controller DB points.
- `controller_dispatch.m`: runtime switchboard.
- `controller_lqr_path_schedule_gated.m`: active scheduled/gated runtime controller.
- `controller_indi_transition.m`: scheduled/gated longitudinal INDI transition controller.
- `presets/build_corridor_test_preset.m`: route/timing/LQR-weight preset.

## Runtime Interface

The `Wrapper/Controller/MATLAB Function` branch uses:

- `ctrlMode` to select the Simulink controller variant.
- `controller_id` to select the helper inside `controller_dispatch.m`.

Current `controller_id` values:

- `0`: trim hold / open-loop trim pass-through
- `1`: LQR hold around `controller_state_ref`
- `2`: trim-point LQR with front and rear collective
- `3`: longitudinal tracking LQR
- `4`: interpolated scheduled path LQR
- `5`: gated scheduled path LQR
- `6`: gated scheduled longitudinal INDI transition controller

## Signal Convention

Controller helpers generally use:

```text
x = [phi; theta; psi; u; v; w; p; q; r]
u = [front_collective; rear_collective; delta_f; delta_a; delta_e; delta_r]
```

Surface mixing converts mixed commands into:

```text
[deltaLW; deltaRW; deltaLT; deltaRT]
```

## Adding A Runtime Controller

1. Add a helper in `controllers/`.
2. Add a case in `controller_dispatch.m`.
3. Keep the Simulink block ports unchanged unless the whole wrapper interface changes.
4. Validate with the smallest wrapper-prep or dispatch smoke test.

Historical controller experiments live in `archive/control_experiments/`.
