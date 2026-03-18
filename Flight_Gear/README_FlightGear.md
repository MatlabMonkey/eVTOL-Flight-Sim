# FlightGear Integration

The primary integration path in this folder uses MathWorks FlightGear support that is already installed with your MATLAB toolboxes. The professor-provided `sfun_FlightGearSender.cpp` path is left in place as reference, but it is Windows-specific and is not the active macOS path.

## Files

- `fg_config.m`: editable project settings
- `fg_build_array6dof.m`: converts logged `positionLLA` and `eul` into FlightGear `Array6DoF`
- `fg_play_array6dof.m`: plays an existing `[time lat lon alt phi theta psi]` matrix
- `fg_replay_from_out.m`: replays directly from an existing `out`/`simOut` object
- `fg_replay_from_sim.m`: runs the model and replays it in FlightGear
- `setup_flightgear_interface.m`: creates a live-streaming subsystem in `Brown_Full_Sim_FlightGear.slx`

## Replay workflow

1. Open `Brown_HW6/fg_config.m`.
2. Set `cfg.flightGearBaseDirectory` if autodetection does not find your FlightGear install.
3. In MATLAB, run:

```matlab
addpath("Brown_HW6");
cfg = fg_config();
fg_replay_from_sim(cfg);
```

This will:

- simulate `Brown_Full_Sim`
- read `positionLLA` and `eul`
- generate a FlightGear launch script
- initialize and play the replay over UDP

## Replay from an existing output object

If another script already ran the simulation and you have an output object, you can replay without simulating again:

```matlab
addpath("Brown_HW6");
cfg = fg_config();
out = sim("Brown_Full_Sim");
fg_replay_from_out(out, cfg);
```

If you already have an Array6DoF matrix, you can pass it straight in:

```matlab
addpath("Brown_HW6");
cfg = fg_config();
fg_play_array6dof(array6dof, cfg);
```

## Live model copy

To add a live FlightGear subsystem without touching the original model:

```matlab
addpath("Brown_HW6");
cfg = fg_config();
setup_flightgear_interface(cfg);
open_system(cfg.modelCopyName);
```

This updates only `Brown_Full_Sim_FlightGear.slx`.

## Notes

- `positionLLA` is expected to be `[lat; lon; alt]`.
- The live FlightGear block expects `[lon; lat; alt; phi; theta; psi]`, so the model copy adds that reorder internally.
- If the FlightGear library path changes across MATLAB releases, update `localResolveFlightGearBlock` in `setup_flightgear_interface.m`.
