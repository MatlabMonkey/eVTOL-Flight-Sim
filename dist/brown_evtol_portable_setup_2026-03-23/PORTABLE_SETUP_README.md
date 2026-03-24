# Portable Aircraft Setup Bundle

These files can be shared together as a small MATLAB-only bundle:

- `Full_Sim_Init.m`
- `aircraft_def.m`
- `scenario_def.m`
- `render_aircraft.m`

## What Works In The Portable Bundle

If these four files stay in the same folder, anyone with MATLAB can:

- initialize the aircraft and scenario workspace variables
- compute mass properties
- render the aircraft geometry
- use the render presets (`'hover'`, `'transition'`, `'cruise'`)

## How To Run

After unzipping, either:

1. open that folder in MATLAB and run:

```matlab
run('Full_Sim_Init.m')
```

or run the renderer directly:

```matlab
render_aircraft
render_aircraft('hover')
render_aircraft('transition', 45)
render_aircraft('cruise')
```

If they do **not** want to change MATLAB's Current Folder, they can still run:

```matlab
run('/path/to/unzipped/folder/Full_Sim_Init.m')
```

`Full_Sim_Init.m` adds its own folder to the MATLAB path at runtime so the
other three files are found automatically.

## Important Limitation

This small bundle does **not** include the Simulink models. It is portable for
the aircraft definition, mass-property setup, scenario setup, and
visualization. To run the full Simulink simulation, these files still need to
live alongside the actual model files in the main project.
