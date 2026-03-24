# Suggested Figures for Progress Report

## Figure 1: Aircraft overview

Use this for a clean cruise-configuration render with vectors visible:

```matlab
render_aircraft('cruise')
```

## Figure 2: Control-surface example

Use this to show that the ruddervators and flaperons are now implemented in the geometry/visualization:

```matlab
render_aircraft('cruise', ...
    'delta_e_deg', 8, ...
    'delta_r_deg', 3, ...
    'delta_f_deg', 12, ...
    'delta_a_deg', 4)
```

## Figure 3: Updated validation case

Use this to generate the updated roll-damping plots:

```matlab
TESTER_external_forces_runner( ...
    'Cases', {'Case 4 - Roll Damping'}, ...
    'ShowRender', false, ...
    'MakePlots', true)
```

## Figure 4: Metrics example table

Use this if you want a figure or screenshot showing the transition-metrics framework:

```matlab
run('scripts/metrics/example_metrics_run.m')
```

## Figure 5: Controller block screenshot

If you want to show that a controller has actually been added to the model, open:

```matlab
open_system('Brown_Full_Sim')
```

Then zoom to the `Simple PD Controller` block in the upper-left portion of the model.
