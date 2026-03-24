# Control Surface Research Notes

## What Archer Midnight Appears To Have

The strongest Archer-specific source I found is Archer's flight-control patent:

- Archer's patent says the aircraft control surfaces include `flaperons` and `ruddervators`.
- It says `flaperons` combine flap, aileron, and spoiler functions.
- It says `ruddervators` combine rudder and elevator functions.
- The patent figures describe `four flaperons` and `six ruddervators` in one example architecture.

For this repo, the safest simplification is:

- keep the current two-piece V-tail,
- model one ruddervator on each V-tail half,
- model one flaperon on each wing half.

That simplification is now stored in `aircraft.controls`.

## Why The V-Tail Ruddervator Signs Look The Way They Do

MIT's aircraft stability notes give the standard fixed-wing sign convention:

- positive elevator deflection `delta_e` means trailing-edge down,
- `Cm_delta_e` is expected to be negative,
- elevator deflection changes both lift and pitching moment.

Applied to this V-tail:

- common ruddervator deflection behaves like an elevator,
- differential ruddervator deflection behaves like a rudder,
- because each tail half is canted at `45 deg`, each local tail force has both vertical and lateral components.

## First-Pass Sim Model Recommendation

For the current `aeroSurface` block, the cleanest first patch is:

```matlab
i_eff = i0 + tau * delta_local
```

with:

- `tau = 0.45`
- positive `delta_local` = trailing-edge down
- standard mixed commands:

```text
delta_L = delta_e - delta_r
delta_R = delta_e + delta_r
```

where:

- `delta_e > 0` means pitch-down command
- `delta_r > 0` means yaw-right command

If you want pilot-style commands instead:

```text
delta_L = -pitch_cmd_nose_up - yaw_cmd_nose_right
delta_R = -pitch_cmd_nose_up + yaw_cmd_nose_right
```

## Expected Coefficient Effects For The Current Geometry

These are not Archer-certified values. They are geometry-based estimates derived from the current tail area, tail arm, and the simple `aeroSurface` model in this repo.

| Quantity | Value | Meaning |
| --- | ---: | --- |
| `local_tail_CL_delta` | `0.991 / rad` | Local tail lift-coefficient change per ruddervator deflection |
| `CZ_delta_e` | `-0.209 / rad` | Common trailing-edge-down ruddervator deflection adds upward tail force |
| `CM_delta_e` | `-0.487 / rad` | Common trailing-edge-down ruddervator deflection gives nose-down moment |
| `CY_delta_r` | `-0.209 / rad` | Positive yaw-right rudder command gives left side force at the tail |
| `CN_delta_r` | `+0.059 / rad` | Positive yaw-right rudder command gives positive yawing moment |
| `CL_delta_r` | `-0.012 / rad` | Small roll coupling from the vertical offset of the V-tail |

## Wing Flaperon Recommendation

The current repo now includes simple one-per-wing flaperon metadata and visualization, but it still does not feed those deflections into the `aeroSurface` block.

The expected trends are:

- symmetric flap mode: increase `CL0`, increase `CD0`, and usually make `CM0` more nose-down
- differential flaperon mode: create roll moment `Cl` and some adverse yaw `Cn`
- spoiler-like mode: primarily dump lift and add drag

The simple command convention used in the renderer/metadata is:

- `delta_f > 0`: both flaperons trailing-edge down
- `delta_a > 0`: roll-right command, left flaperon down and right flaperon up

## Files Updated In This Patch

- `aircraft_def.m`
  - adds `aircraft.controls.ruddervator`
  - adds `aircraft.controls.flaperon`
  - computes first-pass coefficient estimates from the current geometry
- `Full_Sim_Init.m`
  - exports `controls = aircraft.controls` into the workspace

## Sources

- Archer patent: [US20240400200A1](https://patents.google.com/patent/US20240400200A1/en)
- Archer official transition announcement: [Archer Completes Midnight's Transition Flight](https://investors.archer.com/news/news-details/2024/Archer-Completes-Midnights-Transition-Flight/default.aspx)
- MIT OCW stability notes: [16.333 Lecture 2](https://ocw.mit.edu/courses/16-333-aircraft-stability-and-control-fall-2004/resources/lecture_2/)
