# Trim Exploration Notes

Last reviewed: 2026-04-25 17:03 PDT

Status: historical reference. These notes describe verified trim discoveries
from earlier exploration, not the current canonical search workflow. For the
current workflow, use `TRIM_DATABASES.md` and
`docs/EV_SIMULATION_WORKFLOW_MAP.md`.

Last content update: 2026-04-20

This file keeps quick engineering notes about trim points we have actually
verified in the current `Trim_Plant` workflow. The goal is to build up a
reliable map before doing larger trim sweeps.

## Current Reference Case

Primary exact cruise case:

- `TrimCase_Cruise75_FlapElevator()`
- symmetric mixed-control trim
- free: `front collective`, `delta_f`, `delta_e`
- fixed: `delta_a = 0`, `delta_r = 0`
- rear collective can be pinned and the rest of the trim rebalances around it

## Exact 75 m/s Cruise Family at 90 deg Tilt

All of these trimmed exactly with `TrimCase_Cruise75_FlapElevator()` at
`front_tilt_deg = 90 deg`.

| rear collective (rpm) | success | front collective (rpm) | delta_f (deg) | delta_e (deg) | theta (deg) | w (m/s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 0 | yes | 808.896 | +0.794 | -4.353 | +0.006 | +0.0075 |
| 200 | yes | 808.171 | +0.776 | -4.455 | +0.000 | +0.0004 |
| 500 | yes | 804.572 | +0.603 | -5.070 | +0.018 | +0.0231 |

Observation:

- There is a real local trim family in `rear_collective_fixed_rpm` at the
  nominal 75 m/s cruise point.
- As rear collective increases, front collective comes down a bit and elevator
  works harder.

## Exact 75 m/s Family at 45 deg Tilt, Rear Fixed

At `front_tilt_deg = 45 deg` and `V = 75 m/s`, we also found an exact family
when rear collective is fixed and the case is seeded appropriately.

Verified exact trims:

| rear collective (rpm) | success | front collective (rpm) | delta_f (deg) | delta_e (deg) | theta (deg) | w (m/s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 0 | yes | 922.992 | -1.425 | -2.796 | -0.000 | -0.0003 |
| 100 | yes | 922.951 | -1.436 | -2.837 | -0.000 | -0.0004 |
| 200 | yes | 922.849 | -1.471 | -2.963 | +0.001 | +0.0013 |
| 300 | yes | 922.760 | -1.536 | -3.178 | +0.007 | +0.0093 |
| 400 | yes | 922.474 | -1.609 | -3.461 | +0.004 | +0.0055 |
| 500 | yes | 922.258 | -1.718 | -3.794 | +0.009 | +0.0118 |
| 600 | yes | 921.463 | -1.805 | -4.108 | -0.010 | -0.0132 |
| 700 | yes | 919.118 | -1.753 | -4.381 | -0.077 | -0.1004 |

Observation:

- The 45 deg point is not just a one-off at rear `500 rpm`.
- At 75 m/s, there is also a rear-collective trim family at 45 deg.
- Relative to the 90 deg cruise family, the 45 deg family needs
  substantially more front collective and more negative flap.

## Tilt Continuation at 75 m/s, Rear Fixed 500 rpm

Using continuation from the exact 90 deg / rear 500 rpm trim:

| tilt (deg) | success | front collective (rpm) | delta_f (deg) | delta_e (deg) | theta (deg) | w (m/s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 90 | yes | 804.473 | +0.633 | -5.041 | +0.001 | +0.0006 |
| 80 | yes | 801.636 | +0.320 | -4.721 | +0.006 | +0.0080 |
| 70 | yes | 811.217 | +0.026 | -4.373 | -0.002 | -0.0027 |
| 60 | no | near 70-deg seed | not exact | not exact | not exact | not exact |
| 50 | no | near 70-deg seed | not exact | not exact | not exact | not exact |
| 45 | no from that continuation seed | nearby point found | not exact | not exact | not exact | not exact |

Observation:

- The exact family continued cleanly from 90 deg down to 70 deg.
- It broke between 70 deg and 60 deg when approached from that cruise-family
  seed.
- That does not mean 45 deg is impossible. It means the 45 deg solution is in
  a different basin than the 70-to-90 cruise continuation family.

## Speed Continuation at 45 deg Tilt, Rear Fixed 500 rpm

Using the exact 45 deg / 75 m/s solution as the seed and then stepping speed
down:

| tilt (deg) | rear collective (rpm) | speed (m/s) | success | notes |
| --- | ---: | ---: | --- | --- |
| 45 | 500 | 75 | yes | exact trim found |
| 45 | 500 | 70 | no | residuals remain, especially vertical force balance |
| 45 | 500 | 65 | no | same trend, no exact trim from this formulation/seed |
| 45 | 500 | 60 | no | same trend |
| 45 | 500 | 55 | no | same trend |
| 45 | 500 | 50 | no | same trend |
| 45 | 500 | 45 | no | same trend |

Observation:

- At 45 deg tilt, the exact family does not continue downward in speed the way
  it does in rear collective.
- Right now, `45 deg + 75 m/s` looks like a narrow speed pocket, not a broad
  speed family.

## Seed Sensitivity Notes

45 deg trim is much more seed-sensitive than the 90 deg cruise case.

Important successful seed for the 45 deg / 75 m/s family:

- `front_collective_guess_rpm = 1000`
- `rear_collective_fixed_rpm = 500`
- `delta_f_guess_deg = -0.8`
- `delta_e_guess_deg = -3.2`

By contrast, several nearby seeds at the same operating condition fail and
return locally infeasible points.

## Practical Takeaways

1. `TrimCase_Cruise75_FlapElevator()` is the current best baseline case for
   exact cruise trims.
2. Rear collective behaves like a real schedule parameter at both:
   - `90 deg tilt, 75 m/s`
   - `45 deg tilt, 75 m/s`
3. The `45 deg` exact point exists, but it is not simply the same trim family
   continued from the `70-90 deg` cruise family.
4. The next high-value sweep is probably:
   - fixed `tilt = 45 deg`
   - fixed `rear collective`
   - sweep speed around `75 m/s` more finely, or
   - fixed `tilt = 45 deg`, `speed = 75 m/s`
   - sweep rear collective over a broader range until the family breaks
