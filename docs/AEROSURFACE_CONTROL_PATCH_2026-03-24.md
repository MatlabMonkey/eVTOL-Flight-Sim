# AeroSurface Control Patch

This note gives a paste-ready first-pass control-surface upgrade for
`Brown_Flight_Controls_lib/aeroSurface`.

The goal is:

- keep one `aeroSurface` block per lifting surface
- add one scalar control-deflection input to each block
- let the same block work for wing flaperons and V-tail ruddervators

## What To Change In The Library Block

Inside `Brown_Flight_Controls_lib/aeroSurface`:

1. Add a third subsystem input named `delta_ctrl`.
2. Connect that new input to the Stateflow chart / MATLAB chart block.
3. Update the Stateflow chart input data list to include:
   - `delta_ctrl` as `INPUT_DATA`
4. Update the chart function signature to:

```matlab
function [F_cg, M_cg] = aeroSurface(v_body, omega, delta_ctrl, surf, CG)
```

You do **not** need a new mask parameter if you do it this way. Keep the mask parameters as:

- `CG`
- `surf`

The change is a new **input port**, not a new mask parameter.

## Paste-Ready Chart Code

Paste this into the `aeroSurface` chart:

```matlab
function [F_cg, M_cg] = aeroSurface(v_body, omega, delta_ctrl, surf, CG)
if nargin < 3 || isempty(delta_ctrl)
    delta_ctrl = 0;
end

if norm(v_body) < 0.1
    F_cg = [0;0;0];
    M_cg = [0;0;0];
    return;
end

% Optional control-effect fields so older surface structs still work.
ctrl_tau = 0;
CM_delta = 0;
CD_delta2 = 0;
delta_max = inf;

if isfield(surf, 'ctrl_tau') && ~isempty(surf.ctrl_tau)
    ctrl_tau = surf.ctrl_tau;
end
if isfield(surf, 'CM_delta') && ~isempty(surf.CM_delta)
    CM_delta = surf.CM_delta;
end
if isfield(surf, 'CD_delta2') && ~isempty(surf.CD_delta2)
    CD_delta2 = surf.CD_delta2;
end
if isfield(surf, 'delta_max') && ~isempty(surf.delta_max)
    delta_max = surf.delta_max;
end

% Saturate local surface deflection.
delta_ctrl = min(max(delta_ctrl, -delta_max), delta_max);

r_arm = surf.pos - CG;

% Local flow at the surface reference point.
v_local = v_body + cross(omega, r_arm);
V2 = sum(v_local.^2);
v_mag = sqrt(V2);
v_dir = v_local / v_mag;

% Clamp the asin argument for numerical safety.
normal_proj = dot(v_dir, surf.n);
normal_proj = min(max(normal_proj, -1), 1);

% The control surface shifts the effective incidence of the whole lifting
% surface as a simple first-pass model.
alpha_geom = surf.i - asin(normal_proj);
alpha_eff = alpha_geom + ctrl_tau * delta_ctrl;

qS = surf.half_rho_S * V2;

CL = surf.CL0 + surf.CLa * alpha_eff;
CD = surf.CD0 + surf.CDa * (alpha_eff - surf.a0)^2 + CD_delta2 * delta_ctrl^2;
CM = surf.CM0 + surf.CMa * alpha_eff + CM_delta * delta_ctrl;

L = qS * CL;
D = qS * CD;

dir_D = -v_dir;

% Lift direction remains based on the parent surface normal.
dir_L = surf.n - dot(surf.n, v_dir) * v_dir;
if norm(dir_L) > 0
    dir_L = dir_L / norm(dir_L);
else
    dir_L = surf.n;
end

F_surf = L * dir_L + D * dir_D;

m_axis = cross(surf.n, v_dir);
if norm(m_axis) > 0
    m_axis = m_axis / norm(m_axis);
end
M_surf = (qS * surf.c * CM) * m_axis;

F_cg = F_surf;
M_cg = M_surf + cross(r_arm, F_surf);
end
```

## New Surface-Struct Fields

Each `surf` struct should now carry these optional fields:

- `ctrl_tau`
  - effective angle-of-attack change per radian of local control deflection
- `CM_delta`
  - direct pitching-moment change per radian of local control deflection
- `CD_delta2`
  - added drag term per `delta^2`
- `delta_max`
  - local control-deflection limit in radians

## Recommended Values For This Repo

These match the current simplified Archer-style setup:

### Wing Flaperons

- `ctrl_tau = 0.55`
- `CM_delta = -0.35`
- `CD_delta2 = 0.30`
- `delta_max = deg2rad(25)`

Why:

- `ctrl_tau = 0.55` matches the flaperon effectiveness used in `aircraft.controls`
- `CM_delta < 0` gives the expected nose-down tendency when both flaps go down
- `CD_delta2 = 0.30` adds visible but not excessive drag from flap deflection

### V-Tail Ruddervators

- `ctrl_tau = 0.45`
- `CM_delta = 0.00`
- `CD_delta2 = 0.10`
- `delta_max = deg2rad(25)`

Why:

- most of the ruddervator pitch/yaw authority already comes from the tail
  force acting through the tail moment arm
- the direct airfoil pitching moment is less important in this simple model
- the small `CD_delta2` term adds some deflection drag without overpowering
  the main tail-force effect

## Top-Level Mixing You Should Wire In

Use these four high-level control commands, all in radians:

- `delta_f` : symmetric flap command
- `delta_a` : differential roll command
- `delta_e` : symmetric ruddervator / elevator command
- `delta_r` : differential ruddervator / rudder command

Mix them into local surface deflections like this:

```matlab
delta_wing_L = delta_f + delta_a;
delta_wing_R = delta_f - delta_a;
delta_tail_L = delta_e - delta_r;
delta_tail_R = delta_e + delta_r;
```

Recommended sign convention:

- `delta_f > 0` => both flaperons trailing-edge down
- `delta_a > 0` => roll-right command
- `delta_e > 0` => both ruddervators trailing-edge down
- `delta_r > 0` => yaw-right command

Then connect:

- `delta_wing_L` to the `Left Wing` `aeroSurface`
- `delta_wing_R` to the `Right Wing` `aeroSurface`
- `delta_tail_L` to the `Left Tail` `aeroSurface`
- `delta_tail_R` to the `Right Tail` `aeroSurface`

## Exact `aircraft_def.m` Fields To Add

Set these after building `wingL`, `wingR`, `tailL`, and `tailR`:

```matlab
wingL.ctrl_tau = controls.flaperon.effectiveness_tau;
wingR.ctrl_tau = controls.flaperon.effectiveness_tau;
tailL.ctrl_tau = controls.ruddervator.effectiveness_tau;
tailR.ctrl_tau = controls.ruddervator.effectiveness_tau;

wingL.CM_delta = -0.35;
wingR.CM_delta = -0.35;
tailL.CM_delta = 0.00;
tailR.CM_delta = 0.00;

wingL.CD_delta2 = 0.30;
wingR.CD_delta2 = 0.30;
tailL.CD_delta2 = 0.10;
tailR.CD_delta2 = 0.10;

wingL.delta_max = controls.flaperon.max_deflection_rad;
wingR.delta_max = controls.flaperon.max_deflection_rad;
tailL.delta_max = controls.ruddervator.max_deflection_rad;
tailR.delta_max = controls.ruddervator.max_deflection_rad;
```

## Why This Is The Right Level Of Complexity

This patch is intentionally simple:

- it gives each surface real control authority
- it preserves the current block structure
- it makes left/right split-wing control work cleanly
- it avoids a much larger rewrite into separate fixed-surface and
  control-panel aero blocks

It is still an approximation:

- it does not model hinge moments
- it does not model downwash changes from flap deflection
- it does not rotate the whole lift direction with the control panel
- it does not model spoiler mode yet

For this project, that is a good trade.
