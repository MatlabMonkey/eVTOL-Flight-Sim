# AVL Screenshot Guide

This is a short checklist for capturing the AVL screenshots needed for the homework submission.

## What to Screenshot

You only need a few real AVL screenshots because the generated HTML report already contains the plots and tables.

Recommended screenshots:

1. AVL launched and the custom aircraft loaded
2. The AVL geometry window for the custom aircraft
3. One terminal view showing a sweep/derivative case
4. One terminal view showing a trim case

## Launch AVL

Open XQuartz first:

```bash
open -a XQuartz
```

Then start AVL from the project folder:

```bash
cd "/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/AVL software/brown_evtol"
../avl
```

## Load the Brown eVTOL Aircraft

Inside AVL:

```text
load brown_evtol
```

At that point AVL should print the aircraft summary in the terminal. This is a good first screenshot.

## Geometry Screenshot

Inside AVL:

```text
oper
g
```

This enters the geometry plot menu. At the `Geometry plot command:` prompt, press:

```text
Return
```

That should open the geometry window in XQuartz.

If you want a cleaner geometry screenshot, you can try toggling a few overlays before pressing Return:

```text
no
tr
lo
```

Then press:

```text
Return
```

Take a screenshot of the XQuartz geometry window.

## Sweep / Derivative Screenshot

Inside AVL:

```text
load brown_evtol
oper
a a 4
b b 0
x
st
```

This gives:
- executed operating point
- total forces
- stability derivatives
- control derivatives

Take a screenshot of the terminal window showing the output.

## Level-Flight Trim Screenshot

Inside AVL:

```text
load brown_evtol
oper
a c 0.53
b b 0
d2 rm 0
d3 pm 0
d4 ym 0
x
```

This is a simple explicit-`CL` trim style like the one used in the homework.

Take a screenshot of the terminal output after execution.

## Banked-Flight Trim Screenshot

Inside AVL:

```text
load brown_evtol
mass brown_evtol.mass
mset 1
oper
c1
b 15
v 70
m 3040
d 1.225
g 9.81
x -0.0002
y 0
z 0.2089

b b 0
d2 rm 0
d3 pm 0
d4 ym 0
x
```

This uses AVL's `C1` banked horizontal-flight trim style.

Take a screenshot of the terminal output after execution.

## Mac Screenshot Shortcuts

- Area screenshot: `Shift-Command-4`
- Window screenshot: `Shift-Command-4`, then press `Space`, then click the window

## Existing Report Assets

The generated homework report already includes the figures and tables from the scripted study:

- `docs/AVL_BROWN_EVTOL_HOMEWORK_2026-03-28.html`
- `docs/avl_homework/assets/`
- `docs/avl_homework/tables/`

So the AVL screenshots only need to prove that the aircraft was loaded, plotted, and used for at least one sweep and one trim case.
