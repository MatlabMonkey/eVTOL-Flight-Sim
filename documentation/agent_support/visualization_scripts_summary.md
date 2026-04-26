# Visualization Scripts Summary

This was a separate visualization effort built to make the simulation results easier to interpret and easier to present in the report. The goal was not just to dump states, but to make the aircraft motion, actuator schedules, and transition behavior readable in a report-quality format.

## What Was Added

### 1. Report-Quality Static Plotting
Script:
- [`/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/eVTOL_Simulation/plot_report_responses.m`](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/plot_report_responses.m)

What it does:
- Creates white-background figures intended for direct use in the report.
- Splits the outputs into two compact `2x2` figures so the report does not get overloaded with too many pages of plots.
- Plots the key vehicle states:
  - position
  - velocity
  - Euler angles
  - angular rates
- Plots the key transition and actuator states:
  - airspeed
  - tilt states
  - rotor speed states
  - control surface states

Useful detail:
- For rotor speeds and control surfaces, the script overlays the scheduled trim profile and the actual state in the same color.
- The actual trace is solid.
- The scheduled trace is dotted.
- This makes it easy to see tracking behavior without cluttering the figure.

## 2. Animated eVTOL Visualization
Script:
- [`/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/eVTOL_Simulation/make_evtol_video.m`](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/make_evtol_video.m)

What it does:
- Generates an MP4 animation of the aircraft over a simulation run.
- Uses the logged simulation data directly from `out`.
- Animates:
  - aircraft attitude
  - front tilt motion
  - local wing and tail surface deflections
  - rotor RPM-based thrust arrows
  - body-axis vectors at the CG
  - body velocity vector

Useful detail:
- The thrust vectors are reconstructed from grouped RPM states using the propeller thrust model, so the direction and relative size are consistent with the propulsion schedule.
- The video overlay also shows the key telemetry at each frame, including Euler angles, velocity, tilt states, RPM, and thrust magnitude.
- A simple playback-speed scalar was added so the video can run faster than real time when needed for presentations.

## 3. Output Organization

Report plot folder:
- [`/Users/zbrown/Documents/Fifth year Spring/Flight Controls/eVTOL-Flight-Sim/eVTOL_Simulation/report_plots_final`](/Users/zbrown/Documents/Fifth%20year%20Spring/Flight%20Controls/eVTOL-Flight-Sim/eVTOL_Simulation/report_plots_final)

Why this matters:
- Static report figures and video exports now have a dedicated location.
- The plotting and video scripts can save into this folder automatically.
- File naming was aligned with the run/case naming convention so exported figures and videos match the case being discussed in the report.

## Suggested Report Language

One concise way to describe this work in the report is:

> In addition to the core simulation and trim analysis, a dedicated visualization workflow was developed to improve interpretation of the vehicle response during transition. This included white-background report plots for key states and actuator schedules, as well as a 3D animation tool that renders the aircraft attitude, tilt system, control surface motion, rotor thrust direction, and body-velocity vector directly from simulation outputs. These tools were used to make the transition behavior easier to diagnose and easier to communicate in the final report.

## Shorter Version

> A custom visualization pipeline was developed alongside the simulation. This included report-ready response plots and a 3D animated state video showing aircraft attitude, tilt, control surface motion, and RPM-driven thrust behavior during transition.
