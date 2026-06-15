# Agent Brief

## Project Context

This repo contains MATLAB vehicle dynamics scripts for Brennan vehicle simulation work. The active file recently worked on is `VehicleBalance_MF14.m`.

`VehicleBalance_MF14.m` is a steady-state cornering balance tool. It estimates wheel loads, slip angles, required lateral grip, understeer/oversteer tendency, steering effort, and parameter sweep sensitivity. It depends on tire model functions under `Tire Modeling/Function Files`.

Important caveat: the MF14 setup block still includes a warning that the setup currently matches MF13 values. Do not treat MF14 conclusions as final until MF14-specific values are confirmed.

## MATLAB

MATLAB is installed here:

```sh
/usr/local/MATLAB/R2026a/bin/matlab
```

The plain `matlab` command may not be on `PATH`.

Run the script from the repo root:

```sh
/usr/local/MATLAB/R2026a/bin/matlab -batch "run('VehicleBalance_MF14.m');"
```

Inside the MATLAB CLI:

```matlab
run('VehicleBalance_MF14.m')
```

For a clean rerun inside MATLAB:

```matlab
clear; clc; close all; run('VehicleBalance_MF14.m')
```

To launch MATLAB without the full desktop UI but with plot windows:

```sh
/usr/local/MATLAB/R2026a/bin/matlab -nodesktop
```

## Current Outputs

`VehicleBalance_MF14.m` saves generated plots to:

```text
VehicleBalance_MF14_outputs/
```

Current expected files include:

- `front_vs_rear_slip_angle.png`
- `required_lateral_grip.png`
- `front_vs_rear_load_transfer.png`
- `inside_wheel_loads.png`
- `understeer_gradient.png`
- `steering_wheel_force_comparison.png`
- `steering_column_torque_comparison.png`
- `kRoll_r_arb_sweep.png`
- `DFDistF_sweep.png`
- `r_corner_sweep.png`
- `corner_radius_balance_map.png`

The corner radius balance map uses color for `frontSA - rearSA`, white speed contours in mph, and arrow glyphs for balance direction:

- right arrow = understeer trend
- left arrow = oversteer trend
- near-neutral points are suppressed

## Recent Changes In `VehicleBalance_MF14.m`

- Reorganized setup variables into sections.
- Replaced misleading `TL` with `wheelbase`.
- Fixed unsprung load transfer to use `TF` for front and `TR` for rear.
- Added named plot windows and explanatory plot subtitles.
- Added automatic PNG export into `VehicleBalance_MF14_outputs`.
- Added variable-agnostic one-at-a-time parameter sweeps.
- Added multiple independent sweeps in one run.
- Added a corner-radius balance map.
- Added speed contour labels and oversteer/understeer direction arrows to that map.

## Parameter Sweep Configuration

The sweep configuration is near the top of `VehicleBalance_MF14.m` in `%% Parameter Sweep Mode`.

Current form:

```matlab
runParameterSweep = true;
sweepParameterNames = {'kRoll_r_arb', 'DFDistF', 'r_corner'};
sweepParameterLabels = {'Rear ARB stiffness, kRoll\_r\_arb [N*m/deg]', 'Front downforce distribution', 'Corner radius [m]'};
sweepParameterValueSets = {linspace(0, 600, 13), linspace(0.40, 0.55, 10), [5, 7.5, 10, 12.5, 15, 20, 30, 50]};
```

To add a scalar sweep, append one entry to all three cell arrays. The parameter name must match a field in `sweepBaseParams` inside the parameter sweep section.

This is not a full factorial sweep. Each listed parameter is swept independently from the baseline setup.

## Known Modeling Caveats

- `findSlip` caps slip angle at 10 degrees, so plots near the limit can show discontinuities or saturated blocks. Treat those regions as "at/near limit," not precise physics.
- The corner radius balance map currently uses the same lateral-g range for every radius. This is useful for comparison, but not every radius/g combination is necessarily equally realistic.
- The balance map is still coarse in radius unless `cornerRadiusMapValues` is densified.
- The MF14 vehicle constants need confirmation.

## Suggested Next Work

- Confirm and update MF14-specific mass, CG, tracks, wheelbase, aero, springs, roll centers, alignment, caster/KPI, and steering geometry.
- Mask or visually mark saturated points where either axle reaches the 10 degree slip cap.
- Consider turning the balance calculation into a function used by both baseline plotting and sweep plotting to reduce duplication.
- Add CSV export for sweep metrics.
- Add a factorial or 2D sweep mode for pairs such as `kRoll_r_arb` vs `DFDistF`.
