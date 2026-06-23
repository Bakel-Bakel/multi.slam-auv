# auv_sim_stonefish

High-fidelity marine simulation via Stonefish + `stonefish_ros2` (spec §7.2, M4).
Stonefish gives realistic FLS/SSS/MSIS/multibeam acoustic imagery with ground truth —
ideal for validating the sonar-SLAM modules before trusting noisier data.

## Vendoring seam (required to run)
Stonefish is a C++ library built from source (distro-agnostic). It is **not** built in
this environment by default. To enable:
```bash
src/auv_sim/auv_sim_stonefish/scripts/build_stonefish.sh "$PWD"
colcon build --packages-select stonefish_ros2 auv_sim_stonefish
```
Pins live in `dependencies.repos` (`patrykcieslak/stonefish`, `patrykcieslak/stonefish_ros2`).

## Contents
- `scenarios/bluerov2.scn` — ocean + seafloor relief + BlueROV2 (geometry-based
  hydrodynamics) + full sensor suite + 6 vectored thrusters. Sensor topics are named so
  the adapter remaps land them on the exact contract names/frames.
- `launch/sim_stonefish.launch.py` — runs `stonefish_ros2` (if built) + description +
  sim-common + control. Falls back to logging the build hint if Stonefish is absent.

## Convention boundary (spec §4)
Stonefish/marine outputs are NED/FRD-leaning. NED→ENU normalization happens at the
adapter boundary (sim-common), never inside SLAM. The contract downstream is identical
to the Gazebo path, so `sim:=stonefish` swaps simulators with zero SLAM-code changes.

## Run (M4 acceptance)
```bash
ros2 launch auv_bringup bringup.launch.py sim:=stonefish
# same teleop drives it; the same contract topics appear with identical names/frames:
ros2 topic list   # diff against the Gazebo run should be empty for contract topics
```
