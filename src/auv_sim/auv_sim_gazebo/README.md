# auv_sim_gazebo

Gazebo Harmonic (gz-sim 8) underwater bring-up for the BlueROV2 (spec §7.1, M3).

## Contents
- `worlds/ocean.sdf` — surface at z=0, graded buoyancy (seawater 1025 kg/m³ below,
  air above), seafloor at −20 m with relief (mounds/ridge) for bathymetric SLAM.
  Loads Physics/Sensors/Imu/Magnetometer/AirPressure/Buoyancy/Contact systems.
- `config/bridge.yaml` — the **Gazebo-side adapter**: maps gz topics onto the
  interface contract (clock, imu, mag, pressure, stereo, MBES points, ground-truth
  odom, and the 6 thruster command topics). Adjusting names here is a config change.
- `launch/sim_gazebo.launch.py` — gz-sim + spawn + bridge + description + sim-common
  + thruster allocation.

## Run (M3 acceptance)
```bash
# GUI (needs a working DISPLAY):
ros2 launch auv_sim_gazebo sim_gazebo.launch.py

# Headless server only (no GUI — SSH / no display):
ros2 launch auv_sim_gazebo sim_gazebo.launch.py headless:=true

# Harmonic + Humble: apt ros_gz uses Fortress transport; build the Harmonic overlay
# once so the bridge (/imu/data, /clock, …) works with gz sim 8:
src/auv_sim/auv_sim_gazebo/scripts/build_ros_gz_harmonic.sh
source install/setup.bash
ros2 launch auv_sim_gazebo sim_gazebo.launch.py
```

If the world loads but **no ROV appears**, check the log for `Unknown message type [8]`.
That means apt `ros_gz_sim/create` cannot talk to Harmonic — the launch file now uses
`spawn_harmonic.py` automatically when `gz_version:=8` (default).
# in another terminal: drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# verify health
ros2 topic hz /imu/data
ros2 topic echo /ground_truth/odom --once
gz topic -l        # confirm gz topic names if the bridge reports no traffic
```
Expect physically plausible motion (near-neutral buoyancy, no explosion) and
`/ground_truth/odom` tracking commanded motion.

## Notes / seams
- The MBES stream is a downward `gpu_lidar` proxy. Project DAVE's GPU multibeam
  (FRD sensor frame) replaces it for high-fidelity sonar (spec §7.1, §10.4); vendor
  DAVE under `src/third_party/` and point the bridge at its topics.
- Requires a GPU/display for the `Sensors` (rendering) system. Headless rendering
  needs an EGL-capable GPU.
