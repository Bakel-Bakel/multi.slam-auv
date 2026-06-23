# RUNBOOK — how to run & check every aspect of the project

A step-by-step, copy-pasteable guide. It goes **bottom-up**: build → tests → each layer
on its own (with a "what you should see" check) → the full stack → headless/no-GPU path →
troubleshooting.

Conventions used below:
- `WS=/home/lordcruise/Documents/Underwater-robotics-projects/multi-slam` (your workspace).
- Every new terminal must run the **source block** below first.
- "Terminal A/B/C" means open separate terminals (each sourced).

```bash
# ---- source block (run at the top of EVERY terminal) ----
export WS=/home/lordcruise/Documents/Underwater-robotics-projects/multi-slam
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash          # only after the first build
export ROS_LOG_DIR="$WS/.roslog"   # keeps logs inside the workspace
```

---

## 0. Prerequisites (one time)

```bash
source /opt/ros/humble/setup.bash
pip3 install "gtsam==4.2" "evo==1.28.0" "numpy<2" scipy opencv-python
# optional ROS binaries used by some layers (skip what you don't need):
sudo apt install -y ros-humble-robot-localization ros-humble-rtabmap-ros \
  ros-humble-ros-gz ros-humble-teleop-twist-keyboard
rosdep install --from-paths "$WS/src" --ignore-src -y -r || true
```

Check what's available (each is optional; the guide notes when a layer needs one):

```bash
ros2 pkg list | grep -E "robot_localization|rtabmap|ros_gz_sim|teleop_twist" || true
python3 -c "import gtsam, cv2, scipy, numpy; print('py deps OK')"
```

---

## 1. Build the workspace

> Use a **plain** `colcon build`. Do **not** use `--symlink-install` (setuptools ≥ 80 breaks
> it with `option --editable not recognized`).

```bash
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct-
```

**Check — expected:** `Summary: 17 packages finished`, 0 failed. A harmless
`setuptools` deprecation warning may print.

If a single package misbehaves, build just it:

```bash
colcon build --packages-select auv_slam_core
```

---

## 2. Verify the build outputs

```bash
source install/setup.bash

# 2a. All 17 packages are discoverable
ros2 pkg list | grep '^auv_' | sort

# 2b. All 7 custom interfaces generated
ros2 interface list | grep auv_interfaces
# expect: DVL, USBLFix, SlamStatus, LoopClosure (msg) + SaveMap, TriggerLoopSearch (srv) + RunTrajectory (action)

# 2c. Inspect a custom message definition
ros2 interface show auv_interfaces/msg/DVL

# 2d. Every executable is registered
for p in auv_state_estimation auv_dead_reckoning auv_slam_core auv_mapping \
         auv_evaluation auv_sonar_slam auv_visual_slam auv_place_recognition \
         auv_image_enhancement auv_control auv_sim_common auv_drivers; do
  echo "== $p =="; ros2 pkg executables "$p"; done
```

---

## 3. Run the unit tests (logic correctness, no sim needed)

All core algorithms are pure-Python and unit-tested. This is the fastest way to confirm
the math (depth sign, thruster allocation, ICP/CFAR, GTSAM drift reduction, ATE/RPE).

```bash
cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest -q \
  src/auv_sim/auv_sim_common/test \
  src/auv_control/test \
  src/auv_localization/auv_dead_reckoning/test \
  src/auv_slam/auv_image_enhancement/test \
  src/auv_slam/auv_place_recognition/test \
  src/auv_slam/auv_sonar_slam/test \
  src/auv_slam/auv_slam_core/test \
  src/auv_evaluation/test
```

**Check — expected:** `32 passed`.

Run one suite at a time to read individual test names, e.g.:

```bash
python3 -m pytest -v src/auv_slam/auv_slam_core/test   # GTSAM loop-closure drift test
python3 -m pytest -v src/auv_sim/auv_sim_common/test   # includes the depth-sign (z = -d) test
```

---

## 4. Run & check each layer individually

Each subsection is independent. Keep a spare **Terminal X** open for inspection commands
(`ros2 topic list`, `echo`, `hz`, `tf2`).

### 4.1 Vehicle description + TF tree (no sim, no GPU)

```bash
# Terminal A
ros2 launch auv_description description.launch.py
# add rviz:=true to visualize, or gui:=true for the joint_state_publisher GUI
```

**Checks (Terminal X):**
```bash
ros2 topic echo -n1 /robot_description >/dev/null && echo "URDF published"
ros2 run tf2_tools view_frames        # writes frames.pdf of the TF tree
ros2 run tf2_ros tf2_echo base_link mbes_link   # a static extrinsic should resolve
```
Expect the frame tree from `INTERFACE.md`: `base_link → imu_link/dvl_link/.../mbes_link`.

Validate the xacro directly (no ROS graph needed):
```bash
xacro install/auv_description/share/auv_description/urdf/bluerov2.urdf.xacro \
  use_gazebo:=true heavy:=false | head -20      # should be well-formed XML
```

### 4.2 Gazebo simulator (needs Gazebo Harmonic + a display/GPU)

```bash
# Terminal A
ros2 launch auv_sim_gazebo sim_gazebo.launch.py
# args: world:=ocean.sdf  heavy:=false  spawn_z:=-1.0  use_sim_time:=true
```

**Checks (Terminal X):**
```bash
ros2 topic list | sort                 # contract topics should appear
ros2 topic hz /clock                   # sim clock is ticking
ros2 topic hz /imu/data                # IMU streaming
ros2 topic echo -n1 /ground_truth/odom # ground-truth (eval only)
ros2 topic hz /sonar/mbes/points       # MBES point cloud
```
If you have no GPU/display, **skip to §6** (rosbag-driven path) — every downstream layer
can run from a bag instead of live Gazebo.

### 4.3 State estimation (EKF) — needs `robot_localization`

```bash
# Terminal B (with a sim or bag publishing /imu/data, /dvl, /depth)
ros2 launch auv_state_estimation estimation.launch.py
```

**Checks:**
```bash
ros2 node list | grep -E "ekf_filter_node|dvl_to_twist"
ros2 topic hz /odom                                  # smooth fused odometry
ros2 run tf2_ros tf2_echo odom base_link             # EKF owns this edge
```
The `dvl_to_twist` node **gates** DVL samples with no bottom-lock (no bad EKF updates).

### 4.4 Dead-reckoning baseline (reference, publishes no TF)

```bash
ros2 launch auv_dead_reckoning dead_reckoning.launch.py
ros2 topic echo -n1 /dead_reckoning/odom   # IMU-attitude + DVL-velocity integration
```

### 4.5 Image enhancement + visual SLAM front-end

```bash
# enhancement alone (pure OpenCV, works on any /cam/*/image_raw source)
ros2 launch auv_image_enhancement image_enhancement.launch.py

# full visual front-end (enhancement + adapter + optional RTAB-Map)
ros2 launch auv_visual_slam visual_slam.launch.py backend:=rtabmap
#   backend:=none      -> adapter only, expects external /visual/odom
#   backend:=orbslam3  -> vendored seam (see auv_visual_slam/README.md)
```

**Checks:**
```bash
ros2 topic hz /cam/left/image_enhanced     # enhanced stream
ros2 topic echo -n1 /slam/constraints      # adapter emits LoopClosure constraints
```
`backend:=rtabmap` needs `ros-humble-rtabmap-ros`; otherwise the launch logs a hint and
runs adapter-only.

### 4.6 Sonar SLAM front-ends

```bash
ros2 launch auv_sonar_slam sonar_slam.launch.py mbes:=true fls:=false
```

**Checks:**
```bash
ros2 node list | grep -E "mbes_submap_slam|fls_feature_slam"
ros2 topic hz /mbes/submap_cloud           # accumulated/voxelized submap (debug)
ros2 topic echo -n1 /slam/constraints      # ICP-derived constraints (modality=mbes)
```
Enable FLS feature SLAM with `fls:=true` (consumes a mono sonar `Image`).

### 4.7 Place recognition (loop closures)

```bash
ros2 launch auv_place_recognition place_recognition.launch.py visual:=true acoustic:=false
ros2 topic echo /slam/constraints          # emits LoopClosure on revisits
```

### 4.8 Factor-graph core (GTSAM/iSAM2 — the unifier)

```bash
# Terminal C (needs /odom + constraints + optionally /usbl, /depth)
ros2 launch auv_slam_core slam_core.launch.py
```

**Checks:**
```bash
ros2 topic echo -n1 /slam/pose             # optimized PoseWithCovarianceStamped (map frame)
ros2 topic echo -n1 /slam/trajectory       # nav_msgs/Path
ros2 run tf2_ros tf2_echo map odom         # slam_core owns map->odom
ros2 topic echo -n1 /slam/status           # SlamStatus: #keyframes, #loops, opt time, drift
```

### 4.9 Mapping + SaveMap service

```bash
ros2 launch auv_mapping mapping.launch.py
```

**Checks:**
```bash
ros2 topic hz /map/cloud                   # global PointCloud2
ros2 topic echo -n1 /map/bathymetry        # 2.5D OccupancyGrid
# write products to disk:
ros2 service call /mapping/save_map auv_interfaces/srv/SaveMap \
  "{path: /tmp/seafloor, format: all}"
ls -l /tmp/seafloor*                        # expect .pcd / .ply
```

### 4.10 Evaluation (ATE/RPE vs ground truth)

```bash
# Terminal D: log estimate + ground truth to TUM files while a mission runs
ros2 launch auv_evaluation evaluate.launch.py \
  estimate_topic:=/slam/pose estimate_type:=pose_cov output_dir:=/tmp/auv_eval
# ... let it run, then Ctrl-C ...

# compute metrics (uses evo if installed, else the built-in ate.py fallback)
ros2 run auv_evaluation run_evo /tmp/auv_eval/estimate.tum \
  /tmp/auv_eval/ground_truth.tum --ate-threshold 0.5
```
`estimate_type` is `pose_cov | odom | pose`. **Check:** prints ATE/RPE; exits non-zero if
ATE exceeds the threshold (useful in CI).

### 4.11 Control + teleop (drive the vehicle)

```bash
# Terminal E: keyboard teleop -> /cmd_vel  (needs a tty)
ros2 launch auv_control teleop.launch.py
# or publish a one-off command:
ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'
```
**Check:** `ros2 topic echo /thruster_cmds` shows the allocated per-thruster `Float64MultiArray`.

### 4.12 Sim-to-real driver contract (stubs)

```bash
ros2 run auv_drivers driver_stub --ros-args -p driver:=dvl
```
**Check:** logs the exact topic/type/frame a real DVL driver must publish, then idles. Try
`driver:=imu|pressure|usbl|camera|fls|mbes`. This documents the hardware seam.

---

## 5. Full stack (one command)

Live Gazebo + all layers + RViz (needs GPU/display):

```bash
ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=all rviz:=true
# presets:
ros2 launch auv_bringup full_fusion.launch.py sim:=gazebo
ros2 launch auv_bringup visual_only.launch.py sim:=gazebo
ros2 launch auv_bringup sonar_only.launch.py  sim:=stonefish
```

Args: `sim:={gazebo,stonefish}`, `modalities:={visual,sonar,all}`, `use_sim_time:=true`,
`rviz:=true`, `record:=true` (records contract topics to `/tmp/auv_mission`).

**Validate a launch file without running it (no GPU needed):**
```bash
ros2 launch auv_bringup bringup.launch.py --show-args
```

**RViz check:** robot model, TF tree, `/odom`, green `/slam/trajectory`, `/map/cloud`,
and `/map/bathymetry` all display; fixed frame is `map`.

---

## 6. Headless / no-GPU path (rosbag-driven)

You can exercise the **entire** estimation→SLAM→mapping→eval pipeline without Gazebo by
replaying a bag of the contract topics.

```bash
# Terminal A: bring up the processing stack only (no sim), sim time on
ros2 launch auv_state_estimation estimation.launch.py &
ros2 launch auv_slam_core slam_core.launch.py &
ros2 launch auv_mapping mapping.launch.py &

# Terminal B: replay a recorded mission; --clock drives sim time for everyone
ros2 bag play /path/to/mission_bag --clock
```

Automated regression (replay + log + ATE threshold assert):

```bash
src/auv_evaluation/scripts/regression_test.sh /path/to/mission_bag 0.5 /slam/pose pose_cov
```
**Check:** exits 0 if ATE ≤ 0.5 m, non-zero (and prints the value) if it regressed.

To produce a bag in the first place, run the full stack once with `record:=true`, or:
```bash
ros2 bag record -o /tmp/auv_mission /clock /imu/data /pressure /dvl /usbl \
  /cam/left/image_raw /cam/right/image_raw /sonar/mbes/points /odom \
  /slam/pose /slam/trajectory /ground_truth/pose
```

---

## 7. Whole-graph sanity checks (anytime something is running)

```bash
ros2 node list                     # who's alive
ros2 topic list -t                 # topics + types
ros2 topic hz <topic>              # is it flowing?
ros2 topic info -v <topic>         # publisher/subscriber QoS (catches QoS mismatches)
ros2 run rqt_graph rqt_graph       # visual node/topic graph
ros2 run tf2_tools view_frames     # TF tree -> frames.pdf
```

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `colcon build` fails: `option --editable not recognized` | setuptools ≥ 80 + `--symlink-install` | drop `--symlink-install` (plain `colcon build`), or `pip install "setuptools<80"`. |
| Topic exists but `echo` shows nothing | QoS mismatch (SensorData vs Reliable) | `ros2 topic info -v <t>`; sensor topics are Best-Effort, state/pose are Reliable, trajectory/`tf_static` are Transient-Local. |
| Everything frozen / no `/clock` | `use_sim_time` true but no clock source | start the sim or `ros2 bag play --clock`, or set `use_sim_time:=false` for live tests. |
| Depth/altitude has wrong sign | NED vs ENU at the boundary | depth is `z = -d` in ENU; the depth-sign unit test guards this (see §3). |
| `map->odom` missing | `auv_slam_core` not running (or no constraints yet) | start `slam_core`; it owns `map->odom`. EKF owns `odom->base_link`. |
| RTAB-Map / Gazebo / Stonefish "not found" | optional upstream not installed/vendored | install the apt package, or follow the vendoring seam in that package's `README.md` (`docs/BUILD_STATUS.md` lists each seam). |
| `launch` crashes in `os.makedirs(... .ros/log ...)` | log dir not writable | `export ROS_LOG_DIR="$WS/.roslog"` (in the source block above). |

---

## 9. One-shot smoke test (build + interfaces + all unit tests)

```bash
cd "$WS" && source /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct- && source install/setup.bash
ros2 interface list | grep -c auv_interfaces      # expect 7
python3 -m pytest -q \
  src/auv_sim/auv_sim_common/test src/auv_control/test \
  src/auv_localization/auv_dead_reckoning/test \
  src/auv_slam/auv_image_enhancement/test src/auv_slam/auv_place_recognition/test \
  src/auv_slam/auv_sonar_slam/test src/auv_slam/auv_slam_core/test \
  src/auv_evaluation/test                          # expect 32 passed
```
