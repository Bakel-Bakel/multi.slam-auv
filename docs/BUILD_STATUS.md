# Build status by milestone

Target track: **B** (ROS 2 Humble / Ubuntu 22.04 / Gazebo Harmonic gz-sim 8).

Legend: ✅ implemented & buildable here · 🟡 implemented, needs vendored dep or GPU
sim to fully run · 🌱 contract/seam in place, deep work deferred to its milestone.

| Milestone | Packages | Status | Notes |
|---|---|---|---|
| M0 Workspace/tooling | docker, CI, AGENTS.md | ✅ | Track B Dockerfile, GitHub Actions, guardrails. |
| M1 Interface contract | `auv_interfaces`, `INTERFACE.md` | ✅ | DVL/USBLFix/SlamStatus msgs, SaveMap/TriggerLoopSearch srvs, RunTrajectory action. `marine_acoustic_msgs` vendored via `dependencies.repos`. |
| M2 Vehicle description | `auv_description` | ✅ | BlueROV2 xacro, sensor macros, full TF tree, `check_urdf`. |
| M3 Gazebo bring-up | `auv_sim_gazebo`, `auv_sim_common`, `auv_control` | 🟡 | Harmonic world + spawn + buoyancy/hydro/thruster + `ros_gz` bridge + teleop. Needs a GPU/display to render. |
| M4 Stonefish bring-up | `auv_sim_stonefish` | 🟡 | Scenario XML + adapter + setup script. Requires building Stonefish lib from source (vendored). |
| M5 Sensor plumbing | sim pkgs, `auv_interfaces` | 🟡 | Bridge + remap configs for all sensors; depth-sign unit test ✅. |
| M6 State estimation | `auv_state_estimation`, `auv_dead_reckoning` | ✅ | `robot_localization` EKF config + DVL/IMU dead-reckoning + conversion unit tests. |
| M7 Visual SLAM | `auv_image_enhancement`, `auv_visual_slam`, `auv_place_recognition` | 🟡 | Image enhancement node fully works (OpenCV). ORB-SLAM3 wrapper is a documented vendoring seam; RTAB-Map launch ✅. Constraint adapter ✅. |
| M8 Sonar SLAM | `auv_sonar_slam` | 🟡 | MBES submap + GICP registration and FLS CFAR feature extraction implemented (numpy/PCL); validate on Stonefish sonar. |
| M9 Factor-graph core | `auv_slam_core` | ✅ | GTSAM/iSAM2 integrator: odometry/DVL/depth/loop/USBL factors, robust kernels, pluggable modalities. |
| M10 Mapping + eval | `auv_mapping`, `auv_evaluation` | ✅ | Cloud/occupancy/bathymetry accumulation; `evo` ATE/RPE harness + rosbag regression. |
| M11 Robustness/seams | `auv_drivers`, `auv_bringup` | ✅ | Real-hardware contract stubs; launch presets; failure-injection params; docs. |

## What is genuinely runnable in a stock Humble+Harmonic environment
- M0/M1/M2/M6/M9/M10/M11 build and run with only `apt` + `pip` deps.
- M3 runs given a working Gazebo Harmonic install and a display/GPU.

## What requires vendored upstream (network + build time)
- **Stonefish** (M4): build `patrykcieslak/stonefish` + `stonefish_ros2` from source.
- **ORB-SLAM3** (M7): vendor `UZ-SLAMLab/ORB_SLAM3` + `Mechazo11/ros2_orb_slam3`
  (isolate its pinned g2o/DBoW2/Sophus). RTAB-Map is available via apt as a cross-check.
- **Project DAVE GPU sonar** (M5/M8): vendor DAVE's Harmonic plugins for FLS/SSS/MBES;
  Gazebo's stock sensors cover camera/IMU/depth in the meantime.

Each seam is documented in the relevant package README with the exact upstream repo,
branch, and integration point so the deferred work is mechanical, not exploratory.

## Verified build/test (this environment)
Native ROS 2 Humble, `colcon` + `setuptools 80.9.0`.

- `colcon build` → **17 packages finished**, 0 failures.
- Unit tests → **32 passed** (`pytest` over every package's `test/`): conversions &
  depth-sign, thruster allocation, dead-reckoning integrator, image enhancement,
  place-recognition descriptors, ICP/CFAR, GTSAM pose-graph drift reduction, ATE/RPE.
- `ros2 interface list` shows all 7 custom types (DVL, USBLFix, SlamStatus, LoopClosure,
  SaveMap, TriggerLoopSearch, RunTrajectory).
- `bringup.launch.py` + `full_fusion`/`visual_only`/`sonar_only` presets parse cleanly.

### Build note (setuptools ≥ 80)
`colcon build --symlink-install` fails with `option --editable not recognized` because
modern setuptools dropped `develop --editable`. Build **without** `--symlink-install`
(plain `colcon build`), or pin `setuptools<80` if editable installs are required.
