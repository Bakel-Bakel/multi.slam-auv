# AUV Multi-Modal SLAM Stack — Cursor Build Specification

**Target:** A modular ROS 2 system that runs every practical form of SLAM on an autonomous underwater vehicle (visual, visual-inertial, imaging-sonar, profiling/bathymetric-sonar, side-scan, plus DVL/IMU/depth/USBL dead-reckoning and state estimation), fused through a single factor-graph back-end, validated in **two** physics simulators (**Stonefish** and **Gazebo Harmonic / Project DAVE**), on a real **BlueROV2** description.

**Document purpose:** This is a build playbook written *for an AI coding agent* (Cursor). It specifies architecture, interfaces, package layout, exact phasing, and acceptance tests so the project can be built incrementally and verifiably from an empty workspace to a working multi-SLAM system. It is intentionally implementation-detailed but contains no source code — Cursor writes the code, this document tells it what to write, in what order, and how to know each step is correct.

---

## 0. How Cursor should use this document

Read this whole file once before writing anything. Then build **one milestone at a time** (Section 11). Do not jump ahead.

**The iron rule of this build:** *Build, run, and pass the acceptance test for milestone `Mn` before starting `Mn+1`.* Underwater SLAM fails silently — a sign error in a depth conversion or a frame mislabel produces output that looks plausible and is completely wrong. Incremental verification against simulator ground truth is the only defense.

**Workflow per milestone:**
1. State which milestone you are implementing and list the packages you will touch.
2. Implement only what that milestone requires.
3. `colcon build` the affected packages. Fix all build errors before proceeding.
4. Run the milestone's acceptance test exactly as written.
5. Report the result (pass/fail + evidence: RViz screenshot description, `evo` numbers, topic echoes). If it fails, fix before moving on.
6. Commit with a message referencing the milestone (`M6: robot_localization EKF passes ATE < 0.5 m on lawnmower run`).

**Hard constraints (put these in `.cursorrules` / `AGENTS.md` — see Section 18):**
- Never invent a ROS API, message field, or plugin name. If unsure, read the upstream package's README/headers first.
- Pin every external dependency to a known-good version. Prefer vendoring upstream repos as git submodules over copy-paste.
- Ask before adding any new third-party dependency not listed in Section 16.
- Every package must build in isolation and declare its dependencies correctly in `package.xml` / `CMakeLists.txt`.
- Default to `use_sim_time:=true` everywhere; the whole stack is simulation-first.

---

## 1. Scope and non-goals

### In scope (the SLAM modalities)
| Modality | Sensors | Role in the system |
|---|---|---|
| **Inertial / proprioceptive nav** | IMU/AHRS, DVL, pressure (depth), magnetometer | Continuous dead-reckoning + state estimation backbone |
| **Acoustic absolute positioning** | USBL (and surface GNSS when at surface) | Absolute position priors / drift correction |
| **Visual SLAM** | mono / stereo / RGB-D camera | Keyframe poses, landmarks, optical loop closure |
| **Visual-inertial odometry** | camera + IMU | Metric-scale, drift-resistant odometry front-end |
| **Imaging-sonar SLAM** | forward-looking sonar (FLS), mechanically scanned imaging sonar (MSIS) | Acoustic features + scan-matching constraints in turbid/dark water |
| **Bathymetric SLAM** | multibeam echosounder (MBES) | Submap registration, seafloor map, loop closure on terrain |
| **Side-scan mapping** | side-scan sonar (SSS) | Seafloor mosaicking + acoustic place recognition |
| **Fusion / integrator** | all of the above | Single factor-graph back-end producing one consistent trajectory + map |

### Non-goals (explicitly out of scope for v1)
- No mission planning / autonomy stack beyond simple trajectory execution needed to collect data.
- No real-hardware bring-up in v1 (the architecture leaves clean seams for it — Section 9.3 — but v1 is simulation-only).
- No web/cloud dashboard. Visualization is RViz2 + Foxglove + PlotJuggler.
- No claim of real-time on a laptop for *all* modalities simultaneously. Real-time is a per-modality goal; full fusion may run faster-than-real-time on logged data.

---

## 2. System architecture

The design principle that makes "all forms of SLAM" tractable is a **strict separation between modular front-ends and one unified back-end**, mediated by a **stable interface contract** (Section 6). Every perception module is an optional plugin that converts its sensor stream into *constraints*; the back-end consumes constraints and produces the estimate. Nothing in the SLAM layer knows or cares which simulator is running.

```
                        ┌───────────────────────────── SIMULATION LAYER ─────────────────────────────┐
                        │                                                                             │
   ┌──────────────┐     │   ┌────────────────────┐        ┌────────────────────┐   ┌──────────────┐  │
   │  BlueROV2    │     │   │  Stonefish         │        │  Gazebo Harmonic   │   │ (legacy)     │  │
   │  description │────▶│   │  (stonefish_ros2)  │   OR   │  + Project DAVE    │   │ UUV/Gz-Classic│ │
   │  URDF/xacro  │     │   │  scenario XML      │        │  gz-sim plugins    │   │  optional    │  │
   └──────────────┘     │   └─────────┬──────────┘        └─────────┬──────────┘   └──────┬───────┘  │
                        │             │                             │                     │          │
                        │             └───────────┬─────────────────┴─────────────────────┘          │
                        └─────────────────────────┼──────────────────────────────────────────────────┘
                                                  ▼
   ╔══════════════════════════════ INTERFACE CONTRACT (stable topics/frames/msgs) ══════════════════════════════╗
   ║  /imu  /dvl  /pressure  /magnetic  /usbl  /cam/{left,right}/image  /cam/.../camera_info                      ║
   ║  /sonar/fls/image  /sonar/sss/image  /sonar/mbes/points  /ground_truth/pose  /clock  + TF (ENU)             ║
   ╚════════════════════════════════════════════════════════════════════════════════════════════════════════════╝
        │           │            │              │                 │                    │
        ▼           ▼            ▼              ▼                 ▼                    ▼
 ┌─────────────┐ ┌──────────┐ ┌────────────┐ ┌─────────────┐ ┌──────────────┐ ┌────────────────┐
 │ Image       │ │ State    │ │ Visual     │ │ Imaging-    │ │ Bathymetric  │ │ Side-scan      │
 │ enhancement │ │ estim.   │ │ SLAM       │ │ sonar SLAM  │ │ (MBES) SLAM  │ │ mapping +      │
 │ (preproc)   │ │ backbone │ │ ORB-SLAM3 /│ │ FLS/MSIS    │ │ submap +     │ │ place recog.   │
 │             │ │ EKF+GTSAM│ │ RTABMap /  │ │ CFAR + ICP  │ │ probab. ICP  │ │                │
 │             │ │ (IMU+DVL │ │ OpenVINS   │ │             │ │ /GICP        │ │                │
 │             │ │ +depth)  │ │            │ │             │ │              │ │                │
 └──────┬──────┘ └────┬─────┘ └─────┬──────┘ └──────┬──────┘ └──────┬───────┘ └───────┬────────┘
        │             │             │               │               │                 │
        └─────────────┴─────────────┴───────────────┴───────────────┴─────────────────┘
                                          │ constraints (odometry, loop closures,
                                          │ relative poses, priors, landmarks)
                                          ▼
                          ┌──────────────────────────────────────┐
                          │  FACTOR-GRAPH INTEGRATOR (auv_slam_core)│
                          │  GTSAM + iSAM2                          │
                          │  IMU preintegration · DVL/depth factors │
                          │  visual & acoustic loop closures        │
                          │  USBL/GNSS priors                       │
                          └───────────────────┬──────────────────────┘
                                              ▼
                          ┌──────────────────────────────────────┐
                          │  MAPPING OUTPUTS (auv_mapping)         │
                          │  dense cloud · mesh · occupancy grid · │
                          │  2.5D bathymetry · optical/SSS mosaic   │
                          └───────────────────┬──────────────────────┘
                                              ▼
                          ┌──────────────────────────────────────┐
                          │  EVALUATION (auv_evaluation)           │
                          │  evo ATE/RPE vs ground truth · rosbag  │
                          │  regression harness                     │
                          └──────────────────────────────────────┘
```

**Why this shape:**
- *Modularity:* each SLAM modality is enabled/disabled by a launch argument. You can run visual-only, sonar-only, or everything fused.
- *Simulator independence:* both simulators publish into the **same** interface contract, so swapping Stonefish ↔ Gazebo is a launch-file change, never a code change.
- *Research-grade fusion:* a GTSAM/iSAM2 pose-graph back-end is the standard, defensible way to combine heterogeneous underwater constraints; it also leaves clean hooks for learned components later (Section 17).

---

## 3. Technology stack and versions

Pick **Track A** unless you have a specific reason not to. Pin everything; use the provided Docker image (M0) so the whole team/agent shares one environment.

### Track A — recommended (modern, forward-looking)
| Component | Version | Notes |
|---|---|---|
| OS | Ubuntu 24.04 LTS | |
| ROS 2 | **Jazzy Jalisco** (LTS) | Current LTS; matches DAVE's ROS 2 port. |
| Simulator (Gazebo) | **Gazebo Harmonic** | Via `ros_gz` binaries. Project DAVE's ROS 2 branch targets Jazzy + Harmonic. **Do not use Gazebo Classic** — it reached end-of-life in Jan 2025. |
| Simulator (marine) | **Stonefish** (lib) + `stonefish_ros2` | Built from source; distro-agnostic C++. Richest underwater sensor suite (FLS, SSS, MSIS, MBES, DVL, USBL, optical, event camera). |
| Build system | colcon + ament | |
| Estimation | GTSAM 4.2+, `robot_localization` | GTSAM is the factor-graph backbone; r_l for the loosely-coupled EKF/UKF. |
| Perception libs | OpenCV 4.x, PCL 1.14+, Eigen 3.4 | |
| Visual SLAM | ORB-SLAM3 (Jazzy wrapper), RTAB-Map (`rtabmap_ros`), OpenVINS | See Section 10.2. |

### Track B — maximum third-party maturity today
Same as Track A but **ROS 2 Humble + Ubuntu 22.04 + Gazebo Harmonic via gz vendor packages**. Choose this only if a specific upstream wrapper you need is Humble-only and you don't want to port it. Most ORB-SLAM3 ROS 2 wrappers and BlueROV2 repos were first written for Humble, so some will need the `humble` branch or light patching on Jazzy.

> **Decision rule for Cursor:** Default Track A. If an upstream package fails to build on Jazzy after one honest attempt (and a check of its branches), note it explicitly, and either (a) use its Humble-compatible fork/branch, or (b) flag it to the user rather than silently switching the whole project to Humble.

---

## 4. Coordinate frames and conventions — READ THIS TWICE

**This is the single highest-leverage correctness section in the document.** More underwater SLAM projects die here than anywhere else.

### Conventions
- The ROS world is **ENU** (East-North-Up) with **FLU** body frame (x-forward, y-left, z-up), per REP-103. `robot_localization`, RViz, and most ROS tooling assume this.
- The marine/vehicle world is **NED** (North-East-Down) with **FRD** body (x-forward, y-right/starboard, z-down). ArduSub/MAVLink and many sonar conventions are NED/FRD.
- **Project DAVE's multibeam sonar requires its sensor frame to be X-Forward, Y-Starboard, Z-Down (FRD)** for the point cloud to reproject correctly. Stonefish uses its own consistent convention defined in the scenario.

> **You will have both conventions in the system. Make the boundary explicit and convert at exactly one place.** Keep the entire SLAM/estimation stack in ENU. Convert NED/FRD-native sensor outputs (sonar reprojection frames, any ArduSub/MAVLink data) to ENU at the driver/bridge boundary, never deep inside SLAM nodes.

### The TF tree (target)
```
earth (optional, for geo-referencing / USBL/GNSS)
  └── map            # fixed, SLAM-optimized world frame (ENU)
        └── odom     # smooth, continuous, drifts slowly (ENU) — from state-estimation backbone
              └── base_link             # vehicle body (FLU)
                    ├── base_link_frd    # static, FRD mirror for marine-convention consumers
                    ├── imu_link
                    ├── dvl_link
                    ├── pressure_link
                    ├── camera_left_link / camera_right_link / camera_optical_link
                    ├── fls_link         # forward-looking sonar (mind FRD requirement for DAVE)
                    ├── sss_port_link / sss_stbd_link
                    └── mbes_link
```
- `map → odom`: published by the SLAM back-end (`auv_slam_core`). It is the correction that absorbs loop-closure jumps.
- `odom → base_link`: published by the state-estimation backbone (`robot_localization` / GTSAM nav). Smooth, no jumps.
- `base_link → sensor_*`: **static**, published by `robot_state_publisher` from the URDF. These are the extrinsics; get them right once.
- **REP-105** semantics: nav goals/odometry consumers use `odom` for local control and `map` for global consistency.

### The depth/pressure sign trap (the #1 bug)
- Pressure sensors report values that **increase as the vehicle goes deeper**. Depth `d ≥ 0` downward.
- In **ENU**, `z` is **up**. So a measured depth `d` maps to `z = -d` relative to the surface (z=0 at the water surface).
- When you feed depth into `robot_localization` as a `z` position, or into a GTSAM depth factor, **the sign must be `-d`**. Get this wrong and the whole estimate inverts vertically while still "looking" reasonable.
- Document the sign convention in the depth driver/bridge node header and assert on it in a unit test.

### DVL conventions
- DVL reports velocity in its own sensor frame. Bottom-track velocity requires ≥3 of 4 beams locked; with no lock the measurement is invalid (water-track is a different mode). Tag invalid measurements and **do not feed them** to the estimator — drop-outs handled wrong cause violent drift.
- Convert DVL body-frame velocity to `base_link` (FLU) using the static `base_link → dvl_link` transform before fusing.

---

## 5. Repository and workspace structure

```
auv_slam_ws/
├── docker/                      # Dockerfile(s), devcontainer, entrypoints (M0)
├── .github/workflows/           # CI: build + colcon test in container (M0)
├── AGENTS.md / .cursorrules      # agent guardrails (Section 18)
├── README.md
└── src/
    ├── auv_bringup/             # top-level launch, orchestration, global params, RViz/Foxglove configs
    ├── auv_interfaces/          # custom msgs/srv/action (DVL, USBL, sonar wrappers, SLAM status)
    ├── auv_description/         # BlueROV2 URDF/xacro, meshes, sensor xacro macros, gz + sf variants
    ├── auv_sim/
    │   ├── auv_sim_common/      # shared world assets, ground-truth republisher, the sim-abstraction layer
    │   ├── auv_sim_gazebo/      # gz-sim (Harmonic) worlds, model spawn, ros_gz_bridge configs, DAVE sensors
    │   ├── auv_sim_stonefish/   # Stonefish scenario XML, robot definition, launch
    │   └── auv_sim_uuv/         # OPTIONAL legacy Gazebo-Classic/UUV path (not recommended; Section 7.3)
    ├── auv_drivers/             # real-hardware drivers + sim-to-real shims (stubs in v1; Section 9.3)
    ├── auv_localization/
    │   ├── auv_state_estimation/# robot_localization configs + tightly-coupled GTSAM nav node
    │   └── auv_dead_reckoning/   # DVL+IMU dead-reckoning reference implementation
    ├── auv_slam/
    │   ├── auv_slam_core/       # GTSAM/iSAM2 factor-graph integrator (the unifier)
    │   ├── auv_image_enhancement/# underwater image restoration preprocessing
    │   ├── auv_visual_slam/     # ORB-SLAM3 / RTAB-Map / OpenVINS wrappers + constraint adapters
    │   ├── auv_sonar_slam/      # FLS/MSIS feature SLAM + MBES submap SLAM
    │   └── auv_place_recognition/# visual + acoustic loop-closure detection
    ├── auv_mapping/             # dense cloud / mesh / occupancy / 2.5D bathymetry / mosaic outputs
    ├── auv_control/             # thruster allocation + PID/MPC to drive the vehicle for data collection
    ├── auv_evaluation/          # evo-based ATE/RPE, ground-truth comparison, rosbag regression harness
    └── third_party/             # vendored upstream repos as git submodules (Section 16)
```

**Package conventions:**
- C++ packages: `ament_cmake`. Python packages: `ament_python`. Mixed (e.g., a wrapper with a Python driver + C++ node): allowed but document it.
- Every package has a one-paragraph `README.md` stating its inputs (topics/frames consumed), outputs (topics/frames produced), and parameters.
- Interfaces live **only** in `auv_interfaces`. No package defines its own messages.

---

## 6. Interface contract (the glue — define before implementing)

This is what keeps every module decoupled and both simulators interchangeable. **Implement and freeze this in M1, before any sim or SLAM work.** Treat it as the project's API.

### Standard messages (reuse, do not reinvent)
| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | — | sim time; everything uses `use_sim_time`. |
| `/imu/data` | `sensor_msgs/Imu` | `imu_link` | orientation + angular vel + linear accel; fill covariances. |
| `/pressure` | `sensor_msgs/FluidPressure` | `pressure_link` | converted to depth in a small node; remember `z = -d`. |
| `/magnetic` | `sensor_msgs/MagneticField` | `imu_link` | for heading aid. |
| `/cam/left/image_raw`, `/cam/right/image_raw` | `sensor_msgs/Image` | `*_optical_link` | mono/stereo; raw from sim. |
| `/cam/left/camera_info` (+right) | `sensor_msgs/CameraInfo` | — | intrinsics/rectification. |
| `/cam/.../image_enhanced` | `sensor_msgs/Image` | `*_optical_link` | output of `auv_image_enhancement`. |
| `/sonar/mbes/points` | `sensor_msgs/PointCloud2` | `mbes_link` | bathymetric returns. |
| `/odom` | `nav_msgs/Odometry` | `odom`→`base_link` | from state-estimation backbone. |
| `/slam/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | optimized current pose. |
| `/slam/trajectory` | `nav_msgs/Path` | `map` | full optimized trajectory. |
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` (or Odometry) | `map` | sim-only; for evaluation, never consumed by SLAM. |

### Custom messages (define in `auv_interfaces`)
- `auv_interfaces/msg/DVL` — header; per-beam ranges + velocities; aggregated body velocity (Vector3 + 3×3 covariance); `bool bottom_locked`; figure-of-merit. (No universal ROS DVL standard exists — define one cleanly.)
- `auv_interfaces/msg/USBLFix` — header; relative position (range/bearing/elevation or XYZ) + covariance; beacon id; absolute fix when surface GNSS available.
- For imaging sonar, **prefer the community standard `marine_acoustic_msgs`** (`ProjectedSonarImage`, `RawSonarImage`, `SonarImageData`) rather than rolling your own. Vendor it in `third_party/` and re-export. Add a thin `auv_interfaces` alias only if needed.
- `auv_interfaces/msg/SlamStatus` — header; active modules; #keyframes; #loop-closures; last-optimization time; estimated drift.
- `auv_interfaces/srv/SaveMap`, `auv_interfaces/srv/TriggerLoopSearch`, `auv_interfaces/action/RunTrajectory`.

### TF
- Static transforms from URDF via `robot_state_publisher`.
- `odom→base_link` from the estimation backbone; `map→odom` from `auv_slam_core`.
- A `ground_truth` TF chain published on a **separate** frame namespace so it can be overlaid in RViz without contaminating the live tree.

### QoS
- Sensor streams (images, sonar, IMU): **Best-Effort, Volatile, small depth** (sensor-data profile).
- State/trajectory/TF-static: **Reliable**, with TF-static **Transient-Local**.
- Mismatched QoS = silent "no messages" bugs. Document the profile on every publisher/subscriber.

---

## 7. Simulation layer

The goal: **two interchangeable simulators feeding one interface.** Build Gazebo first (richest ROS-native tooling and a known-good BlueROV2 path), then Stonefish (best underwater sensor fidelity, especially sonar), then make both publish identical topic/frame contracts.

### 7.1 Gazebo Harmonic + Project DAVE (primary sim)
- Use **Gazebo Harmonic** with `ros_gz` for the ROS 2 bridge.
- Vehicle + hydrodynamics: start from a known-good BlueROV2-for-Gazebo repo (Section 16) — `CentraleNantesROV/bluerov2` (clean ROS 2 + gz description, hydrodynamics plugins, ground-truth pose, cascaded-PID control) and/or `clydemcqueen/bluerov2_gz` (Harmonic model, vectored + Heavy configs, uses `ardupilot_gazebo` for the ArduSub link). Adapt, don't fork blindly.
- Underwater physics via gz-sim systems: **Buoyancy**, **Hydrodynamics** (Fossen-style added mass + drag), **Thruster** per propeller. Configure neutral/slightly-positive buoyancy and realistic added-mass/drag for the BlueROV2.
- Underwater sensors via **Project DAVE** (ROS 2 / Harmonic branch): DVL, USBL, multibeam FLS, underwater LiDAR, optical cameras. Note the migration status — not every legacy plugin is ported yet; check DAVE's migration-progress page and prefer ported plugins. Multibeam sonar requires the FRD sensor frame (Section 4).
- The connection ArduSub ↔ Gazebo (if you want flight-controller-in-the-loop) is provided by `ardupilot_gazebo`; `clydemcqueen/orca4` is a complete working reference of BlueROV2 + ArduSub + mavros + Nav2 in Harmonic, and `orca5` demonstrates single-camera SLAM without Nav2/mavros. Use Orca4/Orca5 as a skeleton for "what a working BlueROV2 SLAM stack looks like."
- Bridge: a `ros_gz_bridge` YAML maps every gz topic to the interface-contract ROS topic with the correct type and the correct frame_id. This bridge file *is* the Gazebo-side adapter.

### 7.2 Stonefish + `stonefish_ros2` (high-fidelity marine sim)
- Build the **Stonefish** C++ library from source, then the **`stonefish_ros2`** package (graphical "standard simulator" node that loads a **scenario XML** and publishes selected sensors / subscribes actuator setpoints).
- Author a scenario XML in `auv_sim_stonefish` describing: the ocean + seafloor world (with wavelength-dependent absorption/scattering — Stonefish models this properly, not just blue fog), the BlueROV2 (links, joints, geometry-based hydrodynamics, the 6-thruster vectored config), and the sensor suite: stereo camera, IMU, DVL, pressure, GNSS (at surface), USBL, **FLS**, **SSS**, **MSIS**, and **multibeam**. Stonefish's enhanced sonar shaders give realistic acoustic imagery with ground truth — ideal for training/validating the sonar-SLAM modules.
- The Stonefish-side adapter is the scenario's sensor naming + a thin remap layer so Stonefish topics land on the exact interface-contract names/frames.
- Geometry note: Stonefish computes hydrodynamics from actual body geometry, so provide a physics-simplified BlueROV2 mesh (separate from the high-poly visual mesh) — this lives in `auv_description`.

### 7.3 UUV Simulator / Gazebo Classic (optional, legacy — you probably don't need it)
- **Honest assessment:** UUV Simulator is built on **Gazebo Classic, which is EOL (Jan 2025)**. Its valuable parts — the image-sonar and DVL sensor models — were absorbed into **Project DAVE** (DAVE's heritage is literally UUV Simulator + WHOI `ds_sim`). So your "UUV" requirement is effectively satisfied by DAVE on modern Gazebo.
- Only stand up `auv_sim_uuv` (Gazebo Classic + ROS 1 bridge or a Humble side-install) if you specifically need to reproduce a legacy UUV-Sim result. If you do, isolate it in Docker; do not let Gazebo-Classic dependencies leak into the main workspace. Treat it as a throwaway comparison environment, not a maintained target.

### 7.4 The simulation-abstraction layer (`auv_sim_common`) — the keystone
- Define the interface contract (Section 6) as the *only* thing downstream code subscribes to.
- Each simulator gets an adapter (the gz bridge YAML; the Stonefish remap layer) that normalizes its native topics/frames/types onto the contract.
- A single `ground_truth_republisher` node takes whichever sim's ground-truth pose and republishes it on the canonical `/ground_truth/pose` + a separate TF chain.
- Result: `ros2 launch auv_bringup bringup.launch.py sim:=gazebo` vs `sim:=stonefish` swaps simulators with **zero** changes to estimation/SLAM/mapping packages. This single property is what makes the multi-simulator requirement sane.

---

## 8. Vehicle model — BlueROV2

### `auv_description`
- **URDF/Xacro** describing the BlueROV2: central enclosure + frame, 6-thruster **vectored** configuration (4 vectored horizontal thrusters at ~45°, 2 vertical), plus a `vectored_6dof` ("Heavy", 8-thruster) variant behind a xacro arg. Reference thruster allocation: the vectored-frame mixing in `bluerov_ros_playground` (per-thruster X/Y/Z + roll/pitch/yaw map, e.g. `0.707 0.707 0 0 0 ∓0.167`) is a good sanity check for your allocation matrix.
- **Two meshes per body:** a high-poly visual mesh and a simplified collision/hydro mesh (Stonefish needs clean geometry; Gazebo collision should be cheap).
- **Sensor mounting via xacro macros:** one macro per sensor type (camera, imu, dvl, pressure, fls, sss, mbes), each placing a child link with explicit, documented extrinsics and the right axis convention (remember FRD for the multibeam in DAVE). Make sensor poses parameters so you can study sensor-placement effects later.
- **Inertial + hydrodynamic parameters:** mass, center of buoyancy/gravity, added-mass and linear/quadratic drag coefficients. Use BlueROV2 published figures as a starting point and expose them as parameters for tuning.
- **Three consumers of this description:** (1) `robot_state_publisher` for TF, (2) Gazebo spawn (URDF→SDF or direct), (3) Stonefish scenario (the robot definition references the same simplified geometry + extrinsics). Keep a single source of truth (xacro) and generate the per-simulator variants from it.

---

## 9. Sensor suite — what each publishes and where it comes from

| Sensor | Interface topic(s) | Sim source (Gazebo/DAVE) | Sim source (Stonefish) | Consumed by |
|---|---|---|---|---|
| IMU/AHRS | `/imu/data` | gz IMU sensor | Stonefish IMU | state estimation, VIO, GTSAM preintegration |
| Pressure→depth | `/pressure` (+ derived depth) | gz pressure / DAVE | Stonefish pressure | state estimation, GTSAM depth factor |
| Magnetometer | `/magnetic` | gz magnetometer | Stonefish | heading aid |
| DVL | `/dvl` (`auv_interfaces/DVL`) | DAVE DVL plugin | Stonefish DVL | dead reckoning, state estimation, GTSAM velocity factor |
| USBL | `/usbl` (`auv_interfaces/USBLFix`) | DAVE USBL | Stonefish USBL | absolute-position prior |
| Stereo camera | `/cam/{left,right}/image_raw` + `camera_info` | gz camera | Stonefish camera | image enhancement → visual SLAM/VIO |
| FLS | `/sonar/fls/*` (`marine_acoustic_msgs`) | DAVE multibeam FLS | Stonefish FLS | imaging-sonar SLAM |
| MSIS | `/sonar/msis/*` | (limited) | Stonefish MSIS | imaging-sonar SLAM |
| SSS | `/sonar/sss/*` | (limited) | Stonefish SSS | side-scan mapping, place recognition |
| MBES | `/sonar/mbes/points` | DAVE multibeam | Stonefish multibeam | bathymetric SLAM, mapping |
| Ground truth | `/ground_truth/pose` | gz pose | Stonefish pose | evaluation only |

### 9.3 Sim-to-real seam (`auv_drivers`)
- In v1, `auv_drivers` contains **stubs/interfaces only**, but they define the contract real drivers will implement so the SLAM stack is hardware-agnostic from day one.
- The principle: a real DVL driver (e.g., Water Linked / Nortek), a real IMU/AHRS, a real sonar driver, and a real pressure driver each publish the **same interface-contract topics** the simulators do. Swapping sim→real is then "launch the driver instead of the sim adapter." Document this seam; do not implement hardware in v1.

---

## 10. SLAM subsystems

### 10.1 State-estimation backbone (`auv_state_estimation`) — always on
Two layers, build the first, then optionally the second:

1. **Loosely-coupled filter (`robot_localization`):** an EKF (or UKF) fusing IMU (orientation + angular velocity + linear acceleration), DVL (body velocity → twist), depth (z position, `z = -d`), and magnetometer (heading). Output: smooth `/odom` and `odom→base_link`. This is the reliable, well-understood baseline (and the reference you already trust from EKF/`robot_localization` work). Tune process/measurement noise carefully; handle DVL drop-outs by gating invalid measurements.
2. **Tightly-coupled factor-graph nav (optional, feeds `auv_slam_core`):** GTSAM **IMU preintegration** between keyframes + **DVL velocity factors** + **depth factors** + (when available) **USBL/GNSS position priors**. This produces metrically consistent odometry constraints for the back-end and is the research-grade path. Literature consistently uses GTSAM/iSAM2 for this; follow that.

**Acceptance idea (M6):** drive a lawnmower/boustrophedon path in sim; `evo` ATE of `/odom` vs ground truth should be bounded and grow slowly (dead-reckoning-like), with depth tracking within a few cm.

### 10.2 Visual SLAM (`auv_visual_slam`) + image enhancement (`auv_image_enhancement`)
- **Preprocessing is mandatory underwater.** Raw frames have color cast, haze, and low contrast that wreck ORB feature matching. `auv_image_enhancement` provides, behind a parameter, classical (CLAHE + gray-world/white-balance + UDCP-style dehazing) and optional learned restoration (e.g., a FUnIE-GAN / Sea-thru-style model). It publishes `/cam/.../image_enhanced`; visual SLAM consumes the enhanced stream.
- **Primary: ORB-SLAM3** via a ROS 2 wrapper (`Mechazo11/ros2_orb_slam3` has `humble` and `jazzy` branches; build ORB-SLAM3 as a shared lib with its DBoW2/g2o/Sophus thirdparty). Supports mono, stereo, RGB-D, **visual-inertial**, and **multi-map (Atlas)** with built-in loop closure — i.e., it alone covers "visual" and "visual-inertial." Beware: its vendored g2o is an older, pinned version; do not mix it with a system g2o. Pin OpenCV ≥4.2. Pangolin is a build dependency.
- **Alternative / complementary: RTAB-Map (`rtabmap_ros`)** for stereo/RGB-D dense mapping + appearance-based loop closure (binary ROS 2 packages, easy). Good for producing dense seafloor reconstructions and as a cross-check on ORB-SLAM3.
- **VIO front-end: OpenVINS** (MSCKF) for robust, drift-resistant *odometry* (not full SLAM/loop closure) to feed the factor graph when you want tight visual-inertial fusion separate from the full ORB-SLAM3 map.
- **Adapter:** a node converts the chosen visual system's keyframe poses + loop closures into interface-contract constraints for `auv_slam_core`. Working reference: Orca4/Orca5 fuse a down-facing-camera SLAM pose into the vehicle estimator — mirror that pattern.

### 10.3 Imaging-sonar SLAM (`auv_sonar_slam`, FLS/MSIS)
- **No turnkey ROS 2 package exists.** Build it, guided by the established literature (Section 16): FLS feature extraction via **CFAR detection** (e.g., SO-CFAR + adaptive threshold) → feature point clouds → **weighted ICP** registration → factor-graph constraints; or **MSIS pose-SLAM with GMM scan matching** on an iSAM2 back-end (as demonstrated on the Sparus II AUV). These approaches report meaningful improvement over pure dead reckoning.
- Pipeline: sonar image (`marine_acoustic_msgs`) → denoise/segment (CFAR) → extract acoustic landmarks / keypoints → scan-to-scan or scan-to-submap registration (PCL GICP/NDT or a probabilistic ICP) → relative-pose constraints + acoustic loop closures → `auv_slam_core`.
- This module is also the most valuable to validate against **Stonefish's high-fidelity sonar** (known ground truth), then DAVE's GPU sonar.

### 10.4 Bathymetric (MBES) SLAM (`auv_sonar_slam`, profiling)
- **Submap approach (canonical):** compound consecutive MBES swaths using dead-reckoning into seafloor point-cloud **submaps**; register overlapping submaps with **probabilistic ICP / GICP** (point-to-point coarse → point-to-plane fine), propagating uncertainty; add the registrations as loop-closure constraints. This is the Palomer/Ridao/Ribas line of work (UdG/VICOROB) and the standard for 2.5D bathymetric SLAM. Subsample with Difference-of-Normals to cut cost and avoid ICP local minima.
- **Toolkits to leverage:** KTH SMaRC's `auvlib` (multibeam + side-scan processing/IO) and the `UWExploration`/bathymetric-SLAM stack (GP-based submap representations, registration) are real, citable references — use them to bootstrap data structures and registration rather than starting from zero.
- Output feeds both `auv_slam_core` (constraints) and `auv_mapping` (the 2.5D bathymetry grid / mesh).

### 10.5 Side-scan mapping + place recognition (`auv_place_recognition`)
- SSS produces high-resolution "waterfall" imagery, weak for metric SLAM but strong for **mapping (mosaicking)** and **acoustic place recognition** (detecting revisits). Build SSS mosaicking in `auv_mapping` and an acoustic loop-closure detector (image-retrieval-style matching on SSS/FLS) in `auv_place_recognition` that emits loop-closure candidates to `auv_slam_core`.
- Mirror the visual side: `auv_place_recognition` hosts both **visual** (DBoW2 / learned descriptors) and **acoustic** place recognition, so loop closures from any modality enter the graph through one path.

### 10.6 The factor-graph integrator (`auv_slam_core`) — the unifier
- **GTSAM + iSAM2.** This node owns the single global pose graph. It ingests:
  - odometry constraints (from the estimation backbone / IMU preintegration),
  - DVL velocity + depth factors,
  - visual VIO relative poses + visual loop closures,
  - sonar registration constraints (FLS/MSIS/MBES) + acoustic loop closures,
  - USBL/GNSS absolute priors.
- It runs incremental optimization (iSAM2) and publishes `/slam/pose`, `/slam/trajectory`, the `map→odom` correction, and `SlamStatus`. **This is the literal answer to "do all forms of SLAM": every modality is a factor type into one graph.**
- Design factors as pluggable so a modality can be toggled. Use robust noise models (e.g., Huber/Cauchy) on loop closures to survive false acoustic/visual matches. Keep a clean separation between "constraint producers" (the front-ends) and "the graph" (this node).

### 10.7 Mapping outputs (`auv_mapping`)
- Dense point cloud + mesh from visual (RTAB-Map / TSDF) and from MBES.
- 2.5D **bathymetry** grid (gridded seafloor elevation + uncertainty).
- 3D **occupancy** (octomap-style) for volumetric representation.
- Optical and SSS **mosaics**.
- All map products are re-anchored to the optimized trajectory from `auv_slam_core` (re-render on significant loop closures).

---

## 11. Phased build plan (M0 → M11)

Each milestone: **Goal · Packages · Deliverable · Acceptance test.** Do them in order.

### M0 — Workspace, tooling, reproducible environment
- **Goal:** an empty-but-correct workspace that builds, with Docker + CI.
- **Packages:** repo skeleton, `docker/`, `.github/workflows/`, `AGENTS.md`.
- **Deliverable:** Dockerfile pinning Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + build deps; `colcon build` succeeds on an empty `src/`; CI runs build + (empty) test in the container.
- **Acceptance:** `docker build` succeeds; inside it, `colcon build` and `colcon test` run green; ROS 2 CLI works.

### M1 — Interface contract
- **Goal:** freeze the API (Section 6) before anything depends on it.
- **Packages:** `auv_interfaces`, plus a written `INTERFACE.md` in `auv_bringup` listing every topic/type/frame/QoS.
- **Deliverable:** all custom msgs/srvs/actions build; `marine_acoustic_msgs` vendored and building.
- **Acceptance:** `ros2 interface show` works for every custom type; the contract doc is complete and reviewed.

### M2 — Vehicle description
- **Goal:** a correct BlueROV2 URDF/xacro with sensor macros.
- **Packages:** `auv_description`.
- **Deliverable:** xacro → URDF; `robot_state_publisher` brings up the full TF tree; meshes (visual + collision/hydro) load.
- **Acceptance:** in RViz2, the BlueROV2 renders with all sensor frames in the correct poses and the TF tree matches Section 4; `check_urdf` passes.

### M3 — Gazebo bring-up
- **Goal:** BlueROV2 alive in Gazebo Harmonic with underwater physics and teleop.
- **Packages:** `auv_sim_gazebo`, `auv_sim_common`, `auv_control` (thruster allocation + simple teleop).
- **Deliverable:** spawn in an underwater world; Buoyancy + Hydrodynamics + Thruster systems configured; keyboard/joy teleop drives it; `/ground_truth/pose` published.
- **Acceptance:** teleop produces stable, physically plausible motion (neutral buoyancy, no explosion); ground-truth pose tracks commanded motion; `ros2 topic hz` healthy on core topics.

### M4 — Stonefish bring-up
- **Goal:** the *same* vehicle alive in Stonefish, publishing into the *same* contract.
- **Packages:** `auv_sim_stonefish`, `auv_sim_common`.
- **Deliverable:** Stonefish + `stonefish_ros2` load a scenario XML with the BlueROV2 and core sensors; thruster setpoints accepted; sensors published; Stonefish adapter remaps to contract topics/frames.
- **Acceptance:** `sim:=stonefish` launches; the same teleop drives the vehicle; the same core topics appear with identical names/frames as M3 (verify by `ros2 topic list` diff being empty for contract topics).

### M5 — Full sensor plumbing
- **Goal:** every sensor (cameras, IMU, DVL, depth, mag, USBL, FLS, SSS, MSIS, MBES) publishing into the contract from **both** sims.
- **Packages:** `auv_sim_gazebo` (DAVE sensors + bridge), `auv_sim_stonefish` (scenario sensors), `auv_interfaces`.
- **Deliverable:** all topics in Section 9 live; correct frames (FRD where required); depth-sign verified.
- **Acceptance:** `ros2 topic echo` shows sane data for every sensor in both sims; a unit test asserts depth sign (`z = -d`); sonar point clouds reproject into the world correctly in RViz (overlay on ground truth).

### M6 — State-estimation backbone
- **Goal:** robust odometry from IMU + DVL + depth.
- **Packages:** `auv_state_estimation` (robot_localization EKF), `auv_dead_reckoning`.
- **Deliverable:** EKF publishing `/odom` + `odom→base_link`; DVL drop-out gating; depth fused as z.
- **Acceptance:** on a lawnmower mission, `evo_ape` of `/odom` vs `/ground_truth/pose` is bounded and grows slowly; depth error within a few cm; no drift spikes on DVL drop-outs (test by forcing unlock).

### M7 — Visual SLAM front-end
- **Goal:** visual + visual-inertial SLAM producing poses and loop closures.
- **Packages:** `auv_image_enhancement`, `auv_visual_slam`, `auv_place_recognition` (visual half).
- **Deliverable:** enhanced image stream; ORB-SLAM3 (stereo and stereo-inertial) running on the enhanced stream over a textured seafloor world; constraint adapter emitting to `auv_slam_core`'s interface (even if core is still a stub).
- **Acceptance:** on a loop trajectory over textured seafloor, ORB-SLAM3 initializes, tracks, and **detects the loop closure**; estimated trajectory `evo_ape` beats raw dead reckoning. (Expect tracking loss over textureless sand — that's realistic; document it and rely on fusion.)

### M8 — Sonar SLAM front-ends
- **Goal:** MBES submap SLAM + FLS/MSIS imaging-sonar SLAM producing constraints.
- **Packages:** `auv_sonar_slam`, `auv_place_recognition` (acoustic half).
- **Deliverable:** MBES submap building + GICP/probabilistic-ICP registration with uncertainty; FLS CFAR feature extraction + registration; acoustic loop-closure candidates.
- **Acceptance:** MBES submap registration reduces trajectory error vs dead reckoning on a terrain with relief (validate against ground truth); FLS features are stable and registrations are geometrically consistent on Stonefish's high-fidelity sonar.

### M9 — Factor-graph integrator
- **Goal:** fuse everything into one consistent estimate.
- **Packages:** `auv_slam_core` (GTSAM/iSAM2).
- **Deliverable:** the global pose graph ingesting odometry + DVL/depth + visual VIO/loop + sonar registration/loop + USBL priors; robust loss on loop closures; publishing `/slam/pose`, `/slam/trajectory`, `map→odom`, `SlamStatus`.
- **Acceptance:** with all modalities enabled, fused `evo_ape`/`evo_rpe` beats every single-modality result; enabling/disabling any modality via launch arg works; injecting a false loop closure does not break the estimate (robust kernel holds).

### M10 — Mapping + evaluation harness
- **Goal:** map products + a repeatable evaluation/regression system.
- **Packages:** `auv_mapping`, `auv_evaluation`.
- **Deliverable:** dense cloud/mesh, 2.5D bathymetry, octomap, mosaics, all anchored to the optimized trajectory; `auv_evaluation` runs `evo` ATE/RPE automatically and a **rosbag-driven regression test** (record canonical missions once, replay deterministically in CI).
- **Acceptance:** maps are visibly consistent (no double-walls/ghosting after loop closure); `auv_evaluation` emits ATE/RPE tables for a benchmark suite; rosbag regression runs in CI and flags accuracy regressions.

### M11 — Robustness, sim-to-real seams, docs, research extensions
- **Goal:** harden and document; open the seams in Section 17.
- **Packages:** `auv_drivers` (interface stubs), all (docs), `auv_bringup` (one-command launch presets).
- **Deliverable:** failure-injection tests (sensor dropout, turbidity, no-DVL-lock); `auv_drivers` contract stubs for real hardware; per-package READMEs; top-level tutorials; launch presets (`visual_only`, `sonar_only`, `full_fusion`, each × `sim:=gazebo|stonefish`).
- **Acceptance:** every preset launches and self-reports health; docs let a new user run the full stack from a clean clone via Docker; the project README's quick-start works end-to-end.

---

## 12. Testing and validation

- **Ground truth is free in sim — use it relentlessly.** Every estimation/SLAM claim is validated by `evo` ATE/RPE against `/ground_truth/pose`. Never let SLAM consume ground truth.
- **Unit tests** (gtest/pytest): coordinate conversions (ENU↔NED, depth sign, DVL frame transform), message adapters, factor construction.
- **Integration tests** (`launch_testing`): bring up a sim + a subsystem, assert topics/TF/QoS and basic accuracy bounds.
- **Rosbag regression:** record canonical missions (lawnmower, loop, terrain-relief, textureless patch) **once**; replay deterministically in CI; assert ATE/RPE don't regress beyond thresholds.
- **CI:** GitHub Actions building and testing inside the M0 Docker image. Headless sim where possible; otherwise gate heavy sim tests behind a manual/nightly job.
- **Two-sim cross-validation:** the same mission in Gazebo and Stonefish should yield consistent qualitative behavior; large divergence flags a frame/extrinsic/units bug.

---

## 13. Configuration and launch architecture

- **Params:** per-subsystem YAML in each package's `config/`, aggregated by `auv_bringup`. No magic numbers in code.
- **Launch:** small composable launch files (`description.launch.py`, `sim_gazebo.launch.py`, `sim_stonefish.launch.py`, `estimation.launch.py`, `visual_slam.launch.py`, `sonar_slam.launch.py`, `slam_core.launch.py`, `mapping.launch.py`) included by a top-level `bringup.launch.py` driven by args: `sim:={gazebo,stonefish}`, `modalities:={visual,sonar,all}`, `use_sim_time:=true`, `rviz:=true`, `record:=false`.
- **Composition:** run heavy nodes (SLAM, sonar) as components in a container where it helps throughput, but keep them launchable standalone for debugging.

---

## 14. Tooling, visualization, debugging

- **RViz2** with saved configs per subsystem (TF, point clouds, trajectories, ground-truth overlay).
- **Foxglove Studio** for richer multi-stream inspection and remote viewing.
- **PlotJuggler** for time-series (estimator residuals, depth tracking, covariance).
- **`evo`** for ATE/RPE and trajectory plots.
- **tf2 tools** (`view_frames`, `tf2_echo`) — keep `view_frames` output in `docs/`, regenerate when the tree changes.
- **rqt** (graph, reconfigure, console) for live introspection.

---

## 15. Underwater-specific pitfalls (the field guide)

1. **ENU/NED and depth sign** — Section 4. The most common catastrophic bug. Convert at one boundary; assert in tests.
2. **DVL drop-outs** — no bottom lock ⇒ invalid velocity. Gate it; never fuse invalid measurements; expect drift while unlocked and lean on other modalities.
3. **Time synchronization** — everything on `use_sim_time` + `/clock`; use `message_filters` ApproximateTime for stereo + IMU + sonar alignment. Unsynced stamps quietly destroy VIO and registration.
4. **Visual scale ambiguity** — monocular has no metric scale. Use stereo, or VIO, or anchor scale with DVL/depth. Don't ship a mono-only metric map.
5. **Underwater image degradation** — without enhancement, ORB matching collapses. Enhancement is part of the pipeline, not optional.
6. **Sonar noise / CFAR tuning** — speckle and multipath need proper CFAR + robust registration; validate on Stonefish's modeled sonar before trusting on noisier data.
7. **TF consistency** — exactly one publisher per edge; static extrinsics from URDF only; `map→odom` only from `auv_slam_core`; `odom→base_link` only from estimation. Two publishers on one edge ⇒ chaos.
8. **Stonefish vs Gazebo conventions** — they differ in detail (frames, sonar mounting). The adapters in `auv_sim_common` exist precisely to normalize this; never special-case a simulator inside SLAM code.
9. **Build/dep conflicts** — ORB-SLAM3 pins old g2o/DBoW2/Sophus; do not mix with system versions. Pin OpenCV (≥4.2) and Pangolin. Vendor and isolate.
10. **Performance** — GPU sonar sim + ORB-SLAM3 + dense mapping is heavy. Budget GPU, run modalities you need, and use logged-data replay for full fusion if real-time isn't reached.
11. **QoS mismatches** — sensor data is Best-Effort; subscribing Reliable ⇒ "no messages." Document QoS on every endpoint.
12. **Loop-closure false positives** — turbid water + repetitive seabed ⇒ bad matches. Robust kernels (Huber/Cauchy) and geometric verification before accepting any loop closure into the graph.

---

## 16. Repositories and references to clone / draw from

Vendor code repos as git submodules under `third_party/`; pin commits.

**Simulators**
- Stonefish library — `patrykcieslak/stonefish`; ROS 2 interface — `patrykcieslak/stonefish_ros2` (scenario-XML driven; rich underwater sensors incl. sonar). Paid setup support is offered by the author if needed.
- Project DAVE (ROS 2 / Gazebo Harmonic port) — field-robotics-lab DAVE docs + the ROS 2 notion/site; underwater sensors (DVL, USBL, multibeam FLS, lidar), GPU sonar. Check the migration-progress page for ported plugins.
- `ros_gz` — ROS 2 ↔ Gazebo bridge.

**BlueROV2 / vehicle**
- `CentraleNantesROV/bluerov2` — clean BlueROV2 for ROS 2 + Gazebo (description, hydrodynamics, ground-truth pose, cascaded-PID control). Apache-2.0.
- `clydemcqueen/bluerov2_gz` — BlueROV2 for Gazebo Harmonic (vectored + Heavy/`vectored_6dof`), `ardupilot_gazebo` integration.
- `clydemcqueen/orca4` (+ `orca5`) — full working ROS 2 AUV on BlueROV2: ArduSub + mavros + Nav2 + Gazebo Harmonic; **orca4 fuses ORB-SLAM2 pose into the estimator; orca5 demonstrates single-camera SLAM**. Best end-to-end skeleton.
- `patrickelectric/bluerov_ros_playground` — original BlueRobotics playground; thruster mixing / vectored-frame map reference.
- `ardupilot_gazebo` — ArduSub ↔ Gazebo link (use the `ros2` branch).

**Visual SLAM**
- `UZ-SLAMLab/ORB_SLAM3` — the library (mono/stereo/RGB-D, VIO, multi-map). GPLv3.
- `Mechazo11/ros2_orb_slam3` — native ROS 2 wrapper with `humble` and `jazzy` branches. Preferred starting wrapper.
- `introlab/rtabmap_ros` — RTAB-Map for ROS 2 (dense RGB-D/stereo mapping + appearance loop closure).
- OpenVINS (`rpng/open_vins`) — MSCKF visual-inertial odometry.

**State estimation / back-end**
- `cra-ros-pkg/robot_localization` — EKF/UKF fusion.
- GTSAM (`borglab/gtsam`) — factor graphs + iSAM2 (the integrator and the tightly-coupled nav factors).
- PCL — point-cloud registration (GICP/NDT) for sonar SLAM.

**Sonar / bathymetric SLAM (references — mostly build-your-own, guided by these)**
- `marine_acoustic_msgs` (apl-ocean-engineering / ros-perception) — standard sonar message types. Vendor this.
- KTH SMaRC `auvlib` — multibeam + side-scan processing/IO; and the `UWExploration` / bathymetric-SLAM stack — GP submaps + registration. Bootstrap data structures from here.
- Literature to implement against:
  - Palomer, Ridao, Ribas — *Multibeam 3D Underwater SLAM with Probabilistic Registration*, Sensors 2016 (UdG/VICOROB). Canonical submap + probabilistic-ICP bathymetric SLAM.
  - Vial et al. — *Underwater Pose SLAM using GMM scan matching for a mechanical profiling sonar*, J. Field Robotics 2024 (MSIS, iSAM2/GTSAM, Sparus II).
  - FLS factor-graph SLAM with SO-CFAR + adaptive-threshold + weighted-ICP (2024) — FLS feature SLAM recipe.
  - *Underwater SLAM and Calibration with a 3D Profiling Sonar* (Remote Sensing 2026) — pose-graph SLAM + sonar extrinsic calibration via direct scan matching (3DupIC).

---

## 17. Stretch goals / natural research extensions

These open cleanly off the v1 seams and are where the project stops being an integration exercise and starts being publishable. Build the hooks even if you defer the work.

- **Learned loop closure for bathymetry / sonar.** Replace hand-crafted submap descriptors with learned keypoints/descriptors for data association in low-texture seabeds (the data-driven-loop-closure-in-bathymetric-point-clouds line of work, trained on real MBES from AUV missions). `auv_place_recognition` is the drop-in point.
- **Self-supervised / foundation-model features for underwater perception.** Swap classical ORB / acoustic features for self-supervised or foundation representations to improve robustness across turbidity and modality — a clean multi-sensor representation-learning study, and a natural fusion-of-modalities contribution.
- **Uncertainty-aware bathymetric mapping with Gaussian Processes.** GP submaps with calibrated uncertainty (KTH-style) for both mapping and informative/active SLAM.
- **Neural / Gaussian-splat seafloor reconstruction.** Modern implicit or 3DGS reconstruction conditioned on the optimized trajectory for dense, photoreal seafloor maps.
- **Active SLAM.** Use submap saliency / information gain to plan revisits that maximize loop-closure quality — closes the loop from `auv_slam_core` back into `auv_control`.
- **Cross-modal calibration online.** Estimate sonar/camera/DVL extrinsics as graph variables (as in the 3D-profiling-sonar calibration work) rather than fixing them.

Each of these is a self-contained module behind the existing interface contract, so they can be developed and benchmarked without disturbing the core stack — and each maps to a clean industrial-deliverable-plus-paper split.

---

## 18. Cursor operating playbook

### `.cursorrules` / `AGENTS.md` (put this in the repo root)
```
PROJECT: AUV multi-modal SLAM stack (ROS 2 Jazzy, Gazebo Harmonic, Stonefish, BlueROV2).
BUILD ORDER: follow the milestones M0..M11 in AUV_MultiSLAM_ROS2_Cursor_Build_Spec.md. One at a time.
NEVER:
  - invent ROS APIs, message fields, plugin names, or launch parameters. Read upstream READMEs/headers first.
  - add a third-party dependency not listed in Section 16 without asking.
  - mix ORB-SLAM3's vendored g2o/DBoW2/Sophus with system versions.
  - publish the same TF edge from two nodes.
  - let SLAM nodes consume ground truth.
  - special-case a simulator inside SLAM code (use the adapters in auv_sim_common).
ALWAYS:
  - default use_sim_time:=true.
  - put all custom messages in auv_interfaces only.
  - colcon build + run the milestone acceptance test before moving on; report evidence.
  - pin versions; vendor upstream as submodules with fixed commits.
  - document each package's consumed/produced topics, frames, params in its README.
  - validate every estimation/SLAM claim with evo against ground truth.
CONVENTIONS: ENU world / FLU body everywhere in SLAM; convert NED/FRD at sensor boundaries; depth z = -d.
```

### How to drive Cursor through it
1. **One milestone per session.** Prompt: *"Implement milestone M3 from the spec. List the packages you'll touch, then implement, then build, then run the M3 acceptance test and report results. Do not start M4."*
2. **Make it prove each step.** Require the acceptance evidence (topic echoes, `evo` numbers, RViz description) before accepting a milestone as done.
3. **Bootstrap from references, don't reinvent.** Prompt it to study the named upstream repo (e.g., *"read CentraleNantesROV/bluerov2's description + launch, then adapt — don't copy blindly"*) before writing M3.
4. **Force version discipline.** When it proposes a dependency, ask it to state the exact version and why, and to add it to the Docker image, not just apt-install ad hoc.
5. **Guard the frames.** Before M5/M6, have it write and run the coordinate-conversion unit tests *first* (depth sign, ENU/NED, DVL frame). Tests before fusion.
6. **When upstream won't build on Jazzy:** instruct it to check branches/forks and try once honestly, then report — not silently downgrade the whole project to Humble.

---

## 19. Definition of done (v1)

The project is "done" for v1 when, from a clean clone:
1. `docker build` produces the pinned environment; `colcon build` and `colcon test` pass in CI.
2. `ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=all` and the same with `sim:=stonefish` both bring up the BlueROV2 with all sensors and all SLAM modalities.
3. On the benchmark mission suite, fused `auv_slam_core` output beats every single-modality baseline on `evo` ATE/RPE, in both simulators.
4. Map products (dense cloud/mesh, 2.5D bathymetry, octomap, mosaics) are visibly consistent after loop closure.
5. Toggling any modality on/off via launch args works; failure-injection tests (DVL dropout, turbidity, textureless seabed) degrade gracefully rather than diverging.
6. Every package has a README documenting its interface; the quick-start in the top-level README works end-to-end.
7. `auv_drivers` exposes the real-hardware contract stubs so a future BlueROV2-in-the-water bring-up is "launch drivers instead of sim adapters."

---

*Build it in order. Verify against ground truth at every step. Convert frames at one boundary. The factor graph is the place where "all forms of SLAM" actually become one system.*
