# AGENTS.md — AUV Multi-Modal SLAM Stack

PROJECT: AUV multi-modal SLAM stack (ROS 2, Gazebo Harmonic, Stonefish, BlueROV2).

TARGET TRACK: **Track B** — ROS 2 **Humble** + Ubuntu **22.04** + **Gazebo Harmonic**
(gz-sim 8) via the `ros_gz` vendor packages. The build host runs Humble/Harmonic, which
the spec (`AUV_MultiSLAM_ROS2_Cursor_Build_Spec.md`, §3) explicitly supports as Track B.
Everything is written so a later port to Track A (Jazzy/24.04) is a Docker base-image swap.

BUILD ORDER: follow the milestones M0..M11 in `AUV_MultiSLAM_ROS2_Cursor_Build_Spec.md`.
One at a time. Build, run, and pass the acceptance test for `Mn` before starting `Mn+1`.

NEVER:
  - invent ROS APIs, message fields, plugin names, or launch parameters. Read upstream
    READMEs/headers first.
  - add a third-party dependency not listed in spec §16 without asking.
  - mix ORB-SLAM3's vendored g2o/DBoW2/Sophus with system versions.
  - publish the same TF edge from two nodes.
  - let SLAM nodes consume ground truth.
  - special-case a simulator inside SLAM code (use the adapters in `auv_sim_common`).

ALWAYS:
  - default `use_sim_time:=true`.
  - put all custom messages in `auv_interfaces` only.
  - colcon build + run the milestone acceptance test before moving on; report evidence.
  - pin versions; vendor upstream as submodules with fixed commits.
  - document each package's consumed/produced topics, frames, params in its README.
  - validate every estimation/SLAM claim with `evo` against ground truth.

CONVENTIONS:
  - ENU world / FLU body everywhere in SLAM; convert NED/FRD at sensor boundaries.
  - depth: pressure increases with depth; in ENU `z = -d` (the #1 catastrophic bug).
  - exactly one publisher per TF edge: static extrinsics from URDF; `odom->base_link`
    only from estimation; `map->odom` only from `auv_slam_core`.

FRAMES (target TF tree): earth -> map -> odom -> base_link -> {sensor links, base_link_frd}.
