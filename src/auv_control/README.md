# auv_control

Thruster allocation + teleop to drive the BlueROV2 for data collection (spec §8, M3).

## Node: `thruster_allocator`
- Consumes `/cmd_vel` (`geometry_msgs/Twist`, desired body twist).
- Builds a 6xN thrust-allocation matrix from the thruster geometry (kept consistent
  with `auv_description/urdf/thrusters.xacro`), maps a body wrench to per-thruster
  forces with `pinv(TAM)`, saturates, and publishes `std_msgs/Float64` thrust commands
  on `/<prefix>/<name>` (bridged to the gz Thruster system).
- Zeroes thrust on `/cmd_vel` timeout (failsafe).
- Params in `config/allocation.yaml`: `thruster_names`, `thruster_poses_flat`,
  `wrench_gains`, `max_thrust`, `timeout_sec`, `rate_hz`.

## Teleop
`ros2 launch auv_control teleop.launch.py` (or run `teleop_twist_keyboard` directly).

The allocation matrix derivation and a surge-achievability check are unit-tested in
`test/test_allocation.py`.
