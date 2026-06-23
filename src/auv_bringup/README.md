# auv_bringup

Top-level orchestration (spec §13).

## Composable launch

```bash
ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=all rviz:=true
ros2 launch auv_bringup bringup.launch.py sim:=stonefish modalities:=sonar
```

| arg | values | default | meaning |
| --- | --- | --- | --- |
| `sim` | `gazebo`, `stonefish` | `gazebo` | which simulator (adapters keep the contract identical) |
| `modalities` | `visual`, `sonar`, `all` | `all` | which SLAM front-ends to run |
| `use_sim_time` | `true`, `false` | `true` | simulation-first |
| `rviz` | `true`, `false` | `false` | launch RViz with `rviz/auv.rviz` |
| `record` | `true`, `false` | `false` | `ros2 bag record` the contract topics |

## Presets

```bash
ros2 launch auv_bringup full_fusion.launch.py  sim:=gazebo
ros2 launch auv_bringup visual_only.launch.py  sim:=gazebo
ros2 launch auv_bringup sonar_only.launch.py   sim:=stonefish
```

The simulator and modality switches are the *only* place a simulator is named — the SLAM
layer consumes the contract topics in `INTERFACE.md` and never special-cases a backend.
This is what makes the sim→sim and sim→real swaps a launch-argument change.
