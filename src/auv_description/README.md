# auv_description

BlueROV2 URDF/xacro — the single source of truth for the vehicle (spec M2/§8).

## Produces
- The full static TF tree (spec §4): `base_link` (FLU) -> `base_link_frd` (FRD) and all
  sensor links (`imu_link`, `dvl_link`, `pressure_link`, `usbl_link`,
  `camera_{left,right}_link` + `_optical_link`, `fls_link`, `sss_{port,stbd}_link`,
  `mbes_link`).
- A Gazebo-ready model (sensors + hydrodynamics + thrusters) when `use_gazebo:=true`.

## Files
- `urdf/bluerov2.urdf.xacro` — root. Args: `heavy` (8-thruster), `use_gazebo`.
- `urdf/common.xacro` — materials + inertia macros.
- `urdf/sensors.xacro` — one mounting macro per sensor (parameterized extrinsics).
- `urdf/thrusters.xacro` — vectored 6/8 thruster layouts.
- `urdf/bluerov2.gazebo.xacro` — gz Harmonic sensors + Hydrodynamics + Thruster systems.

## Conventions
- FLU body; `base_link_frd` is a static 180° roll mirror for marine-convention sensors.
- Camera optical links follow REP-103 (z-forward, x-right, y-down).
- FLS and MBES links are rotated to **FRD** for the Project DAVE multibeam requirement.

## Run / verify (M2 acceptance)
```bash
# Generate URDF and validate
xacro $(ros2 pkg prefix auv_description)/share/auv_description/urdf/bluerov2.urdf.xacro > /tmp/bluerov2.urdf
check_urdf /tmp/bluerov2.urdf

# View in RViz (renders vehicle + all sensor frames)
ros2 launch auv_description description.launch.py rviz:=true gui:=true
```
