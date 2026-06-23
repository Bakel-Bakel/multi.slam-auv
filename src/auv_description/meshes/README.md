# meshes

Per spec §8, each body has **two** meshes: a high-poly visual mesh and a simplified
collision/hydro mesh (Stonefish needs clean geometry; Gazebo collision should be cheap).

The current model uses primitive geometry (boxes/cylinders) so it builds and renders
with zero binary assets. To use the real BlueROV2 meshes, drop `.dae`/`.stl` files here
and reference them from `urdf/bluerov2.urdf.xacro` (visual) and a `*_collision.stl`
(collision/hydro). Recommended source: `CentraleNantesROV/bluerov2` description meshes
(Apache-2.0) — adapt, don't fork blindly.
