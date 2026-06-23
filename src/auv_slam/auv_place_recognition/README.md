# auv_place_recognition

Visual + acoustic loop-closure detection (spec §10.5). Mirrors the visual and acoustic
sides so loop closures from **any** modality enter the factor graph through one path.

- `descriptors.py` — compact global (tiny-image/gist) descriptors + cosine matching
  (unit-tested). Seam for DBoW2 / learned descriptors (spec §17).
- `place_recognition` node — image stream + `/odom`; emits `auv_interfaces/LoopClosure`
  (`is_sequential=False`) on `/slam/loop_closures` with temporal + spatial + appearance
  gating to suppress false positives (turbid water / repetitive seabed, spec §15.12).

Run two instances (visual on enhanced camera, acoustic on a sonar waterfall image).
The relative-pose seed is the odometry difference; `auv_slam_core`'s robust kernel and
sonar registration refine/guard it.
