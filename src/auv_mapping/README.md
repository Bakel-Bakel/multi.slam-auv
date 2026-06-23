# auv_mapping

Map products anchored to the optimized trajectory (spec §10.7, M10).

- `mapping` node — accumulates `/mbes/submap_cloud` into a voxel-downsampled global
  seafloor cloud (`/map/cloud`), derives a 2.5D bathymetry `OccupancyGrid`
  (`/map/bathymetry`, shallowest-return per cell), and serves `SaveMap`
  (`/mapping/save_map`) to write `.pcd`/`.ply`.
- Dense visual reconstruction is produced in parallel by RTAB-Map (`auv_visual_slam`).
- Octomap / mosaic outputs plug in here (octomap_server on `/map/cloud`; SSS mosaicking).

```bash
ros2 service call /mapping/save_map auv_interfaces/srv/SaveMap "{path: /tmp/seafloor, format: all}"
```
