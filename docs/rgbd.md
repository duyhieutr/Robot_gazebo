# Coloured 3D map with an RGB-D camera

The default robot has a plain RGB camera and a 16-channel LiDAR with a vertical
field of view of ±15°. Mounted at 0.7 m, the LiDAR never sees the upper part of
the 2.4 m racks, so `/rtabmap/cloud_map` is thin and low. With `rgbd:=true` the
camera also measures depth, and RTAB-Map stores colour and depth with every map
node.

```bash
ros2 launch wsl_sim warehouse.launch.py nav:=true explore:=true perception:=false rgbd:=true
```

`rgbd` is off by default, so the benchmarks and every other launch are unchanged.

## What changes with `rgbd:=true`

| Where | Change |
|---|---|
| `wsl_amr.gazebo.xacro` | The `camera` sensor is an `rgbd_camera` (same name, frame, 640 x 480, 1.2 rad). Depth range 0.1 to 10 m. |
| `bridge_rgbd.yaml` | Bridges the depth image to `/camera/depth/image_raw`. Colour image and `camera_info` keep their names. |
| `slam.launch.py`, `rtabmap_rgbd.yaml` | RTAB-Map subscribes to colour, depth and `camera_info` next to the LiDAR scan. |
| RViz | Display **Map cloud colour (RTAB-Map, rgbd:=true)**, off by default. |

Registration and loop closure stay on the LiDAR (`Reg/Strategy 1`). The map
itself is built in 3D from the LiDAR **and** the camera: `rtabmap_rgbd.yaml` sets
`Grid/3D true` and `Grid/Sensor 2`. Without `Grid/3D true` the grid is 2D and
`/rtabmap/cloud_map` is that grid drawn flat at z = 0, whatever the sensors are.
Nav2 still gets a 2D `/rtabmap/map`, the projection of the 3D map. Visual
features are switched off (`Kp/MaxFeatures -1`).

Because camera points now enter the 2D projection, tilt the camera up (see
[3d_map.md](3d_map.md)) so it does not see the floor: floor points above
`Grid/MaxGroundHeight` (0.05 m) would appear as obstacles on the Nav2 map. If the
2D map looks polluted, set `Grid/Sensor` back to `"0"` in `rtabmap_rgbd.yaml`:
the cloud is then built from the LiDAR scans only.

## Viewing it

In RViz, untick **Map cloud (RTAB-Map)**, **Occupancy (RTAB-Map)**, the costmaps
and **LiDAR raw**, tick **Map cloud colour (RTAB-Map, rgbd:=true)**, and switch
the view to `Orbit` to look from the side.

## Cost

Rendering a depth camera and storing images makes the simulation heavier. If it
stutters, lower `camera_rate_hz:=10` or `cloud_decimation` in `rtabmap_rgbd.yaml`
(larger value = fewer points).

## Limits

- Not run in Gazebo yet: the changes are checked for syntax and consistency only.
- A field named `rgb` does not prove the cloud is coloured: the point type always
  has it. Check the saved `.ply` (see [3d_map.md](3d_map.md)): z should span 0 to
  about 2.4 m and the colours should take many distinct values.
- The camera looks forward only: racks are coloured where the robot drove past them
  facing them.
