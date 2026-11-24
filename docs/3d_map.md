# A 3D map you can save

Three tools, from "measured" to "clean model". Use them together.

| | What it is | Where the height comes from | File |
|---|---|---|---|
| Coloured point cloud | What the RGB-D camera actually saw, accumulated by RTAB-Map | Measured | `.ply` |
| Extruded map | The 2D occupancy map with every occupied cell raised to rack height | Assumed (`height`, default 2.4 m) | `.obj` |
| RTAB-Map database | Everything RTAB-Map stored (graph, scans, images) | Measured | `~/.ros/wsl_rtabmap.db` |

## Why the first 3D map looked flat

The main reason was the configuration: `rtabmap.yaml` has `Grid/3D "false"`, so
RTAB-Map built a 2D grid and `/rtabmap/cloud_map` was that grid drawn at z = 0
(a saved `.ply` had z min = max = 0.00). `rtabmap_rgbd.yaml` now sets
`Grid/3D "true"` and `Grid/Sensor "2"`. What follows explains why the camera
mounting matters once the map is 3D.

The camera sits 0.42 m above the floor and looks straight ahead with a vertical
field of view of 54°. In an aisle, 1.5 m from a rack, its top ray reaches
0.42 + 1.5 · tan 27° ≈ **1.2 m**. The racks are 2.4 m high, so the upper half was
never seen. A different camera model does not change that; the mounting does.

Tilting the camera up by 25° and giving it a wider vertical field of view moves the
top ray at 1.5 m to about **2.4 m**:

| Profile | `camera_hfov` | size | `rgbd_far` | vertical FOV | top ray at 1.5 m |
|---|---|---|---|---|---|
| Original | 1.2 | 640 x 480 | 10 | 54° | 1.2 m (no tilt) |
| D456-like | 1.52 (87°) | 848 x 480 | 6 | 57° | 2.4 m (tilt 25°) |
| L515-like | 1.222 (70°) | 640 x 480 | 9 | 55° | 2.4 m (tilt 25°) |

The D456 and L515 rows copy the field of view and range from the datasheets as
remembered; check them before quoting them. The simulation reproduces the optics
and range only, not the stereo artefacts of a D456 or the time-of-flight noise of
an L515.

```bash
# D456-like
ros2 launch wsl_sim warehouse.launch.py nav:=true explore:=true perception:=false \
  rgbd:=true rviz:=false camera_hfov:=1.52 camera_width:=848 camera_height:=480 \
  rgbd_far:=6.0 camera_pitch_deg:=25

# L515-like
ros2 launch wsl_sim warehouse.launch.py nav:=true explore:=true perception:=false \
  rgbd:=true rviz:=false camera_hfov:=1.222 camera_width:=640 camera_height:=480 \
  rgbd_far:=9.0 camera_pitch_deg:=25
```

With the camera tipped up it no longer sees the floor right in front of the robot.
That is fine for mapping racks; navigation and the 2D map use the LiDAR.

## Check a saved cloud

```bash
python3 - <<'EOF'
import numpy as np, os
raw = open(os.path.expanduser('~/warehouse_cloud.ply'), 'rb').read()
i = raw.index(b'end_header\n') + 11
dt = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1')])
a = np.frombuffer(raw[i:], dtype=dt)
print('points', len(a), '| z %.2f .. %.2f m' % (a['z'].min(), a['z'].max()))
print('points above 1.5 m: %.1f%%' % (100 * (a['z'] > 1.5).mean()))
print('distinct colours:', len(np.unique(np.stack([a['r'], a['g'], a['b']], 1), axis=0)))
EOF
```

A good 3D cloud spans z from about 0 to 2.4 m and has hundreds of distinct colours.

## 1. Save the coloured point cloud

After the robot has explored (or at any time):

```bash
ros2 run wsl_exploration save_cloud ~/warehouse_cloud.ply
```

Open the `.ply` in MeshLab, CloudCompare or Blender. If RTAB-Map has not
published the cloud yet, drive the robot a little, or:

```bash
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap \
  "{global_map: true, optimized: true, graph_only: false}"
```

## 2. Building-like model from the 2D map

```bash
ros2 launch wsl_exploration extrude.launch.py            # height:=2.4
```

In RViz: Add > By topic > `/exploration/extruded_map` > Marker. Save it:

```bash
ros2 service call /map_extruder/export std_srvs/srv/Trigger
```

This writes `~/warehouse_3d.obj`. Parameters (`height`, `min_component_cells`,
`smooth_cells`, `with_floor`, `export_path`) are in `map_extruder.py`. Small
isolated groups of cells (LiDAR speckle) are dropped and one-cell gaps are
closed, so the model is made of a few hundred boxes, not thousands.

Every occupied cell gets the same height, so the model is a clean shell of the
layout, not a record of what is on the shelves.

## 3. RTAB-Map database

RTAB-Map writes `~/.ros/wsl_rtabmap.db` when it shuts down, and replaces it at
the next start (`--delete_db_on_start`). Copy it to keep a run:

```bash
cp ~/.ros/wsl_rtabmap.db ~/warehouse_run1.db
```

If the RTAB-Map command-line tools are installed, `rtabmap-databaseViewer` opens
the copy and exports a 3D cloud or mesh.

## Limits

- Not run in Gazebo yet. The extrusion, mesh and PLY writers are unit-tested
  (`test/test_extrusion_utils.py`); the ROS nodes and the xacro changes are checked
  for syntax only.
- The camera looks forward only: a rack face is seen when the robot drives past it,
  at an angle. Expect much more height than before, not a perfect scan.
- A heavier camera (848 x 480, tilted) costs frame rate; add `camera_rate_hz:=10` if
  the simulation stutters.
