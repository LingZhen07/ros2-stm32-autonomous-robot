# Foxglove Visualization v1

The Orange Pi remains headless. Start the complete demo from a prepared shell:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source /usr/local/Ascend/ascend-toolkit/set_env.sh
source /data/ros2_ws/install/setup.bash
ros2 launch robot_visualization demo_v1.launch.py
```

Connect Chrome, Edge, or Foxglove Desktop on the same LAN to:

```text
ws://172.20.10.3:8765
```

The bridge binds only to `172.20.10.3`. Remote access, parameter operations,
services, system information, and hidden topics are disabled. The only enabled
client-publish capability is restricted to `^/visualization/goal$`.

## Minimal layout

Image panel:

- `/camera_processing/detection_image/compressed`
- `Sync timestamps`: off

3D panel, display frame `map`:

- `/map`
- `/scan`
- `/tf`
- `/tf_static`
- `/visualization/planned_path`
- `Sync timestamps`: off

Costmaps and footprints remain exposed for optional debugging, but they are not
part of the normal presentation layout. Foxglove pairs the `*_updates` channels
with each base OccupancyGrid automatically when those debug topics are selected.

The ROS-side raw annotated image remains available at
`/camera_processing/detection_image`, but it is intentionally excluded from the
Foxglove bridge whitelist. The normal demo transports the JPEG-compressed topic
at quality 75 and at no more than 12 Hz.

For planning, select the standard 3D-panel `2D pose` publish tool, set its topic
to `/visualization/goal`, and choose a nearby free-space pose in the `map`
frame. This requests a Navfn plan only. It does not execute the path or move the
robot.

Save the layout locally from Foxglove after arranging the Image and 3D panels;
the robot-side functionality does not depend on a cloud-hosted layout.
