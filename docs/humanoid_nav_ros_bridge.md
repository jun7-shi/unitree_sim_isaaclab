# Humanoid Navigation ROS Bridge

This document records the Unitree Isaac Sim ROS2 bridge path used by HUM-40 and
HUM-41.

## G1 Kitchen Bridge

Launch the G1 Kitchen task with the ROS2 clock and TF/odometry bridges enabled:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
conda run -n unitree_sim_lab python sim_main.py \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-Kitchen-G129-Dex1-Wholebody \
  --robot_type g129 \
  --enable_nav_ros_clock \
  --enable_nav_ros_tf_odom \
  --enable_nav_udp_cmd_bridge \
  --nav_minimal_dds \
  --disable_image_server
```

The V1.0 acceptance path is static-map navigation, so it does not require the
PointCloud2 bridge. For V1.5 RGBD perception, add `--enable_nav_ros_pointcloud`
when the active task exposes a depth-capable `front_camera`.
`--disable_image_server` skips the unrelated teleimager ZMQ/WebRTC image server.
`--nav_minimal_dds` starts only the robot state, run command, reset pose, and
sim-state DDS objects needed for Nav2 command driving; it avoids hand/reward DDS
publishers that are not part of the navigation loop. In GUI mode it keeps the
original Unitree walking loop's render and observation-manager updates so idle
standing behavior matches the upstream simulator.

Add `--no_render` for the V1.0 static-map TF/odom path when running without a
GUI. In that mode the walking action provider skips regular render and
observation-manager updates, and `sim_main.py` explicitly pumps the Isaac Sim
app loop so the ROS bridge OmniGraphs publish `/clock` and TF/odom. Use
`--headless` instead of `--no_render` when the PointCloud2 bridge is enabled.
The camera `depth_pcl` publisher depends on Isaac Sim render product updates,
and `--no_render` intentionally suppresses regular rendering.

The bridge uses Isaac Sim's built-in `isaacsim.ros2.bridge` OmniGraph nodes and
matches NVIDIA's scripted equivalents for the UI graph shortcuts:

```text
Clock shortcut:
OnPlaybackTick -> IsaacReadSimulationTime -> ROS2PublishClock

Camera shortcut:
OnPlaybackTick -> OgnIsaacRunOneSimulationFrame
  -> IsaacCreateRenderProduct
  -> ROS2CameraInfoHelper
  -> ROS2CameraHelper(type=depth_pcl)
```

ROS contract:

```text
map -> odom -> base_link
/clock rosgraph_msgs/Clock
/odom nav_msgs/Odometry
/g1/head_rgbd/camera_info sensor_msgs/CameraInfo
/g1/head_rgbd/points sensor_msgs/PointCloud2
```

`/odom` is local odometry and starts near zero at bridge creation. The bridge
sets `map -> odom` to the robot chassis world position so `map -> base_link`
matches the Isaac Sim environment position. Override that initial map alignment
with `--nav_ros_map_odom_translation X Y Z` when using a static map with a
different origin.

Default frame and topic contract:

```text
map_frame: map
odom_frame: odom
base_frame: base_link
clock_topic: /clock
odom_topic: /odom
tf_topic: tf
camera_info_topic: /g1/head_rgbd/camera_info
pointcloud_topic: /g1/head_rgbd/points
pointcloud_frame: g1_head_d435_depth_optical_frame
```

## Verification

Run these from a ROS 2 Humble shell while Isaac Sim is running:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic echo --once /odom.header.frame_id
ros2 topic echo --once /odom.child_frame_id
ros2 topic echo --once /g1/head_rgbd/camera_info.header.frame_id
ros2 topic hz /g1/head_rgbd/points
ros2 topic echo --once /g1/head_rgbd/points.header.frame_id
```

Expected values:

```text
/odom.header.frame_id: odom
/odom.child_frame_id: base_link
/g1/head_rgbd/camera_info.header.frame_id: g1_head_d435_depth_optical_frame
/g1/head_rgbd/points.header.frame_id: g1_head_d435_depth_optical_frame
```

No conda packages are required for this bridge. ROS publishing is handled by
Isaac Sim's ROS2 Bridge extension rather than `rclpy`.
