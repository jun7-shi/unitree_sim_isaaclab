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
RGBD image bridge. For V1.5 RGBD perception, use
`Isaac-Kitchen-G129-Dex1-Wholebody-Nav` and add
`--enable_nav_ros_rgbd_images`; Isaac Sim publishes RGB/depth images and the ROS
workspace locally downsamples depth into `/g1/head_rgbd/points` for Nav2
VoxelLayer.
`--disable_image_server` skips the unrelated teleimager ZMQ/WebRTC image server.
`--nav_minimal_dds` starts only the robot state, run command, reset pose, and
sim-state DDS objects needed for Nav2 command driving; it avoids hand/reward DDS
publishers that are not part of the navigation loop. In GUI mode the V1
navigation path also skips the original RGB camera observation-manager update,
because static-map navigation does not consume those images.

For GUI V1 performance debugging, `--nav_minimal_dds` renders the
action-provider view every provider tick by default. Use
`--nav_action_render_interval 4` or a larger interval only when intentionally
trading viewport smoothness for loop throughput. This knob does not apply when
a navigation camera bridge is active; the camera ROS graph keeps rendering
every tick so the render product stays current.

With profiling enabled, the Wholebody action provider prints
`[NavActionProfile]` averages every `--profile_interval` provider ticks. Use
that line to distinguish policy inference, command mixing, PhysX stepping,
scene update, render, and observation-manager costs before changing simulator
configuration.

Add `--no_render` for the V1.0 static-map TF/odom path when running without a
GUI. In that mode the walking action provider skips regular render and
observation-manager updates, and `sim_main.py` explicitly pumps the Isaac Sim
app loop so the ROS bridge OmniGraphs publish `/clock` and TF/odom. Use
`--headless` instead of `--no_render` when the RGBD image bridge is enabled.
The camera image publishers depend on Isaac Sim render product updates, and
`--no_render` intentionally suppresses regular rendering.

The bridge uses Isaac Sim's built-in `isaacsim.ros2.bridge` OmniGraph nodes and
matches NVIDIA's scripted equivalents for the UI graph shortcuts:

```text
Clock shortcut:
OnPlaybackTick -> IsaacReadSimulationTime -> ROS2PublishClock

Camera shortcut:
OnPlaybackTick -> OgnIsaacRunOneSimulationFrame
  -> IsaacCreateRenderProduct
  -> ROS2CameraInfoHelper
  -> ROS2CameraHelper(type=rgb)
  -> ROS2CameraHelper(type=depth)
```

ROS contract:

```text
map -> odom -> base_link
/clock rosgraph_msgs/Clock
/odom nav_msgs/Odometry
/g1/head_rgbd/camera_info sensor_msgs/CameraInfo
/g1/head_rgbd/rgb/image_raw sensor_msgs/Image
/g1/head_rgbd/depth/image_raw sensor_msgs/Image
```

`/odom` is local odometry and starts near zero at bridge creation. The bridge
sets `map -> odom` to the robot chassis world pose so `map -> base_link`
matches the Isaac Sim environment position and orientation. Override that
initial map alignment with `--nav_ros_map_odom_translation X Y Z` and
`--nav_ros_map_odom_rotation X Y Z W` when using a static map with a different
origin or yaw.

Default frame and topic contract:

```text
map_frame: map
odom_frame: odom
base_frame: base_link
clock_topic: /clock
odom_topic: /odom
tf_topic: tf
camera_info_topic: /g1/head_rgbd/camera_info
rgb_topic: /g1/head_rgbd/rgb/image_raw
depth_topic: /g1/head_rgbd/depth/image_raw
camera_frame: g1_front_rgbd_optical_frame
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
ros2 topic hz /g1/head_rgbd/rgb/image_raw
ros2 topic hz /g1/head_rgbd/depth/image_raw
ros2 topic echo --once /g1/head_rgbd/depth/image_raw.header.frame_id
```

Expected values:

```text
/odom.header.frame_id: odom
/odom.child_frame_id: base_link
/g1/head_rgbd/camera_info.header.frame_id: g1_front_rgbd_optical_frame
/g1/head_rgbd/depth/image_raw.header.frame_id: g1_front_rgbd_optical_frame
```

## Legacy Direct PointCloud2 Bridge

`--enable_nav_ros_pointcloud` remains available as a debugging path for Isaac
Sim's built-in `ROS2CameraHelper(type=depth_pcl)`, but it is not the V1.5
default because it publishes the full point cloud over the Isaac Sim ROS bridge.

Legacy contract:

```text
/g1/head_rgbd/points sensor_msgs/PointCloud2
```

Legacy verification:

```bash
ros2 topic hz /g1/head_rgbd/points
ros2 topic echo --once /g1/head_rgbd/points.header.frame_id
```

No conda packages are required for this bridge. ROS publishing is handled by
Isaac Sim's ROS2 Bridge extension rather than `rclpy`.
