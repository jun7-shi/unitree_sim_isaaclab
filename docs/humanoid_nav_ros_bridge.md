# Humanoid Navigation ROS Bridge

This document records the Unitree Isaac Sim ROS2 bridge path used by HUM-40.

## G1 TF/Odom

Launch the G1 navigation task with the ROS2 TF/odometry bridge enabled:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
export HUMANOID_NAVIGATION_G1_NAV_USD=/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/assets/g1_nav/g1_29dof_with_dex1_nav_depth.usd
conda run -n unitree_sim_lab python sim_main.py \
  --device cpu \
  --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav \
  --robot_type g129 \
  --enable_dex1_dds \
  --enable_nav_ros_tf_odom \
  --no_render
```

The bridge uses Isaac Sim's built-in `isaacsim.ros2.bridge` OmniGraph nodes:

```text
map -> odom -> base_link
/odom nav_msgs/Odometry
```

Default frame and topic contract:

```text
map_frame: map
odom_frame: odom
base_frame: base_link
odom_topic: /odom
tf_topic: tf
```

## Verification

Run these from a ROS 2 Humble shell while Isaac Sim is running:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /odom
ros2 topic echo --once /odom.header.frame_id
ros2 topic echo --once /odom.child_frame_id
```

Expected values:

```text
/odom.header.frame_id: odom
/odom.child_frame_id: base_link
```

No conda packages are required for this bridge. ROS publishing is handled by
Isaac Sim's ROS2 Bridge extension rather than `rclpy`.
