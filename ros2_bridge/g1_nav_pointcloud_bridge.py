# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Isaac Sim ROS2 PointCloud2 graph for the G1 head RGBD camera."""

from dataclasses import dataclass

from ros2_bridge.g1_nav_tf_odom_bridge import (
    _enable_ros2_bridge_extension,
    _graph_exists,
)


@dataclass(frozen=True)
class G1NavPointCloudBridgeConfig:
    graph_path: str = "/ActionGraph/HumanoidNavPointCloud"
    camera_name: str = "front_camera"
    camera_prim_path: str | None = None
    pointcloud_topic: str = "/g1/head_rgbd/points"
    frame_id: str = "g1_head_d435_depth_optical_frame"
    width: int = 640
    height: int = 480


def resolve_camera_prim_path(env, camera_name: str = "front_camera") -> str:
    if env is not None:
        camera = env.scene[camera_name]
        prim_path = getattr(camera, "prim_path", None)
        if prim_path and "*" not in prim_path:
            return prim_path

        cfg = getattr(camera, "cfg", None)
        cfg_prim_path = getattr(cfg, "prim_path", None)
        if cfg_prim_path:
            return cfg_prim_path.replace("env_.*", "env_0")

    return "/World/envs/env_0/Robot/d435_link/head_d435_depth_camera"


def create_g1_nav_pointcloud_graph(
    env,
    config: G1NavPointCloudBridgeConfig | None = None,
) -> dict[str, str]:
    """Create the ROS2 PointCloud2 OmniGraph for the G1 head RGBD camera."""
    import omni.graph.core as og
    import usdrt.Sdf

    config = config or G1NavPointCloudBridgeConfig()
    _enable_ros2_bridge_extension()

    camera_prim_path = config.camera_prim_path or resolve_camera_prim_path(
        env, config.camera_name
    )
    if _graph_exists(og, config.graph_path):
        return {
            "graph_path": config.graph_path,
            "camera_prim_path": camera_prim_path,
            "pointcloud_topic": config.pointcloud_topic,
            "frame_id": config.frame_id,
        }

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": config.graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                (
                    "CreateRenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),
                ("DepthPointCloud", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                (
                    "CreateRenderProduct.inputs:cameraPrim",
                    [usdrt.Sdf.Path(camera_prim_path)],
                ),
                ("CreateRenderProduct.inputs:width", config.width),
                ("CreateRenderProduct.inputs:height", config.height),
                ("DepthPointCloud.inputs:topicName", config.pointcloud_topic),
                ("DepthPointCloud.inputs:type", "depth_pcl"),
                ("DepthPointCloud.inputs:frameId", config.frame_id),
                ("DepthPointCloud.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "CreateRenderProduct.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "DepthPointCloud.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "DepthPointCloud.inputs:renderProductPath",
                ),
                ("Context.outputs:context", "DepthPointCloud.inputs:context"),
            ],
        },
    )

    return {
        "graph_path": config.graph_path,
        "camera_prim_path": camera_prim_path,
        "pointcloud_topic": config.pointcloud_topic,
        "frame_id": config.frame_id,
    }
