# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Isaac Sim ROS2 RGB/depth image graph for the G1 front RGBD camera."""

from dataclasses import dataclass

from ros2_bridge.g1_nav_pointcloud_bridge import resolve_camera_prim_path
from ros2_bridge.g1_nav_tf_odom_bridge import (
    _enable_ros2_bridge_extension,
    _graph_exists,
)


@dataclass(frozen=True)
class G1NavRgbdImageBridgeConfig:
    graph_path: str = "/ActionGraph/HumanoidNavRgbdImages"
    camera_name: str = "front_camera"
    camera_prim_path: str | None = None
    rgb_topic: str = "/g1/head_rgbd/rgb/image_raw"
    depth_topic: str = "/g1/head_rgbd/depth/image_raw"
    camera_info_topic: str = "/g1/head_rgbd/camera_info"
    frame_id: str = "g1_front_rgbd_optical_frame"
    node_namespace: str = ""
    width: int = 640
    height: int = 480


def create_g1_nav_rgbd_image_graph(
    env,
    config: G1NavRgbdImageBridgeConfig | None = None,
) -> dict[str, str]:
    """Create the ROS2 RGB/depth image OmniGraph for the G1 front camera."""
    import omni.graph.core as og

    config = config or G1NavRgbdImageBridgeConfig()
    _enable_ros2_bridge_extension()

    camera_prim_path = config.camera_prim_path or resolve_camera_prim_path(
        env, config.camera_name
    )
    if _graph_exists(og, config.graph_path):
        return {
            "graph_path": config.graph_path,
            "camera_prim_path": camera_prim_path,
            "rgb_topic": config.rgb_topic,
            "depth_topic": config.depth_topic,
            "camera_info_topic": config.camera_info_topic,
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
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("RgbImagePublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthImagePublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:width", config.width),
                ("CreateRenderProduct.inputs:height", config.height),
                ("CameraInfoPublish.inputs:topicName", config.camera_info_topic),
                ("CameraInfoPublish.inputs:frameId", config.frame_id),
                ("CameraInfoPublish.inputs:nodeNamespace", config.node_namespace),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                ("RgbImagePublish.inputs:topicName", config.rgb_topic),
                ("RgbImagePublish.inputs:type", "rgb"),
                ("RgbImagePublish.inputs:frameId", config.frame_id),
                ("RgbImagePublish.inputs:nodeNamespace", config.node_namespace),
                ("RgbImagePublish.inputs:resetSimulationTimeOnStop", True),
                ("DepthImagePublish.inputs:topicName", config.depth_topic),
                ("DepthImagePublish.inputs:type", "depth"),
                ("DepthImagePublish.inputs:frameId", config.frame_id),
                ("DepthImagePublish.inputs:nodeNamespace", config.node_namespace),
                ("DepthImagePublish.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "CreateRenderProduct.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "CameraInfoPublish.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraInfoPublish.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "RgbImagePublish.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "RgbImagePublish.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "DepthImagePublish.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "DepthImagePublish.inputs:renderProductPath",
                ),
                ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                ("Context.outputs:context", "RgbImagePublish.inputs:context"),
                ("Context.outputs:context", "DepthImagePublish.inputs:context"),
            ],
        },
    )

    return {
        "graph_path": config.graph_path,
        "camera_prim_path": camera_prim_path,
        "rgb_topic": config.rgb_topic,
        "depth_topic": config.depth_topic,
        "camera_info_topic": config.camera_info_topic,
        "frame_id": config.frame_id,
    }
