# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Isaac Sim ROS2 TF bridge for G1 RGBD robot self-filter links."""

from dataclasses import dataclass
from typing import Sequence

from ros2_bridge.g1_nav_tf_odom_bridge import (
    _enable_ros2_bridge_extension,
    _graph_exists,
    resolve_robot_prim_path,
    resolve_robot_root_prim_path,
)


DEFAULT_SELF_FILTER_LINK_NAMES = (
    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
)


@dataclass(frozen=True)
class G1NavSelfFilterTfBridgeConfig:
    graph_path: str = "/ActionGraph/HumanoidNavSelfFilterTf"
    robot_name: str = "robot"
    robot_root_prim_path: str | None = None
    parent_prim_path: str | None = None
    link_names: tuple[str, ...] = DEFAULT_SELF_FILTER_LINK_NAMES
    tf_topic: str = "tf"
    node_namespace: str = ""


def parse_self_filter_link_names(value: str | Sequence[str] | None) -> tuple[str, ...]:
    if value is None:
        return DEFAULT_SELF_FILTER_LINK_NAMES
    if isinstance(value, str):
        names = [name.strip() for name in value.split(",")]
    else:
        names = [str(name).strip() for name in value]
    return tuple(name for name in names if name)


def candidate_self_filter_link_paths(
    robot_root_prim_path: str,
    link_names: Sequence[str],
) -> list[str]:
    root = robot_root_prim_path.rstrip("/")
    return [f"{root}/{link_name}" for link_name in link_names]


def _find_stage_link_prim_paths(
    robot_root_prim_path: str,
    link_names: Sequence[str],
) -> list[str]:
    try:
        import omni.usd
        from pxr import Usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return []
        root_prim = stage.GetPrimAtPath(robot_root_prim_path)
        if not root_prim or not root_prim.IsValid():
            return []

        wanted = set(link_names)
        found: dict[str, str] = {}
        for prim in Usd.PrimRange(root_prim):
            name = prim.GetName()
            if name in wanted and name not in found:
                found[name] = str(prim.GetPath())

        return [found[name] for name in link_names if name in found]
    except Exception:
        return []


def resolve_self_filter_link_prim_paths(
    robot_root_prim_path: str,
    link_names: Sequence[str],
) -> list[str]:
    return _find_stage_link_prim_paths(
        robot_root_prim_path,
        link_names,
    ) or candidate_self_filter_link_paths(robot_root_prim_path, link_names)


def create_g1_nav_self_filter_tf_graph(
    env,
    config: G1NavSelfFilterTfBridgeConfig | None = None,
) -> dict[str, str]:
    """Create a ROS2 TF graph for selected G1 self-filter link poses."""
    import omni.graph.core as og
    import usdrt.Sdf

    config = config or G1NavSelfFilterTfBridgeConfig()
    _enable_ros2_bridge_extension()

    robot_root_prim_path = config.robot_root_prim_path or resolve_robot_root_prim_path(
        env,
        config.robot_name,
    )
    parent_prim_path = config.parent_prim_path or resolve_robot_prim_path(
        env,
        config.robot_name,
    )
    target_prim_paths = resolve_self_filter_link_prim_paths(
        robot_root_prim_path,
        config.link_names,
    )
    if not target_prim_paths:
        raise ValueError("no G1 self-filter link prims resolved")

    if _graph_exists(og, config.graph_path):
        return {
            "graph_path": config.graph_path,
            "robot_root_prim_path": robot_root_prim_path,
            "parent_prim_path": parent_prim_path,
            "target_prim_paths": ",".join(target_prim_paths),
            "tf_topic": config.tf_topic,
        }

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": config.graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishSelfFilterTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("PublishSelfFilterTF.inputs:topicName", config.tf_topic),
                ("PublishSelfFilterTF.inputs:nodeNamespace", config.node_namespace),
                (
                    "PublishSelfFilterTF.inputs:parentPrim",
                    [usdrt.Sdf.Path(parent_prim_path)],
                ),
                (
                    "PublishSelfFilterTF.inputs:targetPrims",
                    [usdrt.Sdf.Path(path) for path in target_prim_paths],
                ),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishSelfFilterTF.inputs:execIn"),
                ("Context.outputs:context", "PublishSelfFilterTF.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishSelfFilterTF.inputs:timeStamp",
                ),
            ],
        },
    )

    return {
        "graph_path": config.graph_path,
        "robot_root_prim_path": robot_root_prim_path,
        "parent_prim_path": parent_prim_path,
        "target_prim_paths": ",".join(target_prim_paths),
        "tf_topic": config.tf_topic,
    }
