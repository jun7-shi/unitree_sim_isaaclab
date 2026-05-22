# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Isaac Sim ROS2 TF and odometry graph for G1 navigation."""

from dataclasses import dataclass


@dataclass(frozen=True)
class G1NavTfOdomBridgeConfig:
    graph_path: str = "/ActionGraph/HumanoidNavTfOdom"
    robot_name: str = "robot"
    robot_prim_path: str | None = None
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    odom_topic: str = "/odom"
    tf_topic: str = "tf"


_PREFERRED_CHASSIS_LINKS = ("pelvis", "base_link", "base", "torso_link")


def _enable_ros2_bridge_extension() -> None:
    import omni.kit.app

    extension_id = "isaacsim.ros2.bridge"
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(extension_id):
        manager.set_extension_enabled_immediate(extension_id, True)


def _candidate_chassis_prim_paths(robot_prim_path: str) -> list[str]:
    robot_prim_path = robot_prim_path.rstrip("/")
    leaf_name = robot_prim_path.rsplit("/", 1)[-1]
    if leaf_name in _PREFERRED_CHASSIS_LINKS:
        return [robot_prim_path]

    return [
        *(f"{robot_prim_path}/{link_name}" for link_name in _PREFERRED_CHASSIS_LINKS),
        robot_prim_path,
    ]


def _find_stage_chassis_prim_path(robot_prim_path: str) -> str | None:
    try:
        import omni.usd
        from pxr import Usd, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None

        for prim_path in _candidate_chassis_prim_paths(robot_prim_path):
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid() and (
                prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                or prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ):
                return prim_path

        root_prim = stage.GetPrimAtPath(robot_prim_path)
        if not root_prim or not root_prim.IsValid():
            return None

        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim.GetPath())
        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return str(prim.GetPath())
    except Exception:
        return None

    return None


def _resolve_chassis_prim_path(robot_prim_path: str) -> str:
    return (
        _find_stage_chassis_prim_path(robot_prim_path)
        or _candidate_chassis_prim_paths(robot_prim_path)[0]
    )


def resolve_robot_prim_path(env, robot_name: str = "robot") -> str:
    robot = env.scene[robot_name]

    prim_path = getattr(robot, "prim_path", None)
    if prim_path and "*" not in prim_path:
        return _resolve_chassis_prim_path(prim_path)

    cfg = getattr(robot, "cfg", None)
    cfg_prim_path = getattr(cfg, "prim_path", None)
    if cfg_prim_path:
        return _resolve_chassis_prim_path(cfg_prim_path.replace("env_.*", "env_0"))

    return _resolve_chassis_prim_path("/World/envs/env_0/Robot")


def resolve_robot_root_prim_path(env, robot_name: str = "robot") -> str:
    robot = env.scene[robot_name]

    prim_path = getattr(robot, "prim_path", None)
    if prim_path and "*" not in prim_path:
        return prim_path

    cfg = getattr(robot, "cfg", None)
    cfg_prim_path = getattr(cfg, "prim_path", None)
    if cfg_prim_path:
        return cfg_prim_path.replace("env_.*", "env_0")

    return "/World/envs/env_0/Robot"


def _graph_exists(og, graph_path: str) -> bool:
    try:
        graph = og.get_graph_by_path(graph_path)
    except Exception:
        return False
    return graph is not None


def create_g1_nav_tf_odom_graph(
    env,
    config: G1NavTfOdomBridgeConfig | None = None,
) -> dict[str, str]:
    """Create the ROS2 TF/odometry OmniGraph for the current G1 env."""
    import omni.graph.core as og
    import usdrt.Sdf

    config = config or G1NavTfOdomBridgeConfig()
    _enable_ros2_bridge_extension()

    robot_prim_path = config.robot_prim_path or resolve_robot_prim_path(
        env, config.robot_name
    )
    if _graph_exists(og, config.graph_path):
        return {
            "graph_path": config.graph_path,
            "robot_prim_path": robot_prim_path,
            "odom_topic": config.odom_topic,
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
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("TFMapOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("TFOdomBase", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                (
                    "ComputeOdometry.inputs:chassisPrim",
                    [usdrt.Sdf.Path(robot_prim_path)],
                ),
                ("PublishOdometry.inputs:topicName", config.odom_topic),
                ("PublishOdometry.inputs:odomFrameId", config.odom_frame),
                ("PublishOdometry.inputs:chassisFrameId", config.base_frame),
                ("PublishOdometry.inputs:publishRawVelocities", False),
                ("TFMapOdom.inputs:topicName", config.tf_topic),
                ("TFMapOdom.inputs:parentFrameId", config.map_frame),
                ("TFMapOdom.inputs:childFrameId", config.odom_frame),
                ("TFMapOdom.inputs:translation", [0.0, 0.0, 0.0]),
                ("TFMapOdom.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
                ("TFOdomBase.inputs:topicName", config.tf_topic),
                ("TFOdomBase.inputs:parentFrameId", config.odom_frame),
                ("TFOdomBase.inputs:childFrameId", config.base_frame),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "TFMapOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                ("Context.outputs:context", "TFMapOdom.inputs:context"),
                ("Context.outputs:context", "TFOdomBase.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "TFMapOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "TFOdomBase.inputs:timeStamp"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdometry.inputs:orientation",
                ),
                (
                    "ComputeOdometry.outputs:linearVelocity",
                    "PublishOdometry.inputs:linearVelocity",
                ),
                (
                    "ComputeOdometry.outputs:angularVelocity",
                    "PublishOdometry.inputs:angularVelocity",
                ),
                ("ComputeOdometry.outputs:execOut", "TFOdomBase.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "TFOdomBase.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "TFOdomBase.inputs:rotation"),
            ],
        },
    )

    return {
        "graph_path": config.graph_path,
        "robot_prim_path": robot_prim_path,
        "odom_topic": config.odom_topic,
        "tf_topic": config.tf_topic,
    }
