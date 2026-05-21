# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Isaac Sim ROS2 clock graph for humanoid navigation."""

from dataclasses import dataclass

from ros2_bridge.g1_nav_tf_odom_bridge import (
    _enable_ros2_bridge_extension,
    _graph_exists,
)


@dataclass(frozen=True)
class G1NavClockBridgeConfig:
    graph_path: str = "/ActionGraph/HumanoidNavClock"
    clock_topic: str = "/clock"


def create_g1_nav_clock_graph(
    config: G1NavClockBridgeConfig | None = None,
) -> dict[str, str]:
    """Create the ROS2 clock OmniGraph used by Nav2 use_sim_time nodes."""
    import omni.graph.core as og

    config = config or G1NavClockBridgeConfig()
    _enable_ros2_bridge_extension()

    if _graph_exists(og, config.graph_path):
        return {"graph_path": config.graph_path, "clock_topic": config.clock_topic}

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": config.graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishClock.inputs:topicName", config.clock_topic),
            ],
        },
    )

    return {"graph_path": config.graph_path, "clock_topic": config.clock_topic}


def ensure_nav_ros_timeline_playing() -> bool:
    """Start Isaac Sim playback so OnPlaybackTick-driven ROS graphs publish."""
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    is_playing = False
    try:
        is_playing = bool(timeline.is_playing())
    except Exception:
        pass

    if not is_playing:
        timeline.play()
        return True
    return False
