# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Runtime mode decisions for G1 navigation simulation."""


def _enabled(args_cli, name):
    return bool(getattr(args_cli, name, False))


def should_render_action_provider(args_cli):
    """Keep GUI runs on the original render path; skip only for no-render minimal nav."""
    return (
        not _enabled(args_cli, "nav_minimal_dds")
        or _enabled(args_cli, "enable_nav_ros_pointcloud")
        or not _enabled(args_cli, "no_render")
    )


def should_compute_action_observations(args_cli):
    """Keep GUI minimal nav close to the original Unitree control loop."""
    return not _enabled(args_cli, "nav_minimal_dds") or not _enabled(
        args_cli, "no_render"
    )


def should_update_nav_ros_app(args_cli, nav_ros_bridge_enabled):
    """Use explicit Kit updates only in the headless no-render ROS-graph path."""
    return (
        _enabled(args_cli, "no_render")
        and _enabled(args_cli, "nav_minimal_dds")
        and bool(nav_ros_bridge_enabled)
        and not _enabled(args_cli, "enable_nav_ros_pointcloud")
    )
