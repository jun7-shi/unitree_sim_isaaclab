# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Runtime mode decisions for G1 navigation simulation."""


def _enabled(args_cli, name):
    return bool(getattr(args_cli, name, False))


def should_render_action_provider(args_cli):
    """Return whether the action provider should manually render sim frames."""
    return action_provider_render_interval(args_cli) > 0


def action_provider_render_interval(args_cli):
    """Return how often the action provider should call env.sim.render().

    The Wholebody action provider owns the manual render call. GUI runs default
    to every provider tick so the viewport FPS reflects actual loop throughput.
    Camera bridge modes also need every render tick for render-product updates.
    """
    if _enabled(args_cli, "enable_nav_ros_pointcloud") or _enabled(
        args_cli,
        "enable_nav_ros_rgbd_images",
    ):
        return 1
    if _enabled(args_cli, "no_render"):
        return 0

    configured_interval = getattr(args_cli, "nav_action_render_interval", None)
    if configured_interval is not None:
        return max(1, int(configured_interval))

    return 1


def should_compute_action_observations(args_cli):
    """Skip original camera observation updates in navigation-only DDS mode."""
    return not _enabled(args_cli, "nav_minimal_dds")


def should_update_nav_ros_app(args_cli, nav_ros_bridge_enabled):
    """Use explicit Kit updates only in the headless no-render ROS-graph path."""
    return (
        _enabled(args_cli, "no_render")
        and _enabled(args_cli, "nav_minimal_dds")
        and bool(nav_ros_bridge_enabled)
        and not _enabled(args_cli, "enable_nav_ros_pointcloud")
        and not _enabled(args_cli, "enable_nav_ros_rgbd_images")
    )
