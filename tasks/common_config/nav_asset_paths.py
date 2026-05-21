# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Navigation-specific external asset paths."""

import os


DEFAULT_G1_NAV_USD = "/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/assets/g1_nav/g1_29dof_with_dex1_nav_depth.usd"

HUMANOID_NAVIGATION_G1_NAV_USD = "HUMANOID_NAVIGATION_G1_NAV_USD"


def get_g1_nav_usd_path() -> str:
    """Return the package-owned G1 navigation USD path."""
    return os.environ.get(HUMANOID_NAVIGATION_G1_NAV_USD, DEFAULT_G1_NAV_USD)
