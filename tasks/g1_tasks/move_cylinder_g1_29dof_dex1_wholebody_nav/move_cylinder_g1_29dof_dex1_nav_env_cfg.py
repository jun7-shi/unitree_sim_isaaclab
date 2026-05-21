# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""G1 wholebody navigation task using the package-owned head RGBD USD."""

from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass

from tasks.common_config import CameraPresets, G1RobotPresets
from tasks.g1_tasks.move_cylinder_g1_29dof_dex1_wholebody.move_cylinder_g1_29dof_dex1_hw_env_cfg import (
    MoveCylinderG129Dex1WholebodyEnvCfg,
    ObjectTableSceneCfg as BaseObjectTableSceneCfg,
)


@configclass
class ObjectTableNavSceneCfg(BaseObjectTableSceneCfg):
    """Scene variant that loads the G1 nav USD and RGBD head camera."""

    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex1_wholebody_nav(
        init_pos=(-3.9, -2.81811, 0.8),
        init_rot=(1, 0, 0, 0),
    )
    front_camera = CameraPresets.g1_nav_depth_camera()


@configclass
class MoveCylinderG129Dex1WholebodyNavEnvCfg(MoveCylinderG129Dex1WholebodyEnvCfg):
    """Navigation-ready G1 Dex1 wholebody task."""

    scene: ObjectTableNavSceneCfg = ObjectTableNavSceneCfg(
        num_envs=1,
        env_spacing=2.5,
        replicate_physics=True,
    )
