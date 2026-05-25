from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HUMANOID_NAV_USD = (
    "/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/"
    "nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/"
    "assets/g1_nav/g1_29dof_with_dex1_nav_depth.usd"
)


class G1NavTaskStaticTests(unittest.TestCase):
    def test_nav_asset_helper_points_to_package_owned_usd(self):
        helper = (ROOT / "tasks/common_config/nav_asset_paths.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("HUMANOID_NAVIGATION_G1_NAV_USD", helper)
        self.assertIn(HUMANOID_NAV_USD, helper)
        self.assertIn("def get_g1_nav_usd_path", helper)

    def test_unitree_robot_cfg_defines_nav_wholebody_asset(self):
        unitree = (ROOT / "robots/unitree.py").read_text(encoding="utf-8")

        self.assertIn("get_g1_nav_usd_path", unitree)
        self.assertIn("G129_CFG_WITH_DEX1_WHOLEBODY_NAV", unitree)
        self.assertIn("usd_path=get_g1_nav_usd_path()", unitree)

    def test_robot_presets_expose_nav_wholebody_variant(self):
        robot_configs = (ROOT / "tasks/common_config/robot_configs.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("G129_CFG_WITH_DEX1_WHOLEBODY_NAV", robot_configs)
        self.assertIn("def g1_29dof_dex1_wholebody_nav", robot_configs)
        self.assertIn("base_config=G129_CFG_WITH_DEX1_WHOLEBODY_NAV", robot_configs)

    def test_camera_presets_expose_nav_rgbd_camera(self):
        camera_configs = (ROOT / "tasks/common_config/camera_configs.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("def g1_nav_depth_camera", camera_configs)
        self.assertIn(
            "/World/envs/env_.*/Robot/d435_link/head_d435_depth_camera",
            camera_configs,
        )
        self.assertIn('data_types=["rgb", "depth"]', camera_configs)
        self.assertIn("spawn_camera=False", camera_configs)

    def test_nav_task_is_registered_and_imported(self):
        task_init = (ROOT / "tasks/g1_tasks/__init__.py").read_text(encoding="utf-8")
        nav_init = (
            ROOT
            / "tasks/g1_tasks/move_cylinder_g1_29dof_dex1_wholebody_nav/__init__.py"
        ).read_text(encoding="utf-8")
        nav_cfg = (
            ROOT
            / "tasks/g1_tasks/move_cylinder_g1_29dof_dex1_wholebody_nav/"
            "move_cylinder_g1_29dof_dex1_nav_env_cfg.py"
        ).read_text(encoding="utf-8")

        self.assertIn("move_cylinder_g1_29dof_dex1_wholebody_nav", task_init)
        self.assertIn("Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav", nav_init)
        self.assertIn("MoveCylinderG129Dex1WholebodyNavEnvCfg", nav_init)
        self.assertIn("g1_29dof_dex1_wholebody_nav", nav_cfg)

    def test_kitchen_wholebody_task_is_available_for_navigation_acceptance(self):
        kitchen_init = (
            ROOT / "tasks/g1_tasks/kitchen_g1_29dof_dex1_wholebody/__init__.py"
        ).read_text(encoding="utf-8")
        kitchen_cfg = (
            ROOT
            / "tasks/g1_tasks/kitchen_g1_29dof_dex1_wholebody/"
            "kitchen_g1_29dof_dex1_hw_env_cfg.py"
        ).read_text(encoding="utf-8")

        self.assertIn("Isaac-Kitchen-G129-Dex1-Wholebody", kitchen_init)
        self.assertIn("KitchenG129Dex1WholebodyEnvCfg", kitchen_init)
        self.assertIn("KITCHEN_USD_PATH", kitchen_cfg)
        self.assertIn("/World/envs/env_.*/Kitchen", kitchen_cfg)
        self.assertIn("g1_29dof_dex1_wholebody", kitchen_cfg)

    def test_kitchen_nav_task_uses_nav_usd_and_rgbd_camera(self):
        task_init = (ROOT / "tasks/g1_tasks/__init__.py").read_text(encoding="utf-8")
        kitchen_init = (
            ROOT / "tasks/g1_tasks/kitchen_g1_29dof_dex1_wholebody/__init__.py"
        ).read_text(encoding="utf-8")
        kitchen_cfg = (
            ROOT
            / "tasks/g1_tasks/kitchen_g1_29dof_dex1_wholebody/"
            "kitchen_g1_29dof_dex1_hw_env_cfg.py"
        ).read_text(encoding="utf-8")

        self.assertIn("kitchen_g1_29dof_dex1_wholebody", task_init)
        self.assertIn("Isaac-Kitchen-G129-Dex1-Wholebody-Nav", kitchen_init)
        self.assertIn("KitchenG129Dex1WholebodyNavEnvCfg", kitchen_init)
        self.assertIn("KitchenNavSceneCfg", kitchen_cfg)
        self.assertIn("g1_29dof_dex1_wholebody_nav", kitchen_cfg)
        self.assertIn("g1_nav_depth_camera", kitchen_cfg)


if __name__ == "__main__":
    unittest.main()
