from types import SimpleNamespace
import unittest


class G1NavRuntimeModeTests(unittest.TestCase):
    def test_gui_minimal_nav_keeps_original_render_and_observation_updates(self):
        from ros2_bridge.g1_nav_runtime_modes import (
            should_compute_action_observations,
            should_render_action_provider,
        )

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=False,
        )

        self.assertTrue(should_render_action_provider(args))
        self.assertTrue(should_compute_action_observations(args))

    def test_headless_no_render_minimal_nav_uses_app_update_for_ros_graphs(self):
        from ros2_bridge.g1_nav_runtime_modes import should_update_nav_ros_app

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=True,
        )

        self.assertTrue(should_update_nav_ros_app(args, nav_ros_bridge_enabled=True))

    def test_gui_minimal_nav_does_not_add_extra_app_update(self):
        from ros2_bridge.g1_nav_runtime_modes import should_update_nav_ros_app

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=False,
        )

        self.assertFalse(should_update_nav_ros_app(args, nav_ros_bridge_enabled=True))


if __name__ == "__main__":
    unittest.main()
