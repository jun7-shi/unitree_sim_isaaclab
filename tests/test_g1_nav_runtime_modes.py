from types import SimpleNamespace
import unittest


class G1NavRuntimeModeTests(unittest.TestCase):
    def test_gui_minimal_nav_skips_original_camera_observation_updates(self):
        from ros2_bridge.g1_nav_runtime_modes import (
            should_compute_action_observations,
            should_render_action_provider,
        )

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=False,
            nav_action_render_interval=None,
        )

        self.assertTrue(should_render_action_provider(args))
        self.assertFalse(should_compute_action_observations(args))

    def test_gui_minimal_nav_throttles_action_provider_rendering_by_default(self):
        from ros2_bridge.g1_nav_runtime_modes import action_provider_render_interval

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=False,
            nav_action_render_interval=None,
        )

        self.assertEqual(action_provider_render_interval(args), 4)

    def test_explicit_nav_action_render_interval_overrides_default(self):
        from ros2_bridge.g1_nav_runtime_modes import action_provider_render_interval

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=False,
            nav_action_render_interval=8,
        )

        self.assertEqual(action_provider_render_interval(args), 8)

    def test_pointcloud_nav_keeps_every_tick_rendering(self):
        from ros2_bridge.g1_nav_runtime_modes import action_provider_render_interval

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=True,
            no_render=True,
            nav_action_render_interval=8,
        )

        self.assertEqual(action_provider_render_interval(args), 1)

    def test_headless_no_render_minimal_nav_uses_app_update_for_ros_graphs(self):
        from ros2_bridge.g1_nav_runtime_modes import should_update_nav_ros_app

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=True,
            nav_action_render_interval=None,
        )

        self.assertTrue(should_update_nav_ros_app(args, nav_ros_bridge_enabled=True))

    def test_gui_minimal_nav_does_not_add_extra_app_update(self):
        from ros2_bridge.g1_nav_runtime_modes import should_update_nav_ros_app

        args = SimpleNamespace(
            nav_minimal_dds=True,
            enable_nav_ros_pointcloud=False,
            no_render=False,
            nav_action_render_interval=None,
        )

        self.assertFalse(should_update_nav_ros_app(args, nav_ros_bridge_enabled=True))


if __name__ == "__main__":
    unittest.main()
