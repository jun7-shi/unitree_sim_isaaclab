from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class G1NavSelfFilterTfBridgeStaticTests(unittest.TestCase):
    def test_bridge_helper_builds_selected_link_tf_graph(self):
        bridge_path = ROOT / "ros2_bridge/g1_nav_self_filter_tf_bridge.py"
        self.assertTrue(bridge_path.exists())
        bridge = bridge_path.read_text(encoding="utf-8")

        self.assertIn("ROS2PublishTransformTree", bridge)
        self.assertIn("IsaacReadSimulationTime", bridge)
        self.assertIn("omni.graph.action.OnPlaybackTick", bridge)
        self.assertIn("targetPrims", bridge)
        self.assertIn("parentPrim", bridge)
        self.assertIn("left_wrist_yaw_link", bridge)
        self.assertIn("right_wrist_yaw_link", bridge)

    def test_sim_main_exposes_self_filter_tf_bridge_flag(self):
        sim_main = (ROOT / "sim_main.py").read_text(encoding="utf-8")

        self.assertIn("--enable_nav_ros_self_filter_tf", sim_main)
        self.assertIn("G1NavSelfFilterTfBridgeConfig", sim_main)
        self.assertIn("create_g1_nav_self_filter_tf_graph", sim_main)
        self.assertIn("nav_ros_self_filter_tf_links", sim_main)
        self.assertIn("enable_nav_ros_self_filter_tf", sim_main)

    def test_bridge_doc_lists_self_filter_tf_verification(self):
        doc = (ROOT / "docs/humanoid_nav_ros_bridge.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("--enable_nav_ros_self_filter_tf", doc)
        self.assertIn("ros2 run tf2_ros tf2_echo pelvis left_wrist_yaw_link", doc)


if __name__ == "__main__":
    unittest.main()
