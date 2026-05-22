from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class G1NavRosBridgeStaticTests(unittest.TestCase):
    def test_resolves_g1_pelvis_as_default_chassis_prim(self):
        from ros2_bridge.g1_nav_tf_odom_bridge import resolve_robot_prim_path

        class FakeRobot:
            prim_path = "/World/envs/env_0/Robot"

        class FakeScene:
            def __getitem__(self, name):
                if name != "robot":
                    raise KeyError(name)
                return FakeRobot()

        class FakeEnv:
            scene = FakeScene()

        self.assertEqual(
            resolve_robot_prim_path(FakeEnv()),
            "/World/envs/env_0/Robot/pelvis",
        )

    def test_bridge_helper_builds_tf_odom_graph(self):
        bridge = (ROOT / "ros2_bridge/g1_nav_tf_odom_bridge.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("isaacsim.ros2.bridge", bridge)
        self.assertIn("ROS2PublishOdometry", bridge)
        self.assertIn("ROS2PublishRawTransformTree", bridge)
        self.assertIn("IsaacComputeOdometry", bridge)
        self.assertIn("omni.graph.action.OnPlaybackTick", bridge)
        self.assertNotIn("isaacsim.core.nodes.OnPhysicsStep", bridge)
        self.assertIn('map_frame: str = "map"', bridge)
        self.assertIn('odom_frame: str = "odom"', bridge)
        self.assertIn('base_frame: str = "base_link"', bridge)
        self.assertIn('odom_topic: str = "/odom"', bridge)
        self.assertIn("usdrt.Sdf.Path(robot_prim_path)", bridge)

    def test_clock_helper_matches_nvidia_clock_shortcut(self):
        bridge_path = ROOT / "ros2_bridge/g1_nav_clock_bridge.py"
        self.assertTrue(bridge_path.exists())
        bridge = bridge_path.read_text(encoding="utf-8")

        self.assertIn("omni.graph.action.OnPlaybackTick", bridge)
        self.assertIn("isaacsim.core.nodes.IsaacReadSimulationTime", bridge)
        self.assertIn("isaacsim.ros2.bridge.ROS2PublishClock", bridge)
        self.assertIn("isaacsim.ros2.bridge.ROS2Context", bridge)
        self.assertIn('clock_topic: str = "/clock"', bridge)

    def test_sim_main_exposes_nav_tf_odom_bridge_flag(self):
        sim_main = (ROOT / "sim_main.py").read_text(encoding="utf-8")

        self.assertIn("--enable_nav_ros_clock", sim_main)
        self.assertIn("--enable_nav_ros_tf_odom", sim_main)
        self.assertIn("create_g1_nav_clock_graph", sim_main)
        self.assertIn("create_g1_nav_tf_odom_graph", sim_main)
        self.assertIn("ensure_nav_ros_timeline_playing", sim_main)
        self.assertIn("G1NavTfOdomBridgeConfig", sim_main)
        self.assertIn("nav_ros_map_frame", sim_main)
        self.assertIn("nav_ros_odom_topic", sim_main)

    def test_bridge_doc_contains_launch_and_verification_commands(self):
        doc = (ROOT / "docs/humanoid_nav_ros_bridge.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("--enable_nav_ros_tf_odom", doc)
        self.assertIn("--enable_nav_ros_clock", doc)
        self.assertIn("/clock", doc)
        self.assertIn("Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav", doc)
        self.assertIn("ros2 run tf2_ros tf2_echo map base_link", doc)
        self.assertIn("ros2 topic hz /odom", doc)
        self.assertIn("map -> odom -> base_link", doc)


if __name__ == "__main__":
    unittest.main()
