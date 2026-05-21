from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class G1NavPointCloudBridgeStaticTests(unittest.TestCase):
    def test_bridge_helper_builds_depth_pointcloud_graph(self):
        bridge_path = ROOT / "ros2_bridge/g1_nav_pointcloud_bridge.py"
        self.assertTrue(bridge_path.exists())
        bridge = bridge_path.read_text(encoding="utf-8")

        self.assertIn("isaacsim.ros2.bridge", bridge)
        self.assertIn("ROS2CameraHelper", bridge)
        self.assertIn("IsaacCreateRenderProduct", bridge)
        self.assertIn('pointcloud_topic: str = "/g1/head_rgbd/points"', bridge)
        self.assertIn(
            'frame_id: str = "g1_head_d435_depth_optical_frame"',
            bridge,
        )
        self.assertIn('type", "depth_pcl"', bridge)
        self.assertIn("renderProductPath", bridge)
        self.assertIn('camera_prim_path: str | None = None', bridge)

    def test_sim_main_exposes_nav_pointcloud_bridge_flag(self):
        sim_main = (ROOT / "sim_main.py").read_text(encoding="utf-8")

        self.assertIn("--enable_nav_ros_pointcloud", sim_main)
        self.assertIn("create_g1_nav_pointcloud_graph", sim_main)
        self.assertIn("G1NavPointCloudBridgeConfig", sim_main)
        self.assertIn("nav_ros_pointcloud_topic", sim_main)
        self.assertIn("nav_ros_camera_frame", sim_main)

    def test_bridge_doc_contains_pointcloud_launch_and_verification(self):
        doc = (ROOT / "docs/humanoid_nav_ros_bridge.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("--enable_nav_ros_pointcloud", doc)
        self.assertIn("/g1/head_rgbd/points", doc)
        self.assertIn("sensor_msgs/PointCloud2", doc)
        self.assertIn("ros2 topic hz /g1/head_rgbd/points", doc)
        self.assertIn(
            "ros2 topic echo --once /g1/head_rgbd/points.header.frame_id",
            doc,
        )


if __name__ == "__main__":
    unittest.main()
