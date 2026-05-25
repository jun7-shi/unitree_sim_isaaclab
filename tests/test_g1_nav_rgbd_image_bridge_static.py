from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class G1NavRgbdImageBridgeStaticTests(unittest.TestCase):
    def test_bridge_helper_builds_rgb_and_depth_image_graph(self):
        bridge_path = ROOT / "ros2_bridge/g1_nav_rgbd_image_bridge.py"
        self.assertTrue(bridge_path.exists())
        bridge = bridge_path.read_text(encoding="utf-8")

        self.assertIn("isaacsim.ros2.bridge", bridge)
        self.assertIn("ROS2CameraHelper", bridge)
        self.assertIn("ROS2CameraInfoHelper", bridge)
        self.assertIn("IsaacCreateRenderProduct", bridge)
        self.assertIn("OgnIsaacRunOneSimulationFrame", bridge)
        self.assertIn("omni.graph.action.OnPlaybackTick", bridge)
        self.assertIn('rgb_topic: str = "/g1/head_rgbd/rgb/image_raw"', bridge)
        self.assertIn('depth_topic: str = "/g1/head_rgbd/depth/image_raw"', bridge)
        self.assertIn('camera_info_topic: str = "/g1/head_rgbd/camera_info"', bridge)
        self.assertIn('frame_id: str = "g1_head_d435_depth_optical_frame"', bridge)
        self.assertIn('inputs:type", "rgb"', bridge)
        self.assertIn('inputs:type", "depth"', bridge)
        self.assertNotIn("depth_pcl", bridge)

    def test_sim_main_exposes_nav_rgbd_image_bridge_flag(self):
        sim_main = (ROOT / "sim_main.py").read_text(encoding="utf-8")

        self.assertIn("--enable_nav_ros_rgbd_images", sim_main)
        self.assertIn("create_g1_nav_rgbd_image_graph", sim_main)
        self.assertIn("G1NavRgbdImageBridgeConfig", sim_main)
        self.assertIn("nav_ros_rgb_topic", sim_main)
        self.assertIn("nav_ros_depth_topic", sim_main)
        self.assertIn("enable_nav_ros_rgbd_images", sim_main)

    def test_bridge_doc_uses_image_topics_for_v15(self):
        doc = (ROOT / "docs/humanoid_nav_ros_bridge.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("--enable_nav_ros_rgbd_images", doc)
        self.assertIn("/g1/head_rgbd/rgb/image_raw", doc)
        self.assertIn("/g1/head_rgbd/depth/image_raw", doc)
        self.assertIn("sensor_msgs/Image", doc)
        self.assertIn("ros2 topic hz /g1/head_rgbd/depth/image_raw", doc)
        self.assertNotIn("For V1.5 RGBD perception, add `--enable_nav_ros_pointcloud`", doc)


if __name__ == "__main__":
    unittest.main()
