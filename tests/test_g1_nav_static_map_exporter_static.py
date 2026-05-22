from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class G1NavStaticMapExporterStaticTests(unittest.TestCase):
    def test_sim_main_exposes_static_map_export_cli(self):
        sim_main = (ROOT / "sim_main.py").read_text(encoding="utf-8")

        self.assertIn("--export_nav_static_map", sim_main)
        self.assertIn("--nav_static_map_cell_size", sim_main)
        self.assertIn("--nav_static_map_origin", sim_main)
        self.assertIn("--nav_static_map_z_bounds", sim_main)
        self.assertIn("--nav_static_map_bound_prim", sim_main)
        self.assertIn("--nav_static_map_exclude_prims", sim_main)
        self.assertIn("--nav_static_map_no_mesh_collision", sim_main)
        self.assertIn('env.scene["robot"].data.root_pos_w', sim_main)
        self.assertIn("export_g1_nav_static_map", sim_main)

    def test_exporter_uses_official_isaac_sim_omap_api(self):
        exporter = (
            ROOT / "ros2_bridge" / "g1_nav_static_map_exporter.py"
        ).read_text(encoding="utf-8")

        self.assertIn("isaacsim.asset.gen.omap", exporter)
        self.assertIn("_omap.Generator", exporter)
        self.assertIn("generator.update_settings", exporter)
        self.assertIn("generator.set_transform", exporter)
        self.assertIn("generator.generate2d", exporter)
        self.assertIn("generator.get_buffer", exporter)
        self.assertIn("_buffer_histogram", exporter)
        self.assertLess(
            exporter.index("_enable_omap_extension()"),
            exporter.index("from isaacsim.asset.gen.omap.bindings import _omap"),
        )

    def test_exporter_can_apply_collision_to_visual_meshes_for_mapping(self):
        exporter = (
            ROOT / "ros2_bridge" / "g1_nav_static_map_exporter.py"
        ).read_text(encoding="utf-8")

        self.assertIn("apply_collision_to_meshes", exporter)
        self.assertIn("_apply_collision_to_meshes", exporter)
        self.assertIn("UsdPhysics.CollisionAPI.Apply", exporter)
        self.assertIn("UsdGeom.Mesh", exporter)

    def test_exporter_temporarily_excludes_robot_and_object(self):
        exporter = (
            ROOT / "ros2_bridge" / "g1_nav_static_map_exporter.py"
        ).read_text(encoding="utf-8")

        self.assertIn("/World/envs/env_0/Robot", exporter)
        self.assertIn("/World/envs/env_0/Object", exporter)
        self.assertIn("_set_prims_active", exporter)
        self.assertIn("_restore_prims_active", exporter)

    def test_pgm_writer_outputs_ros_compatible_binary_pgm(self):
        from ros2_bridge.g1_nav_static_map_exporter import _write_pgm

        with tempfile.TemporaryDirectory() as temp_dir:
            pgm_path = Path(temp_dir) / "map.pgm"
            _write_pgm(
                pgm_path,
                [100, 0, 50, 123],
                width=2,
                height=2,
                occupied_value=100,
                free_value=0,
                unknown_value=50,
            )

            data = pgm_path.read_bytes()

        self.assertTrue(data.startswith(b"P5\n2 2\n255\n"))
        self.assertEqual(data[-4:], bytes([205, 123, 0, 254]))


if __name__ == "__main__":
    unittest.main()
