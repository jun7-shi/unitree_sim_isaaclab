import unittest


class ProcessCleanupTests(unittest.TestCase):
    def test_collects_only_matching_descendants(self):
        from ros2_bridge.process_cleanup import collect_matching_descendants

        processes = [
            (10, 1, "conda run --no-capture-output python sim_main.py"),
            (20, 10, "python sim_main.py --task Isaac-Kitchen-G129-Dex1-Wholebody"),
            (30, 20, "python sim_main.py image server helper"),
            (40, 1, "python sim_main.py unrelated user process"),
            (50, 20, "python unrelated_worker.py"),
        ]

        self.assertEqual(
            collect_matching_descendants(
                processes,
                current_pid=20,
                command_substring="sim_main.py",
            ),
            [30],
        )


if __name__ == "__main__":
    unittest.main()
