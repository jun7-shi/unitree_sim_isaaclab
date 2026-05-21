# HUM-40 G1 TF Odom Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish Nav2-facing `map -> odom -> base_link` TF and `/odom` from the Unitree G1 Isaac Sim navigation task.

**Architecture:** Add a small Unitree-side helper that creates an Isaac Sim OmniGraph using the built-in `isaacsim.ros2.bridge` nodes. Keep `sim_main.py` opt-in via `--enable_nav_ros_tf_odom` so existing tasks are unchanged, and document the command used by HUM-36.

**Tech Stack:** Isaac Sim 5 ROS2 Bridge OmniGraph nodes, Isaac Lab env scene access, Python static tests with stdlib `unittest`.

---

### Task 1: Static Test Contract

**Files:**
- Create: `tests/test_g1_nav_ros_bridge_static.py`

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class G1NavRosBridgeStaticTests(unittest.TestCase):
    def test_bridge_helper_builds_tf_odom_graph(self):
        bridge = (ROOT / "ros2_bridge/g1_nav_tf_odom_bridge.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("isaacsim.ros2.bridge", bridge)
        self.assertIn("ROS2PublishOdometry", bridge)
        self.assertIn("ROS2PublishRawTransformTree", bridge)
        self.assertIn("IsaacComputeOdometry", bridge)
        self.assertIn("map_frame: str = \"map\"", bridge)
        self.assertIn("odom_frame: str = \"odom\"", bridge)
        self.assertIn("base_frame: str = \"base_link\"", bridge)
        self.assertIn("odom_topic: str = \"/odom\"", bridge)
        self.assertIn("usdrt.Sdf.Path(robot_prim_path)", bridge)

    def test_sim_main_exposes_nav_tf_odom_bridge_flag(self):
        sim_main = (ROOT / "sim_main.py").read_text(encoding="utf-8")

        self.assertIn("--enable_nav_ros_tf_odom", sim_main)
        self.assertIn("create_g1_nav_tf_odom_graph", sim_main)
        self.assertIn("G1NavTfOdomBridgeConfig", sim_main)
        self.assertIn("nav_ros_map_frame", sim_main)
        self.assertIn("nav_ros_odom_topic", sim_main)

    def test_bridge_doc_contains_launch_and_verification_commands(self):
        doc = (ROOT / "docs/humanoid_nav_ros_bridge.md").read_text(encoding="utf-8")

        self.assertIn("--enable_nav_ros_tf_odom", doc)
        self.assertIn("Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav", doc)
        self.assertIn("ros2 run tf2_ros tf2_echo map base_link", doc)
        self.assertIn("ros2 topic hz /odom", doc)
        self.assertIn("map -> odom -> base_link", doc)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.test_g1_nav_ros_bridge_static -v`

Expected: FAIL because `ros2_bridge/g1_nav_tf_odom_bridge.py` and `docs/humanoid_nav_ros_bridge.md` do not exist and `sim_main.py` has no nav bridge flag.

### Task 2: Bridge Helper

**Files:**
- Create: `ros2_bridge/__init__.py`
- Create: `ros2_bridge/g1_nav_tf_odom_bridge.py`

- [ ] **Step 1: Implement the helper**

Create `G1NavTfOdomBridgeConfig`, `resolve_robot_prim_path(env, robot_name)`, and `create_g1_nav_tf_odom_graph(env, config)`.

The graph must create:
- `OnPlaybackTick`
- `ROS2Context`
- `IsaacReadSimulationTime`
- `IsaacComputeOdometry`
- `ROS2PublishOdometry`
- `ROS2PublishRawTransformTree` for `map -> odom`
- `ROS2PublishRawTransformTree` for `odom -> base_link`

The helper must enable `isaacsim.ros2.bridge` and connect simulation time, context, odometry pose, linear velocity, and angular velocity exactly once.

- [ ] **Step 2: Run static and syntax tests**

Run:

```bash
python3 -m unittest tests.test_g1_nav_ros_bridge_static -v
python3 -m py_compile ros2_bridge/__init__.py ros2_bridge/g1_nav_tf_odom_bridge.py
```

Expected: tests pass and py_compile exits 0 without launching Isaac Sim.

### Task 3: sim_main Wiring and Docs

**Files:**
- Modify: `sim_main.py`
- Create: `docs/humanoid_nav_ros_bridge.md`

- [ ] **Step 1: Wire CLI arguments**

Add `--enable_nav_ros_tf_odom`, `--nav_ros_graph_path`, `--nav_ros_robot_prim_path`, `--nav_ros_map_frame`, `--nav_ros_odom_frame`, `--nav_ros_base_frame`, `--nav_ros_odom_topic`, and `--nav_ros_tf_topic`.

- [ ] **Step 2: Create the graph after `env.sim.reset()` and `env.reset()`**

When `args_cli.enable_nav_ros_tf_odom` is true, call:

```python
create_g1_nav_tf_odom_graph(
    env,
    G1NavTfOdomBridgeConfig(
        graph_path=args_cli.nav_ros_graph_path,
        robot_prim_path=args_cli.nav_ros_robot_prim_path,
        map_frame=args_cli.nav_ros_map_frame,
        odom_frame=args_cli.nav_ros_odom_frame,
        base_frame=args_cli.nav_ros_base_frame,
        odom_topic=args_cli.nav_ros_odom_topic,
        tf_topic=args_cli.nav_ros_tf_topic,
    ),
)
```

- [ ] **Step 3: Document command and checks**

Create `docs/humanoid_nav_ros_bridge.md` with the Unitree launch command and these checks:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /odom
ros2 topic echo --once /odom.header.frame_id
ros2 topic echo --once /odom.child_frame_id
```

### Task 4: Verify and Commit

**Files:**
- All files above

- [ ] **Step 1: Run verification**

Run:

```bash
python3 -m unittest tests.test_g1_nav_task_static tests.test_g1_nav_ros_bridge_static -v
python3 -m py_compile sim_main.py ros2_bridge/__init__.py ros2_bridge/g1_nav_tf_odom_bridge.py
git diff --check
```

Expected: all tests pass, py_compile exits 0, and diff check exits 0.

- [ ] **Step 2: Commit**

Run:

```bash
git add sim_main.py ros2_bridge docs/humanoid_nav_ros_bridge.md docs/superpowers/plans/2026-05-22-hum-40-g1-tf-odom-bridge.md tests/test_g1_nav_ros_bridge_static.py
git commit -m "feat: add G1 nav TF odom ROS bridge"
```
