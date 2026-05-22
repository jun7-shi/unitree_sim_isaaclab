
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
# main.py
import os

project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
import time
import sys
import signal
import torch
import gymnasium as gym
from pathlib import Path

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

from teleimager.image_server import run_isaacsim_server
from dds.dds_create import create_dds_objects,create_dds_objects_replay
from ros2_bridge.g1_nav_static_map_exporter import (
    DEFAULT_NAV_STATIC_MAP_BOUND_PRIM,
    DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS,
    resolve_nav_static_map_scope,
)
from ros2_bridge.g1_nav_cmd_udp_bridge import (
    G1NavUdpCmdBridge,
    G1NavUdpCmdBridgeConfig,
)
# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds", 
                   choices=["dds", "file", "trajectory", "policy", "replay","dds_wholebody"], 
                   help="Action source")


parser.add_argument("--robot_type", type=str, default="g129", help="robot type")
parser.add_argument("--enable_dex1_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--enable_inspire_dds", action="store_true", help="enable inspire hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

parser.add_argument("--file_path", type=str, default="/home/unitree/Code/xr_teleoperate/teleop/utils/data", help="file path (when action_source=file)")
parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", action="store_true", default=False, help="generate data")
parser.add_argument("--rerun_log", action="store_true", default=False, help="rerun log")
parser.add_argument("--replay_data",  action="store_true", default=False, help="replay data")

parser.add_argument("--modify_light",  action="store_true", default=False, help="modify light")
parser.add_argument("--modify_camera",  action="store_true", default=False,    help="modify camera")

# performance analysis parameters
parser.add_argument("--step_hz", type=int, default=100, help="control frequency")
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")

parser.add_argument("--model_path", type=str, default="assets/model/policy.onnx", help="model path")
parser.add_argument("--reward_interval", type=int, default=10, help="step interval for reward calculation")
parser.add_argument("--enable_wholebody_dds", action="store_true", default=False, help="enable wh dds")
parser.add_argument("--enable_nav_ros_clock", action="store_true", default=False, help="enable ROS2 /clock bridge for humanoid navigation")
parser.add_argument("--nav_ros_clock_graph_path", type=str, default="/ActionGraph/HumanoidNavClock", help="OmniGraph path for the navigation ROS2 clock bridge")
parser.add_argument("--nav_ros_clock_topic", type=str, default="/clock", help="ROS2 clock topic for Nav2 use_sim_time nodes")
parser.add_argument("--enable_nav_ros_tf_odom", action="store_true", default=False, help="enable ROS2 TF and odometry bridge for humanoid navigation")
parser.add_argument("--nav_ros_graph_path", type=str, default="/ActionGraph/HumanoidNavTfOdom", help="OmniGraph path for the navigation ROS2 TF/odom bridge")
parser.add_argument("--nav_ros_robot_prim_path", type=str, default=None, help="explicit robot prim path for the navigation ROS2 TF/odom bridge")
parser.add_argument("--nav_ros_map_frame", type=str, default="map", help="Nav2 map frame id")
parser.add_argument("--nav_ros_odom_frame", type=str, default="odom", help="Nav2 odom frame id")
parser.add_argument("--nav_ros_base_frame", type=str, default="base_link", help="Nav2 base frame id")
parser.add_argument("--nav_ros_odom_topic", type=str, default="/odom", help="ROS2 odometry topic for Nav2")
parser.add_argument("--nav_ros_tf_topic", type=str, default="tf", help="ROS2 TF topic for Nav2")
parser.add_argument(
    "--nav_ros_map_odom_translation",
    type=float,
    nargs=3,
    default=None,
    metavar=("X", "Y", "Z"),
    help="override the map->odom translation; defaults to the robot chassis world position at bridge creation",
)
parser.add_argument("--enable_nav_ros_pointcloud", action="store_true", default=False, help="enable ROS2 PointCloud2 bridge for the G1 head RGBD camera")
parser.add_argument("--nav_ros_pointcloud_graph_path", type=str, default="/ActionGraph/HumanoidNavPointCloud", help="OmniGraph path for the navigation ROS2 PointCloud2 bridge")
parser.add_argument("--nav_ros_camera_prim_path", type=str, default=None, help="explicit camera prim path for the navigation ROS2 PointCloud2 bridge")
parser.add_argument("--nav_ros_pointcloud_topic", type=str, default="/g1/head_rgbd/points", help="ROS2 PointCloud2 topic for the G1 head RGBD camera")
parser.add_argument("--nav_ros_camera_info_topic", type=str, default="/g1/head_rgbd/camera_info", help="ROS2 CameraInfo topic for the G1 head RGBD camera")
parser.add_argument("--nav_ros_camera_frame", type=str, default="g1_head_d435_depth_optical_frame", help="ROS2 frame id for the G1 head RGBD PointCloud2")
parser.add_argument("--nav_ros_node_namespace", type=str, default="", help="ROS2 node namespace for humanoid navigation camera publishers")
parser.add_argument("--nav_ros_camera_width", type=int, default=640, help="render product width for the G1 head RGBD PointCloud2")
parser.add_argument("--nav_ros_camera_height", type=int, default=480, help="render product height for the G1 head RGBD PointCloud2")
parser.add_argument("--enable_nav_udp_cmd_bridge", action="store_true", default=False, help="receive Nav2 velocity commands over UDP and write Unitree run commands")
parser.add_argument("--nav_udp_cmd_host", type=str, default="127.0.0.1", help="UDP host/interface for the Nav2 command bridge")
parser.add_argument("--nav_udp_cmd_port", type=int, default=18080, help="UDP port for the Nav2 command bridge")
parser.add_argument("--nav_udp_cmd_stale_timeout", type=float, default=0.5, help="seconds before the Nav2 UDP command bridge writes a zero command")
parser.add_argument("--export_nav_static_map", type=str, default="", help="export a Nav2 static occupancy map YAML/PGM from the current Unitree IsaacLab env and exit")
parser.add_argument("--nav_static_map_cell_size", type=float, default=0.05, help="cell size in meters for --export_nav_static_map")
parser.add_argument("--nav_static_map_origin", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="free start point for Isaac Sim occupancy map generation; defaults to the robot start x/y with z=0.1")
parser.add_argument("--nav_static_map_z_bounds", type=float, nargs=2, default=(0.05, 1.2), metavar=("MIN_Z", "MAX_Z"), help="height slice for static occupancy map generation")
parser.add_argument("--nav_static_map_bound_prim", type=str, default=DEFAULT_NAV_STATIC_MAP_BOUND_PRIM, help="USD prim whose world bounds define the XY occupancy map extent")
parser.add_argument("--nav_static_map_padding", type=float, default=0.25, help="extra XY padding around --nav_static_map_bound_prim")
parser.add_argument("--nav_static_map_exclude_prims", type=str, nargs="*", default=list(DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS), help="prim paths to temporarily deactivate while exporting the static map")
parser.add_argument("--nav_static_map_no_mesh_collision", action="store_true", default=False, help="do not temporarily apply CollisionAPI to static meshes before occupancy map export")

parser.add_argument("--physics_dt", type=float, default=None, help="physics time step, e.g., 0.005")
parser.add_argument("--render_interval", type=int, default=None, help="render interval steps (>=1)")
parser.add_argument("--camera_write_interval", type=int, default=None, help="camera write interval steps (>=1)")


parser.add_argument("--no_render",action="store_true",default=False,help="disable rendering updates entirely (overrides render interval)",)
parser.add_argument("--public_ip",type=str,default="127.0.0.1",help="public ip")
parser.add_argument("--livestream_type", type=int, default=2, help="livestream type (0: no livestream, 1: WebRTC public network, 2:  WebRTC private network)")

parser.add_argument("--solver_iterations", type=int, default=None, help="physx solver iteration count (e.g., 4)")
parser.add_argument("--gravity_z", type=float, default=None, help="override gravity z (e.g., -9.8)")
parser.add_argument("--skip_cvtcolor", action="store_true", default=False, help="skip cv2.cvtColor if upstream already BGR")

parser.add_argument("--camera_jpeg", action="store_true", default=True, help="enable JPEG compression for camera frames")
parser.add_argument("--camera_jpeg_quality", type=int, default=85, help="JPEG quality (1-100)")

parser.add_argument("--physx_substeps", type=int, default=None, help="physx substeps per step")
parser.add_argument("--camera_include", type=str, default="front_camera,left_wrist_camera,right_wrist_camera", help="comma-separated camera names to enable")
parser.add_argument("--camera_exclude", type=str, default="world_camera", help="comma-separated camera names to disable")

parser.add_argument("--env_reward_interval", type=int, default=5, help="environment reward compute interval (steps)")
parser.add_argument("--seed", type=int, default=42, help="environment seed")
# add AppLauncher parameters
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.no_render:
    os.environ["LIVESTREAM"] = str(args_cli.livestream_type)
    os.environ["PUBLIC_IP"] = args_cli.public_ip
else:
    os.environ["LIVESTREAM"] = "0"

if args_cli.enable_dex3_dds and args_cli.enable_dex1_dds and args_cli.enable_inspire_dds:
    print("Error: enable_dex3_dds and enable_dex1_dds and enable_inspire_dds cannot be enabled at the same time")
    print("Please select one of the options")
    sys.exit(1)


import pinocchio                 
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

from dds.reset_pose_dds import *
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)

from tools.data_json_load import sim_state_to_json
from dds.sim_state_dds import *
from action_provider.create_action_provider import create_action_provider
from tools.get_stiffness import get_robot_stiffness_from_env
from tools.get_reward import get_step_reward_value,get_current_rewards

def setup_signal_handlers(controller,dds_manager=None,image_server=None,nav_udp_cmd_bridge=None):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        try:
            controller.stop()
        except Exception as e:
            print(f"Failed to stop controller: {e}")
        try:
            if dds_manager is not None:
                dds_manager.stop_all_communication()
        except Exception as e:
            print(f"Failed to stop DDS: {e}")
        try:
            if image_server is not None:
                image_server.stop()
        except Exception as e:
            print(f"Failed to stop image server: {e}")
        try:
            if nav_udp_cmd_bridge is not None:
                nav_udp_cmd_bridge.stop()
        except Exception as e:
            print(f"Failed to stop nav UDP command bridge: {e}")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)



def main():
    """main function"""
    # import cProfile
    # import pstats
    # import io
    # profiler = cProfile.Profile()
    # profiler.enable()
    import os
    import atexit
    try:
        os.setpgrp()
        current_pgid = os.getpgrp()
        print(f"Setting process group: {current_pgid}")
        
        def cleanup_process_group():
            try:
                print(f"Cleaning up process group: {current_pgid}")
                import signal
                os.killpg(current_pgid, signal.SIGTERM)
            except Exception as e:
                print(f"Failed to clean up process group: {e}")
        
        atexit.register(cleanup_process_group)
        
    except Exception as e:
        print(f"Failed to set process group: {e}")
    print("=" * 60)
    print("robot control system started")
    print(f"Task: {args_cli.task}")
    print(f"Action source: {args_cli.action_source}")
    print("=" * 60)

    # parse environment configuration
    try:
        env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
        env_cfg.env_name = args_cli.task
    except Exception as e:
        print(f"Failed to parse environment configuration: {e}")
        return
    
    # create environment
    print("\ncreate environment...")
    try:
        env_cfg.seed = args_cli.seed
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        env.seed(args_cli.seed)
        try:
            sensors_dict = getattr(env.scene, "sensors", {})
            if sensors_dict:
                print("Sensors in the environment:")
                for name, sensor in sensors_dict.items():
                    print(name, sensor)
                print("="*60)
        except Exception as e:
            print(f"[sim] failed to list sensors: {e}")
        print(f"\ncreate environment success ...")
        try:
            env._reward_interval = max(1, int(args_cli.env_reward_interval))
            env._reward_counter = 0
            env._reward_last = None
            print(f"[env] reward compute interval set to {env._reward_interval} steps")
        except Exception as e:
            print(f"[env] failed to set reward interval: {e}")
        if args_cli.physics_dt is not None:
            try:
                env.sim.set_substep_time(args_cli.physics_dt)
                print(f"[sim] physics dt set to {args_cli.physics_dt}")
            except Exception:
                try:
                    env.sim.dt = args_cli.physics_dt
                    print(f"[sim] physics dt assigned to env.sim.dt={args_cli.physics_dt}")
                except Exception as e:
                    print(f"[sim] failed to set physics dt: {e}")
        headless_mode = bool(getattr(args_cli, "headless", False))
        render_interval = None
        if args_cli.render_interval is not None:
            try:
                render_interval = max(1, int(args_cli.render_interval))
            except Exception as e:
                print(f"[sim] invalid render_interval value {args_cli.render_interval}: {e}")
        try:
            if args_cli.no_render:
                env.sim.render_interval = 1_000_000
                env.sim.render_mode = "offscreen"
                print("[sim] rendering disabled via --no_render")
            elif headless_mode:
                env.sim.render_mode = "offscreen"
                env.sim.render_interval = render_interval or 1
                print(f"[sim] headless offscreen rendering every {env.sim.render_interval} steps")
            elif render_interval is not None:
                env.sim.render_interval = render_interval
                print(f"[sim] render_interval set to {env.sim.render_interval}")
        except Exception as e:
            print(f"[sim] failed to configure rendering: {e}")
        if args_cli.camera_write_interval is not None:
            try:
                import tasks.common_observations.camera_state as cam_state
                cam_state._camera_cache['write_interval_steps'] = max(1, int(args_cli.camera_write_interval))
                print(f"[camera] write interval steps set to {cam_state._camera_cache['write_interval_steps']}")
            except Exception as e:
                print(f"[camera] failed to set write interval: {e}")

        try:
            if args_cli.solver_iterations is not None:
                env.sim.physx.solver_iteration_count = int(args_cli.solver_iterations)
                print(f"[sim] solver_iteration_count={env.sim.physx.solver_iteration_count}")
            if args_cli.physx_substeps is not None:
                try:
                    env.sim.physx.substeps = int(args_cli.physx_substeps)
                except Exception:
                    try:
                        env.sim.set_substeps(int(args_cli.physx_substeps))
                    except Exception:
                        pass
                print(f"[sim] physx_substeps set to {args_cli.physx_substeps}")
            if args_cli.gravity_z is not None:
                g = float(args_cli.gravity_z)
                env.sim.physx.gravity = (0.0, 0.0, g)
                print(f"[sim] gravity set to {env.sim.physx.gravity}")
        except Exception as e:
            print(f"[sim] failed to set physx params: {e}")
        if args_cli.skip_cvtcolor:
            os.environ["CAMERA_SKIP_CVTCOLOR"] = "1"
        try:
            import tasks.common_observations.camera_state as cam_state
            enable_jpeg = bool(args_cli.camera_jpeg) or (os.getenv("CAMERA_JPEG") == "1")
            jpeg_quality = int(args_cli.camera_jpeg_quality if args_cli.camera_jpeg else os.getenv("CAMERA_JPEG_QUALITY", args_cli.camera_jpeg_quality))
            cam_state.set_writer_options(enable_jpeg=enable_jpeg, jpeg_quality=jpeg_quality, skip_cvtcolor=args_cli.skip_cvtcolor)
            include = [n.strip() for n in (args_cli.camera_include or "").split(',') if n.strip()]
            exclude = [n.strip() for n in (args_cli.camera_exclude or "").split(',') if n.strip()]
            try:
                cam_state.set_camera_allowlist(include)
            except Exception:
                pass
            try:
                sensors_dict = getattr(env.scene, "sensors", {})
                for name, sensor in sensors_dict.items():
                    lname = name.lower()
                    if "camera" not in lname:
                        continue
                    if exclude and name in exclude:
                        for attr_name, value in [("enabled", False), ("is_enabled", False)]:
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, value)
                                except Exception:
                                    pass
                        for meth in ("set_active", "disable", "pause"):
                            if hasattr(sensor, meth):
                                try:
                                    getattr(sensor, meth)(False)
                                except Exception:
                                    pass
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
                    elif include and name not in include:
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
            except Exception as e:
                print(f"[camera] failed to tune sensors: {e}")
        except Exception as e:
            print(f"[camera] failed to apply writer options: {e}")
    except Exception as e:
        print(f"\nFailed to create environment: {e}")
        return
    
    # get robot stiffness and damping parameters from runtime environment
    print("\n" + "="*60)
    print("🔍 Getting robot stiffness and damping parameters from runtime environment")
    print("="*60)
    
    try:
        stiffness_data = get_robot_stiffness_from_env(env)
        if stiffness_data:
            print("✅ Successfully got robot parameters!")
        else:
            print("⚠️ Failed to get robot parameters, will try again after environment reset")
    except Exception as e:
        print(f"⚠️ Error getting robot parameters: {e}")
    
    print("="*60)
    
    if not getattr(args_cli, "headless", False) and not args_cli.no_render:
        print("\n")
        print("***  Please left-click on the Sim window to activate rendering. ***")
        print("\n")
    else:
        print("\n")
        print("***  Running without GUI; rendering handled offscreen. ***")
        print("\n")
    # reset environment
    if args_cli.modify_light:
        update_light(
            prim_path="/World/light",
            color=(0.75, 0.75, 0.75),
            intensity=500.0,
            # position=(1.0, 2.0, 3.0),
            radius=0.1,
            enabled=True,
            cast_shadows=True
        )
    if args_cli.modify_camera:
        batch_augment_cameras_by_name(
            names=["front_cam"],
            focal_length=3.0,
            horizontal_aperture=22.0,
            vertical_aperture=16.0,
            exposure=0.8,                
            focus_distance=1.2
        )
    env.sim.reset()
    env.reset()
    if args_cli.export_nav_static_map:
        try:
            from ros2_bridge.g1_nav_static_map_exporter import (
                G1NavStaticMapExportConfig,
                export_g1_nav_static_map,
            )

            map_origin = args_cli.nav_static_map_origin
            if map_origin is None:
                robot_pos = env.scene["robot"].data.root_pos_w[0].detach().cpu().tolist()
                map_origin = (float(robot_pos[0]), float(robot_pos[1]), 0.1)
            bound_prim_path, exclude_prim_paths = resolve_nav_static_map_scope(
                args_cli.task,
                args_cli.nav_static_map_bound_prim,
                args_cli.nav_static_map_exclude_prims,
            )
            map_info = export_g1_nav_static_map(
                G1NavStaticMapExportConfig(
                    output_yaml=args_cli.export_nav_static_map,
                    cell_size=args_cli.nav_static_map_cell_size,
                    origin=tuple(map_origin),
                    z_bounds=tuple(args_cli.nav_static_map_z_bounds),
                    bound_prim_path=bound_prim_path,
                    padding=args_cli.nav_static_map_padding,
                    exclude_prim_paths=exclude_prim_paths,
                    apply_collision_to_meshes=not args_cli.nav_static_map_no_mesh_collision,
                )
            )
            print(f"[nav_map] Static map exported: {map_info}")
        except Exception as e:
            print(f"[nav_map] failed to export static map: {e}")
            return
        finally:
            env.close()
        return
    if args_cli.enable_nav_ros_clock:
        try:
            from ros2_bridge.g1_nav_clock_bridge import (
                G1NavClockBridgeConfig,
                create_g1_nav_clock_graph,
            )

            clock_info = create_g1_nav_clock_graph(
                G1NavClockBridgeConfig(
                    graph_path=args_cli.nav_ros_clock_graph_path,
                    clock_topic=args_cli.nav_ros_clock_topic,
                )
            )
            print(f"[nav_ros] Clock bridge enabled: {clock_info}")
        except Exception as e:
            print(f"[nav_ros] failed to enable clock bridge: {e}")
            return
    if args_cli.enable_nav_ros_tf_odom:
        try:
            from ros2_bridge.g1_nav_tf_odom_bridge import (
                G1NavTfOdomBridgeConfig,
                create_g1_nav_tf_odom_graph,
            )

            bridge_info = create_g1_nav_tf_odom_graph(
                env,
                G1NavTfOdomBridgeConfig(
                    graph_path=args_cli.nav_ros_graph_path,
                    robot_prim_path=args_cli.nav_ros_robot_prim_path,
                    map_frame=args_cli.nav_ros_map_frame,
                    odom_frame=args_cli.nav_ros_odom_frame,
                    base_frame=args_cli.nav_ros_base_frame,
                    odom_topic=args_cli.nav_ros_odom_topic,
                    tf_topic=args_cli.nav_ros_tf_topic,
                    map_odom_translation=args_cli.nav_ros_map_odom_translation,
                ),
            )
            print(f"[nav_ros] TF/odom bridge enabled: {bridge_info}")
        except Exception as e:
            print(f"[nav_ros] failed to enable TF/odom bridge: {e}")
            return
    if args_cli.enable_nav_ros_pointcloud:
        try:
            from ros2_bridge.g1_nav_pointcloud_bridge import (
                G1NavPointCloudBridgeConfig,
                create_g1_nav_pointcloud_graph,
            )

            pointcloud_info = create_g1_nav_pointcloud_graph(
                env,
                G1NavPointCloudBridgeConfig(
                    graph_path=args_cli.nav_ros_pointcloud_graph_path,
                    camera_prim_path=args_cli.nav_ros_camera_prim_path,
                    pointcloud_topic=args_cli.nav_ros_pointcloud_topic,
                    camera_info_topic=args_cli.nav_ros_camera_info_topic,
                    frame_id=args_cli.nav_ros_camera_frame,
                    node_namespace=args_cli.nav_ros_node_namespace,
                    width=args_cli.nav_ros_camera_width,
                    height=args_cli.nav_ros_camera_height,
                ),
            )
            print(f"[nav_ros] PointCloud2 bridge enabled: {pointcloud_info}")
        except Exception as e:
            print(f"[nav_ros] failed to enable PointCloud2 bridge: {e}")
            return
    if (
        args_cli.enable_nav_ros_clock
        or args_cli.enable_nav_ros_tf_odom
        or args_cli.enable_nav_ros_pointcloud
    ):
        try:
            from ros2_bridge.g1_nav_clock_bridge import (
                ensure_nav_ros_timeline_playing,
            )

            started_timeline = ensure_nav_ros_timeline_playing()
            print(f"[nav_ros] timeline playback active: started={started_timeline}")
        except Exception as e:
            print(f"[nav_ros] failed to start timeline playback: {e}")
            return
    
    # create simplified control configuration
    try:    
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data
        )
    except Exception as e:
        print(f"Failed to create control configuration: {e}")
        return
    
    nav_udp_cmd_bridge = None

    if not args_cli.replay_data:
        print("========= create image server =========")
        try:
            image_server = run_isaacsim_server()
        except Exception as e:
            print(f"Failed to create image server: {e}")
            return
        print("========= create image server success =========")
        print("========= create dds =========")
        try:
            reset_pose_dds,sim_state_dds,dds_manager = create_dds_objects(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
        if args_cli.enable_nav_udp_cmd_bridge:
            try:
                run_command_dds = dds_manager.get_object("run_command")
                if run_command_dds is None:
                    print("[nav_udp_cmd] run_command DDS object is not available. Use a Wholebody task or --enable_wholebody_dds.")
                    return
                nav_udp_cmd_bridge = G1NavUdpCmdBridge(
                    run_command_dds,
                    G1NavUdpCmdBridgeConfig(
                        host=args_cli.nav_udp_cmd_host,
                        port=args_cli.nav_udp_cmd_port,
                        stale_timeout_sec=args_cli.nav_udp_cmd_stale_timeout,
                    ),
                )
                nav_udp_cmd_bridge.start()
                print(
                    f"[nav_udp_cmd] bridge enabled on "
                    f"{args_cli.nav_udp_cmd_host}:{nav_udp_cmd_bridge.port}"
                )
            except Exception as e:
                print(f"[nav_udp_cmd] failed to start bridge: {e}")
                return
    else:
        print("========= create dds =========")
        try:
            create_dds_objects_replay(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
        from tools.data_json_load import get_data_json_list
        print("========= get data json list =========")
        data_idx=0
        data_json_list = get_data_json_list(args_cli.file_path)
        if args_cli.action_source != "replay":
            args_cli.action_source = "replay"
        print("========= get data json list success =========")
    # create action provider
    
    print(f"\ncreate action provider: {args_cli.action_source}...")
    try:
        print(f"args_cli.task: {args_cli.task}")
        if not args_cli.replay_data and ("Wholebody" in args_cli.task or args_cli.enable_wholebody_dds):
            args_cli.action_source = "dds_wholebody"
            args_cli.enable_wholebody_dds = True
            control_config.use_rl_action_mode = True
        action_provider = create_action_provider(env,args_cli)
        if action_provider is None:
            print("action provider creation failed, exiting")
            return
    except Exception as e:
        print(f"Failed to create action provider: {e}")
        return
    
    # set action provider
    print("========= create controller =========")
    controller = RobotController(env, control_config)
    controller.set_action_provider(action_provider)
    print("========= create controller success =========")
    
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")


    # set signal handlers
    if not args_cli.replay_data:
        setup_signal_handlers(controller,dds_manager,image_server,nav_udp_cmd_bridge)
    else:
        setup_signal_handlers(controller)
        
    print("Note: The DDS in Sim transmits messages on channel 1. Please ensure that other DDS instances use the same channel for message exchange by setting: ChannelFactoryInitialize(1).")
    try:
        # start controller - start asynchronous components
        print("========= start controller =========")
        controller.start()
        print("========= start controller success =========")
        
        # main loop - execute in main thread to support rendering
        last_stats_time = time.time()
        loop_start_time = time.time()
        loop_count = 0
        last_loop_time = time.time()
        recent_loop_times = []  # for calculating moving average frequency
        
        
        reward_interval = max(1, args_cli.reward_interval)

        # use torch.inference_mode() and exception suppression
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while simulation_app.is_running() and controller.is_running:
                current_time = time.time()
                loop_count += 1
                if not args_cli.replay_data:
                    try:
                        env_state = env.scene.get_state()
                        env_state_json =  sim_state_to_json(env_state)
                        sim_state = {"init_state":env_state_json,"task_name":args_cli.task}
                    except Exception as e:
                        print(f"Failed to get env state: {e}")
                        raise e
                    try:
                    # sim_state = json.dumps(sim_state)
                        sim_state_dds.write_sim_state_data(sim_state)
                    except Exception as e:
                        print(f"Failed to write sim state: {e}")
                        raise e
                    try:
                        reset_pose_cmd = reset_pose_dds.get_reset_pose_command()
                    except Exception as e:
                        print(f"Failed to get reset pose command: {e}")
                        raise e
                    # Compute current reward values manually if needed for debugging
                    try:
                        if (loop_count % reward_interval) == 0:
                            pass
                            # current_reward = get_step_reward_value(env)
                    except Exception as e:
                        print(f"奖励计算失败: {e}")
                        pass
                    
                    if reset_pose_cmd is not None:
                        try:
                            reset_category = reset_pose_cmd.get("reset_category")
                            if (args_cli.enable_wholebody_dds and (reset_category == '1' or reset_category == '2')) or (not args_cli.enable_wholebody_dds and reset_category == '1'):
                                print("reset object")
                                env_cfg.event_manager.trigger("reset_object_self", env)
                                reset_pose_dds.write_reset_pose_command(-1)
                            elif reset_category == '2' and not args_cli.enable_wholebody_dds:
                                print("reset all")
                                env_cfg.event_manager.trigger("reset_all_self", env)
                                reset_pose_dds.write_reset_pose_command(-1)
                        except Exception as e:
                            print(f"Failed to write reset pose command: {e}")
                            raise e
                else:
                    if action_provider.get_start_loop() and data_idx<len(data_json_list):
                        print(f"data_idx: {data_idx}")
                        try:
                            sim_state,task_name = action_provider.load_data(data_json_list[data_idx])
                            if task_name!=args_cli.task:
                                raise ValueError(f" The {task_name} in the dataset is different from the {args_cli.task} being executed .")
                        except Exception as e:
                            print(f"Failed to load data: {e}")
                            raise e
                        try:
                            env.reset_to(sim_state, torch.tensor([0], device=env.device), is_relative=True)
                            env.sim.reset()
                            time.sleep(1)
                            action_provider.start_replay()
                            data_idx+=1
                        except Exception as e:
                            print(f"Failed to start replay: {e}")
                            raise e
                # print(f"env_state: {env_state}")
                # calculate instantaneous loop time
                loop_dt = current_time - last_loop_time
                last_loop_time = current_time
                recent_loop_times.append(loop_dt)
                
                # keep recent 100 loop times
                if len(recent_loop_times) > 100:
                    recent_loop_times.pop(0)
                
                # execute control step (in main thread, support rendering)
                controller.step()

                # print statistics and loop frequency periodically
                if current_time - last_stats_time >= args_cli.stats_interval:
                    # calculate while loop execution frequency
                    elapsed_time = current_time - loop_start_time
                    loop_frequency = loop_count / elapsed_time if elapsed_time > 0 else 0
                    
                    # calculate moving average frequency (based on recent loop times)
                    if recent_loop_times:
                        avg_loop_time = sum(recent_loop_times) / len(recent_loop_times)
                        moving_avg_frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
                        min_loop_time = min(recent_loop_times)
                        max_loop_time = max(recent_loop_times)
                        max_freq = 1.0 / min_loop_time if min_loop_time > 0 else 0
                        min_freq = 1.0 / max_loop_time if max_loop_time > 0 else 0
                    else:
                        moving_avg_frequency = 0
                        min_freq = max_freq = 0
                    
                    print(f"\n=== While loop execution frequency statistics ===")
                    print(f"loop execution count: {loop_count}")
                    print(f"running time: {elapsed_time:.2f} seconds")
                    print(f"overall average frequency: {loop_frequency:.2f} Hz")
                    print(f"moving average frequency: {moving_avg_frequency:.2f} Hz (last {len(recent_loop_times)} times)")
                    print(f"frequency range: {min_freq:.2f} - {max_freq:.2f} Hz")
                    print(f"average loop time: {(elapsed_time/loop_count*1000):.2f} ms")
                    if recent_loop_times:
                        print(f"recent loop time: {(avg_loop_time*1000):.2f} ms")
                    print(f"=============================")
                    
                    # print_stats(controller)
                    last_stats_time = current_time
       
                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                # rate_limiter.sleep(env)
    except KeyboardInterrupt:
        print("\nuser interrupted program")
    
    except Exception as e:
        print(f"\nprogram exception: {e}")
    
    finally:
        # clean up resources
        print("\nclean up resources...")
        if nav_udp_cmd_bridge is not None:
            nav_udp_cmd_bridge.stop()
        controller.cleanup()
        image_server.stop()
        env.close()
        print("cleanup completed")
    # profiler.disable()
    # s = io.StringIO()
    # ps = pstats.Stats(profiler, stream=s).strip_dirs().sort_stats("time")
    # ps.print_stats(30)  

    # print(s.getvalue())

if __name__ == "__main__":
    try:
        main()
    finally:
        print("Performing final cleanup...")
        
        # Get current process information
        import os
        from ros2_bridge.process_cleanup import cleanup_matching_descendants
        
        current_pid = os.getpid()
        print(f"Current main process PID: {current_pid}")
        
        try:
            cleanup_matching_descendants(command_substring="sim_main.py")
        except Exception as e:
            print(f"Error during process cleanup: {e}")
        
        try:
            simulation_app.close()
        except Exception as e:
            print(f"Failed to close simulation application: {e}")
            
        print("Program exit completed")
        
        # Force exit
        os._exit(0)

# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint   --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129

# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-RedBlock-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint     --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Inspire-Joint     --enable_inspire_dds --robot_type g129




# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody  --robot_type g129 --enable_dex1_dds 
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Dex3-Wholebody  --robot_type g129 --enable_dex3_dds 
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody  --robot_type g129 --enable_inspire_dds 


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint --enable_inspire_dds --robot_type h1_2
