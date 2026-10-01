#!/usr/bin/python3
"""Two YDLIDAR T-mini Pro on ALICE M2, plus their mount TFs.

The mounts (base_link -> laser_*_frame) come from a calibration file instead of
this launch file:

  ros2 launch ydlidar_ros2_driver ydlidar_dual.launch.py                      # params/lidar_extrinsics.yaml
  ros2 launch ydlidar_ros2_driver ydlidar_dual.launch.py extrinsics_file:=/path/file.yaml
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

ROOT_KEY = "alice_m2_lidar_extrinsics"
FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def load_extrinsics(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    root = data.get(ROOT_KEY, data)
    parent = root.get("parent_frame", "base_link")
    lidars = root.get("lidars", {})
    for name in ("front", "rear"):
        if name not in lidars:
            raise RuntimeError(f"{path}: lidars.{name} missing")
        missing = [k for k in ("frame_id",) + FIELDS if k not in lidars[name]]
        if missing:
            raise RuntimeError(f"{path}: lidars.{name} missing {missing}")
    return parent, lidars


def _nodes(context):
    share_dir = get_package_share_directory("ydlidar_ros2_driver")
    path = LaunchConfiguration("extrinsics_file").perform(context)
    if not path:
        path = os.path.join(share_dir, "params", "lidar_extrinsics.yaml")
    parent, lidars = load_extrinsics(path)

    actions = [LogInfo(msg=f"[ydlidar_dual] lidar extrinsics: {path}")]
    for name in ("front", "rear"):
        m = lidars[name]
        actions.append(LogInfo(msg=(
            f"  {name}: {parent} -> {m['frame_id']}  xyz ({m['x']}, {m['y']}, {m['z']})  "
            f"rpy ({m['roll']}, {m['pitch']}, {m['yaw']})")))
        actions.append(LifecycleNode(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            namespace=f"ydlidar_{name}",
            output="screen",
            emulate_tty=True,
            parameters=[os.path.join(share_dir, "params", f"TminiPro_{name}.yaml")],
        ))
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_pub_laser_{name}",
            arguments=["--x", str(m["x"]), "--y", str(m["y"]), "--z", str(m["z"]),
                       "--roll", str(m["roll"]), "--pitch", str(m["pitch"]), "--yaw", str(m["yaw"]),
                       "--frame-id", parent, "--child-frame-id", m["frame_id"]],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "extrinsics_file", default_value="",
            description="Lidar mount calibration YAML (default: params/lidar_extrinsics.yaml)"),
        OpaqueFunction(function=_nodes),
    ])
