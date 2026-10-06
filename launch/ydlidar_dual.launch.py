#!/usr/bin/python3
"""Two YDLIDAR T-mini Pro on ALICE M2.

The scans come in the URDF links front_lidar_link / rear_lidar_link, whose
mounts base_link -> link the base launch publishes from the robot's URDF
(alice_common m2_sensors.xacro + alice_parameters robot_N sensor_extrinsics.yaml).

  ros2 launch ydlidar_ros2_driver ydlidar_dual.launch.py robot_id:=1
      with robot 1's range offsets (alice_parameters robot_1/sensor_intrinsics.yaml);
      without robot_id, the M2 robot of the profile (ALICE_GENERATION=alice_m2,
      ALICE_ROBOT_ID 1 or more), else none
  alice_parameters is looked up only then, so the driver also runs on its own:
  a robot taken from the environment without alice_parameters only warns and
  runs without offsets; robot_id:= given without it is an error.
  ros2 launch ydlidar_ros2_driver ydlidar_dual.launch.py publish_mount_tf:=true
      without the base launch: mounts from params/lidar_extrinsics.yaml
      (or extrinsics_file:=/path/file.yaml)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
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


def load_range_offsets(robot_id):
    """(path, URDF link -> range offset [m]) of robot_id from alice_parameters (none: {}).

    Raises PackageNotFoundError when alice_parameters is not installed.
    """
    if not robot_id.isdigit():
        raise RuntimeError(f"robot_id 는 호기 번호(0 이상의 정수)여야 합니다: {robot_id!r}")
    path = os.path.join(get_package_share_directory("alice_parameters"), "config", "alice_m2",
                        f"robot_{int(robot_id)}", "sensor_intrinsics.yaml")
    if not os.path.isfile(path):
        return path, {}
    with open(path) as f:
        sensors = ((yaml.safe_load(f) or {}).get("sensor_intrinsics") or {}).get("sensors") or {}
    return path, {link: float(values["range_offset"]) for link, values in sensors.items()
                  if "range_offset" in (values or {})}


def _env_robot_id():
    """The M2 real robot of the profile (ALICE_GENERATION=alice_m2, ALICE_ROBOT_ID >= 1), or ''."""
    robot_id = os.environ.get("ALICE_ROBOT_ID", "").strip()
    if os.environ.get("ALICE_GENERATION") == "alice_m2" and robot_id.isdigit() and int(robot_id) >= 1:
        return robot_id
    return ""


def _nodes(context):
    share_dir = get_package_share_directory("ydlidar_ros2_driver")
    actions = []

    given = LaunchConfiguration("robot_id").perform(context).strip()
    robot_id = given or _env_robot_id()
    offsets = {}
    if robot_id:
        try:
            path, offsets = load_range_offsets(robot_id)
            actions.append(LogInfo(msg=f"[ydlidar_dual] robot_{robot_id} range offsets: "
                                       + (path if offsets else f"none ({path} absent)")))
        except PackageNotFoundError:
            if given:
                raise RuntimeError(f"robot_id:={robot_id}: alice_parameters 가 없어 호기 거리"
                                   " 오프셋을 읽을 수 없습니다 (워크스페이스를 source 하거나"
                                   " robot_id 를 빼고 실행)") from None
            actions.append(LogInfo(msg=f"[ydlidar_dual] ALICE_ROBOT_ID={robot_id} 이지만"
                                       " alice_parameters 가 없어 거리 오프셋 없이 실행합니다"))

    for name in ("front", "rear"):
        params = os.path.join(share_dir, "params", f"TminiPro_{name}.yaml")
        with open(params) as f:
            frame_id = next(iter(yaml.safe_load(f).values()))["ros__parameters"]["frame_id"]
        offset = offsets.get(frame_id, 0.0)
        actions.append(LogInfo(msg=f"  {name}: {frame_id}, range offset {offset:+.4f} m"))
        actions.append(LifecycleNode(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            namespace=f"ydlidar_{name}",
            output="screen",
            emulate_tty=True,
            parameters=[params, {"range_offset": offset}],
        ))

    if LaunchConfiguration("publish_mount_tf").perform(context).lower() in ("true", "1"):
        path = LaunchConfiguration("extrinsics_file").perform(context)
        if not path:
            path = os.path.join(share_dir, "params", "lidar_extrinsics.yaml")
        parent, lidars = load_extrinsics(path)
        actions.append(LogInfo(msg=f"[ydlidar_dual] mount TFs from {path}"
                                   " (only without the base launch, which publishes them)"))
        for name in ("front", "rear"):
            m = lidars[name]
            actions.append(LogInfo(msg=(
                f"  {name}: {parent} -> {m['frame_id']}  xyz ({m['x']}, {m['y']}, {m['z']})  "
                f"rpy ({m['roll']}, {m['pitch']}, {m['yaw']})")))
            actions.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_pub_laser_{name}",
                arguments=["--x", str(m["x"]), "--y", str(m["y"]), "--z", str(m["z"]),
                           "--roll", str(m["roll"]), "--pitch", str(m["pitch"]),
                           "--yaw", str(m["yaw"]),
                           "--frame-id", parent, "--child-frame-id", m["frame_id"]],
            ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id", default_value="",
            description="M2 robot N: range offsets from alice_parameters"
                        " config/alice_m2/robot_N/sensor_intrinsics.yaml (empty: ALICE_ROBOT_ID"
                        " when ALICE_GENERATION=alice_m2 and it is 1 or more, else none)"),
        DeclareLaunchArgument(
            "publish_mount_tf", default_value="false",
            description="true: publish base_link -> lidar links from extrinsics_file, for"
                        " running without the base launch (which publishes them from the URDF)"),
        DeclareLaunchArgument(
            "extrinsics_file", default_value="",
            description="Lidar mounts for publish_mount_tf (default: params/lidar_extrinsics.yaml)"),
        OpaqueFunction(function=_nodes),
    ])
