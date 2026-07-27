#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share_dir = get_package_share_directory("ydlidar_ros2_driver")
    
    driver_front = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        namespace="lidar_front",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(share_dir, "params", "TminiPro_front.yaml")],
    )
    
    driver_rear = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        namespace="lidar_rear",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(share_dir, "params", "TminiPro_rear.yaml")],
    )
    
    tf_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser_front",
        arguments=["--x", "0.44", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "3.14159265359", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "laser_front_frame"],
    )
    
    tf_rear = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser_rear",
        arguments=["--x", "-0.44", "--y", "0", "--z", "0",
                   "--roll", "3.14159265359", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "laser_rear_frame"],
    )
    
    return LaunchDescription([
        driver_front,
        driver_rear,
        tf_front,
        tf_rear,
    ])
