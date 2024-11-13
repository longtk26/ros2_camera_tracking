from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # Define paths to URDF and RViz config
    rviz_config_path = "/home/ngoclong/ros2_ws/src/opencv_tools/rviz/camera.rviz"

    publish_image_node = Node(
        package="opencv_tools",
        executable="image_publish_node",
    )

    detect_tracking_node = Node(
        package="opencv_tools",
        executable="object_detection_node",
    )

    socket_node = Node(
        package="opencv_tools",
        executable="socket_node",
    )

    http_node = Node(
        package="opencv_tools",
        executable="http_node",
    )

    # RViz2 node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        # rviz2_node,
        publish_image_node,
        detect_tracking_node,
        socket_node,
        http_node
    ])
