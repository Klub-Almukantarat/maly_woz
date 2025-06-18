import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_description = get_package_share_directory("maly_woz_description")
    pkg_share = get_package_share_directory("maly_woz_bringup")

    xacro_file = os.path.join(pkg_description, "urdf", "basic_rover.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    params = {"use_sim_time": True, "robot_description": robot_description}

    traj_no = 3
    rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            f"/home/developer/rover_ws/data/traj{traj_no}",
            "-r",
            "1.0",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg_share, "config", "config.rviz"),
            "--ros-args",
            "--log-level",
            "error",
        ],
    )

    record_odom = Node(
        package="localization_dtu",
        executable="odom_metrics",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "name": f"traj{traj_no}_wheel",
            }
        ],
        remappings=[
            # ("odom", "/rtabmap/odom"),
            # ("odom", "/model/rover/odometry"),
            # ("odom", "/sim_odom"),
            ("odom", "/wheel_odom"),
            ("gt", "/sim_odom"),
        ],
        namespace="",
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rtabmap_mapping.launch.py")
        ),
    )

    mikf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "localization_pipeline.launch.py")
        ),
    )

    return LaunchDescription(
        [
            rosbag,
            # rtabmap,
            mikf,
            record_odom,
            TimerAction(period=0.0, actions=[rviz]),
        ]
    )
