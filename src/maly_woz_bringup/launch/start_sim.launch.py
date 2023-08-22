import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_description = get_package_share_directory("maly_woz_description")
    pkg_share = get_package_share_directory("maly_woz_bringup")

    xacro_file = os.path.join(pkg_description, "urdf", "basic_rover.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    params = {"use_sim_time": True, "robot_description": robot_description}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
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

    world_file = "lunar_world.sdf"  # TODO make this a parameter
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rover",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
            "-topic",
            "/robot_description",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "qos_overrides./model/rover.subscriber.reliability": "reliable",
                "config_file": os.path.join(pkg_share, "config", "ros_gz_bridge.yaml"),
            }
        ],
        output="screen",
    )

    # TODO load depending on parameter
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[
            {
                "require_enable_button": False,
                "axis_linear": {"x": 4},
                "scale_linear": {"x": 4.0},
                "axis_angular": {"yaw": 3},
            }
        ],
        remappings=[("cmd_vel", "/model/rover/cmd_vel")],
    )

    return LaunchDescription(
        [
            robot_state_pub_node,
            gazebo,
            spawn,
            bridge,
            # joy_node,
            # joy_teleop_node,
            TimerAction(period=2.0, actions=[rviz]),
        ]
    )
