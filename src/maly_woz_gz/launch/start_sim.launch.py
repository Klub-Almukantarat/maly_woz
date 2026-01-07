import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    pkg_share = get_package_share_directory("maly_woz_gz")
    pkg_description = get_package_share_directory("maly_woz_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(pkg_description, "urdf", "basic_rover.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    # world_file = "warehouse.sdf"  # TODO make this a parameter
    world_file = "lunar_world.sdf"
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

    return LaunchDescription(
        [
            gazebo,
            spawn,
            bridge,
        ]
    )
