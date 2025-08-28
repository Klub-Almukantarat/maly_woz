from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    ff_control = Node(
        package="maly_woz_hw",
        executable="ff_control",
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
                "scale_linear": {"x": 1.0},
                "axis_angular": {"yaw": 1.0},
            }
        ],
        # remappings=[("cmd_vel", "/model/rover/cmd_vel")],
    )

    return LaunchDescription(
        [
            joy_node,
            joy_teleop_node,
            ff_control,
        ]
    )
