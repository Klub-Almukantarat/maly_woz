from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    comp_filter = Node(
        package="localization_dtu",
        executable="velocity_complementary_filter_node",
        remappings=[
            ("primary_odom", "/model/rover/odometry"),
            ("secondary_odom", "/rtabmap/odom"),
            ("output", "/model/rover/odom_filtered"),
        ],
    )

    localization = Node(
        package="localization_dtu",
        executable="localization",
        remappings=[
            ("twist", "/model/rover/odom_filtered"),
            ("output", "/model/rover/odom_localized"),
        ],
    )

    return LaunchDescription(
        [
            comp_filter,
            localization,
        ]
    )
