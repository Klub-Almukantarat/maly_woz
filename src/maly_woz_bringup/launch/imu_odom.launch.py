from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_transformer = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        parameters=[
            {
                "target_frame": "base_link",
            }
        ],
        remappings=[
            ("imu_in", "/realsense_d435/imu"),
            ("imu_out", "/model/rover/imu_base_link"),
        ],
    )

    imu_odom = Node(
        package="localization_dtu",
        executable="imu_odom",
        remappings=[
            ("imu", "/model/rover/imu_base_link"),
            ("odom", "/model/rover/odom_imu"),
        ],
    )

    return LaunchDescription(
        [
            imu_transformer,
            imu_odom,
        ]
    )
