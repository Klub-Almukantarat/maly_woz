from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mikf_filter = Node(
        package="localization_dtu",
        executable="mikf_node",
        remappings=[
            ("wheel_odom", "/model/rover/odometry"),
            # ("vo_odom", "/rtabmap/odom"),
            # ("pose_odom", "/global_odom"),
            ("output", "/wheel_odom"),
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    global_localization = Node(
        package="localization_dtu",
        executable="global_localization",
        remappings=[
            ("odom_in", "/mikf_odom"),
            ("points", "/realsense_d435/points"),
            ("odom_out", "/global_odom"),
        ],
        parameters=[
            {
                "map_path": "/home/developer/rover_ws/data/warehouse.ply",
                "visualize": True,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            mikf_filter,
            global_localization,
        ]
    )
