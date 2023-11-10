import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_rtabmap = get_package_share_directory("rtabmap_launch")

    rtabmap_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rtabmap, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "args": "--delete_db_on_start -d --Reg/Force3DoF true --GFTT/MinDistance 10",
            "depth_topic": "/realsense_d435/depth_image",
            "rgb_topic": "/realsense_d435/image",
            "camera_info_topic": "/realsense_d435/camera_info",
            "imu_topic": "/realsense_d435/imu",
            "wait_imu_to_init": "true",
            "approx_sync": "false",
            "use_sim_time": "true",
            "frame_id": "base_link",
        }.items(),
    )

    return LaunchDescription(
        [
            rtabmap_mapping,
        ]
    )
