import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rtabmap = get_package_share_directory("rtabmap_launch")

    rtabmap_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rtabmap, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "args": "--delete_db_on_start -d --Reg/Force3DoF true --GFTT/MinDistance 10",
            "odom_args": "--Odom/Strategy 1",
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

    # RGB-D odometry
    rgbd_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "ground_truth_frame_id": "",
                "ground_truth_base_frame_id": "",
                "wait_for_transform": 0.2,
                "wait_imu_to_init": True,
                "approx_sync": False,
                "approx_sync_max_interval": 0.0,
                "config_path": "",
                "queue_size": 10,
                "qos": 1,
                "qos_camera_info": 1,
                "qos_imu": 1,
                "subscribe_rgbd": False,
                "guess_frame_id": "",
                "guess_min_translation": 0.0,
                "guess_min_rotation": 0.0,
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("rgb/image", "/realsense_d435/image"),
            ("depth/image", "/realsense_d435/depth_image"),
            ("rgb/camera_info", "/realsense_d435/camera_info"),
            ("rgbd_image", "rgbd_image"),
            ("odom", "odom"),
            ("imu", "/realsense_d435/imu"),
        ],
        arguments=[
            # "--delete_db_on_start -d --Reg/Force3DoF true --GFTT/MinDistance 5 --Vis/MaxFeatures 500",
            "-d --Reg/Force3DoF true",
            "--Odom/Strategy 1",  # frame to frame, feature based
        ],
        namespace="rtabmap",
    )

    return LaunchDescription(
        [
            # rgbd_odom,
            rtabmap_mapping,
        ]
    )
