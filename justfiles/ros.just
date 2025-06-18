[group("ROS")]
ros-build:
    docker compose run --rm maly_woz bash -c "\
        . /opt/ros/humble/setup.bash && \
        colcon build \
            --symlink-install \
            --merge-install \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "

[group("ROS")]
sim:
    docker compose run --rm maly_woz bash -c "\
        . /opt/ros/humble/setup.bash && \
        . /workspace/ros_ws/install/setup.bash && \
        ros2 launch maly_woz_bringup start_sim.launch.py use_sim_time:=true \
    "

[group("ROS")]
teleop:
    docker compose run --rm maly_woz bash -c "\
        . /opt/ros/humble/setup.bash && \
        . /workspace/ros_ws/install/setup.bash && \
        ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/model/rover/cmd_vel \
    "
