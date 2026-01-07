[group("ROS")]
ros-build:
    docker compose run --rm software bash -c "\
        . /opt/ros/jazzy/setup.bash && \
        colcon build \
            --symlink-install \
            --merge-install \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "

[group("ROS")]
start-sim-nvidia:
    docker compose run --rm simulation-nvidia bash -c "\
        . /workspace/install/setup.bash && \
        ros2 launch maly_woz_bringup start_sim.launch.py use_sim_time:=true \
    "

[group("ROS")]
start-teleop:
    docker compose run --rm software bash -c "\
        . /workspace/install/setup.bash && \
        ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/model/rover/cmd_vel \
    "
