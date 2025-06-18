[group("docker")]
docker-build:
    #!/usr/bin/env bash
    docker build --build-arg="user_id=1000" --rm -t almu_ros2_base docker/almu_ros2_base
    docker compose build maly_woz

[group("docker")]
docker-start:
    docker compose up -d maly_woz

[group("docker")]
docker-stop:
    docker compose down

[group("docker")]
docker-attach:
    xhost +local:docker
    docker exec -it rover bash
