[group("docker")]
docker-build:
    #!/usr/bin/env bash
    set -e

    echo "Building OpenGL base images..."
    (cd docker/opengl ; ./build.sh)
    echo "Done"

    docker compose build software
    docker compose build hardware
    docker compose build simulation-nvidia
    docker compose build simulation-intel-amd

[group("docker")]
docker-attach-soft:
    #!/usr/bin/env bash
    xhost +local:docker
    docker compose run --rm -e force_color_prompt=yes software bash

[group("docker")]
docker-attach-sim-nvidia:
    #!/usr/bin/env bash
    xhost +local:docker
    docker compose run --rm -e force_color_prompt=yes simulation-nvidia bash

[group("docker")]
docker-attach-hard:
    #!/usr/bin/env bash
    xhost +local:docker
    docker compose run --rm -e force_color_prompt=yes hardware bash
