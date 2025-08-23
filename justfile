docker-build:
    ./build_image.sh

docker-sim-nvidia:
    docker compose --profile sim-nvidia up -d

docker-sim-intel-amd:
    docker compose --profile sim-intel-amd up -d

docker-run:
    docker compose --profile hw up -d

docker-attach:
    ./attach.sh

ros-build:
    #!/usr/bin/env bash
    . sor
    . bild

ros-run:
    #!/usr/bin/env bash
    . sor
    ./start_sim

ros-teleop:
    #!/usr/bin/env bash
    . sor
    ./teleop
