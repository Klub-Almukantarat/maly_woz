[group("init")]
just-autocomplete:
    #!/usr/bin/env bash
    mkdir -p ~/bin
    just --completions bash > ~/bin/just-completions.bash
    echo ". ~/bin/just-completions.bash" >> ~/.bashrc

[group("init")]
hello-world-docker:
    docker run hello-world

[group("init")]
hello-world-nvidia:
    docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
