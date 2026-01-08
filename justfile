set dotenv-load
set dotenv-required

ls:
    just --list

import "justfiles/init.just"
import "justfiles/ros.just"
import "docker/docker.just"

