#!/usr/bin/env bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash

sudo chown vscode:vscode /workspace /workspace/src

mkdir -p /workspace/src && cd /workspace/src && catkin_init_workspace

exec "$@"
