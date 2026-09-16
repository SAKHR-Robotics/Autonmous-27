#!/bin/bash

# Allow local root and docker access to X11 display server for RViz2 / Gazebo GUIs
xhost +local:root >/dev/null 2>&1
xhost +local:docker >/dev/null 2>&1

# Auto-handle permission denied by invoking with sg docker if needed
if ! docker ps >/dev/null 2>&1; then
    if sg docker -c "docker ps" >/dev/null 2>&1; then
        exec sg docker -c "$0 $*"
    else
        echo "⚠️ Docker permission denied for your current user!"
        echo "   Please run: newgrp docker"
        echo "   Or run: sudo ./run_docker.sh"
        echo "   (To fix permanently across all terminals: Log out of Ubuntu and log back in)."
        exit 1
    fi
fi

# Navigate to script directory and start container
CDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$CDIR" || exit

docker compose run --name autonomous27_container --rm autonomous_rover bash
