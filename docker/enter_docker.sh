#!/bin/bash

# Auto-handle permission denied by invoking with sg docker if needed
if ! docker ps >/dev/null 2>&1; then
    if sg docker -c "docker ps" >/dev/null 2>&1; then
        exec sg docker -c "$0 $*"
    else
        echo "⚠️ Docker permission denied for your current user!"
        echo "   Please run: newgrp docker"
        echo "   Or run: sudo ./enter_docker.sh"
        echo "   (To fix permanently across all terminals: Log out of Ubuntu and log back in)."
        exit 1
    fi
fi

# Find running container by image name or container name
CONTAINER_ID=$(docker ps -q -f ancestor=autonomous27:jazzy | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    docker exec -it "$CONTAINER_ID" bash
else
    echo "No running autonomous27 container found! Please run ./run_docker.sh first."
fi
