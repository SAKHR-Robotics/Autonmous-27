#!/usr/bin/env bash
# Launch Phase 1 entirely in the project's Jazzy/Harmonic environment.
set -e
CONTROL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_DIR="$(cd "$CONTROL_DIR/../docker" && pwd)"
: "${DISPLAY:?Run this from your desktop session with DISPLAY set}"
if [[ -z "${XAUTHORITY:-}" ]]; then
    if [[ -f "${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/gdm/Xauthority" ]]; then
        export XAUTHORITY="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/gdm/Xauthority"
    elif [[ -f "$HOME/.Xauthority" ]]; then
        export XAUTHORITY="$HOME/.Xauthority"
    fi
fi
if [[ ! -r "${XAUTHORITY:-}" ]]; then
    echo 'Set XAUTHORITY to your desktop Xauthority file before launching.' >&2
    exit 2
fi
if ! docker image inspect autonomous27:jazzy >/dev/null 2>&1; then
    echo 'Build the existing autonomous27:jazzy base image using docker/Dockerfile first.' >&2
    exit 2
fi
if ! docker image inspect autonomous27:gui >/dev/null 2>&1; then
    docker build -t autonomous27:gui -f "$DOCKER_DIR/Dockerfile.gui" "$DOCKER_DIR"
fi
exec docker compose -f "$DOCKER_DIR/compose.gui.yaml" up --abort-on-container-exit
