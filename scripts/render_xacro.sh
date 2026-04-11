#!/bin/bash
# Manage a persistent Docker container for xacro rendering.
#
# Usage:
#   render_xacro.sh start                                  — build image + start container
#   render_xacro.sh render <repo_dir> <xacro_file> <out>   — render a xacro file to URDF
#   render_xacro.sh stop                                   — stop + remove container

set -e

IMAGE_NAME="robot-explorer-xacro"
CONTAINER_NAME="robot-explorer-xacro"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CACHE_DIR="${ROBOT_DESC_CACHE:-$HOME/.cache/robot_descriptions}"

case "${1:-}" in
  start)
    # Build image if it doesn't exist (Docker layer cache makes rebuilds instant)
    if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
      echo "Building xacro Docker image..."
      docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/../Dockerfile.xacro" "$SCRIPT_DIR/.." 2>&1
    fi
    # Remove stale container if any
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
    # Start persistent container with repos cache mounted
    mkdir -p "$CACHE_DIR"
    docker run -d --name "$CONTAINER_NAME" \
      -v "$CACHE_DIR":/repos:ro \
      "$IMAGE_NAME" \
      sleep infinity > /dev/null
    echo "Xacro container started"
    ;;

  stop)
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
    ;;

  render)
    REPO_DIR="$2"
    XACRO_FILE="$3"
    OUTPUT_URDF="$4"

    if [ -z "$REPO_DIR" ] || [ -z "$XACRO_FILE" ] || [ -z "$OUTPUT_URDF" ]; then
      echo "Usage: $0 render <repo_dir> <xacro_file> <output_urdf>" >&2
      exit 1
    fi

    # Translate host path to container path (repos are under CACHE_DIR)
    REL_PATH="${REPO_DIR#$CACHE_DIR/}"
    CONTAINER_REPO="/repos/$REL_PATH"

    mkdir -p "$(dirname "$OUTPUT_URDF")"
    docker exec "$CONTAINER_NAME" bash -c "
      source /opt/ros/noetic/setup.bash
      export ROS_PACKAGE_PATH=$CONTAINER_REPO
      xacro $CONTAINER_REPO/$XACRO_FILE
    " > "$OUTPUT_URDF"
    ;;

  *)
    echo "Usage: $0 {start|stop|render <repo_dir> <xacro_file> <output_urdf>}" >&2
    exit 1
    ;;
esac
