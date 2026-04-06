#!/bin/bash
# Render xacro files to URDF using a ROS Noetic Docker container.
# Usage: ./scripts/render_xacro.sh <repo_dir> <xacro_file> <output_urdf>
#
# repo_dir:   Path to the cloned ROS package repo (mounted as ROS_PACKAGE_PATH)
# xacro_file: Path to the .xacro file (relative to repo_dir)
# output_urdf: Where to write the rendered URDF

set -e

REPO_DIR="$1"
XACRO_FILE="$2"
OUTPUT_URDF="$3"

if [ -z "$REPO_DIR" ] || [ -z "$XACRO_FILE" ] || [ -z "$OUTPUT_URDF" ]; then
    echo "Usage: $0 <repo_dir> <xacro_file> <output_urdf>" >&2
    exit 1
fi

docker run --rm \
    -v "$REPO_DIR":/ws \
    ros:noetic-ros-core \
    bash -c "
        apt-get update -qq > /dev/null 2>&1
        apt-get install -qq -y ros-noetic-xacro > /dev/null 2>&1
        source /opt/ros/noetic/setup.bash
        export ROS_PACKAGE_PATH=/ws
        xacro /ws/$XACRO_FILE
    " > "$OUTPUT_URDF"
