#!/bin/bash
# ==============================================================================
# Start Insight GUI (RQT) - Auto-launch
# ==============================================================================

set -euo pipefail

CONTAINER_NAME="vyra_insight_rqt"
IMAGE_NAME="vyra_insight_gui:latest"

echo "=== VYRA Insight GUI Launcher ==="

# Check if image exists, build if not
if ! docker images | grep -q "vyra_insight_gui"; then
    echo "📦 Building Insight GUI image..."
    cd "$(dirname "$0")"
    docker build -t "$IMAGE_NAME" .
fi

# Stop existing container if running
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "🛑 Stopping existing container..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

# Allow X11 connections from localhost
echo "🔓 Configuring X11 permissions..."
xhost +local:docker >/dev/null 2>&1

# Use host network for ROS2 DDS communication
echo "📡 Using host network mode for ROS2 DDS..."
echo "🚀 Starting RQT GUI..."
echo ""

# Start container with RQT GUI
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=42 \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$(pwd)":/workspace \
    --device /dev/dri:/dev/dri \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/kilted/setup.bash && rqt"

# Restore X11 permissions
echo ""
echo "🔒 Restoring X11 permissions..."
xhost -local:docker >/dev/null 2>&1

echo "✅ RQT closed"
