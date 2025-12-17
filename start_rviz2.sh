#!/bin/bash
# ==============================================================================
# Start RVIZ2 - 3D Visualization
# ==============================================================================

set -euo pipefail

CONTAINER_NAME="vyra_insight_rviz2"
IMAGE_NAME="vyra_insight_gui:latest"

echo "=== VYRA RVIZ2 Launcher ==="

# Check if image exists
if ! docker images | grep -q "vyra_insight_gui"; then
    echo "❌ Error: vyra_insight_gui image not found. Build it first with ./start_insight_gui.sh"
    exit 1
fi

# Stop existing container if running
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

# Allow X11 connections
echo "🔓 Configuring X11 permissions..."
xhost +local:docker >/dev/null 2>&1

echo "🚀 Starting RVIZ2..."
echo ""

# Start container with RVIZ2
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=42 \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$(pwd)":/workspace \
    --device /dev/dri:/dev/dri \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/kilted/setup.bash && rviz2"

# Restore X11 permissions
echo ""
echo "🔒 Restoring X11 permissions..."
xhost -local:docker >/dev/null 2>&1

echo "✅ RVIZ2 closed"
