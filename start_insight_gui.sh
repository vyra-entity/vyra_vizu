#!/bin/bash
# ==============================================================================
# Start Insight GUI Container with X11 forwarding
# ==============================================================================

set -euo pipefail

CONTAINER_NAME="vyra_insight_gui"
IMAGE_NAME="vyra_insight_gui:latest"

echo "=== Insight GUI Launcher ==="

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
xhost +local:docker

# Use host network for ROS2 DDS communication (multicast)
echo "📡 Using host network mode for ROS2 DDS multicast..."
NETWORK_ARG="--network host"

# Start container with X11 forwarding and network access
echo "🚀 Starting Insight GUI container..."
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    $NETWORK_ARG \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=42 \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$(pwd)":/workspace \
    --volume "$(cd .. && pwd)/modules":/modules:ro \
    --device /dev/dri:/dev/dri \
    "$IMAGE_NAME" \
    bash -c "
# Source ROS2 base
source /opt/ros/kilted/setup.bash

# Source all module interfaces
for module_dir in /modules/v2_*/install; do
    if [ -d \"\$module_dir\" ]; then
        for setup_file in \"\$module_dir\"/*/local_setup.bash; do
            if [ -f \"\$setup_file\" ]; then
                echo \"📦 Sourcing: \$setup_file\"
                source \"\$setup_file\"
            fi
        done
    fi
done

echo '
╔════════════════════════════════════════════════════════════╗
║           VYRA Insight GUI - ROS2 Visualization            ║
╔════════════════════════════════════════════════════════════╗
║                                                            ║
║  ROS2 Commands:                                            ║
║    ros2 node list           - List all ROS2 nodes          ║
║    ros2 topic list          - List all topics              ║
║    ros2 service list        - List all services            ║
║    ros2 topic echo <topic>  - Monitor topic messages       ║
║                                                            ║
║  GUI Tools:                                                ║
║    rqt                      - Start RQT GUI                ║
║    rqt_graph               - Node graph visualization      ║
║    rqt_console             - Log console                   ║
║    rviz2                   - 3D visualization              ║
║                                                            ║
║  Type exit to quit                                         ║
╚════════════════════════════════════════════════════════════╝
'
bash"

# Restore X11 permissions
echo "🔒 Restoring X11 permissions..."
xhost -local:docker

echo "✅ Insight GUI closed"
