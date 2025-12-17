#!/bin/bash
# ==============================================================================
# Test Insight GUI - ROS2 Connectivity
# ==============================================================================

echo "=== Testing Insight GUI ROS2 Connectivity ==="

CONTAINER_NAME="vyra_insight_gui_test"
IMAGE_NAME="vyra_insight_gui:latest"

# Check if image exists
if ! docker images | grep -q "vyra_insight_gui"; then
    echo "❌ Error: vyra_insight_gui image not found. Build it first with ./start_insight_gui.sh"
    exit 1
fi

echo "📡 Testing ROS2 connectivity with host network..."
echo ""

# Test 1: List nodes
echo "1️⃣  Testing ros2 node list..."
docker run --rm --network host \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/kilted/setup.bash && timeout 5 ros2 node list"

echo ""

# Test 2: List topics
echo "2️⃣  Testing ros2 topic list..."
docker run --rm --network host \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/kilted/setup.bash && timeout 5 ros2 topic list"

echo ""

# Test 3: List services
echo "3️⃣  Testing ros2 service list..."
docker run --rm --network host \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/kilted/setup.bash && timeout 5 ros2 service list"

echo ""
echo "✅ Test completed!"
echo ""
echo "To start interactive GUI, run: ./start_insight_gui.sh"
