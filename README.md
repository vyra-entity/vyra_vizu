# VYRA Visualization - Insight GUI

ROS2 visualization tools in a Docker container for monitoring and debugging VYRA modules.

## Quick Start

```bash
# Build and start Insight GUI
./start_insight_gui.sh

# Inside the container, test ROS2 connectivity:
ros2 node list
ros2 topic list
ros2 service list
```

## Available Tools

### Command Line Tools
- `ros2 node list` - List all ROS2 nodes
- `ros2 topic list` - List all topics
- `ros2 service list` - List all services
- `ros2 topic echo <topic>` - Monitor topic messages
- `ros2 service call <service> <type> <args>` - Call a service

### GUI Tools
- `rqt` - Main RQT interface (plugin-based)
- `rqt_graph` - Visualize node and topic connections
- `rqt_console` - View and filter ROS2 logs
- `rqt_plot` - Plot numeric topic data
- `rviz2` - 3D visualization (for robot models, point clouds, etc.)

## Network Configuration

- **ROS_DOMAIN_ID**: 42 (matches docker-compose services)
- **RMW_IMPLEMENTATION**: rmw_cyclonedds_cpp
- **Network**: Uses Docker vyra-network or bridge

## Troubleshooting

### Can't see ROS2 nodes/topics
1. Check ROS_DOMAIN_ID matches: `echo $ROS_DOMAIN_ID` (should be 42)
2. Verify network connectivity: `docker network ls | grep vyra`
3. Check if services are running: `docker ps | grep v2_modulemanager`

### X11 display issues
1. Ensure X11 forwarding is enabled: `echo $DISPLAY`
2. Check xhost permissions: `xhost +local:docker`
3. Try restarting the container

### Network isolation
If using Docker Swarm overlay network, DDS multicast may not work properly.
Consider using host network mode for development:
```bash
# Edit start_insight_gui.sh to use:
--network host
```

## Examples

### Monitor Module Manager Status
```bash
ros2 topic echo /v2_modulemanager/status
```

### Call Health Check Service
```bash
ros2 service call /v2_modulemanager/health_check std_srvs/srv/Trigger
```

### View System Graph
```bash
rqt_graph
```
