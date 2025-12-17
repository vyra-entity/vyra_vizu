# VYRA Visualization - Insight GUI

ROS2 visualization tools in a Docker container for monitoring and debugging VYRA modules.

## Quick Start

### Option 1: Interactive Shell (Recommended)
```bash
# Start interactive bash with all tools available
./start_insight_gui.sh

# Inside the container, test ROS2 connectivity:
ros2 node list
ros2 topic list
ros2 service list

# Then start any GUI tool:
rqt              # Main RQT interface
rqt_graph        # Node graph
rqt_console      # Log viewer
rviz2            # 3D visualization
```

### Option 2: Direct GUI Launch
```bash
# Launch specific GUI tools directly:
./start_rqt.sh         # Main RQT interface
./start_rqt_graph.sh   # Node/Topic graph
./start_rviz2.sh       # 3D visualization
```

### Option 3: Test Connectivity (No GUI)
```bash
# Quick connectivity test
./test_connectivity.sh
```

## Available Tools

### Command Line Tools
- `ros2 node list` - List all ROS2 nodes
- `ros2 topic list` - List all topics  
- `ros2 service list` - List all services
- `ros2 topic echo <topic>` - Monitor topic messages
- `ros2 service call <service> <type> <args>` - Call a service

### GUI Tools
- `rqt` - Main RQT interface (plugin-based GUI)
- `rqt_graph` - Visualize node and topic connections
- `rqt_console` - View and filter ROS2 logs
- `rqt_plot` - Plot numeric topic data
- `rqt_topic` - Topic monitor and publisher
- `rviz2` - 3D visualization (for robot models, point clouds, etc.)

## Network Configuration

- **ROS_DOMAIN_ID**: 42 (matches docker-compose services)
- **RMW_IMPLEMENTATION**: rmw_cyclonedds_cpp
- **Network**: Host network mode (required for DDS multicast)

## Examples

### Example 1: Monitor Module Manager Topics
```bash
# Start interactive shell
./start_insight_gui.sh

# List all topics
ros2 topic list

# Monitor state feed
ros2 topic echo /v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/speaker/StateFeeder

# Monitor news feed  
ros2 topic echo /v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/speaker/NewsFeeder
```

### Example 2: Call Services
```bash
# Get module status
ros2 service call /v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/callable/get_status vyra_module_interfaces/srv/MMGetStatus

# Get registered modules
ros2 service call /v2_modulemanager_733256b82d6b48a48bc52b5ec73ebfff/callable/get_registered_modules vyra_module_interfaces/srv/MMGetRegisteredModules
```

### Example 3: Visualize System Graph
```bash
# Launch RQT Graph directly
./start_rqt_graph.sh

# Or from interactive shell:
./start_insight_gui.sh
# Then run: rqt_graph
```

## Troubleshooting

### Can't see ROS2 nodes/topics
1. Check ROS_DOMAIN_ID: `echo $ROS_DOMAIN_ID` (should be 42)
2. Verify services are running: `docker ps | grep v2_modulemanager`
3. Check network: Script uses host network mode

### X11 display issues
1. Ensure DISPLAY is set: `echo $DISPLAY`
2. Check xhost: `xhost +local:docker`
3. Try restarting X server

### GUI doesn't open
1. Make sure you're running on a machine with X11 display
2. For remote connections, use X11 forwarding: `ssh -X user@host`
3. Check if container is running: `docker ps | grep vyra_insight`

### Performance issues
- Close unnecessary rqt plugins
- Reduce topic update rates
- Use `ros2 topic hz <topic>` to check message rates
