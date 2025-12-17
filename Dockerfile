# ==============================================================================
# Insight GUI - ROS2 Visualization Tool
# ==============================================================================
FROM osrf/ros:kilted-desktop-full

# Install additional tools
RUN apt-get update && apt-get install -y \
    ros-kilted-rqt* \
    ros-kilted-rviz2 \
    ros-kilted-cyclonedds \
    ros-kilted-rmw-cyclonedds-cpp \
    python3-pip \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Set ROS2 environment
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Create config directory
RUN mkdir -p /config

# Copy CycloneDDS configuration
COPY cyclonedds.xml /config/cyclonedds.xml

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/kilted/setup.bash" >> /root/.bashrc

WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
