FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Set Fast DDS as default RMW implementation
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SHELL ["/bin/bash", "-c"]

WORKDIR /ws

# Copy source
COPY ros2_ws/src /ws/src

# Setup and build ROS2 packages
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source on container start
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc && \
    echo "export PATH=/usr/local/bin:\$PATH" >> /root/.bashrc

# Expose ports
# EXPOSE 9000  # ui_proxy TCP server
# EXPOSE 8765  # foxglove_bridge WebSocket server

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && bash"]

