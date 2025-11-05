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
    && rm -rf /var/lib/apt/lists/*

# Set Fast DDS as default RMW implementation
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SHELL ["/bin/bash", "-c"]

WORKDIR /ws

# Copy source
COPY ros2_ws/src /ws/src
COPY live_server /ws/live_server

# Setup and build ROS2 packages
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build live_server (standalone, no ROS2)
RUN mkdir -p /ws/live_server/build && \
    cd /ws/live_server/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make && \
    make install && \
    rm -rf /ws/live_server/build

# Source on container start
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc && \
    echo "export PATH=/usr/local/bin:\$PATH" >> /root/.bashrc

# Expose TCP port for ui_proxy server (future step)
EXPOSE 9000

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && bash"]

