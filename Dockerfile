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

WORKDIR /workspace

# Source ROS2 on container start
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "export PATH=/usr/local/bin:\$PATH" >> /root/.bashrc

CMD ["/bin/bash"]

