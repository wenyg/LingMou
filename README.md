# 灵眸Demo

基于 ROS2 Fast DDS 的最小化智能驾驶控制消息转发 demo，使用 C++ 实现。

## 架构

- **control_module**: ROS2 节点，10Hz 周期性发布 `/control_cmd` 话题（Control 消息）
- **ui_proxy**: ROS2 节点，订阅 `/control_cmd`，同时作为 TCP Server（端口 9000）转发 JSON Lines 格式消息
- **live_server**: 独立 TCP 客户端程序，连接到 ui_proxy，接收并打印 Control 消息

## 构建

```bash
docker build -t lingmou-jazzy:dev .
```

## 运行

```bash
docker run --rm -it --net=host lingmou-jazzy:dev
```

在容器内的三个终端中分别运行：

**终端 1 - 发布者：**
```bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 run control_module control_publisher
```

**终端 2 - UI Proxy（TCP Server）：**
```bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 run ui_proxy ui_proxy_node
```

**终端 3 - Live Server（TCP Client）：**
```bash
live_server localhost 9000
```

## 技术栈

- ROS2 Jazzy
- Fast DDS (RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
- Docker (ros:jazzy-ros-base)