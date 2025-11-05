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

在容器内的终端中分别运行：

**终端 1 - 发布者（自定义消息）：**
```bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 run control_module control_publisher
```

**终端 2 - Marker 转换器（可选，用于 Foxglove 可视化）：**
```bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 run control_module marker_converter
```

**终端 3 - UI Proxy（TCP Server，可选）：**
```bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 run ui_proxy ui_proxy_node
```

**终端 4 - Live Server（TCP Client，可选）：**
```bash
live_server localhost 9000
```

## 录制数据（Foxglove 回放）

### 同时录制原始消息和转换后的 Marker 消息

**推荐方式：同时录制所有可视化相关话题**

```bash
# 在容器内，确保已启动 control_publisher 和 marker_converter
# 然后在另一个终端录制：
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash

# 录制所有原始自定义消息 + 转换后的 Marker 消息
ros2 bag record \
  /control_cmd \
  /visualization/box \
  /visualization/box_array \
  /visualization/point_array \
  /visualization/line \
  /visualization/marker \
  /visualization/marker_array \
  --storage mcap \
  -o ./bags/complete_data

# 或者只录制原始消息（用于数据分析）
ros2 bag record \
  /control_cmd \
  /visualization/box \
  /visualization/box_array \
  /visualization/point_array \
  /visualization/line \
  --storage mcap \
  -o ./bags/raw_data

# 或者只录制 Marker 消息（用于 Foxglove 可视化）
ros2 bag record \
  /control_cmd \
  /visualization/marker \
  /visualization/marker_array \
  --storage mcap \
  -o ./bags/marker_data
```

### 录制格式选项

**MCAP 格式（推荐，Foxglove 原生支持）：**
```bash
ros2 bag record <topics> --storage mcap -o ./bags/data_name
```

**SQLite3 格式（默认）：**
```bash
ros2 bag record <topics> -o ./bags/data_name
```

### 回放数据

```bash
# 回放 bag 文件
ros2 bag play ./bags/complete_data

# 回放时只发布部分话题（例如只回放 Marker）
ros2 bag play ./bags/complete_data --topics /visualization/marker /visualization/marker_array
```

### 在 Foxglove 中使用

1. 打开 [Foxglove Studio](https://studio.foxglove.dev/)
2. 选择 "Open file" 或 "Open connection"
3. 选择录制的 bag 文件（MCAP 或 SQLite3 格式都支持）
4. 在 Foxglove 中：
   - 订阅 `/visualization/marker` 和 `/visualization/marker_array` 进行 3D 可视化
   - 订阅原始自定义消息（`/visualization/box` 等）查看原始数据
   - 使用 "Raw Messages" 面板查看自定义消息的详细字段

## 技术栈

- ROS2 Jazzy
- Fast DDS (RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
- Docker (ros:jazzy-ros-base)
- rosbag2 (数据录制)