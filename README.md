# 灵眸Demo

基于 ROS2 Fast DDS 的最小化智能驾驶控制消息转发 demo，使用 C++ 实现。

## 架构

- **control_module**: ROS2 节点，10Hz 周期性发布 `/control_cmd` 话题（Control 消息）和自定义可视化消息
- **marker_converter**: ROS2 节点，将自定义消息转换为 `visualization_msgs/Marker`，供 Foxglove 自动可视化
- **ui_proxy**: ROS2 节点，订阅 `/control_cmd`，同时作为 TCP Server（端口 9000）转发 JSON Lines 格式消息
- **xviz_converter**: ROS2 节点，基于 Boost Beast 的 WebSocket Server（端口 8080），订阅 `/control_cmd` 并转发 JSON 格式消息
- **foxglove_bridge**: ROS2 节点，提供 WebSocket 服务器（端口 8765），连接 Foxglove Studio 与 ROS2 系统

## 构建

```bash
docker build -t lingmou-jazzy:dev .
```

## 运行

使用开发脚本进入容器（自动挂载代码目录）：

```bash
./dev.sh
```

首次进入容器后，需要编译项目：

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

后续在容器内的终端中分别运行：

**终端 1 - 发布者（自定义消息）：**
```bash
ros2 run control_module control_publisher
```

**终端 2 - Marker 转换器（可选，用于 Foxglove 可视化）：**
```bash
ros2 run control_module marker_converter
```

**终端 3 - Foxglove Bridge（WebSocket Server，用于 Foxglove 连接）：**
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

**终端 4 - UI Proxy（TCP Server，可选）：**
```bash
ros2 run ui_proxy ui_proxy_node
```

**终端 5 - XVIZ Converter（WebSocket Server，可选）：**
```bash
ros2 run xviz_converter xviz_converter_node
```

> 注意：打开新终端需要再次运行 `./dev.sh` 进入容器

### 连接 XVIZ Converter（WebSocket）

XVIZ Converter 监听端口 8080，客户端连接后会实时接收 JSON 格式的 control 消息：

```bash
# 使用 websocat 测试连接
websocat ws://localhost:8080
```

JSON 消息格式：
```json
{"timestamp":{"sec":123,"nanosec":456},"speed_mps":5.0,"steering_deg":15.0,"gear":1}
```

### 连接 Foxglove Studio（WebSocket）

1. 确保已启动 `foxglove_bridge`（终端 3）
2. 打开 [Foxglove Studio](https://studio.foxglove.dev/)
3. 选择 "Open connection" → "Foxglove WebSocket"
4. 输入连接地址：`ws://localhost:8765`
5. 点击 "Open" 即可实时查看 ROS2 话题数据

## 录制数据（Foxglove 回放）

### 同时录制原始消息和转换后的 Marker 消息

**推荐方式：同时录制所有可视化相关话题**

```bash
# 在容器内，确保已启动 control_publisher 和 marker_converter
# 然后在另一个终端录制：

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

#### 方式 1: WebSocket 实时连接（推荐）

1. 启动 `foxglove_bridge`（见上方运行说明）
2. 打开 [Foxglove Studio](https://studio.foxglove.dev/)
3. 选择 "Open connection" → "Foxglove WebSocket"
4. 输入 `ws://localhost:8765`
5. 实时查看和可视化 ROS2 话题数据

#### 方式 2: 回放录制的 bag 文件

1. 打开 [Foxglove Studio](https://studio.foxglove.dev/)
2. 选择 "Open file"
3. 选择录制的 bag 文件（MCAP 或 SQLite3 格式都支持）
4. 在 Foxglove 中：
   - 订阅 `/visualization/marker` 和 `/visualization/marker_array` 进行 3D 可视化
   - 订阅原始自定义消息（`/visualization/box` 等）查看原始数据
   - 使用 "Raw Messages" 面板查看自定义消息的详细字段
   - 使用 "3D" 面板查看 Marker 的 3D 渲染

## 技术栈

- ROS2 Jazzy
- Fast DDS (RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
- Docker (ros:jazzy-ros-base)
- rosbag2 (数据录制)
- Boost Beast (WebSocket 服务器)
- foxglove_bridge (WebSocket 连接 Foxglove Studio)