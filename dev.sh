#!/bin/bash

# 获取脚本所在目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 容器配置
CONTAINER_NAME="lingmou-dev"
IMAGE_NAME="lingmou-jazzy:dev"

# 检查容器是否存在
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "容器 ${CONTAINER_NAME} 已存在，进入容器..."
    
    # 检查容器是否正在运行
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "启动容器..."
        docker start ${CONTAINER_NAME}
    fi
    
    # 进入容器
    docker exec -it ${CONTAINER_NAME} /bin/bash
else
    echo "容器 ${CONTAINER_NAME} 不存在，创建并启动容器..."
    
    # 检查镜像是否存在
    if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
        echo "镜像 ${IMAGE_NAME} 不存在，请先构建镜像："
        echo "  cd ${SCRIPT_DIR} && docker build -t ${IMAGE_NAME} ."
        exit 1
    fi
    
    # 创建并启动容器
    docker run -it \
        --name ${CONTAINER_NAME} \
        --net=host \
        -v "${SCRIPT_DIR}:/workspace" \
        -w /workspace \
        ${IMAGE_NAME}
fi

