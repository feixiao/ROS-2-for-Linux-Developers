# 第 7 章：容器化与生产环境交付（Docker & DevContainer）

在现代软件工程中，环境一致性是第一生产力。
由于 ROS 2 的发行版与 Ubuntu LTS 版本存在强绑定关系（例如：ROS 2 **Humble** 官方绑定 **Ubuntu 22.04**，ROS 2 **Jazzy** 绑定 **Ubuntu 24.04**），对于使用 Arch Linux、Fedora、macOS 或 Windows WSL2 的开发者，或者需要在云端 Kubernetes/边缘嵌入式设备（如 Jetson）上部署时，**Docker 容器化是业界的绝对标准**。

本章详细拆解生产级多阶段 Dockerfile 构建、网络通信穿透与设备挂载方案。

---

## 7.1 为什么标准 Docker 桥接网络中 ROS 2 会失效？

这是 Linux 后端工程师初次将 ROS 2 放入容器时最常遇到的坑：
**容器内的节点无法与宿主机或其他容器的节点互相发现。**

### 根因分析：
1. **DDS 组播发现机制**：默认情况下，DDS 在 `239.255.0.1` 组播地址广播。
2. **Docker Bridge NAT 隔离**：Docker 默认的 `bridge` 网络在宿主机上创建了一个虚拟网桥（`docker0`），通过 iptables 进行网络地址转换（SNAT/DNAT）。
3. 默认网桥通常**不转发 IGMP 组播数据包**，导致容器内的 SPDP 组播宣告根本出不去，外部的广播也进不来。

### 生产级解决方案：
1. **方案 A（最简单可靠，推荐）：使用 Host 网络模式**
   ```bash
   docker run -it --rm --net=host --ipc=host ros2-app:latest
   ```
   - `--net=host`：容器直接复用宿主机的网络命名空间，无 NAT 开销，组播与端口完全透传。
   - `--ipc=host`：容器共享宿主机的 `/dev/shm` 共享内存命名空间，保证跨容器与宿主机之间的 **零拷贝（Iceoryx / FastDDS SHM）** 正常生效。

2. **方案 B：配置 CycloneDDS 单播路由（云端/Kubernetes Pod 跨主机场景）**
   在容器内挂载前文介绍的 `cyclonedds.xml`，显式指定对端 Pod 的 IP，彻底避开组播依赖。

---

## 7.2 生产级多阶段构建（Multi-Stage Dockerfile）

直接在单一镜像中安装编译工具会导致镜像体积暴增至 3GB~5GB，且包含了大量头文件、静态库与编译器，存在安全漏洞与体积浪费。

采用多阶段构建，将最终运行时镜像压缩至极简尺寸：

```dockerfile
# ==========================================
# 阶段 1: 编译构建阶段 (Builder Stage)
# ==========================================
FROM ros:humble-ros-base AS builder

# 安装构建依赖与编译器
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/

# 编译所有包 (Release 模式并优化)
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --install-base /opt/ros2_app

# ==========================================
# 阶段 2: 纯净运行时镜像 (Runtime Stage)
# ==========================================
FROM ros:humble-ros-core AS runtime

# 设置非 root 用户 (保障容器生产安全)
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG dialout,video $USERNAME

# 仅从 Builder 阶段复制编译好的二进制产物
COPY --from=builder /opt/ros2_app /opt/ros2_app

# 编写入口脚本
COPY <<-"EOF" /ros_entrypoint.sh
#!/usr/bin/env bash
set -e
source "/opt/ros/humble/setup.bash"
source "/opt/ros2_app/setup.bash"
exec "$@"
EOF

RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
WORKDIR /home/$USERNAME

CMD ["ros2", "launch", "launch_demo", "demo.launch.py"]
```

---

## 7.3 硬件外设直通（串口、雷达、USB 摄像头）

机器人系统必然涉及硬件设备的访问。在运行 Docker 容器时，需将宿主机设备节点挂载进容器：

```bash
docker run -it --rm \
  --net=host \
  --ipc=host \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \       # 串口激光雷达 / 惯导
  --device=/dev/video0:/dev/video0 \         # USB 工业相机
  --group-add dialout \                      # 注入串口访问权限
  --group-add video \                        # 注入视频捕获权限
  my_ros2_robot:latest
```

---

## 7.4 Docker Compose 编排多节点集群

在边缘计算设备上，通常使用 `docker-compose.yml` 统一管理感知、规划与底盘服务：

```yaml
version: '3.8'

services:
  # 传感器驱动服务
  sensor_driver:
    image: ros2_robot_app:latest
    network_mode: host
    ipc: host
    privileged: true
    devices:
      - "/dev/ttyUSB0:/dev/ttyUSB0"
    command: ros2 launch launch_demo demo.launch.py
    restart: unless-stopped

  # 规划决策算法服务
  decision_planner:
    image: ros2_robot_app:latest
    network_mode: host
    ipc: host
    environment:
      - ROS_DOMAIN_ID=42
    command: ros2 run py_topic_demo py_listener
    depends_on:
      - sensor_driver
    restart: unless-stopped
```

通过简单的 `docker compose up -d` 即可在任何安装了 Docker 的 Linux 主机上一键拉起完整的机器人系统。
