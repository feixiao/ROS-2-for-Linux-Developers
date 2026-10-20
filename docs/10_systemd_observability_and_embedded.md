# 第 10 章：集群运维、生产守护进程、Prometheus 可观测性与嵌入式 Linux

在机器人或自动化车队进入量产（Fleet Deployment）后，开发者的关注点将从“单机算法调试”彻底跃迁至**系统工程化运维**：
- 机器人开机如何实现秒级、确定性自启？崩溃后如何由内核看门狗接管自愈？
- 如何对分布在不同机房或室外运行的成百上千台机器人进行指标遥测（Telemetry）与报警监控？
- 当计算平台是定制化嵌入式单板计算机（如 NVIDIA Jetson、树莓派或工业级 ARM64 主板）时，如何摆脱厚重的通用系统，进行轻量级交叉编译（Cross-Compilation）与 Yocto 镜像裁剪？

本章为 Linux 运维架构师与嵌入式工程师提供完整的生产级落地方案。

---

## 10.1 生产级守护进程与看门狗（Systemd & Watchdog）

在 Linux 生产环境中，绝对严禁使用 `rc.local`、`nohup &` 或 `@reboot` crontab 启动机器人核心节点（缺乏依赖拓扑编排、无法捕获进程退出信号、无法做资源配额限制）。

**Systemd 是 Linux 服务管理的绝对事实标准。**

### 1. 生产级 Unit 配置文件解构 (`/etc/systemd/system/ros2-robot.service`)

```ini
[Unit]
Description=ROS 2 Autonomous Robot Core Pipeline
Documentation=https://github.com/frank/ros2_for_linux
# 严格等待网络接口连通与系统时钟同步 (DDS 数据包时间戳严格依赖单调时钟与精确授时)
After=network.target network-online.target time-sync.target
Wants=network-online.target time-sync.target

[Service]
Type=simple
User=rosdev
Group=rosdev
WorkingDirectory=/home/rosdev/ros2_ws

# 环境变量隔离注入 (避免非交互式 shell 丢失环境配置)
Environment="ROS_DOMAIN_ID=42"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="PYTHONUNBUFFERED=1"
Environment="ROS_LOG_DIR=/var/log/ros2"

# 优雅拉起守护脚本
ExecStart=/bin/bash -c '\
  source /opt/ros/humble/setup.bash && \
  source /home/rosdev/ros2_ws/install/setup.bash && \
  exec ros2 launch launch_demo demo.launch.py'

# 崩溃自愈策略: 异常退出时在 3 秒后自动重启
Restart=on-failure
RestartSec=3s
TimeoutStopSec=10s

# 硬件与软件看门狗 (Watchdog) 集成: 若 15 秒内未喂狗则直接强制杀进程重启
WatchdogSec=15s

# Linux Cgroups v2 资源配额限制 (防止算法节点内存泄漏导致整机死机)
MemoryHigh=3500M
MemoryMax=4096M
CPUQuota=300%

# 统一接入 Linux journal 日志系统
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ros2-robot-core

[Install]
WantedBy=multi-user.target
```

### 2. 服务生命周期命令
```bash
# 1. 注册并开机自启
sudo cp scripts/ros2-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ros2-robot.service

# 2. 启动与状态检查
sudo systemctl start ros2-robot.service
sudo systemctl status ros2-robot.service

# 3. 实时跟踪日志流 (类似 ros2 run 的实时输出)
journalctl -u ros2-robot.service -f -o cat
```

---

## 10.2 机器人群体可观测性：Prometheus 与 Grafana

当管理 50 台仓储 AGV 时，不可能挨个通过 SSH 登录节点去执行 `ros2 topic hz`。
业界标准的解决方案是：**为 ROS 2 节点接入 Prometheus Metrics Exporter**，将机器人内部的通信指标暴露为标准的 HTTP 抓取端点。

### 1. Prometheus 监控架构
```
+------------------+         HTTP GET /metrics         +-------------------+
|  ROS 2 机器人节点  | <-------------------------------- | Prometheus Server |
| (:9100 Exporter) |                                   +-------------------+
+------------------+                                             |
         ^                                                       v
         | 订阅话题并统计 Hz / 延迟                        +-------------------+
+------------------+                                   | Grafana Dashboard |
| 激光雷达 / 轮速计 |                                   |  (实时报警与大屏)  |
+------------------+                                   +-------------------+
```

### 2. 核心监控指标定义（OpenMetrics 规范）
1. `ros2_messages_total{topic="/chatter"}` (Counter)：累计接收消息总数。
2. `ros2_topic_frequency_hz{topic="/chatter"}` (Gauge)：实时发布频率。
3. `ros2_node_uptime_seconds` (Gauge)：节点连续稳定运行秒数。

### 3. Prometheus 抓取配置示例 (`prometheus.yml`)
```yaml
scrape_configs:
  - job_name: 'ros2_robot_fleet'
    scrape_interval: 2s
    static_configs:
      - targets: ['192.168.1.101:9100', '192.168.1.102:9100']
        labels:
          robot_id: 'agv_alpha_01'
```

### 4. 报警规则设定（Alertmanager）
在 `alerting_rules.yml` 中配置阈值报警：
```yaml
groups:
  - name: robot_safety_alerts
    rules:
      - alert: LidarFrequencyLow
        expr: ros2_topic_frequency_hz{topic="/sensor/lidar"} < 8.0
        for: 5s
        labels:
          severity: critical
        annotations:
          summary: "激光雷达帧率过低，可能存在硬件松脱或驱动卡死！"
```

### 5. 云边协同分布式链路追踪：OpenTelemetry (OTel)
在大型自主移动机器人（AMR）车队与云端协同调度系统中，一个工单指令从云端下发、经由 5G 网关到达边缘底盘、驱动电机执行并最终上报状态，整条链路跨越了多个网络与语言实体。

通过引入 **OpenTelemetry**：
+- **Trace 上下文透传**：在自定义 ROS 2 消息 Header 中注入 `traceparent` 上下文（含 128 位 Trace ID 与 64 位 Span ID）。
+- **无侵入分布式追踪**：下游节点在接收到话题数据后提取上下文创建子 Span，并通过标准 OTLP/gRPC 协议统一异步上报至 **Jaeger** 或 **Grafana Tempo**。
+- 这使得系统架构师能够像排查微服务链路一样，精准洞察网络往返、DDS 队列与算法处理的端到端耗时拆解。

---

## 10.3 嵌入式 Linux 与交叉编译（Yocto / BitBake / aarch64）

对于工业级车载控制器或低功耗边缘端设备，通用 Ubuntu 桌面版过于臃肿（耗费 4GB+ 存储与数百个无用后台服务）。
工程师通常采用 **Yocto Project** 定制专用 Linux 发行版。

### 1. Yocto Project 与 `meta-ros` 架构
- **Layer 组织**：在 Yocto `bblayers.conf` 中追加 `meta-ros/meta-ros2-humble`。
- **构建配方（BitBake Recipes）**：`meta-ros` 将每一个 ROS 2 核心包（`rclcpp`、`rmw_cyclonedds_cpp`、`std_msgs`）转化为标准的 BitBake 配方，在宿主机上一次性交叉编译为 ARM64 ELF 目标二进制。

### 2. 宿主机快速交叉编译（`colcon-cross-compile`）
若不使用 Yocto，可在 x86_64 Ubuntu 开发机上通过目标板 Sysroot 进行快速交叉构建：

```bash
# 1. 准备 aarch64 交叉编译工具链
sudo apt-get install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 2. 编写 aarch64-toolchain.cmake
cat <<-"EOF" > aarch64-toolchain.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH /path/to/target_sysroot)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
EOF

# 3. 执行跨架构编译
colcon build \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$(pwd)/aarch64-toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
```
编译产物可直接打包通过 `rsync` 同步至目标板 `/opt/ros2_app` 运行。

### 3. Yocto meta-ros 生产级 BitBake 配方模板
在量产车载与工业整机中，功能包通常被编写为 BitBake Recipe（`.bb` 文件），直接打包进只读 rootfs 固件：

```bitbake
SUMMARY = "C++ Topic Demo Package for Autonomous Robot"
SECTION = "devel"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://package.xml;beginline=8;endline=8;md5=3dce4ba60d7e51ec64f3c3dc186716e0"

SRC_URI = "git://github.com/myorg/cpp_topic_demo.git;protocol=https;branch=main"
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/git"

# 核心：继承 Yocto 的 ament 构建类
inherit ros_ament_cmake

# 声明构建与运行期依赖
ROS_BUILD_DEPENDS = " \
    rclcpp \
    std-msgs \
"

ROS_EXEC_DEPENDS = " \
    rclcpp \
    std-msgs \
"

FILES:${PN} += "${datadir}/cpp_topic_demo"
```
通过 `bitbake core-image-robot`，Yocto 自动完成跨架构交叉编译、strip 剥离符号、剔除无用依赖并生成最小化的生产 squashfs 镜像。

---

## 10.4 结构化日志收集与中心化聚合（Fluent Bit + Grafana Loki）

成百上千台机器人在工厂车间运行时，发生段错误或规划失败后如何追溯排查？
传统的文本日志零散存放在每台机器的 `/var/log/ros2/` 目录下，难以检索。

Linux 现代化日志收集方案采用 **Fluent Bit** 轻量级守护进程，从 `journald` 读取日志并流式推送到云端 **Grafana Loki**：

```
+----------------------+          Fluent Bit          +------------------+
| Linux systemd-journal| ===> [Regex Parser Filter] ==>| Grafana Loki     |
| (ros2-robot.service) |      提取 node/severity/msg  | (云端日志中心)    |
+----------------------+                              +------------------+
```

### Fluent Bit 解析配置 (`/etc/fluent-bit/fluent-bit.conf`)
```ini
[INPUT]
    Name            systemd
    Tag             robot.ros2.*
    Systemd_Filter  _SYSTEMD_UNIT=ros2-robot.service
    Read_From_Tail  On

[FILTER]
    Name            parser
    Match           robot.ros2.*
    Key_Name        MESSAGE
    Parser          ros2_log_parser

[OUTPUT]
    Name            loki
    Match           *
    Host            loki.ops.internal
    Port            3100
    Labels          robot_id=agv_alpha_01, environment=production
```
开发人员可直接在 Grafana Web 界面通过 LogQL（如 `{robot_id="agv_alpha_01"} |= "ERROR"`）秒级检索异常堆栈。

---

## 10.5 生产级 OTA 双分区无损固件升级（A/B Partition with RAUC）

在嵌入式 Linux 设备上，固件远程升级（Over-The-Air, OTA）必须具备**断电保护与自动回滚机制**：

```
+---------------------------------------------------------------+
|               Linux eMMC 存储双分区布局 (A/B)                  |
---------------------------------------------------------------+
| Partition 1: Kernel A + Rootfs A (Active 当前运行分区)        |
---------------------------------------------------------------+
| Partition 2: Kernel B + Rootfs B (Standby 待升级分区)         |
---------------------------------------------------------------+
| Partition 3: /data (持久化日志、高精地图、校准配置数据)       |
---------------------------------------------------------------+
```
- **升级流程**：云端推送签名 RAUC Bundle 镜像，静默解压写入未激活的分区 B。
- **看门狗联锁验证**：U-Boot 引导切换至分区 B 启动。若 `ros2-robot.service` 成功拉起且在 60 秒内正常喂狗，标记分区 B 为健康（`good`）；若发生段错误或服务崩溃导致超时未喂狗，硬件看门狗强制重启，U-Boot 自动回退（Rollback）至已知完好的分区 A。
- 这彻底避免了因网络中断或程序 Bug 导致机器人设备“变砖”的致命工程事故。

---

## 10.6 自动化集群健康体检（Health Check 脚本）

在机器人出厂或每日开机自检时，运行本项目提供的诊断工具 `scripts/robot_health_check.sh`：

```bash
bash scripts/robot_health_check.sh
```

体检内容涵盖：
- Linux PREEMPT_RT 实时内核验证与 Swap 内存页置换状态；
- `net.core.rmem_max` 套接字缓冲区与 `/proc/net/snmp` 内核丢包计数；
- 组播路由 `224.0.0.0/4` 与 `ROS_DOMAIN_ID` 碰撞检查；
- 硬件 Udev 串口固化与 SocketCAN 设备存在性检测；
- Systemd 核心服务存活状态。

---

## 10.7 本章总结

- 摒弃脆弱的手工拉进程方式，使用 **Systemd + Watchdog + Cgroups** 打造工业级无人值守自愈守护进程。
- 引入 **Prometheus + Grafana + Fluent Bit**，使机器人集群具备像现代微服务一样透明的可观测性、指标大屏与日志聚合。
- 面向嵌入式边缘设备，掌握 **Yocto / 交叉编译** 与 **A/B RAUC OTA** 技术，打造高可靠、可自我修复的量产级机器人固件。
