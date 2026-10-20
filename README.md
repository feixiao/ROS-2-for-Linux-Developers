# ROS 2 工业级实战手册：Linux 工程师快速入门指南

> **面向人群**：熟悉 Linux 系统编程、网络与运维（进程、Socket、多线程、POSIX 信号、CMake、Python、Docker），希望以最快速度切入 ROS 2（Robot Operating System 2）并进行机器人系统开发的工程师。

---

## 📖 手册目录导航

本手册拒绝泛泛的图形界面教学，直接以 Linux 工程师熟悉的底层系统视角建立心智模型，并辅以工业级全套代码工程与容器化实践：

- **[第 1 章：从 Linux 视角构建 ROS 2 心智模型](docs/01_linux_to_ros2_mental_model.md)**
  - Linux vs ROS 2 核心抽象对照表（进程、Socket、IPC、信号、网络）
  - 去中心化 DDS/RTPS 架构取代 ROS 1 roscore 集中式架构的本质
  - 单进程多节点（Composition）、执行器（Executor）与回调组并发模型
  - Linux 文件描述符与 DDS 底层端口映射关系

- **[第 2 章：工作空间与构建系统底层解构](docs/02_workspace_and_build_system.md)**
  - colcon 的本质：拓扑图驱动的多包元构建编排工具
  - Underlay 与 Overlay 机制（`setup.bash` 与 `AMENT_PREFIX_PATH` 环境变量剖析）
  - C++ `ament_cmake` 与 Python `ament_python` 标准规约
  - 提升研发效率的黄金实践：`--symlink-install` 与 `compile_commands.json`

- **[第 3 章：三大核心通信范式深度剖析：Topic、Service 与 Action](docs/03_communication_patterns.md)**
  - 选型决策矩阵：连续流 vs 原子 RPC vs 耗时可取消任务
  - 话题（Topic）与 QoS（服务质量）调优（Reliability、Durability、History）及不兼容陷阱
  - 服务（Service）在单线程事件循环中的自死锁反模式与异步解法
  - 动作（Action）状态机及其底层 3 个 Service + 2 个 Topic 通道解构
  - 自定义接口契约（ROSIDL 代码生成流水线、CDR 序列化与 Protobuf 对照）

- **[第 4 章：参数系统与多进程编排（Launch 系统）](docs/04_parameters_and_launch_system.md)**
  - 去中心化参数系统：显式声明、强类型校验与动态参数变更监听
  - 结构化 YAML 配置文件与命令行注入
  - 工业级 Python Launch 文件编写实践（条件启动、事件监听、异常重启）
  - 命名空间（Namespace）隔离与话题重映射（Remapping）
  - 组件化容器（ComposableNodeContainer）与生命周期管理（Lifecycle Nodes 状态机）
- **[第 5 章：高性能进阶与 DDS 网络中间件调优](docs/05_dds_and_high_performance_ipc.md)**
  - DDS 发现机制：SPDP 组播宣告与 `ROS_DOMAIN_ID` 端口数学公式
  - 中间件自由切换：Cyclone DDS vs Fast DDS
  - 极低延迟通信：Intra-Process 进程内零拷贝与 Iceoryx 跨进程共享内存（Loaned Messages）
  - 本地单机隔离 `ROS_LOCALHOST_ONLY=1` 与单播直连配置

- **[第 6 章：Linux 高手的调试与排障工具箱](docs/06_linux_debugging_and_ops_toolkit.md)**
  - ROS 2 核心 CLI 速查（node, topic, service, action, doctor）
  - 组播联通性排查：`ros2 multicast` 结合 `tcpdump` 抓包
  - 套接字与句柄探查：`ss -u -a -p` 与 `lsof -p <PID>`
  - 系统调用追踪：`strace` 定位死锁与 futex 阻塞
  - 崩溃事后复盘：Core Dump、GDB 回溯与 AddressSanitizer 内存检测

- **[第 7 章：容器化与生产环境交付（Docker & DevContainer）](docs/07_docker_and_deployment.md)**
  - 破解 Docker 默认 Bridge 隔离导致 DDS 组播发现失效的根因
  - 生产级多阶段构建（Multi-Stage Dockerfile），将运行时镜像压缩至极简
  - 硬件外设直通（激光雷达串口、USB 摄像头视频流与 GPU 权限）
  - Docker Compose 多节点编排与非 root 安全规范

- **[第 8 章：工业级生产硬化：实时性、安全性、硬件驱动与数据管线](docs/08_production_hardening_and_realtime.md)**
  - Linux PREEMPT_RT 实时内核调优、物理内存锁定（`mlockall`）与消除缺页异常
  - SROS 2：DDS-Security 规范、X.509 证书体系与 AES-GCM-256 链路加密
  - 硬件直连实践：Linux Udev 串口固化规则与 SocketCAN 汽车级总线接入
  - 高吞吐数据流录制：MCAP 追加写容器架构与 Zstandard 工业级无损压缩
---

## 🛠️ 配套工业级 Demo 工程

在 `demos/` 目录下，我们为你准备了结构规范、即拷即用的完整 ROS 2 工作空间：

```
demos/
├── custom_interfaces/   # 工业级 ROSIDL 接口工程 (自定义 .msg, .srv, .action)
├── cpp_composition_demo/# C++ 零拷贝组件化工程 (共享库 dlopen 动态加载)
├── cpp_topic_demo/      # C++ 高性能 Pub/Sub 工程 (含 QoS 与 Timer)
├── py_topic_demo/       # Python 全能工程 (Topic, Service, Action 三合一)
├── cpp_service_demo/    # C++ 双向 RPC 服务工程 (加法计算与错误处理)
├── cpp_action_demo/     # C++ 动作服务器与客户端 (进度流反馈与取消支持)
├── cpp_lifecycle_demo/  # C++ 托管生命周期工程 (LifecycleNode 确定性状态机)
├── launch_demo/         # 生产级 Python Launch 编排工程 (含 YAML 注入与重映射)
└── docker/              # 开箱即用的 Dockerfile 与 docker-compose.yml 容器化环境
```

---

## 🚀 5 分钟极速上手验证

### 方式 1：使用 Docker 一键运行（跨平台、零依赖）

无论你在 macOS、Windows WSL2 还是未安装 ROS 2 的 Linux 上，只需安装了 Docker：

```bash
# 1. 切换到 docker 目录
cd demos/docker

# 2. 一键启动容器集群
docker compose up --build
```
此时容器将自动构建并运行 `launch_demo`，控制台将实时输出节点间通信与日志。

### 方式 2：在 Ubuntu Linux 原生环境运行

若已安装 ROS 2 Humble / Jazzy：

```bash
# 1. 创建工作空间并软链接 demo 代码
mkdir -p ~/ros2_ws/src
cp -r demos/* ~/ros2_ws/src/
# 移除 docker 目录以保持包纯净
rm -rf ~/ros2_ws/src/docker

# 2. 进入工作空间并编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 3. 激活环境
source install/setup.bash

# 4. 运行 Launch 编排
ros2 launch launch_demo demo.launch.py
```

### 方式 3：使用 Makefile 自动化运维（Linux 开发者最爱）

本项目根目录提供了即拷即用的 `Makefile`，封装了日常高频操作：

```bash
make help          # 查看所有可用目标与说明
make build         # 一键调用 colcon 编译并导出 compile_commands.json
make test          # 自动化语法校验与完整性测试套件
make docker-build  # 构建生产级多阶段瘦身镜像
make docker-up     # 启动容器化集群
make clean         # 清理 build / install / log 缓存
```
