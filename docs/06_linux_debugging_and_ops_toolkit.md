# 第 6 章：Linux 高手的调试与排障工具箱

对于具备 Linux 运维和系统编程背景的工程师，在遇到 ROS 2 节点崩溃、延迟异常飙高、节点无法发现或数据丢包时，切忌像初学者一样盲目猜测。

本章将 **ROS 2 语义化 CLI 工具** 与 **Linux 原生系统级追踪工具（tcpdump / strace / gdb / perf / lsof）** 无缝结合，构建一套坚实的排障方法论。

---

## 6.1 ROS 2 核心 CLI 工具速查

ROS 2 的命令行工具遵循模块化子命令结构：

```bash
# 1. 节点与拓扑探查
ros2 node list                          # 列出系统中所有活跃节点
ros2 node info /my_node                 # 深入查看某节点的发布者、订阅者、服务与 Action

# 2. 话题与实时数据流
ros2 topic list -t                      # 列出话题及对应的消息类型 (如 std_msgs/msg/String)
ros2 topic hz /sensor/lidar             # 测量话题发布频率 (计算均值与抖动)
ros2 topic bw /camera/image_raw         # 测量话题带宽占用 (MB/s)
ros2 topic echo /my_topic               # 实时打印数据包 (支持 --field, --truncate-length)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10 # 10Hz 模拟注入

# 3. 服务与动作交互
ros2 service list                       # 列出所有 RPC 服务
ros2 service call /reset_odom std_srvs/srv/Trigger "{}" # 发起单次请求
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}" --feedback

# 4. 全局健康检查与环境诊断
ros2 doctor                             # 检查环境配置、网络、平台版本告警
ros2 doctor --report                    # 打印详尽的环境变量与 RMW 中间件诊断报告
```

---

## 6.2 组播联通性排查：`ros2 multicast` 与 `tcpdump`

**故障表现**：两台 Linux 机器处于同一个局域网，`ROS_DOMAIN_ID` 相同，但彼此无法发现节点。

### Step 1: 使用 ROS 2 内置组播测试器
在机器 A 上监听组播：
```bash
ros2 multicast receive
```
在机器 B 上发送组播：
```bash
ros2 multicast send
```
- 如果机器 A 输出了 `Received from ...`，说明底层组播通信正常。
- 如果毫无响应，说明网络交换机开启了 IGMP Snooping 屏蔽了组播，或者 Linux 本机防火墙拦截了 UDP 报文。

### Step 2: 使用 `tcpdump` 抓包定位
作为 Linux 工程师，直接抓取 7400 组播发现端口：
```bash
sudo tcpdump -i eth0 udp port 7400 -nn -X
```
若能看到源 IP 发送的 RTPS 报文，但 ROS 2 无法处理，检查 Linux 防火墙策略：
```bash
sudo ufw status
# 允许 ROS 2 专属端口区间 (Domain ID 0-100)
sudo ufw allow 7400:7900/udp
```

---

## 6.3 网络与文件描述符探查：`ss` 与 `lsof`

使用 `ss` 探查当前节点打开的 UDP 单播与组播监听套接字：
```bash
# 查看所有绑定在 UDP 的 ROS 2 相关进程
ss -u -a -p | grep -E "7400|7410|7411"
```

使用 `lsof` 深入观察节点的句柄泄漏：
```bash
# 查看节点 PID 占用的所有文件描述符
lsof -p $(pgrep -f "cpp_talker")
# 重点检查: 
# - /dev/shm (共享内存段是否正常映射)
# - anon_inode:[eventpoll] (epoll 事件分发器)
# - socket (UDP 套接字连接)
```

---

## 6.4 系统调用级追踪：`strace`

当节点出现假死（卡在某个系统调用）、死锁或不可预期的阻塞时，`strace` 是终极利器：

```bash
# 追踪某个正在运行的 ROS 2 节点进程
sudo strace -f -e trace=network,ipc,futex -t -p $(pgrep -f "my_buggy_node")
```

### 典型输出诊断：
1. **持续陷入 `futex(..., FUTEX_WAIT_PRIVATE, ...)`**：
   - 说明程序发生了多线程死锁，或者单线程执行器中调用了阻塞的服务客户端。
2. **高频连续调用 `recvmsg` / `sendmsg`**：
   - 证明底层 DDS 正在疯狂收发网络包，可配合数据量判断是否存在流量风暴。
3. **阻塞在 `poll` 或 `epoll_wait` 且无超时**：
   - 进程正处于空闲等待事件状态，若预期有数据但仍在此阻塞，说明网络数据根本没到达内核套接字缓冲区。

---

## 6.5 崩溃与核心转储分析：GDB & Core Dump

C++ ROS 2 节点遭遇指针越界或段错误（Segmentation fault）时，在 Linux 上进行事后追查的标准流程：

### 1. 开启系统 Core Dump 收集
```bash
ulimit -c unlimited
# 确保内核将转储文件存放在指定目录
sudo sysctl -w kernel.core_pattern=/tmp/core-%e.%p.%t
```

### 2. 使用 GDB 复盘崩溃调用栈
```bash
# 载入编译出的 debug ELF 二进制与 core 转储文件
gdb ./install/my_pkg/lib/my_pkg/my_node /tmp/core-my_node.12345.1678900
```
在 GDB 交互界面中：
```gdb
(gdb) bt full       # 查看崩溃点的完整调用栈与局部变量
(gdb) info threads  # 查看所有工作线程与 Executor 执行线程的状态
(gdb) thread 3      # 切换到崩溃所在的工作线程
```

### 3. 构建时开启 ASan（AddressSanitizer）
在开发编译期直接抓出内存溢出与悬挂指针：
```bash
colcon build --packages-select my_pkg --cmake-args \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"
```

---

## 6.6 实时性与 CPU 绑核调优（PREEMPT_RT & taskset）

在机器人运动控制回路（如 1kHz 电机力矩控制）中，Linux 工程师常使用 PREEMPT_RT 实时内核并结合绑核操作：

```bash
# 1. 将 ROS 2 核心控制节点绑定至独立隔离核心 (CPU 3)
taskset -cp 3 $(pgrep -f "controller_node")

# 2. 设置 SCHED_FIFO 实时调度策略与最高优先级 99
sudo chrt -f -p 99 $(pgrep -f "controller_node")

# 3. 验证当前调度策略
chrt -p $(pgrep -f "controller_node")
# 输出: pid 12345's current scheduling policy: SCHED_FIFO, priority: 99
```

---

## 6.7 本章总结

Linux 开发者进入 ROS 2 的最大优势就是拥有一整套系统级工具。
遇到问题时：
- 先用 `ros2 doctor` 与 `ros2 topic info` 做业务层判断；
- 再用 `tcpdump` 与 `ss` 排查网络传输层；
- 最后用 `strace`、`gdb` 与 `ASan` 深入进程内核系统调用与内存堆栈。
