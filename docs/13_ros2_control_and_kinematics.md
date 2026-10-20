# 第 13 章：实时硬件抽象层（ros2_control）与机器人动力学闭环

在工业机械臂（6 轴/7 轴）、四足机器狗、人形双足机器人以及移动底盘的底层控制中，电机的力矩（Effort）、速度（Velocity）与位置（Position）控制回路通常要求以 **100Hz ~ 1000Hz（1kHz，即 1 毫秒周期）** 严苛运行。

如果我们使用普通的 ROS 2 话题发布订阅来传输 1kHz 的控制指令：
- **DDS 网络抖动** 会使数据包到达时间飘忽不定（时而 0.2ms，时而 3ms）；
- **动态内存分配与序列化** 会引发不可控的垃圾回收与缺页中断；
- **电机伺服将产生剧烈的高频振动、尖啸甚至发生过流烧毁**。

为了在 ROS 2 中实现工业伺服级的硬实时闭环控制，官方与业界制定了标杆级实时框架——**`ros2_control`**。
本章将深入其架构底层，剖析硬件抽象层、控制器管理器以及 URDF/Xacro 机器人运动学契约。

---

## 13.1 `ros2_control` 分层架构与 Linux 实时分工

```
+-----------------------------------------------------------+
|      上层用户空间 / AI 决策 (非实时, 异步 Topic / Action)   |
|         Nav2 (导航)  /  MoveIt 2 (机械臂运动规划)          |
+-----------------------------------------------------------+
                             |
                             v 下发 /cmd_vel 或 JointTrajectory
+-----------------------------------------------------------+
|              控制器管理器 (Controller Manager)             |
|  +-----------------------------------------------------+  |
|  | 实时控制器: diff_drive_controller / joint_trajectory |  |
|  +-----------------------------------------------------+  |
+-----------------------------------------------------------+
                             | 共享内存直写 (Zero-Allocation)
                             v
+-----------------------------------------------------------+
|              硬件接口层 (Hardware Interface)               |
|  - SystemInterface / ActuatorInterface / SensorInterface  |
|  - 纯 C++ 插件 (pluginlib 动态加载)                       |
+-----------------------------------------------------------+
                             | 1kHz 硬实时线程 (PREEMPT_RT)
                             v
+-----------------------------------------------------------+
|      Linux 底层硬件总线 (SocketCAN / EtherCAT / SPI)       |
+-----------------------------------------------------------+
```

### 核心分层职责：
1. **Controller Manager（控制器管理器）**：
   运行在具有最高优先级（如 `SCHED_FIFO 90`）的独立 POSIX 实时线程中。它以确定性的周期（如 100Hz/1000Hz）严格驱动执行循环：
   $$ \text{read()} \longrightarrow \text{update\_controllers()} \longrightarrow \text{write()} $$
2. **State & Command Interfaces（状态与命令接口）**：
   摒弃了话题与网络序列化，直接在内存中暴露基本数据类型（`double*`）的指针地址。控制器直接读写指针，耗时仅几纳秒！
3. **Hardware Interface Plugin（硬件接口插件）**：
   开发者只需实现 `read()`（从总线读编码器并写入 state 指针）和 `write()`（从 command 指针取数值并下发给电机驱动器）。

### 硬件接口三大抽象组件类型选型决策
在编写自定义硬件驱动插件时，`ros2_control` 提供了三种基类接口供架构师选型：
- **`ActuatorInterface`（单执行器接口）**：
  适用于单一自由度的独立驱动单元（如单一舵机、电动夹爪、气动推杆）。它具有 1 个物理命令通道与可选的状态反馈，逻辑高度内聚。
- **`SensorInterface`（纯只读传感器接口）**：
  适用于无电机驱动能力的只读硬件（如 6 维末端力/力矩传感器 FT-Sensor、高精绝对值光学编码器、IMU 惯导单元）。该接口不导出任何 `CommandInterface`，避免误写入损坏硬件。
- **`SystemInterface`（复杂多自由度系统接口）**：
  适用于多关节多轴协同的复杂整机系统（如 6 轴/7 轴工业机械臂、四轮阿克曼转向底盘、四足机器狗与人形双足机器人）。所有轮子或关节由同一个物理驱动板卡（如多轴一体式伺服驱动器）经由单根 CAN/EtherCAT 总线统一交互，因此在同一个插件实例中集中导出所有关节的命令与状态接口。

---

## 13.2 硬实时编码黄金戒律：消除一切非确定性调用

在 `ros2_control` 的 `read()` 与 `write()` 函数中，代码每秒被循环调用 1000 次（可用时间窗口仅 1000 微秒）。
Linux 系统工程师必须严格恪守以下**硬实时准则**：

1. **绝对禁止任何堆内存动态分配（Zero Dynamic Allocation）**：
   - 严禁调用 `malloc`、`free`、`new`、`delete`；
   - 严禁调用可能触发容器扩容的函数（如 `std::vector::push_back`、`std::string` 拼接）；所有数组必须在初始化阶段（`on_init`）预分配完毕（`reserve` / `resize`）。
2. **绝对禁止任何可能引发线程阻塞的互斥锁（No Blocking Mutex）**：
   - 普通的 `std::mutex` 在高并发竞争下可能引发上下文切换与无界优先级反转。必须使用**无锁队列（Lock-Free RingBuffer / SPSC Queue）**或原子变量（`std::atomic`）。
3. **绝对禁止控制台 I/O 与日志打印**：
   - 严禁在实时主循环中调用 `std::cout` 或常规 `RCLCPP_INFO`（因 `write()` 系统调用涉及内核控制台驱动与 TTY 锁，一次耗时可达数毫秒，瞬间造成控制回路超时超时）。
   - 若需调试，必须使用节流日志 `RCLCPP_INFO_THROTTLE` 或将数据投递至无锁环形队列后由非实时异步线程打印。

---

## 13.3 机器人机械运动学建模：URDF 与 Xacro 规范

在 Linux 机器人工程中，机械本体的刚体拓扑与动力学参数由 **URDF（Unified Robot Description Format）** 描述。
而 **Xacro（XML Macros）** 则是 URDF 的宏扩展语言（类似 C 语言的宏定义），支持属性传参、数学公式计算与模块化复用。

### 1. 刚体结构核心要素
- **`<link>`（连杆刚体）**：
  - `<visual>`：3D 渲染几何外形（STL/DAE 网格文件或基本几何体）。
  - `<collision>`：物理引擎接触碰撞几何体（通常用简化的 Box/Cylinder 加速碰撞检测）。
  - `<inertial>`：动力学参数，包含物理质量 `mass` 以及 $3 \times 3$ 正定**转动惯量张量矩阵（Inertia Tensor: $I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{xz}, I_{yz}$）**。
- **`<joint>`（运动关节）**：
  - `revolute`（带限位的旋转关节，如机械臂肩关节）；
  - `continuous`（连续无界旋转关节，如轮式底盘驱动轮）；
  - `prismatic`（直线滑动关节，如升降机构）；
  - `fixed`（固定刚体连接）。

### 2. URDF 注入 `ros2_control` 契约标签
通过在 Xacro 文件中嵌入 `<ros2_control>` 标签，定义机械关节与 C++ 硬件驱动插件的绑定关系：

```xml
<ros2_control name="DiffBotHardware" type="system">
  <hardware>
    <!-- 指定编译生成的硬件插件名称 (对应 pluginlib 导出类名) -->
    <plugin>cpp_ros2_control_demo/DiffDriveHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="gear_ratio">15.0</param>
  </hardware>

  <joint name="left_wheel_joint">
    <!-- 声明左轮可接收速度指令 -->
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <!-- 声明左轮向外界提供的位置与速度状态反馈 -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## 13.4 运行时动态控制器切换（Zero-Downtime Controller Switching）

在工业产线中，机器人可能在不同工序间无缝切换控制模式（例如：从“关节位置伺服模式”切换为“力控柔顺拖动示教模式”）。
传统方案必须重启进程，而在 `ros2_control` 中，控制器状态由有限状态机统一管理，支持**毫秒级无停机动态热切换**。

### 核心 CLI 管控工具集
```bash
# 1. 探查当前硬件接口所声明的所有命令与状态接口
ros2 control list_hardware_interfaces

# 2. 列出当前注册的所有控制器及其运行状态 (active / inactive)
ros2 control list_controllers

# 3. 动态热切换控制器: 停用手动遥控控制器，瞬间无缝激活自主差速控制器
ros2 control switch_controllers \
  --deactivate manual_teleop_controller \
  --activate diff_drive_controller \
  --strict
```

---

## 13.5 跨时钟域安全桥接：SPSC 无锁环形队列实战

在机器人真实系统中，非实时的 ROS 2 Executor 线程（处理高层 10Hz 的导航目标）与 1kHz 的 `ros2_control` 硬实时线程必须交换数据。
如果直接使用 `std::mutex` 加锁，非实时线程发生 CPU 抢占或缺页停顿，实时线程便会随之陷入阻塞，产生严重的控制抖动。

### 1. 单生产者单消费者（SPSC）无锁队列设计
基于 C++17 `std::atomic` 内存序（Memory Ordering），实现纯硬件级无锁原子交换：

```cpp
#include <atomic>
#include <array>
#include <optional>

template <typename T, size_t Capacity>
class SpscLockFreeRingBuffer {
public:
  SpscLockFreeRingBuffer() : head_(0), tail_(0) {}

  // 非实时线程调用: 写入目标位置/速度
  bool push(const T & item) {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    size_t next_tail = (current_tail + 1) % Capacity;
    if (next_tail == head_.load(std::memory_order_acquire)) {
      return false; // 队列已满，丢弃或报警
    }
    buffer_[current_tail] = item;
    tail_.store(next_tail, std::memory_order_release);
    return true;
  }

  // 1kHz 硬实时线程调用: 零等待读取最新指令
  std::optional<T> pop() {
    size_t current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire)) {
      return std::nullopt; // 队列为空
    }
    T item = buffer_[current_head];
    head_.store((current_head + 1) % Capacity, std::memory_order_release);
    return item;
  }

private:
  std::array<T, Capacity> buffer_;
  alignas(64) std::atomic<size_t> head_; // 缓存行对齐，避免伪共享 (False Sharing)
  alignas(64) std::atomic<size_t> tail_;
};
```
该无锁数据结构保证硬实时线程在取数据时**执行耗时严格小于 50 纳秒**，彻底斩断跨线程锁竞争。

---

## 13.6 工业总线进阶：EtherCAT 分布式时钟 (DC) 与微秒级同步

当机械臂拥有 6~7 个伺服关节，或四足机器人拥有 12 个电机时，CAN 总线的 1Mbps 带宽（单帧仅 8 字节）在 1kHz 频率下会发生严重的总线争用与延迟。
工业级机器人普遍采用 **EtherCAT（以太网控制自动化技术）**。

### 1. EtherCAT 与 Linux 内核交互机制
+- **报文飞读飞写（Processing on the fly）**：单一以太网帧穿过所有从站伺服驱动器，各从站硬件芯片（ESC）在报文流过瞬间直接提取命令并写入编码器反馈，总线利用率 > 90%。
+- **分布式时钟（Distributed Clocks, DC）**：
  EtherCAT 主站（运行于 Linux PREEMPT_RT）定期计算传输传播延迟，将所有伺服驱动器的硬件时钟同步到 **小于 1 微秒** 的确定性误差内！

### 2. SOEM 主站与 `ros2_control` 结合架构
在 `DiffDriveHardware::read()` 与 `write()` 中：
```cpp
// 1. 发送周期过程数据 (Process Data Object, PDO)
ec_send_processdata();

// 2. 接收从站硬件返回数据 (超时窗口设为 400 微秒)
int wkc = ec_receive_processdata(EC_TIMEOUTRET);
if (wkc >= expectedWKC) {
  for (int i = 0; i < num_joints; i++) {
    // 从以太网解包物理电机位置与力矩
    hw_positions_[i] = decode_encoder(ec_slave[i + 1].inputs);
  }
}
```

---

## 13.7 本章总结

- 摒弃常规话题的不可控抖动，高频电机控制必须依托 **`ros2_control`** 的指针直写与硬件抽象层。
- 坚决贯彻硬实时准则：在 `read()` 与 `write()` 循环中**彻底消灭动态堆内存分配、阻塞互斥锁与标准 I/O 打印**。
- 引入 **SPSC 无锁环形缓冲区**，优雅安全地打通非实时 ROS 2 异步逻辑与 1kHz 硬实时内核回路。
- 基于 **URDF/Xacro** 定义包含惯量张量的运动学模型，并在 XML 中显式绑定硬件插件契约。
- 面向多关节复杂装备，拥抱 **EtherCAT 分布式时钟**，达成亚微秒级多轴精准协同，为工业机器人底盘与机械臂的高性能运动控制打下坚实基石。
