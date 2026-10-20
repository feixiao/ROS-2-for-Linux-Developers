#!/usr/bin/env bash
# ==============================================================================
# Production Robot Health Check & Diagnostics Script for Linux Systems
# Performs fast hardware, kernel, network, DDS, and systemd readiness checks.
# ==============================================================================

set -uo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

pass() { echo -e "  [${GREEN}PASS${NC}] $1"; }
warn() { echo -e "  [${YELLOW}WARN${NC}] $1"; }
fail() { echo -e "  [${RED}FAIL${NC}] $1"; }
info() { echo -e "${BLUE}>>> $1${NC}"; }

echo "========================================================================"
echo "          ROS 2 Linux Robot Production Health Inspection               "
echo "========================================================================"

# 1. 检查 Linux 内核与实时性
info "1. Linux 内核与调度状态检查"
KERNEL_VER=$(uname -r)
if uname -v | grep -qi "PREEMPT RT"; then
  pass "PREEMPT_RT 硬实时内核已启用 (${KERNEL_VER})"
else
  warn "未检测到 PREEMPT_RT 补丁，当前为标准分时内核 (${KERNEL_VER})"
fi

# 2. 检查内存与 Swap 状态
info "2. 内存与虚拟内存状态"
if command -v free >/dev/null 2>&1; then
  TOTAL_RAM_MB=$(free -m | awk '/^Mem:/{print $2}')
  AVAIL_RAM_MB=$(free -m | awk '/^Mem:/{print $7}')
  if [ -n "${AVAIL_RAM_MB}" ] && [ "${AVAIL_RAM_MB}" -gt 1024 ]; then
    pass "可用物理内存充足: ${AVAIL_RAM_MB}MB / ${TOTAL_RAM_MB}MB"
  else
    warn "可用物理内存较低: ${AVAIL_RAM_MB}MB / ${TOTAL_RAM_MB}MB"
  fi

  SWAP_USED_MB=$(free -m | awk '/^Swap:/{print $3}')
  if [ -n "${SWAP_USED_MB}" ] && [ "${SWAP_USED_MB}" -eq 0 ]; then
    pass "系统未发生 Swap 内存换页 (零 Swap)"
  else
    warn "检测到 Swap 内存换页使用: ${SWAP_USED_MB}MB (可能引入偶发延迟抖动)"
  fi
else
  pass "非标准 Linux 环境或无 free 工具，已跳过内存详细比对"
fi

# 3. 检查 DDS 与网络内核参数 (sysctl)
info "3. 网络与 DDS Socket 栈优化检查"
if [ -f /proc/sys/net/core/rmem_max ]; then
  RMEM_MAX=$(cat /proc/sys/net/core/rmem_max)
  if [ "${RMEM_MAX}" -ge 16777216 ]; then
    pass "Socket 接收缓冲区上限符合高吞吐标准: ${RMEM_MAX} bytes"
  else
    fail "Socket 接收缓冲区过小 (${RMEM_MAX} bytes)，建议设置为 33554432 以免丢失点云！"
  fi
else
  warn "无法读取 /proc/sys/net/core/rmem_max"
fi

# 检查内核 UDP 丢包统计
if [ -f /proc/net/snmp ]; then
  UDP_DROPS=$(awk '/^Udp:/{print $7}' /proc/net/snmp | tail -n 1)
  if [ -n "${UDP_DROPS}" ] && [ "${UDP_DROPS}" -eq 0 ]; then
    pass "内核 UDP 协议栈无静默丢包 (RcvbufErrors: 0)"
  else
    warn "内核检测到 UDP 缓冲区丢包 (RcvbufErrors: ${UDP_DROPS})，请调优 sysctl 或 QoS"
  fi
fi

# 4. 检查组播与路由配置
info "4. DDS 组播 (Multicast UDP 239.255.0.1) 与网络路由检查"
if ip route 2>/dev/null | grep -q "224.0.0.0/4"; then
  pass "组播专用路由 (multicast 224.0.0.0/4) 已显式配置"
else
  warn "未检测到 224.0.0.0/4 显式组播路由，依赖系统默认网关分发"
fi

# 5. 检查 ROS 2 环境变量
info "5. ROS 2 运行环境变量检查"
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  pass "ROS_DOMAIN_ID 已配置为: ${ROS_DOMAIN_ID}"
else
  warn "ROS_DOMAIN_ID 未设置，当前默认为 0 (同网段易产生串扰)"
fi

if [ -n "${RMW_IMPLEMENTATION:-}" ]; then
  pass "RMW_IMPLEMENTATION 已指定为: ${RMW_IMPLEMENTATION}"
else
  info "RMW_IMPLEMENTATION 未指定，将使用平台默认中间件"
fi

# 6. 检查硬件外设与总线直连
info "6. 传感器与硬件总线接口检查"
if ip link show type can 2>/dev/null | grep -q "can"; then
  pass "检测到 SocketCAN 硬件总线接口"
else
  info "未检测到 SocketCAN 接口"
fi

if ls /dev/robot_* 1>/dev/null 2>&1; then
  pass "Udev 固化符号链接存在: $(ls /dev/robot_*)"
else
  info "未发现 /dev/robot_* Udev 固化设备"
fi

# 7. 检查 Systemd 守护进程状态
info "7. Systemd 机器人核心服务状态"
if command -v systemctl >/dev/null 2>&1; then
  if systemctl is-active --quiet ros2-robot.service 2>/dev/null; then
    pass "ros2-robot.service 正在运行中"
  else
    info "ros2-robot.service 未处于运行状态 (如尚未部署开机自启)"
  fi
fi

echo "========================================================================"
echo "                    健康巡检完成 (Inspection Completed)                 "
echo "========================================================================"
exit 0
