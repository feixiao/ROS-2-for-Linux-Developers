# ===============================================================
# 阶段 1: 构建阶段 (Builder Stage)
# ===============================================================
FROM ros:humble-ros-base AS builder

# 安装构建与系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间并复制源码
WORKDIR /ros2_ws
COPY custom_interfaces /ros2_ws/src/custom_interfaces
COPY cpp_topic_demo /ros2_ws/src/cpp_topic_demo
COPY py_topic_demo /ros2_ws/src/py_topic_demo
COPY cpp_service_demo /ros2_ws/src/cpp_service_demo
COPY cpp_action_demo /ros2_ws/src/cpp_action_demo
COPY cpp_composition_demo /ros2_ws/src/cpp_composition_demo
COPY cpp_lifecycle_demo /ros2_ws/src/cpp_lifecycle_demo
COPY launch_demo /ros2_ws/src/launch_demo

# 编译所有功能包
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --install-base /opt/ros2_app

# ===============================================================
# 阶段 2: 纯净运行时镜像 (Runtime Stage)
# ===============================================================
FROM ros:humble-ros-core AS runtime

# 安装运行时基础包
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-example-interfaces \
    ros-humble-ros2launch \
    ros-humble-rclcpp-components \
    ros-humble-rclcpp-lifecycle \
    && rm -rf /var/lib/apt/lists/*

# 创建安全非 root 用户
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG dialout,video $USERNAME

# 从 Builder 阶段复制最终二进制产物
COPY --from=builder /opt/ros2_app /opt/ros2_app

# 编写入口脚本
RUN echo '#!/usr/bin/env bash\n\
set -e\n\
source "/opt/ros/humble/setup.bash"\n\
source "/opt/ros2_app/setup.bash"\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
WORKDIR /home/$USERNAME

# 默认启动生产 Launch 编排
CMD ["ros2", "launch", "launch_demo", "demo.launch.py"]
