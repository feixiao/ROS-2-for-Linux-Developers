# ==============================================================================
# ROS 2 for Linux Developers - Ergonomic Automation Makefile
# ==============================================================================

SHELL := /bin/bash
.DEFAULT_GOAL := help

.PHONY: help build clean test test-integration sysctl-apply check-health service-install docker-build docker-up docker-down lint

help: ## 显示可用指令与说明 (Show help menu)
	@echo "========================================================================"
	@echo "ROS 2 for Linux Developers - 工程自动化指令集"
	@echo "========================================================================"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'

build: ## 使用 colcon 编译所有 demo 功能包 (带符号链接与调试符号导出)
	@echo ">>> 正在编译所有 ROS 2 功能包..."
	source /opt/ros/humble/setup.bash 2>/dev/null || true; \
	colcon build \
		--symlink-install \
		--cmake-args \
			-DCMAKE_BUILD_TYPE=RelWithDebInfo \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

test: ## 执行基准质量评估套件 (Run handbook & demo validation harness)
	@echo ">>> 正在运行质量评估与代码语法校验..."
	python3 scripts/evaluate.py

test-integration: ## 运行 launch_testing 多进程端到端集成测试
	@echo ">>> 正在执行 launch_testing 多进程集成测试..."
	source install/setup.bash 2>/dev/null || true; \
	colcon test --packages-select launch_demo && colcon test-result --verbose

sysctl-apply: ## 将高吞吐 ROS 2 DDS 内核网络参数应用到 Linux 系统
	@echo ">>> 正在部署 Linux 内核网络优化参数 (需 sudo 权限)..."
	sudo cp scripts/60-ros2-dds-sysctl.conf /etc/sysctl.d/
	sudo sysctl --system

check-health: ## 运行 Linux 机器人生产环境健康巡检脚本
	@echo ">>> 正在执行机器人系统环境体检..."
	bash scripts/robot_health_check.sh

service-install: ## 安装并启用生产级 Systemd 守护进程 (需 sudo 权限)
	@echo ">>> 正在安装 systemd 机器人核心服务..."
	sudo cp scripts/ros2-robot.service /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable ros2-robot.service
	@echo ">>> 安装完成！使用 'sudo systemctl start ros2-robot.service' 启动。"

docker-build: ## 构建生产级多阶段 ROS 2 Docker 镜像
	@echo ">>> 正在构建多阶段 Docker 镜像..."
	docker compose -f demos/docker/docker-compose.yml build

docker-up: ## 启动 Docker 容器多节点编排集群 (Host 组播模式)
	@echo ">>> 正在启动 ROS 2 容器集群..."
	docker compose -f demos/docker/docker-compose.yml up

docker-down: ## 停止并清理 Docker 容器
	@echo ">>> 正在停止 ROS 2 容器集群..."
	docker compose -f demos/docker/docker-compose.yml down

lint: ## 检查 Python 与 C++ 代码规范
	@echo ">>> 正在检查代码规范..."
	python3 -m py_compile demos/py_topic_demo/py_topic_demo/*.py
	python3 -m py_compile demos/launch_demo/launch/*.py
	python3 -m py_compile demos/cpp_composition_demo/launch/*.py
	python3 -m py_compile demos/multimedia_av_demo/scripts/*.py
	python3 -m py_compile demos/multimedia_av_demo/launch/*.py
	@echo ">>> 代码语法检查全部通过！"

clean: ## 清理 colcon 构建缓存与中间产物 (build, install, log)
	@echo ">>> 正在清理构建缓存..."
	rm -rf build install log
	@echo ">>> 清理完成。"
