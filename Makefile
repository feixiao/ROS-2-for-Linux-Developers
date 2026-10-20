# ==============================================================================
# ROS 2 for Linux Developers - Ergonomic Automation Makefile
# ==============================================================================

SHELL := /bin/bash
.DEFAULT_GOAL := help

.PHONY: help build clean test docker-build docker-up docker-down lint

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
	@echo ">>> 代码语法检查全部通过！"

clean: ## 清理 colcon 构建缓存与中间产物 (build, install, log)
	@echo ">>> 正在清理构建缓存..."
	rm -rf build install log
	@echo ">>> 清理完成。"
