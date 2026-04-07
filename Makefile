# =============================================================================
# UniCalib 项目管理 Makefile（主框架: C++ unicalib_C_plus_plus，与 README 一致）
# =============================================================================

SHELL := /bin/bash
WORKSPACE_DIR := $(shell pwd)
DOCKER_IMAGE  := calib_env:humble
COMPOSE_FILE  := docker/docker-compose.yaml

# 数据/结果目录（可通过环境变量覆盖）
CALIB_DATA_DIR    ?= /tmp/calib_data
CALIB_RESULTS_DIR ?= /tmp/calib_results
# 编译/运行日志统一写入 logs/ 并带时间戳
LOGS_DIR          ?= $(WORKSPACE_DIR)/logs
BUILD_LOG_TS      := $(shell date +%Y%m%d_%H%M%S)
BUILD_LOG_FILE    ?= $(LOGS_DIR)/build_$(BUILD_LOG_TS).log

.PHONY: help build build-cpp run run-intrinsic run-fine run-python run-python-intrinsic run-python-fine shell test lint check-data visualize clean

help:
	@echo ""
	@echo "UniCalib 多传感器标定系统（主框架: C++）"
	@echo ""
	@echo "使用方法:"
	@echo "  make build           构建 Docker 镜像"
	@echo "  make build-cpp       在容器内构建 C++ 主框架（可选，run 时会自动构建）"
	@echo "  make run             在容器中运行完整标定（C++ 主框架，一键四阶段）"
	@echo "  make run-intrinsic   同上（C++ 当前为全流水线）"
	@echo "  make run-fine        同上（C++ 当前为全流水线）"
	@echo "  make run-python      使用 Python 参考实现运行完整标定"
	@echo "  make run-python-intrinsic  仅内参（Python）"
	@echo "  make run-python-fine      仅精外参（Python）"
	@echo "  make shell           进入容器交互式 Shell"
	@echo "  make test            运行单元测试"
	@echo "  make lint            代码质量检查"
	@echo "  make check-data      检查数据质量"
	@echo "  make visualize       生成标定结果可视化"
	@echo "  make clean           清理结果目录"
	@echo ""
	@echo "环境变量:"
	@echo "  CALIB_DATA_DIR    数据目录 (默认: /tmp/calib_data)"
	@echo "  CALIB_RESULTS_DIR 结果目录 (默认: /tmp/calib_results)"
	@echo "  LOGS_DIR          编译/运行日志目录 (默认: \$(WORKSPACE_DIR)/logs)"
	@echo ""

## -------------------------------------------------------------------------
## Docker 操作
## -------------------------------------------------------------------------

build:
	@echo ">>> 构建 Docker 镜像: $(DOCKER_IMAGE)"
	@mkdir -p $(LOGS_DIR)
	cd docker && docker build -t $(DOCKER_IMAGE) . 2>&1 | tee docker/build.log | tee $(LOGS_DIR)/docker_build_$(BUILD_LOG_TS).log
	@echo ">>> 构建完成 | 日志: $(LOGS_DIR)/docker_build_$(BUILD_LOG_TS).log"

build-cpp:
	@echo ">>> 在容器内构建 C++ 主框架 (unicalib_C_plus_plus)"
	@mkdir -p $(LOGS_DIR)
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib \
		bash -c "cd /root/calib_ws/unicalib_C_plus_plus && mkdir -p build && cd build && cmake .. && make -j\$$(nproc)" 2>&1 | tee $(BUILD_LOG_FILE)
	@echo ">>> C++ 构建完成 | 日志: $(BUILD_LOG_FILE)"

shell:
	@echo ">>> 进入容器 Shell"
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) \
	CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib bash

## -------------------------------------------------------------------------
## 标定执行（C++ 主框架，与 README 一致）
## -------------------------------------------------------------------------

run:
	@echo ">>> 运行完整标定流水线（C++ 主框架）"
	@mkdir -p $(CALIB_RESULTS_DIR) $(LOGS_DIR)
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) \
	CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	CALIB_LOGS_DIR=$(LOGS_DIR) \
	UNICALIB_LOGS_DIR=/root/calib_ws/logs \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib /opt/scripts/run_calib_cpp.sh

run-intrinsic:
	@echo ">>> 运行标定（C++ 全流水线，含内参）"
	mkdir -p $(CALIB_RESULTS_DIR)
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib /opt/scripts/run_calib_cpp.sh

run-fine:
	@echo ">>> 运行标定（C++ 全流水线，含精外参）"
	mkdir -p $(CALIB_RESULTS_DIR)
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib /opt/scripts/run_calib_cpp.sh

## -------------------------------------------------------------------------
## 标定执行（Python 参考实现，分阶段可选）
## -------------------------------------------------------------------------

run-python:
	@echo ">>> 运行完整标定流水线（Python 参考实现）"
	mkdir -p $(CALIB_RESULTS_DIR)
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib \
		bash -c "source /opt/ros/humble/setup.bash && \
		         python3 /root/calib_ws/src/UniCalib/scripts/run_calibration.py \
		                 --config /root/calib_ws/src/UniCalib/config/unicalib_config.yaml \
		                 --data /root/calib_ws/data \
		                 --output /root/calib_ws/results"

run-python-intrinsic:
	@echo ">>> 仅运行内参标定（Python）"
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib \
		bash -c "source /opt/ros/humble/setup.bash && \
		         python3 /root/calib_ws/src/UniCalib/scripts/run_calibration.py \
		                 --config /root/calib_ws/src/UniCalib/config/unicalib_config.yaml \
		                 --data /root/calib_ws/data \
		                 --output /root/calib_ws/results --stage intrinsic"

run-python-fine:
	@echo ">>> 仅运行精外参标定（Python）"
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib \
		bash -c "source /opt/ros/humble/setup.bash && \
		         python3 /root/calib_ws/src/UniCalib/scripts/run_calibration.py \
		                 --config /root/calib_ws/src/UniCalib/config/unicalib_config.yaml \
		                 --data /root/calib_ws/data \
		                 --output /root/calib_ws/results --stage fine"

## -------------------------------------------------------------------------
## 测试
## -------------------------------------------------------------------------

test:
	@echo ">>> 在容器内运行单元测试"
	docker run --rm \
		-v $(WORKSPACE_DIR)/UniCalib:/root/calib_ws/src/UniCalib:ro \
		$(DOCKER_IMAGE) \
		bash -c "cd /root/calib_ws/src/UniCalib && \
		         pip install pytest numpy scipy opencv-python-headless open3d -q && \
		         python3 -m pytest tests/ -v --tb=short 2>&1"

test-local:
	@echo ">>> 本地运行单元测试 (需要已安装依赖)"
	cd UniCalib && python3 -m pytest tests/ -v --tb=short

## -------------------------------------------------------------------------
## 代码质量
## -------------------------------------------------------------------------

lint:
	@echo ">>> 代码质量检查"
	docker run --rm \
		-v $(WORKSPACE_DIR)/UniCalib:/src:ro \
		$(DOCKER_IMAGE) \
		bash -c "pip install flake8 -q && flake8 /src/unicalib --max-line-length=100 \
		         --ignore=E501,W503,E226 --count"

## -------------------------------------------------------------------------
## 数据/可视化
## -------------------------------------------------------------------------

check-data:
	@echo ">>> 检查数据质量"
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib \
		bash -c "source /opt/ros/humble/setup.bash && \
		         python3 /root/calib_ws/src/UniCalib/scripts/collect_data.py \
		                 --config /root/calib_ws/src/UniCalib/config/unicalib_config.yaml \
		                 --data /root/calib_ws/data"

visualize:
	@echo ">>> 生成标定结果可视化"
	CALIB_DATA_DIR=$(CALIB_DATA_DIR) CALIB_RESULTS_DIR=$(CALIB_RESULTS_DIR) \
	docker-compose -f $(COMPOSE_FILE) run --rm unicalib \
		bash -c "source /opt/ros/humble/setup.bash && \
		         python3 /root/calib_ws/src/UniCalib/scripts/visualize_results.py \
		                 --config /root/calib_ws/src/UniCalib/config/unicalib_config.yaml \
		                 --results /root/calib_ws/results \
		                 --data /root/calib_ws/data"

## -------------------------------------------------------------------------
## 清理
## -------------------------------------------------------------------------

clean:
	@echo ">>> 清理结果目录"
	rm -rf $(CALIB_RESULTS_DIR)/*
	@echo ">>> 完成"

clean-all: clean
	@echo ">>> 清理 Docker 容器"
	docker-compose -f $(COMPOSE_FILE) down --remove-orphans
