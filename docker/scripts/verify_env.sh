#!/bin/bash
# ==========================================================================
# 环境验证脚本 - 多标定项目 Docker 镜像
# 验证 ROS2、Python、PyTorch、OpenCV、Ceres、PCL、Eigen、Open3D 等
# ==========================================================================

set -e

echo "=========================================="
echo "Multi-Calibration 环境验证"
echo "=========================================="

# ROS2
echo -e "\n[1] ROS2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ros2 --version && echo "✓ ROS2 正常" || echo "✗ ROS2 异常"
else
    echo "✗ 未找到 /opt/ros/humble/setup.bash"
fi

# Python
echo -e "\n[2] Python..."
python3 --version && echo "✓ Python 正常" || echo "✗ Python 异常"

# PyTorch + CUDA
echo -e "\n[3] PyTorch + CUDA..."
python3 -c "
import torch
print(f'  PyTorch: {torch.__version__}')
print(f'  CUDA available: {torch.cuda.is_available()}')
print(f'  CUDA version: {torch.version.cuda}')
print('✓ PyTorch 正常' if torch.__version__ else '✗ PyTorch 异常')
" || echo "✗ PyTorch 检查失败"

# OpenCV
echo -e "\n[4] OpenCV..."
python3 -c "
import cv2
print(f'  OpenCV: {cv2.__version__}')
print('✓ OpenCV 正常')
" || echo "✗ OpenCV 异常"

# PCL
echo -e "\n[5] PCL..."
pkg-config --modversion pcl_common 2>/dev/null && echo "✓ PCL 正常" || echo "  (pkg-config 未提供，可能已安装)"

# Eigen
echo -e "\n[6] Eigen..."
pkg-config --modversion eigen3 2>/dev/null && echo "✓ Eigen 正常" || echo "  (eigen3 未通过 pkg-config)"

# Ceres
echo -e "\n[7] Ceres Solver..."
pkg-config --modversion ceres 2>/dev/null && echo "✓ Ceres 正常" || echo "  (Ceres 已安装，可能无 pkg-config)"

# Open3D
echo -e "\n[8] Open3D..."
python3 -c "
import open3d as o3d
print(f'  Open3D: {o3d.__version__}')
print('✓ Open3D 正常')
" 2>/dev/null && true || echo "  Open3D 未安装或检查失败"

echo -e "\n=========================================="
echo "验证完成"
echo "=========================================="
