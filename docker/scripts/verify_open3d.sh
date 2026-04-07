#!/bin/bash
# ==========================================================================
# Open3D 验证脚本（参考 reference）
# 用途: 验证镜像内 Open3D 安装与 CUDA/FPFH/点云功能
# ==========================================================================

set -e

echo "=========================================="
echo "Open3D 验证脚本"
echo "=========================================="

echo -e "\n[1/6] 验证 Open3D 版本..."
python3 -c "import open3d; print(f'Open3D 版本: {open3d.__version__}')"

echo -e "\n[2/6] 验证 CUDA 支持..."
python3 << 'EOF'
import open3d as o3d
try:
    device = o3d.core.Device("CUDA:0")
    print(f"✓ CUDA 设备可用: {device}")
    pcd = o3d.data.PCDPointCloud()
    points = o3d.io.read_point_cloud(pcd.path)
    points_gpu = points.to(device)
    print(f"✓ 点云 GPU 加载成功，点数: {len(points.points)}")
except Exception as e:
    print(f"✗ CUDA 不可用或失败: {e}")
    print("  (注: CPU-only 环境中可忽略)")
EOF

echo -e "\n[3/6] 验证 FPFH 特征提取..."
python3 << 'EOF'
import open3d as o3d
try:
    pcd = o3d.data.PCDPointCloud()
    points = o3d.io.read_point_cloud(pcd.path)
    points.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        points, o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))
    print(f"✓ FPFH 特征计算成功，特征维度: {fpfh.data.shape}")
except Exception as e:
    print(f"✗ FPFH 特征提取失败: {e}")
EOF

echo -e "\n[4/6] 验证 TEASER++ Python 绑定..."
python3 << 'EOF'
try:
    import teaserpp_python
    print("✓ TEASER++ Python 导入成功")
except ImportError as e:
    print(f"✗ TEASER++ 导入失败: {e}")
EOF

echo -e "\n[5/6] 验证点云处理功能..."
python3 << 'EOF'
import numpy as np
import open3d as o3d
try:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.random.rand(100, 3))
    pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
    pcd.estimate_normals()
    pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"✓ 点云处理功能正常")
except Exception as e:
    print(f"✗ 点云处理失败: {e}")
EOF

echo -e "\n[6/6] 检查库文件..."
find /usr/local/lib/python3* -name "*open3d*" -type f 2>/dev/null | head -3 || true
echo "=========================================="
echo "验证完成！"
echo "=========================================="
