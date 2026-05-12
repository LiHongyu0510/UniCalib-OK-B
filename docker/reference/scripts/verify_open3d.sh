#!/bin/bash
# ==========================================================================
# Open3D 验证脚本
# 用途: 验证 dockerfile 优化后 Open3D 是否正确安装
# ==========================================================================

set -e

echo "=========================================="
echo "Open3D 验证脚本"
echo "=========================================="

# 1. 验证版本
echo -e "\n[1/6] 验证 Open3D 版本..."
python3 -c "import open3d; print(f'Open3D 版本: {open3d.__version__}')"

# 2. 验证 CUDA 支持
echo -e "\n[2/6] 验证 CUDA 支持..."
python3 << 'EOF'
import open3d as o3d
try:
    # 测试 CUDA 设备
    device = o3d.core.Device("CUDA:0")
    print(f"✓ CUDA 设备可用: {device}")

    # 创建 GPU 点云测试
    pcd = o3d.data.PCDPointCloud()
    points = o3d.io.read_point_cloud(pcd.path)
    points_gpu = points.to(device)
    print(f"✓ 点云 GPU 加载成功，点数: {len(points.points)}")
except Exception as e:
    print(f"✗ CUDA 不可用或失败: {e}")
    print("  (注: 这在 CPU-only 环境中是正常的)")
EOF

# 3. 验证 FPFH 特征提取 (TEASER++ 依赖)
echo -e "\n[3/6] 验证 FPFH 特征提取..."
python3 << 'EOF'
import open3d as o3d
try:
    # 加载点云
    pcd = o3d.data.PCDPointCloud()
    points = o3d.io.read_point_cloud(pcd.path)

    # 计算法线
    points.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # 计算 FPFH 特征
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        points,
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100)
    )
    print(f"✓ FPFH 特征计算成功，特征维度: {fpfh.data.shape}")
except Exception as e:
    print(f"✗ FPFH 特征提取失败: {e}")
EOF

# 4. 验证 TEASER++ Python 绑定
echo -e "\n[4/6] 验证 TEASER++ Python 绑定..."
python3 << 'EOF'
try:
    import teaserpp_python
    print("✓ TEASER++ Python 导入成功")

    # 测试基本功能
    import numpy as np
    solution = teaserpp_python.teaserpp_python.Solution()
    print(f"✓ TEASER++ 基本功能可用")
except ImportError as e:
    print(f"✗ TEASER++ 导入失败: {e}")
except Exception as e:
    print(f"✗ TEASER++ 功能测试失败: {e}")
EOF

# 5. 验证点云处理功能
echo -e "\n[5/6] 验证点云处理功能..."
python3 << 'EOF'
import open3d as o3d
try:
    # 创建测试点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.random.rand(100, 3))

    # 测试各种操作
    pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
    print("✓ 体素下采样成功")

    # 估计法线
    pcd.estimate_normals()
    print("✓ 法线估计成功")

    # 统计滤波
    pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print("✓ 统计滤波成功")

    print(f"✓ 点云处理功能正常，点数: {len(pcd.points)} -> {len(pcd_clean.points)}")
except Exception as e:
    print(f"✗ 点云处理失败: {e}")
EOF

# 6. 检查库文件
echo -e "\n[6/6] 检查库文件..."
echo "Open3D Python 库文件:"
find /usr/local/lib/python3* -name "*open3d*" -type f 2>/dev/null | head -5 || echo "  (未找到 Python 库文件)"

echo -e "\nOpen3D C++ 库文件:"
find /usr/local/lib -name "*open3d*" -type f 2>/dev/null | head -5 || echo "  (未找到 C++ 库文件)"

echo -e "\n=========================================="
echo "验证完成！"
echo "=========================================="
