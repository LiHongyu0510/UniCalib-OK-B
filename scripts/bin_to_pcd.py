#!/usr/bin/env python3
"""将 KITTI .bin 点云转换为 .pcd 格式"""

import os
import numpy as np
from pathlib import Path

def bin_to_pcd(bin_file, pcd_file):
    """将 KITTI .bin 转换为 .pcd"""
    points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
    # 只保留 x,y,z
    xyz = points[:, :3]
    
    # 写入 PCD 格式
    with open(pcd_file, 'w') as f:
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH {}\n".format(len(xyz)))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS {}\n".format(len(xyz)))
        f.write("DATA ascii\n")
        for p in xyz:
            f.write("{} {} {}\n".format(p[0], p[1], p[2]))

def convert_kitti_velodyne(src_dir, dest_dir):
    """批量转换"""
    os.makedirs(dest_dir, exist_ok=True)
    
    bin_files = sorted([f for f in os.listdir(src_dir) if f.endswith('.bin')])
    print(f"转换 {len(bin_files)} 个文件...")
    
    for i, bin_file in enumerate(bin_files):
        bin_path = os.path.join(src_dir, bin_file)
        pcd_name = bin_file.replace('.bin', '.pcd')
        pcd_path = os.path.join(dest_dir, pcd_name)
        
        if not os.path.exists(pcd_path):
            bin_to_pcd(bin_path, pcd_path)
        
        if (i + 1) % 50 == 0:
            print(f"  已转换 {i+1}/{len(bin_files)}")
    
    print(f"完成！输出目录: {dest_dir}")

if __name__ == "__main__":
    base = "/home/lihongyu/Unicalib/UniCalib-OK/data"
    
    print("转换 kitti_drive_0084...")
    convert_kitti_velodyne(
        os.path.join(base, "2011_09_26_drive_0084_sync/2011_09_26/2011_09_26_drive_0084_sync/velodyne_points/data"),
        os.path.join(base, "kitti_drive_0084/velodyne_points_pcd")
    )
    
    print("\n转换 kitti_drive_0048...")
    convert_kitti_velodyne(
        os.path.join(base, "2011_09_26_drive_0048_sync/2011_09_26/2011_09_26_drive_0048_sync/velodyne_points/data"),
        os.path.join(base, "kitti_drive_0048/velodyne_points_pcd")
    )