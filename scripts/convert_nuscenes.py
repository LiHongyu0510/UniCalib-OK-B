#!/usr/bin/env python3
"""
nuScenes 数据转换脚本
将 nuScenes v1.0-trainval06_blobs 转换为 UniCalib 可用格式
"""

import os
import sys
import numpy as np
from pathlib import Path
from PIL import Image

def convert_lidar_bin_to_pcd(src_file, dest_file):
    """将 nuScenes .pcd.bin 转换为 .pcd 格式"""
    try:
        points = np.fromfile(src_file, dtype=np.float32).reshape(-1, 4)
        xyz = points[:, :3]
        
        with open(dest_file, 'w') as f:
            f.write("VERSION .7\n")
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write("WIDTH {}\n".format(len(xyz)))
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS {}\n".format(len(xyz)))
            f.write("DATA ascii\n")
            for p in points:
                f.write("{} {} {} {}\n".format(p[0], p[1], p[2], p[3]))
        return True
    except Exception as e:
        print(f"转换失败 {src_file}: {e}")
        return False

def convert_nuscenes_dataset(base_dir, output_dir, max_frames=500):
    """转换 nuScenes 数据集"""
    
    src_dir = base_dir
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取所有相机的图像
    camera_names = ['CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 
                    'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT']
    
    # 创建相机目录并复制图像
    camera_files = {}
    for cam in camera_names:
        cam_dir = os.path.join(src_dir, 'samples', cam)
        if os.path.exists(cam_dir):
            files = sorted(os.listdir(cam_dir))
            # 只取前 max_frames 帧，确保 LiDAR 和相机同步
            camera_files[cam] = files[:max_frames]
            print(f"  {cam}: {len(camera_files[cam])} 帧")
    
    # 统计可用的同步帧数
    if not camera_files:
        print("错误: 未找到相机数据")
        return False
    
    # 使用 CAM_FRONT 作为参考，取前 max_frames 帧
    n_frames = len(camera_files['CAM_FRONT'])
    
    # 复制图像（取前 n_frames 帧）
    print(f"\n复制 {n_frames} 帧图像...")
    for cam in camera_names:
        if cam not in camera_files:
            continue
        dest_cam_dir = os.path.join(output_dir, cam.lower())
        os.makedirs(dest_cam_dir, exist_ok=True)
        
        for i, fname in enumerate(camera_files[cam][:n_frames]):
            src = os.path.join(src_dir, 'samples', cam, fname)
            # 重命名为序号
            dest = os.path.join(dest_cam_dir, f"{i:010d}.jpg")
            if not os.path.exists(dest):
                # 复制并转换为 jpg
                img = Image.open(src)
                img.save(dest, 'JPEG')
        
        print(f"  已处理 {cam}: {n_frames} 帧")
    
    # 转换 LiDAR 数据
    lidar_src_dir = os.path.join(src_dir, 'sweeps', 'LIDAR_TOP')
    lidar_dest_dir = os.path.join(output_dir, 'lidar_top_pcd')
    os.makedirs(lidar_dest_dir, exist_ok=True)
    
    if os.path.exists(lidar_src_dir):
        lidar_files = sorted(os.listdir(lidar_src_dir))
        # 尝试匹配 LiDAR 和相机的时间戳
        print(f"\n转换 LiDAR 点云...")
        
        # 从文件名提取时间戳
        def extract_timestamp(fname):
            # 格式: n008-2018-08-27-11-48-51-0400__LIDAR_TOP__1535385032199407.pcd.bin
            parts = fname.split('__')
            if len(parts) >= 3:
                ts_str = parts[2].replace('.pcd.bin', '')
                return int(ts_str)
            return 0
        
        # 相机时间戳
        cam_timestamps = []
        for fname in camera_files['CAM_FRONT']:
            parts = fname.split('__')
            if len(parts) >= 3:
                cam_timestamps.append(int(parts[2].replace('.jpg', '')))
        
        # 为每帧相机找到最近的 LiDAR
        converted = 0
        for i, ts in enumerate(cam_timestamps):
            # 找到最近的 LiDAR 文件
            best_match = None
            best_diff = float('inf')
            
            for lf in lidar_files:
                lts = extract_timestamp(lf)
                diff = abs(lts - ts)
                if diff < best_diff:
                    best_diff = diff
                    best_match = lf
            
            if best_match and best_diff < 100000:  # 100ms 内
                src_lidar = os.path.join(lidar_src_dir, best_match)
                dest_pcd = os.path.join(lidar_dest_dir, f"{i:010d}.pcd")
                
                if not os.path.exists(dest_pcd):
                    if convert_lidar_bin_to_pcd(src_lidar, dest_pcd):
                        converted += 1
                else:
                    converted += 1
            
            if (i + 1) % 100 == 0:
                print(f"  已处理 {i+1}/{n_frames} 帧, LiDAR: {converted}")
        
        print(f"  LiDAR 转换完成: {converted} 帧")
    else:
        print("错误: 未找到 LiDAR 数据")
        return False
    
    print(f"\n转换完成!")
    print(f"  图像: {n_frames} 帧 x {len(camera_names)} 相机")
    print(f"  LiDAR: {converted} 帧")
    print(f"  输出目录: {output_dir}")
    
    return True

def main():
    base_dir = "/home/lihongyu/Unicalib/UniCalib-OK/data/v1.0-trainval06_blobs"
    output_dir = "/home/lihongyu/Unicalib/UniCalib-OK/data/nuscenes_trainval06"
    
    print("=" * 50)
    print("nuScenes 数据转换")
    print("=" * 50)
    
    convert_nuscenes_dataset(base_dir, output_dir, max_frames=500)

if __name__ == "__main__":
    main()