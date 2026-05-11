#!/usr/bin/env python3
"""
KITTI 数据转换脚本
将 KITTI oxts 数据转换为 UniCalib 所需的 IMU CSV 格式
"""

import os
import sys
from pathlib import Path

def convert_oxts_to_imu_csv(oxts_dir, output_csv, timestamps_file):
    """将 oxts 数据转换为 IMU CSV 格式"""
    
    # 读取时间戳
    timestamps = []
    if os.path.exists(timestamps_file):
        with open(timestamps_file, 'r') as f:
            for line in f:
                ts = line.strip()
                if ts:
                    timestamps.append(ts)
    
    # 读取 oxts 数据
    oxts_data_dir = os.path.join(oxts_dir, 'data')
    if not os.path.exists(oxts_data_dir):
        print(f"Error: oxts data directory not found: {oxts_data_dir}")
        return False
    
    oxts_files = sorted([f for f in os.listdir(oxts_data_dir) if f.endswith('.txt')])
    
    if not oxts_files:
        print(f"Error: No oxts .txt files found in {oxts_data_dir}")
        return False
    
    print(f"Found {len(oxts_files)} oxts data files")
    
    # 写入 CSV
    with open(output_csv, 'w') as f:
        f.write("timestamp,gx,gy,gz,ax,ay,az\n")
        
        for i, oxts_file in enumerate(oxts_files):
            oxts_path = os.path.join(oxts_data_dir, oxts_file)
            with open(oxts_path, 'r') as of:
                line = of.read().strip()
                values = line.split()
                
                # oxts 格式: lat,lon,alt,roll,pitch,yaw,vn,ve,vf,vl,vu,ax,ay,az,af,al,au,wx,wy,wz,wf,wl,wu,...
                # 索引: ax=12, ay=13, az=14, wx=18, wy=19, wz=20
                if len(values) >= 21:
                    try:
                        ax = float(values[12])  # m/s^2
                        ay = float(values[13])
                        az = float(values[14])
                        wx = float(values[18])  # rad/s
                        wy = float(values[19])
                        wz = float(values[20])
                        
                        # 使用时间戳或使用索引作为伪时间戳
                        if i < len(timestamps):
                            timestamp = timestamps[i]
                        else:
                            timestamp = f"{i:010d}"
                        
                        f.write(f"{timestamp},{wx},{wy},{wz},{ax},{ay},{az}\n")
                    except (ValueError, IndexError) as e:
                        print(f"Warning: Error parsing line {i}: {e}")
                        continue
    
    print(f"Converted to {output_csv}")
    return True

def copy_images_and_pointclouds(src_dir, dest_dir):
    """复制图像和点云数据"""
    
    # 图像 (image_00 是左前相机，image_02 是右后相机)
    for img_dir in ['image_00', 'image_01', 'image_02', 'image_03']:
        src_img = os.path.join(src_dir, img_dir)
        if os.path.exists(src_img):
            dest_img = os.path.join(dest_dir, img_dir)
            os.makedirs(dest_img, exist_ok=True)
            
            src_data = os.path.join(src_img, 'data')
            if os.path.exists(src_data):
                # 复制所有图片
                for f in os.listdir(src_data):
                    if f.endswith('.png'):
                        src_file = os.path.join(src_data, f)
                        dest_file = os.path.join(dest_img, f)
                        if not os.path.exists(dest_file):
                            os.symlink(os.path.relpath(src_file, dest_img), dest_file)
            print(f"Copied/symlinked {img_dir}")
    
    # 点云
    src_lidar = os.path.join(src_dir, 'velodyne_points')
    if os.path.exists(src_lidar):
        dest_lidar = os.path.join(dest_dir, 'velodyne_points')
        os.makedirs(dest_lidar, exist_ok=True)
        
        src_data = os.path.join(src_lidar, 'data')
        if os.path.exists(src_data):
            for f in os.listdir(src_data):
                if f.endswith('.bin'):
                    src_file = os.path.join(src_data, f)
                    dest_file = os.path.join(dest_lidar, f)
                    if not os.path.exists(dest_file):
                        os.symlink(os.path.relpath(src_file, dest_lidar), dest_file)
            print(f"Copied/symlinked velodyne_points")
    
    # 复制标定文件
    calib_dir = os.path.dirname(os.path.dirname(src_dir))
    calib_src = os.path.join(calib_dir, '2011_09_26')
    if os.path.exists(calib_src):
        dest_calib = os.path.join(dest_dir, 'calib')
        os.makedirs(dest_calib, exist_ok=True)
        for f in ['calib_cam_to_cam.txt', 'calib_imu_to_velo.txt', 'calib_velo_to_cam.txt']:
            src_f = os.path.join(calib_src, f)
            if os.path.exists(src_f):
                dest_f = os.path.join(dest_calib, f)
                if not os.path.exists(dest_f):
                    os.symlink(os.path.relpath(src_f, dest_calib), dest_f)
        print(f"Copied/symlinked calib files")

def convert_kitti_dataset(kitti_base_dir, output_dir):
    """转换完整的 KITTI 数据集"""
    
    # KITTI structure: base/2011_09_26/drive_xxxx_sync/2011_09_26/drive_xxxx_sync/
    sync_dir = None
    for root, dirs, files in os.walk(kitti_base_dir):
        if 'oxts' in dirs:
            sync_dir = root
            break
    
    if sync_dir is None:
        print(f"Error: No sync directory with oxts found in {kitti_base_dir}")
        return False
    
    print(f"Using sync directory: {sync_dir}")
    
    if not os.path.exists(sync_dir):
        print(f"Error: Dataset directory not found: {sync_dir}")
        return False
    
    os.makedirs(output_dir, exist_ok=True)
    
    # 转换 IMU 数据
    oxts_dir = os.path.join(sync_dir, 'oxts')
    timestamps_file = os.path.join(oxts_dir, 'timestamps.txt')
    imu_csv = os.path.join(output_dir, 'imu.csv')
    
    if not convert_oxts_to_imu_csv(oxts_dir, imu_csv, timestamps_file):
        return False
    
    # 复制图像和点云
    copy_images_and_pointclouds(sync_dir, output_dir)
    
    # 创建配置文件
    dataset_name = os.path.basename(kitti_base_dir).replace('_sync', '')
    create_unicalib_config(output_dir, dataset_name)
    
    print(f"\n转换完成: {output_dir}")
    return True

def create_unicalib_config(output_dir, dataset_name):
    """创建 UniCalib 配置文件"""
    
    config_content = f'''# UniCalib 配置 - KITTI {dataset_name}
# 自动生成

sensors:
  - id: imu_0
    type: imu
    imu:
      rate_hz: 10.0
      model: scale_misalignment

  - id: lidar_0
    type: lidar
    lidar:
      type: spinning
      scan_lines: 16
      rate_hz: 10.0

  - id: cam_left
    type: camera
    camera:
      model: pinhole
      width: 1392
      height: 512
      fps: 10.0
      rolling_shutter: false

  - id: cam_right
    type: camera
    camera:
      model: pinhole
      width: 1392
      height: 512
      fps: 10.0
      rolling_shutter: false

reference_imu: imu_0

data:
  imu:
    imu_0: imu.csv

  lidar:
    lidar_0: lidar_0.yaml

  camera:
    cam_left:
      images_dir: image_00/data
      intrinsic_yaml: results/camera_intrinsic/cam_left.yaml
    cam_right:
      images_dir: image_02/data
      intrinsic_yaml: results/camera_intrinsic/cam_right.yaml
'''
    
    config_path = os.path.join(output_dir, 'config.yaml')
    with open(config_path, 'w') as f:
        f.write(config_content)
    print(f"Created config: {config_path}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_base = os.path.join(os.path.dirname(script_dir), 'data')
    
    # 转换 drive_0048
    print("=" * 50)
    print("转换 2011_09_26_drive_0048_sync")
    print("=" * 50)
    output_0048 = os.path.join(data_base, 'kitti_drive_0048')
    convert_kitti_dataset(
        os.path.join(data_base, '2011_09_26_drive_0048_sync'),
        output_0048
    )
    
    # 转换 drive_0084
    print("\n" + "=" * 50)
    print("转换 2011_09_26_drive_0084_sync")
    print("=" * 50)
    output_0084 = os.path.join(data_base, 'kitti_drive_0084')
    convert_kitti_dataset(
        os.path.join(data_base, '2011_09_26_drive_0084_sync'),
        output_0084
    )

if __name__ == '__main__':
    main()