#!/usr/bin/env python3
import struct
import os
from pathlib import Path

RELLIS_ROOT = Path('/home/lihongyu/Unicalib/UniCalib-OK/data/RELLIS3D')
OUTPUT_ROOT = Path('/home/lihongyu/Unicalib/UniCalib-OK/data/RELLIS3D_data')

BIN_DIR = RELLIS_ROOT / 'Rellis_3D_os1_cloud_node_kitti_bin' / 'Rellis_3D_os1_cloud_node_kitti_bin' / 'Rellis-3D' / '00000' / 'os1_cloud_node_kitti_bin'
CAM_DIR = RELLIS_ROOT / 'Rellis_3D_pylon_camera_node' / 'Rellis_3D_pylon_camera_node' / 'Rellis-3D' / '00000' / 'pylon_camera_node'

def convert_bin_to_pcd(input_file, output_file, max_points=50000):
    with open(input_file, 'rb') as f:
        data = f.read()

    n_points = len(data) // 16
    valid_points = []
    
    for i in range(n_points):
        x, y, z, intensity = struct.unpack('<ffff', data[i*16:(i*16)+16])
        if abs(x) < 100 and abs(y) < 100 and abs(z) < 10:
            valid_points.append((x, y, z))
    
    if len(valid_points) > max_points:
        valid_points = valid_points[:max_points]
    
    with open(output_file, 'w') as f:
        f.write('VERSION .7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {len(valid_points)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write('POINTS %d\n' % len(valid_points))
        f.write('DATA ascii\n')
        for pt in valid_points:
            f.write('%f %f %f\n' % pt)

print(f"Converting LiDAR bin -> pcd...")
BIN_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
PCD_DIR = OUTPUT_ROOT / 'lidar_pcd'
PCD_DIR.mkdir(exist_ok=True)

bin_files = sorted(BIN_DIR.glob('*.bin'))[:100]
for i, bin_file in enumerate(bin_files):
    pcd_file = PCD_DIR / f'{i:06d}.pcd'
    convert_bin_to_pcd(bin_file, pcd_file)
    if i % 20 == 0:
        print(f"  Converted {i+1}/{len(bin_files)}")

print(f"LiDAR: {len(bin_files)} .bin -> .pcd")

print(f"Copying camera images...")
CAM_DIR.mkdir(parents=True, exist_ok=True)
IMG_DIR = OUTPUT_ROOT / 'Image' / 'cam_left'
IMG_DIR.mkdir(parents=True, exist_ok=True)

jpg_files = sorted(CAM_DIR.glob('*.jpg'))[:100]
for i, jpg_file in enumerate(jpg_files):
    dst = IMG_DIR / f'{i:06d}.jpg'
    dst.write_bytes(jpg_file.read_bytes())
    if i % 20 == 0:
        print(f"  Copied {i+1}/{len(jpg_files)}")

print(f"Camera: {len(jpg_files)} .jpg images")

print(f"Creating IMU placeholder...")
IMU_FILE = OUTPUT_ROOT / 'imu.csv'
IMU_FILE.write_text('timestamp,gx,gy,gz,ax,ay,az\n')
print(f"IMU: placeholder created (need real IMU data)")

print(f"\nDone! Output: {OUTPUT_ROOT}")
print(f"  lidar_pcd/: {len(bin_files)} .pcd files")
print(f"  Image/cam_left/: {len(jpg_files)} .jpg files")
print(f"  imu.csv: placeholder")