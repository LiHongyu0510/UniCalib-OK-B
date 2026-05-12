#!/usr/bin/env python3
from pathlib import Path

LIDAR_DIR = Path("/home/lihongyu/Unicalib/UniCalib-OK/data/drive_dusk/lidar")

def flip_z(input_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # 找到DATA行之后的位置
    data_idx = 0
    for i, line in enumerate(lines):
        if line.startswith('DATA'):
            data_idx = i + 1
            break
    
    # 修改Z坐标
    with open(input_file, 'w') as f:
        for i, line in enumerate(lines):
            if i < data_idx:
                f.write(line)
            else:
                parts = line.strip().split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    intensity = float(parts[3]) if len(parts) > 3 else 0.0
                    # Z取反
                    f.write(f'{-x:.6f} {y:.6f} {-z:.6f} {intensity:.6f}\n')
                else:
                    f.write(line)

pcd_files = sorted(LIDAR_DIR.glob('*.pcd'))
print(f"修改 {len(pcd_files)} 个文件的Z坐标...")

for i, pcd_file in enumerate(pcd_files):
    flip_z(pcd_file)
    if (i + 1) % 1000 == 0:
        print(f"  {i+1}/{len(pcd_files)}")

print("完成!")

# 验证
print("\n验证:")
with open(LIDAR_DIR / "00000000.pcd", 'r') as f:
    for _ in range(12):
        f.readline()
    for i in range(3):
        print(f.readline().strip())